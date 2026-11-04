// SPDX-License-Identifier: BSD-2-Clause-Patent
// pdx203 3.5 mm headphone jack: the WCD9380/9385 behind the LPASS "bolero" RX/TX macros and their SoundWire masters, polled (no
// interrupts). The DSP side (AFE clocks 0x30C/0x30D, the RX_CODEC_DMA_RX_0 port 0xB030, ASM/ADM) stays in Pdx203Audio.cpp, which calls
// the four entry points below with the LPASS MACRO/DCODEC votes held and the TX core clocks on.
//
// Specs (from LineageOS android_kernel_sony_sm8250 techpack/audio and upstream Linux; [read]/[inferred] marked there):
//   /tmp/kmdconn/hph-lpass-spec.md    RX/TX/VA macro clocks, the RX macro headphone path, LPI pins
//   /tmp/kmdconn/hph-swr-spec.md      SoundWire master v1.5.1 init, command FIFO, enumeration, data ports and bank switch
//   /tmp/kmdconn/hph-wcd938x-spec.md  WCD938x init, mechanical jack detection, HPH power-up/down (class-H FSM)
// Every WCD938x register goes through the TX SoundWire slave (dev 1 on 0x03230000); the RX slave (dev 1 on 0x03210000) only carries the
// headphone data ports. Commands use the special command id 0xF and wait for SPECIAL_CMD_ID_FINISHED, as upstream qcom.c on this IP.
//
// Profiles (boot argument pdx203-hph-profile): 0 (default) Sony's stock headphones path: class-H HiFi with the companders, SoundWire
// ports HPH + CLSH + COMP; 1 class-AB HiFi without compander, the HPH port only, analog gain pdx203-hph-again (default 0x0A).
// pdx203-hph-vol: the RX macro digital gain byte (default 0xF7 = -9 dB, Sony's headphones path; 0 = 0 dB).
#include <IOKit/IOLib.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
#include <libkern/libkern.h>

extern "C" {
// [0] stage (positive progress, negative the step that failed), [1] detail of the failure, [2] TX master version, [3] RX master version,
// [4] TX slave status, [5] RX slave status, [6] TX enumerated id, [7] RX enumerated id, [8] chip id 0x3401..0x3404, [9] variant
// (0 WCD9380, 5 WCD9385), [10] jack (-1 unknown, 0 out, 1 in), [11] jack events, [12] ANA_HPH << 8 | RX_SUPPLIES after power-up,
// [13] INTR status 0 | 1 << 8 | 2 << 16 (last read), [14] SoundWire command failures, [15] power-ups
volatile int32_t pdx203_hph_stats[16];
int pdx203_hph_init(void);
int pdx203_hph_jack(void);
int pdx203_hph_on(void);
void pdx203_hph_off(void);
void pdx203_hph_print(const char *what);
}

namespace {
struct Region {IOMemoryMap *map; uintptr_t va;};
bool mapRegion(uint64_t phys, uint64_t length, Region &r) {
    IODeviceMemory *memory = IODeviceMemory::withRange(phys, length);
    r.map = memory ? memory->map(kIOMapAnywhere | kIOMapInhibitCache) : nullptr;
    if (memory) memory->release();
    r.va = r.map ? static_cast<uintptr_t>(r.map->getVirtualAddress()) : 0;
    return r.va != 0;
}
uint32_t rd(const Region &r, uint32_t at) {return *reinterpret_cast<volatile uint32_t *>(r.va + at);}
void wr(const Region &r, uint32_t at, uint32_t value) {*reinterpret_cast<volatile uint32_t *>(r.va + at) = value; __asm__ volatile("dsb sy" ::: "memory");}
// The codec macros' registers are 32-bit words carrying 8 bits.
void rmw(const Region &r, uint32_t at, uint32_t mask, uint32_t value) {wr(r, at, (rd(r, at) & ~mask & 0xffu) | (value & mask));}
void setB(const Region &r, uint32_t at, uint32_t bits) {rmw(r, at, bits, bits);}
void clrB(const Region &r, uint32_t at, uint32_t bits) {rmw(r, at, bits, 0);}
void delayUs(uint32_t us) {
    uint64_t start, now, ticks = (uint64_t)us * 192 / 10;
    __asm__ volatile("isb\n\tmrs %0, cntpct_el0" : "=r"(start));
    do {__asm__ volatile("isb\n\tmrs %0, cntpct_el0" : "=r"(now));} while (now - start < ticks);
}
int fail(int step, int detail) {pdx203_hph_stats[0] = -step; pdx203_hph_stats[1] = detail; return -step;}

constexpr uint64_t kRxMacro = 0x03200000, kRxSwr = 0x03210000, kTxMacro = 0x03220000, kTxSwr = 0x03230000, kVaMacro = 0x03370000;
constexpr uint64_t kLpi = 0x033c0000, kLpiSlew = 0x0355a000, kTlmmReset = 0x0f520000;   // gpio32, south tile
Region gRx, gRxS, gTx, gTxS, gVa, gLpi, gSlew, gReset;
bool gReady, gOn;
unsigned gProfile, gVol = 0xf7, gAgain = 0x0a;
uint32_t gVariant;

// ---- SoundWire master (swr-mstr-ctrl.c v1.5.1 register map) ----
constexpr uint32_t kHwVersion = 0x000, kCompCfg = 0x004, kSwReset = 0x008, kCompStatus = 0x014, kCompParams = 0x100, kIntStatus = 0x200,
                   kIntMask = 0x204, kIntClear = 0x208, kCpuIntEn = 0x210, kWrCmd = 0x300, kRdCmd = 0x304, kFifoCmd = 0x308,
                   kFifoStatus = 0x30c, kFifoCfg = 0x314, kRdFifo = 0x318, kEnumCfg = 0x500, kBusCtrl = 0x1044, kMcpCfg = 0x1048,
                   kMcpStatus = 0x104c, kSlvStatus = 0x1090;
constexpr uint32_t kSpecialDone = 0x400, kCmdError = 0x80, kIntAll = 0x1fdfd;
constexpr uint32_t frameCtrl(unsigned bank) {return 0x101c + 0x40 * bank;}
constexpr uint32_t devId1(unsigned dev) {return 0x530 + 8 * dev;}
constexpr uint8_t kBroadcast = 0xf;

struct Master {Region *r; const char *name; uint32_t wrDepth;};
Master gTxM = {&gTxS, "tx", 0}, gRxM = {&gRxS, "rx", 0};

void clearInt(const Master &m, uint32_t bits) {wr(*m.r, kIntClear, bits); wr(*m.r, kIntClear, 0);}
// One command through the FIFO with the special id: wait for room, issue, wait for SPECIAL_CMD_ID_FINISHED (or CMD_ERROR, then flush).
int swrCmd(const Master &m, bool read, uint8_t dev, uint16_t reg, uint8_t data, uint8_t *out) {
    for (unsigned i = 0; i < 30 && ((rd(*m.r, kFifoStatus) >> 8) & 0x1f) >= m.wrDepth; ++i) delayUs(500);
    clearInt(m, kSpecialDone | kCmdError);
    const uint32_t word = reg | 0xfu << 16 | (uint32_t)(dev & 0xf) << 20 | (uint32_t)(read ? 1 : data) << 24;
    wr(*m.r, read ? kRdCmd : kWrCmd, word);
    uint32_t st = 0;
    for (unsigned i = 0; i < 20000; ++i) {   // 100 ms
        st = rd(*m.r, kIntStatus);
        if (st & (kSpecialDone | kCmdError)) break;
        delayUs(5);
    }
    if (!(st & kSpecialDone) || (st & kCmdError)) {
        wr(*m.r, kFifoCmd, 1);
        clearInt(m, kSpecialDone | kCmdError);
        pdx203_hph_stats[14] = pdx203_hph_stats[14] + 1;
        return (st & kCmdError) ? -2 : -1;
    }
    clearInt(m, kSpecialDone);
    if (!read) return 0;
    for (unsigned i = 0; i < 30 && !((rd(*m.r, kFifoStatus) >> 16) & 0x1f); ++i) delayUs(500);
    if (!((rd(*m.r, kFifoStatus) >> 16) & 0x1f)) {pdx203_hph_stats[14] = pdx203_hph_stats[14] + 1; return -3;}
    *out = (uint8_t)(rd(*m.r, kRdFifo) & 0xff);
    return 0;
}
int swrWrite(const Master &m, uint8_t dev, uint16_t reg, uint8_t v) {return swrCmd(m, false, dev, reg, v, nullptr);}

// swrm_master_init (spec 1.4): software reset, the init list with 50 us between writes, then continue-on-ignore.
int masterInit(Master &m, int32_t &version) {
    wr(*m.r, kSwReset, 1); wr(*m.r, kSwReset, 1);
    delayUs(200);
    version = (int32_t)rd(*m.r, kHwVersion);
    const uint32_t params = rd(*m.r, kCompParams);
    m.wrDepth = (params >> 10) & 0x1f;
    if (!m.wrDepth) m.wrDepth = 8;
    const uint32_t mcp = (rd(*m.r, kMcpCfg) & ~0x003e0000u) | (0x1fu << 17);
    const uint32_t list[][2] = {{frameCtrl(0), 0x002f0008}, {kEnumCfg, 1}, {kMcpCfg, mcp}, {kFifoCfg, 3}, {kBusCtrl, 2}, {kCompCfg, 2},
                                {kIntClear, 0xffffffffu}, {kIntMask, kIntAll}, {kCpuIntEn, kIntAll}, {kCompCfg, 3}};
    for (const auto &e : list) {delayUs(55); wr(*m.r, e[0], e[1]);}
    delayUs(110);
    wr(*m.r, kIntClear, 0);
    for (unsigned i = 0; i < 100 && !(rd(*m.r, kCompStatus) & 1); ++i) delayUs(500);   // advisory on 1.5.1
    wr(*m.r, kFifoCfg, rd(*m.r, kFifoCfg) | 0x80000000u);
    return 0;
}
// Hardware auto-enumeration gives the WCD938x device number 1 on each bus (spec 3.2).
bool attached(const Master &m) {const uint32_t s = (rd(*m.r, kSlvStatus) >> 2) & 3; return s == 1 || s == 2;}

// ---- WCD938x registers over the TX slave ----
int gWcdErr;
uint8_t wcdRd(uint16_t reg) {uint8_t v = 0; if (swrCmd(gTxM, true, 1, reg, 0, &v)) gWcdErr++; return v;}
void wcdWr(uint16_t reg, uint8_t v) {if (swrWrite(gTxM, 1, reg, v)) gWcdErr++;}
void wcdRmw(uint16_t reg, uint8_t mask, uint8_t v) {wcdWr(reg, (uint8_t)((wcdRd(reg) & ~mask) | (v & mask)));}

// ---- headphone data ports (spec 4.3/4.4): {master port, slave port idx, slave ch, master ch, si, off1, off2, hstart, hstop, wd, bp,
// bgp, lane}, 0xff = not written ----
struct Port {uint8_t mport, sidx, sch, mch, si, off1, off2, hstart, hstop, wd, bp, bgp, lane;};
constexpr Port kPorts[3] = {
    {1, 0, 3, 3, 3, 0, 0, 0xff, 0xff, 1, 0xff, 0xff, 1},     // HPH L+R, data lane 1 (LPI gpio5)
    {2, 1, 1, 1, 31, 0, 0, 3, 6, 7, 0, 0xff, 0},             // CLSH
    {3, 2, 3, 3, 31, 11, 11, 0xff, 0xff, 4, 1, 0xff, 0},     // COMP L+R
};
unsigned portCount() {return gProfile == 0 ? 3u : 1u;}
int portConfig(unsigned bank, bool on) {
    int rc = 0;
    for (unsigned i = 0; i < portCount(); ++i) {
        const Port &p = kPorts[i];
        const uint16_t s = (uint16_t)(0x100 * (p.sidx + 1)), b = (uint16_t)(0x10 * bank);
        const uint32_t n = 0x100u * (p.mport - 1), mb = 0x40u * bank;
        if (!on) {
            rc |= swrWrite(gRxM, 1, s + 0x20 + b, 0);
            wr(gRxS, 0x1124 + n + mb, (uint32_t)p.off2 << 16 | (uint32_t)p.off1 << 8 | p.si);
            continue;
        }
        const uint16_t regs[] = {(uint16_t)(s + 0x20 + b), (uint16_t)(s + 0x22 + b), (uint16_t)(s + 0x24 + b)};
        const uint8_t vals[] = {p.sch, p.si, p.off1};
        for (unsigned k = 0; k < 3; ++k) {delayUs(55); rc |= swrWrite(gRxM, 1, regs[k], vals[k]);}
        if (p.off2 != 0xff) rc |= swrWrite(gRxM, 1, s + 0x25 + b, p.off2);
        if (p.hstart != 0xff && p.hstop != 0xff) rc |= swrWrite(gRxM, 1, s + 0x26 + b, (uint8_t)(p.hstart << 4 | p.hstop));
        if (p.wd != 0xff) rc |= swrWrite(gRxM, 1, s + 0x03, p.wd);
        if (p.bp != 0xff) rc |= swrWrite(gRxM, 1, s + 0x27 + b, p.bp);
        if (p.bgp != 0xff) rc |= swrWrite(gRxM, 1, s + 0x21 + b, p.bgp);
        if (p.lane != 0xff) rc |= swrWrite(gRxM, 1, s + 0x28 + b, p.lane);
        wr(gRxS, 0x1124 + n + mb, (uint32_t)p.mch << 24 | (p.off2 != 0xff ? (uint32_t)p.off2 << 16 : 0) | (uint32_t)p.off1 << 8 | p.si);
        if (p.lane != 0xff) wr(gRxS, 0x1128 + n + mb, p.lane);
        if (p.wd != 0xff) wr(gRxS, 0x112c + n, p.wd);
        if (p.bgp != 0xff) wr(gRxS, 0x1130 + n + mb, p.bgp);
        wr(gRxS, 0x1134 + n + mb, (p.hstart == 0xff || p.hstop == 0xff) ? 0xf0u : (uint32_t)(p.hstop << 4 | p.hstart));
        if (p.bp != 0xff) wr(gRxS, 0x1138 + n + mb, p.bp);
    }
    delayUs(110);
    return rc;
}
// Frame shape into the inactive bank and the broadcast that switches every device to it at the next frame (spec 4.4 step 5-6).
int bankSwitch(unsigned bank, unsigned ssp, unsigned col) {
    const uint32_t v = (rd(gRxS, frameCtrl(bank)) & ~0x00ff07ffu) | ssp << 16 | 1u << 3 | col;
    wr(gRxS, frameCtrl(bank), v);
    const int rc = swrWrite(gRxM, kBroadcast, (uint16_t)(0x60 + 0x10 * bank), (uint8_t)(1u << 3 | col));
    for (unsigned i = 0; i < 100 && (rd(gRxS, kMcpStatus) & 1) != bank; ++i) delayUs(100);
    return rc ? rc : ((rd(gRxS, kMcpStatus) & 1) == bank ? 0 : -4);
}
int portsOn() {
    const unsigned bank = (rd(gRxS, kMcpStatus) & 1) ? 0 : 1;
    wr(gRxS, kCpuIntEn, kIntAll);
    int rc = portConfig(bank, true);
    if (!rc) rc = bankSwitch(bank, 5, 7);   // 50 x 16 while ports run
    if (!rc) rc = portConfig(bank ^ 1, true);
    return rc;
}
int portsOff() {
    const unsigned bank = (rd(gRxS, kMcpStatus) & 1) ? 0 : 1;
    int rc = portConfig(bank, false);
    rc |= bankSwitch(bank, 0x2f, 0);        // back to 50 x 2
    rc |= portConfig(bank ^ 1, false);
    return rc;
}

// ---- RX macro (offsets from 0x03200000) ----
constexpr uint32_t kPathCtl[2] = {0x400, 0x480}, kCfg0[2] = {0x404, 0x484}, kCfg1[2] = {0x408, 0x488}, kCfg2[2] = {0x40c, 0x48c},
                   kCfg3[2] = {0x410, 0x490}, kVol[2] = {0x414, 0x494}, kMixCtl[2] = {0x418, 0x498}, kDsm[2] = {0x440, 0x4c0},
                   kComp0[2] = {0x800, 0x840}, kComp7[2] = {0x81c, 0x85c}, kCoefLsb[2] = {0x020, 0x034}, kCoefMsb[2] = {0x024, 0x038},
                   kLut[2] = {0x028, 0x03c};
// rx-macro.c comp_coeff_table[LOHIFI] (Sony's RX_HPH_PWR_MODE)
constexpr uint8_t kCompCoeff[25][2] = {{0x40, 0}, {0x4C, 0}, {0x5A, 0}, {0x6B, 0}, {0x80, 0}, {0x98, 0}, {0xB4, 0}, {0xD5, 0}, {0xFE, 0},
    {0x2E, 1}, {0x66, 1}, {0xA9, 1}, {0xF8, 1}, {0x56, 2}, {0xC4, 2}, {0x4F, 3}, {0xF0, 3}, {0xAE, 4}, {0x8B, 5}, {0x8E, 6}, {0xBC, 7},
    {0x56, 9}, {0x0F, 0xB}, {0x13, 0xD}, {0x6F, 0xF}};
void muteRx(unsigned n, bool mute) {
    if (mute) {setB(gRx, kPathCtl[n], 0x10); setB(gRx, kMixCtl[n], 0x10);}
    else {clrB(gRx, kPathCtl[n], 0x10); clrB(gRx, kMixCtl[n], 0x10);}
}
// The "headphones" mixer path and the interpolator power-up (lpass spec 3.3/3.4), both channels; the path stays muted.
void macroPathOn() {
    const bool a = gProfile == 0;
    rmw(gRx, 0x180, 0x0f, 0x05);   // INT0 MIX1 INP0 = RX0
    rmw(gRx, 0x188, 0x0f, 0x06);   // INT1 MIX1 INP0 = RX1
    for (unsigned n = 0; n < 2; ++n) {
        if (a) {setB(gRx, kCfg0[n], 0x08); setB(gRx, kCfg1[n], 0x01);}   // DEM input = CLSH_DSM_OUT (with look-ahead delay)
        else {clrB(gRx, kCfg0[n], 0x08); clrB(gRx, kCfg1[n], 0x01);}
        wr(gRx, kVol[n], gVol & 0xff);
        rmw(gRx, kPathCtl[n], 0x0f, 0x04);                                 // 48 kHz
    }
    for (unsigned n = 0; n < 2; ++n) {
        setB(gRx, kPathCtl[n], 0x10);                                      // PGA mute
        setB(gRx, kDsm[n], 0x01);                                          // DSM clock
        rmw(gRx, kCfg2[n], 0x03, 0x03);                                    // HPF
        if (a) for (unsigned k = 0; k < 25; ++k) {wr(gRx, kCoefLsb[n], kCompCoeff[k][0]); wr(gRx, kCoefMsb[n], kCompCoeff[k][1]);}
        setB(gRx, kLut[n], 0x80);                                          // HPH delay LUT bypass
        clrB(gRx, kComp7[n], 0x20);                                        // LOHIFI
        rmw(gRx, kCfg3[n], 0x03, 0x03);
        if (a) {setB(gRx, kComp0[n], 0x01); setB(gRx, kComp0[n], 0x02); clrB(gRx, kComp0[n], 0x02); setB(gRx, kCfg0[n], 0x02);}
        if (a) {
            if (n == 0) {
                setB(gRx, 0x200, 0x01);                                    // class-H block clock
                wr(gRx, 0x220, 0xc0); rmw(gRx, 0x21c, 0x0f, 0x00);         // K1 (16 ohm)
                rmw(gRx, 0x20c, 0x3f, 0x1c); rmw(gRx, 0x208, 0x07, 0x00);
            }
            setB(gRx, kCfg0[n], 0x40);                                     // CLSH_EN
        }
        wr(gRx, kVol[n], rd(gRx, kVol[n]) & 0xff);                         // gain latches now that the path is clocked
    }
}
void macroPathOff() {
    const bool a = gProfile == 0;
    for (unsigned n = 0; n < 2; ++n) {
        setB(gRx, kPathCtl[n], 0x10);
        clrB(gRx, kDsm[n], 0x01);
        clrB(gRx, kPathCtl[n], 0x20);
        setB(gRx, kPathCtl[n], 0x40); clrB(gRx, kPathCtl[n], 0x40);        // path reset pulse
        rmw(gRx, kPathCtl[n], 0x0f, 0x04);
        rmw(gRx, kCfg2[n], 0x03, 0x00);
        rmw(gRx, kCfg3[n], 0x03, 0x03);
        if (a) {setB(gRx, kComp0[n], 0x04); clrB(gRx, kCfg0[n], 0x02); clrB(gRx, kComp0[n], 0x01); clrB(gRx, kComp0[n], 0x04);}
        clrB(gRx, kLut[n], 0x80);
        setB(gRx, kComp7[n], 0x20);
    }
    if (a) clrB(gRx, 0x200, 0x01);
    clrB(gRx, kCfg1[0], 0x02);
}

// ---- WCD938x init and jack detection (wcd spec 0.2/0.3) ----
void wcdInit() {
    wcdWr(0x3474, 0); wcdWr(0x3475, 0); wcdWr(0x3476, 0);                 // all interrupts edge
    wcdWr(0x346B, 0xff); wcdWr(0x346C, 0xff); wcdRmw(0x346D, 0x0f, 0x0f);  // masked
    for (uint16_t i = 0; i < 3; ++i) {const uint8_t s = wcdRd(0x346E + i); if (s) {wcdWr(0x3471 + i, s); wcdWr(0x3471 + i, 0);}}
    static const uint16_t kMicb[4] = {0x3022, 0x3023, 0x3025, 0x3026}, kMicbTest[4] = {0x306B, 0x306E, 0x3071, 0x3074};
    for (uint16_t r : kMicb) wcdRmw(r, 0x3f, 0x10);
    wcdRmw(0x3103, 0x0e, 0x0e);
    wcdRmw(0x3103, 0x80, 0x80); IOSleep(1);
    wcdRmw(0x3103, 0x40, 0x40); IOSleep(1);
    wcdRmw(0x312A, 0x10, 0x00);
    wcdRmw(0x3029, 0xf0, 0x80);
    wcdRmw(0x3001, 0x80, 0x80);
    wcdRmw(0x3001, 0x40, 0x40); IOSleep(10);
    wcdRmw(0x3001, 0x40, 0x00);
    wcdRmw(0x3132, 0xf0, 0x00);
    wcdRmw(0x3140, 0x1f, 0x15);
    wcdRmw(0x3141, 0x1f, 0x15);
    wcdRmw(0x30DD, 0xc0, 0x80);
    wcdRmw(0x345B, 0x02, 0x02);
    wcdRmw(0x31E1, 0xff, 0x14);
    wcdRmw(0x31DF, 0x1f, 0x08);
    const uint8_t fb[5] = {0x55, 0x44, 0x11, 0x00, 0x00};
    for (uint16_t i = 0; i < 5; ++i) wcdWr(0x34D0 + i, fb[i]);
    for (uint16_t r : kMicbTest) wcdRmw(r, 0xe0, 0xe0);
    wcdRmw(0x308F, 0x01, 0x00);
    wcdRmw(0x3103, 0x0e, (uint8_t)((wcdRd(0x34CE) & 0x07) << 1));
    wcdRmw(0x30E2, 0xc0, 0xc0);                                            // surge protection
    // mechanical plug detection (hphl-swh = 1, gnd-swh = 1)
    wcdRmw(0x31B1, 0x1f, 0x08);
    wcdRmw(0x3121, 0x0c, 0x00);
    wcdRmw(0x3014, 0x10, 0x10); wcdRmw(0x3014, 0x08, 0x08); wcdRmw(0x3014, 0x01, 0x01); wcdRmw(0x3014, 0x04, 0x04);
    wcdRmw(0x3014, 0x20, 0x20);                                            // look for insertion
    wcdRmw(0x3014, 0x80, 0x80);                                            // L_DET_EN
    wcdRmw(0x3122, 0x0f, 0x06);
    wcdRmw(0x3120, 0x03, 0x02);
    wcdRmw(0x3015, 0x01, 0x01);
    wcdRmw(0x3120, 0x80, 0x80);
    wcdRmw(0x3121, 0x03, 0x03);
    wcdWr(0x3471, 0x10); wcdWr(0x3471, 0x00);
    wcdRmw(0x346B, 0x10, 0x00);                                            // unmask MBHC_SW_DET
}

// ---- headphone power (wcd spec 0.4/0.5 with the lpass spec's RX macro steps) ----
void wcdOn() {
    const bool a = gProfile == 0;
    // RDAC1/RDAC2 PRE_PMU: RX clocks (first user), RXD0/RXD1 clocks, HPH RX enables
    wcdRmw(0x3408, 0x01, 0x01); wcdRmw(0x3008, 0x01, 0x01);
    wcdRmw(0x340D, 0x40, 0x00); wcdRmw(0x340E, 0x40, 0x00); wcdRmw(0x340F, 0x40, 0x00);
    wcdRmw(0x3408, 0x02, 0x02); wcdRmw(0x3128, 0x10, 0x10);
    wcdRmw(0x3409, 0x01, 0x01); wcdRmw(0x344E, 0x04, 0x04); wcdRmw(0x30D9, 0x80, 0x00);
    wcdRmw(0x3409, 0x02, 0x02); wcdRmw(0x344E, 0x08, 0x08); wcdRmw(0x30D9, 0x80, 0x00);
    // RDAC POST_PMU: HD2, compander or register gain
    wcdRmw(0x3133, 0x0f, 0x02);
    if (a) {wcdRmw(0x3414, 0x02, 0x02); wcdRmw(0x313A, 0x02, 0x00);}
    else {wcdRmw(0x3414, 0x02, 0x00); wcdRmw(0x30D3, 0x20, 0x20);}
    wcdRmw(0x3136, 0x0f, 0x02);
    if (a) {wcdRmw(0x3414, 0x01, 0x01); IOSleep(5); wcdRmw(0x313A, 0x02, 0x00);}
    else {wcdRmw(0x3414, 0x01, 0x00); wcdRmw(0x30D6, 0x20, 0x20); wcdRmw(0x30D3, 0x1f, (uint8_t)gAgain); wcdRmw(0x30D6, 0x1f, (uint8_t)gAgain);}
}
int paOn() {
    const bool a = gProfile == 0;
    const bool hifi = gVariant == 5;
    const uint8_t mode = hifi ? 0x08 : 0x0c;   // ANA_HPH power level: HiFi on WCD9385, class-AB normal on WCD9380
    // class-H FSM, first channel
    wcdRmw(0x3008, 0x02, 0x00);
    wcdRmw(0x3008, 0x04, hifi ? 0x04 : 0x00);
    wcdRmw(0x30A8, 0xf0, hifi ? 0x80 : 0x70);
    wcdRmw(0x30AF, 0xe0, 0xa0); delayUs(100);
    wcdRmw(0x3099, 0x02, 0x02);
    wcdWr(0x3098, 0x1c);
    wcdRmw(0x30A5, 0xe0, 0xe0);
    wcdRmw(0x3008, 0x40, 0x40); delayUs(100);
    wcdRmw(0x30AF, 0xe0, 0xe0); delayUs(500);
    wcdRmw(0x30C7, 0x0f, 0x0a); wcdRmw(0x30C7, 0xf0, 0xa0); delayUs(100);
    wcdRmw(0x3008, 0x08, hifi ? 0x08 : 0x00);
    wcdRmw(0x3008, 0x80, 0x80); delayUs(500);
    if (a) wcdRmw(0x3099, 0x02, 0x00);
    wcdWr(0x3098, 0x3a); delayUs(500);
    wcdRmw(0x3009, 0x0c, mode);
    // HPHL / HPHR PA PRE_PMU
    wcdRmw(0x3009, 0x0c, 0x08); wcdRmw(0x3009, 0x20, 0x20); wcdRmw(0x3009, 0x0c, mode); delayUs(100);
    wcdRmw(0x3465, 0x07, 0x03);
    wcdRmw(0x3009, 0x0c, 0x08); wcdRmw(0x3009, 0x10, 0x10); wcdRmw(0x3009, 0x0c, mode); delayUs(100);
    wcdRmw(0x3466, 0x07, 0x03);
    // both PAs in one write, then the settle time
    wcdRmw(0x3009, 0xc0, 0xc0);
    IOSleep(a ? 7 : 20);
    wcdRmw(0x313A, 0x02, 0x02);
    if (!a) wcdRmw(0x3008, 0x02, 0x02);
    return 0;
}
void paOff() {
    const bool a = gProfile == 0;
    muteRx(0, true);
    if (a) wr(gRx, kComp0[0], rd(gRx, kComp0[0]) & 0xff);
    IOSleep(a ? 7 : 20);
    wcdRmw(0x3009, 0x80, 0x00);
    muteRx(1, true);
    if (a) wr(gRx, kComp0[1], rd(gRx, kComp0[1]) & 0xff);
    IOSleep(a ? 7 : 20);
    wcdRmw(0x3009, 0x40, 0x00);
    IOSleep(a ? 7 : 21);
    wcdRmw(0x3009, 0x20, 0x00); wcdRmw(0x3465, 0x07, 0x00);
    wcdRmw(0x3009, 0x10, 0x00); wcdRmw(0x3466, 0x07, 0x00);
    // class-H off (last channel)
    wcdRmw(0x3009, 0x0c, 0x00);
    wcdRmw(0x30A5, 0xe0, 0xe0);
    wcdRmw(0x3008, 0x40, 0x00); delayUs(100);
    wcdRmw(0x30AF, 0xe0, 0xe0); delayUs(500);
    wcdRmw(0x3008, 0x80, 0x00); delayUs(500);
    wcdWr(0x3098, 0x3a); delayUs(500);
    wcdRmw(0x3138, 0x20, 0x00);
    wcdRmw(0x30BF, 0xf0, 0x80);
    wcdRmw(0x30D1, 0x0e, 0x06);
    wcdRmw(0x3008, 0x04, 0x00);
    wcdRmw(0x30A8, 0xf0, 0x70);
    wcdRmw(0x3008, 0x08, 0x00);
    // RX clocks off (last user), RDAC POST_PMD
    wcdRmw(0x3008, 0x40, 0x00); wcdRmw(0x3008, 0x80, 0x00); wcdRmw(0x3008, 0x01, 0x00);
    wcdRmw(0x3408, 0x02, 0x00); wcdRmw(0x3408, 0x01, 0x00);
    wcdRmw(0x3409, 0x01, 0x00); wcdRmw(0x3409, 0x02, 0x00);
    wcdRmw(0x3136, 0x0f, 0x01);
}

// LPI pins of both SoundWire buses (lpass spec 5.2): TX clk/data/data, RX clk/data/data1 (func2).
void lpiPins() {
    const uint32_t cfg[6] = {0x004, 0x006, 0x006, 0x004, 0x006, 0x00a};
    for (unsigned i = 0; i < 6; ++i) wr(gLpi, i * 0x1000, (rd(gLpi, i * 0x1000) & ~0x3ffu) | cfg[i]);
    wr(gSlew, 0, (rd(gSlew, 0) & ~0x3f3fu) | 0x1515u);
}
void resetPin(bool high) {
    wr(gReset, 4, high ? 2u : 0u);
    wr(gReset, 0, (rd(gReset, 0) & ~0x3ffu) | (7u << 6) | 0x200u);   // GPIO, no pull, 16 mA, output
}
}  // namespace

// With the votes held and AFE clocks 0x30C/0x30D on: macro clocks, both SoundWire buses, the codec out of reset and enumerated, its
// init and jack detection. 0 when the headphone path is usable.
extern "C" int pdx203_hph_init(void) {
    PE_parse_boot_argn("pdx203-hph-profile", &gProfile, sizeof gProfile);
    PE_parse_boot_argn("pdx203-hph-vol", &gVol, sizeof gVol);
    PE_parse_boot_argn("pdx203-hph-again", &gAgain, sizeof gAgain);
    if (gProfile > 1) gProfile = 0;
    pdx203_hph_stats[10] = -1;
    if (!mapRegion(kRxMacro, 0x1000, gRx) || !mapRegion(kRxSwr, 0x2000, gRxS) || !mapRegion(kTxMacro, 0x1000, gTx) ||
        !mapRegion(kTxSwr, 0x2000, gTxS) || !mapRegion(kVaMacro, 0x1000, gVa) ||
        !mapRegion(kLpi, 6 * 0x1000, gLpi) || !mapRegion(kLpiSlew, 0x1000, gSlew) || !mapRegion(kTlmmReset, 0x1000, gReset))
        return fail(1, -12);
    pdx203_hph_stats[0] = 1;
    resetPin(false);                                   // the codec held in reset while the buses come up
    // 0x033240D8 (the RX MCLK mux) is only written for 44.1 kHz-family rates and is left alone (lpass spec 2.3)
    lpiPins();
    setB(gVa, 0x000, 0x01); setB(gVa, 0x004, 0x01); setB(gVa, 0x080, 0x02);   // FS generator
    rmw(gRx, 0x434, 0x07, 0x02); rmw(gRx, 0x4b4, 0x07, 0x02); rmw(gRx, 0x540, 0x07, 0x02);   // rx_macro_init
    rmw(gRx, 0x410, 0x03, 0x02); rmw(gRx, 0x490, 0x03, 0x02); rmw(gRx, 0x510, 0x03, 0x02);
    setB(gRx, 0x100, 0x01); setB(gRx, 0x100, 0x02); clrB(gRx, 0x104, 0x02); setB(gRx, 0x104, 0x01);   // RX MCLK
    setB(gRx, 0x108, 0x02); setB(gRx, 0x108, 0x01); clrB(gRx, 0x108, 0x02);                            // RX SWR clock
    setB(gTx, 0x090, 0x01); setB(gTx, 0x000, 0x01); setB(gTx, 0x004, 0x01);                            // TX MCLK
    setB(gTx, 0x008, 0x02); setB(gTx, 0x008, 0x01); clrB(gTx, 0x008, 0x02);                            // TX SWR clock
    pdx203_hph_stats[0] = 2;
    int32_t txv = 0, rxv = 0;
    masterInit(gTxM, txv); masterInit(gRxM, rxv);
    pdx203_hph_stats[2] = txv; pdx203_hph_stats[3] = rxv;
    if (((uint32_t)txv & 0xffff0000u) != 0x01050000u || ((uint32_t)rxv & 0xffff0000u) != 0x01050000u) return fail(3, txv);
    pdx203_hph_stats[0] = 3;
    delayUs(30); resetPin(true); delayUs(30);
    IOSleep(10);
    for (unsigned i = 0; i < 100 && !(attached(gTxM) && attached(gRxM)); ++i) IOSleep(2);
    pdx203_hph_stats[4] = (int32_t)rd(gTxS, kSlvStatus); pdx203_hph_stats[5] = (int32_t)rd(gRxS, kSlvStatus);
    pdx203_hph_stats[6] = (int32_t)rd(gTxS, devId1(1)); pdx203_hph_stats[7] = (int32_t)rd(gRxS, devId1(1));
    clearInt(gTxM, 0x1806); clearInt(gRxM, 0x1806);
    if (!attached(gTxM) || !attached(gRxM)) return fail(4, (int32_t)rd(gTxS, kIntStatus));
    pdx203_hph_stats[0] = 4;
    uint32_t id = 0;
    for (uint16_t i = 0; i < 4; ++i) id = id << 8 | wcdRd(0x3401 + i);
    gVariant = (wcdRd(0x34B0) & 0x1e) >> 1;
    pdx203_hph_stats[8] = (int32_t)id; pdx203_hph_stats[9] = (int32_t)gVariant;
    if (gWcdErr) return fail(5, gWcdErr);
    if (gVariant != 5 && gProfile == 0) gProfile = 1;   // class-H HiFi needs the WCD9385; a WCD9380 runs class-AB
    pdx203_hph_stats[0] = 5;
    wcdInit();
    if (gWcdErr) return fail(6, gWcdErr);
    pdx203_hph_stats[0] = 6;
    gReady = true;
    return 0;
}

// Polled every ~100 ms: the mechanical switch event (MBHC_SW_DET) flips the detection type, as wcd_mbhc_swch_irq_handler does.
// Returns -1 while the codec is not up, else 0 out / 1 in.
extern "C" int pdx203_hph_jack(void) {
    if (!gReady) return -1;
    const uint8_t s0 = wcdRd(0x346E);
    pdx203_hph_stats[13] = (int32_t)(s0 | (uint32_t)wcdRd(0x346F) << 8 | (uint32_t)wcdRd(0x3470) << 16 | (uint32_t)wcdRd(0x3019) << 24);
    if (s0 & 0x10) {
        wcdWr(0x3471, 0x10); wcdWr(0x3471, 0x00);
        const uint8_t mech = wcdRd(0x3014);
        const int type = (mech >> 5) & 1;
        wcdRmw(0x3014, 0x20, type ? 0x00 : 0x20);
        pdx203_hph_stats[10] = type;
        pdx203_hph_stats[11] = pdx203_hph_stats[11] + 1;
    } else if (pdx203_hph_stats[10] < 0) {
        pdx203_hph_stats[10] = 0;
    }
    return pdx203_hph_stats[10];
}

// With the AFE port 0xB030 started (the DMA runs): the RX macro path, the WCD's RX clocks, the SoundWire data ports, class-H and the PAs,
// then unmute and the interpolator clocks.
extern "C" int pdx203_hph_on(void) {
    if (!gReady) return -19;
    if (gOn) return 0;
    gWcdErr = 0;
    macroPathOn();
    wcdOn();
    muteRx(0, true); muteRx(1, true);
    const int prc = portsOn();
    if (prc) {printf("PDX203 HPH data ports failed (%d)\n", prc); portsOff(); macroPathOff(); return -5;}
    paOn();
    muteRx(0, false); muteRx(1, false);
    setB(gRx, kPathCtl[0], 0x20); setB(gRx, kPathCtl[1], 0x20);           // interpolator clocks: audio flows
    gOn = true;
    pdx203_hph_stats[12] = (int32_t)((uint32_t)wcdRd(0x3009) << 8 | wcdRd(0x3008));
    pdx203_hph_stats[15] = pdx203_hph_stats[15] + 1;
    pdx203_hph_print("on");
    return gWcdErr ? -gWcdErr : 0;
}
extern "C" void pdx203_hph_off(void) {
    if (!gReady || !gOn) return;
    paOff();
    portsOff();
    macroPathOff();
    gOn = false;
}

extern "C" void pdx203_hph_print(const char *what) {
    const volatile int32_t *s = pdx203_hph_stats;
    printf("PDX203 HPH %s stage %d detail %x | swr tx %x rx %x slv %x/%x id %x/%x | chip %08x variant %d profile %u | jack %d events %d "
           "intr %08x | pa %x | cmd fail %d ons %d\n", what, s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8], s[9], gProfile, s[10],
           s[11], s[13], s[12], s[14], s[15]);
}
