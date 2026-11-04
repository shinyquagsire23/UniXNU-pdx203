// SPDX-License-Identifier: BSD-2-Clause-Patent
// Audio on the pdx203 board, stage 1: reach both halves of the speaker path and report what answers. Nothing plays yet.
//
// Hardware (Sony kona-edo-common.dtsi, LineageOS android_kernel_sony_sm8250):
//   - two Cirrus Logic CS35L41 amplifiers at I2C 0x40 (left) and 0x41 (right) on QUP wrapper 0 SE2 (0x00988000, i2c@988000,
//     SDA gpio115 / SCL gpio116, function qup2 on the north TLMM tile 0x0f900000), shared reset gpio10 (active low) and interrupt gpio11;
//   - their audio comes from the DSP's senary MI2S on the LPASS LPI pins (lpi_i2s2), configured through APR (AFE) later.
// GCC (Linux gcc-sm8250.c): wrapper 0 core 2x / core / m_ahb / s_ahb and s2 are vote bits 9, 8, 6, 7 and 12 of 0x52008; the s2 clock
// source (an RCG with DFS) is at 0x17270.
//
// Boot argument pdx203-audio (bits): 1 reads the amplifiers' identity over I2C; 2 opens the audio DSP's APR channel "apr_audio_svc"
// over GLINK (needs pdx203-adsp=2); 4 (with 1 and 2) plays test beeps pdx203-audio-delay seconds (default 40) after the probe starts:
// pdx203-audio-beeps (default 6) beeps of 1 s of 750 Hz at -20 dBFS (left only, right only, both, repeating), 2 s apart, through AFE port SENARY_MI2S_RX, an ADM COPP and an
// ASM stream (/tmp/kmdconn/q6-playback-spec.md), with the CS35L41s set up to play ASP RX1 without their DSP
// (/tmp/kmdconn/cs35l41-init-spec.md) at pdx203-audio-vol (AMP_DIG_VOL_CTRL, default 0xbe80 = -6 dB) and pdx203-audio-gain
// (AMP_GAIN_CTRL, default 0 = 0.5 dB). Rows print as "PDX203 ADSP", "QRTR", "AUDIO", "APR" and "PLAY" on the console.
#include <IOKit/IOLib.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
#include <libkern/libkern.h>
#include <string.h>
extern "C" {
#include "ShengAdsp.h"
#include "GeniI2c.h"
#include "Cs35l41Otp.h"
#include "SmmuBypass.h"
#include "ShengAudioAbi.h"
}
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <libkern/c++/OSMetaClass.h>
#include <mach/kmod.h>
extern kmod_info_t g_kernel_kmod_info;
extern "C" void native_audio_service_static_init(void) __asm__("__GLOBAL__sub_I_ShengAudioService.cpp");
extern "C" void sheng_audio_service_start(void);
extern "C" {
extern volatile int32_t native_audio_service[8];
extern volatile int native_gpu_addon5_stage __attribute__((weak_import)), native_gpu_addon5_status __attribute__((weak_import));
}

extern "C" ShengAdspChannel *sheng_adsp_channel(const char *, uint32_t, uint32_t, int, const ShengAdspClient *, void *) __attribute__((weak_import));
extern "C" int sheng_adsp_send(ShengAdspChannel *, const void *, uint32_t) __attribute__((weak_import));
extern "C" {
extern volatile int32_t native_adsp_stats[16] __attribute__((weak_import));
extern char native_adsp_text[96] __attribute__((weak_import));
extern volatile int32_t native_qrtr_stats[16] __attribute__((weak_import));
extern char native_qrtr_text[96] __attribute__((weak_import));
// the headphone jack (Pdx203Hph.cpp): codec, SoundWire and RX macro; the DSP side is here
int pdx203_hph_init(void);
int pdx203_hph_jack(void);
int pdx203_hph_on(void);
void pdx203_hph_off(void);
void pdx203_hph_print(const char *what);
extern volatile int32_t pdx203_hph_stats[16];
// [0] stage (positive progress, negative the step that failed), [1] SE2 protocol << 24 | M_CLK_CFG, [2] amplifiers that answered (bit
// per amplifier), [3] I2C failures, [4] last I2C error status, [5] APR channel up, [6] APR messages in, [7] APR messages out,
// [8] the DSP's audio state (-1 not asked yet), [9] state queries, [10] last APR opcode in, [11] last APR basic result (opcode << 0
// of the low 16 bits | status << 16), [12] SE2 rcg cfg, [13] SE2 DFS, [14] GCC vote word after voting, [15] ms from start
volatile int32_t pdx203_audio_stats[16];
// Per amplifier: DEVID, REVID, OTPID, IRQ1 status 1
volatile uint32_t pdx203_audio_amps[2][8];
}

namespace {
constexpr uint64_t kGcc = 0x00100000, kTlmmNorth = 0x0f900000, kSe2 = 0x00988000;
constexpr uint32_t kGccVote = 0x52008, kGccVoteBits = (1u << 6) | (1u << 7) | (1u << 8) | (1u << 9) | (1u << 12);
constexpr uint32_t kSeRcg = 0x17270;
constexpr unsigned kPinSda = 115, kPinScl = 116, kPinReset = 10, kPinIrq = 11;
constexpr uint8_t kAmpAddress[2] = {0x40, 0x41};
// CS35L41 registers (cs35l41.h)
constexpr uint32_t kDevid = 0x00000000, kRevid = 0x00000004, kOtpid = 0x00000010, kIrq1Status1 = 0x00010010;

struct Region {IOMemoryMap *map; uintptr_t va;};
bool mapRegion(uint64_t phys, uint64_t length, Region &r, IOOptionBits cache = kIOMapInhibitCache) {
    IODeviceMemory *memory = IODeviceMemory::withRange(phys, length);
    r.map = memory ? memory->map(kIOMapAnywhere | cache) : nullptr;
    if (memory) memory->release();
    r.va = r.map ? static_cast<uintptr_t>(r.map->getVirtualAddress()) : 0;
    return r.va != 0;
}
uint32_t rd(const Region &r, uint32_t at) {return *reinterpret_cast<volatile uint32_t *>(r.va + at);}
void wr(const Region &r, uint32_t at, uint32_t value) {*reinterpret_cast<volatile uint32_t *>(r.va + at) = value; __asm__ volatile("dsb sy" ::: "memory");}
void fail(int step, int error) {pdx203_audio_stats[4] = error; pdx203_audio_stats[0] = -step;}

// IODelay costs a whole timer tick on this kernel (Pdx203TouchProbe.cpp), so short waits spin on the 19.2 MHz counter.
void delayUs(uint32_t us) {
    uint64_t start, now, ticks = (uint64_t)us * 192 / 10;
    __asm__ volatile("isb\n\tmrs %0, cntpct_el0" : "=r"(start));
    do {__asm__ volatile("isb\n\tmrs %0, cntpct_el0" : "=r"(now));} while (now - start < ticks);
}
bool waitBits(const Region &r, uint32_t at, uint32_t mask, uint32_t want, uint32_t us) {
    for (uint32_t i = 0; i <= us; ++i) {if ((rd(r, at) & mask) == want) return true; delayUs(1);}
    return false;
}

// ---- the amplifiers' bus ----
Region gGcc, gSe, gPins[4];
GeniI2c gBus;
uint32_t ioRd(void *ctx, uint32_t reg) {return *reinterpret_cast<volatile uint32_t *>(reinterpret_cast<uintptr_t>(ctx) + reg);}
void ioWr(void *ctx, uint32_t reg, uint32_t value) {
    *reinterpret_cast<volatile uint32_t *>(reinterpret_cast<uintptr_t>(ctx) + reg) = value;
    __asm__ volatile("dsb sy" ::: "memory");
}

// One TLMM pin's config and in/out registers (a 4 KiB page per pin): mux function, pull (0 none, 1 down, 3 up), drive code (mA / 2 - 1).
void pinConfig(const Region &p, unsigned mux, unsigned pull, unsigned drive, bool out, bool high) {
    if (out) wr(p, 4, high ? 2u : 0u);
    wr(p, 0, (rd(p, 0) & ~0x3ffu) | (mux << 2) | pull | (drive << 6) | (out ? 0x200u : 0u));
}

bool busClocks() {
    wr(gGcc, kGccVote, rd(gGcc, kGccVote) | kGccVoteBits);
    pdx203_audio_stats[14] = (int32_t)rd(gGcc, kGccVote);
    const uint32_t cfg = rd(gGcc, kSeRcg + 4), dfs = rd(gGcc, kSeRcg + 0x14);
    pdx203_audio_stats[12] = (int32_t)cfg;
    pdx203_audio_stats[13] = (int32_t)dfs;
    if (dfs & 1) {
        // DFS on: level 0 must already be the XO with divider 1 (as the touch engine's); the table is not changed
        if ((rd(gGcc, kSeRcg + 0x1c) & 0x371f) != 1) return false;
    } else {
        wr(gGcc, kSeRcg + 4, (cfg & ~(0x371fu | (1u << 20))) | 1);
        wr(gGcc, kSeRcg, rd(gGcc, kSeRcg) | 1);
        if (!waitBits(gGcc, kSeRcg, 1, 0, 5000)) return false;
    }
    return true;
}

// The engine at 400 kHz from the 19.2 MHz XO (Linux i2c-qcom-geni's table: divider 2, t_high 5, t_low 11, cycle 22) if the boot
// loader left it unconfigured, then the transfer core.
bool busSetup() {
    const uint32_t proto = (rd(gSe, GENI_FW_REVISION) >> 8) & 0xff;
    pdx203_audio_stats[1] = (int32_t)(proto << 24 | (rd(gSe, GENI_CLK_CFG) & 0xffffff));
    if (proto != GENI_PROTO_I2C || (rd(gSe, GENI_IF_DISABLE) & 1)) return false;
    if (rd(gSe, 0x40) & 0x1001) return false;   // SE_STATUS: a command is running
    if (!(rd(gSe, GENI_CLK_CFG) & 1u) || !rd(gSe, GENI_SCL_COUNTERS)) {
        wr(gSe, 0x28, rd(gSe, 0x28) | 0x7fu);    // clock gating controls on
        wr(gSe, 0xe30, rd(gSe, 0xe30) | 0xfu);   // DMA clock gating controls on
        wr(gSe, 0x24, 0x7fu);                    // default output control
        wr(gSe, 0x20, 1u);                       // force default
        wr(gSe, 0xe18, 0);                       // no GSI events
        wr(gSe, 0x7c, 0);                        // clock source level 0 (the XO)
        wr(gSe, GENI_CLK_CFG, (2u << 4) | 1u);
        wr(gSe, GENI_SCL_COUNTERS, (5u << 20) | (11u << 10) | 22u);
    }
    const GeniIo io = {ioRd, ioWr, reinterpret_cast<void *>(gSe.va)};
    uint32_t config = 0;
    if (geni_i2c_prepare(&gBus, &io, kAmpAddress[0], &config)) return false;
    pdx203_audio_stats[1] = (int32_t)config;
    return true;
}

// One transfer, polled to completion (at most 20 ms, then the core's cancel and abort steps).
bool transfer(uint8_t address, const uint8_t *bytes, uint32_t length, uint32_t readLength) {
    gBus.address = address;
    const int started = readLength ? geni_i2c_start_read_bytes(&gBus, bytes, length, readLength) : geni_i2c_start_write_bytes(&gBus, bytes, length);
    if (started) {pdx203_audio_stats[3] = pdx203_audio_stats[3] + 1; return false;}
    for (unsigned step = 0; step < 3; ++step) {
        for (unsigned us = 0; us < 20000; us += 10) {
            const int r = geni_i2c_poll(&gBus);
            if (r == GeniDone) return true;
            if (r == GeniFailed) {pdx203_audio_stats[3] = pdx203_audio_stats[3] + 1; pdx203_audio_stats[4] = (int32_t)gBus.error_status; return false;}
            delayUs(10);
        }
        const int r = geni_i2c_timeout(&gBus);
        if (r == GeniDone) return true;
        if (r == GeniFailed) break;
    }
    pdx203_audio_stats[3] = pdx203_audio_stats[3] + 1;
    pdx203_audio_stats[4] = -60;
    return false;
}
// CS35L41 registers: 32-bit address and value, both big endian on the wire.
bool ampRead(unsigned amp, uint32_t reg, uint32_t *value) {
    const uint8_t a[4] = {uint8_t(reg >> 24), uint8_t(reg >> 16), uint8_t(reg >> 8), uint8_t(reg)};
    if (!transfer(kAmpAddress[amp], a, 4, 4)) return false;
    const uint32_t rx = gBus.rx;   // first byte in bits 7:0
    *value = (rx & 0xff) << 24 | ((rx >> 8) & 0xff) << 16 | ((rx >> 16) & 0xff) << 8 | rx >> 24;
    return true;
}
bool ampWrite(unsigned amp, uint32_t reg, uint32_t value) {
    const uint8_t b[8] = {uint8_t(reg >> 24), uint8_t(reg >> 16), uint8_t(reg >> 8), uint8_t(reg),
                          uint8_t(value >> 24), uint8_t(value >> 16), uint8_t(value >> 8), uint8_t(value)};
    return transfer(kAmpAddress[amp], b, 8, 0);
}
bool ampRmw(unsigned amp, uint32_t reg, uint32_t mask, uint32_t value) {
    uint32_t v = 0;
    return ampRead(amp, reg, &v) && ampWrite(amp, reg, (v & ~mask) | (value & mask));
}
bool ampUnlock(unsigned amp) {return ampWrite(amp, 0x40, 0x55) && ampWrite(amp, 0x40, 0xaa);}
bool ampLock(unsigned amp) {return ampWrite(amp, 0x40, 0xcc) && ampWrite(amp, 0x40, 0x33);}
// Polls reg until (value & bit), every 1 ms for at most 100 ms.
bool ampPoll(unsigned amp, uint32_t reg, uint32_t bit) {
    for (unsigned i = 0; i < 100; ++i) {uint32_t v = 0; if (ampRead(amp, reg, &v) && (v & bit)) return true; IOSleep(1);}
    return false;
}

// ---- CS35L41 set-up for ASP RX1 straight to the DAC, the HALO DSP halted and never loaded (/tmp/kmdconn/cs35l41-init-spec.md:
// Linux cs35l41-lib.c / cs35l41.c / cs35l41_hda.c, Sony's driver for the board values). 16-bit I2S slots at 48 kHz, BCLK 1.536 MHz.
// Per amplifier: [0] DEVID, [1] REVID, [2] OTPID, [3] IRQ1 status 1 at the last look, then the bring-up: [4] the step that failed
// (0: none), [5] IRQ1 status 3 (bit 1 PLL lock), [6] PWR_CTRL2, [7] OTP fields written.
uint32_t gAmpVolume = 0x8000, gAmpGain = 0x80;   // 0 dB digital, 4.5 dB analog (the 0.5 dB default was a touch quiet; <=0x140/10.5 dB is safe without DSP protection; plug-in applies system volume) (pdx203-audio-vol / -gain override)

int ampOtpUnpack(unsigned amp) {
    uint32_t id = 0, otp[24];
    if (!ampRead(amp, 0x10, &id)) return 1;
    if (id != 0x01 && id != 0x08) return 2;   // only map 1 is carried (the pdx203 parts read 0x08)
    for (unsigned i = 0; i < 24; ++i) if (!ampRead(amp, 0x400 + 4 * i, &otp[i])) return 3;
    unsigned pos = CS35L41_OTP_START_BIT, written = 0;
    for (const Cs35l41OtpField &f : kCs35l41OtpMap1) {
        uint32_t value = 0;
        for (unsigned k = 0; k < f.size; ++k, ++pos) value |= ((otp[pos / 32] >> (pos % 32)) & 1u) << k;
        if (!f.reg) continue;
        const uint32_t mask = (f.size == 32 ? 0xffffffffu : ((1u << f.size) - 1)) << f.shift;
        if (!ampRmw(amp, f.reg, mask, value << f.shift)) return 4;
        ++written;
    }
    pdx203_audio_amps[amp][7] = written;
    return 0;
}

// Once after the shared reset: OTP boot, errata, trim, HALO halt, interrupt pin, boost, the ASP format and routing, muted.
int ampInit(unsigned amp) {
    uint32_t v = 0;
    if (!ampPoll(amp, 0x1001c, 0x2)) return 10;                       // OTP_BOOT_DONE
    if (!ampRead(amp, 0x10018, &v) || (v & 0x80000000u)) return 11;   // OTP_BOOT_ERR
    if (!ampUnlock(amp)) return 12;
    static const uint32_t errata[][2] = {{0x4100, 0}, {0x4310, 0}, {0x4400, 0}, {0x381c, 0x51}, {0x02bc20e0, 0}, {0x02bc2020, 0},
                                         {0x2018, 0}, {0x6c04, 0}, {0x4c28, 0}, {0x4c2c, 0}, {0x02bc1000, 0}};
    for (const auto &w : errata) if (!ampWrite(amp, w[0], w[1])) return 13;
    if (const int rc = ampOtpUnpack(amp)) return 20 + rc;
    if (!ampLock(amp)) return 14;
    static const uint32_t setup[][3] = {   // reg, mask, value (mask 0: plain write)
        {0x11008, 0x80001000, 0x80000000}, {0x1100c, 0x80001000, 0}, {0x242c, 0x07000000, 0x02000000}, {0x10110, 0, 0x7ffcfe3f},
        {0x3810, 0xffff, 0x2424}, {0x3814, 0xff03, 0x7500}, {0x3808, 0x7f, 0x40}, {0x2018, 0x30, 0x20}, {0x480c, 0x3, 0x3},
        {0x2c04, 0, 0x370}, {0x2c0c, 0x1f, 0x3}, {0x2c08, 0, 0x3}, {0x4804, 0, 0x1b}, {0x4808, 0, 0x10100200}, {0x4840, 0x3f, 0x10},
        {0x4820, 0x3f3f, 0}, {0x4c00, 0x7f, 0x8}, {0x4800, 0, 0x10000}, {0x6000, 0, 0xa678}, {0x6c04, 0, 0}, {0x201c, 0x10, 0x10},
    };
    for (const auto &w : setup) {
        uint32_t value = w[2];
        if (w[0] == 0x4820) value = amp ? 0x0001 : 0x0100;   // the left amplifier plays slot 0, the right one slot 1 (Sony's)
        if (!(w[1] ? ampRmw(amp, w[0], w[1], value) : ampWrite(amp, w[0], value))) return 15;
    }
    // TST_FS_MON0 for BCLK 1.536 MHz: FS1 = 52, FS2 = 84 (Sony's dai_set_sysclk formula), under the test key
    if (!ampUnlock(amp) || !ampWrite(amp, 0x2d10, 0x00054034) || !ampLock(amp)) return 16;
    return 0;
}

// With BCLK and LRCLK running: the power-up patch, global enable, PUP_DONE, amplifier on, unmute; then the status words.
int ampStart(unsigned amp) {
    if (!ampUnlock(amp) || !ampWrite(amp, 0x2084, 0x002f1aa0) || !ampLock(amp)) return 30;
    if (!ampRmw(amp, 0x2014, 0x1, 0x1)) return 31;
    if (!ampPoll(amp, 0x10010, 0x01000000)) return 32;
    if (!ampWrite(amp, 0x10010, 0x01000000) || !ampRmw(amp, 0x2018, 0x1, 0x1)) return 33;
    if (!ampWrite(amp, 0x6000, gAmpVolume) || !ampWrite(amp, 0x6c04, gAmpGain)) return 34;
    return 0;
}
void ampStatus(unsigned amp) {
    uint32_t s1 = 0, s3 = 0, p2 = 0;
    ampRead(amp, 0x10010, &s1); ampRead(amp, 0x10018, &s3); ampRead(amp, 0x2018, &p2);
    pdx203_audio_amps[amp][3] = s1; pdx203_audio_amps[amp][5] = s3; pdx203_audio_amps[amp][6] = p2;
}
int ampStop(unsigned amp) {
    if (!ampWrite(amp, 0x6c04, 0) || !ampWrite(amp, 0x6000, 0xa678)) return 40;
    if (!ampRmw(amp, 0x2018, 0x1, 0) || !ampRmw(amp, 0x2014, 0x1, 0)) return 41;
    const bool down = ampPoll(amp, 0x10010, 0x00800000);
    ampWrite(amp, 0x10010, 0x00800000);
    if (!ampUnlock(amp) || !ampWrite(amp, 0x2084, 0x002f1aa3) || !ampLock(amp)) return 42;
    return down ? 0 : 43;
}

void probeAmps() {
    if (!mapRegion(kGcc, 0x60000, gGcc) || !mapRegion(kSe2, 0x4000, gSe)) {fail(1, -12); return;}
    const unsigned pins[4] = {kPinSda, kPinScl, kPinReset, kPinIrq};
    for (unsigned i = 0; i < 4; ++i) if (!mapRegion(kTlmmNorth + pins[i] * 0x1000ull, 0x1000, gPins[i])) {fail(1, -12); return;}
    pdx203_audio_stats[0] = 1;
    if (!busClocks()) {fail(2, -5); return;}
    pdx203_audio_stats[0] = 2;
    pinConfig(gPins[0], 1, 3, 0, false, false);   // SDA: qup2, pull-up, 2 mA
    pinConfig(gPins[1], 1, 3, 0, false, false);   // SCL
    pinConfig(gPins[3], 0, 0, 0, false, false);   // interrupt: input, no pull (the amplifiers drive it)
    pinConfig(gPins[2], 0, 0, 0, true, false);    // reset low
    IOSleep(2);
    wr(gPins[2], 4, 2u);                          // release
    IOSleep(5);                                   // the amplifiers load their OTP after reset
    pdx203_audio_stats[0] = 3;
    if (!busSetup()) {fail(4, -19); return;}
    pdx203_audio_stats[0] = 4;
    int32_t found = 0;
    for (unsigned amp = 0; amp < 2; ++amp) {
        const uint32_t regs[4] = {kDevid, kRevid, kOtpid, kIrq1Status1};
        bool ok = true;
        for (unsigned r = 0; r < 4 && ok; ++r) {uint32_t v = 0; ok = ampRead(amp, regs[r], &v); if (ok) pdx203_audio_amps[amp][r] = v;}
        if (ok) found |= 1 << amp;
    }
    pdx203_audio_stats[2] = found;
    pdx203_audio_stats[0] = found ? 5 : -5;
}

// ---- APR (the audio DSP's packet router, as downstream audio-kernel apr.h and q6core.c define it) ----
struct AprHeader {
    uint16_t hdr_field, pkt_size;
    uint8_t src_svc, src_domain; uint16_t src_port;
    uint8_t dest_svc, dest_domain; uint16_t dest_port;
    uint32_t token, opcode;
};
static_assert(sizeof(AprHeader) == 20, "APR header");
constexpr uint16_t kAprSeqCmd = (2u << 8) | ((sizeof(AprHeader) / 4) << 4);   // APR_HDR_FIELD(SEQ_CMD, APR_HDR_LEN(20), version 0)
constexpr uint8_t kDomainAdsp = 4, kDomainApps = 5;
constexpr uint8_t kSvcAfe = 4, kSvcAsm = 7, kSvcAdm = 8;
// Opcodes (/tmp/kmdconn/q6-playback-spec.md: downstream apr_audio-v2.h, q6afe.c, q6asm.c, q6adm.c on kona)
constexpr uint32_t kBasicResult = 0x000110e8;
constexpr uint32_t kAfeVote = 0x000100f4, kAfeVoteRsp = 0x000100f5, kAfeDevote = 0x000100f6, kAfeSvcSetParam = 0x000100f3,
                   kAfePortSetParam = 0x000100ef, kAfeStart = 0x000100e5, kAfeStop = 0x000100e6;
constexpr uint32_t kAsmMap = 0x00010d92, kAsmMapRsp = 0x00010d93, kAsmUnmap = 0x00010d94, kAsmOpenWrite = 0x00010db3,
                   kAsmMediaFmt = 0x00010d98, kAsmRun = 0x00010daa, kAsmWrite = 0x00010dab, kAsmWriteDone = 0x00010d99,
                   kAsmPause = 0x00010bd3, kAsmClose = 0x00010bcd;
constexpr uint32_t kAdmOpen = 0x00010326, kAdmOpenRsp = 0x00010329, kAdmMap = 0x00010325, kAdmClose = 0x00010327;
constexpr uint16_t kSenMi2sRx = 0x1018, kAsmPort = 0x0101;   // session 1, stream 1
constexpr uint16_t kRxCdcDma0 = 0xb030;                      // RX_CODEC_DMA_RX_0: the RX macro, the WCD938x's headphone amplifiers
constexpr uint32_t kAsmSession = 1;

ShengAdspChannel *gApr;
volatile uint32_t gAprUp;
// Messages the DSP thread hands over, already split at the header's own length (APRv2.1 replies carry a 32-byte header: trace word and
// timestamp after the 20 bytes of v2), copied out by the audio thread.
struct AprMessage {uint32_t opcode, token, words; uint16_t src_port, dest_port; uint8_t src_svc, version; uint32_t payload[24];};
AprMessage gRing[16];
volatile uint32_t gRingHead, gRingTail;
volatile uint32_t gWake;

void wakeAudio() {__atomic_store_n(&gWake, 1, __ATOMIC_RELEASE); thread_wakeup((event_t)&gWake);}
void aprState(void *, int up) {
    __atomic_store_n(&gAprUp, up ? 1u : 0u, __ATOMIC_RELEASE);
    pdx203_audio_stats[5] = up;
    wakeAudio();
}
void aprReceive(void *, const uint8_t *data, uint32_t length) {
    pdx203_audio_stats[6] = pdx203_audio_stats[6] + 1;
    if (length < sizeof(AprHeader)) return;
    AprHeader h;
    memcpy(&h, data, sizeof h);
    const uint32_t headerBytes = ((h.hdr_field >> 4) & 0xfu) * 4u;
    if (headerBytes < sizeof(AprHeader) || headerBytes > length) return;
    const uint32_t head = gRingHead;
    if (head - __atomic_load_n(&gRingTail, __ATOMIC_ACQUIRE) >= 16) {pdx203_audio_stats[9] = pdx203_audio_stats[9] + 1; return;}
    AprMessage &m = gRing[head % 16];
    m.opcode = h.opcode; m.token = h.token; m.src_port = h.src_port; m.dest_port = h.dest_port; m.src_svc = h.src_svc;
    m.version = (uint8_t)(h.hdr_field & 0xf);
    m.words = (length - headerBytes) / 4 > 24 ? 24 : (length - headerBytes) / 4;
    memcpy(m.payload, data + headerBytes, m.words * 4);
    __atomic_store_n(&gRingHead, head + 1, __ATOMIC_RELEASE);
    wakeAudio();
}
void sleepMs(uint32_t ms) {
    if (__atomic_exchange_n(&gWake, 0, __ATOMIC_ACQ_REL)) return;
    assert_wait_timeout((event_t)&gWake, THREAD_UNINT, ms, kMillisecondScale);
    if (__atomic_exchange_n(&gWake, 0, __ATOMIC_ACQ_REL)) {clear_wait(current_thread(), THREAD_AWAKENED); return;}
    thread_block(THREAD_CONTINUE_NULL);
    __atomic_store_n(&gWake, 0, __ATOMIC_RELEASE);
}
bool popMessage(AprMessage &m) {
    const uint32_t tail = __atomic_load_n(&gRingTail, __ATOMIC_ACQUIRE);
    if (tail == __atomic_load_n(&gRingHead, __ATOMIC_ACQUIRE)) return false;
    m = gRing[tail % 16];
    __atomic_store_n(&gRingTail, tail + 1, __ATOMIC_RELEASE);
    pdx203_audio_stats[10] = (int32_t)m.opcode;
    return true;
}
void logMessage(const char *why, const AprMessage &m) {
    char text[140];
    unsigned at = 0;
    for (uint32_t i = 0; i < m.words && i < 8 && at + 10 < sizeof text; ++i) at += snprintf(text + at, sizeof text - at, " %x", m.payload[i]);
    text[at] = 0;
    printf("PDX203 APR %s op %x svc %u port %x->%x token %x v%u:%s\n", why, m.opcode, m.src_svc, m.src_port, m.dest_port, m.token, m.version, text);
}

// ---- the playback path (pdx203-audio bit 4) ----
// [0] step (positive progress, negative the step that failed), [1] the DSP status of the last command, [2] MACRO vote handle, [3] DCODEC
// vote handle, [4] memory map handle, [5] COPP id, [6] writes sent, [7] writes returned, [8] writes returned with an error, [9] SMMU
// (slot | bank << 8, or -1 refused), [10] left amplifier start result, [11] right, [12] messages we did not expect, [13] beeps played
volatile int32_t gPlay[16];
const char *gSmmuError = "";

// Packets: little-endian fields at byte offsets after the 20-byte command header.
struct Packet {
    uint8_t b[128] = {};
    uint32_t size = 0;
    Packet(uint8_t svc, uint16_t srcPort, uint16_t dstPort, uint32_t token, uint32_t opcode, uint32_t bytes) {
        size = bytes;
        AprHeader h = {kAprSeqCmd, (uint16_t)bytes, svc, kDomainApps, srcPort, svc, kDomainAdsp, dstPort, token, opcode};
        memcpy(b, &h, sizeof h);
    }
    void u16(uint32_t at, uint16_t v) {memcpy(b + at, &v, 2);}
    void u32(uint32_t at, uint32_t v) {memcpy(b + at, &v, 4);}
    void u8(uint32_t at, uint8_t v) {b[at] = v;}
    uint32_t opcode() const {uint32_t v; memcpy(&v, b + 16, 4); return v;}
};

// Sends the packet and waits up to ms for its answer: the given reply opcode, or the basic result naming this command. Anything else
// that arrives meanwhile is handled as a playback event (write done) or logged. Returns the DSP status (0 ok), -60 on timeout, -5 if the
// send was refused; reply gets the answer's payload.
void dispatch(const AprMessage &m);
int aprCall(const Packet &p, uint32_t reply, AprMessage *answer = nullptr, uint32_t ms = 2000) {
    if (!sheng_adsp_send || sheng_adsp_send(gApr, p.b, p.size)) return -5;
    pdx203_audio_stats[7] = pdx203_audio_stats[7] + 1;
    const uint32_t op = p.opcode();
    uint64_t deadline;
    clock_interval_to_deadline(ms, kMillisecondScale, &deadline);
    for (;;) {
        AprMessage m;
        while (popMessage(m)) {
            if (m.opcode == kBasicResult && m.words >= 2 && m.payload[0] == op) {
                gPlay[1] = (int32_t)m.payload[1];
                if (m.payload[1]) logMessage("error", m);
                if (answer) *answer = m;
                return (int)m.payload[1];
            }
            if (reply && m.opcode == reply) {
                if (answer) *answer = m;
                gPlay[1] = 0;
                return 0;
            }
            dispatch(m);
        }
        if (mach_absolute_time() >= deadline) {printf("PDX203 APR timeout op %x\n", op); return -60;}
        sleepMs(20);
    }
}

// The apps SMMU stream the DSP's audio buffers arrive with (kona-audio.dtsi msm-audio-ion: iommus <&apps_smmu 0x1801 0>), mapped
// pass-through into the bypass bank NativeUfs already uses (SmmuBypass.h, sheng's). The buffer: 64 KiB of PIL CVP (0x87300000), DRAM
// below 4 GiB the firmware reserves for a coprocessor this board never starts; the DSP gets its physical address with the stream's
// index (1) in the upper word, as msm_audio_ion and Linux q6asm-dai hand it out.
constexpr uint64_t kSmmu = 0x15000000, kSmmuSize = 0x100000, kBufPhys = 0x87300000, kBufSize = 0x10000;
constexpr uint16_t kAudioStream = 0x1801;
constexpr uint32_t kBufMsw = 1;
constexpr uint32_t kPeriodFrames = 480, kPeriodBytes = kPeriodFrames * 4, kSlotBytes = 2048, kSlots = 8;
Region gSmmuMap, gBuf, gLpi;
uint32_t smmuRd(void *ctx, unsigned at) {return *reinterpret_cast<volatile uint32_t *>(reinterpret_cast<uintptr_t>(ctx) + at);}
void smmuWr(void *ctx, unsigned at, uint32_t value) {
    *reinterpret_cast<volatile uint32_t *>(reinterpret_cast<uintptr_t>(ctx) + at) = value;
    __asm__ volatile("dsb sy" ::: "memory");
}
bool smmuPassThrough() {
    if (!mapRegion(kSmmu, kSmmuSize, gSmmuMap)) {gSmmuError = "map"; return false;}
    const smmu_io io = {reinterpret_cast<void *>(gSmmuMap.va), smmuRd, smmuWr};
    const smmu_plan plan = smmu_plan_bypass(&io, kAudioStream, kBufPhys, kBufSize);
    const char *error = smmu_configure_bypass(&io, &plan, kAudioStream);
    if (error) {gSmmuError = error; gPlay[9] = -1; return false;}
    gPlay[9] = (int32_t)(plan.slot | plan.bank << 8 | (plan.reuse ? 1u << 16 : 0) | (plan.bank_ready ? 1u << 17 : 0));
    return true;
}

// The LPASS LPI pins of the senary MI2S (Sony kona-edo-common.dtsi lpi_i2s2: gpio10 SCK func1, gpio11 WS func1, gpio12 SD0 func2,
// gpio13 SD1 func2; 2 mA, no pull); Linux pinctrl-lpass-lpi.h: pin n at 0x033c0000 + n * 0x1000, function in bits 5:2, drive 8:6.
// Only touched while the MACRO vote is held (the LPI TLMM refuses access without it).
bool lpiPins() {
    if (!mapRegion(0x033c0000 + 10 * 0x1000, 4 * 0x1000, gLpi)) return false;
    const unsigned funcs[4] = {1, 1, 2, 2};
    for (unsigned i = 0; i < 4; ++i) wr(gLpi, i * 0x1000, (rd(gLpi, i * 0x1000) & ~0x1ffu) | funcs[i] << 2);
    return true;
}

int afeVote(uint32_t block, int32_t *handle) {
    Packet p(kSvcAfe, 0, 0, block, kAfeVote, 0x20);
    p.u32(0x14, block);
    memcpy(p.b + 0x18, "LPASS_H", 8);
    AprMessage m;
    const int rc = aprCall(p, kAfeVoteRsp, &m);
    if (!rc && m.opcode == kAfeVoteRsp && m.words >= 1) *handle = (int32_t)m.payload[0];
    return rc;
}
[[maybe_unused]] int afeDevote(uint32_t block, int32_t handle) {   // the votes and the map stay for the boot
    Packet p(kSvcAfe, 0, 0, 0, kAfeDevote, 0x1c);
    p.u32(0x14, block); p.u32(0x18, (uint32_t)handle);
    return aprCall(p, 0);
}
int afeClockSet(uint32_t id, uint32_t hz, bool enable) {   // AFE_SVC_CMD_SET_PARAM, module CLOCK_SET, attri COUPLE_NO, root 0
    Packet p(kSvcAfe, 0, 0, 1024, kAfeSvcSetParam, 0x44);
    p.u32(0x14, 32); p.u32(0x24, 0x0001028f); p.u32(0x28, 0x00010290); p.u16(0x2c, 20);
    p.u32(0x30, 1); p.u32(0x34, id); p.u32(0x38, hz); p.u16(0x3c, 1); p.u16(0x3e, 0); p.u32(0x40, enable ? 1 : 0);
    return aprCall(p, 0);
}
int afeClock(bool enable) {return afeClockSet(0x10d, 1536000, enable);}   // SEN_MI2S_IBIT at 1.536 MHz
// RX_CODEC_DMA_RX_0: AFE_PARAM_ID_CODEC_DMA_CONFIG {minor 1, 48 kHz, 16 bit, linear PCM, 2 channels, channels 0 and 1}
int afeCdcConfig() {
    Packet p(kSvcAfe, 0, 0, 0, kAfePortSetParam, 0x40);
    p.u16(0x14, kRxCdcDma0); p.u16(0x16, 28); p.u32(0x24, 0x0001020c); p.u32(0x28, 0x000102b8); p.u16(0x2c, 16);
    p.u32(0x30, 1); p.u32(0x34, 48000); p.u16(0x38, 16); p.u16(0x3a, 0); p.u16(0x3c, 2); p.u16(0x3e, 3);
    return aprCall(p, 0);
}
int afePortConfig() {   // AFE_PORT_CMD_SET_PARAM_V2, AFE_PARAM_ID_I2S_CONFIG: 16 bit, SD0, stereo, internal WS, 48 kHz, linear PCM
    Packet p(kSvcAfe, 0, 0, 0, kAfePortSetParam, 0x44);
    p.u16(0x14, kSenMi2sRx); p.u16(0x16, 32); p.u32(0x24, 0x0001020c); p.u32(0x28, 0x0001020d); p.u16(0x2c, 20);
    p.u32(0x30, 1); p.u16(0x34, 16); p.u16(0x36, 1); p.u16(0x38, 1); p.u16(0x3a, 1); p.u32(0x3c, 48000); p.u16(0x40, 0);
    return aprCall(p, 0);
}
int afePort(uint32_t opcode, uint16_t dev = kSenMi2sRx) {
    Packet p(kSvcAfe, 0, 0, 0, opcode, 0x18);
    p.u16(0x14, dev);
    return aprCall(p, 0);
}
int asmMap(int32_t *handle) {
    Packet p(kSvcAsm, 0, 0, kAsmSession << 8, kAsmMap, 0x28);
    p.u16(0x14, 3); p.u16(0x16, 1); p.u32(0x18, 0); p.u32(0x1c, (uint32_t)kBufPhys); p.u32(0x20, kBufMsw); p.u32(0x24, kBufSize);
    AprMessage m;
    const int rc = aprCall(p, kAsmMapRsp, &m);
    if (!rc && m.opcode == kAsmMapRsp && m.words >= 1) *handle = (int32_t)m.payload[0];
    return rc;
}
int asmOpen() {   // legacy stream to the device matrix, 16 bit, the DSP's default post-processing topology, multichannel PCM v2
    Packet p(kSvcAsm, kAsmPort, kAsmPort, 0, kAsmOpenWrite, 0x24);
    p.u32(0x14, 0); p.u16(0x18, 0); p.u16(0x1a, 16); p.u32(0x1c, 0x00010be4); p.u32(0x20, 0x00010da5);
    return aprCall(p, 0);
}
int admOpen(int32_t *copp, uint16_t dev) {   // COPP on the device port with the null topology, 2 channels, 16 bit, 48 kHz, FL/FR
    Packet p(kSvcAdm, dev, dev, 0, kAdmOpen, 0x30);
    p.u16(0x14, 0); p.u16(0x16, 1); p.u16(0x18, dev); p.u16(0x1a, 0xffff); p.u32(0x1c, 0x00010312);
    p.u16(0x20, 2); p.u16(0x22, 16); p.u32(0x24, 48000); p.u8(0x28, 1); p.u8(0x29, 2);
    AprMessage m;
    const int rc = aprCall(p, kAdmOpenRsp, &m);
    if (rc) return rc;
    if (m.opcode == kAdmOpenRsp && m.words >= 2) {
        if (m.payload[0]) {gPlay[1] = (int32_t)m.payload[0]; logMessage("error", m); return (int)m.payload[0];}
        *copp = (int32_t)(m.payload[1] & 0xffff);
    }
    return 0;
}
int admRoute(int32_t copp) {
    Packet p(kSvcAdm, 0, 0, 0, kAdmMap, 0x24);
    p.u32(0x14, 0); p.u32(0x18, 1); p.u16(0x1c, kAsmSession); p.u16(0x1e, 1); p.u16(0x20, (uint16_t)copp);
    return aprCall(p, 0);
}
int admClose(int32_t copp, uint16_t dev) {
    Packet p(kSvcAdm, dev, (uint16_t)copp, 0, kAdmClose, 0x14);
    return aprCall(p, 0);
}
int asmFormat() {
    Packet p(kSvcAsm, kAsmPort, kAsmPort, 0, kAsmMediaFmt, 0x2c);
    p.u32(0x14, 20); p.u16(0x18, 2); p.u16(0x1a, 16); p.u32(0x1c, 48000); p.u16(0x20, 1); p.u8(0x24, 1); p.u8(0x25, 2);
    return aprCall(p, 0);
}
int asmSimple(uint32_t opcode, uint32_t bytes) {
    Packet p(kSvcAsm, kAsmPort, kAsmPort, 0, opcode, bytes);
    return aprCall(p, 0);
}
[[maybe_unused]] int asmUnmap(int32_t handle) {
    Packet p(kSvcAsm, 0, 0, 0, kAsmUnmap, 0x18);
    p.u32(0x14, (uint32_t)handle);
    return aprCall(p, 0);
}
bool asmWrite(uint32_t slot) {   // no answer on success; the buffer comes back as a write done event
    Packet p(kSvcAsm, kAsmPort, kAsmPort, slot, kAsmWrite, 0x34);
    p.u32(0x14, (uint32_t)kBufPhys + slot * kSlotBytes); p.u32(0x18, kBufMsw); p.u32(0x1c, (uint32_t)gPlay[4]);
    p.u32(0x20, kPeriodBytes); p.u32(0x24, slot);
    if (!sheng_adsp_send || sheng_adsp_send(gApr, p.b, p.size)) return false;
    gPlay[6] = gPlay[6] + 1;
    return true;
}

// 750 Hz is exactly 64 samples at 48 kHz; a period (480 frames) holds 7.5 cycles, so the phase carries over from period to period
// (restarting it each period made a 100 Hz croak, usb-74). -20 dBFS.
const int16_t kSine64[64] = {0, 321, 639, 951, 1254, 1545, 1821, 2079, 2317, 2533, 2725, 2890, 3028, 3136, 3214, 3261, 3277, 3261, 3214,
    3136, 3028, 2890, 2725, 2533, 2317, 2079, 1821, 1545, 1254, 951, 639, 321, 0, -321, -639, -951, -1254, -1545, -1821, -2079, -2317,
    -2533, -2725, -2890, -3028, -3136, -3214, -3261, -3277, -3261, -3214, -3136, -3028, -2890, -2725, -2533, -2317, -2079, -1821, -1545,
    -1254, -951, -639, -321};

void printPlay(const char *what);

// ---- the path and the audio service ----
// The service iOS's audio server plug-in opens is sheng's (ShengAudioService.cpp, ShengAudioAbi.h, hal/ShengAudioPlugIn.c, unchanged):
// a control page and a playback ring of 16-bit stereo frames shared with the plug-in on one timeline anchored at boot. This thread takes
// period n (timeline frames n * 480 on) from the ring into DSP buffer n % 8 and keeps four queued, refilled on each write done; the
// frames it takes are cleared, so a period the audio server did not write in time plays as silence. The path comes up when the plug-in
// starts output (AFE clock, I2S config and start, amplifiers, ASM open, ADM open and route, media format, run) and goes down 5 s after it
// stops (the amplifiers' power-up would cut the start of the next sound). Capture is not wired yet: input requests are accepted, the
// capture ring stays silent.
constexpr uint32_t kRingFrames = SHENG_AUDIO_RING_FRAMES, kInFlight = 4, kLeadFrames = 30 * kPeriodFrames, kKeepMs = 5000;
constexpr int64_t kDriftLimit = 3 * kPeriodFrames;
struct Path {
    int32_t macro = -1, dcodec = -1, map = -1, copp = -1;
    bool clock, port, ampsOn, opened, running, tone;
    bool hphReady, hphOn, hphForce;   // the headphone codec is up; this run plays on it; pdx203-hph-force=1 treats the jack as in
    int jack = -1;                    // -1 unknown, 0 out, 1 in
    uint16_t dev = kSenMi2sRx;        // the AFE port this run plays on
    uint64_t next, done, first, stopAsked;
    uint32_t inFlight;
    IOBufferMemoryDescriptor *control, *outRing, *inRing;
    ShengAudioControl *ctl;
    int16_t *out;
    volatile uint32_t want, serving;
    int64_t baseLead;
};
Path gPath;
// [audio] starts, stops, failed starts, periods sent late (the plug-in had not written them), resyncs, least and most drift
volatile int32_t gServe[8] = {0, 0, 0, 0, 0, 0x7fffffff, -0x7fffffff, 0};
uint64_t nowMs() {uint64_t ns = 0; absolutetime_to_nanoseconds(mach_absolute_time(), &ns); return ns / 1000000;}

bool setupShared() {
    const IOOptionBits shared = static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryKernelUserShared);
    gPath.control = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, shared, 4096, 4096);
    gPath.outRing = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, shared, kRingFrames * SHENG_AUDIO_OUT_CHANNELS * 2, 4096);
    gPath.inRing = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, shared, kRingFrames * SHENG_AUDIO_IN_CHANNELS * 2, 4096);
    if (!gPath.control || !gPath.outRing || !gPath.inRing) return false;
    gPath.ctl = static_cast<ShengAudioControl *>(gPath.control->getBytesNoCopy());
    gPath.out = static_cast<int16_t *>(gPath.outRing->getBytesNoCopy());
    memset(gPath.ctl, 0, 4096);
    memset(gPath.out, 0, kRingFrames * SHENG_AUDIO_OUT_CHANNELS * 2);
    memset(gPath.inRing->getBytesNoCopy(), 0, kRingFrames * SHENG_AUDIO_IN_CHANNELS * 2);
    mach_timebase_info_data_t tb;
    clock_timebase_info(&tb);
    ShengAudioControl *c = gPath.ctl;
    c->rate = SHENG_AUDIO_RATE; c->out_channels = SHENG_AUDIO_OUT_CHANNELS; c->in_channels = SHENG_AUDIO_IN_CHANNELS;
    c->period_frames = kPeriodFrames; c->ring_frames = kRingFrames; c->lead_frames = kLeadFrames;
    // host ticks per frame in 32.32, through 128 bits (the 19.2 MHz timebase overflows 64; sheng i535)
    c->ticks_per_frame_q32 = (uint64_t)(((__uint128_t)1000000000ULL * tb.denom << 32) / ((uint64_t)SHENG_AUDIO_RATE * tb.numer));
    c->out_anchor_host = mach_absolute_time();
    c->in_anchor_host = c->out_anchor_host;
    c->version = SHENG_AUDIO_VERSION;
    __atomic_store_n(&c->magic, SHENG_AUDIO_MAGIC, __ATOMIC_RELEASE);
    return true;
}
uint64_t framesNow() {
    const uint64_t now = mach_absolute_time(), anchor = gPath.ctl->out_anchor_host;
    return now > anchor ? (uint64_t)(((__uint128_t)(now - anchor) << 32) / gPath.ctl->ticks_per_frame_q32) : 0;
}
// sheng's: the steady distance between the next period and the play position settles once the DSP's queue has filled; when the DSP's
// clock and the host's drift three periods from it, the next period is set back to it (one click). The published lead is that plus two
// periods, so the plug-in's latency and safety offset follow what the DSP really takes.
void syncPlayback() {
    const uint64_t now = framesNow(), played = gPath.done - gPath.first;
    const int64_t lead = (int64_t)(gPath.next * kPeriodFrames) - (int64_t)now;
    if (played < 2 * kInFlight) return;
    if (played == 2 * kInFlight) {
        gPath.baseLead = lead;
        const int64_t want = lead + 2 * (int64_t)kPeriodFrames;
        gPath.ctl->lead_frames = (uint32_t)(want < (int64_t)kLeadFrames ? kLeadFrames : want > 24000 ? 24000 : want);
        return;
    }
    const int64_t drift = lead - gPath.baseLead;
    if (drift < gServe[5]) gServe[5] = (int32_t)drift;
    if (drift > gServe[6]) gServe[6] = (int32_t)drift;
    if (drift > kDriftLimit || drift < -kDriftLimit) {
        gPath.next = (uint64_t)((int64_t)now + gPath.baseLead + kPeriodFrames / 2) / kPeriodFrames;
        gServe[4] = gServe[4] + 1;
    }
}

// Period n into DSP buffer n % 8: the test beeps (1 s on, 2 s off; left, right, both) or the plug-in's ring.
bool submit(uint64_t n) {
    const uint32_t slot = (uint32_t)(n % kSlots);
    int16_t *dst = reinterpret_cast<int16_t *>(gBuf.va + slot * kSlotBytes);
    if (gPath.tone) {
        const uint64_t k = n - gPath.first;
        const unsigned channels = (k % 300) < 100 ? 1u + (unsigned)((k / 300) % 3) : 0u;
        for (uint32_t i = 0; i < kPeriodFrames; ++i) {
            const int16_t s = kSine64[(k * kPeriodFrames + i) % 64];
            dst[2 * i] = (channels & 1) ? s : 0;
            dst[2 * i + 1] = (channels & 2) ? s : 0;
        }
        if (k % 300 == 0) gPlay[13] = gPlay[13] + 1;
    } else {
        const uint32_t at = (uint32_t)((n * kPeriodFrames) % kRingFrames);
        const uint32_t head = kRingFrames - at < kPeriodFrames ? kRingFrames - at : kPeriodFrames;
        memcpy(dst, gPath.out + at * 2, head * 4);
        if (head < kPeriodFrames) memcpy(dst + head * 2, gPath.out, (kPeriodFrames - head) * 4);
        memset(gPath.out + at * 2, 0, head * 4);
        if (head < kPeriodFrames) memset(gPath.out, 0, (kPeriodFrames - head) * 4);
        uint64_t recent = 0;
        nanoseconds_to_absolutetime(100000000ULL, &recent);
        if ((gPath.want & ShengAudioOutputRunning) && mach_absolute_time() - gPath.ctl->out_written_host < recent &&
            gPath.ctl->out_written < (n + 1) * kPeriodFrames) {
            gPath.ctl->out_late = gPath.ctl->out_late + 1;
            gServe[3] = gServe[3] + 1;
        }
    }
    __asm__ volatile("dsb sy" ::: "memory");
    if (!asmWrite(slot)) return false;
    gPath.inFlight++;
    return true;
}
void onWriteDone(const AprMessage &m) {
    gPlay[7] = gPlay[7] + 1;
    if (m.words >= 4 && m.payload[3]) gPlay[8] = gPlay[8] + 1;
    if (gPath.inFlight) gPath.inFlight--;
    gPath.done++;
    if (gPath.ctl) gPath.ctl->out_frames = (gPath.done - gPath.first) * kPeriodFrames;
    if (!gPath.running) return;
    if (!gPath.tone) syncPlayback();
    while (gPath.inFlight < kInFlight && submit(gPath.next)) gPath.next++;
}

// Takes down whatever of the stream is up (also the remains of a failed start).
void stopOutput() {
    const bool was = gPath.running;
    gPath.running = false;
    if (gPath.opened) asmSimple(kAsmPause, 0x14);
    if (gPath.ampsOn) for (unsigned amp = 0; amp < 2; ++amp) ampStop(amp);
    gPath.ampsOn = false;
    if (gPath.hphOn) pdx203_hph_off();   // the codec down while the DMA still runs (the PDM stream must outlive the amplifiers)
    gPath.hphOn = false;
    if (gPath.opened) asmSimple(kAsmClose, 0x14);
    gPath.opened = false;
    if (gPath.copp >= 0) admClose(gPath.copp, gPath.dev);
    gPath.copp = -1;
    if (gPath.port) afePort(kAfeStop, gPath.dev);
    gPath.port = false;
    if (gPath.clock) afeClock(false);
    gPath.clock = false;
    if (gPath.ctl) __atomic_and_fetch(&gPath.ctl->state, ~ShengAudioOutputRunning, __ATOMIC_ACQ_REL);
    if (was) gServe[1] = gServe[1] + 1;
}
bool startOutput() {
    if (gPath.running) return true;
    int rc = 0;
    auto failed = [&](int n) {gPlay[0] = -n; gPlay[1] = rc; gServe[2] = gServe[2] + 1; stopOutput(); printPlay("failed"); return false;};
    const bool hph = gPath.hphReady && (gPath.jack == 1 || gPath.hphForce);
    gPath.dev = hph ? kRxCdcDma0 : kSenMi2sRx;
    if (hph) {
        // the codec DMA port first (it streams zeros), then the codec path on top of it (lpass spec 1.5: BE prepare, then DAPM)
        if ((rc = afeCdcConfig())) return failed(5);
        if ((rc = afePort(kAfeStart, kRxCdcDma0))) return failed(5);
        gPath.port = true;
        gPath.hphOn = true;
        if ((rc = pdx203_hph_on())) {pdx203_hph_print("power-up failed"); return failed(6);}
    } else {
        if ((rc = afeClock(true))) return failed(4);
        gPath.clock = true;
        if ((rc = afePortConfig())) return failed(5);
        if ((rc = afePort(kAfeStart))) return failed(5);
        gPath.port = true;
        IOSleep(5);   // the bit clock runs: the amplifiers' PLL locks to it
        gPath.ampsOn = true;
        for (unsigned amp = 0; amp < 2; ++amp) gPlay[10 + amp] = ampStart(amp);
    }
    if ((rc = asmOpen())) return failed(8);
    gPath.opened = true;
    int32_t copp = -1;
    if ((rc = admOpen(&copp, gPath.dev))) return failed(9);
    gPath.copp = copp;
    gPlay[5] = copp;
    if ((rc = admRoute(copp))) return failed(10);
    if ((rc = asmFormat())) return failed(11);
    if ((rc = asmSimple(kAsmRun, 0x20))) return failed(12);
    // The DSP plays the first period at once, so it is the period of the timeline that plays now (sheng i533).
    if (!gPath.tone) memset(gPath.out, 0, kRingFrames * 4);   // the plug-in adds each IO context's mix: no frames of the last run
    gPath.next = gPath.done = gPath.first = (framesNow() + kPeriodFrames - 1) / kPeriodFrames;
    gPath.ctl->out_frames = 0;
    gPath.inFlight = 0;
    gPath.running = true;
    while (gPath.inFlight < kInFlight && submit(gPath.next)) gPath.next++;
    __atomic_or_fetch(&gPath.ctl->state, ShengAudioOutputRunning, __ATOMIC_ACQ_REL);
    gServe[0] = gServe[0] + 1;
    gPlay[0] = 12;
    printf("PDX203 PLAY route %s\n", hph ? "headphones" : "speakers");
    return true;
}

// Once at boot: amplifiers configured (muted, off), SMMU pass-through, the DSP buffer, the LPASS votes, the LPI pins, the ASM memory map.
bool bootPath() {
    int rc = 0;
    auto failed = [&](int n) {gPlay[0] = -n; gPlay[1] = rc; printPlay("failed"); return false;};
    for (unsigned amp = 0; amp < 2; ++amp) {
        if (!(pdx203_audio_stats[2] & (1 << amp))) {rc = -19; return failed(1);}
        if ((rc = ampInit(amp))) {pdx203_audio_amps[amp][4] = (uint32_t)rc; return failed(1);}
    }
    gPlay[0] = 1;
    if (!smmuPassThrough()) {rc = -1; return failed(2);}
    if (!mapRegion(kBufPhys, kBufSize, gBuf, kIOMapWriteCombineCache)) {rc = -12; return failed(2);}
    bzero(reinterpret_cast<void *>(gBuf.va), kBufSize);
    gPlay[0] = 2;
    if ((rc = afeVote(3, &gPath.macro))) return failed(3);
    gPlay[2] = gPath.macro;
    if (afeVote(4, &gPath.dcodec) == 0) gPlay[3] = gPath.dcodec;
    if (!lpiPins()) {rc = -12; return failed(3);}
    gPlay[0] = 3;
    if ((rc = asmMap(&gPath.map))) return failed(7);
    gPlay[4] = gPath.map;
    gPlay[0] = 7;
    return true;
}

// pdx203-audio bit 8: the headphone codec, once at boot after the path. The macros run on the TX core clocks (AFE 0x30C, 0x30D at
// 19.2 MHz), which stay on: every codec register, the jack switch included, is read over the TX SoundWire bus they clock.
void bootHeadphones() {
    unsigned force = 0;
    PE_parse_boot_argn("pdx203-hph-force", &force, sizeof force);
    gPath.hphForce = force == 1;
    int rc = afeClockSet(0x30c, 19200000, true);
    if (!rc) rc = afeClockSet(0x30d, 19200000, true);
    if (rc) {printf("PDX203 HPH: TX core clocks refused (%d)\n", rc); return;}
    rc = pdx203_hph_init();
    gPath.hphReady = rc == 0;
    IOSleep(150);   // insertion debounce (96 ms): a plug already in shows on the first poll
    if (gPath.hphReady) gPath.jack = pdx203_hph_jack();
    pdx203_hph_print(gPath.hphReady ? "ready" : "failed");
}

void printPlay(const char *what) {
    const volatile int32_t *p = gPlay;
    printf("PDX203 PLAY %s step %d status %x votes %x/%x map %x copp %d writes %d done %d bad %d smmu %x %s amps %d/%d other %d beeps %d\n",
           what, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], gSmmuError, p[10], p[11], p[12], p[13]);
    for (unsigned amp = 0; amp < 2; ++amp) {
        ampStatus(amp);
        printf("PDX203 PLAY amp %x irq1 %x irq3 %x pwr2 %x otp %u\n", kAmpAddress[amp], pdx203_audio_amps[amp][3],
               pdx203_audio_amps[amp][5], pdx203_audio_amps[amp][6], pdx203_audio_amps[amp][7]);
    }
}

void dispatch(const AprMessage &m) {
    if (m.opcode == kAsmWriteDone) {onWriteDone(m); return;}
    gPlay[12] = gPlay[12] + 1;
    logMessage("event", m);
}

// The service's classes (ShengAudioService.cpp, static initializer taken out of __mod_init_func by the build) registered late, as the
// battery's, then the service published.
bool publishService() {
    void *handle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if (!handle) return false;
    native_audio_service_static_init();
    if (OSMetaClass::postModLoad(handle)) return false;
    sheng_audio_service_start();
    return true;
}

// ---- rows ----
uint32_t hashRows() {
    uint32_t h = 2166136261u;
    auto mix = [&](const volatile int32_t *v, unsigned n) {for (unsigned i = 0; i < n; ++i) h = (h ^ (uint32_t)v[i]) * 16777619u;};
    if (&native_adsp_stats) mix(native_adsp_stats, 12);   // [12..15] are SCM results and vote counters; they change with the rest
    if (&native_qrtr_stats) mix(native_qrtr_stats, 14);
    mix(pdx203_audio_stats, 15);
    return h;
}
void printRows() {
    if (&native_adsp_stats) {
        const volatile int32_t *a = native_adsp_stats;
        printf("PDX203 ADSP stage %d err %d ipcc %d slave %x items %x qmp %x drv %x fw %dK ready %dms glink %d/%d ch %x unk %d "
               "scm %x/%x votes %x %s\n", a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11], a[12], a[13], a[14],
               a[15], native_adsp_text);
    }
    if (&native_qrtr_stats) {
        const volatile int32_t *q = native_qrtr_stats;
        printf("PDX203 QRTR stage %d in %d data %d bad %d out %d refused %d loc %d/%d node %d svcs %d up %d down %d unrouted %d req %x | %s\n",
               q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], q[9], q[10], q[11], q[12], q[13], native_qrtr_text);
    }
    const volatile int32_t *s = pdx203_audio_stats;
    printf("PDX203 AUDIO stage %d se2 %08x amps %x i2c fail %d err %x | apr up %d in %d out %d dropped %d op %x | rcg %x dfs %x vote %x | %d ms\n",
           s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[9], s[10], s[12], s[13], s[14], s[15]);
    if (s[2]) {
        for (unsigned amp = 0; amp < 2; ++amp)
            if (s[2] & (1 << amp))
                printf("PDX203 AUDIO amp %x devid %x revid %x otpid %x irq1 %x init %u\n", kAmpAddress[amp], pdx203_audio_amps[amp][0],
                       pdx203_audio_amps[amp][1], pdx203_audio_amps[amp][2], pdx203_audio_amps[amp][3], pdx203_audio_amps[amp][4]);
    }
}

void printServe() {
    const ShengAudioControl *c = gPath.ctl;
    const ShengAudioDiag *d = c ? reinterpret_cast<const ShengAudioDiag *>(reinterpret_cast<const uint8_t *>(c) + SHENG_AUDIO_DIAG_OFFSET) : nullptr;
    printf("PDX203 SERVE svc %d opens %d closes %d reqs %d refused %d maps %d | want %x running %d starts %d stops %d failed %d late %d "
           "resync %d drift %d..%d lead %u | written %llu frames %llu | mount5 %d/%d | plugin loads %u vol %u peak %u clock %u/%d\n",
           native_audio_service[0], native_audio_service[1], native_audio_service[2], native_audio_service[3], native_audio_service[4],
           native_audio_service[5], gPath.want, gPath.running, gServe[0], gServe[1], gServe[2], gServe[3], gServe[4],
           gServe[5] == 0x7fffffff ? 0 : gServe[5], gServe[6] == -0x7fffffff ? 0 : gServe[6], c ? c->lead_frames : 0,
           c ? (unsigned long long)c->out_written : 0ull, c ? (unsigned long long)c->out_frames : 0ull,
           &native_gpu_addon5_stage ? native_gpu_addon5_stage : -9, &native_gpu_addon5_status ? native_gpu_addon5_status : -9,
           d ? d->loads : 0, d ? d->volume_milli : 0, d ? d->mix_peak : 0, d ? d->host_clock : 0, d ? d->host_clock_result : 0);
}

// The plug-in's record of what the audio server asked for (ShengAudioDiag: properties it does not answer) and of every distinct property
// access, failing VirtualAudio call and VirtualAudio throw (ShengAudioTrace), printed as they appear.
char fourcc(uint32_t v, unsigned shift) {const char c = (char)(v >> shift); return c >= 32 && c < 127 ? c : '.';}
void printPlugin() {
    static uint32_t diagShown, traceShown;
    if (!gPath.ctl) return;
    const uint8_t *page = reinterpret_cast<const uint8_t *>(gPath.ctl);
    const ShengAudioDiag *d = reinterpret_cast<const ShengAudioDiag *>(page + SHENG_AUDIO_DIAG_OFFSET);
    const ShengAudioTrace *t = reinterpret_cast<const ShengAudioTrace *>(page + SHENG_AUDIO_TRACE_OFFSET);
    const uint32_t dn = d->count < SHENG_AUDIO_DIAG_SLOTS ? d->count : SHENG_AUDIO_DIAG_SLOTS;
    for (; diagShown < dn; ++diagShown) {
        const uint32_t v = d->selector[diagShown], w = d->where[diagShown];
        printf("PDX203 HAL unanswered %u: '%c%c%c%c' object %u scope '%c'\n", diagShown, fourcc(v, 24), fourcc(v, 16), fourcc(v, 8), fourcc(v, 0),
               w >> 24, fourcc(w, 0));
    }
    const uint32_t tn = t->count < SHENG_AUDIO_TRACE_SLOTS ? t->count : SHENG_AUDIO_TRACE_SLOTS;
    for (; traceShown < tn; ++traceShown) {
        const uint32_t v = t->entry[traceShown][0], w = t->entry[traceShown][1];
        const uint32_t kind = w & 0xffff;
        if (kind & 8)
            printf("PDX203 HAL trace %u: VirtualAudio throw at +0x%x info %x\n", traceShown, v, w);
        else if (kind & 16)
            printf("PDX203 HAL trace %u: thrown value %x '%c%c%c%c'\n", traceShown, v, fourcc(v, 24), fourcc(v, 16), fourcc(v, 8), fourcc(v, 0));
        else if (kind & 32)
            printf("PDX203 HAL trace %u: thrown type '%c%c%c%c...'\n", traceShown, fourcc(v, 24), fourcc(v, 16), fourcc(v, 8), fourcc(v, 0));
        else
            printf("PDX203 HAL trace %u: '%c%c%c%c' object %u scope '%c' %s%s%s\n", traceShown, fourcc(v, 24), fourcc(v, 16), fourcc(v, 8),
                   fourcc(v, 0), w >> 24, fourcc(w, 16), (kind & 4) ? "va-call" : (kind & 1) ? "set" : "get", (kind & 2) ? " FAILED" : "",
                   (kind & 4) ? " (failed in VirtualAudio)" : "");
    }
}

void audioThread(void *, wait_result_t) {
    thread_set_thread_name(current_thread(), "pdx203-audio");
    unsigned mode = 0, delay = 40, beeps = 0;   // pdx203-audio-beeps=6 plays the test beeps before the service starts
    PE_parse_boot_argn("pdx203-audio", &mode, sizeof mode);
    PE_parse_boot_argn("pdx203-audio-delay", &delay, sizeof delay);
    PE_parse_boot_argn("pdx203-audio-beeps", &beeps, sizeof beeps);
    PE_parse_boot_argn("pdx203-audio-vol", &gAmpVolume, sizeof gAmpVolume);
    PE_parse_boot_argn("pdx203-audio-gain", &gAmpGain, sizeof gAmpGain);
    const uint64_t began = mach_absolute_time();
    if (mode & 1) probeAmps();
    if ((mode & 2) && sheng_adsp_channel) {
        static const ShengAdspClient client = {aprState, aprReceive};
        // the DSP opens apr_audio_svc (Linux's APR binds when the remote announces it); 20 intents of 512 bytes, as the DTB offers
        gApr = sheng_adsp_channel("apr_audio_svc", 512, 20, 0, &client, nullptr);
        if (!gApr) pdx203_audio_stats[0] = -10;
    }
    // Wait for the DSP's APR channel (up to five minutes), printing the bring-up rows as they change.
    uint32_t last = 0;
    for (unsigned tick = 0; tick < 600 && !(gApr && __atomic_load_n(&gAprUp, __ATOMIC_ACQUIRE)); ++tick) {
        AprMessage m;
        while (popMessage(m)) logMessage("in", m);
        const uint32_t h = hashRows();
        if (h != last) {last = h; printRows();}
        sleepMs(500);
    }
    printRows();
    if (!(mode & 4) || !gApr || !__atomic_load_n(&gAprUp, __ATOMIC_ACQUIRE)) return;
    // Service mode (bit 4): the path, then the test beeps (pdx203-audio-beeps, 0 for none), then the audio service for iOS.
    if (!setupShared() || !bootPath()) return;
    if (mode & 8) bootHeadphones();
    printPlay("ready");
    if (beeps) {
        uint64_t ns = 0;
        absolutetime_to_nanoseconds(mach_absolute_time() - began, &ns);
        if (ns / 1000000000ull < delay) IOSleep((uint32_t)(delay * 1000 - ns / 1000000));
        printf("PDX203 PLAY start: %u beeps, volume %x gain %x\n", beeps, gAmpVolume, gAmpGain);
        gPath.tone = true;
        if (startOutput()) {
            const uint64_t end = nowMs() + beeps * 3000ull;
            while (nowMs() < end) {AprMessage m; while (popMessage(m)) dispatch(m); sleepMs(20);}
            printPlay("played");
        }
        stopOutput();
        gPath.tone = false;
        printPlay("down");
    }
    if (!publishService()) {printf("PDX203 SERVE: the service did not register\n"); return;}
    __atomic_store_n(&gPath.ctl->state, gPath.ctl->state | ShengAudioOutputReady, __ATOMIC_RELEASE);
    __atomic_store_n(&gPath.serving, 1u, __ATOMIC_RELEASE);
    printServe();
    uint64_t nextRow = nowMs() + 30000;
    uint32_t lastServe = 0;
    for (;;) {
        const uint32_t want = __atomic_load_n(&gPath.want, __ATOMIC_ACQUIRE);
        if (want & ShengAudioOutputRunning) {
            gPath.stopAsked = 0;
            if (!gPath.running) {startOutput(); printServe();}
        } else if (gPath.running && !gPath.stopAsked) {
            gPath.stopAsked = nowMs();
        } else if (gPath.running && nowMs() - gPath.stopAsked >= kKeepMs) {
            gPath.stopAsked = 0;
            stopOutput();
            printServe();
        }
        AprMessage m;
        while (popMessage(m)) dispatch(m);
        // the jack, every 100 ms: a change while playing moves the stream to the other output
        static uint64_t nextJack;
        if (gPath.hphReady && nowMs() >= nextJack) {
            nextJack = nowMs() + 100;
            const int jack = pdx203_hph_jack();
            if (jack != gPath.jack) {
                gPath.jack = jack;
                pdx203_hph_print(jack == 1 ? "jack in" : "jack out");
                if (gPath.running && !gPath.hphForce) {stopOutput(); startOutput(); printServe();}
            }
        }
        printPlugin();
        const uint32_t sig = (uint32_t)native_audio_service[1] << 24 ^ (uint32_t)native_audio_service[3] << 12 ^ gServe[0] ^ gServe[2] << 8;
        if (sig != lastServe || nowMs() >= nextRow) {
            lastServe = sig;
            nextRow = nowMs() + (gPath.running ? 10000 : 60000);
            printServe();
        }
        sleepMs(gPath.running ? 5 : 250);
    }
}
}  // namespace

// The audio service's side (ShengAudioService.cpp): its memory and the plug-in's start and stop requests.
extern "C" IOBufferMemoryDescriptor *sheng_audio_memory(uint32_t type) {
    if (!__atomic_load_n(&gPath.serving, __ATOMIC_ACQUIRE)) return nullptr;
    IOBufferMemoryDescriptor *m = type == ShengAudioMemoryControl ? gPath.control : type == ShengAudioMemoryOutput ? gPath.outRing :
                                  type == ShengAudioMemoryInput ? gPath.inRing : nullptr;
    if (m) m->retain();
    return m;
}
extern "C" int sheng_audio_request(uint32_t selector) {
    if (!__atomic_load_n(&gPath.serving, __ATOMIC_ACQUIRE)) return -1;
    switch (selector) {
    case ShengAudioStartOutput: __atomic_or_fetch(&gPath.want, ShengAudioOutputRunning, __ATOMIC_ACQ_REL); break;
    case ShengAudioStopOutput: __atomic_and_fetch(&gPath.want, ~ShengAudioOutputRunning, __ATOMIC_ACQ_REL); break;
    case ShengAudioStartInput: case ShengAudioStopInput: return 0;   // no capture yet: the capture ring stays silent
    default: return -1;
    }
    wakeAudio();
    return 0;
}

extern "C" void pdx203_audio_probe(void) {
    unsigned mode = 0;
    if (!PE_parse_boot_argn("pdx203-audio", &mode, sizeof mode) || !mode || pdx203_audio_stats[0]) return;
    thread_t thread = nullptr;
    if (kernel_thread_start(audioThread, nullptr, &thread) != KERN_SUCCESS) {fail(9, -7); return;}
    thread_deallocate(thread);
}
