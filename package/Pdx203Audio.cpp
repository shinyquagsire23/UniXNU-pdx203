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
// pdx203-audio-beeps (default 5) beeps of 1 s of 750 Hz at -20 dBFS, 2 s apart, through AFE port SENARY_MI2S_RX, an ADM COPP and an
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
}

extern "C" ShengAdspChannel *sheng_adsp_channel(const char *, uint32_t, uint32_t, int, const ShengAdspClient *, void *) __attribute__((weak_import));
extern "C" int sheng_adsp_send(ShengAdspChannel *, const void *, uint32_t) __attribute__((weak_import));
extern "C" {
extern volatile int32_t native_adsp_stats[16] __attribute__((weak_import));
extern char native_adsp_text[96] __attribute__((weak_import));
extern volatile int32_t native_qrtr_stats[16] __attribute__((weak_import));
extern char native_qrtr_text[96] __attribute__((weak_import));
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
bool mapRegion(uint64_t phys, uint64_t length, Region &r) {
    IODeviceMemory *memory = IODeviceMemory::withRange(phys, length);
    r.map = memory ? memory->map(kIOMapAnywhere | kIOMapInhibitCache) : nullptr;
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
uint32_t gAmpVolume = 0xbe80, gAmpGain = 0;   // -6 dB digital, 0.5 dB analog (pdx203-audio-vol / -gain override)

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
volatile uint32_t gWritesDone;

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
void otherMessage(const AprMessage &m) {
    if (m.opcode == kAsmWriteDone) {
        gPlay[7] = gPlay[7] + 1;
        if (m.words >= 4 && m.payload[3]) gPlay[8] = gPlay[8] + 1;
        __atomic_fetch_add(&gWritesDone, 1u, __ATOMIC_RELEASE);
        return;
    }
    gPlay[12] = gPlay[12] + 1;
    logMessage("event", m);
}
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
            otherMessage(m);
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
int afeDevote(uint32_t block, int32_t handle) {
    Packet p(kSvcAfe, 0, 0, 0, kAfeDevote, 0x1c);
    p.u32(0x14, block); p.u32(0x18, (uint32_t)handle);
    return aprCall(p, 0);
}
int afeClock(bool enable) {   // AFE_SVC_CMD_SET_PARAM, module CLOCK_SET: SEN_MI2S_IBIT at 1.536 MHz
    Packet p(kSvcAfe, 0, 0, 1024, kAfeSvcSetParam, 0x44);
    p.u32(0x14, 32); p.u32(0x24, 0x0001028f); p.u32(0x28, 0x00010290); p.u16(0x2c, 20);
    p.u32(0x30, 1); p.u32(0x34, 0x10d); p.u32(0x38, 1536000); p.u16(0x3c, 1); p.u16(0x3e, 0); p.u32(0x40, enable ? 1 : 0);
    return aprCall(p, 0);
}
int afePortConfig() {   // AFE_PORT_CMD_SET_PARAM_V2, AFE_PARAM_ID_I2S_CONFIG: 16 bit, SD0, stereo, internal WS, 48 kHz, linear PCM
    Packet p(kSvcAfe, 0, 0, 0, kAfePortSetParam, 0x44);
    p.u16(0x14, kSenMi2sRx); p.u16(0x16, 32); p.u32(0x24, 0x0001020c); p.u32(0x28, 0x0001020d); p.u16(0x2c, 20);
    p.u32(0x30, 1); p.u16(0x34, 16); p.u16(0x36, 1); p.u16(0x38, 1); p.u16(0x3a, 1); p.u32(0x3c, 48000); p.u16(0x40, 0);
    return aprCall(p, 0);
}
int afePort(uint32_t opcode) {
    Packet p(kSvcAfe, 0, 0, 0, opcode, 0x18);
    p.u16(0x14, kSenMi2sRx);
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
int admOpen(int32_t *copp) {   // COPP on the senary MI2S with the null topology, 2 channels, 16 bit, 48 kHz, FL/FR
    Packet p(kSvcAdm, kSenMi2sRx, kSenMi2sRx, 0, kAdmOpen, 0x30);
    p.u16(0x14, 0); p.u16(0x16, 1); p.u16(0x18, kSenMi2sRx); p.u16(0x1a, 0xffff); p.u32(0x1c, 0x00010312);
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
int admClose(int32_t copp) {
    Packet p(kSvcAdm, kSenMi2sRx, (uint16_t)copp, 0, kAdmClose, 0x14);
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
int asmUnmap(int32_t handle) {
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

// 750 Hz is exactly 64 samples at 48 kHz, so every 10 ms period (480 frames) holds the same 7.5 cycles. -20 dBFS.
const int16_t kSine64[64] = {0, 321, 639, 951, 1254, 1545, 1821, 2079, 2317, 2533, 2725, 2890, 3028, 3136, 3214, 3261, 3277, 3261, 3214,
    3136, 3028, 2890, 2725, 2533, 2317, 2079, 1821, 1545, 1254, 951, 639, 321, 0, -321, -639, -951, -1254, -1545, -1821, -2079, -2317,
    -2533, -2725, -2890, -3028, -3136, -3214, -3261, -3277, -3261, -3214, -3136, -3028, -2890, -2725, -2533, -2317, -2079, -1821, -1545,
    -1254, -951, -639, -321};
void fillSlot(uint32_t slot, bool tone) {
    volatile uint32_t *frames = reinterpret_cast<volatile uint32_t *>(gBuf.va + slot * kSlotBytes);
    for (uint32_t i = 0; i < kPeriodFrames; ++i) {
        const uint16_t s = tone ? (uint16_t)kSine64[i % 64] : 0;
        frames[i] = (uint32_t)s | (uint32_t)s << 16;   // left and right
    }
    __asm__ volatile("dsb sy" ::: "memory");
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

// Brings the path up, plays beeps (1 s tone, 2 s silence) and takes it down again. Steps report in gPlay[0].
void playBeeps(unsigned beeps) {
    auto step = [](int n) {gPlay[0] = n;};
    auto failed = [](int n, int rc) {gPlay[0] = -n; gPlay[1] = rc; printPlay("failed"); return;};
    int rc = 0;
    int32_t macro = -1, dcodec = -1, map = -1, copp = -1;
    bool clock = false, port = false, opened = false, ampsOn = false;
    for (unsigned amp = 0; amp < 2; ++amp) {
        if (!(pdx203_audio_stats[2] & (1 << amp))) {failed(1, -19); return;}
        if ((rc = ampInit(amp))) {pdx203_audio_amps[amp][4] = (uint32_t)rc; failed(1, rc); return;}
    }
    step(1);
    if (!smmuPassThrough()) {failed(2, -1); return;}
    if (!mapRegion(kBufPhys, kBufSize, gBuf)) {failed(2, -12); return;}
    for (uint32_t s = 0; s < kSlots; ++s) fillSlot(s, false);
    step(2);
    if ((rc = afeVote(3, &macro))) {failed(3, rc); return;}
    gPlay[2] = macro;
    if (afeVote(4, &dcodec) == 0) gPlay[3] = dcodec;
    if (!lpiPins()) {failed(3, -12); goto down;}
    step(3);
    if ((rc = afeClock(true))) {failed(4, rc); goto down;}
    clock = true;
    if ((rc = afePortConfig())) {failed(5, rc); goto down;}
    if ((rc = afePort(kAfeStart))) {failed(5, rc); goto down;}
    port = true;
    step(5);
    IOSleep(5);   // the bit clock runs: the amplifiers' PLL locks to it
    for (unsigned amp = 0; amp < 2; ++amp) {gPlay[10 + amp] = ampStart(amp); ampsOn = true;}
    step(6);
    if ((rc = asmMap(&map))) {failed(7, rc); goto down;}
    gPlay[4] = map;
    if ((rc = asmOpen())) {failed(8, rc); goto down;}
    opened = true;
    if ((rc = admOpen(&copp))) {failed(9, rc); goto down;}
    gPlay[5] = copp;
    if ((rc = admRoute(copp))) {failed(10, rc); goto down;}
    if ((rc = asmFormat())) {failed(11, rc); goto down;}
    if ((rc = asmSimple(kAsmRun, 0x20))) {failed(12, rc); goto down;}
    step(12);
    printPlay("running");
    {
        // Periods in order: each beep is 100 tone periods then 200 silent ones. Four are kept queued at the DSP.
        const uint32_t total = beeps * 300;
        uint32_t sent = 0, done0 = __atomic_load_n(&gWritesDone, __ATOMIC_ACQUIRE);
        uint64_t stall;
        clock_interval_to_deadline(2000, kMillisecondScale, &stall);
        while (sent < total || __atomic_load_n(&gWritesDone, __ATOMIC_ACQUIRE) - done0 < sent) {
            const uint32_t done = __atomic_load_n(&gWritesDone, __ATOMIC_ACQUIRE) - done0;
            while (sent < total && sent - done < 4) {
                const uint32_t slot = sent % kSlots;
                fillSlot(slot, (sent % 300) < 100);
                if (!asmWrite(slot)) break;
                if (sent % 300 == 0) gPlay[13] = gPlay[13] + 1;
                ++sent;
                clock_interval_to_deadline(2000, kMillisecondScale, &stall);
            }
            AprMessage m;
            bool any = false;
            while (popMessage(m)) {otherMessage(m); any = true;}
            if (any) clock_interval_to_deadline(2000, kMillisecondScale, &stall);
            if (mach_absolute_time() >= stall) {printf("PDX203 PLAY stalled: sent %u done %u\n", sent, done); break;}
            if (sent == 100 || sent == 400) printPlay("tone");
            sleepMs(5);
        }
    }
    step(13);
    printPlay("played");
down:
    if (opened) asmSimple(kAsmPause, 0x14);
    if (ampsOn) for (unsigned amp = 0; amp < 2; ++amp) ampStop(amp);
    if (opened) asmSimple(kAsmClose, 0x14);
    if (copp >= 0) admClose(copp);
    if (port) afePort(kAfeStop);
    if (clock) afeClock(false);
    if (map >= 0) asmUnmap(map);
    if (dcodec >= 0) afeDevote(4, dcodec);
    if (macro >= 0) afeDevote(3, macro);
    printPlay("down");
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

void audioThread(void *, wait_result_t) {
    thread_set_thread_name(current_thread(), "pdx203-audio");
    unsigned mode = 0, delay = 40, beeps = 5;
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
    uint32_t last = 0;
    bool played = false;
    for (unsigned tick = 0; tick < 1200; ++tick) {   // ten minutes of 500 ms steps
        AprMessage m;
        while (popMessage(m)) logMessage("in", m);
        uint64_t ns = 0;
        absolutetime_to_nanoseconds(mach_absolute_time() - began, &ns);
        pdx203_audio_stats[15] = (int32_t)(ns / 1000000);
        const uint32_t h = hashRows();
        if (h != last || tick % 120 == 0) {last = h; printRows();}
        if ((mode & 5) == 5 && gApr && !played && __atomic_load_n(&gAprUp, __ATOMIC_ACQUIRE) && ns / 1000000000ull >= delay) {
            played = true;
            printf("PDX203 PLAY start: %u beeps, volume %x gain %x\n", beeps, gAmpVolume, gAmpGain);
            playBeeps(beeps);
        }
        sleepMs(500);
    }
    printRows();
}
}  // namespace

extern "C" void pdx203_audio_probe(void) {
    unsigned mode = 0;
    if (!PE_parse_boot_argn("pdx203-audio", &mode, sizeof mode) || !mode || pdx203_audio_stats[0]) return;
    thread_t thread = nullptr;
    if (kernel_thread_start(audioThread, nullptr, &thread) != KERN_SUCCESS) {fail(9, -7); return;}
    thread_deallocate(thread);
}
