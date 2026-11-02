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
// Boot argument pdx203-audio (bits): 1 reads the amplifiers' identity over I2C; 2 talks to the audio DSP's APR core service over GLINK
// channel "apr_audio_svc" (needs pdx203-adsp=2): it asks whether the DSP's audio framework is up (AVCS_CMD_ADSP_EVENT_GET_STATE) and
// for its service versions (AVCS_GET_VERSIONS). Rows print as "PDX203 ADSP", "PDX203 QRTR", "PDX203 AUDIO" and "PDX203 APR" on the
// console whenever something changed, for ten minutes.
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
volatile uint32_t pdx203_audio_amps[2][4];
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
constexpr uint8_t kDomainAdsp = 4, kDomainApps = 5, kSvcCore = 3;
constexpr uint32_t kBasicResult = 0x000110e8, kGetState = 0x0001326c, kGetStateRsp = 0x0001326d, kGetVersions = 0x00012905,
                   kGetVersionsRsp = 0x00012906;

ShengAdspChannel *gApr;
volatile uint32_t gAprUp;
// Messages the DSP thread hands over: a small ring, copied out by the audio thread.
struct AprMessage {uint32_t length; uint8_t data[256];};
AprMessage gRing[8];
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
    const uint32_t head = gRingHead;
    if (head - __atomic_load_n(&gRingTail, __ATOMIC_ACQUIRE) >= 8) return;   // full: dropped (counted in [6] only)
    AprMessage &m = gRing[head % 8];
    m.length = length < sizeof m.data ? length : sizeof m.data;
    memcpy(m.data, data, m.length);
    __atomic_store_n(&gRingHead, head + 1, __ATOMIC_RELEASE);
    wakeAudio();
}
bool aprSend(uint8_t svc, uint32_t opcode, uint32_t token) {
    AprHeader h = {kAprSeqCmd, sizeof(AprHeader), svc, kDomainApps, 0, svc, kDomainAdsp, 0, token, opcode};
    if (!sheng_adsp_send || sheng_adsp_send(gApr, &h, sizeof h)) return false;
    pdx203_audio_stats[7] = pdx203_audio_stats[7] + 1;
    return true;
}
void sleepMs(uint32_t ms) {
    if (__atomic_exchange_n(&gWake, 0, __ATOMIC_ACQ_REL)) return;
    assert_wait_timeout((event_t)&gWake, THREAD_UNINT, ms, kMillisecondScale);
    if (__atomic_exchange_n(&gWake, 0, __ATOMIC_ACQ_REL)) {clear_wait(current_thread(), THREAD_AWAKENED); return;}
    thread_block(THREAD_CONTINUE_NULL);
    __atomic_store_n(&gWake, 0, __ATOMIC_RELEASE);
}

void handleApr(const AprMessage &m) {
    if (m.length < sizeof(AprHeader)) return;
    AprHeader h;
    memcpy(&h, m.data, sizeof h);
    const uint32_t *payload = reinterpret_cast<const uint32_t *>(m.data + sizeof h);
    const uint32_t words = (m.length - sizeof h) / 4;
    pdx203_audio_stats[10] = (int32_t)h.opcode;
    char text[160];
    unsigned at = 0;
    for (uint32_t i = 0; i < words && i < 12 && at + 10 < sizeof text; ++i) at += snprintf(text + at, sizeof text - at, " %x", payload[i]);
    text[at] = 0;
    printf("PDX203 APR in op %x from %u:%u:%u to %u:%u:%u token %x len %u:%s\n", h.opcode, h.src_domain, h.src_svc, h.src_port,
           h.dest_domain, h.dest_svc, h.dest_port, h.token, m.length, text);
    if (h.opcode == kBasicResult && words >= 2) pdx203_audio_stats[11] = (int32_t)((payload[0] & 0xffff) | payload[1] << 16);
    if (h.opcode == kGetStateRsp && words >= 1) pdx203_audio_stats[8] = (int32_t)payload[0];
    if (h.opcode == kGetVersionsRsp) printf("PDX203 APR versions: %u words\n", words);
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
    printf("PDX203 AUDIO stage %d se2 %08x amps %x i2c fail %d err %x | apr up %d in %d out %d state %d asks %d op %x result %x | "
           "rcg %x dfs %x vote %x | %d ms\n", s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8], s[9], s[10], s[11], s[12], s[13], s[14],
           s[15]);
    if (s[2]) {
        for (unsigned amp = 0; amp < 2; ++amp)
            if (s[2] & (1 << amp))
                printf("PDX203 AUDIO amp %x devid %x revid %x otpid %x irq1 %x\n", kAmpAddress[amp], pdx203_audio_amps[amp][0],
                       pdx203_audio_amps[amp][1], pdx203_audio_amps[amp][2], pdx203_audio_amps[amp][3]);
    }
}

void audioThread(void *, wait_result_t) {
    thread_set_thread_name(current_thread(), "pdx203-audio");
    unsigned mode = 0;
    PE_parse_boot_argn("pdx203-audio", &mode, sizeof mode);
    const uint64_t began = mach_absolute_time();
    pdx203_audio_stats[8] = -1;
    if (mode & 1) probeAmps();
    if ((mode & 2) && sheng_adsp_channel) {
        static const ShengAdspClient client = {aprState, aprReceive};
        // the DSP opens apr_audio_svc (Linux's APR binds when the remote announces it); 20 intents of 512 bytes, as the DTB offers
        gApr = sheng_adsp_channel("apr_audio_svc", 512, 20, 0, &client, nullptr);
        if (!gApr) pdx203_audio_stats[0] = -10;
    }
    uint32_t last = 0;
    bool asked = false, versions = false;
    uint64_t nextAsk = 0;
    for (unsigned tick = 0; tick < 1200; ++tick) {   // ten minutes of 500 ms steps
        while (__atomic_load_n(&gRingTail, __ATOMIC_ACQUIRE) != __atomic_load_n(&gRingHead, __ATOMIC_ACQUIRE)) {
            const uint32_t tail = gRingTail;
            handleApr(gRing[tail % 8]);
            __atomic_store_n(&gRingTail, tail + 1, __ATOMIC_RELEASE);
        }
        if (gApr && __atomic_load_n(&gAprUp, __ATOMIC_ACQUIRE)) {
            const uint64_t now = mach_absolute_time();
            if (pdx203_audio_stats[8] != 1 && pdx203_audio_stats[9] < 40 && now >= nextAsk) {
                if (aprSend(kSvcCore, kGetState, 0x5001)) {pdx203_audio_stats[9] = pdx203_audio_stats[9] + 1; asked = true;}
                clock_interval_to_deadline(500, kMillisecondScale, &nextAsk);
            }
            if (pdx203_audio_stats[8] == 1 && !versions) versions = aprSend(kSvcCore, kGetVersions, 0x5002);
        }
        uint64_t ns = 0;
        absolutetime_to_nanoseconds(mach_absolute_time() - began, &ns);
        pdx203_audio_stats[15] = (int32_t)(ns / 1000000);
        const uint32_t h = hashRows();
        if (h != last || tick % 120 == 0) {last = h; printRows();}
        sleepMs(500);
    }
    (void)asked;
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
