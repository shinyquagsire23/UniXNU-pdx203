// SPDX-License-Identifier: BSD-2-Clause-Patent
// pdx203: UniXNU-alioth package/CpuFreqProbe.cpp (same SM8250 EPSS windows), unchanged down to the pdx203 poll thread at the end.
// Default-off CPU frequency control for this board's EPSS frequency domains (kona).
// Adapted from the reference board's CpuFreqProbe (UniXNU-sheng, SM8550): the EPSS register
// layout is the same qcom-cpufreq-hw-epss interface, and the SM8250 addresses and domain CPU
// counts come from this board's own device tree (kona.dtsi:325-330) and the same SoC's pipa
// port, which votes the identical hardware. This board's device tree declares no EPSS L3, so
// the L3 window the SM8550 version maps is absent here. `sheng-cpufreq=<n>`:
//   1  read-only: report each domain's enable bit, current/requested level and its highest LUT entry
//   2  additionally request that highest non-turbo level in domain 0 (the boot CPU's cluster)
//   3  the same in every domain once it reads enabled, and the highest L3 level
//   4  mode 3, and threads kept off the Cortex-A55 cluster (cpus 0-3; see pdx203-cores below)
//   5  mode 4 with an interactive governor: the top level while the screen is touched or frames are being drawn, and for
//      1.5 s after; at most 845 MHz otherwise (see cpufreqThread)
// Optional `sheng-cpufreq-cap=<MHz>`: mode 3 requests the highest non-turbo level at or below it (domains and L3).
// Register layout is the qcom-cpufreq-hw "epss" data of the pinned mainline tree (reg_enable 0x0, reg_domain_state 0x20,
// reg_dcvs_ctrl 0xb0, reg_freq_lut 0x100 with 4-byte rows, reg_perf_state 0x320, per-core perf_state rows +4*i for the
// policy's other CPUs when dcvs_ctrl bit 0 is set); the three domain windows come from the cpufreq@17d91000 node of
// sm8550.dtsi, their CPU counts from the cpus' qcom,freq-domain (3, 4, 1). The L3 is osm-l3.c "qcom,epss-l3" (the sm8550
// node interconnect@17d90000): REG_ENABLE 0x0, LUT 0x100 with 4-byte rows, vote EPSS_REG_L3_VOTE 0x90; only those three
// offsets are touched there. LUT row: bits 31:30 source, 18:16 core count (1 = turbo), 7:0 L value; an L value is a
// multiple of the 19.2 MHz bi_tcxo_div2 clock, source 0 is GPLL0/2.
// A cluster whose cores are all off reads enable 0 and state 0 (int-143 read at 7 s, before the secondary cores ran), so the
// probe maps the windows once and sheng_cpufreq_poll(), called every log round, votes each domain the first time it reads
// enabled and refreshes the row. Nothing here is validated on the device until a boot has shown it.
#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <pexpert/pexpert.h>
#include <kern/clock.h>

extern "C" {
char native_cpufreq_row[120];
char native_thermal_row[120];
volatile uint32_t native_cpufreq_stage;
volatile int native_thermal_max_mc = -100000;   // hottest sensor, milliCelsius; the future throttle governor reads this
volatile uint32_t pdx203_ui_boosted = 1;       // 1 while the interactive boost holds (GpuProbe's DCVS follows it)
}

namespace {
constexpr uint64_t kDomainBase[3] = {0x18591000, 0x18592000, 0x18593000};
constexpr unsigned kDomainCpus[3] = {4, 3, 1};
constexpr uint64_t kWindowSize = 0x1000;
constexpr uint32_t kRegEnable = 0x0, kRegDomainState = 0x20, kRegDcvsCtrl = 0xb0, kRegFreqLut = 0x100, kRegPerfState = 0x320;
constexpr unsigned kLutEntries = 40;
constexpr uint32_t kXoKHz = 19200;        // bi_tcxo_div2
constexpr uint32_t kGpllHalfKHz = 300000; // GPLL0 600 MHz / CLK_HW_DIV 2

struct Window {
    IOMemoryMap *map;
    uintptr_t va;
    int voted;          // level written, -1 none
    uint32_t votedKHz;
    uint32_t preKHz;    // domain state the first time it read enabled, before the vote
};
Window gDomain[3];
unsigned gMode;
uint32_t gCapKHz;
IOLock *gLock;

// TSENS die temperature (qcom,tsens24xx v2, from this board's DTB tsens@c222000 / @c223000): two controllers, their thermal-
// monitor (TM) blocks at 0x0c263000 (TSENS0) and 0x0c265000 (TSENS1), each with up to 16 sensors. Per-sensor status register at
// TM + 0xa0 + 4*n: bit 21 = valid, bits[11:0] = a signed temperature code (sign-extend from bit 11), 0.1 C per LSB -- the mainline
// qcom tsens driver converts v2 as sign_extend(code,11)*100 mC with no per-sensor slope. Only the NS-readable TM block is touched
// (never SROT). readThermal reports the hottest sensor of each kind and the overall max (the CPU and GPU throttles read it).
constexpr uint64_t kTsensTmBase[2] = {0x0c263000, 0x0c265000};
constexpr uint32_t kTsensSnStatus = 0xa0;   // Sn_STATUS = 0xa0 + 4*n
constexpr unsigned kTsensSensors = 16;
Window gTsens[2];

int tsensMilliC(uint32_t status) {
    int code = (int)(status & 0xfff);
    if (code & 0x800) code -= 0x1000;       // sign-extend the 12-bit field
    return code * 100;                       // milliCelsius
}

uint32_t rd32(uintptr_t base, uint32_t reg) {return *reinterpret_cast<volatile uint32_t *>(base + reg);}
void wr32(uintptr_t base, uint32_t reg, uint32_t value) {
    *reinterpret_cast<volatile uint32_t *>(base + reg) = value;
    __asm__ volatile("dsb sy" ::: "memory");
}
uint32_t lutKHz(uint32_t row) {return (row >> 30) ? kXoKHz * (row & 0xff) : kGpllHalfKHz;}

// Highest non-turbo LUT level (at or below the cap when one is set); the table ends at two equal frequencies.
int pickLevel(uintptr_t va, bool turboMatters, uint32_t *khzOut, unsigned *rowsOut) {
    int best = -1; uint32_t bestKHz = 0, prev = 0; unsigned rows = 0;
    for (unsigned i = 0; i < kLutEntries; ++i) {
        uint32_t row = rd32(va, kRegFreqLut + i * 4);
        uint32_t khz = lutKHz(row);
        if (i > 0 && khz == prev) break;
        rows = i + 1;
        prev = khz;
        if (turboMatters && ((row >> 16) & 7) == 1) continue;
        if (gCapKHz && khz > gCapKHz) continue;
        if (khz >= bestKHz) {best = (int)i; bestKHz = khz;}
    }
    if (khzOut) *khzOut = bestKHz;
    if (rowsOut) *rowsOut = rows;
    return best;
}

bool mapWindow(Window &w, uint64_t base) {
    IODeviceMemory *memory = IODeviceMemory::withRange(base, kWindowSize);
    w.map = memory ? memory->map(kIOMapAnywhere | kIOMapInhibitCache) : nullptr;
    if (memory) memory->release();
    w.va = w.map ? static_cast<uintptr_t>(w.map->getVirtualAddress()) : 0;
    w.voted = -1;
    return w.va != 0;
}

void voteDomain(unsigned d) {
    Window &w = gDomain[d];
    uint32_t khz = 0;
    int level = pickLevel(w.va, true, &khz, nullptr);
    if (level < 0) return;
    w.preKHz = (rd32(w.va, kRegDomainState) & 0xff) * kXoKHz;
    wr32(w.va, kRegPerfState, (uint32_t)level);
    if (rd32(w.va, kRegDcvsCtrl) & 1)
        for (unsigned c = 1; c < kDomainCpus[d]; ++c) wr32(w.va, kRegPerfState + c * 4, (uint32_t)level);
    w.voted = level;
    w.votedKHz = khz;
}

void refreshRow() {
    char *out = native_cpufreq_row;
    const size_t cap = sizeof(native_cpufreq_row);
    unsigned n = (unsigned)snprintf(out, cap, "CPUFREQ");
    for (unsigned d = 0; d < 3 && n < cap; ++d) {
        Window &w = gDomain[d];
        if (!w.va) {n += (unsigned)snprintf(out + n, cap - n, " d%u:nomap", d); continue;}
        uint32_t enable = rd32(w.va, kRegEnable) & 1, state = (rd32(w.va, kRegDomainState) & 0xff) * kXoKHz / 1000;
        if (w.voted >= 0)
            n += (unsigned)snprintf(out + n, cap - n, " d%u:e%u c%u v%d@%u pre%u", d, enable, state, w.voted, w.votedKHz / 1000,
                                    w.preKHz / 1000);
        else
            n += (unsigned)snprintf(out + n, cap - n, " d%u:e%u c%u p%u", d, enable, state, rd32(w.va, kRegPerfState));
    }

}

// What each sensor sits on, from this board's DTB thermal-zones (the <name>-usr zones and their thermal-sensors): TSENS0 s0 aoss0,
// s1-4 the Cortex-A55s, s5-6 cpuss, s7-14 the Cortex-A77s, s15 gpuss-0; TSENS1 s0 aoss-1, s1 wlan, s2 video, s3 ddr, s4 q6-hvx,
// s5 camera, s6 cmpss, s7 npu, s8 gpuss-1. Qualcomm's gpuss-max-step zone (both gpuss sensors) starts stepping the GPU down at 95 C.
enum SensorClass : uint8_t {kOther, kA55, kCpuss, kA77, kGpu, kDdr, kClasses};
constexpr SensorClass kSensorClass[2][kTsensSensors] = {
    {kOther, kA55, kA55, kA55, kA55, kCpuss, kCpuss, kA77, kA77, kA77, kA77, kA77, kA77, kA77, kA77, kGpu},
    {kOther, kOther, kOther, kDdr, kOther, kOther, kOther, kOther, kGpu, kOther, kOther, kOther, kOther, kOther, kOther, kOther}};
constexpr const char *kClassName[kClasses] = {"soc", "a55", "cpuss", "a77", "gpu", "ddr"};

// One sensor, -100000 if its controller is unmapped or it has no valid sample yet.
int sensorMilliC(unsigned t, unsigned s) {
    if (!gTsens[t].va) return -100000;
    uint32_t status = rd32(gTsens[t].va, kTsensSnStatus + s * 4);
    return ((status >> 21) & 1) ? tsensMilliC(status) : -100000;
}

void readThermal() {
    int cls[kClasses], gmax = -100000; unsigned hotT = 0, hotS = 0;
    for (auto &c : cls) c = -100000;
    for (unsigned t = 0; t < 2; ++t)
        for (unsigned s = 0; s < kTsensSensors; ++s) {
            int mC = sensorMilliC(t, s);
            if (mC <= -100000) continue;
            if (mC > cls[kSensorClass[t][s]]) cls[kSensorClass[t][s]] = mC;
            if (mC > gmax) {gmax = mC; hotT = t; hotS = s;}
        }
    char *out = native_thermal_row;
    const size_t cap = sizeof(native_thermal_row);
    unsigned n = (unsigned)snprintf(out, cap, "THERMAL");
    for (unsigned c = kA55; c < kClasses && n < cap; ++c)
        if (cls[c] > -100000) n += (unsigned)snprintf(out + n, cap - n, " %s %d.%d", kClassName[c], cls[c] / 1000, (cls[c] % 1000) / 100);
    native_thermal_max_mc = gmax;
    if (gmax > -100000 && n < cap)
        snprintf(out + n, cap - n, " max %d.%dC %s t%us%u", gmax / 1000, (gmax % 1000) / 100, kClassName[kSensorClass[hotT][hotS]], hotT, hotS);
    else if (n < cap) snprintf(out + n, cap - n, " none");
}

// Thermal governor: we pin the CPU domains at their highest non-turbo level for 60 fps, which cooks the SoC to ~88 C even on the
// idle lock screen (TSENS). Step a frequency cap down when the hottest sensor crosses the high mark and re-vote every domain; step
// it back up (toward uncapped) below the low mark, rate-limited (thermalGovern). The hottest sensor is an A77 core under load (a game:
// A77s 84-87 C while the gpuss sensors read ~72 C); GpuProbe caps the GPU on its own sensors. Thresholds baked (the kernel command
// line is at its 1000-byte limit, no room for boot-args); they stay mutable globals so a runtime knob can tune them later.
int gThermHighMc = 83000, gThermLowMc = 73000;
constexpr uint32_t kThrottleCapKHz[] = {0, 2400000, 2000000, 1612800, 1305600};  // step 0 = uncapped; higher step = cooler
constexpr unsigned kThrottleSteps = sizeof(kThrottleCapKHz) / sizeof(kThrottleCapKHz[0]);
unsigned gThermStep;

// sheng-cpufreq=5: the idle cap cpufreqThread applies while nothing is touched or drawn.
constexpr uint32_t kIdleCapKHz = 844800;   // the lowest cap every domain has a level under (A55 787, A77 826, prime 845 MHz on the sm8250 tables)
bool gIdle;
uint32_t effectiveCapKHz() {
    uint32_t cap = gCapKHz, t = kThrottleCapKHz[gThermStep];
    if (t && (!cap || t < cap)) cap = t;
    if (gIdle && (!cap || kIdleCapKHz < cap)) cap = kIdleCapKHz;
    return cap;
}
void revoteAll() {
    uint32_t saved = gCapKHz;
    gCapKHz = effectiveCapKHz();           // pickLevel reads gCapKHz
    for (unsigned d = 0; d < 3; ++d) {
        Window &w = gDomain[d];
        if (w.va && (rd32(w.va, kRegEnable) & 1)) {w.voted = -1; voteDomain(d);}
    }
    gCapKHz = saved;
}
// Called on every TSENS read: every 100 ms from cpufreqThread's ticks once the interactive governor runs, every poll before. One step
// down at most every 500 ms while above the high mark, one step up at most every 3 s below the low mark. (Read every 5 s, the A77s
// went from 64 to 88 C between two reads while a game loaded, and the cap hunted a step per read.)
constexpr uint64_t kThermDownMs = 500, kThermUpMs = 3000;
uint64_t gThermChangedMs;
void thermalGovern() {
    int t = native_thermal_max_mc;
    if (t <= -100000) return;              // no reading yet
    uint64_t ns = 0;
    absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
    const uint64_t nowMs = ns / 1000000, since = nowMs - gThermChangedMs;
    unsigned old = gThermStep;
    if (t > gThermHighMc && gThermStep + 1 < kThrottleSteps && since >= kThermDownMs) ++gThermStep;
    else if (t < gThermLowMc && gThermStep > 0 && since >= kThermUpMs) --gThermStep;
    if (gThermStep != old) {
        gThermChangedMs = nowMs;
        revoteAll();
        uint32_t cap = effectiveCapKHz();
        printf("PDX203 THROTTLE step %u cap %uMHz at %d.%dC\n", gThermStep, cap ? cap / 1000 : 0, t / 1000, (t % 1000) / 100);
    }
}
}

// GpuProbe's thermal cap reads the two gpuss sensors every 50 ms (a few register reads, no lock: TSENS updates them by itself) and
// the hottest sensor overall. Returns the hotter gpuss sensor in milliCelsius, -100000 before the TSENS windows are mapped.
extern "C" int pdx203_gpu_temp_mc(int *hottestMc) {
    int gpu = -100000, hot = -100000;
    for (unsigned t = 0; t < 2; ++t)
        for (unsigned s = 0; s < kTsensSensors; ++s) {
            int mC = sensorMilliC(t, s);
            if (mC > hot) hot = mC;
            if (kSensorClass[t][s] == kGpu && mC > gpu) gpu = mC;
        }
    if (hottestMc) *hottestMc = hot;
    return gpu;
}

extern "C" void sheng_cpufreq_poll(void) {
    if (!gMode || !gLock) return;
    IOLockLock(gLock);
    for (unsigned d = 0; d < 3; ++d) {
        Window &w = gDomain[d];
        bool wanted = gMode >= 3 || (gMode == 2 && d == 0);
        if (w.va && wanted && w.voted < 0 && (rd32(w.va, kRegEnable) & 1)) {
            uint32_t saved = gCapKHz;
            gCapKHz = effectiveCapKHz();   // a domain that comes up while idle (or throttled) gets the capped level too
            voteDomain(d);
            gCapKHz = saved;
        }
    }
    refreshRow();
    readThermal();
    thermalGovern();
    native_cpufreq_stage = 2;
    IOLockUnlock(gLock);
}

extern "C" void sheng_cpufreq_probe(void) {
    unsigned mode = 0, capMHz = 0;
    PE_parse_boot_argn("sheng-cpufreq", &mode, sizeof(mode));
    PE_parse_boot_argn("sheng-cpufreq-cap", &capMHz, sizeof(capMHz));
    if (!mode || gLock) return;
    gLock = IOLockAlloc();
    if (!gLock) return;
    native_cpufreq_stage = 1;
    gCapKHz = capMHz * 1000;
    for (unsigned d = 0; d < 3; ++d) mapWindow(gDomain[d], kDomainBase[d]);
    for (unsigned t = 0; t < 2; ++t) mapWindow(gTsens[t], kTsensTmBase[t]);
    gMode = mode;
    sheng_cpufreq_poll();
}
// pdx203: alioth runs sheng_cpufreq_poll() from its boot log's rounds, which this board does not have. A thread polls instead:
// every second for the first minute (a cluster whose cores are still off reads disabled and is voted once it comes up), then
// every 30 s, and prints the CPUFREQ row to the console (and so the USB log) whenever it changes.
#include <kern/thread.h>
#include <libkern/libkern.h>
// sheng-cpufreq=4 (mode 3's votes) or pdx203-cores=<mask>: the cores the scheduler may run threads on, applied once every domain has its
// vote (or after the first minute); mode 4 means 0xf0 unless pdx203-cores says otherwise (the kernel command line has no room for both).
// XNU sees this SoC as eight like cores in one cluster (ml_processor_register: cluster 0, type 0 for all, the Clutch SMP scheduler),
// so the render server's, WebKit's and the compositor's frame threads land on the 1.8 GHz Cortex-A55s (cpus 0-3) as readily as on the
// 2.4/2.8 GHz Cortex-A77s (4-7). 0xf0 keeps them on the A77s. The same call the performance controller makes to park cores
// (sched_perfcontrol_update_recommended_cores): cores left out still take interrupts and run threads bound to them, and the scheduler's
// own failsafe brings every core back while a realtime thread is starved. 0 (the default) leaves every core recommended.
extern "C" void sched_perfcontrol_update_recommended_cores(uint32_t recommended_cores);
// sheng-cpufreq=5, the interactive governor (iOS's own CLPC raises the clocks the moment a finger lands and lets them fall when
// the screen goes still; this board pinned every domain at its top level instead, ~74 C idle). Every 25 ms the thread looks at two
// counters: native_touch_seq (Pdx203TouchProbe, bumped on every touch report) and pdx203_swap_submits (LegacyDisplayProbe, one per
// frame the compositor submits). A touch, or at least 4 frames in the last 200 ms (20 fps: an animation, scrolling, video), keeps
// the top level for 1.5 s more; otherwise every domain is re-voted at most kIdleCapKHz. The thermal cap still applies on top.
extern "C" {
extern volatile uint32_t native_touch_seq;
extern volatile uint32_t pdx203_swap_submits __attribute__((weak));
unsigned pdx203_swap_trace(char *out, unsigned cap) __attribute__((weak_import));   // LegacyDisplayProbe
extern volatile uint32_t pdx203_gpu_dcvs_stats[6] __attribute__((weak));            // GpuProbe: [0] the GPU level
}
namespace {
constexpr uint64_t kTickMs = 25, kHoldMs = 1500;
constexpr unsigned kFrameWindow = 8, kFrameBoost = 4;  // 8 ticks = 200 ms
struct Interactive {
    uint32_t touchSeq, swaps, frames[kFrameWindow];
    unsigned slot;
    uint64_t boostUntilMs, nowMs, boostedTicks, ticks, boosts;
    uint32_t touchTicks, frameTicks, windowSwaps;   // what renewed the hold this window, and the frames swapped
};
// Returns true when the idle state changed (the caller re-votes).
bool interactiveTick(Interactive &s) {
    s.nowMs += kTickMs;
    uint32_t seq = native_touch_seq, swaps = &pdx203_swap_submits ? pdx203_swap_submits : 0;
    s.frames[s.slot] = swaps - s.swaps;
    s.slot = (s.slot + 1) % kFrameWindow;
    s.swaps = swaps;
    unsigned frames = 0;
    for (unsigned i = 0; i < kFrameWindow; ++i) frames += s.frames[i];
    s.windowSwaps += s.frames[(s.slot + kFrameWindow - 1) % kFrameWindow];
    if (seq != s.touchSeq) ++s.touchTicks;
    else if (frames >= kFrameBoost) ++s.frameTicks;
    if (seq != s.touchSeq || frames >= kFrameBoost) s.boostUntilMs = s.nowMs + kHoldMs;
    s.touchSeq = seq;
    bool idle = s.nowMs >= s.boostUntilMs;
    ++s.ticks;
    if (!idle) ++s.boostedTicks;
    if (idle == gIdle) return false;
    if (!idle) ++s.boosts;
    gIdle = idle;
    pdx203_ui_boosted = idle ? 0 : 1;
    return true;
}
void cpufreqThread(void *, wait_result_t)
{
    char last[sizeof(native_cpufreq_row)] = {};
    uint32_t cores = gMode >= 4 ? 0xf0 : 0;
    PE_parse_boot_argn("pdx203-cores", &cores, sizeof(cores));
    bool coresApplied = cores == 0;
    Interactive inter = {};
    inter.boostUntilMs = 60000;   // the first minute (boot, first unlock) at the top level
    for (unsigned round = 0;; ++round) {
        if (gMode >= 5 && round >= 60) {
            // Between the governor's rounds: 25 ms ticks for the next 5 s.
            for (unsigned t = 0; t < 5000 / kTickMs; ++t) {
                const bool changed = interactiveTick(inter), thermal = (t & 3) == 0;
                if (changed || thermal) {
                    IOLockLock(gLock);
                    if (changed) revoteAll();
                    if (thermal) {readThermal(); thermalGovern();}   // the throttle's 100 ms read
                    IOLockUnlock(gLock);
                }
                // SWAPT <b|i><GPU level> t<ms> <gap>/<done> ...: every finished swap (pdx203_swap_trace), every 100 ms.
                if ((t & 3) == 3 && pdx203_swap_trace) {
                    char trace[320];
                    if (pdx203_swap_trace(trace, sizeof trace))
                        printf("SWAPT %c%u %s\n", gIdle ? 'i' : 'b', &pdx203_gpu_dcvs_stats ? pdx203_gpu_dcvs_stats[0] : 0, trace);
                }
                IOSleep(kTickMs);
            }
            printf("PDX203 CPUBOOST %s, boosted %llu%% of %llus, %llu boosts | touch %u frame %u ticks, %u swaps\n",
                   gIdle ? "idle" : "boosted", inter.ticks ? inter.boostedTicks * 100 / inter.ticks : 0, inter.ticks * kTickMs / 1000,
                   inter.boosts, inter.touchTicks, inter.frameTicks, inter.windowSwaps);
            inter.ticks = inter.boostedTicks = inter.boosts = 0;
            inter.touchTicks = inter.frameTicks = inter.windowSwaps = 0;
        } else if (gMode >= 5) {
            inter.nowMs = (uint64_t)round * 1000;
        }
        sheng_cpufreq_poll();
        if (!coresApplied && (round >= 60 || (gDomain[0].voted >= 0 && gDomain[1].voted >= 0 && gDomain[2].voted >= 0))) {
            coresApplied = true;
            sched_perfcontrol_update_recommended_cores(cores);
            printf("PDX203 CORES recommended %#x (round %u)\n", cores, round);
        }
        if (strncmp(last, native_cpufreq_row, sizeof(last)) != 0) {
            strlcpy(last, native_cpufreq_row, sizeof(last));
            printf("PDX203 %s\n", last);
        }
        // Temperature changes every read, so print it every round (a warmup trace at 1 s for the first minute, then every 30 s) --
        // this is the thermal visibility pass; a throttle governor off native_thermal_max_mc comes once we see how hot it runs.
        printf("PDX203 %s\n", native_thermal_row);
        // 1 s for the first minute (warmup + fast initial throttle response), then every 5 s so the governor still reacts to load
        // within a few seconds rather than the old 30 s.
        if (gMode < 5 || round < 60) IOSleep(round < 60 ? 1000 : 5000);
    }
}
}

extern "C" void pdx203_cpufreq_start(void)
{
    static bool started;
    sheng_cpufreq_probe();
    if (!gMode || __atomic_exchange_n(&started, true, __ATOMIC_ACQ_REL)) return;
    thread_t thread;
    if (kernel_thread_start(cpufreqThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
}
