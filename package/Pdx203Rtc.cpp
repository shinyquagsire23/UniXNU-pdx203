// SPDX-License-Identifier: BSD-2-Clause-Patent
// pdx203 time of day: the PM8150 RTC (rtc-pm8xxx.c's pm8941 layout), offered to Apple's platform expert as the IORTC resource.
//
// AppleARMPlatform's AppleARMPE::getGMTTimeOfDay waits up to 30 s for the "IORTC" resource, then calls
// callPlatformFunction("getGMTTimeOfDay", false, clock_sec_t *secs, uint32_t *nsecs, 0, 0) on the object published under it, and
// setGMTTimeOfDay(..., clock_sec_t *secs, uint32_t *nsecs) when the clock is set (kernelcache 21H461, AppleARMPlatform
// 0xfffffff00598fef4 / 0xfffffff00598ffb4). Without it the calendar starts at 0 every boot.
//
// The RTC counts seconds and keeps counting while the phone is off; Android never writes it (Qualcomm's time_daemon keeps its
// own offset in /data), and the apps EE may not own its write channel. So this keeps Android's RTC untouched and stores its own
// 32-bit offset (UTC = counter + offset) in the RTC alarm's data registers (ALARM_RW 0x6140..0x6143), which hold their value
// across power-off like the counter and only matter while the alarm is enabled (ALARM_CTRL 0x6146 bit 7). Used only when this EE
// owns the alarm peripheral's write channel and the alarm is disabled; otherwise the time is read but not kept.
// Boot argument pdx203-rtc=<n>: 0 off, 1 read and publish (default when the argument is absent), 2 read only (no publish).
#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>
#include <libkern/c++/OSSymbol.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>

extern "C" int pdx203_spmi_read(uint32_t ppid, uint8_t addr, uint8_t *value);
extern "C" int pdx203_spmi_write(uint32_t ppid, uint8_t addr, uint8_t value);
extern "C" int pdx203_spmi_writable(uint32_t ppid);
extern "C" void pdx203_rtc_start(void);

// [0] stage (1 thread, 2 counter read, 3 published, 4 calendar set; negative: failed step), [1] counter, [2] offset,
// [3] stored-offset state (0 none, 1 valid, 2 implausible, 3 no storage), [4] RTC write APID, [5] alarm write APID,
// [6] rtc ctrl, [7] alarm ctrl, [8] sets received, [9] offsets written
extern "C" {volatile int32_t pdx203_rtc_stats[10];}

namespace {
constexpr uint32_t kRtcPpid = 0x060, kAlarmPpid = 0x061;   // PM8150 (SID 0) RTC_RW 0x6000, RTC_ALARM 0x6100
constexpr uint8_t kRtcRead = 0x48, kRtcCtrl = 0x46, kAlarmRw = 0x40, kAlarmCtrl = 0x46;
constexpr uint8_t kEnable = 0x80;
// 2024-01-01 .. 2100-01-01: a stored offset that lands outside this is not ours (or was never written).
constexpr uint64_t kSane = 1704067200ull, kSaneEnd = 4102444800ull;

IOLock *gLock;
bool gStorage;            // the alarm registers may hold our offset
bool gHaveOffset;
uint32_t gOffset;         // UTC - counter, mod 2^32
uint32_t gCounter;        // counter at gCounterAt
uint64_t gCounterAt;      // mach_absolute_time of that read
volatile bool gSelfSet;

int read32(uint32_t ppid, uint8_t at, uint32_t *value) {
    uint8_t b[4];
    for (unsigned i = 0; i < 4; ++i) if (int rc = pdx203_spmi_read(ppid, (uint8_t)(at + i), &b[i])) return rc;
    *value = (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24;
    return 0;
}
// The counter, read byte by byte: as rtc-pm8xxx, the low byte is read again and the value re-read if it rolled over meanwhile.
int readCounter(uint32_t *secs) {
    for (unsigned tries = 0; tries < 4; ++tries) {
        uint32_t v = 0;
        uint8_t again = 0;
        if (int rc = read32(kRtcPpid, kRtcRead, &v)) return rc;
        if (int rc = pdx203_spmi_read(kRtcPpid, kRtcRead, &again)) return rc;
        if (again >= (uint8_t)v) {*secs = v; return 0;}
    }
    return 2;
}
bool plausible(uint32_t counter, uint32_t offset) {
    const uint64_t t = (uint64_t)counter + offset;   // offsets are stored mod 2^32: try both wraps
    return (t >= kSane && t < kSaneEnd) || ((t & 0xffffffffull) >= kSane && (t & 0xffffffffull) < kSaneEnd);
}
uint64_t utcFrom(uint32_t counter, uint32_t offset) {
    const uint64_t t = (uint64_t)counter + offset;
    return (t >= kSane && t < kSaneEnd) ? t : (t & 0xffffffffull);
}
// The counter now, from the last read and the time since (the counter has 1 s resolution; this keeps sub-second sets exact).
bool counterNow(uint32_t *secs, uint32_t *nsecs) {
    uint32_t c = 0;
    if (readCounter(&c)) return false;
    const uint64_t now = mach_absolute_time();
    if (c == gCounter && gCounterAt) {
        uint64_t ns = 0;
        absolutetime_to_nanoseconds(now - gCounterAt, &ns);
        *nsecs = ns < 1000000000ull ? (uint32_t)ns : 999999999u;
    } else {
        gCounter = c; gCounterAt = now; *nsecs = 0;
    }
    *secs = c;
    return true;
}
bool storeOffset(uint32_t offset) {
    for (unsigned i = 0; i < 4; ++i)
        if (pdx203_spmi_write(kAlarmPpid, (uint8_t)(kAlarmRw + i), (uint8_t)(offset >> (8 * i)))) return false;
    uint32_t check = 0;
    return read32(kAlarmPpid, kAlarmRw, &check) == 0 && check == offset;
}
}  // namespace

class Pdx203RTC : public IOService {
    OSDeclareDefaultStructors(Pdx203RTC);
public:
    IOReturn callPlatformFunction(const OSSymbol *name, bool wait, void *p1, void *p2, void *p3, void *p4) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(Pdx203RTC, IOService);

IOReturn Pdx203RTC::callPlatformFunction(const OSSymbol *name, bool wait, void *p1, void *p2, void *p3, void *p4) {
    if (name && name->isEqualTo("getGMTTimeOfDay") && p1) {
        IOLockLock(gLock);
        uint32_t c = 0, ns = 0;
        const bool ok = gHaveOffset && counterNow(&c, &ns);
        const uint64_t utc = ok ? utcFrom(c, gOffset) : 0;
        IOLockUnlock(gLock);
        *static_cast<clock_sec_t *>(p1) = (clock_sec_t)utc;
        if (p2) *static_cast<uint32_t *>(p2) = ok ? ns : 0;
        return ok ? kIOReturnSuccess : kIOReturnNotReady;
    }
    if (name && name->isEqualTo("setGMTTimeOfDay") && p1) {
        const clock_sec_t secs = *static_cast<clock_sec_t *>(p1);
        pdx203_rtc_stats[8] = pdx203_rtc_stats[8] + 1;
        if (gSelfSet) return kIOReturnSuccess;   // our own boot-time set: the offset it came from is already stored
        IOLockLock(gLock);
        uint32_t c = 0, ns = 0;
        IOReturn result = kIOReturnNotReady;
        if (counterNow(&c, &ns) && secs >= (clock_sec_t)kSane && secs < (clock_sec_t)kSaneEnd) {
            const uint32_t offset = (uint32_t)((uint64_t)secs - c);
            const bool changed = !gHaveOffset || offset != gOffset;
            gOffset = offset; gHaveOffset = true;
            pdx203_rtc_stats[2] = (int32_t)offset;
            result = kIOReturnSuccess;
            if (changed && gStorage) {
                if (storeOffset(offset)) pdx203_rtc_stats[9] = pdx203_rtc_stats[9] + 1;
                else result = kIOReturnIOError;
            }
            printf("PDX203 RTC set %lu: counter %u offset %u %s\n", (unsigned long)secs, c, offset,
                   !changed ? "unchanged" : gStorage ? (result == kIOReturnSuccess ? "stored" : "store failed") : "not kept (no storage)");
        }
        IOLockUnlock(gLock);
        return result;
    }
    return IOService::callPlatformFunction(name, wait, p1, p2, p3, p4);
}

namespace {
void rtcThread(void *, wait_result_t) {
    thread_set_thread_name(current_thread(), "pdx203-rtc");
    unsigned mode = 1;
    PE_parse_boot_argn("pdx203-rtc", &mode, sizeof mode);
    // The arbiter is mapped by the button thread: wait for it (up to 20 s).
    uint8_t rtcCtrl = 0, alarmCtrl = 0;
    int rc = -1;
    for (unsigned i = 0; i < 400 && (rc = pdx203_spmi_read(kRtcPpid, kRtcCtrl, &rtcCtrl)) == -1; ++i) IOSleep(50);
    if (rc) {pdx203_rtc_stats[0] = -1; pdx203_rtc_stats[1] = rc; printf("PDX203 RTC: no SPMI access (%d)\n", rc); return;}
    pdx203_spmi_read(kAlarmPpid, kAlarmCtrl, &alarmCtrl);
    uint32_t counter = 0, stored = 0;
    rc = readCounter(&counter);
    const int rtcWrite = pdx203_spmi_writable(kRtcPpid), alarmWrite = pdx203_spmi_writable(kAlarmPpid);
    const int alarmRead = read32(kAlarmPpid, kAlarmRw, &stored);
    pdx203_rtc_stats[1] = (int32_t)counter; pdx203_rtc_stats[4] = rtcWrite; pdx203_rtc_stats[5] = alarmWrite;
    pdx203_rtc_stats[6] = rtcCtrl; pdx203_rtc_stats[7] = alarmCtrl;
    if (rc) {pdx203_rtc_stats[0] = -2; printf("PDX203 RTC: counter read failed (%d)\n", rc); return;}
    gCounter = counter; gCounterAt = mach_absolute_time();
    pdx203_rtc_stats[0] = 2;
    gStorage = alarmWrite >= 0 && !alarmRead && !(alarmCtrl & kEnable);
    int state = 3;
    if (gStorage) {
        state = plausible(counter, stored) ? 1 : stored ? 2 : 0;
        if (state == 1) {gOffset = stored; gHaveOffset = true; pdx203_rtc_stats[2] = (int32_t)stored;}
    }
    pdx203_rtc_stats[3] = state;
    static const char *const kState[] = {"none stored", "stored", "implausible", "no storage"};
    printf("PDX203 RTC counter %u ctrl %02x (rtc write apid %d) alarm ctrl %02x data %08x (write apid %d): offset %s%s\n", counter, rtcCtrl,
           rtcWrite, alarmCtrl, stored, alarmWrite, kState[state], gHaveOffset ? "" : " - the clock waits for a network or manual set");
    if (mode != 1) return;
    Pdx203RTC *rtc = OSTypeAlloc(Pdx203RTC);
    if (!rtc || !rtc->init()) {pdx203_rtc_stats[0] = -3; OSSafeReleaseNULL(rtc); return;}
    rtc->setName("pdx203-rtc");
    rtc->registerService();
    IOService::publishResource("IORTC", rtc);   // the resource dictionary holds its own reference; this one stays for the boot
    pdx203_rtc_stats[0] = 3;
    // The calendar was initialised (to 0) before the late probes ran; set it from the RTC now that the resource exists.
    if (gHaveOffset) {
        uint32_t c = 0, ns = 0;
        IOLockLock(gLock);
        const bool ok = counterNow(&c, &ns);
        IOLockUnlock(gLock);
        if (ok) {
            const uint64_t utc = utcFrom(c, gOffset);
            gSelfSet = true;
            clock_set_calendar_microtime((clock_sec_t)utc, ns / 1000);
            gSelfSet = false;
            pdx203_rtc_stats[0] = 4;
            printf("PDX203 RTC calendar set to %llu (counter %u + offset %u)\n", (unsigned long long)utc, c, gOffset);
        }
    }
}
}  // namespace

extern "C" void pdx203_rtc_start(void) {
    if (pdx203_rtc_stats[0]) return;
    gLock = IOLockAlloc();
    if (!gLock) {pdx203_rtc_stats[0] = -9; return;}
    pdx203_rtc_stats[0] = 1;
    thread_t thread = nullptr;
    if (kernel_thread_start(rtcThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
    else pdx203_rtc_stats[0] = -8;
}
