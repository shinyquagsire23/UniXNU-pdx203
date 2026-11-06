// SPDX-License-Identifier: BSD-2-Clause-Patent
// pdx203 panel brightness: DCS 0x51 through DSI0's command FIFO, and the iOS brightness service CoreBrightness drives.
// UniXNU-alioth package/BacklightProbe.cpp (the same SM8250 DSI controller, qcom,dsi-ctrl-hw-v2.4, and a command-mode DSC panel) with
// this board's panel and without its visible test mode. Boot argument pdx203-backlight (default 1; 0 leaves the panel alone).
//
// The panel. Samsung AMB650WH07 on a SOUXP00-A driver IC: single DSI0, four lanes, command mode, DSC. Sony's panel node
// (somc/dsi-panel-souxp00_a-amb650wh07-uhd.dtsi) declares bl_ctrl_dcs with bl-min-level 1 and bl-max-level 4095, and the stock
// XBL's Panel_samsung_oled_singledsi_dsc_4k_amb650wh07_cmd.xml BacklightType 3 (DSI), BacklightBitWidth 12. Its power-on sequence
// (which the boot loader ran) already sends 53 20 (brightness control on), so DCS 0x51 with two bytes, high byte first, is all a
// level needs. An OLED has no separate backlight switch; level 0 is held at the minimum rather than putting the panel to sleep.
//
// How a command reaches the panel. Through the controller's TPG command FIFO, word by word, with no DMA buffer and therefore no
// display SMMU mapping: dsi_ctrl_hw_cmn_kickoff_fifo_command (techpack/display/msm/dsi/dsi_ctrl_hw_cmn.c:703), as alioth and the
// device-verified asteroids port do. A command-mode panel needs no front-porch scheduling; the software trigger starts the transfer
// once the command engine reads idle (STATUS bit 2), and the controller arbitrates it against the DPU's frame transfers.
//
// Packet words (alioth's derivation): DCS long write 0x39 with three bytes, 0xC0390003, then 0x51 | high << 8 | low << 16 | 0xff << 24.
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOService.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOPlatformExpert.h>
#include <libkern/OSKextLib.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSSymbol.h>
#include <mach/kmod.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <mach/mach_time.h>
#include <pexpert/pexpert.h>

// [0] stage (1 mapped, 2 starting, 3 published, 4 published as AppleARMBacklight; negative: the step that failed), [1] brightness
// requests, [2] last level written, [3] unused, [4] transfers that failed, [5] last STATUS/INT_CTRL of a failed
// transfer, [6] DSI_CTRL at start, [7] STATUS at start, [8] HW_VERSION, [9] COMMAND_MODE_DMA_CTRL at start, [10] commands sent,
// [11] last command's time in microseconds, [12] last error
extern "C" {volatile int32_t pdx203_backlight_stats[13];}

namespace {
constexpr uint64_t kDsi0 = 0xae94000, kDsiBytes = 0x400;
constexpr uint32_t kHwVersion = 0x000, kCtrl = 0x004, kStatus = 0x008, kDmaCtrl = 0x03c, kDmaLength = 0x04c,
                   kIntCtrl = 0x110, kTpgCtrl = 0x15c, kTpgInit = 0x17c, kTpgFifoReset = 0x1ec,
                   kSwTrigger = 0x090;
constexpr uint32_t kStatusMdpBusy = 1u << 2, kIntDmaDone = 1u << 0;
constexpr uint32_t kMaxLevel = 4095, kMinLevel = 1, kDefaultLevel = 2048;
constexpr uint32_t kTimeoutUs = 50000, kIdleTimeoutUs = 20000;

IOMemoryMap *gMap;
volatile uint32_t *gDsi;
IOLock *gLock;

uint32_t rd(uint32_t reg) {return gDsi[reg / 4];}
void wr(uint32_t reg, uint32_t value) {gDsi[reg / 4] = value; __asm__ volatile("dsb sy" ::: "memory");}
void fail(int step, int error) {pdx203_backlight_stats[12] = error; pdx203_backlight_stats[0] = -step;}
void bump(int index) {pdx203_backlight_stats[index] = pdx203_backlight_stats[index] + 1;}

// One packet through the TPG FIFO; 0, or -1 when the command engine stayed busy or the DMA-done interrupt never came.
int sendPacket(const uint32_t *words, unsigned count) {
    const uint64_t began = mach_absolute_time();
    bool idle = false;
    for (uint32_t waited = 0; waited < kIdleTimeoutUs; waited += 2) {
        if (!(rd(kStatus) & kStatusMdpBusy)) {idle = true; break;}
        IODelay(2);
    }
    if (!idle) {bump(4); pdx203_backlight_stats[5] = (int32_t)rd(kStatus); return -1;}
    wr(kIntCtrl, (rd(kIntCtrl) & ~((1u << 24) | (1u << 28))) | kIntDmaDone);   // clear a stale done
    wr(kTpgCtrl, (1u << 1) | (1u << 2) | (3u << 16));                          // TPG enable, FIFO mode, custom pattern
    for (unsigned i = 0; i < count; ++i) wr(kTpgInit, words[i]);
    if (count & 1) wr(kTpgInit, 0);                                            // the extra word the hardware wants
    wr(kDmaCtrl, (rd(kDmaCtrl) & ~((1u << 31) | (1u << 30) | (1u << 26))) | (1u << 28));   // embedded mode, high speed
    wr(kDmaLength, count * 4);
    wr(kSwTrigger, 1);
    bool done = false;
    for (uint32_t waited = 0; waited < kTimeoutUs; waited += 20) {
        if (rd(kIntCtrl) & kIntDmaDone) {done = true; break;}
        IODelay(20);
    }
    if (!done) {bump(4); pdx203_backlight_stats[5] = (int32_t)rd(kIntCtrl);}
    wr(kIntCtrl, (rd(kIntCtrl) & ~((1u << 24) | (1u << 28))) | kIntDmaDone);
    wr(kTpgCtrl, 0);
    wr(kTpgFifoReset, 1);
    IODelay(1);
    wr(kTpgFifoReset, 0);
    bump(10);
    uint64_t ns = 0;
    absolutetime_to_nanoseconds(mach_absolute_time() - began, &ns);
    pdx203_backlight_stats[11] = (int32_t)(ns / 1000);
    return done ? 0 : -1;
}

uint32_t clampLevel(uint32_t level) {return level > kMaxLevel ? kMaxLevel : level < kMinLevel ? kMinLevel : level;}

int writeBrightness(uint32_t level) {
    level = clampLevel(level);
    const uint32_t words[2] = {0xC0390003u, 0x51u | ((level >> 8) & 0xffu) << 8 | (level & 0xffu) << 16 | 0xffu << 24};
    const int rc = sendPacket(words, 2);
    if (!rc) pdx203_backlight_stats[2] = (int32_t)level;
    return rc;
}

}  // namespace

// The iOS-facing service (sheng's AppleARMBacklight.cpp, alioth's AliothBacklight): IODisplayParameters brightness min/max/value,
// brightness and FadePeriod (16.16 seconds) through setProperties, a fade stepped every 16 ms on the service's work loop, and the
// AppleARMBacklight conformance CoreBrightness requires (CBDisplayContaineriOS findBacklight; without it backlight-control stops the
// libEDR server from starting and freezes the screen, sheng int-236) and backboardd's sandbox profile checks (int-275/276).
class Pdx203Backlight : public IOService {
    OSDeclareDefaultStructors(Pdx203Backlight)
    IOWorkLoop *loop = nullptr;
    IOTimerEventSource *fade = nullptr, *request = nullptr;
    uint32_t level = 0, from = 0, to = 0;
    uint64_t start = 0, period = 0;
    volatile uint64_t pending = 0;   // bit 63 set: goal << 16 | fade milliseconds not yet taken by the work loop

    static uint64_t now() {return mach_absolute_time();}
    void publish(uint32_t value) {
        OSDictionary *brightness = OSDictionary::withCapacity(3), *parameters = OSDictionary::withCapacity(1);
        OSNumber *min = OSNumber::withNumber((uint64_t)kMinLevel, 32), *max = OSNumber::withNumber((uint64_t)kMaxLevel, 32),
                 *current = OSNumber::withNumber((uint64_t)value, 32);
        if (brightness && parameters && min && max && current) {
            brightness->setObject("min", min); brightness->setObject("max", max); brightness->setObject("value", current);
            parameters->setObject("brightness", brightness);
            setProperty("IODisplayParameters", parameters);
        }
        OSSafeReleaseNULL(min); OSSafeReleaseNULL(max); OSSafeReleaseNULL(current);
        OSSafeReleaseNULL(brightness); OSSafeReleaseNULL(parameters);
    }
public:
    static inline const OSMetaClass *gAppleClass = nullptr;
    static inline bool gImpersonating = false;
private:
    static const OSMetaClass *appleMetaClass(const OSMetaClassBase *) {return gAppleClass;}
    bool impersonate() {
        enum {kSlots = 320, kGetMetaClass = 7};
        const OSSymbol *name = OSSymbol::withCString("AppleARMBacklight");
        gAppleClass = name ? OSMetaClass::getMetaClassWithName(name) : nullptr;
        OSSafeReleaseNULL(name);
        if (!gAppleClass) return false;
#if __has_feature(ptrauth_calls)
        __atomic_store_n(&gImpersonating, true, __ATOMIC_RELEASE);
        return true;
#else
        uintptr_t *copy = static_cast<uintptr_t *>(IOMalloc(kSlots * sizeof(uintptr_t)));
        if (!copy) return false;
        memcpy(copy, *reinterpret_cast<uintptr_t **>(this), kSlots * sizeof(uintptr_t));
        copy[kGetMetaClass] = reinterpret_cast<uintptr_t>(&appleMetaClass);
        __asm__ volatile("dmb ish" ::: "memory");
        *reinterpret_cast<uintptr_t **>(this) = copy;
        return true;
#endif
    }
    void write(uint32_t value) {
        IOLockLock(gLock);
        writeBrightness(value);
        IOLockUnlock(gLock);
        level = value;
    }
    static void fadeStep(OSObject *owner, IOTimerEventSource *sender) {
        Pdx203Backlight *self = static_cast<Pdx203Backlight *>(owner);
        if (!self) return;
        const uint64_t elapsed = now() - self->start;
        uint32_t next = self->to;
        if (elapsed < self->period)
            next = (uint32_t)((int64_t)self->from + ((int64_t)self->to - (int64_t)self->from) * (int64_t)elapsed / (int64_t)self->period);
        if (next != self->level) self->write(next);
        if (next != self->to) sender->setTimeoutMS(16);
    }
    // The newest request, on the work loop (setProperties arms this timer and returns: a runAction there would hold backboardd's
    // brightness thread on the work loop's gate, sheng int-277).
    static void requestFired(OSObject *owner, IOTimerEventSource *) {
        Pdx203Backlight *self = static_cast<Pdx203Backlight *>(owner);
        if (!self) return;
        const uint64_t taken = __atomic_exchange_n(&self->pending, 0, __ATOMIC_ACQ_REL);
        if (!(taken >> 63)) return;
        const uint32_t goal = (uint32_t)(taken >> 16) & 0xffffu, milliseconds = (uint32_t)taken & 0xffffu;
        self->fade->cancelTimeout();
        self->from = self->level;
        self->to = goal;
        self->start = now();
        nanoseconds_to_absolutetime((uint64_t)milliseconds * NSEC_PER_MSEC, &self->period);
        if (!self->period) {self->write(goal); return;}
        fadeStep(self, self->fade);
    }
    static IOReturn begin(OSObject *owner, void *, void *, void *, void *) {
        Pdx203Backlight *self = static_cast<Pdx203Backlight *>(owner);
        if (!self) return kIOReturnBadArgument;
        // The level the boot loader left is not read back (a DCS 0x52 read turns the lanes around, which nothing on this board has
        // done yet); CoreBrightness sets its own level as soon as it finds the service, so the default is only what IODisplayParameters
        // shows until then.
        const int32_t read = -1;
        self->level = kDefaultLevel;
        pdx203_backlight_stats[0] = 2;
        self->publish(self->level);
        const bool impersonated = self->impersonate();
        if (impersonated) {
            self->setProperty("NativeSandboxGrantProcess", "backboardd");
            self->setProperty("backlight-control", true);
        }
        self->registerService();
        pdx203_backlight_stats[0] = impersonated ? 4 : 3;
        printf("PDX203 BACKLIGHT published%s: read back %d, hw %#x ctrl %#x status %#x dma %#x\n", impersonated ? " as AppleARMBacklight" : "",
               read, pdx203_backlight_stats[8], pdx203_backlight_stats[6], pdx203_backlight_stats[7], pdx203_backlight_stats[9]);
        return kIOReturnSuccess;
    }
public:
    bool startService() {
        loop = IOWorkLoop::workLoop();
        fade = IOTimerEventSource::timerEventSource(this, &Pdx203Backlight::fadeStep);
        request = IOTimerEventSource::timerEventSource(this, &Pdx203Backlight::requestFired);
        if (!loop || !fade || !request || loop->addEventSource(fade) || loop->addEventSource(request)) return false;
        loop->runAction(&Pdx203Backlight::begin, this);
        return true;
    }
    IOReturn setProperties(OSObject *properties) override {
        OSDictionary *dictionary = OSDynamicCast(OSDictionary, properties);
        if (!dictionary) return kIOReturnBadArgument;
        OSNumber *brightness = OSDynamicCast(OSNumber, dictionary->getObject("brightness"));
        if (!brightness) return kIOReturnSuccess;
        OSNumber *fadePeriod = OSDynamicCast(OSNumber, dictionary->getObject("FadePeriod"));
        const int32_t requested = (int32_t)brightness->unsigned32BitValue();
        const uint32_t goal = clampLevel(requested < 0 ? 0 : (uint32_t)requested);
        uint64_t milliseconds = fadePeriod ? ((uint64_t)fadePeriod->unsigned32BitValue() * 1000) >> 16 : 0;
        if (milliseconds > 5000) milliseconds = 5000;
        const uint32_t requests = (uint32_t)pdx203_backlight_stats[1] + 1;
        pdx203_backlight_stats[1] = (int32_t)requests;
        if (requests <= 8 || !(requests % 64))
            printf("PDX203 BACKLIGHT request %u: %u over %llu ms (written %d, failed %d, last %d us)\n", requests, goal, milliseconds,
                   pdx203_backlight_stats[2], pdx203_backlight_stats[4], pdx203_backlight_stats[11]);
        __atomic_store_n(&pending, 1ull << 63 | (uint64_t)goal << 16 | milliseconds, __ATOMIC_RELEASE);
        request->setTimeoutUS(1);
        publish(goal);
        return kIOReturnSuccess;
    }
};
#if __has_feature(ptrauth_calls)
// OSDefineMetaClassAndStructors(Pdx203Backlight, IOService) with getMetaClass answering the AppleARMBacklight metaclass once
// impersonate() has run (arm64e signs the vtable, so it cannot be copied).
Pdx203Backlight::MetaClass Pdx203Backlight::gMetaClass;
const OSMetaClass * const Pdx203Backlight::metaClass = &Pdx203Backlight::gMetaClass;
const OSMetaClass * const Pdx203Backlight::superClass = &IOService::gMetaClass;
Pdx203Backlight::Pdx203Backlight(const OSMetaClass *meta) : IOService(meta) {}
Pdx203Backlight::~Pdx203Backlight() {}
const OSMetaClass *Pdx203Backlight::getMetaClass() const
{
    return __atomic_load_n(&gImpersonating, __ATOMIC_ACQUIRE) && gAppleClass ? gAppleClass : &gMetaClass;
}
OSMetaClassConstructorInit(Pdx203Backlight, IOService, )
OSDefineDefaultStructors(Pdx203Backlight, IOService)
#else
OSDefineMetaClassAndStructors(Pdx203Backlight, IOService)
#endif

extern kmod_info_t g_kernel_kmod_info;
// The class's static initializer leaves __mod_init_func (scripts/build-pdx203-legacy-display.py) so it can run here, between
// preModLoad and postModLoad, as the battery's and the RTC's do.
extern "C" void pdx203_backlight_static_init(void) __asm__("__GLOBAL__sub_I_Pdx203Backlight.cpp");

extern "C" void pdx203_backlight_probe(void) {
    unsigned mode = 1;
    PE_parse_boot_argn("pdx203-backlight", &mode, sizeof mode);
    if (!mode || pdx203_backlight_stats[0]) return;
    gLock = IOLockAlloc();
    IODeviceMemory *memory = IODeviceMemory::withRange(kDsi0, kDsiBytes);
    gMap = memory ? memory->map(kIOMapAnywhere | kIOMapInhibitCache) : nullptr;
    if (memory) memory->release();
    gDsi = gMap ? reinterpret_cast<volatile uint32_t *>(gMap->getVirtualAddress()) : nullptr;
    if (!gLock || !gDsi) {gDsi = nullptr; fail(1, -12); return;}
    pdx203_backlight_stats[8] = (int32_t)rd(kHwVersion);
    pdx203_backlight_stats[6] = (int32_t)rd(kCtrl);
    pdx203_backlight_stats[7] = (int32_t)rd(kStatus);
    pdx203_backlight_stats[9] = (int32_t)rd(kDmaCtrl);
    pdx203_backlight_stats[0] = 1;
    // DSI_CTRL bit 0 (DSI_EN) and bit 2 (command engine) are what the boot loader's display left on; anything else is not this panel.
    if ((pdx203_backlight_stats[6] & 0x5) != 0x5) {fail(2, pdx203_backlight_stats[6]); printf("PDX203 BACKLIGHT DSI0 not in command mode: ctrl %#x\n", pdx203_backlight_stats[6]); return;}
    void *handle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if (!handle) {fail(5, -12); return;}
    pdx203_backlight_static_init();
    const OSReturn status = OSMetaClass::postModLoad(handle);
    if (status) {fail(5, status); return;}
    Pdx203Backlight *backlight = OSTypeAlloc(Pdx203Backlight);
    if (!backlight || !backlight->init() || !backlight->attach(IOService::getPlatform()) || !backlight->startService()) {
        if (backlight) backlight->release();
        fail(6, -19);
        return;
    }
    backlight->release();
}
