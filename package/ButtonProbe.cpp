// SPDX-License-Identifier: BSD-2-Clause-Patent
// pdx203 (Xperia 1 II) power and volume buttons: UniXNU-alioth package/ButtonProbe.cpp (same SM8250 SPMI arbiter v5 and
// PDC hop) with this board's keys from ButtonsCore.h: power = PM8150 PON kpdpwr, volume up = PON resin, volume down =
// PM8150 GPIO 1 (active low, input with pull-up as Sony's pinctrl sets it; the pad is checked and set up if it is not), and the
// camera key's full press (PM8150B GPIO 1, same pad setup) as the Home button.
// Boot argument pdx203-bt=<n>: 1 map the arbiter and find the APIDs, 2 also read the levels once, 3 the real path (arm the
// interrupts and feed NativeHidInput's report 2: bit 0 home, 1 power, 2 volume up, 3 volume down). Each stage and every change of
// the pressed set prints a BUTTONS row on the console (this board has no BootLog page to export native_buttons_stats).
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOService.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
#include "ButtonsCore.h"

extern "C" int armcpu_spi_attach(uint32_t, bool, void (*)(void *), void *, uint32_t *, uint64_t *) __attribute__((weak_import));

// BootLog.c already composes this row, in sheng's wording:
//   [0] stage (1 mapped, 2 configured, 3 armed, negative: the step that failed)
//   [1] interrupts                    [2] reports sent to the HID device
//   [3] pressed mask (report 2 bits)  [4] PON APID     [5] GPIO APID
//   [6] interrupts of other APIDs cleared     [7] PDC version
//   [8] last error                    [9] spurious interrupts
// Report 2 bits, matching NativeHidInput's buttonsAction (mask & 0xe): 1 power, 2 volume up,
// 3 volume down.
extern "C" {volatile int32_t native_buttons_stats[10];}

namespace {
// The window bases, the PPIDs, the report bits and the channel arithmetic live in ButtonsCore.h,
// where a host test checks them. These aliases keep the driver's own names.
constexpr uint64_t kCore = BT_CORE, kChnls = BT_CHNLS, kObsrvr = BT_OBSRVR, kIntr = BT_INTR,
                   kCnfg = BT_CNFG, kPdc = BT_PDC;
constexpr uint32_t kPonPpid = BT_PON_PPID, kGpioPpid = BT_GPIO_PPID, kCamPpid = BT_CAM_PPID;
constexpr uint8_t kPowerBit = 1u << 0, kResinBit = 1u << 1;
constexpr uint8_t kGpioBit = 1u << 0;
// The report bits are only used through bt_pressed_mask in ButtonsCore.h now; the aliases below
// document which slot of native_buttons_stats[3] each value occupies.
// Peripheral registers, addressed through a channel.
constexpr uint8_t kRtSts = 0x10, kSetType = 0x11, kPolHigh = 0x12, kPolLow = 0x13, kLatchedClr = 0x14, kEnSet = 0x15;
// The status word is decoded by bt_status_result in ButtonsCore.h, not by bit tests here.
// PDC: pin 1, SPI 481, IRQ_i_CFG at 0x110, enable bank at 0x10 (qcom-pdc.c:31-34).
constexpr uint32_t kPdcPin = BT_PDC_PIN, kSpi = BT_SPI, kPdcCfg = 0x110, kPdcEnableBank = 0x10, kPdcVersion = 0x1000;

struct Window {IOMemoryMap *map; volatile uint32_t *va;};
Window gCore, gCnfg, gObs, gChn, gIntr, gPdc;
int gPon = -1, gGpio = -1, gCam = -1;
uint32_t gApids;
uint32_t gVersion;
// The channel page size and the in-channel offsets, chosen from the arbiter's version.
struct bt_layout gLayout;
uint32_t gApidStride, gAccEnable, gIrqStatus, gIrqClear, gApidMap, gObsStride;
volatile uint32_t gEvents;
void (*volatile gSink)(void *, uint32_t);
void *volatile gSinkContext;
unsigned gMode;
bool gPolled;
constexpr unsigned kPollMs = 30;

bool mapWindow(Window &w, uint64_t base, uint64_t size) {
    IODeviceMemory *memory = IODeviceMemory::withRange(base, size);
    w.map = memory ? memory->map(kIOMapAnywhere | kIOMapInhibitCache) : nullptr;
    if (memory) memory->release();
    w.va = w.map ? reinterpret_cast<volatile uint32_t *>(w.map->getVirtualAddress()) : nullptr;
    return w.va != nullptr;
}
void fail(int step, int error) {native_buttons_stats[8] = error; native_buttons_stats[0] = -step; printf("BUTTONS failed at step %d error %d\n", step, error);}

// The channel page for an APID, in whichever window carries the read/write channels.
volatile uint32_t *apidReg(int apid, uint32_t reg) {return gChn.va + (gApidStride * (uint32_t)apid + reg) / 4;}

// One byte of a peripheral through a channel. opc 1 = EXT_READL, 0 = EXT_WRITEL; the command
// word is (opc << 27) | (byte address << 4) | byte count (pmic_arb_fmt_cmd_v2, spmi-pmic-arb.c).
int transfer(int apid, uint8_t addr, uint8_t *value, bool write) {
    volatile uint32_t *ch = gChn.va + (gApidStride * (uint32_t)apid) / 4;
    if (write) ch[0x10 / 4] = *value;                       // WDATA0
    ch[0x00 / 4] = bt_command(write, addr);
    __asm__ volatile("dsb sy" ::: "memory");
    for (uint32_t n = 0; n < 100000; ++n) {
        const uint32_t s = ch[0x08 / 4];                    // STATUS
        const int result = bt_status_result(s);
        if (result == BT_NOT_DONE) continue;
        if (result != BT_OK) return result;                 // failed, denied or dropped
        if (!write) *value = (uint8_t)ch[0x18 / 4];         // RDATA0
        return 0;
    }
    return 1;
}
// pdx203: reads go through the observer channel, the read-only window every APID has (spmi-pmic-arb.c v5 reads through
// rd_base = obsrvr too); only writes need the APID's own channel, which the arbiter refuses to an EE that does not own it.
int observe(int apid, uint8_t addr, uint8_t *value) {
    volatile uint32_t *ch = gObs.va + (gObsStride * (uint32_t)apid) / 4;
    ch[0x00 / 4] = bt_command(false, addr);
    __asm__ volatile("dsb sy" ::: "memory");
    for (uint32_t n = 0; n < 100000; ++n) {
        const int result = bt_status_result(ch[0x08 / 4]);
        if (result == BT_NOT_DONE) continue;
        if (result != BT_OK) return result;
        *value = (uint8_t)ch[0x18 / 4];
        return 0;
    }
    return 1;
}
int read8(int apid, uint8_t addr, uint8_t *value) {return observe(apid, addr, value);}
int write8(int apid, uint8_t addr, uint8_t value) {return transfer(apid, addr, &value, true);}

// The APID that serves ppid, preferring one whose interrupts and writes this EE owns; *owned says whether it does. -1 when
// the arbiter maps no APID to ppid. pdx203: Sony's firmware left the PON without an APID alioth's test accepted (usb-62
// "failed at step 4"), so every match is printed with its IRQ-owner bit and owning EE, and an unowned APID is still used
// for reading (observer channel) with the levels polled.
int findApid(uint32_t ppid, bool *owned) {
    int found = -1;
    *owned = false;
    for (uint32_t i = 0; i < gApids; ++i) {
        const uint32_t map = gCore.va[(gApidMap + 4 * i) / 4];
        if (!map || ((map >> 8) & 0xfff) != ppid) continue;
        const bool irqOwner = (map >> 24) & 1;
        const uint32_t ee = gCnfg.va[(0x00 + 4 * i) / 4] & 7u;
        printf("BUTTONS ppid %03x apid %u map %08x irq-owner %d ee %u\n", ppid, i, map, irqOwner ? 1 : 0, ee);
        const bool mine = irqOwner && ee == 0;
        if (found < 0 || (mine && !*owned)) {found = (int)i; *owned = mine;}
    }
    return found;
}

// Both edges of the given interrupt bits, latched state cleared, then enabled in the peripheral.
int armPeripheral(int apid, uint8_t bits) {
    uint8_t type = 0, high = 0, low = 0;
    int rc = read8(apid, kSetType, &type);
    if (!rc) rc = read8(apid, kPolHigh, &high);
    if (!rc) rc = read8(apid, kPolLow, &low);
    if (!rc) rc = write8(apid, kSetType, type | bits);
    if (!rc) rc = write8(apid, kPolHigh, high | bits);
    if (!rc) rc = write8(apid, kPolLow, low | bits);
    if (!rc) rc = write8(apid, kLatchedClr, bits);
    if (!rc) rc = write8(apid, kEnSet, bits);
    if (!rc) *apidReg(apid, gAccEnable) = 1;
    return rc;
}

// Primary interrupt context: acknowledge in the arbiter and the peripheral, then wake the thread.
// An APID that is not ours but raised the line only gets its arbiter status cleared, as the
// vendor driver's cleanup_irq does; reading another APID's registers can end in a synchronous
// external abort (sheng measured int-227 doing exactly that).
void buttonsIrq(void *) {
    bool ours = false;
    for (uint32_t word = 0; word * 32 < gApids; ++word) {
        uint32_t pending = gIntr.va[word];
        while (pending) {
            const int apid = (int)(word * 32 + __builtin_ctz(pending));
            pending &= pending - 1;
            if (apid != gPon && apid != gGpio && apid != gCam) {
                *apidReg(apid, gIrqClear) = 0xff;
                native_buttons_stats[6] = native_buttons_stats[6] + 1;
                continue;
            }
            const uint32_t status = *apidReg(apid, gIrqStatus);
            *apidReg(apid, gIrqClear) = status;
            if (status) write8(apid, kLatchedClr, (uint8_t)status);
            ours = true;
        }
    }
    __asm__ volatile("dsb sy" ::: "memory");
    // A line that keeps firing with none of our peripherals behind it would hold this core in the
    // handler: after 1000 such interrupts in a row the PDC pin is masked, which drops the line.
    static uint32_t idle;
    if (!ours) {
        native_buttons_stats[9] = native_buttons_stats[9] + 1;
        if (++idle >= 1000) {
            gPdc.va[(kPdcCfg + 4 * kPdcPin) / 4] = 4u;
            __asm__ volatile("dsb sy" ::: "memory");
            native_buttons_stats[0] = -9;
        }
        return;
    }
    idle = 0;
    native_buttons_stats[1] = native_buttons_stats[1] + 1;
    __atomic_fetch_add(&gEvents, 1, __ATOMIC_RELEASE);
    thread_wakeup((event_t)&gEvents);
}

// The pressed set, read from the three levels. All three are active low.
uint32_t pressed() {
    uint8_t pon = 0, gpio = 0;
    const int ponRead = read8(gPon, kRtSts, &pon);
    const int gpioRead = gGpio >= 0 ? read8(gGpio, kRtSts, &gpio) : -1;
    // A failed read must not read as "released" for the PON (which would drop a held key), and
    // must not read as "pressed" for the GPIO (which is active low). bt_pressed_mask takes the
    // GPIO only when it was read, so pass the read result as have_gpio.
    if (ponRead) pon = 0;
    uint8_t cam = 0;
    const int camRead = gCam >= 0 ? read8(gCam, kRtSts, &cam) : -1;
    return bt_pressed_mask(pon, gpio, gpioRead == 0) | bt_camera_mask(cam, camRead == 0);
}

bool setup() {
    if (!mapWindow(gCore, kCore, 0x3000) || !mapWindow(gCnfg, kCnfg, 0x4000) || !mapWindow(gIntr, kIntr, 0x1000) ||
        !mapWindow(gPdc, kPdc, 0x2000)) {fail(1, -12); return false;}
    gVersion = gCore.va[0];
    gApids = gCore.va[0x04 / 4] & 0x7ffu;
    native_buttons_stats[7] = (int32_t)gPdc.va[kPdcVersion / 4];
    // The layout follows the version, as the vendor driver's ops table does. Only the v5 layout
    // is implemented: it keeps every channel register inside the 0x10000 * apid page, which is
    // what this board's 32 MiB chnls window implies. A version this code does not know is
    // refused rather than half-supported (the v2 layout puts the interrupt registers in the
    // "intr" window, which this driver does not read).
    gLayout = bt_layout_for(gVersion);
    if (!gLayout.apid_stride) {fail(2, (int)gVersion); return false;}
    gApidStride = gLayout.apid_stride; gAccEnable = gLayout.acc_enable; gIrqStatus = gLayout.irq_status;
    gIrqClear = gLayout.irq_clear; gApidMap = gLayout.apid_map; gObsStride = gLayout.obs_stride;
    if (!gApids || gApids > 1024) {fail(2, (int)gVersion); return false;}
    if (!mapWindow(gObs, kObsrvr, ((uint64_t)gObsStride * gApids + 0xfff) & ~0xfffull) ||
        !mapWindow(gChn, kChnls, (uint64_t)gApidStride * gApids)) {fail(3, -12); return false;}
    native_buttons_stats[0] = 1;
    printf("BUTTONS arb v%08x apids %u\n", gVersion, gApids);
    bool ponOwned = false, gpioOwned = false, camOwned = false;
    gPon = findApid(kPonPpid, &ponOwned); gGpio = findApid(kGpioPpid, &gpioOwned); gCam = findApid(kCamPpid, &camOwned);
    native_buttons_stats[4] = gPon; native_buttons_stats[5] = gGpio;
    if (gPon < 0) {fail(4, -2); return false;}
    native_buttons_stats[0] = 2;
    // Interrupts only when this EE owns every key's APID it found; otherwise the thread polls the levels.
    gPolled = !ponOwned || (gGpio >= 0 && !gpioOwned) || (gCam >= 0 && !camOwned);
    printf("BUTTONS pon %d%s vol- gpio %d%s cam gpio %d%s -> %s\n", gPon, ponOwned ? "" : " (not ours)", gGpio,
        gpioOwned ? "" : " (not ours)", gCam, camOwned ? "" : " (not ours)", gPolled ? "polling" : "interrupts");
    // The key pads as Sony's pinctrl leaves them: digital input (MODE_CTL 0x40 mode bits 0), 30 uA pull-up (DIG_PULL_CTL
    // 0x42 = 0), enabled (EN_CTL 0x46 bit 7). Read first; set only what differs.
    const int pads[2] = {gGpio, gCam};
    for (unsigned k = 0; k < 2; ++k) {
        const int apid = pads[k];
        if (apid < 0) continue;
        uint8_t mode = 0xff, pull = 0xff, en = 0xff, level = 0xff;
        read8(apid, 0x40, &mode); read8(apid, 0x42, &pull); read8(apid, 0x46, &en); read8(apid, kRtSts, &level);
        printf("BUTTONS pad apid %d mode %02x pull %02x en %02x rt %02x\n", apid, mode, pull, en, level);
        const bool ownPad = (apid == gGpio) ? gpioOwned : camOwned;
        if (gMode >= 3 && ownPad && ((mode & 0x3) != 0 || (pull & 0x7) != 0 || !(en & 0x80))) {
            if (mode & 0x3) write8(apid, 0x40, (uint8_t)(mode & ~0x3));
            if (pull & 0x7) write8(apid, 0x42, (uint8_t)(pull & ~0x7));
            if (!(en & 0x80)) write8(apid, 0x46, 0x80);
            printf("BUTTONS pad apid %d set to input, pull-up, enabled\n", apid);
        }
    }
    if (gMode == 2) {uint8_t pon = 0; read8(gPon, kRtSts, &pon); printf("BUTTONS pon rt %02x mask %x\n", pon, pressed());}
    if (gMode == 2) return true;      // read-only: no arming, no interrupt
    if (gPolled) {native_buttons_stats[0] = 3; printf("BUTTONS polling every %u ms\n", kPollMs); return true;}
    int rc = armPeripheral(gPon, kPowerBit | kResinBit);
    if (!rc && gGpio >= 0) rc = armPeripheral(gGpio, kGpioBit);
    if (!rc && gCam >= 0) rc = armPeripheral(gCam, kGpioBit);
    if (rc) {fail(5, rc); return false;}
    // PDC pin 1: level high, enabled. The version decides which register carries the enable.
    const uint32_t pdcVersion = gPdc.va[kPdcVersion / 4];
    if (pdcVersion >= 0x30200) {
        gPdc.va[(kPdcCfg + 4 * kPdcPin) / 4] = (1u << 3) | 4u;
    } else {
        gPdc.va[(kPdcCfg + 4 * kPdcPin) / 4] = 4u;
        gPdc.va[(kPdcEnableBank + 4 * (kPdcPin / 32)) / 4] |= 1u << (kPdcPin % 32);
    }
    __asm__ volatile("dsb sy" ::: "memory");
    if (!armcpu_spi_attach) {fail(6, -78); return false;}
    // The probe runs before the interrupt controller driver has started, so wait for it, up to a
    // minute, as sheng does (int-228: kIOReturnNotReady at 4 s).
    for (unsigned tries = 0;; ++tries) {
        rc = armcpu_spi_attach(32 + kSpi, false, buttonsIrq, nullptr, nullptr, nullptr);
        if (rc != kIOReturnNotReady || tries == 240) break;
        IOSleep(250);
    }
    if (rc) {fail(6, rc); return false;}
    native_buttons_stats[0] = 3;
    printf("BUTTONS armed: SPI %u, PDC pin %u (version %x)\n", kSpi, kPdcPin, pdcVersion);
    return true;
}

void buttonsThread(void *, wait_result_t) {
    thread_set_thread_name(current_thread(), "pdx203-bt");
    if (!setup()) return;
    uint32_t seen = gEvents, last = 0;
    for (;;) {
        const uint32_t mask = pressed();
        if (mask != last) {
            last = mask;
            native_buttons_stats[3] = (int32_t)mask;
            printf("BUTTONS pressed %x%s%s%s%s\n", mask, (mask & BT_REPORT_HOME) ? " home" : "", (mask & BT_REPORT_POWER) ? " power" : "",
                (mask & BT_REPORT_VOLUME_UP) ? " vol+" : "", (mask & BT_REPORT_VOLUME_DOWN) ? " vol-" : "");
            void (*sink)(void *, uint32_t) = __atomic_load_n(&gSink, __ATOMIC_ACQUIRE);
            if (sink) {sink(gSinkContext, mask); native_buttons_stats[2] = native_buttons_stats[2] + 1;}
        }
        if (gPolled) {IOSleep(kPollMs); continue;}
        assert_wait((event_t)&gEvents, THREAD_UNINT);
        if (gEvents != seen) clear_wait(current_thread(), THREAD_AWAKENED);
        else thread_block(THREAD_CONTINUE_NULL);
        seen = gEvents;
    }
}
}  // namespace

// pdx203: one byte of any PMIC peripheral through the arbiter the button thread mapped, for the battery (Pdx203Battery.cpp).
// ppid = sid << 8 | (peripheral address >> 8). Reads go through the observer channel, so any mapped APID will do. -1 until the
// button thread has mapped the arbiter, -2 when no APID serves ppid; otherwise the transfer's result (0 on success).
extern "C" int pdx203_spmi_read(uint32_t ppid, uint8_t addr, uint8_t *value) {
    if (native_buttons_stats[0] < 2 || !gObs.va || !gCore.va) return -1;
    static struct {uint32_t ppid; int apid;} cache[8];
    static unsigned cached;
    int apid = -2;
    for (unsigned i = 0; i < cached; ++i) if (cache[i].ppid == ppid) apid = cache[i].apid;
    if (apid == -2) {
        for (uint32_t i = 0; i < gApids && apid < 0; ++i) {
            const uint32_t map = gCore.va[(gApidMap + 4 * i) / 4];
            if (map && ((map >> 8) & 0xfff) == ppid) apid = (int)i;
        }
        if (apid < 0) return -2;
        if (cached < 8) {cache[cached].ppid = ppid; cache[cached].apid = apid; cached++;}
    }
    return observe(apid, addr, value);
}

extern "C" void native_buttons_set_sink(void (*sink)(void *, uint32_t), void *context) {
    gSinkContext = context;
    __atomic_store_n(&gSink, sink, __ATOMIC_RELEASE);
}

extern "C" void pdx203_buttons_probe(void) {
    unsigned mode = 0;
    if (!PE_parse_boot_argn("pdx203-bt", &mode, sizeof mode) || !mode || mode > 3 || native_buttons_stats[0]) return;
    gMode = mode;
    native_buttons_stats[0] = 1;
    thread_t thread = nullptr;
    if (kernel_thread_start(buttonsThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
    else fail(7, -12);
}
