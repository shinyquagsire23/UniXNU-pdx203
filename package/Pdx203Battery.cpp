// SPDX-License-Identifier: BSD-2-Clause-Patent
// pdx203 battery as iOS reads it (boot argument pdx203-battery=1): an IOPMPowerSource, as ref/UniXNU-sheng's ShengBattery.cpp
// publishes one, whose values come straight from the PM8150B over SPMI (package/ButtonProbe.cpp's pdx203_spmi_read). SM8250 has no
// battery manager in the audio DSP (sheng's source); Linux's qpnp-fg-gen4 and qpnp-smb5 read the same registers:
//   charge     FG BATT_SOC 0x4000 (sid 2, ppid 0x240) + 0x09 MONOTONIC_SOC, 0..255; percent as fg_get_prop_capacity:
//              0 -> 0, 255 -> 100, else (msoc - 1) * 98 / 253 + 1, rounded (fg-reg.h, qpnp-fg-gen4.c)
//   charging   SMB5 CHGR 0x1000 (ppid 0x210) + 0x06 BATTERY_CHARGER_STATUS_1, bits 2:0: 0 inhibit, 1 trickle, 2 pre, 3 full-on,
//              4 taper, 5 terminate, 6 pause, 7 disabled (smb5-reg.h); 1..4 charging, 5 full
//   external   USBIN 0x1300 (ppid 0x213) + 0x10 USB_INT_RT_STS bit 4 USBIN_PLUGIN_RT_STS
// No temperature: gen4 reads the battery thermistor through the PMIC's ADC5 channel, not a gauge register (BATT_INFO 0x4150/51
// read 0 on the phone, usb-64: -273 C), so none is published until that channel is driven.
// Charging itself stays with the PMIC's own charger as the boot loader left it; nothing is written. powerd finds every
// IOPMPowerSource, reads CurrentCapacity/MaxCapacity, IsCharging, ExternalConnected and listens for updateStatus; the status bar,
// the lock screen's charging state and Settings > Battery follow. The voltage is not a register on this gauge (gen4 keeps it in its
// SRAM), so a nominal value is reported. A BATTERY row on the console for every change.
#include <IOKit/IOLib.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/pwr_mgt/IOPM.h>
#include <IOKit/pwr_mgt/IOPMPowerSource.h>
#include <libkern/c++/OSBoolean.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSSymbol.h>

extern "C" int pdx203_spmi_read(uint32_t ppid, uint8_t addr, uint8_t *value);
// [0] stage (1 class up, 2 timer running, 3 published; negative: the step that failed), [1] percent, [2] raw monotonic SOC,
// [3] charger status, [4] USB present, [5] unused, [6] reads, [7] read failures, [8] last failure
extern "C" {volatile int32_t native_battery_stats[10];}

namespace {
constexpr uint32_t kFastMs = 2000, kRefreshMs = 20000, kNominalMv = 3850;
constexpr uint32_t kSocPpid = 0x240, kChgrPpid = 0x210, kUsbPpid = 0x213;
}

class Pdx203Battery : public IOPMPowerSource {
    OSDeclareDefaultStructors(Pdx203Battery)
    IOWorkLoop *loop = nullptr;
    IOTimerEventSource *timer = nullptr;
    bool published = false;
    int lastPercent = -1, lastStatus = -1, lastUsb = -1;

    void number(const char *key, int value) {
        const OSSymbol *symbol = OSSymbol::withCString(key);
        OSNumber *n = OSNumber::withNumber((unsigned long long)(long long)value, 32);
        if (symbol && n) setPSProperty(symbol, n);
        OSSafeReleaseNULL(n); OSSafeReleaseNULL(symbol);
    }
    void flag(const char *key, bool value) {
        const OSSymbol *symbol = OSSymbol::withCString(key);
        if (symbol) setPSProperty(symbol, value ? kOSBooleanTrue : kOSBooleanFalse);
        OSSafeReleaseNULL(symbol);
    }
    bool read(uint32_t ppid, uint8_t addr, uint8_t *value) {
        const int rc = pdx203_spmi_read(ppid, addr, value);
        native_battery_stats[6] = native_battery_stats[6] + 1;
        if (rc) {native_battery_stats[7] = native_battery_stats[7] + 1; native_battery_stats[8] = rc;}
        return rc == 0;
    }
    // One reading; false when the charge could not be read (the arbiter is not mapped yet, or the read failed).
    bool refresh() {
        uint8_t msoc = 0, status = 0, usb = 0;
        if (!read(kSocPpid, 0x09, &msoc)) return false;
        const bool haveStatus = read(kChgrPpid, 0x06, &status);
        const bool haveUsb = read(kUsbPpid, 0x10, &usb);
        const int percent = msoc == 0 ? 0 : msoc == 255 ? 100 : (int)(((msoc - 1) * 98 + 126) / 253) + 1;
        const int charger = haveStatus ? (status & 7) : -1;
        const bool external = haveUsb && (usb & (1u << 4));
        const bool charging = external && charger >= 1 && charger <= 4;
        const bool full = external && (charger == 5 || percent >= 100);
        native_battery_stats[1] = percent; native_battery_stats[2] = msoc; native_battery_stats[3] = charger;
        native_battery_stats[4] = external;
        setBatteryInstalled(true);
        setExternalConnected(external);
        setExternalChargeCapable(external);
        setIsCharging(charging);
        setAtCriticalLevel(percent <= 2);
        setCurrentCapacity((unsigned)percent);
        setMaxCapacity(100);
        setVoltage(kNominalMv);
        setAmperage(0);
        setTimeRemaining(0xffff);
        flag("FullyCharged", full);
        if (percent != lastPercent || charger != lastStatus || (int)external != lastUsb) {
            printf("BATTERY %d%% (msoc %u) charger %d%s usb %d\n", percent, msoc, charger, charging ? " charging" : full ? " full" : "",
                external ? 1 : 0);
            lastPercent = percent; lastStatus = charger; lastUsb = external;
        }
        return true;
    }
    static void timerAction(OSObject *owner, IOTimerEventSource *sender) {
        Pdx203Battery *self = OSDynamicCast(Pdx203Battery, owner);
        if (!self) return;
        if (!self->refresh()) {sender->setTimeoutMS(kFastMs); return;}
        if (!self->published) {
            // The properties are in place before anyone finds the service.
            self->published = true;
            self->registerService();
            native_battery_stats[0] = 3;
        }
        self->updateStatus();
        sender->setTimeoutMS(kRefreshMs);
    }

public:
    bool startBattery() {
        loop = IOWorkLoop::workLoop();
        timer = IOTimerEventSource::timerEventSource(this, &Pdx203Battery::timerAction);
        if (!loop || !timer || loop->addEventSource(timer) != kIOReturnSuccess) return false;
        native_battery_stats[0] = 2;
        timer->setTimeoutMS(kFastMs);
        return true;
    }
};
OSDefineMetaClassAndStructors(Pdx203Battery, IOPMPowerSource)

// Called by Pdx203BatteryProbe.cpp once this file's classes are registered.
extern "C" void pdx203_battery_start(void) {
    Pdx203Battery *battery = OSTypeAlloc(Pdx203Battery);
    if (!battery || !battery->init() || !battery->attach(IOService::getPlatform())) {
        if (battery) battery->release();
        native_battery_stats[0] = -1;
        return;
    }
    if (!battery->startBattery()) {
        native_battery_stats[0] = -2;
        battery->detach(IOService::getPlatform());
        battery->release();
        return;
    }
    battery->release();  // the platform's attachment keeps it
}
