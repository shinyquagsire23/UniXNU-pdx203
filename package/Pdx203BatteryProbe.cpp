// SPDX-License-Identifier: BSD-2-Clause-Patent
// Starts the battery power source (Pdx203Battery.cpp) with boot argument pdx203-battery=1, as ref/UniXNU-sheng's BatteryProbe.cpp:
// it registers that file's classes with the kernel's module (its static initializer, which the build takes out of __mod_init_func),
// then creates the service, which is published once the first reading over SPMI succeeds (the button thread maps the arbiter).
#include <IOKit/IOService.h>
#include <libkern/OSKextLib.h>
#include <mach/kmod.h>
#include <pexpert/pexpert.h>

extern kmod_info_t g_kernel_kmod_info;
extern "C" void pdx203_battery_static_init(void) __asm__("__GLOBAL__sub_I_Pdx203Battery.cpp");
extern "C" void pdx203_battery_start(void);
extern "C" {extern volatile int32_t native_battery_stats[10];}

extern "C" void pdx203_battery_probe(void) {
    unsigned enabled = 0;
    if (!PE_parse_boot_argn("pdx203-battery", &enabled, sizeof enabled) || enabled != 1 || native_battery_stats[0]) return;
    void *handle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if (!handle) {native_battery_stats[0] = -10; return;}
    pdx203_battery_static_init();
    if (OSMetaClass::postModLoad(handle)) {native_battery_stats[0] = -11; return;}
    native_battery_stats[0] = 1;
    pdx203_battery_start();
}
