// SPDX-License-Identifier: BSD-2-Clause-Patent
// Starts the time of day source (Pdx203Rtc.cpp): registers that file's class with the kernel's module (its static initializer,
// which the build takes out of __mod_init_func), as Pdx203BatteryProbe.cpp does, then the RTC thread. pdx203-rtc=0 turns it off.
#include <IOKit/IOService.h>
#include <libkern/OSKextLib.h>
#include <mach/kmod.h>
#include <pexpert/pexpert.h>

extern kmod_info_t g_kernel_kmod_info;
extern "C" void pdx203_rtc_static_init(void) __asm__("__GLOBAL__sub_I_Pdx203Rtc.cpp");
extern "C" void pdx203_rtc_start(void);
extern "C" {extern volatile int32_t pdx203_rtc_stats[10];}

extern "C" void pdx203_rtc_probe(void) {
    unsigned mode = 1;
    PE_parse_boot_argn("pdx203-rtc", &mode, sizeof mode);
    if (!mode || pdx203_rtc_stats[0]) return;
    void *handle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if (!handle) {pdx203_rtc_stats[0] = -10; return;}
    pdx203_rtc_static_init();
    if (OSMetaClass::postModLoad(handle)) {pdx203_rtc_stats[0] = -11; return;}
    pdx203_rtc_start();
}
