/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/* weak fallbacks for symbols the base tree references whether or not their archive is linked: the sheng
 * legacy display probe (bsd_init calls it behind offline-legacy-register) and the boot progress poll
 * NativeUfs calls from its root path. a configuration that links the real archive drops these. */
#include <pexpert/pexpert.h>
#include <stdint.h>

__attribute__((weak)) void
native_legacy_display_probe(void)
{
	kprintf("pdx203: native_legacy_display_probe: no display provider on this board yet\n");
}

__attribute__((weak)) void
boot_progress_poll(void)
{
}

/* the legacy display probe (package/LegacyDisplayProbe.cpp, sheng's) imports the boot progress layer weakly, and a static kernel
 * link refuses an undefined weak_import. without that layer: no pulse anchor, no diagnostic writer holding the scanout
 * (prepare answers 1, the "nothing to prepare" result the probe expects), no stage text. from UniXNU-uke LegacyDisplayStubs.c */
__attribute__((weak)) int
boot_progress_pulse_anchor(uintptr_t base, uint64_t bytes, unsigned width, unsigned height, unsigned row_bytes)
{
	(void)base; (void)bytes; (void)width; (void)height; (void)row_bytes;
	return 0;
}

__attribute__((weak)) int
boot_progress_prepare_scanout(void)
{
	return 1;
}

__attribute__((weak)) void
boot_progress_cancel_scanout(void)
{
}

__attribute__((weak)) void
boot_progress_stage(const char *text)
{
	(void)text;
}

/* drivers/QcomXnuUsbDebug imports these two weakly. without the boot progress page the refusal line (which precondition the
 * inherited controller failed: power domain, clocks, reset, role, smmu) goes to the message buffer, which the verbose console
 * shows, so one photograph still names it. there is no board reset to the bootloader yet: refusing makes the module issue
 * its own psci reset. */
extern int printf(const char *, ...);

__attribute__((weak)) void
boot_progress_persist(const char *text, uint32_t slot)
{
	printf("pdx203 persist[%u]: %s\n", slot, text ? text : "");
}

__attribute__((weak)) int
boot_reset_bootloader(char *text, unsigned size)
{
	static const char why[] = "no board reset on pdx203";
	if (text && size) {
		unsigned i = 0;
		for (; i + 1 < size && why[i]; ++i) text[i] = why[i];
		text[i] = 0;
	}
	return 1;
}
