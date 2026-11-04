#!/usr/bin/env python3
"""Run the pdx203 board archive builders the selected configuration links (format 2).

board.toml lists the catalog components (ArmDTPlatform, ArmCpu, SoftwareAES, ShengUfs, RamBlock, ...) whose recipes
the unixnu driver runs first, then this one script in [kernel] archives. It reads the defconfig's XNU_* flags from its
environment and runs the board's own builders for what that configuration links, in sheng's order. A builder that
fails stops the kernel step, as a listed one would.

Two builders bind owned drivers to original kernelcache classes and read a prelinked candidate of the pinned
kernelcache from build/native-prelink/ (docs/bring-up.md, the pre_build input). The candidate's original addresses
come from the unchanged 21H461 kernelcache, so a candidate of an earlier pdx203 build is a valid input.
"""
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
CANDIDATE = 'build/native-prelink/native-defaults-candidate.macho'


def on(name):
    return os.environ.get('XNU_' + name, '0') == '1'


def main():
    steps = [
        ['build-corecrypto-14.py'],
        ['build-corecrypto-14.py', '--namespace'],
    ]
    # the weak fallbacks: the base bsd_init references native_legacy_display_probe() whatever the display flag says, and the
    # display probe imports the boot progress layer weakly. the defconfig links the archive through CONFIG_XNU_EXTRA_LD_ARCHIVES
    steps.append(['build-pdx203-stubs.py'])
    if on('NATIVE_AES'):
        steps.append(['build-software-aes-bindings.py'])
    if on('NATIVE_UFS'):
        # the bring-up transport (ShengUfs recipe, unverified under XNU_BRINGUP_OWNER_CACHE) bound to the original
        # IOBlockStorageDevice of the prelinked candidate (docs/bring-up.md, stage 6)
        steps.append(['build-ufs-original-bindings.py', '--candidate', CANDIDATE])
    if on('NATIVE_RAM_BLOCK'):
        # the Data seed's IOMedia (RamBlock recipe), bound like the UFS transport
        steps.append(['build-ufs-original-bindings.py', '--component', 'NativeRamBlock', '--candidate', CANDIDATE])
    if on('SOURCE_MOCKFS'):
        steps.append(['build-mockfs-fixture.py'])
    if on('SHENG_PMGR'):
        steps.append(['build-pdx203-platform-power.py'])
    if on('NATIVE_LEGACY_DISPLAY'):
        # the GOP display (NativeLegacyDisplay bound to the original IOMobileFramebufferLegacy, the board probe and
        # NativeGOPPresenter) with the touch, HID, buttons and keystore probes, linked by the base build as
        # build/native-legacy-display/libNativeLegacyDisplay.late.a
        steps.append(['build-pdx203-legacy-display.py'])
    if on('NATIVE_ADRENO'):
        # the Adreno 650: drivers/Adreno650 (FFT-infra bdee54c and this repository's patch stack), the kits, the board probe and
        # its firmware, linked by the base build as build/adreno740/libAdreno740.a
        steps.append(['build-pdx203-adreno.py'])
    if on('QCOM_USB_DEBUG'):
        # the CDC-ACM kernel log over the DWC3 (drivers/QcomXnuUsbDebug, sm8250 profile), linked through
        # CONFIG_XNU_EXTRA_LD_ARCHIVES as build/qcom-usb-debug/libQcomXnuUsbDebug.a
        steps.append(['build-pdx203-usb-debug.py'])
    if on('SOURCE_PTHREAD'):
        steps.append(['build-libpthread-kernel.py'])
    for index, step in enumerate(steps, 1):
        script = ROOT / 'scripts' / step[0]
        if not script.is_file():
            raise SystemExit('build-pdx203-archives: %s missing from the build tree' % script)
        print('pdx203 archive %d/%d: %s' % (index, len(steps), ' '.join(step)), flush=True)
        subprocess.run([sys.executable, str(script), *step[1:]], cwd=ROOT, check=True)


if __name__ == '__main__':
    main()
