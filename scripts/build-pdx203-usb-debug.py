#!/usr/bin/env python3
"""build drivers/QcomXnuUsbDebug (the CDC-ACM kernel log over the DWC3) into build/qcom-usb-debug/libQcomXnuUsbDebug.a.

the module's own builder compiles its five units with the kernel's pe_kprintf compile record and audits the archive's
imports and exports. it only writes under tmp/ or .cache/, so it builds in .cache/qcom-usb-debug and the archive is
copied to build/, where CONFIG_XNU_EXTRA_LD_ARCHIVES names it. the six hook units of the xnu patches get
-DXNU_QCOM_USB_DEBUG=1 from the base build script when CONFIG_XNU_QCOM_USB_DEBUG is set.
"""
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / 'drivers/QcomXnuUsbDebug'
RECORD = ROOT / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/pexpert/DEVELOPMENT/pe_kprintf.o.json'
WORK = ROOT / '.cache/qcom-usb-debug'
OUT = ROOT / 'build/qcom-usb-debug'


def tool(name):
    found = shutil.which(name)
    if not found:
        raise SystemExit('build-pdx203-usb-debug: %s not on PATH (homebrew llvm)' % name)
    return found


def main():
    if not (MODULE / 'scripts/build.py').is_file():
        raise SystemExit('build-pdx203-usb-debug: %s missing; run git submodule update --init in the device repository' % MODULE)
    # changes to the module ride as patches (patches/modules/QcomXnuUsbDebug/*.patch in the UniXNU fork, which the build tree links, in name order), as
    # NativeMobileDisplay's do (build-pdx203-legacy-display.py): applied as a stack, the applied part being the longest prefix whose last
    # patch reverses. 0001 adds the board input verbs (TOUCH, KEY) of the remote-control rig. One that applies neither way stops the build.
    patches = sorted((ROOT / 'patches/modules/QcomXnuUsbDebug').glob('*.patch'))
    applied = 0
    for k in range(len(patches), 0, -1):
        if subprocess.run(['git', '-C', str(MODULE), 'apply', '--reverse', '--check', str(patches[k - 1].resolve())],
                          capture_output=True).returncode == 0:
            applied = k
            break
    for patch in patches[applied:]:
        subprocess.run(['git', '-C', str(MODULE), 'apply', str(patch.resolve())], check=True)
        print('applied', patch.name, 'to drivers/QcomXnuUsbDebug')
    if not RECORD.is_file():
        raise SystemExit('build-pdx203-usb-debug: %s missing; the kernel bootstrap pass writes it' % RECORD)
    # the sm8250 profile: gcc usb30_prim branches at 0xf010.., legacy gdsc PWR_ON bit, stream 0 behind the apps smmu
    subprocess.run([sys.executable, str(MODULE / 'scripts/build.py'), '--compile-record', str(RECORD),
                    '--soc', 'soc/Sm8250.h', '--output', str(WORK), '--replace',
                    '--ar', tool('llvm-ar'), '--nm', tool('llvm-nm')], check=True, env=os.environ)
    OUT.mkdir(parents=True, exist_ok=True)
    for name in ('libQcomXnuUsbDebug.a', 'build-report.json'):
        shutil.copy2(WORK / name, OUT / name)
    print('pdx203 usb debug: ' + str(OUT / 'libQcomXnuUsbDebug.a'))


if __name__ == '__main__':
    main()
