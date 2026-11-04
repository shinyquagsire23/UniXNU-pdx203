#!/usr/bin/env python3
"""Build the unified-log forwarder (userland/OsLogConsole.mm) for the GPU addon.

  build-oslog-console.py OUTDIR

OUTDIR gets IOMFB_FDR_Loader (the forwarder, under the name of the boot job it replaces), filter.txt (the default process list) and
trust.json ({"cdhash": ...}, for make-owner-trust-bundle.py --extra-trust). make-gpu-addon.py --oslog OUTDIR carries the binary as
usrbin/IOMFB_FDR_Loader (overlaid on /usr/bin by patch zzy-zoslog-addon-mount) and the filter as oslog/filter.txt."""
import json
import plistlib
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=True)
sdk = subprocess.check_output(['xcrun', '--sdk', 'iphoneos', '--show-sdk-path'], text=True).strip()
exe = out / 'IOMFB_FDR_Loader'
subprocess.run(['xcrun', 'clang++', '-target', 'arm64-apple-ios17.0', '-isysroot', sdk, '-O2', '-Wall', '-Wextra', '-Werror', '-std=c++17',
                '-x', 'objective-c++', '-fobjc-arc', str(ROOT / 'userland/OsLogConsole.mm'), '-framework', 'Foundation', '-framework', 'OSLog', '-framework', 'MobileCoreServices', '-o', str(exe)],
               check=True)
# The logging grants adrenoprobed carries for its OSLogStore export (UniXNU scripts/build-adreno-metal-plugin.py), outside every container.
entitlements = out / 'entitlements.plist'
entitlements.write_bytes(plistlib.dumps({'com.apple.private.security.no-container': True,
                                         # CPUPC profiles (OsLogConsole.mm profile): another process's task read port and thread states
                                         'task_for_pid-allow': True, 'com.apple.system-task-ports': True, 'com.apple.system-task-ports.read': True,
                                         'com.apple.private.thread-set-state': True,
                                         # CPUK* profiles (OsLogConsole.mm kperfProfile): kperf and kdebug without root
                                         'com.apple.private.ktrace-allow': True, 'com.apple.logging.local-store': True,
                                         'com.apple.lsapplicationworkspace.rebuildappdatabases': True,
                                         'com.apple.private.logging.admin': True,
                                         'com.apple.private.logging.diagnostic': True, 'com.apple.private.logging.stream': True,
                                         # SystemAppMigrator run in-process (installStagedSystemApps): com.apple.datamigrator's install grants
                                         'com.apple.private.InstallCoordination.allowed': True,
                                         'com.apple.private.MobileContainerManager.allowed': True,
                                         'com.apple.private.amfi.can-check-trust-cache': True, 'com.apple.appstored.private': True,
                                         'com.apple.private.mobileinstall.allowedSPI': [
                                             'DataMigratorUpdateDataProtection', 'InstallForDataMigrator', 'InstallForLaunchServices',
                                             'RegisterSafeHarbor', 'ProcessRestoredContainer', 'UninstallForLaunchServices',
                                             'LookupSystemAppState', 'UpdateSystemAppState', 'SetSystemAppMigrationComplete']}))
# The identifier of the program it stands in for, in case the launchd job's constraints name it.
subprocess.run(['codesign', '--force', '--sign', '-', '--pagesize', '4096', '--identifier', 'com.apple.IOMFB_FDR_Loader',
                '--entitlements', str(entitlements), str(exe)], check=True)
subprocess.run(['codesign', '--verify', '--strict', str(exe)], check=True)
detail = subprocess.run(['codesign', '--display', '--verbose=4', str(exe)], capture_output=True, text=True, check=True).stderr
cdhash = re.search(r'^CDHash=([a-f0-9]{40})$', detail, re.M)[1]
assert re.search(r'^Page size=4096$', detail, re.M), 'the signature must use 4 KiB pages'
(out / 'trust.json').write_text(json.dumps({'cdhash': cdhash}) + '\n')
filter_txt = out / 'filter.txt'
if not filter_txt.exists():
    filter_txt.write_text('# OsLogConsole: processes whose unified-log entries go to the USB console (see userland/OsLogConsole.mm)\n'
                          'level 3\nrate 60\nlocationd\nbackboardd\nSpringBoard\nMobileSafari\ncom.apple.WebKit.*\nkbd\n')
print('built', exe, 'cdhash', cdhash)
