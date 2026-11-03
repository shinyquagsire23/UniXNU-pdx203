#!/usr/bin/env python3
"""Compile the WiFi module (Ath11kWifi: ath11k for the QCA6390 on PCIe0) to kernel objects for the pdx203 board build.

Usage: build-ath11k-wifi.py [--module DIR] [--tree TREE] [--out DIR] [--prepare] [--boot-settle-ms N]

  --module  the Ath11kWifi checkout; default $ATH11K_WIFI_DIR, else ../Ath11kWifi next to this device repository
  --tree    the resolved build tree (ports/UniXNU/out/pdx203/tree); default: the tree this script runs in
  --out     where the objects go; default TREE/build/native-legacy-display/ath11k-wifi
  --prepare lay out the module's generated trees again even when they look current
  --boot-settle-ms  NW_BOOT_SETTLE_MS of src/nw_mhi.c: the wait before the image is offered over BHI (module default 200, a
            WCN7850 workaround; 0 is worth a try on the QCA6390); $ATH11K_BOOT_SETTLE_MS also sets it

UniXNU-sheng scripts/build-ath12k-wifi.py adapted to ath11k: the module fetches its pinned public sources with
tools/fetch-sources.sh and lays out its generated include trees with tools/prepare.sh (both run only when needed), then every
source of tools/sources.txt is compiled with the kernel's own C code generation line (the osfmk startup.o compile command of the
tree, as board_c() in build-pdx203-legacy-display.py takes it) without the kernel's defines, include paths, warnings and
sanitizers, plus the module's own freestanding flags. Warnings are off (-w -Wno-error) as in sheng's build; any error stops it.
Every global the objects define is renamed to _nwk<name> (llvm-objcopy --redefine-syms) except the board entry points of
include/nw_abi.h. Prints one line "OBJ <path>" per object; build() returns the same list to a caller that imports this file.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
KERNEL_C = 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/osfmk/DEVELOPMENT/startup.o.json'
# the board entry points (include/nw_abi.h): the only globals that keep their names
KEEP = {'_nw_wifi_start', '_nw_wifi_step', '_nw_wifi_stop', '_nw_wifi_stats', '_nw_wifi_set_events', '_nw_wifi_scan', '_nw_wifi_bss_count',
        '_nw_wifi_bss_get', '_nw_wifi_connect', '_nw_wifi_disconnect', '_nw_wifi_tx', '_nw_wifi_status_get', '_nw_wifi_where',
        '_nw_wifi_tx_reasons'}
# what the kernel provides to the objects (the C library functions the driver calls)
KERNEL_PROVIDES = {'_memcpy', '_memmove', '_memset', '_memcmp', '_bzero', '_strlen', '_strcmp', '_strncmp', '_strncpy', '_strchr',
                   '_snprintf', '_vsnprintf', '_strtoul'}


def device_repo():
    """this device repository: the script's own, or (for the copy in a build tree) the one the tree's firmware cache links to"""
    for cand in (HERE.parent, (HERE.parent / '.cache/firmware').resolve().parent.parent):
        if (cand / 'board.toml').exists():
            return cand
    return None


def default_module():
    if os.environ.get('ATH11K_WIFI_DIR'):
        return Path(os.environ['ATH11K_WIFI_DIR'])
    dev = device_repo()
    if not dev:
        raise SystemExit('cannot find the device repository: pass --module or set ATH11K_WIFI_DIR')
    return dev.parent / 'Ath11kWifi'


def default_tree():
    if (HERE.parent / KERNEL_C).exists():
        return HERE.parent
    dev = device_repo()
    cand = dev.parent / 'UniXNU/out/pdx203/tree' if dev else None
    if cand and (cand / KERNEL_C).exists():
        return cand
    raise SystemExit('cannot find the build tree: pass --tree')


def kernel_c_line(tree):
    """the compiler and the code generation flags of the kernel's C line: no source, output, defines, includes, warnings or sanitizers"""
    j = json.loads((tree / KERNEL_C).read_text().strip().rstrip(','))
    args = j['arguments']
    compiler, flags, i = args[0], [], 1
    with_value = {'-o', '-D', '-U', '-I', '-include', '-imacros', '-isystem', '-iquote', '-x', '-MF', '-MT', '-MQ'}
    while i < len(args):
        a = args[i]
        if a == j['file'] or a == '-c' or a in ('-xc', '-nostdlibinc', '-nostdinc') or a == '-g' or a.startswith('-W') or a.startswith('-fsanitize'):
            pass
        elif a in with_value:
            i += 1
        elif a[:2] in ('-D', '-U', '-I') or a.startswith('-include'):
            pass
        elif a == '-Xclang' and i + 1 < len(args) and args[i + 1].startswith('-W'):
            i += 1
        else:
            flags.append(a)
        i += 1
    return compiler, flags, Path(j['directory'])


def tool(name):
    """llvm-nm / llvm-objcopy: Homebrew's llvm (as sheng's build), else whatever is on PATH"""
    try:
        prefix = subprocess.run(['brew', '--prefix', 'llvm'], check=True, capture_output=True, text=True).stdout.strip()
        if (Path(prefix) / 'bin' / name).exists():
            return str(Path(prefix) / 'bin' / name)
    except (OSError, subprocess.CalledProcessError):
        pass
    found = shutil.which(name)
    if not found:
        raise SystemExit(f'{name} not found (brew install llvm)')
    return found


def prepare(module, force=False):
    if not (module / 'deps/linux/drivers/net/wireless/ath/ath11k').is_dir() or not (module / 'deps/freebsd').is_dir():
        subprocess.run(['bash', str(module / 'tools/fetch-sources.sh')], check=True, stdout=subprocess.DEVNULL)
    stamp = module / 'out/ath11k-src/trace.h'
    inputs = [module / 'tools/prepare.sh', module / 'tools/pin-headers.sh', module / 'shim/mhi/trace.h']
    current = (stamp.exists() and (module / 'out/mhi-src/boot.c').exists() and (module / 'out/pinned').is_dir()
               and all(p.stat().st_mtime <= stamp.stat().st_mtime for p in inputs))
    if force or not current:
        subprocess.run(['bash', str(module / 'tools/prepare.sh')], check=True, stdout=subprocess.DEVNULL)


def jobs_of(module, out):
    """(source, object, extra flags) per line of tools/sources.txt, with the include trees of tools/flags.sh"""
    jobs = []
    for line in (module / 'tools/sources.txt').read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        tree, _, name = line.partition(':') if ':' in line else ('', '', line)
        if tree == 'ath11k':
            src, obj, extra = module / 'out/ath11k-src' / name, 'ath11k_' + Path(name).stem, ['-I', str(module / 'out/ath11k-src')]
        elif tree == 'mhi':
            src, obj, extra = module / 'out/mhi-src' / name, 'mhi_' + Path(name).stem, ['-I', str(module / 'out/mhi-src')]
        else:
            src, obj = module / name, Path(name).stem
            extra = (['-I', str(module / 'out/ath11k-src')] if name.startswith('src/') else
                     ['-I', str(module / 'out/mhi-src')] if name.startswith('shim/src/') else [])
        jobs.append((src, out / (obj + '.o'), extra))
    return jobs


def build(module, tree, out, force_prepare=False, boot_settle_ms=None, log=sys.stderr):
    module, tree, out = Path(module).resolve(), Path(tree).resolve(), Path(out).resolve()
    if not (module / 'include/nw_abi.h').exists() or not (module / 'tools/sources.txt').exists():
        raise SystemExit(f'{module}: not an Ath11kWifi checkout')
    prepare(module, force_prepare)
    compiler, kflags, cwd = kernel_c_line(tree)
    resource = subprocess.run([compiler, '-print-resource-dir'], check=True, capture_output=True, text=True).stdout.strip() + '/include'
    flags = [*kflags, '-std=gnu11', '-g0', '-ffreestanding', '-fwrapv', '-fno-delete-null-pointer-checks', '-fno-stack-protector',
             '-mgeneral-regs-only', '-w', '-Wno-error', '-DCONFIG_ATH11K_DEBUG',
             '-nostdinc', '-isystem', str(module / 'shim/libc'), '-isystem', resource, '-include', 'lx_pci.h',
             '-I', str(module / 'shim/include'), '-I', str(module / 'include'), '-I', str(module / 'out/pinned'), '-I', str(module / 'src')]
    if boot_settle_ms is not None:
        flags.append(f'-DNW_BOOT_SETTLE_MS={int(boot_settle_ms)}')
    out.mkdir(parents=True, exist_ok=True)
    for old in out.glob('*.o'):
        old.unlink()
    jobs = jobs_of(module, out)

    def compile_one(job):
        src, obj, extra = job
        r = subprocess.run([compiler, '-c', *flags, *extra, str(src), '-o', str(obj)], capture_output=True, text=True, cwd=cwd)
        if r.returncode:
            log.write(r.stderr)
            raise SystemExit(f'compile failed: {src}')
        return obj

    with ThreadPoolExecutor(os.cpu_count() or 8) as pool:
        objs = list(pool.map(compile_one, jobs))

    # The module's global names (sock_release, kernel_sendmsg, mhi_*, qmi_*, ...) are common in a kernel: every one gets the prefix _nwk
    # except the entry points the board uses.
    nm, objcopy = tool('llvm-nm'), tool('llvm-objcopy')
    defined, undefined = set(), set()
    for o in objs:
        for line in subprocess.run([nm, '-g', str(o)], check=True, capture_output=True, text=True).stdout.splitlines():
            parts = line.split()
            if len(parts) == 3:
                defined.add(parts[2])
            elif len(parts) == 2 and parts[0] == 'U':
                undefined.add(parts[1])
    missing = undefined - defined - KERNEL_PROVIDES
    if missing:
        raise SystemExit('the module needs symbols the kernel does not provide: ' + ' '.join(sorted(missing)))
    absent = KEEP - defined
    if absent:
        raise SystemExit('board entry points missing from the module: ' + ' '.join(sorted(absent)))
    mapping = out / 'rename.txt'
    mapping.write_text(''.join(f'{n} _nwk{n}\n' for n in sorted(defined - KEEP)))
    for o in objs:
        subprocess.run([objcopy, f'--redefine-syms={mapping}', str(o)], check=True)
    log.write(f'ath11k-wifi: {len(objs)} objects in {out}, {len(defined - KEEP)} globals renamed, kernel symbols used: '
              f'{" ".join(sorted(undefined - defined))}\n')
    return objs


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    ap.add_argument('--module', type=Path)
    ap.add_argument('--tree', type=Path)
    ap.add_argument('--out', type=Path)
    ap.add_argument('--prepare', action='store_true')
    ap.add_argument('--boot-settle-ms', type=int, default=os.environ.get('ATH11K_BOOT_SETTLE_MS'))
    a = ap.parse_args()
    tree = a.tree or default_tree()
    for o in build(a.module or default_module(), tree, a.out or tree / 'build/native-legacy-display/ath11k-wifi', a.prepare, a.boot_settle_ms):
        print(f'OBJ {o}')


if __name__ == '__main__':
    main()
