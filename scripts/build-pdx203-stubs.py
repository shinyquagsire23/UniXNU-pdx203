#!/usr/bin/env python3
"""compile the board's small kernel sources with the kernel's own c flags into build/pdx203-stubs/libPdx203Stubs.a.

  Pdx203Stubs.c          weak fallbacks for symbols the base tree and the linked drivers reference (osfmk flags)
  BootWatchdogPolicy.c   sheng's launchd policy writer (disabled.plist on Data before launchd), called from
                         load_init_program by patches/xnu/zzzzzzzzza-launch-policy.patch (bsd flags: it uses the
                         syscall argument structures)

the defconfig names the archive in CONFIG_XNU_EXTRA_LD_ARCHIVES, the base link script appends it as is.
"""
import json
from pathlib import Path
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'build/pdx203-stubs'
OBJ = ROOT / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837'
# (source, the compile record whose flags it takes, extra warnings it is held to)
UNITS = [
    ('Pdx203Stubs.c', 'osfmk/DEVELOPMENT/startup.o.json', ['-Wall', '-Wextra', '-Werror']),
    # carried unchanged from sheng, so it is held to the kernel's own warning set rather than -Wextra
    ('BootWatchdogPolicy.c', 'bsd/DEVELOPMENT/kern_exec.o.json', ['-Werror=implicit-function-declaration']),
]


def compile_unit(name, record_path, warnings):
    record = json.loads((OBJ / record_path).read_text().strip().rstrip(','))
    source = ROOT / 'kits/pdx203' / name
    obj = OUT / (Path(name).stem + '.o')
    command, index = [], 0
    arguments = record['arguments']
    while index < len(arguments):
        value = arguments[index]
        if value == record['file']:
            command.append(str(source))
        elif value in ('-o', '-MF', '-MT', '-MJ'):
            if value == '-o':
                command += ['-o', str(obj)]
            index += 1
        elif value == '-Xclang' and arguments[index + 1].startswith('-W'):
            index += 1
        elif not value.startswith('-W'):
            command.append(value)
        index += 1
    command += warnings
    subprocess.run(command, cwd=record['directory'], check=True)
    magic, cpu, subtype, kind = struct.unpack_from('<4I', obj.read_bytes())
    if (magic, cpu, subtype, kind) != (0xfeedfacf, 0x100000c, 0, 1):
        raise SystemExit('build-pdx203-stubs: %s is not an arm64 object' % obj.name)
    return obj


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    objects = [str(compile_unit(*unit)) for unit in UNITS]
    archive = OUT / 'libPdx203Stubs.a'
    archive.unlink(missing_ok=True)
    subprocess.run(['xcrun', 'ar', 'rcs', str(archive), *objects], check=True)
    print('pdx203 stubs: ' + str(archive))


if __name__ == '__main__':
    main()
