#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause-Patent
"""pack-pdx203-sensors.py SRC OUT: the sensor DSP's configuration files as one pack for the boot add-on (sensors/sensors.pack), which
package/Pdx203Sensors.cpp serves to the SLPI's sensors domain over FastRPC (as hexagonrpcd does on Linux).

SRC holds the phone's own files, pulled from Android with TWRP (they are this device's, so they stay local and out of every repository):
  vendor-etc-sensors/   /vendor/etc/sensors        (config/*.json, sns_reg_config)
  persist-sensors/      /mnt/vendor/persist/sensors (registry/registry/*, the registry Android generated from the JSONs)
  socinfo/              /sys/devices/soc0          (hw_platform, soc_id, revision, platform_*)

The pack: "SNSPACK1", u32 count, then per file sorted by path: u16 path bytes, u32 size, the canonical absolute path (/vendor/..., /persist/...,
/sys/devices/soc0/...), the bytes."""
import struct
import sys
from pathlib import Path

ROOTS = [("vendor-etc-sensors", "/vendor/etc/sensors", ("config", "sns_reg_config")),
         ("persist-sensors", "/persist/sensors", ("registry",)),
         ("socinfo", "/sys/devices/soc0", None)]


def main():
    # --no-config: serve /vendor/etc/sensors/config empty, so the sensors domain loads the registry as Android left it instead of
    # re-deriving it from the JSONs and writing it back (writes are refused; the SLPI then retries them forever, sn3 2026-11-08).
    args = [a for a in sys.argv[1:] if a != "--no-config"]
    no_config = len(args) != len(sys.argv) - 1
    if len(args) != 2:
        raise SystemExit(__doc__)
    src, out = Path(args[0]), Path(args[1])
    files = {}
    for sub, prefix, keep in ROOTS:
        base = src / sub
        if not base.is_dir():
            raise SystemExit(f"missing {base}")
        for f in sorted(base.rglob("*")):
            if not f.is_file():
                continue
            rel = f.relative_to(base)
            if keep and rel.parts[0] not in keep:
                continue
            if no_config and rel.parts[0] == "config":
                continue
            files[f"{prefix}/{rel.as_posix()}"] = f.read_bytes()
    blob = bytearray(b"SNSPACK1" + struct.pack("<I", len(files)))
    for path in sorted(files):
        p = path.encode()
        blob += struct.pack("<HI", len(p), len(files[path])) + p + files[path]
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(bytes(blob))
    print(f"{out}: {len(files)} files, {len(blob)} bytes")


if __name__ == "__main__":
    main()
