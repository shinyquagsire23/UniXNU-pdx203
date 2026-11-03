#!/usr/bin/env python3
"""Remote control of a running pdx203 over the QcomXnuUsbDebug control channel (collect-xnu-usb.py --control-socket).

  pdx203-rig.py --socket <capture>/usb-control.sock status
  pdx203-rig.py --socket ... shot out.png                  the panel as shown (downscaled by the kernel's pdx203-shot-scale)
  pdx203-rig.py --socket ... tap 0.5 0.3                   x, y as fractions of the panel's width and height (portrait)
  pdx203-rig.py --socket ... swipe 0.5 0.8 0.5 0.2 [300]   from, to, duration in ms
  pdx203-rig.py --socket ... key power                     press and release (power, volup, voldown, home)
  pdx203-rig.py --socket ... bootloader                    restart into fastboot (REBOOT: IMEM + PON reason, warm reset)
  pdx203-rig.py --socket ... flash <boot.img>              bootloader, wait for fastboot, flash boot_a, reboot

Touch and keys go through the kernel's board input hook (package/Pdx203TouchProbe.cpp qcom_usb_debug_board_input): an extra finger
merged into the touch reports, buttons merged into the button reports. Each request is its own control transaction (HELLO, then
the command), as the collector allows one request per connection.
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

MODULE = Path(__file__).resolve().parents[1] / 'drivers/QcomXnuUsbDebug'
sys.path.insert(0, str(MODULE / 'scripts'))
import xnu_usb_control as control  # noqa: E402


def request(sock, verb, args=None, timeout=5.0):
    reply = control.execute_cli_request(sock, verb, timeout, args)
    if reply.status != 'OK':
        raise SystemExit(f'{verb} {args or ""}: {reply.status} {reply.fields}')
    return reply


def point(x, y):
    fx, fy = float(x), float(y)
    if not (0 <= fx <= 1 and 0 <= fy <= 1):
        raise SystemExit('coordinates are fractions of the panel, 0..1')
    return min(9999, int(fx * 10000)), min(9999, int(fy * 10000))


def touch(sock, phase, x=None, y=None):
    request(sock, 'TOUCH', 'u' if phase == 'u' else f'{phase},{x},{y}')


def tap(sock, x, y, hold_ms=60):
    px, py = point(x, y)
    touch(sock, 'd', px, py)
    time.sleep(hold_ms / 1000)
    touch(sock, 'u')


def swipe(sock, x1, y1, x2, y2, ms=300, steps=None):
    (ax, ay), (bx, by) = point(x1, y1), point(x2, y2)
    steps = steps or max(4, int(ms / 16))
    touch(sock, 'd', ax, ay)
    for i in range(1, steps + 1):
        t = i / steps
        touch(sock, 'm', int(ax + (bx - ax) * t), int(ay + (by - ay) * t))
        time.sleep(ms / 1000 / steps)
    touch(sock, 'u')


def key(sock, name, hold_ms=120):
    request(sock, 'KEY', f'{name},d')
    time.sleep(hold_ms / 1000)
    request(sock, 'KEY', f'{name},u')


def fastboot_devices():
    out = subprocess.run(['fastboot', 'devices'], capture_output=True, text=True).stdout
    return [line.split()[0] for line in out.splitlines() if line.strip()]


def flash(sock, image, wait=90):
    image = Path(image)
    if not image.is_file():
        raise SystemExit(f'{image}: no such image')
    if not fastboot_devices():
        try:
            request(sock, 'REBOOT')
        except (control.ControlError, OSError) as exc:
            print(f'REBOOT request failed ({exc}); waiting for fastboot anyway', file=sys.stderr)
    deadline = time.monotonic() + wait
    while not fastboot_devices():
        if time.monotonic() > deadline:
            raise SystemExit('no fastboot device appeared')
        # A reset that lands in recovery (TWRP, with adb) instead: from there adb reboot bootloader reaches fastboot.
        adb = subprocess.run(['adb', 'devices'], capture_output=True, text=True).stdout
        if any(line.split()[1:] == ['recovery'] for line in adb.splitlines()[1:] if line.strip()):
            print('in recovery; adb reboot bootloader', file=sys.stderr)
            subprocess.run(['adb', 'reboot', 'bootloader'], check=False)
            deadline = time.monotonic() + wait
            time.sleep(5)
        time.sleep(1)
    subprocess.run(['fastboot', 'flash', 'boot_a', str(image)], check=True)
    subprocess.run(['fastboot', 'reboot'], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--socket', required=True, type=Path)
    sub = parser.add_subparsers(dest='cmd', required=True)
    sub.add_parser('status')
    p = sub.add_parser('shot'); p.add_argument('output')
    p = sub.add_parser('tap'); p.add_argument('x'); p.add_argument('y')
    p = sub.add_parser('swipe'); [p.add_argument(n) for n in ('x1', 'y1', 'x2', 'y2')]; p.add_argument('ms', nargs='?', type=int, default=300)
    p = sub.add_parser('key'); p.add_argument('name', choices=('power', 'volup', 'voldown', 'home'))
    sub.add_parser('bootloader')
    p = sub.add_parser('flash'); p.add_argument('image')
    a = parser.parse_args()
    if a.cmd == 'status':
        print(json.dumps(request(a.socket, 'STATUS').fields, sort_keys=True))
    elif a.cmd == 'shot':
        subprocess.run([sys.executable, str(MODULE / 'scripts/screenshot.py'), '--socket', str(a.socket), '--output', a.output], check=True)
    elif a.cmd == 'tap':
        tap(a.socket, a.x, a.y)
    elif a.cmd == 'swipe':
        swipe(a.socket, a.x1, a.y1, a.x2, a.y2, a.ms)
    elif a.cmd == 'key':
        key(a.socket, a.name)
    elif a.cmd == 'bootloader':
        print(json.dumps(request(a.socket, 'REBOOT').fields))
    elif a.cmd == 'flash':
        flash(a.socket, a.image)


if __name__ == '__main__':
    main()
