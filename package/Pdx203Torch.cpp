// SPDX-License-Identifier: BSD-2-Clause-Patent
// pdx203 flashlight: the PM8150L flash LED module (SID 5, peripheral 0xd300) in torch mode.
//
// The rear flash LED is wired to channels 1 and 2 of the PM8150L's 3-channel flash module (Sony kona-edo-common.dtsi: torch0/torch1,
// qcom,id 0 and 1, 120 mA default and 500 mA maximum each; channel 3 unused). The register map and the torch sequence are mainline
// Linux drivers/leds/flash/leds-qcom-flash.c (mvflash_3ch_regs, qcom_flash_led_brightness_set): per-channel ITARGET 0x43+ch in 5 mA
// steps when the channel's IRESOLUTION field (0x47, 2 bits per channel) is 3, a per-channel safety timer 0x40+ch (0 = off for torch),
// software strobe 0x49+ch, the channel enables 0x4c and the module enable 0x46 bit 7. Linux writes no other register for torch.
//
// iOS drives it through pdx203_torch_set (level 0..1000): the board's text fixups turn the camera daemon's FigFlashlight device call
// (flashlight_setLevelInternal's TorchLevel property on a capture stream this board does not have) into the pdx203_torch syscall
// (zzzzzzzzzt-pdx203-torch-syscall.patch), and /product/camera "flash" (board.toml) makes MobileGestalt's camera-flash true, so
// Control Center offers the flashlight. The rig's knob "torch" sets the same level.
#include <IOKit/IOLib.h>
#include <libkern/OSAtomic.h>
#include <kern/clock.h>

extern "C" int pdx203_spmi_read(uint32_t ppid, uint8_t addr, uint8_t *value);
extern "C" int pdx203_spmi_write(uint32_t ppid, uint8_t addr, uint8_t value);
extern "C" int pdx203_spmi_writable(uint32_t ppid);

// [0] state (0 never set, 1 off, 2 on; negative: the failed step), [1] level asked (0..1000), [2] mA per channel, [3] sets, [4] failures,
// [5] last SPMI result, [6] TYPE, [7] SUBTYPE, [8] STATUS1..3 packed at the last change, [9] write APID
extern "C" {volatile int32_t pdx203_torch_stats[10];}

namespace {
constexpr uint32_t kPpid = 0x5d3;                 // SID 5, 0xd300 (pm8150l.dtsi pm8150l_flash led-controller@d300)
enum : uint8_t {
    kType = 0x04, kSubtype = 0x05, kStatus1 = 0x08, kStatus2 = 0x09, kStatus3 = 0x0a,
    kChanTimer = 0x40, kItarget = 0x43, kModuleEn = 0x46, kIres = 0x47, kChanStrobe = 0x49, kChanEn = 0x4c,
};
constexpr uint8_t kTypeFlash = 0x18, kSubtype3ch = 0x04;
constexpr unsigned kChannels = 2;                 // channels 1 and 2 (chan_id 0, 1)
constexpr uint8_t kChanMask = 0x03;
constexpr unsigned kMaxMaPerChannel = 160;        // level 1000; Sony's torch default is 120 mA, its maximum 500 mA
constexpr uint8_t kStrobeSwLevelHigh = 0x01;      // SW strobe (bit 2 = 0), level trigger (bit 1 = 0), active high (bit 0 = 1)
constexpr uint8_t kIres5mA = 0x03;                // 3CH: 5 mA steps

IOLock *volatile gLock;
bool gChecked;

int rmw(uint8_t addr, uint8_t mask, uint8_t value) {
    uint8_t old = 0;
    int rc = pdx203_spmi_read(kPpid, addr, &old);
    if (!rc) rc = pdx203_spmi_write(kPpid, addr, (uint8_t)((old & ~mask) | (value & mask)));
    return rc;
}

// The peripheral must be the 3-channel module this map is for, and this EE must own its write channel.
int check() {
    uint8_t type = 0, subtype = 0;
    int rc = pdx203_spmi_read(kPpid, kType, &type);
    if (!rc) rc = pdx203_spmi_read(kPpid, kSubtype, &subtype);
    pdx203_torch_stats[6] = type; pdx203_torch_stats[7] = subtype;
    if (rc) return rc;
    const int apid = pdx203_spmi_writable(kPpid);
    pdx203_torch_stats[9] = apid;
    printf("PDX203 TORCH flash module type %02x subtype %02x write apid %d\n", type, subtype, apid);
    if (type != kTypeFlash || subtype != kSubtype3ch) return -10;
    if (apid < 0) return apid == -2 ? -1 : -3;
    return 0;
}

int apply(unsigned ma) {
    int rc = 0;
    // off first, as Linux does on every change: strobe config, channels off, module off
    for (unsigned ch = 0; ch < kChannels && !rc; ++ch) rc = rmw((uint8_t)(kChanStrobe + ch), 0x07, kStrobeSwLevelHigh);
    if (!rc) rc = rmw(kChanEn, kChanMask, 0);
    if (!rc) rc = rmw(kModuleEn, 0x80, 0);
    if (rc || !ma) return rc;
    // current: ITARGET = mA / 5 - 1 per channel, 5 mA resolution, no safety timer
    const uint8_t itarget = (uint8_t)(ma / 5 - 1);
    for (unsigned ch = 0; ch < kChannels && !rc; ++ch) rc = rmw((uint8_t)(kItarget + ch), 0x7f, itarget);
    if (!rc) rc = rmw(kIres, 0x0f, (uint8_t)(kIres5mA | kIres5mA << 2));
    for (unsigned ch = 0; ch < kChannels && !rc; ++ch) rc = pdx203_spmi_write(kPpid, (uint8_t)(kChanTimer + ch), 0);
    if (!rc) rc = rmw(kModuleEn, 0x80, 0x80);
    if (!rc) rc = rmw(kChanEn, kChanMask, kChanMask);
    return rc;
}
}  // namespace

// level 0 (off) .. 1000 (full); returns 0, an SPMI result, or -1 before the arbiter is mapped, -3 when this EE cannot write the module,
// -10 when the peripheral is not the expected flash module.
extern "C" int pdx203_torch_set(int level) {
    if (level < 0) level = 0;
    if (level > 1000) level = 1000;
    if (!gLock) {
        IOLock *lock = IOLockAlloc();
        if (!OSCompareAndSwapPtr(nullptr, lock, (void *volatile *)&gLock)) IOLockFree(lock);
    }
    IOLockLock(gLock);
    int rc = 0;
    if (!gChecked) {
        rc = check();
        if (!rc) gChecked = true;
    }
    unsigned ma = 0;
    if (!rc) {
        ma = level ? (unsigned)level * kMaxMaPerChannel / 1000 : 0;
        if (level && ma < 10) ma = 10;
        ma -= ma % 5;
        rc = apply(ma);
    }
    uint8_t s1 = 0, s2 = 0, s3 = 0;
    if (gChecked) {pdx203_spmi_read(kPpid, kStatus1, &s1); pdx203_spmi_read(kPpid, kStatus2, &s2); pdx203_spmi_read(kPpid, kStatus3, &s3);}
    pdx203_torch_stats[1] = level; pdx203_torch_stats[2] = (int32_t)ma; pdx203_torch_stats[3] = pdx203_torch_stats[3] + 1;
    pdx203_torch_stats[5] = rc; pdx203_torch_stats[8] = s1 | s2 << 8 | s3 << 16;
    if (rc) {pdx203_torch_stats[4] = pdx203_torch_stats[4] + 1; pdx203_torch_stats[0] = -1;}
    else pdx203_torch_stats[0] = ma ? 2 : 1;
    IOLockUnlock(gLock);
    printf("PDX203 TORCH level %d -> %u mA x %u rc %d status %02x %02x %02x\n", level, ma, kChannels, rc, s1, s2, s3);
    return rc;
}
