// SPDX-License-Identifier: BSD-2-Clause-Patent
// pdx203 WiFi, stages 1-3: the QCA6390 on PCIe0 (sheng's WifiProbe.cpp stages 1-4 with the SM8250 values of /tmp/kmdconn/wifi-pcie-spec.md:
// Sony's kona DT, LineageOS android_kernel_sony_sm8250 cnss2/pci-msm, upstream gcc-sm8250.c, phy-qcom-qmp-pcie.c, pcie-qcom.c).
// Boot argument pdx203-wifi=<n>:
//   1  read-only: GDSC, clock votes and halts, the pins
//   2  power and link: the chip's rails over RPMh (through AdspProbe's TCS 1 helper; needs pdx203-adsp), CX at NOM for Gen3, WLAN_EN,
//      GDSC, clocks, resets, PARF, the QMP PHY (the DT's 99-write sequence), PERST#, LTSSM, link up, the endpoint's ID (17cb:1101)
//   3  also the endpoint: memory window, BAR0, the MHI and BHI registers (no firmware yet)
//   4  also the WiFi module (Ath11kWifi: ath11k, MHI, QRTR/QMI) with DMA, firmware (/private/var/ngpu/wifi) and MSIs from this board
// pdx203-wifi-gen1=1 forces a Gen1 link (no CX vote, refgen at 19.2 MHz). Rows print as "PDX203 WIFI".
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IODeviceMemory.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
#include <IOKit/IORegistryEntry.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <libkern/c++/OSData.h>
#include <string.h>
#include <stdarg.h>
#include <libkern/zlib.h>
#include <mach/kmod.h>
#include <libkern/c++/OSMetaClass.h>
extern "C" {
#include "SmmuBypass.h"
#include "nw_abi.h"
}
extern "C" int sheng_fw_open(const char *path, void **handle, uint64_t *size);
extern "C" int sheng_fw_read(void *handle, uint64_t offset, void *to, size_t length);
extern "C" void sheng_fw_close(void *handle);
extern "C" int armcpu_spi_attach(uint32_t, bool, void (*)(void *), void *, uint32_t *, uint64_t *) __attribute__((weak_import));
// the boot add-on's staging into /private/var/ngpu (zzy-native-gpu-addon-mount.patch): files in the add-on, then each written, already the
// same or failed
// the boot add-on as the payload's RAM block holds it (NativeRamBlock): NGPUADD2, then per file u32 name bytes, u32 mode, u64 size,
// u64 packed size, the name padded to 8, the zlib stream padded to 8 (make-gpu-addon.py)
extern "C" int native_ram_block_gpu_addon(const uint8_t **data, uint64_t *bytes) __attribute__((weak_import));
extern "C" volatile int native_gpu_addon_files __attribute__((weak_import)), native_gpu_addon_written __attribute__((weak_import)),
    native_gpu_addon_same __attribute__((weak_import)), native_gpu_addon_failed __attribute__((weak_import));

extern "C" int pdx203_rpmh_find(const char *name, uint32_t *address, uint32_t level, uint32_t *index) __attribute__((weak_import));
extern "C" int pdx203_rpmh_send(const uint32_t *addresses, const uint32_t *values, unsigned count) __attribute__((weak_import));

extern "C" {
// [0] stage (positive progress, negative the failed step), [1] GDSC, [2] vote 0x52008, [3] halt bits (bit i: branch i off), [4] pins
// 20|79<<8|80<<16|81<<24 io, [5] rail results (2 bits each: 0 done, 1 sent unconfirmed, 3 failed), [6] CX index, [7] PCS_STATUS1 | polls<<16,
// [8] root port id, [9] ELBI SYS_STTS, [10] LNKCTL/STA, [11] link ms, [12] endpoint id, [13] BAR0 mask, [14] BAR0, [15] MHI version,
// [16] BHI offset, [17] BHI execution environment, [18] MHI status, [19] WLAON pwr ctrl
volatile int32_t pdx203_wifi_stats[20];
void pdx203_wifi_probe(void);
}

namespace {
struct Window {IOMemoryMap *map; volatile uint32_t *va;};
bool mapWindow(Window &w, uint64_t base, uint64_t size, IOOptionBits cache = kIOMapInhibitCache) {
    IODeviceMemory *memory = IODeviceMemory::withRange(base, size);
    w.map = memory ? memory->map(kIOMapAnywhere | cache) : nullptr;
    if (memory) memory->release();
    w.va = w.map ? reinterpret_cast<volatile uint32_t *>(w.map->getVirtualAddress()) : nullptr;
    return w.va != nullptr;
}
void release(Window &w) {if (w.map) w.map->release(); w = {};}
void dsb() {__asm__ volatile("dsb sy" ::: "memory");}
void delayUs(uint32_t us) {
    uint64_t start, now, ticks = (uint64_t)us * 192 / 10;
    __asm__ volatile("isb\n\tmrs %0, cntpct_el0" : "=r"(start));
    do {__asm__ volatile("isb\n\tmrs %0, cntpct_el0" : "=r"(now));} while (now - start < ticks);
}
volatile int32_t *const S = pdx203_wifi_stats;
void fail(int step) {S[0] = -step;}
// A line before each step that could stall the bus: the USB console flushes every 5 ms, so a hang shows where it happened (alioth, the
// other SM8250 + QCA6390 port, went down at its first DBI access with fewer clocks voted than here).
void step(const char *what) {printf("PDX203 WIFI step %d: %s\n", (int)S[0], what); IOSleep(20);}

constexpr uint64_t kGcc = 0x00100000, kTlmm = 0x0f900000, kParf = 0x01c00000, kPhy = 0x01c06000, kDbi = 0x60000000, kAtu = 0x60001000,
                   kConfig = 0x60100000, kMemBase = 0x60300000, kMemSize = 0x03d00000;
// gcc-sm8250.c (offsets from the GCC base)
constexpr uint32_t kGdsc = 0x6b004, kVote = 0x52008, kPcieBcr = 0x6b000, kPhyBcr = 0x6c01c, kAuxRcg = 0x6b038, kRefgenRcg = 0x6f014;
// slv_axi, mstr_axi, cfg_ahb, aux, pipe, slv_q2a, aggre-NoC TBU, ddrss SF TBU, wifi clkref, refgen
constexpr uint32_t kBranches[10] = {0x6b014, 0x6b01c, 0x6b024, 0x6b028, 0x6b02c, 0x6b010, 0x9000c, 0x8d058, 0x8c004, 0x6f02c};
constexpr uint32_t kDirect[4] = {0x9000c, 0x8d058, 0x8c004, 0x6f02c};   // branches enabled by their own bit 0
constexpr unsigned kWlanEn = 20, kPerst = 79, kClkReq = 80, kWake = 81;
// PARF (pcie-qcom.c, ops_1_9_0)
constexpr uint32_t kParfSysCtrl = 0x0, kParfPmCtrl = 0x20, kParfPhyCtrl = 0x40, kParfMhiClockReset = 0x174, kParfAxiHalt = 0x1a8,
                   kParfLtssm = 0x1b0, kParfDbiBase = 0x350, kParfDbiBaseHi = 0x354, kParfSlvSpace = 0x358, kParfSlvSpaceHi = 0x35c,
                   kParfAtuBase = 0x634, kParfAtuBaseHi = 0x638, kParfDeviceType = 0x1000, kParfBdfToSid = 0x2000, kParfBdfToSidCfg = 0x2c00;

// The pcie0 PHY sequence from the phone's own DT (qcom,phy-sequence; equal to upstream sm8250 gen3x1 plus RX 0x574 = 0x3f): POWER_DOWN
// first, SW_RESET and START last.
struct Init {uint16_t offset; uint8_t value;};
constexpr Init kPhySeq[] = {
    {0x840,0x03},
    {0x094,0x08},{0x154,0x34},{0x16c,0x08},{0x058,0x0f},{0x0a4,0x42},{0x110,0x24},{0x11c,0x03},{0x118,0xb4},
    {0x10c,0x02},{0x1bc,0x11},{0x0bc,0x82},{0x0d4,0x03},{0x0d0,0x55},{0x0cc,0x55},{0x0b0,0x1a},{0x0ac,0x0a},
    {0x0c4,0x68},{0x0e0,0x02},{0x0dc,0xaa},{0x0d8,0xab},{0x0b8,0x34},{0x0b4,0x14},{0x158,0x01},{0x074,0x06},
    {0x07c,0x16},{0x084,0x36},{0x078,0x06},{0x080,0x16},{0x088,0x36},{0x1b0,0x1e},{0x1ac,0xca},{0x1b8,0x18},
    {0x1b4,0xa2},{0x050,0x07},{0x010,0x01},{0x01c,0x31},{0x020,0x01},{0x024,0xde},{0x028,0x07},{0x030,0x4c},
    {0x034,0x06},
    {0x29c,0x12},{0x284,0x35},{0x23c,0x11},
    {0x51c,0x03},{0x518,0x1c},{0x524,0x1e},{0x4e8,0x00},{0x4ec,0x0e},{0x4f0,0x4a},{0x4f4,0x0f},{0x5b4,0x04},
    {0x434,0x7f},{0x444,0x70},{0x510,0x17},{0x4d4,0x04},{0x4d8,0x07},{0x598,0xd4},{0x59c,0x54},{0x5a0,0xdb},
    {0x5a4,0x3b},{0x5a8,0x31},{0x584,0x24},{0x588,0xe4},{0x58c,0xec},{0x590,0x3b},{0x594,0x36},{0x570,0x3f},
    {0x574,0x3f},{0x578,0xff},{0x57c,0x7f},{0x580,0x14},{0x4fc,0x00},{0x4f8,0xc0},{0x460,0x30},{0x464,0x00},
    {0x5bc,0x0c},{0x4dc,0x1b},{0x408,0x0c},{0x414,0x03},{0x5b8,0x30},
    {0x9a4,0x01},{0xc90,0x00},{0xc40,0x01},{0xc48,0x01},{0xc50,0x00},{0xcb4,0x33},{0xcbc,0x00},{0xce0,0x58},
    {0xca4,0x0f},{0x048,0x90},{0xc1c,0xc1},{0x988,0x77},{0x998,0x0b},{0x8dc,0x0d},{0x9ec,0x12},
    {0x800,0x00},{0x844,0x03},
};
static_assert(sizeof kPhySeq / sizeof kPhySeq[0] == 99, "pcie0 phy sequence");
constexpr uint32_t kPhyStatus = 0x814;

// TLMM: pull bits 1:0, function 5:2, drive 8:6, OE bit 9; io bit 1 drives. io written before and after, as sheng's setPin.
void setPin(volatile uint32_t *t, unsigned pin, uint32_t cfg, int level) {
    if (level >= 0) t[(pin * 0x1000 + 4) / 4] = level ? 2u : 0u;
    t[pin * 0x1000 / 4] = cfg;
    if (level >= 0) t[(pin * 0x1000 + 4) / 4] = level ? 2u : 0u;
    dsb();
}
uint32_t pins(volatile uint32_t *t) {
    const unsigned list[4] = {kWlanEn, kPerst, kClkReq, kWake};
    uint32_t w = 0;
    for (unsigned i = 0; i < 4; ++i) w |= (t[(list[i] * 0x1000 + 4) / 4] & 3u) << (8 * i);
    return w;
}
uint32_t haltBits(volatile uint32_t *g) {
    uint32_t off = 0;
    for (unsigned i = 0; i < 10; ++i) if (g[kBranches[i] / 4] & (1u << 31)) off |= 1u << i;
    return off;
}
bool waitClear(volatile uint32_t *reg, uint32_t mask, unsigned tries) {
    for (unsigned i = 0; i < tries; ++i) {if (!(*reg & mask)) return true; delayUs(10);}
    return !(*reg & mask);
}
void pulseReset(volatile uint32_t *g, uint32_t bcr) {
    g[bcr / 4] |= 1u; dsb(); IOSleep(1);
    g[bcr / 4] &= ~1u; dsb(); IOSleep(1);
}

// ---- the QCA6390's rails (cnss2 cnss_vreg_list order: io, ant-switch, aon, dig, rfa1, rfa2): VRM voltage at +0, enable at +4, mode +8
struct Rail {const char *name; uint32_t mV; int mode;};
constexpr Rail kRails[6] = {{"smpa4", 1800, -1}, {"ldoc5", 1800, 7}, {"smpa6", 950, -1}, {"smpf2", 950, -1}, {"smpa5", 1900, -1},
                            {"smpc8", 1350, 6}};
int powerRails(bool gen3) {
    if (!pdx203_rpmh_find || !pdx203_rpmh_send) return -1;
    uint32_t addr = 0;
    int rc = -19;
    for (unsigned i = 0; i < 1200 && (rc = pdx203_rpmh_find("smpa4", &addr, 0, nullptr)) == -19; ++i) IOSleep(50);   // the ADSP thread maps the RSC
    if (rc) return -2;
    uint32_t results = 0;
    for (unsigned i = 0; i < 6; ++i) {
        const Rail &r = kRails[i];
        if (pdx203_rpmh_find(r.name, &addr, 0, nullptr)) {results |= 3u << (2 * i); continue;}
        uint32_t a[3] = {addr, addr + 4, addr + 8}, v[3] = {r.mV, 1, (uint32_t)r.mode};
        const int s = pdx203_rpmh_send(a, v, r.mode >= 0 ? 3 : 2);
        results |= (s < 0 ? 3u : (uint32_t)s) << (2 * i);
        IOSleep(2);
    }
    S[5] = (int32_t)results;
    if (gen3) {
        uint32_t idx = 0;
        if (pdx203_rpmh_find("cx.lvl", &addr, 256, &idx) == 0) {
            const uint32_t a[1] = {addr}, v[1] = {idx};
            S[6] = pdx203_rpmh_send(a, v, 1) < 0 ? -1 : (int32_t)idx;
        } else S[6] = -2;
    }
    IOSleep(20);
    return (results & 0xaaa) == 0xaaa ? -3 : 0;   // every rail failed
}

void survey(volatile uint32_t *g, volatile uint32_t *t) {
    S[1] = (int32_t)g[kGdsc / 4];
    S[2] = (int32_t)g[kVote / 4];
    S[3] = (int32_t)haltBits(g);
    S[4] = (int32_t)pins(t);
}

Window gBar;
uint64_t gBarBytes;
void endpoint(volatile uint32_t *a, volatile uint32_t *c, volatile uint32_t *d) {
    step("bar0");
    // outbound iATU region 1: the memory window, CPU address = PCI address; the root port's memory base/limit
    volatile uint32_t *r1 = a + 0x200 / 4;
    r1[0x08 / 4] = uint32_t(kMemBase); r1[0x0c / 4] = 0;
    r1[0x10 / 4] = uint32_t(kMemBase + kMemSize - 1); r1[0x20 / 4] = 0;
    r1[0x14 / 4] = uint32_t(kMemBase); r1[0x18 / 4] = 0;
    r1[0x00 / 4] = 0x0;
    r1[0x04 / 4] = 1u << 31;
    for (unsigned i = 0; i < 5 && !(r1[0x04 / 4] & (1u << 31)); ++i) IODelay(10);
    d[0x20 / 4] = uint32_t(((kMemBase + kMemSize - 1) & 0xfff00000u) | ((kMemBase >> 16) & 0xfff0u));
    // BAR0: size, place at the first size-aligned address of the window
    const uint32_t saved = c[0x10 / 4];
    c[0x10 / 4] = 0xffffffffu;
    const uint32_t mask = c[0x10 / 4];
    c[0x10 / 4] = saved;
    S[13] = (int32_t)mask;
    const uint32_t size = ~(mask & ~0xfu) + 1;
    if (!(mask & ~0xfu) || !size || size > kMemSize) {fail(30); return;}
    const uint32_t address = uint32_t((kMemBase + size - 1) & ~uint64_t(size - 1));
    c[0x10 / 4] = address | (mask & 0xfu);
    if ((mask & 6u) == 4u) c[0x14 / 4] = 0;
    c[0x04 / 4] = (c[0x04 / 4] & 0xffffu) | 0x6u;
    S[14] = (int32_t)c[0x10 / 4];
    S[0] = 30;
    Window bar = {};
    if (!mapWindow(bar, address, size < 0x100000 ? size : 0x100000)) {fail(31); return;}
    volatile uint32_t *m = bar.va;
    S[15] = (int32_t)m[0x08 / 4];                 // MHIVER
    const uint32_t bhi = m[0x28 / 4];             // BHIOFF
    S[16] = (int32_t)bhi;
    S[18] = (int32_t)m[0x48 / 4];                 // MHISTATUS
    if (bhi && bhi < 0x80000 - 0x40) S[17] = (int32_t)m[(bhi + 0x28) / 4];   // BHI_EXECENV
    // WLAON_QFPROM_PWR_CTRL (0x1f8031c) through the window register (BAR + 0x310c, ath11k_pci_select_window): read only here
    m[0x310c / 4] = 0x40000000u | ((0x1f8031cu >> 19) & 0x3f);
    dsb();
    S[19] = (int32_t)m[(0x80000 + (0x1f8031cu & 0x7ffff)) / 4];
    S[0] = 31;
    gBar = bar; gBarBytes = size < 0x100000 ? size : 0x100000;   // kept for stage 4
}

void print(const char *what);
// ---- stage 4: the WiFi module (Ath11kWifi) on this board, as sheng's stage 6. The board gives it the endpoint's registers and configuration
// space, DMA memory, firmware files, a clock and a log; the module does the chip's power-up, the MHI firmware boot, QRTR/QMI and ath11k. ----
// DMA: 32 MiB of the PIL CDSP + SLPI carve-outs (0x87800000, 20 + 21 MiB, reserved in the UEFI memory map; this board never starts either
// DSP), below 4 GiB as ath11k's 32-bit coherent mask wants, mapped normal non-cacheable (write-combine: a device mapping faults bzero's
// DC ZVA and unaligned accesses, pdx203-wifi2 panic); the endpoint's SMMU stream (BDF 1:0.0 -> 0x1c01, the BDF->SID
// table set in stage 2) passes through to physical addresses (SmmuBypass.h, as the audio buffer's stream). The first page is the MSI sink.
constexpr uint64_t kArenaPhys = 0x87800000, kArenaBytes = 32u << 20, kSmmu = 0x15000000;
constexpr uint16_t kStream = 0x1c01;
constexpr unsigned kArenaBlocks = 16384;
struct Arena {uint8_t *va; uint64_t iova; size_t bytes; struct Block {uint32_t off, size; bool used;} *blocks; unsigned count;} gArena;
Window gArenaMap, gSmmuMap;
const char *gSmmuError = "";
uint32_t smmuRd(void *ctx, unsigned at) {return *reinterpret_cast<volatile uint32_t *>(reinterpret_cast<uintptr_t>(ctx) + at);}
void smmuWr(void *ctx, unsigned at, uint32_t value) {*reinterpret_cast<volatile uint32_t *>(reinterpret_cast<uintptr_t>(ctx) + at) = value; dsb();}
// The audio thread maps its stream the same way at its boot; this runs only once the firmware files exist on the data volume, long after.
bool smmuPassThrough() {
    if (!gSmmuMap.va && !mapWindow(gSmmuMap, kSmmu, 0x100000)) {gSmmuError = "map"; return false;}
    const smmu_io io = {const_cast<uint32_t *>(gSmmuMap.va), smmuRd, smmuWr};
    const smmu_plan plan = smmu_plan_bypass(&io, kStream, kArenaPhys, kArenaBytes);
    if (const char *error = smmu_configure_bypass(&io, &plan, kStream)) {gSmmuError = error; return false;}
    return true;
}
bool arenaInit() {
    if (!gArena.blocks) gArena.blocks = static_cast<Arena::Block *>(IOMallocData(kArenaBlocks * sizeof(Arena::Block)));
    if (!gArena.blocks || (!gArenaMap.va && !mapWindow(gArenaMap, kArenaPhys, kArenaBytes, kIOMapWriteCombineCache))) return false;
    gArena.va = reinterpret_cast<uint8_t *>(const_cast<uint32_t *>(gArenaMap.va));
    gArena.iova = kArenaPhys; gArena.bytes = kArenaBytes;
    gArena.blocks[0] = {4096, (uint32_t)(gArena.bytes - 4096), false}; gArena.count = 1;   // the first page is the interrupt sink
    bzero(gArena.va, gArena.bytes);
    return true;
}
void pLog(int level, const char *line);
// sheng's allocator: buffers of a page or more are aligned to their size rounded up to a power of two (the boot loader refuses an image
// address that is only page aligned; an IOMMU-backed allocator aligns that way too).
void *arenaAlloc(size_t n, uint64_t *bus) {
    n = (n + 63) & ~(size_t)63;
    if (n >= 4096) n = (n + 4095) & ~(size_t)4095;
    uint32_t align = 64;
    if (n >= 4096) {align = 4096; while (align < n) align <<= 1;}
    for (unsigned i = 0; i < gArena.count; ++i) {
        Arena::Block &b = gArena.blocks[i];
        if (b.used) continue;
        const uint32_t start = (b.off + align - 1) & ~(align - 1), pad = start - b.off;
        if (b.size < pad + n) continue;
        const bool tail = b.size > pad + n;
        const unsigned extra = (pad ? 1u : 0u) + (tail ? 1u : 0u);
        if (gArena.count + extra > kArenaBlocks) continue;
        const Arena::Block whole = b;
        memmove(&gArena.blocks[i + extra], &gArena.blocks[i], (gArena.count - i) * sizeof(Arena::Block));
        gArena.count += extra;
        unsigned at = i;
        if (pad) {gArena.blocks[at] = {whole.off, pad, false}; ++at;}
        gArena.blocks[at] = {start, (uint32_t)n, true};
        if (tail) gArena.blocks[at + 1] = {start + (uint32_t)n, whole.size - pad - (uint32_t)n, false};
        uint8_t *p = gArena.va + start;
        bzero(p, n);
        if (bus) *bus = gArena.iova + start;
        return p;
    }
    uint32_t freeTotal = 0, freeMax = 0;
    for (unsigned i = 0; i < gArena.count; ++i) if (!gArena.blocks[i].used) {freeTotal += gArena.blocks[i].size; if (gArena.blocks[i].size > freeMax) freeMax = gArena.blocks[i].size;}
    char line[96];
    snprintf(line, sizeof line, "dma window full: want %zu free %u largest %u blocks %u", n, freeTotal, freeMax, gArena.count);
    pLog(3, line);
    return nullptr;
}
void arenaFree(void *p) {
    if (!p || (uint8_t *)p < gArena.va || (uint8_t *)p >= gArena.va + gArena.bytes) return;
    const uint32_t off = (uint32_t)((uint8_t *)p - gArena.va);
    for (unsigned i = 0; i < gArena.count; ++i) {
        if (gArena.blocks[i].off != off || !gArena.blocks[i].used) continue;
        gArena.blocks[i].used = false;
        if (i + 1 < gArena.count && !gArena.blocks[i + 1].used) {gArena.blocks[i].size += gArena.blocks[i + 1].size; memmove(&gArena.blocks[i + 1], &gArena.blocks[i + 2], (gArena.count - i - 2) * sizeof(Arena::Block)); --gArena.count;}
        if (i > 0 && !gArena.blocks[i - 1].used) {gArena.blocks[i - 1].size += gArena.blocks[i].size; memmove(&gArena.blocks[i], &gArena.blocks[i + 1], (gArena.count - i - 1) * sizeof(Arena::Block)); --gArena.count;}
        return;
    }
}

uint64_t pNow() {uint64_t ns = 0; absolutetime_to_nanoseconds(mach_absolute_time(), &ns); return ns / 1000000;}
void pSleep(unsigned ms) {IOSleep(ms);}
void pPoll() {}
void pHalt() {for (;;) IOSleep(1000);}
// The module's log on the console: notices, warnings and errors always; debug lines (level 7) only with pdx203-wifi-verbose=1, minus the
// per-frame and per-interrupt ones. A line equal to one of the last sixteen is only counted.
unsigned gVerbose = 2;
uint32_t gLogRepeats;
bool has(const char *s, const char *w) {for (; *s; ++s) {size_t k = 0; while (w[k] && s[k] == w[k]) ++k; if (!w[k]) return true;} return false;}
// pdx203-wifi-verbose: 0 none of the driver's debug lines, 2 (default) only those of the join path (vdev, peer, BSS, management transmit
// and its completion, authentication/association frames received), 1 all but the per-frame ones.
bool joinPath(const char *l) {
    if (has(l, "event mgmt rx")) return has(l, "stype b0") || has(l, "stype 10") || has(l, "stype 30") || has(l, "stype c0") || has(l, "stype a0");
    return has(l, "vdev") || has(l, "peer") || has(l, "bss") || has(l, "mgmt") || has(l, "assoc") || has(l, "auth") || has(l, "tx compl") ||
           has(l, "chanctx") || has(l, "sta state") || has(l, "key");
}
void pLog(int level, const char *line) {
    if (level >= 7 && gVerbose == 2) {if (!joinPath(line) || has(line, "chan info vdev_id")) {++gLogRepeats; return;}}
    else if (level >= 7 && (!gVerbose || has(line, "htc rx completion") || has(line, "rx ce pipe") || has(line, "ext irq:") ||
                       has(line, "event mgmt rx") || has(line, "mgmt rx event") || has(line, "chan info vdev_id") ||
                       has(line, "chan scan list") || has(line, "mac channel ["))) {++gLogRepeats; return;}
    static uint32_t recent[16];
    uint32_t h = 2166136261u;
    for (const char *c = line; *c; ++c) h = (h ^ static_cast<uint8_t>(*c)) * 16777619u;
    h ^= static_cast<uint32_t>(level);
    for (uint32_t r : recent) if (r == h) {++gLogRepeats; return;}
    memmove(&recent[1], &recent[0], sizeof recent - sizeof recent[0]);
    recent[0] = h;
    static uint64_t first;
    const uint64_t now = pNow();
    if (!first) first = now;
    printf("PDX203 WIFI [%u %d] %.150s\n", (uint32_t)(now - first), level, line);
}
void *hAlloc(size_t n) {
    uint8_t *p = static_cast<uint8_t *>(IOMallocData(n + 16));
    if (!p) return nullptr;
    *reinterpret_cast<size_t *>(p) = n + 16;
    return p + 16;
}
void hFree(void *q) {
    if (!q) return;
    uint8_t *p = static_cast<uint8_t *>(q) - 16;
    IOFreeData(p, *reinterpret_cast<size_t *>(p));
}
void *pDmaAlloc(size_t n, uint64_t *bus) {return arenaAlloc(n, bus);}
void pDmaFree(size_t, void *p, uint64_t) {arenaFree(p);}
uint64_t pDmaMap(void *p, size_t, int) {
    if ((uint8_t *)p < gArena.va || (uint8_t *)p >= gArena.va + gArena.bytes) return 0;
    return gArena.iova + ((uint8_t *)p - gArena.va);
}
void pDmaSync(uint64_t, size_t, int) {dsb();}   // the arena is mapped uncached

// The radio's address: locally administered, from /chosen/unique-chip-id (FNV-1a), the same at every boot (sheng's boardMac).
bool boardMac(uint8_t m[6]) {
    IORegistryEntry *chosen = IORegistryEntry::fromPath("/chosen", gIODTPlane);
    OSData *id = chosen ? OSDynamicCast(OSData, chosen->getProperty("unique-chip-id")) : nullptr;
    uint64_t seed = 0;
    if (id && id->getLength() >= sizeof seed) memcpy(&seed, id->getBytesNoCopy(), sizeof seed);
    OSSafeReleaseNULL(chosen);
    if (!seed) return false;
    uint64_t h = 0xcbf29ce484222325ull;
    for (unsigned i = 0; i < 8; ++i) h = (h ^ ((seed >> (8 * i)) & 0xff)) * 0x100000001b3ull;
    for (unsigned i = 0; i < 6; ++i) m[i] = (uint8_t)(h >> (8 * i));
    m[0] = (uint8_t)((m[0] & 0xfe) | 0x02);
    return true;
}
int macFile(const uint8_t **data, size_t *size) {
    static char text[40]; static size_t len;
    if (!len) {
        uint8_t m[6];
        if (!boardMac(m)) return -1;
        len = (size_t)snprintf(text, sizeof text, "address=%02x%02x%02x%02x%02x%02x\n", m[0], m[1], m[2], m[3], m[4], m[5]);
    }
    *data = (const uint8_t *)text; *size = len;
    return 0;
}
// Firmware: the files the boot add-on carries in wifi/ (make-gpu-addon.py --wifi; linux-firmware ath11k/QCA6390/hw2.0, local only), which the
// kernel stages below /private/var/ngpu once the data volume is mounted. Each is read once and kept.
constexpr const char *kFwDir = "/private/var/ngpu/wifi/";
// wifi/<name> straight from the add-on in RAM: always this boot's file (pdx203-wifi10: the staged copy below /private/var/ngpu read back
// as the previous add-on's board.bin), and there before the data volume is.
voidpf zAlloc(voidpf, uInt items, uInt size) {
    const size_t n = (size_t)items * size + 16;
    uint8_t *p = static_cast<uint8_t *>(IOMallocData(n));
    if (!p) return Z_NULL;
    memcpy(p, &n, sizeof n);
    return p + 16;
}
void zFree(voidpf, voidpf ptr) {
    if (!ptr) return;
    uint8_t *p = static_cast<uint8_t *>(ptr) - 16;
    size_t n; memcpy(&n, p, sizeof n);
    IOFreeData(p, n);
}
bool addonFile(const char *base, uint8_t **out, size_t *outBytes) {
    const uint8_t *a = nullptr; uint64_t bytes = 0;
    if (!native_ram_block_gpu_addon || !native_ram_block_gpu_addon(&a, &bytes) || bytes < 24 || memcmp(a, "NGPUADD2", 8)) return false;
    uint32_t count; memcpy(&count, a + 8, 4);
    char want[64]; snprintf(want, sizeof want, "wifi/%s", base);
    uint64_t at = 24;
    for (uint32_t i = 0; i < count && at + 24 <= bytes; ++i) {
        uint32_t nb; uint64_t size, packed;
        memcpy(&nb, a + at, 4); memcpy(&size, a + at + 8, 8); memcpy(&packed, a + at + 16, 8);
        at += 24;
        const char *name = reinterpret_cast<const char *>(a + at);
        at += (nb + 7) & ~7ull;
        if (packed > bytes - at) return false;
        if (nb >= 2 && !strncmp(name, want, nb) && size && size < (64u << 20)) {
            uint8_t *buf = static_cast<uint8_t *>(IOMallocData(size));
            if (!buf) return false;
            z_stream z; memset(&z, 0, sizeof z);
            z.zalloc = zAlloc; z.zfree = zFree;
            z.next_in = const_cast<Bytef *>(a + at); z.avail_in = (uInt)packed; z.next_out = buf; z.avail_out = (uInt)size;
            int r = inflateInit(&z);
            if (r == Z_OK) {r = inflate(&z, Z_FINISH); inflateEnd(&z);}
            if (r == Z_STREAM_END && z.total_out == size) {*out = buf; *outBytes = (size_t)size; return true;}
            IOFreeData(buf, size);
            return false;
        }
        at += (packed + 7) & ~7ull;
    }
    return false;
}
struct FwCache {char name[32]; uint8_t *data; size_t size; bool tried;} gFw[8];
int pFwGet(const char *path, const uint8_t **data, size_t *size) {
    const char *base = path;
    for (const char *c = path; *c; ++c) if (*c == '/') base = c + 1;
    if (!strcmp(base, "wlan_mac.bin")) return macFile(data, size);
    FwCache *slot = nullptr;
    for (auto &f : gFw) if (f.tried && !strncmp(f.name, base, sizeof f.name)) {slot = &f; break;}
    if (!slot) for (auto &f : gFw) if (!f.tried) {slot = &f; strlcpy(f.name, base, sizeof f.name); f.tried = true; break;}
    if (!slot) return -1;
    if (!slot->data) {
        uint8_t *mem = nullptr; size_t memBytes = 0;
        if (addonFile(base, &mem, &memBytes)) {
            slot->data = mem; slot->size = memBytes;
            char line[96]; snprintf(line, sizeof line, "firmware %s: %zu bytes (add-on in RAM)", base, slot->size); pLog(6, line);
            *data = slot->data; *size = slot->size;
            return 0;
        }
        char full[96];
        snprintf(full, sizeof full, "%s%s", kFwDir, base);
        void *h = nullptr; uint64_t bytes = 0;
        if (sheng_fw_open(full, &h, &bytes) || !bytes || bytes > (64u << 20)) {if (h) sheng_fw_close(h); return -1;}
        uint8_t *buf = static_cast<uint8_t *>(IOMallocData((size_t)bytes));
        if (!buf || sheng_fw_read(h, 0, buf, (size_t)bytes)) {if (buf) IOFreeData(buf, (size_t)bytes); sheng_fw_close(h); return -1;}
        sheng_fw_close(h);
        slot->data = buf; slot->size = (size_t)bytes;
        char line[96]; snprintf(line, sizeof line, "firmware %s: %zu bytes", base, slot->size); pLog(6, line);
    }
    *data = slot->data; *size = slot->size;
    return 0;
}
// The system's record of the WiFi address, as a boot loader leaves it: wifid derives each network's private address from it.
void publishMac() {
    uint8_t m[6];
    if (!boardMac(m)) return;
    if (IORegistryEntry *chosen = IORegistryEntry::fromPath("/chosen", gIODTPlane)) {
        if (!chosen->getProperty("mac-address-wifi0")) chosen->setProperty("mac-address-wifi0", m, sizeof m);
        chosen->release();
    }
}

// Configuration space of the endpoint through the iATU window; offsets with bit 31 set are relative to the PCI Express capability.
volatile uint32_t *gCfg, *gDbi;
uint32_t gPcieCap, gMsiCap;
bool findCaps() {
    uint8_t cap = (uint8_t)(gCfg[0x34 / 4] & 0xff);
    for (unsigned guard = 0; cap >= 0x40 && guard < 48; ++guard) {
        const uint32_t w = gCfg[cap / 4];
        if ((uint8_t)w == 0x10) gPcieCap = cap;
        if ((uint8_t)w == 0x05) gMsiCap = cap;
        cap = (uint8_t)(w >> 8);
    }
    return gPcieCap && gMsiCap;
}
int pCfgRead(int off, int size, uint32_t *v) {
    uint32_t o = (uint32_t)off;
    if (o & 0x80000000u) o = gPcieCap + (o & 0x7fffffffu);
    const uint32_t w = gCfg[(o & ~3u) / 4];
    *v = size == 4 ? w : size == 2 ? (w >> ((o & 2) * 8)) & 0xffff : (w >> ((o & 3) * 8)) & 0xff;
    return 0;
}
int pCfgWrite(int off, int size, uint32_t v) {
    uint32_t o = (uint32_t)off;
    if (o & 0x80000000u) o = gPcieCap + (o & 0x7fffffffu);
    if (size == 4) {gCfg[o / 4] = v; dsb(); return 0;}
    const unsigned shift = (size == 2 ? (o & 2) : (o & 3)) * 8;
    const uint32_t mask = (size == 2 ? 0xffffu : 0xffu) << shift;
    gCfg[(o & ~3u) / 4] = (gCfg[(o & ~3u) / 4] & ~mask) | ((v << shift) & mask);
    dsb();
    return 0;
}
int pMsiTarget(uint32_t *lo, uint32_t *hi, uint32_t *data) {*lo = (uint32_t)gArena.iova; *hi = (uint32_t)(gArena.iova >> 32); *data = 0; return 0;}

// The endpoint's MSIs: the root complex's DesignWare iMSI takes inbound writes to MSI_ADDR (the arena's first page, the address the endpoint
// is given) instead of memory, sets bit <data> of group 0's status and raises "msi0" (SPI 141, level). The handler clears the status and
// wakes the thread; without the line the loop polls every millisecond.
constexpr uint32_t kMsiAddrLo = 0x820, kMsiAddrHi = 0x824, kMsiEnable0 = 0x828, kMsiMask0 = 0x82c, kMsiStatus0 = 0x830, kMsiSpi = 141;
volatile uint32_t gMsiCount, gMsiBits, gMsiWake;
bool gMsiAttached;
void msiIrq(void *) {
    const uint32_t status = gDbi[kMsiStatus0 / 4];
    if (status) gDbi[kMsiStatus0 / 4] = status;
    gMsiCount = gMsiCount + 1;
    gMsiBits = gMsiBits | status;
    static uint32_t empty;
    empty = status ? 0 : empty + 1;
    if (empty > 100000) {gDbi[kMsiEnable0 / 4] = 0; gMsiAttached = false;}   // a line with nothing to clear would storm
    gMsiWake = 1;
    thread_wakeup(event_t(&gMsiWake));
}
int msiAttach() {
    gDbi[kMsiAddrLo / 4] = uint32_t(gArena.iova);
    gDbi[kMsiAddrHi / 4] = uint32_t(gArena.iova >> 32);
    gDbi[kMsiMask0 / 4] = 0;
    gDbi[kMsiEnable0 / 4] = 0xffffffffu;
    dsb();
    if (!armcpu_spi_attach) return -78;
    const int rc = armcpu_spi_attach(32 + kMsiSpi, false, msiIrq, nullptr, nullptr, nullptr);
    gMsiAttached = rc == 0;
    return rc;
}
void msiWait(unsigned ms) {
    if (__atomic_exchange_n(&gMsiWake, 0u, __ATOMIC_ACQ_REL)) return;
    assert_wait_timeout(event_t(&gMsiWake), THREAD_UNINT, ms, NSEC_PER_MSEC);
    if (__atomic_exchange_n(&gMsiWake, 0u, __ATOMIC_ACQ_REL)) {clear_wait(current_thread(), THREAD_AWAKENED); return;}
    thread_block(THREAD_CONTINUE_NULL);
    gMsiWake = 0;
}

// The network side (WifiNet.cpp, sheng's): its classes register with the kernel's module like the other late classes (static initialiser
// globalised by the build), then en0 and the IO80211 registry entries are made once the module has a station interface.
}  // namespace
extern kmod_info_t g_kernel_kmod_info;
extern "C" void native_wifi_net_static_init(void) __asm__("__GLOBAL__sub_I_WifiNet.cpp");
extern "C" void sheng_wifi_net_start(const uint8_t mac[6]);
extern "C" void native_wifi_logf(int level, const char *fmt, ...) {
    char line[160]; va_list ap; va_start(ap, fmt); vsnprintf(line, sizeof line, fmt, ap); va_end(ap);
    pLog(level, line);
}
// WifiNet's unattended join reads join.txt here (absent unless the add-on carries one); wlan_mac.bin is made from the chip id.
extern "C" int sheng_wifi_file(const char *name, const uint8_t **data, size_t *size) {return pFwGet(name, data, size);}
// WifiNet queued a frame: the WiFi thread sends it now instead of at its next wake.
extern "C" void sheng_wifi_wake(void) {gMsiWake = 1; thread_wakeup(event_t(&gMsiWake));}
namespace {
void netStart() {
    struct nw_wifi_status status; nw_wifi_status_get(&status);
    void *handle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if (!handle) {pLog(3, "wifi: network classes: no module handle"); return;}
    native_wifi_net_static_init();
    if (const OSReturn r = OSMetaClass::postModLoad(handle)) {char t[64]; snprintf(t, sizeof t, "wifi: network classes not registered: %d", (int)r); pLog(3, t); return;}
    sheng_wifi_net_start(status.mac);
}

void wifiStack(volatile uint32_t *config, volatile uint32_t *dbi) {
    gCfg = config; gDbi = dbi;
    PE_parse_boot_argn("pdx203-wifi-verbose", &gVerbose, sizeof gVerbose);
    publishMac();
    // The firmware arrives with the data volume, staged from the boot add-on. /private/var/ngpu keeps the last boot's files, so wait until
    // this boot's staging has gone through every file (pdx203-wifi9: the driver read the previous board.bin while the new one was still
    // being written); without staging at all (no add-on), go on after two minutes.
    unsigned waited = 0;
    uint8_t *probe = nullptr; size_t probeBytes = 0;
    const bool inRam = addonFile("amss.bin", &probe, &probeBytes);
    if (probe) IOFreeData(probe, probeBytes);
    if (!inRam && &native_gpu_addon_files) {
        for (; waited < 600; ++waited) {
            const int files = native_gpu_addon_files;
            if (files >= 0 && native_gpu_addon_written + native_gpu_addon_same + native_gpu_addon_failed >= files) break;
            if (files < 0 && waited >= 120) break;
            IOSleep(1000);
        }
    }
    const uint8_t *fw = nullptr; size_t fwBytes = 0;
    for (; waited < 600 && pFwGet("amss.bin", &fw, &fwBytes); ++waited) IOSleep(1000);
    if (!fw) {pLog(3, "wifi: no firmware in /private/var/ngpu/wifi"); fail(40); print("failed"); return;}
    S[0] = 40;
    if (!smmuPassThrough()) {char t[96]; snprintf(t, sizeof t, "wifi: SMMU stream 0x1c01: %s", gSmmuError); pLog(3, t); fail(41); print("failed"); return;}
    if (!arenaInit()) {pLog(3, "wifi: DMA arena not mapped"); fail(42); print("failed"); return;}
    if (!findCaps()) {pLog(3, "wifi: no PCIe/MSI capability"); fail(43); print("failed"); return;}
    S[0] = 43;
    print("module start");
    static const struct lx_platform plat = {pNow, pSleep, pPoll, pLog, pHalt, hAlloc, hFree, pDmaAlloc, pDmaFree, pDmaMap, pDmaSync, pFwGet};
    static struct nw_board board;
    board.bar = const_cast<uint32_t *>(gBar.va); board.bar_len = gBarBytes; board.device_id = (uint16_t)(gCfg[0] >> 16);
    board.msi_cap = (uint16_t)gMsiCap; board.irq_base = 1000;
    board.cfg_read = pCfgRead; board.cfg_write = pCfgWrite; board.msi_target = pMsiTarget; board.map_phys = nullptr;
    const int msi = msiAttach();
    char t[112];
    snprintf(t, sizeof t, "wifi: arena %llx+%lluM, stream %x bypassed, MSI %s (%d), waited %us for firmware", (unsigned long long)kArenaPhys,
             (unsigned long long)(kArenaBytes >> 20), kStream, msi ? "polled" : "on SPI 141", msi, waited);
    pLog(6, t);
    const int r = nw_wifi_start(&plat, &board);
    S[0] = r ? -44 : 44;
    snprintf(t, sizeof t, "wifi: module start %d", r);
    pLog(r ? 3 : 6, t);
    uint32_t lastStage = 0xffffffffu;
    uint64_t nextReport = 0;
    bool netDone = false;
    unsigned net = 1;
    PE_parse_boot_argn("pdx203-wifi-net", &net, sizeof net);   // 0: the radio only, no en0 for iOS
    for (;;) {
        const int busy = nw_wifi_step();
        if (net && !netDone && nw_wifi_stats.stage >= 4) {netDone = true; netStart();}
        if (!busy) {if (gMsiAttached) msiWait(10); else IOSleep(1);}
        const uint64_t now = pNow();
        if (nw_wifi_stats.stage != lastStage || now >= nextReport) {
            lastStage = nw_wifi_stats.stage;
            nextReport = now + 60000;
            snprintf(t, sizeof t, "wifi: stage %u scans %u frames %u beacons %u bss %u err %d | msi %u bits %08x | repeats %u", nw_wifi_stats.stage,
                     nw_wifi_stats.scans, nw_wifi_stats.frames, nw_wifi_stats.beacons, nw_wifi_stats.bss_seen, nw_wifi_stats.last_err, gMsiCount,
                     gMsiBits, gLogRepeats);
            pLog(6, t);
        }
    }
}

void bringUp(volatile uint32_t *g, volatile uint32_t *t, unsigned stage, bool gen3) {
    step("rails and pins");
    // PERST# asserted, CLKREQ# to pci_e0, WAKE# input; WLAN_EN low (the chip starts from nothing)
    setPin(t, kPerst, 0x201, 0);
    setPin(t, kClkReq, 0x007, -1);
    setPin(t, kWake, 0x003, -1);
    setPin(t, kWlanEn, 0x201, 0);
    const uint64_t lowSince = mach_absolute_time();
    S[0] = 10;
    if (int rc = powerRails(gen3)) {fail(11); S[1] = rc; return;}
    S[0] = 11;
    // WLAN_EN low >= 100 ms, then high and 60 ms (pwrseq-qcom-wcn qca6390)
    uint64_t ns = 0;
    absolutetime_to_nanoseconds(mach_absolute_time() - lowSince, &ns);
    if (ns < 150000000ull) IOSleep((uint32_t)((150000000ull - ns) / 1000000) + 1);
    setPin(t, kWlanEn, 0x3c3, 1);
    IOSleep(60);
    S[4] = (int32_t)pins(t);
    S[0] = 12;
    step("gdsc");
    g[kGdsc / 4] &= ~1u; dsb();
    for (unsigned i = 0; i < 100 && !(g[kGdsc / 4] & (1u << 31)); ++i) delayUs(10);
    S[1] = (int32_t)g[kGdsc / 4];
    if (!(g[kGdsc / 4] & (1u << 31))) {fail(13); return;}
    S[0] = 13;
    step("clocks and resets");
    // RCGs (CMD bit 0 latches CFG): aux 19.2 MHz from TCXO, refgen 100 MHz (GPLL0 / 6) for Gen3 or 19.2 MHz
    g[(kAuxRcg + 4) / 4] = 0x1; g[kAuxRcg / 4] |= 1u;
    g[(kRefgenRcg + 4) / 4] = gen3 ? 0x10bu : 0x1u; g[kRefgenRcg / 4] |= 1u;
    dsb();
    waitClear(&g[kAuxRcg / 4], 1u, 100);
    waitClear(&g[kRefgenRcg / 4], 1u, 100);
    g[kVote / 4] |= 0x2fu;                           // slv_axi, mstr_axi, cfg_ahb, aux, slv_q2a (pipe after the PHY)
    for (uint32_t r : kDirect) g[r / 4] |= 1u;
    dsb();
    delayUs(200);
    S[2] = (int32_t)g[kVote / 4];
    S[3] = (int32_t)haltBits(g);
    if (S[3] & ~(1 << 4)) {fail(14); return;}
    pulseReset(g, kPcieBcr);
    pulseReset(g, kPhyBcr);
    S[0] = 14;

    Window parf = {}, dbi = {}, phy = {};
    if (!mapWindow(parf, kParf, 0x3000) || !mapWindow(dbi, kDbi, 0x1000) || !mapWindow(phy, kPhy, 0x1000)) {
        release(parf); release(dbi); fail(15); return;
    }
    volatile uint32_t *f = parf.va, *d = dbi.va, *p = phy.va;
    step("parf");
    // PARF before the PHY (qcom_pcie_init_2_7_0, configure_dbi_atu_base)
    f[kParfDeviceType / 4] = 4u;
    f[kParfPmCtrl / 4] &= ~(1u << 5);
    f[kParfPhyCtrl / 4] &= ~1u;
    f[kParfDbiBase / 4] = uint32_t(kDbi); f[kParfDbiBaseHi / 4] = 0;
    f[kParfAtuBase / 4] = uint32_t(kAtu); f[kParfAtuBaseHi / 4] = 0;
    f[kParfSlvSpace / 4] = 0; f[kParfSlvSpaceHi / 4] = 0x80000000u;
    f[kParfSysCtrl / 4] &= ~(1u << 29);
    f[kParfMhiClockReset / 4] |= 1u << 4;
    f[kParfAxiHalt / 4] |= 1u << 31;
    dsb();
    S[0] = 15;
    step("phy");
    // PHY: the sequence, 1 ms, the pipe clock, PHYSTATUS low
    for (const Init &e : kPhySeq) p[e.offset / 4] = e.value;
    dsb();
    IOSleep(1);
    g[kVote / 4] |= 1u << 4; dsb();
    unsigned polls = 0;
    while (polls < 20 && (p[kPhyStatus / 4] & (1u << 6))) {IOSleep(1); ++polls;}
    S[7] = (int32_t)((p[kPhyStatus / 4] & 0xffffu) | polls << 16);
    if (p[kPhyStatus / 4] & (1u << 6)) {release(phy); release(dbi); release(parf); fail(16); return;}
    // every branch, the pipe included, must run before the first DBI access
    S[3] = (int32_t)haltBits(g);
    if (S[3]) {release(phy); release(dbi); release(parf); fail(16); return;}
    S[0] = 16;
    step("first dbi access");
    // BDF -> SMMU stream table (qcom_pcie_config_sid_1_9_0): requester 0x100 -> stream offset 1 (0x1c01)
    for (unsigned i = 0; i < 256; ++i) f[(kParfBdfToSid + 4 * i) / 4] = 0;
    f[kParfBdfToSidCfg / 4] &= ~1u;
    {
        uint8_t crc = 0;
        const uint8_t bytes[2] = {0x01, 0x00};
        for (uint8_t byte : bytes) {crc ^= byte; for (int b = 0; b < 8; ++b) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 7) : (uint8_t)(crc << 1);}
        f[(kParfBdfToSid + 4 * crc) / 4] = 0x100u << 16 | 1u << 8;
    }
    // root port: bus numbers, memory + bus master; link setup (pci-msm msm_pcie_link_train)
    d[0x18 / 4] = 0x00010100;
    d[0x04 / 4] |= 0x6u;
    d[0x80c / 4] = (d[0x80c / 4] & ~(0x1fu << 8)) | (1u << 8);   // one lane
    d[0x890 / 4] &= ~1u;
    d[0x8bc / 4] |= 1u; d[0x154 / 4] = 0x55555555u; d[0x8bc / 4] &= ~1u;
    if (!gen3) d[0xa0 / 4] = (d[0xa0 / 4] & ~0xfu) | 1u;          // target speed Gen1
    d[0x78 / 4] = (d[0x78 / 4] & ~(7u << 12)) | (5u << 12);
    dsb();
    S[8] = (int32_t)d[0];
    S[0] = 17;
    step("perst and ltssm");
    // PERST# low >= 100 ms in all, then high, 1 ms, LTSSM
    absolutetime_to_nanoseconds(mach_absolute_time() - lowSince, &ns);
    IOSleep(100);
    setPin(t, kPerst, 0x201, 1);
    IOSleep(1);
    f[kParfLtssm / 4] |= 1u << 8; dsb();
    const uint64_t start = mach_absolute_time();
    bool up = false;
    for (unsigned i = 0; i < 60 && !up; ++i) {
        up = (d[0xf28 / 4] & (1u << 10)) && (d[0x80 / 4] & (1u << 29));
        if (!up) IOSleep(5);
    }
    absolutetime_to_nanoseconds(mach_absolute_time() - start, &ns);
    S[11] = (int32_t)(ns / 1000000);
    S[9] = (int32_t)d[0xf28 / 4];
    S[10] = (int32_t)d[0x80 / 4];
    if (!up) {release(phy); release(dbi); release(parf); fail(18); return;}
    S[0] = 18;
    d[0x70c / 4] |= 1u << 15;
    d[0xb40 / 4] = 0x14;                                         // aux clock 19.2 MHz
    step("config read");
    // outbound iATU region 0: the config window as type 0 accesses to bus 1 device 0; then the endpoint's ID
    Window atu = {}, config = {};
    if (mapWindow(atu, kAtu, 0x1000) && mapWindow(config, kConfig, 0x1000)) {
        volatile uint32_t *a = atu.va;
        a[0x04 / 4] = 0;
        a[0x00 / 4] = 0x4;
        a[0x08 / 4] = uint32_t(kConfig); a[0x0c / 4] = 0;
        a[0x10 / 4] = uint32_t(kConfig + 0xfff); a[0x20 / 4] = 0;
        a[0x14 / 4] = 1u << 24; a[0x18 / 4] = 0;
        a[0x04 / 4] = 1u << 31;
        for (unsigned i = 0; i < 5 && !(a[0x04 / 4] & (1u << 31)); ++i) IODelay(10);
        uint32_t id = 0xffffffffu;
        for (unsigned i = 0; i < 1000 && (id = config.va[0]) == 0xffffffffu; ++i) IOSleep(1);
        S[12] = (int32_t)id;
        if (id == 0x110117cbu) {S[0] = 20; if (stage >= 3) endpoint(a, config.va, d);}
        else fail(19);
    } else fail(19);
    if (stage >= 4 && S[0] == 31) {
        wifiStack(config.va, dbi.va);   // runs for the rest of the boot; the windows stay mapped
        return;
    }
    release(gBar);
    release(config); release(atu); release(phy); release(dbi); release(parf);
}

void print(const char *what) {
    const volatile int32_t *s = S;
    printf("PDX203 WIFI %s stage %d | gdsc %x vote %x halt %x pins %x | rails %x cx %d | phy %x | rc %x stts %x lnk %x %dms | ep %x bar %x/%x "
           "mhi %x bhi %x ee %x st %x wlaon %x\n", what, s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8], s[9], s[10], s[11], s[12],
           s[13], s[14], s[15], s[16], s[17], s[18], s[19]);
}

void wifiThread(void *arg, wait_result_t) {
    thread_set_thread_name(current_thread(), "pdx203-wifi");
    const unsigned stage = (unsigned)reinterpret_cast<uintptr_t>(arg);
    unsigned gen1 = 0;
    PE_parse_boot_argn("pdx203-wifi-gen1", &gen1, sizeof gen1);
    Window gcc = {}, tlmm = {};
    if (!mapWindow(gcc, kGcc, 0x100000) || !mapWindow(tlmm, kTlmm, 0x60000)) {fail(1); release(gcc); return;}
    S[0] = 1;
    survey(gcc.va, tlmm.va);
    print("survey");
    if (stage >= 2) {bringUp(gcc.va, tlmm.va, stage, gen1 != 1); print(S[0] > 0 ? "up" : "failed");}
    release(tlmm); release(gcc);
}
}  // namespace

extern "C" void pdx203_wifi_probe(void) {
    unsigned stage = 0;
    if (!PE_parse_boot_argn("pdx203-wifi", &stage, sizeof stage) || !stage || pdx203_wifi_stats[0]) return;
    thread_t thread = nullptr;
    if (kernel_thread_start(wifiThread, reinterpret_cast<void *>(uintptr_t(stage)), &thread) == KERN_SUCCESS) thread_deallocate(thread);
    else fail(7);
}
