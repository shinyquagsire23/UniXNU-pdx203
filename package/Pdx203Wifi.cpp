// SPDX-License-Identifier: BSD-2-Clause-Patent
// pdx203 WiFi, stages 1-3: the QCA6390 on PCIe0 (sheng's WifiProbe.cpp stages 1-4 with the SM8250 values of /tmp/kmdconn/wifi-pcie-spec.md:
// Sony's kona DT, LineageOS android_kernel_sony_sm8250 cnss2/pci-msm, upstream gcc-sm8250.c, phy-qcom-qmp-pcie.c, pcie-qcom.c).
// Boot argument pdx203-wifi=<n>:
//   1  read-only: GDSC, clock votes and halts, the pins
//   2  power and link: the chip's rails over RPMh (through AdspProbe's TCS 1 helper; needs pdx203-adsp), CX at NOM for Gen3, WLAN_EN,
//      GDSC, clocks, resets, PARF, the QMP PHY (the DT's 99-write sequence), PERST#, LTSSM, link up, the endpoint's ID (17cb:1101)
//   3  also the endpoint: memory window, BAR0, the MHI and BHI registers (no firmware yet)
// pdx203-wifi-gen1=1 forces a Gen1 link (no CX vote, refgen at 19.2 MHz). Rows print as "PDX203 WIFI".
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IODeviceMemory.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>

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
bool mapWindow(Window &w, uint64_t base, uint64_t size) {
    IODeviceMemory *memory = IODeviceMemory::withRange(base, size);
    w.map = memory ? memory->map(kIOMapAnywhere | kIOMapInhibitCache) : nullptr;
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
    release(bar);
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
