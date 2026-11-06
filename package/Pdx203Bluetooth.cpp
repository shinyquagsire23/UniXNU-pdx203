// SPDX-License-Identifier: BSD-2-Clause-Patent
// pdx203 Bluetooth: the QCA6390's Bluetooth core on QUP serial engine 6 (wrapper 0, 0x00998000) as a 4-wire UART, from the phone's kona DT
// (qupv3_se6_4uart: gpio16 CTS, 17 RFR, 18 TX, 19 RX, function qup6; BT_EN gpio21; SPI 607) and upstream Linux (qcom_geni_serial.c,
// qcom-geni-se.c, gcc-sm8250.c, hci_qca.c, btqca.c). The chip's rails are the ones the WiFi bring-up (Pdx203Wifi.cpp) turns on, so this
// waits for it. Then: BT_EN, the UART at 115200, the version, 3 Mbit/s, the rampatch and the NVM (linux-firmware qca/htbtfw20.tlv and
// htnv20.bin, from the add-on's wifi/ directory: local only), HCI reset, the address. The controller then runs Qualcomm's in-band
// sleep even with the NVM's sleep bits cleared, which the bridge answers (ibsByte). After that the controller is iOS's: the control
// socket com.apple.uart(.sk).bluetooth (Pdx203BtTty.c) carries the H4 stream both ways.
// Boot argument pdx203-blue=<n> (pdx203-bt is the buttons'): 0 off, 1 survey only, 2 bring-up without the tty, 3 (default) all.
// Rows print as "PDX203 BLUE".
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IORegistryEntry.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOService.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSData.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
#include <string.h>

extern "C" int armcpu_spi_attach(uint32_t, bool, void (*)(void *), void *, uint32_t *, uint64_t *) __attribute__((weak_import));
// Pdx203Wifi.cpp: [0] its stage (11: rails on, 12: WLAN_EN high), and its add-on file reader (wifi/<name>, kept for the boot)
extern "C" volatile int32_t pdx203_wifi_stats[20];
extern "C" int sheng_wifi_file(const char *name, const uint8_t **data, size_t *size);
// Pdx203BtTty.c
extern "C" int pdx203_bt_tty_attach(void (*kick)(void), void (*opened)(int));
extern "C" int pdx203_bt_tty_is_open(void);
extern "C" size_t pdx203_bt_tty_rx(const uint8_t *bytes, size_t n);
extern "C" size_t pdx203_bt_tty_tx(uint8_t *bytes, size_t max);

extern "C" {
// [0] stage (positive progress, negative the failed step), [1] protocol | fifo depths, [2] clock (kHz), [3] QUP version, [4] product id,
// [5] soc id, [6] rom | patch version, [7] rampatch bytes, [8] nvm bytes, [9] irqs, [10] rx bytes, [11] tx bytes, [12] rx dropped (port
// closed), [13] commands from the app, [14] events to the app, [15] rx errors (overrun, framing)
volatile int32_t pdx203_bt_stats[16];
void pdx203_bluetooth_probe(void);
}

namespace {
volatile int32_t *const S = pdx203_bt_stats;
void fail(int step) {S[0] = -step;}

struct Window {IOMemoryMap *map; volatile uint32_t *va;};
bool mapWindow(Window &w, uint64_t base, uint64_t size) {
    IODeviceMemory *memory = IODeviceMemory::withRange(base, size);
    w.map = memory ? memory->map(kIOMapAnywhere | kIOMapInhibitCache) : nullptr;
    if (memory) memory->release();
    w.va = w.map ? reinterpret_cast<volatile uint32_t *>(w.map->getVirtualAddress()) : nullptr;
    return w.va != nullptr;
}
void dsb() {__asm__ volatile("dsb sy" ::: "memory");}
uint64_t nowMs() {uint64_t ns; absolutetime_to_nanoseconds(mach_absolute_time(), &ns); return ns / 1000000;}

constexpr uint64_t kGcc = 0x00100000, kTlmm = 0x0f900000, kSe = 0x00998000, kWrap = 0x009c0000;
constexpr unsigned kCts = 16, kRfr = 17, kTx = 18, kRx = 19, kBtEn = 21, kSpi = 607;
// gcc-sm8250.c: the vote register's wrapper 0 bits (m_ahb 6, s_ahb 7, core 8, core_2x 9) and SE6 (16); SE6's clock source
constexpr uint32_t kGccVote = 0x52008, kVoteMask = (1u << 6) | (1u << 7) | (1u << 8) | (1u << 9) | (1u << 16), kRcg = 0x17730;

// GENI registers (geni-se.h, qcom-geni-se.c, qcom_geni_serial.c)
enum : uint32_t {
    FORCE_DEFAULT = 0x20, OUTPUT_CTRL = 0x24, CGC_CTRL = 0x28, GENI_STATUS = 0x40, M_CLK_CFG = 0x48, S_CLK_CFG = 0x4c, FW_REVISION = 0x68,
    CLK_SEL = 0x7c, LOOPBACK = 0x22c, IO_MACRO = 0x240, BYTE_GRAN = 0x254, DMA_MODE = 0x258, TX_TRANS_CFG = 0x25c, TX_PACK0 = 0x260,
    TX_PACK1 = 0x264, TX_WORD_LEN = 0x268, TX_STOP_LEN = 0x26c, TX_TRANS_LEN = 0x270, RX_TRANS_CFG = 0x280, RX_PACK0 = 0x284,
    RX_PACK1 = 0x288, RX_WORD_LEN = 0x28c, RX_STALE = 0x294, TX_PARITY = 0x2a4, RX_PARITY = 0x2a8, MANUAL_RFR = 0x2ac,
    M_CMD0 = 0x600, M_CMD_CTRL = 0x604, M_IRQ_STATUS = 0x610, M_IRQ_EN = 0x614, M_IRQ_CLEAR = 0x618,
    S_CMD0 = 0x630, S_CMD_CTRL = 0x634, S_IRQ_STATUS = 0x640, S_IRQ_EN = 0x644, S_IRQ_CLEAR = 0x648,
    TX_FIFO = 0x700, RX_FIFO = 0x780, TX_FIFO_STATUS = 0x800, RX_FIFO_STATUS = 0x804, TX_WATERMARK = 0x80c, RX_WATERMARK = 0x810,
    RX_RFR_WATERMARK = 0x814, DMA_TX_IRQ_CLR = 0xc44, DMA_RX_IRQ_CLR = 0xd44, GSI_EVENT_EN = 0xe18, SE_IRQ_EN = 0xe1c,
    HW_PARAM0 = 0xe24, HW_PARAM1 = 0xe28, DMA_GENERAL_CFG = 0xe30,
};
constexpr uint32_t M_CMD_DONE = 1u << 0, M_CMD_CANCEL = 1u << 4, M_CMD_ABORT = 1u << 5, M_TX_WATERMARK = 1u << 30;
constexpr uint32_t S_CMD_CANCEL = 1u << 4, S_CMD_ABORT = 1u << 5, S_RX_WR_ERR = 1u << 25, S_RX_WATERMARK = 1u << 26, S_RX_LAST = 1u << 27,
                   S_GP_IRQS = 0xfu << 9;
constexpr uint32_t kUartStartTx = 1, kUartStartRead = 1;

Window gGccW, gTlmmW, gSeW, gWrapW;
inline uint32_t se(uint32_t r) {return gSeW.va[r / 4];}
inline void seW(uint32_t r, uint32_t v) {gSeW.va[r / 4] = v;}
unsigned gTxDepth = 16, gRxDepth = 16, gSampling = 16;
uint32_t gClockHz;

// TLMM pin: pull 1:0 (0 none, 1 down, 3 up), function 5:2, drive 8:6 (2 mA = 0), output enable 9; io bit 1 drives
void pin(unsigned n, uint32_t function, uint32_t pull, bool output, int level) {
    volatile uint32_t *t = gTlmmW.va;
    if (level >= 0) t[(n * 0x1000 + 4) / 4] = level ? 2u : 0u;
    t[n * 0x1000 / 4] = pull | (function << 2) | (output ? 1u << 9 : 0u);
    if (level >= 0) t[(n * 0x1000 + 4) / 4] = level ? 2u : 0u;
    dsb();
}

// The SE clock source: one rate that divides to 3 Mbit/s exactly and to 115200 within 0.2%: 3 MHz x the oversampling. Under DFS (the
// boot loader's table, which the engine's CLK_SEL indexes) the level with that rate is chosen; otherwise the source is set to it.
bool clockSource() {
    volatile uint32_t *g = gGccW.va;
    g[kGccVote / 4] |= kVoteMask;
    dsb();
    const uint32_t want = 3000000u * gSampling;
    if (g[(kRcg + 0x14) / 4] & 1u) {
        for (unsigned level = 0; level < 8; ++level) {
            const uint32_t cfg = g[(kRcg + 0x1c + 4 * level) / 4];
            const uint32_t src = (cfg >> 8) & 7u, hid = cfg & 0x1fu, mode = (cfg >> 12) & 3u;
            uint64_t rate = src == 0 ? 19200000ull : src == 1 ? 600000000ull : src == 6 ? 300000000ull : 0;
            if (!rate) continue;
            if (hid) rate = rate * 2 / (hid + 1);
            if (mode) {
                const uint32_t m = g[(kRcg + 0x5c + 4 * level) / 4] & 0xffffu;
                const uint32_t n = ((~g[(kRcg + 0x9c + 4 * level) / 4]) & 0xffffu) + m;
                if (!n) continue;
                rate = rate * m / n;
            }
            if (rate == want || rate == 2ull * want) {
                seW(CLK_SEL, level);
                gClockHz = (uint32_t)rate;
                printf("PDX203 BLUE clock: dfs level %u = %u Hz\n", level, gClockHz);
                return true;
            }
        }
        printf("PDX203 BLUE clock: no dfs level gives %u Hz\n", want);
        return false;
    }
    // GPLL0 even (300 MHz) x m/n, half integer divider 1 (gcc_qupv3_wrap0_s2 table: 48 MHz = 4/25, 96 MHz = 8/25)
    const uint32_t m = want == 48000000u ? 4u : 8u, n = 25;
    if (want != 48000000u && want != 96000000u) return false;
    g[(kRcg + 0x8) / 4] = m;
    g[(kRcg + 0xc) / 4] = ~(n - m) & 0xffffu;
    g[(kRcg + 0x10) / 4] = ~n & 0xffffu;
    g[(kRcg + 0x4) / 4] = (g[(kRcg + 0x4) / 4] & ~0x371fu) | 1u | (6u << 8) | (2u << 12);
    g[kRcg / 4] |= 1u;
    dsb();
    for (unsigned i = 0; i < 500 && (g[kRcg / 4] & 1u); ++i) IODelay(10);
    if (g[kRcg / 4] & 1u) return false;
    gClockHz = want;
    printf("PDX203 BLUE clock: source set to %u Hz\n", gClockHz);
    return true;
}

void setBaud(uint32_t baud) {
    uint32_t div = (gClockHz + baud * gSampling / 2) / (baud * gSampling);
    if (!div) div = 1;
    const uint32_t cfg = 1u | (div << 4);
    seW(M_CLK_CFG, cfg);
    seW(S_CLK_CFG, cfg);
    dsb();
}

void cancelRx() {
    if (!(se(GENI_STATUS) & (1u << 12))) return;
    seW(S_CMD_CTRL, 1u << 2);
    for (unsigned i = 0; i < 100 && !(se(S_IRQ_STATUS) & S_CMD_CANCEL); ++i) IODelay(10);
    seW(S_IRQ_CLEAR, se(S_IRQ_STATUS));
    if (se(GENI_STATUS) & (1u << 12)) {
        seW(S_CMD_CTRL, 1u << 1);
        for (unsigned i = 0; i < 100 && (se(S_CMD_CTRL) & (1u << 1)); ++i) IODelay(10);
        seW(S_IRQ_CLEAR, S_CMD_ABORT | 1u);
        seW(FORCE_DEFAULT, 1u);
    }
}

void cancelTx() {
    if (!(se(GENI_STATUS) & 1u)) return;
    seW(M_CMD_CTRL, 1u << 2);
    for (unsigned i = 0; i < 100 && !(se(M_IRQ_STATUS) & M_CMD_CANCEL); ++i) IODelay(10);
    if (!(se(M_IRQ_STATUS) & M_CMD_CANCEL)) {
        seW(M_CMD_CTRL, 1u << 1);
        for (unsigned i = 0; i < 100 && !(se(M_IRQ_STATUS) & M_CMD_ABORT); ++i) IODelay(10);
        seW(M_IRQ_CLEAR, M_CMD_ABORT);
    }
    seW(M_IRQ_CLEAR, M_CMD_CANCEL);
}

// qcom_geni_serial_port_setup + set_termios + startup, FIFO mode: 8N1, CTS/RFR flow control, 4 bytes per FIFO word
bool uartSetup() {
    const uint32_t fw = se(FW_REVISION), proto = (fw >> 8) & 0xffu;
    const uint32_t version = gWrapW.va[4 / 4];
    gSampling = version >= 0x20050000u ? 16u : 32u;
    const bool wide = version >= 0x300a0000u;
    gTxDepth = (se(HW_PARAM0) >> 16) & (wide ? 0xffu : 0x3fu);
    gRxDepth = (se(HW_PARAM1) >> 16) & (wide ? 0xffu : 0x3fu);
    S[1] = (int32_t)(proto << 24 | gTxDepth << 8 | gRxDepth);
    S[3] = (int32_t)version;
    printf("PDX203 BLUE se6: fw 0x%x (protocol %u), qup 0x%x, fifo tx %u rx %u words, io macro 0x%x\n", fw, proto, version, gTxDepth, gRxDepth,
           se(IO_MACRO));
    if (proto != 2 || !gTxDepth || !gRxDepth) return false;
    if (!clockSource()) return false;
    cancelRx();
    cancelTx();
    seW(RX_STALE, 10 * 16);
    // packing: 4 x 8 bits, LSB first, both directions
    seW(TX_PACK0, 0x4380e); seW(TX_PACK1, 0xc3e0e); seW(RX_PACK0, 0x4380e); seW(RX_PACK1, 0xc3e0e); seW(BYTE_GRAN, 0);
    // geni_se_init: interrupts cleared, clock gating, outputs, FIFO mode, watermarks
    seW(GSI_EVENT_EN, 0);
    seW(M_IRQ_CLEAR, ~0u); seW(S_IRQ_CLEAR, ~0u); seW(DMA_TX_IRQ_CLR, ~0u); seW(DMA_RX_IRQ_CLR, ~0u);
    seW(CGC_CTRL, se(CGC_CTRL) | 0x7fu);
    seW(DMA_GENERAL_CFG, se(DMA_GENERAL_CFG) | 0xfu);
    seW(OUTPUT_CTRL, 0x7fu);
    seW(FORCE_DEFAULT, 1u);
    seW(DMA_MODE, se(DMA_MODE) & ~1u);
    seW(RX_WATERMARK, 2);
    seW(RX_RFR_WATERMARK, gRxDepth - 2);
    seW(TX_WATERMARK, 0);
    // line: no parity, 8 bits, 1 stop bit, CTS honoured (CTS_MASK clear), RFR automatic, no loopback
    seW(TX_TRANS_CFG, se(TX_TRANS_CFG) & ~3u);
    seW(RX_TRANS_CFG, se(RX_TRANS_CFG) & ~(1u << 3));
    seW(TX_PARITY, 0); seW(RX_PARITY, 0);
    seW(TX_WORD_LEN, 8); seW(RX_WORD_LEN, 8); seW(TX_STOP_LEN, 0);
    seW(LOOPBACK, 0);
    seW(MANUAL_RFR, 0);
    setBaud(115200);
    // the engine's line to the GIC: M and S interrupts; which of them is enabled per event below
    seW(M_IRQ_EN, M_CMD_DONE);
    seW(S_IRQ_EN, S_RX_WATERMARK | S_RX_LAST | S_RX_WR_ERR | S_GP_IRQS);
    seW(SE_IRQ_EN, (1u << 2) | (1u << 3));
    seW(S_CMD0, kUartStartRead << 27);
    dsb();
    return true;
}

// ---- interrupt and the thread's wait
volatile uint32_t gWake;
bool gIrq;
void seIrq(void *) {
    // level line: masked at the engine until the thread has drained what raised it
    seW(SE_IRQ_EN, 0);
    dsb();
    S[9] = S[9] + 1;
    __atomic_store_n(&gWake, 1u, __ATOMIC_RELEASE);
    thread_wakeup(event_t(&gWake));
}
void kick() {
    __atomic_store_n(&gWake, 1u, __ATOMIC_RELEASE);
    thread_wakeup(event_t(&gWake));
}
bool gTxActive;
void waitWork(unsigned ms) {
    // a command-done nobody waits for would hold the level line up
    if (!gTxActive) seW(M_IRQ_CLEAR, M_CMD_DONE | M_TX_WATERMARK);
    if (gIrq) seW(SE_IRQ_EN, (1u << 2) | (1u << 3));
    if (__atomic_exchange_n(&gWake, 0u, __ATOMIC_ACQ_REL)) return;
    assert_wait_timeout(event_t(&gWake), THREAD_UNINT, ms, NSEC_PER_MSEC);
    if (__atomic_exchange_n(&gWake, 0u, __ATOMIC_ACQ_REL)) {clear_wait(current_thread(), THREAD_AWAKENED); return;}
    thread_block(THREAD_CONTINUE_NULL);
    gWake = 0;
}

// ---- receive: the FIFO into a ring
constexpr size_t kRing = 64 * 1024;
uint8_t *gRx;
size_t gRxHead, gRxTail;   // bytes [tail, head) are held
size_t rxHeld() {return gRxHead - gRxTail;}
void rxDrain() {
    const uint32_t status = se(S_IRQ_STATUS);
    if (status) seW(S_IRQ_CLEAR, status);
    if (status & (S_RX_WR_ERR | S_GP_IRQS)) S[15] = S[15] + 1;
    for (;;) {
        const uint32_t fifo = se(RX_FIFO_STATUS);
        uint32_t words = fifo & 0x1ffffffu;
        if (!words) break;
        const bool last = fifo >> 31;
        const uint32_t lastBytes = (fifo >> 28) & 7u;
        for (uint32_t i = 0; i < words; ++i) {
            const uint32_t w = se(RX_FIFO);
            const uint32_t n = (i + 1 == words && last && lastBytes) ? lastBytes : 4u;
            for (uint32_t b = 0; b < n; ++b) {
                if (rxHeld() < kRing) {gRx[gRxHead % kRing] = (uint8_t)(w >> (8 * b)); ++gRxHead;}
                else S[15] = S[15] + 1;
            }
        }
        S[10] = S[10] + (int32_t)(words * 4);
    }
}

// ---- transmit: one engine command per chunk, the FIFO fed as it empties
const uint8_t *gTxData;
size_t gTxLen, gTxSent;
void txPump() {
    if (!gTxActive) return;
    const uint32_t m = se(M_IRQ_STATUS);
    if (m & M_CMD_DONE) {
        seW(M_IRQ_CLEAR, M_CMD_DONE | M_TX_WATERMARK);
        if (gTxSent >= gTxLen) {gTxActive = false; return;}
    }
    uint32_t inFifo = se(TX_FIFO_STATUS) & 0x0fffffffu;
    while (gTxSent < gTxLen && inFifo < gTxDepth) {
        uint32_t w = 0;
        for (uint32_t b = 0; b < 4 && gTxSent < gTxLen; ++b) w |= (uint32_t)gTxData[gTxSent++] << (8 * b);
        seW(TX_FIFO, w);
        ++inFifo;
    }
    dsb();
}
void txStart(const uint8_t *data, size_t len) {
    gTxData = data; gTxLen = len; gTxSent = 0; gTxActive = true;
    seW(M_IRQ_CLEAR, M_CMD_DONE | M_TX_WATERMARK);
    seW(TX_TRANS_LEN, (uint32_t)len);
    seW(M_CMD0, kUartStartTx << 27);
    dsb();
    txPump();
    S[11] = S[11] + (int32_t)len;
}
// sends and waits for the engine to finish (the last byte has left the shift register when the command is done)
bool txBlocking(const uint8_t *data, size_t len, unsigned timeoutMs = 2000) {
    txStart(data, len);
    const uint64_t end = nowMs() + timeoutMs;
    while (gTxActive && nowMs() < end) {
        txPump();
        if (gTxActive) {rxDrain(); IODelay(50);}
    }
    if (gTxActive) {cancelTx(); gTxActive = false; return false;}
    return true;
}

// ---- in-band sleep. The controller runs Qualcomm's IBS whatever the NVM says: idle, it sends SLEEP_IND (0xfe) and sleeps; to send it
// sends WAKE_IND (0xfd) until the host answers WAKE_ACK (0xfc); to send to it while it sleeps the host sends WAKE_IND and waits for its
// WAKE_ACK. The host side never announces sleep of its own, so the controller may sleep but the host never does.
bool gCtrlAsleep, gIbsAckDue;
uint32_t gIbsSleeps, gIbsWakes, gIbsAcks;
void ibsByte(uint8_t b) {
    if (b == 0xfe) {gCtrlAsleep = true; ++gIbsSleeps;}
    else if (b == 0xfd) {gCtrlAsleep = false; gIbsAckDue = true; ++gIbsWakes;}
    else {gCtrlAsleep = false; ++gIbsAcks;}
}

// ---- H4 packets out of the receive ring (setup phase)
// Returns the length of the complete packet at the ring's tail, 0 when incomplete; bytes that start no packet are dropped.
size_t packetAtTail() {
    for (;;) {
        if (!rxHeld()) return 0;
        const uint8_t type = gRx[gRxTail % kRing];
        size_t header = type == 4 ? 3 : type == 2 ? 5 : type == 3 ? 4 : 0;
        if (type == 0xfe || type == 0xfd || type == 0xfc) {   // in-band sleep (hci_qca.c HCI_IBS_*): between packets, never forwarded
            ibsByte(type);
            ++gRxTail;
            continue;
        }
        if (!header) {
            if (S[15] < 64) printf("PDX203 BLUE stray byte %02x\n", type);
            ++gRxTail; S[15] = S[15] + 1; continue;
        }
        if (rxHeld() < header) return 0;
        size_t payload = 0;
        if (type == 4) payload = gRx[(gRxTail + 2) % kRing];
        else if (type == 2) payload = gRx[(gRxTail + 3) % kRing] | (size_t)gRx[(gRxTail + 4) % kRing] << 8;
        else payload = gRx[(gRxTail + 3) % kRing];
        if (rxHeld() < header + payload) return 0;
        return header + payload;
    }
}
void copyOut(uint8_t *to, size_t n) {for (size_t i = 0; i < n; ++i) to[i] = gRx[(gRxTail + i) % kRing];}

// Waits for an event: a Command Complete or Status for opcode, or (vendor) an 0xFF event. Returns its parameters (after the opcode for
// Command Complete) in out. Other packets are logged and dropped.
int waitEvent(uint16_t opcode, bool vendor, uint8_t *out, size_t outMax, size_t *outLen, unsigned timeoutMs) {
    const uint64_t end = nowMs() + timeoutMs;
    uint8_t p[260];
    for (;;) {
        rxDrain();
        while (size_t n = packetAtTail()) {
            if (n > sizeof p) {gRxTail += n; continue;}
            copyOut(p, n);
            gRxTail += n;
            if (p[0] != 4) continue;
            const uint8_t ev = p[1], plen = p[2];
            if (ev == 0x0e && plen >= 3 && (p[4] | p[5] << 8) == opcode && !vendor) {
                const size_t k = plen - 3 < outMax ? plen - 3 : outMax;
                memcpy(out, p + 6, k); *outLen = k;
                return 0;
            }
            if (ev == 0x0f && plen >= 4 && (p[5] | p[6] << 8) == opcode) {*outLen = 0; return p[3] ? -p[3] : 1;}
            if (ev == 0xff && vendor) {
                const size_t k = plen < outMax ? plen : outMax;
                memcpy(out, p + 3, k); *outLen = k;
                return 0;
            }
            printf("PDX203 BLUE event %02x len %u (%02x %02x %02x %02x) ignored\n", ev, plen, plen > 0 ? p[3] : 0, plen > 1 ? p[4] : 0,
                   plen > 2 ? p[5] : 0, plen > 3 ? p[6] : 0);
        }
        if (nowMs() >= end) return -1000;
        IOSleep(1);
    }
}
uint8_t gCmd[300];
int command(uint16_t opcode, const uint8_t *params, uint8_t len, uint8_t *out, size_t outMax, size_t *outLen, bool vendorEvent = false,
            unsigned timeoutMs = 2000) {
    gCmd[0] = 1; gCmd[1] = (uint8_t)opcode; gCmd[2] = (uint8_t)(opcode >> 8); gCmd[3] = len;
    if (len) memcpy(gCmd + 4, params, len);
    if (!txBlocking(gCmd, 4u + len)) return -1001;
    if (!out) return 0;
    return waitEvent(opcode, vendorEvent, out, outMax, outLen, timeoutMs);
}

// ---- firmware: the rampatch and the NVM, 243-byte segments of EDL_PATCH_TLV_REQ (0xfc00 0x1e)
bool sendTlv(const uint8_t *data, size_t size, bool patch) {
    uint8_t mode = 0;
    if (patch) {
        if (size < 4 + 24) return false;
        mode = data[4 + 10];   // tlv_type_patch.download_mode: 3 = no event but for the last segment
    }
    size_t at = 0;
    unsigned segment = 0;
    while (at < size) {
        const size_t n = size - at < 243 ? size - at : 243;
        uint8_t p[245];
        p[0] = 0x1e; p[1] = (uint8_t)n;
        memcpy(p + 2, data + at, n);
        at += n;
        const bool lastOne = at >= size || n < 243;
        if (mode == 3 && !lastOne) {
            if (command(0xfc00, p, (uint8_t)(n + 2), nullptr, 0, nullptr)) {printf("PDX203 BLUE tlv segment %u: send failed\n", segment); return false;}
        } else {
            uint8_t r[16]; size_t rl = 0;
            const int rc = command(0xfc00, p, (uint8_t)(n + 2), r, sizeof r, &rl, false, 3000);
            if (rc || rl < 2 || r[0] != 0 || r[1] != 0x1e) {
                printf("PDX203 BLUE tlv segment %u: rc %d len %zu resp %02x %02x\n", segment, rc, rl, rl > 0 ? r[0] : 0, rl > 1 ? r[1] : 0);
                return false;
            }
        }
        ++segment;
        // the engine has sent each segment before the next starts; the controller paces through RFR
        rxDrain();
    }
    return true;
}

// The address: the add-on's wifi/bt_address.txt ("xx:xx:xx:xx:xx:xx", the same one the build puts in bluetoothd's launchd job as
// BT_DEVICE_ADDRESS: bluetoothd does not program it over UART), else one derived from the chip id.
int hexDigit(char c) {return c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;}
bool addonAddress(uint8_t a[6]) {
    const uint8_t *t = nullptr; size_t n = 0;
    if (sheng_wifi_file("bt_address.txt", &t, &n) || !t || n < 17) return false;
    for (unsigned i = 0; i < 6; ++i) {
        const int h = hexDigit((char)t[3 * i]), l = hexDigit((char)t[3 * i + 1]);
        if (h < 0 || l < 0) return false;
        a[i] = (uint8_t)(h << 4 | l);
    }
    return true;
}
bool boardAddress(uint8_t a[6]) {
    if (addonAddress(a)) return true;
    IORegistryEntry *chosen = IORegistryEntry::fromPath("/chosen", gIODTPlane);
    OSData *id = chosen ? OSDynamicCast(OSData, chosen->getProperty("unique-chip-id")) : nullptr;
    uint64_t seed = 0;
    if (id && id->getLength() >= sizeof seed) memcpy(&seed, id->getBytesNoCopy(), sizeof seed);
    OSSafeReleaseNULL(chosen);
    if (!seed) return false;
    // as the WiFi address (Pdx203Wifi.cpp boardMac), salted, so the two differ
    uint64_t h = 0xcbf29ce484222325ull;
    for (unsigned i = 0; i < 8; ++i) h = (h ^ ((seed >> (8 * i)) & 0xff)) * 0x100000001b3ull;
    h = (h ^ 0x42) * 0x100000001b3ull;
    for (unsigned i = 0; i < 6; ++i) a[i] = (uint8_t)(h >> (8 * i));   // a[0] is the address's first (most significant) byte
    a[0] &= 0xfc;
    return true;
}

// NVM edits (btqca.c qca_tlv_check_data, without enabling sleep): HCI tag 17 byte 0 bit 7 (in-band sleep) cleared, byte 1 the
// operating speed; deep sleep tag 27 bit 0 cleared; tag 2 the board's address (least significant byte first).
uint8_t *nvmPatched(const uint8_t *data, size_t size, uint8_t baudIndex, const uint8_t *address) {
    uint8_t *d = static_cast<uint8_t *>(IOMallocData(size));
    if (!d) return nullptr;
    memcpy(d, data, size);
    size_t o = 4;
    uint32_t tl; memcpy(&tl, d, 4);
    uint32_t type = tl & 0xff, length = tl >> 8;
    if (type == 4 && size >= 8) {memcpy(&tl, d + 4, 4); type = tl & 0xff; length = tl >> 8; o = 8;}
    if (type != 2 || o + length > size) return d;
    for (size_t i = 0; i + 12 <= length;) {
        uint16_t tag, tagLen;
        memcpy(&tag, d + o + i, 2); memcpy(&tagLen, d + o + i + 2, 2);
        uint8_t *v = d + o + i + 12;
        if (o + i + 12 + tagLen > size) break;
        if (tag == 17 && tagLen >= 3) {v[0] &= 0x7f; v[1] = baudIndex;}
        if (tag == 27 && tagLen >= 1) v[0] &= 0xfe;
        if (tag == 2 && tagLen == 6 && address) for (unsigned b = 0; b < 6; ++b) v[b] = address[5 - b];
        i += 12 + tagLen;
    }
    return d;
}

bool controllerUp() {
    uint8_t r[64]; size_t rl = 0;
    // QCA version (EDL_PATCH_VER_REQ): for the QCA6390 the answer is a Command Complete {status, 0x19, ?, product id, patch, rom, soc id}
    auto version = [&](const char *when) -> bool {
        const uint8_t q = 0x19;
        int rc = command(0xfc00, &q, 1, r, sizeof r, &rl, false, 1000);
        if (rc || rl < 15) {printf("PDX203 BLUE version (%s): rc %d len %zu\n", when, rc, rl); return false;}
        uint32_t product, soc; uint16_t patch, rom;
        memcpy(&product, r + 3, 4); memcpy(&patch, r + 7, 2); memcpy(&rom, r + 9, 2); memcpy(&soc, r + 11, 4);
        S[4] = (int32_t)product; S[5] = (int32_t)soc; S[6] = (int32_t)((uint32_t)rom << 16 | patch);
        printf("PDX203 BLUE version (%s): product 0x%x soc 0x%x rom 0x%x patch 0x%x\n", when, product, soc, rom, patch);
        return true;
    };
    S[0] = 30;
    if (!version("115200")) {fail(30); return false;}
    // 3 Mbit/s (QCA_BAUDRATE_3000000 = 14): the command at the old speed, 300 ms for the controller, then the host
    S[0] = 31;
    const uint8_t baud[5] = {1, 0x48, 0xfc, 1, 14};
    if (!txBlocking(baud, sizeof baud)) {fail(31); return false;}
    IOSleep(300);
    setBaud(3000000);
    IOSleep(10);
    rxDrain(); gRxTail = gRxHead;   // whatever came during the switch
    S[0] = 32;
    if (!version("3M")) {fail(32); return false;}
    const uint32_t soc = (uint32_t)S[5];
    const uint16_t romVer = (uint16_t)((uint32_t)S[6] >> 16);
    const uint32_t socVer = (soc << 16) | romVer;
    const uint8_t rom = (uint8_t)(((socVer & 0xf00) >> 4) | (socVer & 0xf));
    char name[32];
    const uint8_t *fw = nullptr; size_t fwBytes = 0;
    snprintf(name, sizeof name, "htbtfw%02x.tlv", rom);
    S[0] = 33;
    if (sheng_wifi_file(name, &fw, &fwBytes) || !fw) {printf("PDX203 BLUE no %s in the add-on\n", name); fail(33); return false;}
    S[7] = (int32_t)fwBytes;
    const uint64_t t0 = nowMs();
    if (!sendTlv(fw, fwBytes, true)) {fail(34); return false;}
    printf("PDX203 BLUE rampatch %s: %zu bytes in %llu ms\n", name, fwBytes, nowMs() - t0);
    IOSleep(10);
    S[0] = 35;
    snprintf(name, sizeof name, "htnv%02x.bin", rom);
    if (sheng_wifi_file(name, &fw, &fwBytes) || !fw) {printf("PDX203 BLUE no %s in the add-on\n", name); fail(35); return false;}
    uint8_t address[6];
    const bool haveAddress = boardAddress(address);
    uint8_t *nvm = nvmPatched(fw, fwBytes, 14, haveAddress ? address : nullptr);
    if (!nvm) {fail(35); return false;}
    S[8] = (int32_t)fwBytes;
    const bool ok = sendTlv(nvm, fwBytes, false);
    IOFreeData(nvm, fwBytes);
    if (!ok) {fail(36); return false;}
    printf("PDX203 BLUE nvm %s: %zu bytes\n", name, fwBytes);
    S[0] = 37;
    // soc logging off (QCA_DISABLE_LOGGING 0xfc17 {0x14, 0}), then HCI reset
    const uint8_t logging[2] = {0x14, 0x00};
    int rc = command(0xfc17, logging, 2, r, sizeof r, &rl);
    if (rc) printf("PDX203 BLUE logging off: rc %d\n", rc);
    rc = command(0x0c03, nullptr, 0, r, sizeof r, &rl);
    if (rc || rl < 1 || r[0]) {printf("PDX203 BLUE reset: rc %d status %02x\n", rc, rl ? r[0] : 0xff); fail(37); return false;}
    S[0] = 38;
    rc = command(0x1001, nullptr, 0, r, sizeof r, &rl);   // Read Local Version Information
    if (!rc && rl >= 9)
        printf("PDX203 BLUE local version: hci %u rev 0x%x lmp %u manufacturer %u subversion 0x%x\n", r[1], r[2] | r[3] << 8, r[4],
               r[5] | r[6] << 8, r[7] | r[8] << 8);
    rc = command(0x1009, nullptr, 0, r, sizeof r, &rl);   // Read BD_ADDR
    if (!rc && rl >= 7)
        printf("PDX203 BLUE address %02x:%02x:%02x:%02x:%02x:%02x\n", r[6], r[5], r[4], r[3], r[2], r[1]);
    S[0] = 40;
    return true;
}

// ---- the bridge: controller <-> host (the control socket's client, or the tty)
// Both directions are framed into H4 packets. The host's vendor commands (OGF 0x3f) are answered here with Command Complete, status
// 0x01 (unknown command), and never reach the controller: iOS speaks Broadcom's vendor set, whose opcodes mean other things to a QCA6390
// (0xfc48 is its UART speed). Everything else passes unchanged.
uint8_t gTxBuf[8192];             // the packets on the wire (as large as gCtlQ)
uint8_t gHost[2048];              // the host's bytes not yet a whole packet
size_t gHostLen;
uint8_t gCtlQ[8192];              // whole packets for the controller
size_t gCtlLen;
uint8_t gToHost[8192];            // whole packets for the host
size_t gToHostLen;
uint32_t gLogged, gEvLogged, gOtherLogged, gAclLogged;
uint32_t gRemap;                  // LE Set Random Address commands sent for the host's 0xfd48, whose answers are dropped
volatile uint32_t gResetFraming;

void opened(int what) {
    printf("PDX203 BLUE host %s\n", what == 0 ? "tty closed" : what == 1 ? "tty opened" : what == 2 ? "socket connected" : "socket disconnected");
    if (what == 1 || what == 2) __atomic_store_n(&gResetFraming, 1u, __ATOMIC_RELEASE);
    kick();
}

size_t hostPacketLength(const uint8_t *b, size_t n, bool *bad) {
    *bad = false;
    if (!n) return 0;
    switch (b[0]) {
    case 1: return n < 4 ? 0 : 4u + b[3];
    case 2: return n < 5 ? 0 : 5u + (b[3] | (size_t)b[4] << 8);
    case 3: return n < 4 ? 0 : 4u + b[3];
    case 5: return n < 5 ? 0 : 5u + ((b[3] | (size_t)b[4] << 8) & 0x3fff);
    default: *bad = true; return 0;
    }
}

void toHost(const uint8_t *p, size_t n) {memcpy(gToHost + gToHostLen, p, n); gToHostLen += n; S[14] = S[14] + 1;}

void hostPackets() {
    for (;;) {
        bool bad;
        const size_t n = hostPacketLength(gHost, gHostLen, &bad);
        if (bad) {memmove(gHost, gHost + 1, --gHostLen); S[15] = S[15] + 1; continue;}
        if (!n || n > gHostLen) {
            if (n > sizeof gHost) {gHostLen = 0; S[15] = S[15] + 1;}   // larger than any packet the controller takes
            return;
        }
        if ((gHost[0] == 2 || gHost[0] == 5) && gAclLogged < 400) {
            ++gAclLogged;
            printf("PDX203 BLUE host acl %u:", (unsigned)n);
            for (size_t i = 1; i < n && i < 21; ++i) printf(" %02x", gHost[i]);
            printf("\n");
        }
        if (gHost[0] == 1) {
            const uint16_t op = (uint16_t)(gHost[1] | gHost[2] << 8);
            S[13] = S[13] + 1;
            // the periodic scan and inquiry traffic and the vendor scan tuning are not logged
            const bool routine = op == 0x200b || op == 0x200c || op == 0x0401 || op == 0x0402 || op == 0xfce9 || op == 0xfce2 ||
                                 op == 0xfd47 || op == 0xfd48 || op == 0x2005;
            if (!routine && gLogged < 3000) {
                ++gLogged;
                printf("PDX203 BLUE host cmd %04x len %u:", op, gHost[3]);
                for (unsigned i = 0; i < gHost[3] && i < 12; ++i) printf(" %02x", gHost[4 + i]);
                printf("%s\n", (op >> 10) == 0x3f ? " (answered here)" : "");
            }
            // Write Default Erroneous Data Reporting (0x0c5b): Apple's stack requires it in its init and the QCA6390 lacks it; it only
            // asks the controller to report erroneous SCO data, so it is answered here (with its supported bit set in 0x1002's answer)
            if (op == 0x0c5b || op == 0x0c5a) {
                if (gToHostLen + 8 > sizeof gToHost) return;
                const uint8_t cc[8] = {4, 0x0e, (uint8_t)(op == 0x0c5a ? 5 : 4), 1, (uint8_t)op, 0x0c, 0x00, 0x00};   // read: disabled
                toHost(cc, op == 0x0c5a ? 8 : 7);
                memmove(gHost, gHost + n, gHostLen -= n);
                continue;
            }
            // LE Read Periodic Advertiser List Size (0x204a), in Apple's init: to the QCA6390 it is an extended-advertising command, after
            // which the legacy LE commands Apple uses (Set Scan Parameters, Read Advertising Channel Tx Power, ...) are Command Disallowed
            // (Core 5.x Vol 4 Part E 3.1.1; Broadcom's firmware does not enforce it). Answered here so the controller stays legacy.
            if (op == 0x204a) {
                if (gToHostLen + 8 > sizeof gToHost) return;
                const uint8_t cc[8] = {4, 0x0e, 5, 1, 0x4a, 0x20, 0x00, 8};
                toHost(cc, sizeof cc);
                memmove(gHost, gHost + n, gHostLen -= n);
                continue;
            }
            // Broadcom's set-random-address (0xfd48 {0x01, address}; Apple's resolvable private address): the address also goes to the
            // controller as the standard LE Set Random Address (0x2005), whose Command Complete is dropped (gRemap); the host still gets
            // the vendor command's "unknown" answer below (a success needs return parameters it parses: it aborted on a bare status).
            // Without the address, scanning with a random own address fails (0x12).
            if (op == 0xfd48 && gHost[3] == 7 && gHost[4] == 0x01) {
                if (gCtlLen + 10 > sizeof gCtlQ) return;
                const uint8_t cmd[4] = {1, 0x05, 0x20, 6};
                memcpy(gCtlQ + gCtlLen, cmd, 4); memcpy(gCtlQ + gCtlLen + 4, gHost + 5, 6); gCtlLen += 10;
                ++gRemap;
            }
            if ((op >> 10) == 0x3f) {
                if (gToHostLen + 7 > sizeof gToHost) return;   // after the host has read some
                const uint8_t cc[7] = {4, 0x0e, 4, 1, (uint8_t)op, (uint8_t)(op >> 8), 0x01};
                toHost(cc, sizeof cc);
                memmove(gHost, gHost + n, gHostLen -= n);
                continue;
            }
        }
        if (gCtlLen + n > sizeof gCtlQ) return;
        memcpy(gCtlQ + gCtlLen, gHost, n); gCtlLen += n;
        memmove(gHost, gHost + n, gHostLen -= n);
    }
}

void controllerPackets() {
    while (size_t n = packetAtTail()) {
        if (n > sizeof gToHost) {gRxTail += n; S[15] = S[15] + 1; continue;}
        if (gToHostLen + n > sizeof gToHost) return;
        uint8_t *p = gToHost + gToHostLen;
        copyOut(p, n);
        gRxTail += n;
        if (gRemap && p[0] == 4 && p[1] == 0x0e && p[2] >= 4 && (p[4] | p[5] << 8) == 0x2005) {
            if (p[6]) printf("PDX203 BLUE le set random address: status %02x\n", p[6]);
            --gRemap;
            continue;
        }
        // Read Local Supported Commands: octet 18 bits 2 and 3, Read/Write Default Erroneous Data Reporting (answered by hostPackets)
        if (p[0] == 4 && p[1] == 0x0e && p[2] >= 4 + 64 && (p[4] | p[5] << 8) == 0x1002 && p[6] == 0) p[7 + 18] |= 0x0c;
        // other events (connections, security, pairing), all but the advertising, inquiry and completed-packet traffic; controller ACL
        if (p[0] == 4 && p[1] != 0x0e && p[1] != 0x0f && p[1] != 0x02 && p[1] != 0x22 && p[1] != 0x2f && p[1] != 0x13 &&
            !(p[1] == 0x3e && (p[3] == 0x02 || p[3] == 0x0d || p[3] == 0x0b)) && gOtherLogged < 3000) {
            ++gOtherLogged;
            printf("PDX203 BLUE event %02x:", p[1]);
            for (size_t i = 2; i < n && i < 22; ++i) printf(" %02x", p[i]);
            printf("\n");
        }
        if (p[0] == 2 && gAclLogged < 400) {
            ++gAclLogged;
            printf("PDX203 BLUE ctl acl %u:", (unsigned)n);
            for (size_t i = 1; i < n && i < 21; ++i) printf(" %02x", p[i]);
            printf("\n");
        }
        // every failed command, and the first 400 command events
        const bool failed = p[0] == 4 && ((p[1] == 0x0e && p[2] >= 4 && p[6]) || (p[1] == 0x0f && p[2] >= 4 && p[3]));
        const uint16_t evOp = p[0] == 4 && p[1] == 0x0e && p[2] >= 3 ? (uint16_t)(p[4] | p[5] << 8)
                              : p[0] == 4 && p[1] == 0x0f && p[2] >= 4 ? (uint16_t)(p[5] | p[6] << 8) : 0;
        const bool routineEv = evOp == 0x200b || evOp == 0x200c || evOp == 0x0401 || evOp == 0x0402;
        if (p[0] == 4 && (failed || !routineEv) && gEvLogged < 3000 && (p[1] == 0x0e || p[1] == 0x0f)) {
            ++gEvLogged;
            if (p[1] == 0x0e && p[2] >= 4) printf("PDX203 BLUE event cc %04x status %02x len %u\n", p[4] | p[5] << 8, p[6], p[2]);
            if (p[1] == 0x0f && p[2] >= 4) printf("PDX203 BLUE event cs %04x status %02x\n", p[5] | p[6] << 8, p[3]);
        }
        gToHostLen += n;
        S[14] = S[14] + 1;
    }
}

void bridge() {
    gRxTail = gRxHead;
    uint64_t nextStats = 0, wakeAt = 0;
    unsigned wakeTries = 0;
    int32_t last[4] = {-1, -1, -1, -1};
    for (;;) {
        if (nowMs() >= nextStats) {
            nextStats = nowMs() + 5000;
            if (S[10] != last[0] || S[11] != last[1] || S[13] != last[2] || S[15] != last[3]) {
                // (rx/tx bytes, host commands or errors changed since the last row)
                last[0] = S[10]; last[1] = S[11]; last[2] = S[13]; last[3] = S[15];
                printf("PDX203 BLUE stats: rx %d tx %d host cmds %d to host %d errors %d irqs %d | tx active %d len %zu sent %zu | se %x m %x s %x "
                       "txfifo %x rxfifo %x | ibs sleeps %u wakes %u acks %u asleep %d\n", (int)S[10], (int)S[11], (int)S[13], (int)S[14], (int)S[15], (int)S[9], gTxActive, gTxLen,
                       gTxSent, se(GENI_STATUS), se(M_IRQ_STATUS), se(S_IRQ_STATUS), se(TX_FIFO_STATUS), se(RX_FIFO_STATUS), gIbsSleeps, gIbsWakes, gIbsAcks, gCtrlAsleep);
            }
        }
        if (__atomic_exchange_n(&gResetFraming, 0u, __ATOMIC_ACQ_REL)) {
            // a new host: nothing half-sent is kept from the last one
            gHostLen = 0; gToHostLen = 0; gCtlLen = 0; gRemap = 0;
        }
        rxDrain();
        controllerPackets();
        // to the host
        if (gToHostLen) {
            size_t taken;
            if (pdx203_bt_tty_is_open()) taken = pdx203_bt_tty_rx(gToHost, gToHostLen);
            else {taken = gToHostLen; S[12] = S[12] + (int32_t)taken;}
            if (taken) memmove(gToHost, gToHost + taken, gToHostLen -= taken);
        }
        // from the host
        if (gHostLen < sizeof gHost) {
            const size_t n = pdx203_bt_tty_tx(gHost + gHostLen, sizeof gHost - gHostLen);
            gHostLen += n;
        }
        hostPackets();
        if (gTxActive) txPump();
        else if (gIbsAckDue) {
            static const uint8_t ack = 0xfc;
            gIbsAckDue = false;
            txStart(&ack, 1);
        } else if (gCtlLen && gCtrlAsleep) {
            // wake it: WAKE_IND every 10 ms until its WAKE_ACK (or its own WAKE_IND); after 100 tries send anyway
            if (nowMs() >= wakeAt) {
                static const uint8_t ind = 0xfd;
                if (++wakeTries > 100) {gCtrlAsleep = false; printf("PDX203 BLUE ibs: no wake ack, sending anyway\n");}
                else {wakeAt = nowMs() + 10; txStart(&ind, 1);}
            }
        } else if (gCtlLen) {
            wakeTries = 0;
            memcpy(gTxBuf, gCtlQ, gCtlLen);
            txStart(gTxBuf, gCtlLen);
            gCtlLen = 0;
        }
        waitWork(rxHeld() || gTxActive || gToHostLen || gHostLen ? 1 : 50);
    }
}

// What Apple's device tree has as the UART's "bluetooth" child, as a registered service of that name: bluetoothd finds the controller
// with IOServiceNameMatching("bluetooth") and reads local-mac-address (all zero or missing: "No bluetooth on this device", for good),
// transport-encoding (3: H4 with BT_WAKE) and transport-speed from it.
void publishService() {
    uint8_t a[6];
    if (!boardAddress(a)) {printf("PDX203 BLUE no address for the bluetooth service\n"); return;}
    IOService *service = new IOService;
    if (!service || !service->init()) {OSSafeReleaseNULL(service); return;}
    service->setName("bluetooth");
    service->setProperty("local-mac-address", a, 6);
    const uint32_t encoding = 3, speed = 3000000, vendor = 684, product = 2336;
    service->setProperty("transport-encoding", (void *)&encoding, 4);
    service->setProperty("transport-speed", (void *)&speed, 4);
    service->setProperty("vendor-id", (void *)&vendor, 4);
    service->setProperty("product-id", (void *)&product, 4);
    service->setProperty("compatible", (void *)"bluetooth,n88", sizeof "bluetooth,n88");
    service->setProperty("device_type", (void *)"bluetooth", sizeof "bluetooth");
    IOService *resources = IOService::getResourceService();
    if (resources && service->attach(resources)) {
        service->registerService();
        printf("PDX203 BLUE service \"bluetooth\" registered\n");
    }
    service->release();
}

void btThread(void *arg, wait_result_t) {
    thread_set_thread_name(current_thread(), "pdx203-blue");
    const unsigned mode = (unsigned)reinterpret_cast<uintptr_t>(arg);
    if (!mapWindow(gGccW, kGcc, 0x60000) || !mapWindow(gTlmmW, kTlmm, 0x20000) || !mapWindow(gSeW, kSe, 0x4000) ||
        !mapWindow(gWrapW, kWrap, 0x1000)) {fail(1); printf("PDX203 BLUE windows not mapped\n"); return;}
    // the windows stay mapped for the rest of the boot
    S[0] = 1;
    printf("PDX203 BLUE survey: bt_en cfg %x io %x, se6 fw %x status %x, rcg cfg %x dfs %x, vote %x\n", gTlmmW.va[kBtEn * 0x1000 / 4],
           gTlmmW.va[(kBtEn * 0x1000 + 4) / 4], se(FW_REVISION), se(GENI_STATUS), gGccW.va[(kRcg + 4) / 4], gGccW.va[(kRcg + 0x14) / 4],
           gGccW.va[kGccVote / 4]);
    if (mode < 2) return;
    // the chip's rails: the WiFi bring-up's (stage 12: rails on and WLAN_EN high)
    unsigned waited = 0;
    while (pdx203_wifi_stats[0] < 12 && pdx203_wifi_stats[0] >= 0 && waited < 1200) {IOSleep(100); ++waited;}
    if (pdx203_wifi_stats[0] < 12) {printf("PDX203 BLUE no rails: wifi stage %d\n", (int)pdx203_wifi_stats[0]); fail(2); return;}
    S[0] = 2;
    gRx = static_cast<uint8_t *>(IOMallocData(kRing));
    if (!gRx) {fail(3); return;}
    // BT_EN low; the UART's pins to the engine (CTS in, RFR out pulled down, TX pulled up, RX)
    pin(kBtEn, 0, 1, true, 0);
    const uint64_t low = nowMs();
    pin(kCts, 1, 0, false, -1);
    pin(kRfr, 1, 1, false, -1);
    pin(kTx, 1, 3, false, -1);
    pin(kRx, 1, 0, false, -1);
    S[0] = 3;
    if (!uartSetup()) {fail(4); printf("PDX203 BLUE uart setup failed\n"); return;}
    S[2] = (int32_t)(gClockHz / 1000);
    S[0] = 5;
    if (armcpu_spi_attach) {
        int rc = -1;
        for (unsigned tries = 0; tries < 40; ++tries) {
            rc = armcpu_spi_attach(32 + kSpi, false, seIrq, nullptr, nullptr, nullptr);
            if (rc != (int)kIOReturnNotReady) break;
            IOSleep(250);
        }
        gIrq = rc == 0;
        printf("PDX203 BLUE interrupt: SPI %u %s (%d)\n", kSpi, gIrq ? "attached" : "not attached, polling", rc);
    }
    // BT_EN: low >= 100 ms, then high; the controller boots in about 100 ms
    const uint64_t lowFor = nowMs() - low;
    if (lowFor < 150) IOSleep((uint32_t)(150 - lowFor));
    pin(kBtEn, 0, 1, true, 1);
    IOSleep(150);
    rxDrain();
    if (rxHeld()) {
        printf("PDX203 BLUE after BT_EN: %zu bytes:", rxHeld());
        for (size_t i = 0; i < rxHeld() && i < 16; ++i) printf(" %02x", gRx[(gRxTail + i) % kRing]);
        printf("\n");
        gRxTail = gRxHead;
    }
    S[0] = 10;
    if (!controllerUp()) {
        printf("PDX203 BLUE bring-up failed at %d | se status %x m %x s %x rxfifo %x\n", (int)S[0], se(GENI_STATUS), se(M_IRQ_STATUS),
               se(S_IRQ_STATUS), se(RX_FIFO_STATUS));
        return;
    }
    if (mode < 3) return;
    publishService();
    const int rc = pdx203_bt_tty_attach(kick, opened);
    printf("PDX203 BLUE tty: %s (%d)\n", rc ? "failed" : "/dev/cu.bluetooth and /dev/uart.bluetooth", rc);
    if (rc) {fail(41); return;}
    S[0] = 41;
    bridge();
}
}  // namespace

extern "C" void pdx203_bluetooth_probe(void) {
    unsigned mode = 3;
    PE_parse_boot_argn("pdx203-blue", &mode, sizeof mode);
    if (!mode || S[0]) return;
    thread_t thread = nullptr;
    if (kernel_thread_start(btThread, reinterpret_cast<void *>(uintptr_t(mode)), &thread) == KERN_SUCCESS) thread_deallocate(thread);
    else fail(7);
}
