// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// The pdx203 touchscreen: the Samsung sec_ts in the panel, on I2C address 0x48 of QUP serial engine 13 (0x00a94000, wrapper 1 SE5),
// read by polling and published to NativeHidInput through the native_touch_* seqlock (the contract of UniXNU-alioth's FocalTech
// probe, kits/alioth-touch/FocalTouchProbe.cpp, which drivers/NativeHidInput reads).
//
// Everything about the hardware is ios-sweet's, verified on this phone (xnu/patches 0085-0087: bsd/dev/sweet_touch.c,
// sweet_geni.c; tools/boards/pdx203.py /sweet-touch), extracted to a register-level spec with citations (2026-11-01):
//   - ABL leaves SE13 running the GENI I2C firmware (protocol 3) and the panel rails (pm8150a l1 1.8 V, l10 3.3 V) on for the splash,
//     so nothing is uploaded and no RPMh vote is made; the clocks are voted and the SE5 clock source set to the 19.2 MHz XO.
//   - TLMM south tile 0x0f500000: SDA 36 / SCL 37 (function 1), reset 38 (active low output), irq 39 (input, pull-up; not used).
//   - One FIFO message at a time, polled: a register read is a 1-byte write that keeps the bus (STOP_STRETCH), then a read.
//   - sec_ts: reset pulse, wait for the boot-complete status event, boot status 0x55 must read 0x20, device id 0x22,
//     touch function 0x30 {0x61,0x00}, sense on 0x10; then 8-byte events through 0x60 every 10 ms.
//   - Coordinates arrive in the IC's 1644x3840 space (12 bit), no axis swap.
//
// Boot arguments: pdx203-touch=1 starts it (from native_legacy_display_late_probe); pdx203-touch-delay=<s> delays the bring-up
// (default 5; ios-sweet waited 45 s in case the bus hangs the SoC, which it never did on this phone).
#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
#include <libkern/c++/OSSharedPtr.h>
#include <sys/errno.h>
#include <libkern/libkern.h>

// The seqlock NativeHidInput reads (it increments native_touch_hid_reports itself). Coordinates are in the HID layer's fixed
// space, X 0..30479 across the panel's width and Y 0..20319 along its height, as alioth's probe publishes them.
extern "C" {
volatile uint32_t native_touch_seq, native_touch_count, native_touch_hid_reports;
volatile int native_touch_xs[12], native_touch_ys[12], native_touch_ids[12];
// Diagnostics: [0] stage (negative: the step that failed), [1] boot status, [2] events read, [3] coordinate events,
// [4] publishes, [5] i2c errors, [6] status events, [7] IC reboots seen, [8] fingers down now, [9] polls
volatile int32_t pdx203_touch_stats[10];
}

static void (*gTouchSink)(void *);
static void *gTouchSinkContext;
extern "C" void native_touch_set_sink(void (*sink)(void *), void *context) {
	gTouchSinkContext = context;
	__atomic_store_n(&gTouchSink, sink, __ATOMIC_RELEASE);
}

namespace {

constexpr uint64_t kGccBase = 0x00100000, kGccSize = 0x60000;
constexpr uint32_t kGccVote = 0x52008, kGccVoteMask = 0x083c0000;        // wrapper 1 core 2x/core, m/s ahb, s5
constexpr uint32_t kSeRcg = 0x18600;                                      // SE5 clock source
constexpr uint32_t kGccHalt[] = {0x18004, 0x18008, 0x23138, 0x23140, 0x185fc};
constexpr uint64_t kTlmmBase = 0x0f500000, kTlmmSize = 0x28000;
constexpr unsigned kPinSda = 36, kPinScl = 37, kPinReset = 38, kPinIrq = 39;
constexpr uint64_t kSeBase = 0x00a94000, kSeSize = 0x4000;
constexpr uint32_t kAddr = 0x48;

// GENI serial engine registers (offsets follow Linux qcom-geni-se / i2c-qcom-geni, as ios-sweet's do)
enum : uint32_t {
	SE_FORCE_DEFAULT = 0x020, SE_OUTPUT_CTRL = 0x024, SE_CGC_CTRL = 0x028, SE_STATUS = 0x040, SE_M_CLK_CFG = 0x048,
	SE_IF_DISABLE = 0x064, SE_FW_REVISION = 0x068, SE_CLK_SEL = 0x07c, SE_BYTE_GRAN = 0x254, SE_DMA_MODE_EN = 0x258,
	SE_TX_PACK0 = 0x260, SE_TX_PACK1 = 0x264, SE_I2C_TX_LEN = 0x26c, SE_I2C_RX_LEN = 0x270, SE_I2C_SCL_COUNTERS = 0x278,
	SE_RX_PACK0 = 0x284, SE_RX_PACK1 = 0x288, SE_M_CMD0 = 0x600, SE_M_CMD_CTRL = 0x604, SE_M_IRQ_STATUS = 0x610,
	SE_M_IRQ_EN = 0x614, SE_M_IRQ_CLEAR = 0x618, SE_S_IRQ_EN = 0x644, SE_S_IRQ_CLEAR = 0x648, SE_TX_FIFO = 0x700,
	SE_RX_FIFO = 0x780, SE_TX_FIFO_STATUS = 0x800, SE_RX_FIFO_STATUS = 0x804, SE_TX_WATERMARK = 0x80c,
	SE_RX_WATERMARK = 0x810, SE_RX_RFR_WATERMARK = 0x814, SE_GSI_EVENT_EN = 0xe18, SE_IRQ_EN = 0xe1c,
	SE_HW_PARAM_0 = 0xe24, SE_HW_PARAM_1 = 0xe28, SE_DMA_GENERAL_CFG = 0xe30,
};
constexpr uint32_t M_CMD_DONE = 1u << 0, M_CMD_ABORT = 1u << 5, M_NACK = 1u << 10, M_RX_WM = 1u << 26, M_RX_LAST = 1u << 27,
	M_TX_WM = 1u << 30, M_ERRORS = 0x3300363eu;
constexpr uint32_t OP_WRITE = 1, OP_READ = 2, STOP_STRETCH = 1u << 2;

// sec_ts (Sony sec_ts.h as ios-sweet carries it)
constexpr uint8_t TS_SENSE_ON = 0x10, TS_READ_DEVICE_ID = 0x22, TS_SET_TOUCHFUNC = 0x30, TS_READ_BOOT_STATUS = 0x55,
	TS_READ_ONE_EVENT = 0x60, TS_STATUS_APP_MODE = 0x20, TS_FUNC_DEFAULT = 0x61;
constexpr unsigned kEventLen = 8, kMaxFingers = 10, kMaxX = 1644, kMaxY = 3840;
constexpr int kHidX = 30480, kHidY = 20320;

struct Region {
	OSSharedPtr<IOMemoryMap> keep;
	uintptr_t va = 0;
};

bool mapRegion(uint64_t phys, uint64_t length, Region &region)
{
	IODeviceMemory *memory = IODeviceMemory::withRange(phys, length);
	if (!memory) {
		return false;
	}
	region.keep = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache), OSNoRetain);
	memory->release();
	region.va = region.keep ? static_cast<uintptr_t>(region.keep->getVirtualAddress()) : 0;
	return region.va != 0;
}

// returns the previous value
int32_t statAdd(unsigned i) { return __atomic_fetch_add(&pdx203_touch_stats[i], 1, __ATOMIC_RELAXED); }

Region gGcc, gTlmm, gSe;
bool gDead;
unsigned gRxDepth;

uint32_t rd(const Region &r, uint32_t reg) { return *reinterpret_cast<volatile uint32_t *>(r.va + reg); }
void wr(const Region &r, uint32_t reg, uint32_t value)
{
	*reinterpret_cast<volatile uint32_t *>(r.va + reg) = value;
	__asm__ volatile ("dsb sy" ::: "memory");
}

// IODelay costs a whole timer tick on this kernel (the ADT has no powergate-latency, alioth measured ~830 us), so short
// waits spin on the 19.2 MHz counter.
void delayUs(uint32_t us)
{
	uint64_t start, now, ticks = (uint64_t)us * 192 / 10;
	__asm__ volatile ("isb\n\tmrs %0, cntpct_el0" : "=r"(start));
	do {
		__asm__ volatile ("isb\n\tmrs %0, cntpct_el0" : "=r"(now));
	} while (now - start < ticks);
}

bool waitBits(const Region &r, uint32_t reg, uint32_t mask, uint32_t want, uint32_t us)
{
	for (uint32_t i = 0; i <= us; ++i) {
		if ((rd(r, reg) & mask) == want) {
			return true;
		}
		delayUs(1);
	}
	return false;
}

void gpio(unsigned pin, unsigned mux, unsigned pull, unsigned drive, bool out, bool high)
{
	uint32_t cfg = pin * 0x1000, io = cfg + 4;
	if (out) {
		wr(gTlmm, io, (rd(gTlmm, io) & ~2u) | (high ? 2u : 0u));
	}
	wr(gTlmm, cfg, (rd(gTlmm, cfg) & ~0x3ffu) | (mux << 2) | pull | (drive << 6) | (out ? 0x200u : 0u));
}

bool clocks(void)
{
	wr(gGcc, kGccVote, rd(gGcc, kGccVote) | kGccVoteMask);
	uint32_t cfg = rd(gGcc, kSeRcg + 4), dfs = rd(gGcc, kSeRcg + 0x14);
	if (dfs & 1) {
		// DFS on: level 0 must already be the XO with divider 1, the table is not changed
		uint32_t level0 = rd(gGcc, kSeRcg + 0x1c);
		if ((level0 & 0x371f) != 1) {
			printf("PDX203 TOUCH: se5 clock source in dfs mode, level 0 0x%x is not the xo\n", level0);
			return false;
		}
	} else {
		wr(gGcc, kSeRcg + 4, (cfg & ~(0x371fu | (1u << 20))) | 1);
		wr(gGcc, kSeRcg, rd(gGcc, kSeRcg) | 1);
		if (!waitBits(gGcc, kSeRcg, 1, 0, 5000)) {
			printf("PDX203 TOUCH: se5 clock source update did not complete\n");
			return false;
		}
	}
	// on kona the wrapper's s_ahb branch reads halted even when voted (ios-sweet 0087); the SE probe is the real test
	uint32_t halted = 0;
	for (unsigned i = 0; i < sizeof(kGccHalt) / sizeof(kGccHalt[0]); ++i) {
		if (!waitBits(gGcc, kGccHalt[i], 1u << 31, 0, 5000)) {
			halted |= 1u << i;
		}
	}
	printf("PDX203 TOUCH: clocks voted, rcg cfg 0x%x dfs %u, halted branches 0x%x\n", rd(gGcc, kSeRcg + 4), dfs & 1, halted);
	return true;
}

bool seSetup(void)
{
	uint32_t fw = rd(gSe, SE_FW_REVISION), tx = rd(gSe, SE_HW_PARAM_0), rx = rd(gSe, SE_HW_PARAM_1);
	if (((fw >> 8) & 0xff) != 3 || (rd(gSe, SE_IF_DISABLE) & 1) || ((tx >> 24) & 0x3f) != 32 || ((rx >> 24) & 0x3f) != 32 ||
	    ((tx >> 16) & 0x3f) < 2 || ((rx >> 16) & 0x3f) < 2) {
		printf("PDX203 TOUCH: se13 is not an i2c engine with fifos (fw 0x%x tx 0x%x rx 0x%x)\n", fw, tx, rx);
		return false;
	}
	if (rd(gSe, SE_STATUS) & 0x1001) {
		printf("PDX203 TOUCH: se13 busy at setup (status 0x%x)\n", rd(gSe, SE_STATUS));
		return false;
	}
	gRxDepth = (rx >> 16) & 0x3f;
	wr(gSe, SE_GSI_EVENT_EN, 0);
	wr(gSe, SE_IRQ_EN, 0);
	wr(gSe, SE_S_IRQ_EN, 0);
	wr(gSe, SE_M_IRQ_CLEAR, 0xffffffff);
	wr(gSe, SE_S_IRQ_CLEAR, 0xffffffff);
	wr(gSe, SE_DMA_MODE_EN, 0);
	wr(gSe, SE_CGC_CTRL, rd(gSe, SE_CGC_CTRL) | 0x7f);
	wr(gSe, SE_DMA_GENERAL_CFG, rd(gSe, SE_DMA_GENERAL_CFG) | 0x0f);
	wr(gSe, SE_OUTPUT_CTRL, 0x7f);
	wr(gSe, SE_FORCE_DEFAULT, 1);
	wr(gSe, SE_CLK_SEL, 0);
	wr(gSe, SE_M_CLK_CFG, (7u << 4) | 1);                  // divider 7 of 19.2 MHz
	wr(gSe, SE_I2C_SCL_COUNTERS, (10u << 20) | (11u << 10) | 26);   // 100 kHz
	wr(gSe, SE_BYTE_GRAN, 0);
	wr(gSe, SE_TX_PACK0, 0x0007f8fe);                       // 8-bit words, MSB first on the wire, byte 0 in the FIFO word's LSB
	wr(gSe, SE_TX_PACK1, 0x000ffefe);
	wr(gSe, SE_RX_PACK0, 0x0007f8fe);
	wr(gSe, SE_RX_PACK1, 0x000ffefe);
	wr(gSe, SE_TX_WATERMARK, 0);
	wr(gSe, SE_RX_WATERMARK, 0);
	wr(gSe, SE_RX_RFR_WATERMARK, gRxDepth - 1);
	wr(gSe, SE_M_IRQ_EN, 0x7f00363f);                       // latches status only; nothing routes it to the gic
	if (rd(gSe, SE_TX_PACK0) != 0x0007f8fe || rd(gSe, SE_RX_PACK1) != 0x000ffefe || rd(gSe, SE_BYTE_GRAN) != 0) {
		printf("PDX203 TOUCH: se13 packing did not read back\n");
		return false;
	}
	if ((rd(gSe, SE_TX_FIFO_STATUS) & 0x0fffffff) || (rd(gSe, SE_RX_FIFO_STATUS) & 0x01ffffff)) {
		printf("PDX203 TOUCH: se13 has inherited fifo data\n");
		return false;
	}
	printf("PDX203 TOUCH: se13 i2c fw 0x%x, fifo depth tx %u rx %u\n", fw, (tx >> 16) & 0x3f, gRxDepth);
	return true;
}

void abortCommand(void)
{
	wr(gSe, SE_M_CMD_CTRL, 2);
	bool ok = waitBits(gSe, SE_M_IRQ_STATUS, M_CMD_ABORT, M_CMD_ABORT, 5000) && waitBits(gSe, SE_STATUS, 1, 0, 5000);
	for (unsigned i = 0; i < 64 && (rd(gSe, SE_RX_FIFO_STATUS) & 0x01ffffff); ++i) {
		(void)rd(gSe, SE_RX_FIFO);
	}
	wr(gSe, SE_M_IRQ_CLEAR, 0xffffffff);
	if (!ok || (rd(gSe, SE_TX_FIFO_STATUS) & 0x0fffffff)) {
		gDead = true;
		printf("PDX203 TOUCH: se13 abort failed, engine left alone\n");
	}
}

// One message: a write of 1..3 bytes or a read of 1..90 bytes, never both.
int message(uint32_t op, const uint8_t *txb, unsigned nt, uint8_t *rxb, unsigned nr, bool stretch)
{
	if (gDead) {
		return EIO;
	}
	if (!waitBits(gSe, SE_STATUS, 1, 0, 5000)) {
		gDead = true;
		printf("PDX203 TOUCH: se13 stayed busy, engine left alone\n");
		return EBUSY;
	}
	wr(gSe, SE_M_IRQ_CLEAR, 0xffffffff);
	wr(gSe, SE_TX_WATERMARK, nt ? 1 : 0);
	if (nt) {
		wr(gSe, SE_I2C_TX_LEN, nt);
	} else {
		wr(gSe, SE_I2C_RX_LEN, nr);
	}
	wr(gSe, SE_M_CMD0, (op << 27) | (kAddr << 9) | (stretch ? STOP_STRETCH : 0));
	bool txPending = nt != 0;
	unsigned got = 0;
	for (unsigned loop = 0; loop < 20000; ++loop) {
		uint32_t irq = rd(gSe, SE_M_IRQ_STATUS);
		if (irq & M_ERRORS) {
			int error = (irq & M_NACK) ? ENXIO : EIO;
			bool done = waitBits(gSe, SE_M_IRQ_STATUS, M_CMD_DONE, M_CMD_DONE, 5000);
			if (done && !(rd(gSe, SE_STATUS) & 1) && !(rd(gSe, SE_TX_FIFO_STATUS) & 0x0fffffff) &&
			    !(rd(gSe, SE_RX_FIFO_STATUS) & 0x01ffffff)) {
				wr(gSe, SE_M_IRQ_CLEAR, 0xffffffff);
			} else {
				abortCommand();
			}
			return error;
		}
		if (txPending && (irq & M_TX_WM)) {
			uint32_t word = 0;
			for (unsigned i = 0; i < nt; ++i) {
				word |= (uint32_t)txb[i] << (8 * i);
			}
			wr(gSe, SE_TX_FIFO, word);
			wr(gSe, SE_TX_WATERMARK, 0);
			txPending = false;
		}
		uint32_t words = rd(gSe, SE_RX_FIFO_STATUS) & 0x01ffffff;
		if (words > 64 || words > (nr - got + 3) / 4) {
			abortCommand();
			return EIO;
		}
		for (uint32_t w = 0; w < words; ++w) {
			uint32_t word = rd(gSe, SE_RX_FIFO);
			for (unsigned b = 0; b < 4 && got < nr; ++b) {
				rxb[got++] = (uint8_t)(word >> (8 * b));
			}
		}
		if (irq & M_CMD_DONE) {
			uint32_t late = rd(gSe, SE_M_IRQ_STATUS);
			wr(gSe, SE_M_IRQ_CLEAR, 0xffffffff);
			if (late & M_ERRORS) {
				return (late & M_NACK) ? ENXIO : EIO;
			}
			return (!txPending && got == nr) ? 0 : EIO;
		}
		wr(gSe, SE_M_IRQ_CLEAR, irq & (M_RX_WM | M_RX_LAST | M_TX_WM));
		delayUs(1);
	}
	abortCommand();
	return ETIMEDOUT;
}

int tsWrite(const uint8_t *bytes, unsigned n) { return message(OP_WRITE, bytes, n, nullptr, 0, false); }
int tsRead(uint8_t reg, uint8_t *out, unsigned n)
{
	// STOP_STRETCH on the register write keeps the bus for the repeated start of the read
	int error = message(OP_WRITE, &reg, 1, nullptr, 0, true);
	return error ? error : message(OP_READ, nullptr, 0, out, n, false);
}

struct Finger {
	bool down;
	int x, y;          // IC space
};
Finger gFingers[kMaxFingers];

// Touch timing for the boot log (UI smoothness, 2026-11-03): while a finger is down, the intervals between publishes (what the HID
// layer sees as samples) and between polls, printed every two seconds as "PDX203 TOUCH timing ...". Stays quiet when nothing is touched.
struct TouchTiming {
	uint64_t lastPublish, lastPoll, windowStart;
	uint64_t pubCount, pubSum, pubMin, pubMax, pubLate;   // late: over 20 ms
	uint64_t pollCount, pollSum, pollMax;
};
TouchTiming gTiming;

uint64_t
nowNs(void)
{
	uint64_t ns = 0;
	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns;
}

void
timingReport(uint64_t now)
{
	TouchTiming &t = gTiming;
	if (!t.windowStart) {
		t.windowStart = now;
	}
	if (now - t.windowStart < 2000000000ull) {
		return;
	}
	if (t.pubCount) {
		printf("PDX203 TOUCH timing: %llu samples, interval min/avg/max %llu.%02llu/%llu.%02llu/%llu.%02llu ms, %llu over 20 ms; "
		    "%llu polls, period avg/max %llu.%02llu/%llu.%02llu ms\n", t.pubCount, t.pubMin / 1000000, t.pubMin / 10000 % 100,
		    t.pubSum / t.pubCount / 1000000, t.pubSum / t.pubCount / 10000 % 100, t.pubMax / 1000000, t.pubMax / 10000 % 100, t.pubLate,
		    t.pollCount, t.pollCount ? t.pollSum / t.pollCount / 1000000 : 0, t.pollCount ? t.pollSum / t.pollCount / 10000 % 100 : 0,
		    t.pollMax / 1000000, t.pollMax / 10000 % 100);
	}
	const uint64_t lastPublish = t.lastPublish, lastPoll = t.lastPoll;
	t = TouchTiming{};
	t.lastPublish = lastPublish;
	t.lastPoll = lastPoll;
	t.windowStart = now;
}

// One poll of the loop; counts the period only while a finger is down.
void
timingPoll(bool touching)
{
	const uint64_t now = nowNs();
	TouchTiming &t = gTiming;
	if (touching && t.lastPoll) {
		const uint64_t d = now - t.lastPoll;
		t.pollCount++;
		t.pollSum += d;
		if (d > t.pollMax) {
			t.pollMax = d;
		}
	}
	t.lastPoll = touching ? now : 0;
	timingReport(now);
}

void
timingPublish(unsigned fingers)
{
	const uint64_t now = nowNs();
	TouchTiming &t = gTiming;
	if (fingers && t.lastPublish) {
		const uint64_t d = now - t.lastPublish;
		if (!t.pubCount || d < t.pubMin) {
			t.pubMin = d;
		}
		if (d > t.pubMax) {
			t.pubMax = d;
		}
		t.pubCount++;
		t.pubSum += d;
		if (d > 20000000ull) {
			t.pubLate++;
		}
	}
	t.lastPublish = fingers ? now : 0;
}

// The USB control channel's TOUCH (pdx203_touch_inject): one more finger, in the panel's touch space, merged into every report.
// The touch thread and the control worker both publish, so the publishing itself is serialized.
struct {volatile bool down; volatile int x, y;} gInjected;
IOLock *volatile gPublishLock;
void publishLocked(void);
void publish(void)
{
	IOLock *lock = __atomic_load_n(&gPublishLock, __ATOMIC_ACQUIRE);
	if (!lock) {
		IOLock *fresh = IOLockAlloc();
		IOLock *expected = nullptr;
		if (fresh && !__atomic_compare_exchange_n(&gPublishLock, &expected, fresh, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) IOLockFree(fresh);
		lock = __atomic_load_n(&gPublishLock, __ATOMIC_ACQUIRE);
	}
	if (lock) IOLockLock(lock);
	publishLocked();
	if (lock) IOLockUnlock(lock);
}
void publishLocked(void)
{
	unsigned n = 0;
	int xs[kMaxFingers + 1], ys[kMaxFingers + 1], ids[kMaxFingers + 1];
	for (unsigned i = 0; i < kMaxFingers; ++i) {
		if (!gFingers[i].down) {
			continue;
		}
		xs[n] = (int)((int64_t)gFingers[i].x * kHidX / kMaxX);
		ys[n] = (int)((int64_t)gFingers[i].y * kHidY / kMaxY);
		ids[n] = (int)i + 1;
		++n;
	}
	if (gInjected.down) {
		xs[n] = (int)((int64_t)gInjected.x * kHidX / kMaxX);
		ys[n] = (int)((int64_t)gInjected.y * kHidY / kMaxY);
		ids[n] = (int)kMaxFingers + 1;
		++n;
	}
	native_touch_seq = native_touch_seq + 1;
	__asm__ volatile ("dmb ish" ::: "memory");
	native_touch_count = n;
	for (unsigned i = 0; i < n; ++i) {
		native_touch_xs[i] = xs[i];
		native_touch_ys[i] = ys[i];
		native_touch_ids[i] = ids[i];
	}
	__asm__ volatile ("dmb ish" ::: "memory");
	native_touch_seq = native_touch_seq + 1;
	statAdd(4);
	pdx203_touch_stats[8] = (int32_t)n;
	timingPublish(n);
	void (*sink)(void *) = __atomic_load_n(&gTouchSink, __ATOMIC_ACQUIRE);
	if (sink) {
		sink(gTouchSinkContext);
	}
}

void releaseAll(void)
{
	bool any = false;
	for (unsigned i = 0; i < kMaxFingers; ++i) {
		any |= gFingers[i].down;
		gFingers[i].down = false;
	}
	if (any) {
		publish();
	}
}

int senseOn(void)
{
	uint8_t command = TS_SENSE_ON;
	return tsWrite(&command, 1);
}

// Returns true when the finger set changed.
bool handleEvent(const uint8_t *e)
{
	unsigned eid = e[0] & 3;
	if (eid == 0) {
		statAdd(3);
		unsigned tid = (e[0] >> 2) & 0xf, action = (e[0] >> 6) & 3;
		unsigned ttype = (((e[6] >> 6) & 3) << 2) | ((e[7] >> 6) & 3);
		if (tid == 0 || tid > kMaxFingers || ttype != 0) {
			return false;              // palms, hover and side touches are dropped
		}
		Finger &f = gFingers[tid - 1];
		if (action == 3) {
			bool was = f.down;
			f.down = false;
			return was;
		}
		if (action == 1 || action == 2) {
			int x = (e[1] << 4) | (e[3] >> 4), y = (e[2] << 4) | (e[3] & 0xf);
			if (x >= (int)kMaxX || y >= (int)kMaxY) {
				return false;
			}
			bool changed = !f.down || f.x != x || f.y != y;
			f.down = true;
			f.x = x;
			f.y = y;
			return changed;
		}
		return false;
	}
	if (eid == 1) {
		statAdd(6);
		unsigned stype = (e[0] >> 2) & 0xf;
		if (stype == 2 && e[1] == 0x00 && e[2] == 0x20) {
			// the IC rebooted (its own watchdog): everything lifted, sense on again
			statAdd(7);
			printf("PDX203 TOUCH: sec_ts rebooted, sense on again\n");
			releaseAll();
			(void)senseOn();
		} else if (stype == 1 && e[1] == 0x01) {
			releaseAll();              // event queue full
		}
	}
	return false;
}

bool start(void)
{
	// reset pulse: 5 ms low, then 300 ms (vendor: 200 ms after reset, 70 ms before the boot wait)
	gpio(kPinReset, 0, 3, 3, true, false);
	IOSleep(5);
	gpio(kPinReset, 0, 3, 3, true, true);
	IOSleep(300);
	uint8_t e[kEventLen];
	bool booted = false;
	for (unsigned tries = 0; tries < 100 && !booted; ++tries) {
		if (tsRead(TS_READ_ONE_EVENT, e, kEventLen)) {
			if (gDead) {
				return false;
			}
			IOSleep(20);
			continue;
		}
		if (e[0] == 0) {
			IOSleep(20);
			continue;
		}
		unsigned stype = (e[0] >> 2) & 0xf;
		booted = (e[0] & 3) == 1 && (stype == 2 || stype == 7) && e[1] == 0x00;
	}
	if (!booted) {
		printf("PDX203 TOUCH: no boot-complete event from the sec_ts, continuing\n");
	}
	uint8_t boot = 0, id[5] = {};
	int error = tsRead(TS_READ_BOOT_STATUS, &boot, 1);
	if (!error) {
		error = tsRead(TS_READ_DEVICE_ID, id, sizeof(id));
	}
	pdx203_touch_stats[1] = boot;
	printf("PDX203 TOUCH: sec_ts boot status 0x%02x id %02x %02x %02x %02x %02x (error %d)\n", boot, id[0], id[1], id[2], id[3], id[4],
	    error);
	if (error || boot != TS_STATUS_APP_MODE) {
		return false;
	}
	const uint8_t func[3] = {TS_SET_TOUCHFUNC, TS_FUNC_DEFAULT, 0x00};
	if ((error = tsWrite(func, 3))) {
		printf("PDX203 TOUCH: set touch function failed (%d), continuing\n", error);
	}
	if ((error = senseOn())) {
		printf("PDX203 TOUCH: sense on failed (%d)\n", error);
		return false;
	}
	return true;
}

void touchThread(void *, wait_result_t)
{
	unsigned delay = 5;
	PE_parse_boot_argn("pdx203-touch-delay", &delay, sizeof(delay));
	if (delay > 600) {
		delay = 600;
	}
	IOSleep(delay * 1000);
	pdx203_touch_stats[0] = 2;
	if (!clocks()) {
		pdx203_touch_stats[0] = -2;
		return;
	}
	if (!seSetup()) {
		pdx203_touch_stats[0] = -3;
		return;
	}
	gpio(kPinIrq, 0, 3, 3, false, false);
	gpio(kPinSda, 1, 0, 0, false, false);
	gpio(kPinScl, 1, 0, 0, false, false);
	if (!start()) {
		pdx203_touch_stats[0] = -4;
		return;
	}
	pdx203_touch_stats[0] = 5;
	printf("PDX203 TOUCH: running, %ux%u\n", kMaxX, kMaxY);
	unsigned failures = 0;
	while (!gDead) {
		statAdd(9);
		bool changed = false, failed = false;
		uint8_t e[kEventLen];
		for (unsigned n = 0; n < 32; ++n) {
			if (tsRead(TS_READ_ONE_EVENT, e, kEventLen)) {
				failed = true;
				break;
			}
			if (e[0] == 0) {
				break;
			}
			statAdd(2);
			changed |= handleEvent(e);
			if ((e[7] & 0x3f) == 0) {
				break;
			}
		}
		if (failed) {
			if ((statAdd(5) % 100) == 0) {
				printf("PDX203 TOUCH: event read failed (%d so far)\n", pdx203_touch_stats[5] + 1);
			}
			if (++failures == 10) {
				releaseAll();
			}
		} else {
			failures = 0;
		}
		if (changed) {
			publish();
		}
		timingPoll(pdx203_touch_stats[8] > 0);
		IOSleep(10);
	}
	releaseAll();
	pdx203_touch_stats[0] = -5;
	printf("PDX203 TOUCH: stopped, the engine is left alone\n");
}

} // namespace

// From native_legacy_display_late_probe on pdx203-touch=1: map the windows and start the polling thread, which does the bring-up
// after pdx203-touch-delay seconds. Returns at once.
// TOUCH from the USB control channel: phase 'd' (down), 'm' (move) or 'u' (up) at x, y in 1/10000 of the panel's width and height.
extern "C" int pdx203_touch_inject(char phase, int x, int y)
{
	if (phase != 'u' && (x < 0 || x > 9999 || y < 0 || y > 9999)) return -2;
	if (phase == 'u') gInjected.down = false;
	else if (phase == 'd' || phase == 'm') {
		gInjected.x = (int)((int64_t)x * kMaxX / 10000);
		gInjected.y = (int)((int64_t)y * kMaxY / 10000);
		gInjected.down = true;
	} else return -1;
	publish();
	return 0;
}

// The control channel's board input (drivers/QcomXnuUsbDebug qcom_usb_debug_board_input): TOUCH:<d|m|u>,<x>,<y> and
// KEY:<power|volup|voldown|home>,<d|u>. Runs on the USB worker thread; no USB logging from here.
extern "C" int pdx203_buttons_inject(uint32_t bit, bool down);
static bool argInt(const char *&p, int &value)
{
	if (*p < '0' || *p > '9') return false;
	long v = 0;
	while (*p >= '0' && *p <= '9' && v < 100000) v = v * 10 + (*p++ - '0');
	value = (int)v;
	return true;
}
static void reason(char *out, size_t cap, const char *why)
{
	size_t i = 0;
	for (; out && i + 1 < cap && why[i]; ++i) out[i] = why[i];
	if (out && cap) out[i] = 0;
}
extern "C" int qcom_usb_debug_board_input(const char *verb, const char *args, char *out, size_t cap)
{
	if (!verb || !args) {reason(out, cap, "badargs"); return 1;}
	if (!strcmp(verb, "TOUCH")) {
		const char *p = args;
		const char phase = *p++;
		int x = 0, y = 0;
		if (phase == 'u' && !*p) return pdx203_touch_inject('u', 0, 0);
		if (*p++ != ',' || !argInt(p, x) || *p++ != ',' || !argInt(p, y) || *p) {reason(out, cap, "badargs"); return 1;}
		const int rc = pdx203_touch_inject(phase, x, y);
		if (rc) reason(out, cap, rc == -2 ? "range" : "badphase");
		return rc ? 1 : 0;
	}
	if (!strcmp(verb, "KEY")) {
		static const struct {const char *name; uint32_t bit;} keys[] = {{"home", 1u << 0}, {"power", 1u << 1}, {"volup", 1u << 2},
			{"voldown", 1u << 3}};
		const char *comma = strchr(args, ',');
		if (!comma || (strcmp(comma + 1, "d") && strcmp(comma + 1, "u"))) {reason(out, cap, "badargs"); return 1;}
		for (auto &k : keys) {
			if ((size_t)(comma - args) == strlen(k.name) && !strncmp(args, k.name, strlen(k.name))) {
				if (pdx203_buttons_inject(k.bit, comma[1] == 'd')) {reason(out, cap, "nosink"); return 1;}
				return 0;
			}
		}
		reason(out, cap, "nokey");
		return 1;
	}
	reason(out, cap, "noverb");
	return 1;
}

extern "C" bool pdx203_touch_start(void)
{
	static bool started;
	if (__atomic_exchange_n(&started, true, __ATOMIC_ACQ_REL)) {
		return pdx203_touch_stats[0] >= 0;
	}
	pdx203_touch_stats[0] = 1;
	if (!mapRegion(kGccBase, kGccSize, gGcc) || !mapRegion(kTlmmBase, kTlmmSize, gTlmm) || !mapRegion(kSeBase, kSeSize, gSe)) {
		pdx203_touch_stats[0] = -1;
		printf("PDX203 TOUCH: mapping the gcc, tlmm or se13 window failed\n");
		return false;
	}
	thread_t thread;
	if (kernel_thread_start(touchThread, nullptr, &thread) != KERN_SUCCESS) {
		pdx203_touch_stats[0] = -1;
		return false;
	}
	thread_deallocate(thread);
	return true;
}
