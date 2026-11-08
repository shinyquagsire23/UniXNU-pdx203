// SPDX-License-Identifier: BSD-2-Clause-Patent
// pdx203 motion sensors: the IMU (ST LSM6DSM accelerometer + gyroscope) behind the sensor DSP (SLPI), read through Qualcomm's Snapdragon
// Sensor Core (SSC), and the screen orientation it gives (NativeHidInput's orientation device, as the volume chord sets it).
//
// The IMU sits on the SSC's own SPI bus (kona_lsm6dsm_0.json: bus_type 1, bus_instance 3, 9.6 MHz, IRQ 123); only the SLPI reaches it.
// AdspProbe.cpp boots the SLPI and runs its GLINK edge; this file is the application processor's side of the two channels the SLPI
// opens, as postmarketOS does it on SM8250 (hexagonrpcd + libssc):
//  - fastrpcglink-apps-dsp (FastRPC): the sensors protection domain asks the application processor for its configuration files over a
//    reverse-call tunnel. We attach to that domain (INIT_ATTACH, pd 2), register the default listener, and answer its calls: remotectl
//    (open/close by name) and apps_std (read-only fopen/fread/fseek/fclose/opendir/readdir/closedir/stat) over an in-memory tree of the
//    phone's own /vendor/etc/sensors (config JSONs, sns_reg_config), /mnt/vendor/persist/sensors/registry and /sys/devices/soc0 (pulled
//    from Android with TWRP; local only, scripts/pack-pdx203-sensors.py packs them into the boot add-on's sensors/sensors.pack). Wire
//    formats: linux-msm/hexagonrpc (hexagonrpcd listener.c, iobuffer.c, apps_std.c, localctl.c) and Linux drivers/misc/fastrpc.c
//    (fastrpc_get_args: the payload's remote_arg/invoke_buf/phy_page tables, 128-byte aligned inline buffers).
//  - IPCRTR (QRTR): the SLPI's router node. We answer the service locator (msm/slpi/{root,sensor}_pd) and are a client of the SSC service
//    (QMI service 400): SUID lookup for "accel", then enable it at kRateHz; its events (msg 1025) are m/s^2 in Android's frame (x right,
//    y up, z out of the screen), with Sony's mount matrix already applied by the SSC. Protobuf field numbers: libssc data/*.proto.
// FastRPC payload memory is physically contiguous below 4 GiB; the SLPI's FastRPC SMMU streams (0x541..0x543, pdx203.dtb
// sdsprpc-smd compute banks) pass through to physical addresses (SmmuBypass.h), and addresses carry session 1 in bits 32+ as Linux's do.
// Boot argument pdx203-sns=0 turns this off (it needs pdx203-slpi, on by default).
#include <stdarg.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IODeviceMemory.h>
#include <libkern/zlib.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>

extern "C" {
#include "QcomQrtr.h"
#include "ShengAdsp.h"
ShengAdspChannel *pdx203_dsp_channel(unsigned edge, const char *name, uint32_t intent_size, uint32_t intent_count, int initiator,
                                     const ShengAdspClient *client, void *context) __attribute__((weak_import));
}
#include "SmmuBypass.h"
extern "C" int native_ram_block_gpu_addon(const uint8_t **data, uint64_t *bytes) __attribute__((weak_import));
extern "C" void native_hid_orientation_step(intptr_t step) __attribute__((weak_import));
extern "C" volatile int32_t pdx203_slpi_stats[12] __attribute__((weak_import));
extern "C" unsigned pdx203_slpi_glink_summary(char *out, unsigned cap) __attribute__((weak_import));

// [0] stage (see Stage; negative: the failed step), [1] last error, [2] files in the pack, [3] FastRPC invokes, [4] reverse calls served,
// [5] reverse calls refused, [6] files opened, [7] opens of missing paths, [8] SSC indications, [9] accel samples, [10] orientation
// (1..4), [11] orientation changes, [12] accel x, [13] y, [14] z (mm/s^2), [15] FastRPC last retval
extern "C" {volatile int32_t pdx203_sns_stats[16]; char pdx203_sns_text[128];}
extern "C" {volatile int32_t pdx203_knob_autorotate = 1;}

namespace {
enum Stage {StStarted = 1, StPack = 2, StChannels = 3, StSmmu = 4, StAttached = 5, StListening = 6, StSsc = 7, StAccel = 8, StStreaming = 9};
volatile int32_t *const S = pdx203_sns_stats;
void stage(int value, int error = 0) {
    S[0] = value;
    if (error) S[1] = error;
    printf("PDX203 SNS stage %d err %d | %s\n", value, error, pdx203_sns_text);
}
void note(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void note(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(pdx203_sns_text, sizeof pdx203_sns_text, fmt, ap);
    va_end(ap);
}

uint32_t le32(const uint8_t *p) {uint32_t v; memcpy(&v, p, 4); return v;}
uint64_t le64(const uint8_t *p) {uint64_t v; memcpy(&v, p, 8); return v;}
void put32(uint8_t *p, uint32_t v) {memcpy(p, &v, 4);}
void put64(uint8_t *p, uint64_t v) {memcpy(p, &v, 8);}

// ---- the files: the boot add-on's sensors/sensors.pack, inflated once ----
// "SNSPACK1", u32 count, then per file (sorted by path): u16 path bytes, u32 size, the path (canonical, absolute), the data.
struct VFile {const char *path; uint16_t pathLen; const uint8_t *data; uint32_t size;};
uint8_t *gPack;
size_t gPackBytes;
VFile *gFiles;
uint32_t gFileCount;

void *zAlloc(void *, uInt items, uInt size) {
    const size_t n = (size_t)items * size + 16;
    uint8_t *p = static_cast<uint8_t *>(IOMallocData(n));
    if (!p) return nullptr;
    memcpy(p, &n, sizeof n);
    return p + 16;
}
void zFree(void *, void *ptr) {
    if (!ptr) return;
    uint8_t *p = static_cast<uint8_t *>(ptr) - 16;
    size_t n; memcpy(&n, p, sizeof n);
    IOFreeData(p, n);
}
bool addonFile(const char *want, uint8_t **out, size_t *outBytes) {
    const uint8_t *a = nullptr; uint64_t bytes = 0;
    if (!native_ram_block_gpu_addon || !native_ram_block_gpu_addon(&a, &bytes) || bytes < 24 || memcmp(a, "NGPUADD2", 8)) return false;
    uint32_t count; memcpy(&count, a + 8, 4);
    const size_t wantLen = strlen(want);
    uint64_t at = 24;
    for (uint32_t i = 0; i < count && at + 24 <= bytes; ++i) {
        uint32_t nb; uint64_t size, packed;
        memcpy(&nb, a + at, 4); memcpy(&size, a + at + 8, 8); memcpy(&packed, a + at + 16, 8);
        at += 24;
        const char *name = reinterpret_cast<const char *>(a + at);
        at += (nb + 7) & ~7ull;
        if (packed > bytes - at) return false;
        if (nb >= wantLen && !strncmp(name, want, wantLen) && (nb == wantLen || !name[wantLen]) && size && size < (64u << 20)) {
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
bool loadPack() {
    if (!addonFile("sensors/sensors.pack", &gPack, &gPackBytes)) return false;
    if (gPackBytes < 12 || memcmp(gPack, "SNSPACK1", 8)) return false;
    const uint32_t count = le32(gPack + 8);
    if (!count || count > 4096) return false;
    gFiles = static_cast<VFile *>(IOMallocData(sizeof(VFile) * count));
    if (!gFiles) return false;
    size_t at = 12;
    for (uint32_t i = 0; i < count; ++i) {
        if (at + 6 > gPackBytes) return false;
        uint16_t pl; memcpy(&pl, gPack + at, 2);
        const uint32_t size = le32(gPack + at + 2);
        at += 6;
        if (!pl || at + pl + size > gPackBytes || gPack[at] != '/') return false;
        gFiles[i] = {reinterpret_cast<const char *>(gPack + at), pl, gPack + at + pl, size};
        at += pl + size;
    }
    gFileCount = count;
    return true;
}

// A DSP path to the pack's canonical form: "." and ".." resolved, repeated slashes folded, /system/vendor -> /vendor and
// /mnt/vendor/persist -> /persist (hexagonrpcd links both pairs). Relative paths start at base. Returns the length, 0 when too long.
size_t canonical(const char *base, const char *path, char *out, size_t cap) {
    char joined[512];
    if (path[0] == '/') strlcpy(joined, path, sizeof joined);
    else snprintf(joined, sizeof joined, "%s/%s", base, path);
    size_t n = 0;
    out[0] = 0;
    for (const char *p = joined; *p;) {
        while (*p == '/') ++p;
        if (!*p) break;
        const char *end = p;
        while (*end && *end != '/') ++end;
        const size_t len = (size_t)(end - p);
        if (len == 1 && p[0] == '.') {p = end; continue;}
        if (len == 2 && p[0] == '.' && p[1] == '.') {
            while (n && out[n - 1] != '/') --n;
            if (n) --n;
            out[n] = 0;
            p = end;
            continue;
        }
        if (n + 1 + len + 1 > cap) return 0;
        out[n++] = '/';
        memcpy(out + n, p, len);
        n += len;
        out[n] = 0;
        p = end;
    }
    if (!n) {if (cap < 2) return 0; out[0] = '/'; out[1] = 0; n = 1;}
    auto strip = [&](const char *prefix, const char *with) {
        const size_t pl = strlen(prefix), wl = strlen(with);
        if (strncmp(out, prefix, pl) || (out[pl] && out[pl] != '/')) return;
        memmove(out + wl, out + pl, n - pl + 1);
        memcpy(out, with, wl);
        n = n - pl + wl;
        if (!n) {out[0] = '/'; out[1] = 0; n = 1;}
    };
    strip("/system/vendor", "/vendor");
    strip("/mnt/vendor/persist", "/persist");
    return n;
}
// The tree the DSP sees: the pack's files (read only, in the pack) plus what it writes this boot (in RAM: new files, renames, removals and
// directories; nothing persists). A removed or renamed pack file only leaves the table.
constexpr unsigned kMaxNodes = 1024;
struct Node {char *path; uint32_t pathLen; uint8_t *data; uint32_t size, cap; bool owned, dir, used;};
Node *gNodes;
uint32_t gNodeCount;
uint32_t gWrites, gRenames, gRemoves;
char *dupPath(const char *path, size_t n) {
    char *p = static_cast<char *>(IOMallocData(n + 1));
    if (p) {memcpy(p, path, n); p[n] = 0;}
    return p;
}
bool initNodes() {
    gNodes = static_cast<Node *>(IOMallocData(sizeof(Node) * kMaxNodes));
    if (!gNodes || gFileCount > kMaxNodes) return false;
    bzero(gNodes, sizeof(Node) * kMaxNodes);
    for (uint32_t i = 0; i < gFileCount; ++i) {
        char *path = dupPath(gFiles[i].path, gFiles[i].pathLen);
        if (!path) return false;
        gNodes[i] = {path, gFiles[i].pathLen, const_cast<uint8_t *>(gFiles[i].data), gFiles[i].size, gFiles[i].size, false, false, true};
    }
    gNodeCount = gFileCount;
    return true;
}
int findNode(const char *path, bool dirs = false) {
    const size_t n = strlen(path);
    for (uint32_t i = 0; i < gNodeCount; ++i)
        if (gNodes[i].used && gNodes[i].dir == dirs && gNodes[i].pathLen == n && !memcmp(gNodes[i].path, path, n)) return (int)i;
    return -1;
}
int findFile(const char *path) {return findNode(path, false);}
bool isDir(const char *path) {
    const size_t n = strlen(path);
    if (n == 1) return true;  // "/"
    if (findNode(path, true) >= 0) return true;
    for (uint32_t i = 0; i < gNodeCount; ++i)
        if (gNodes[i].used && gNodes[i].pathLen > n && !memcmp(gNodes[i].path, path, n) && gNodes[i].path[n] == '/') return true;
    return false;
}
int newNode(const char *path, bool dir) {
    uint32_t i = 0;
    while (i < gNodeCount && gNodes[i].used) ++i;
    if (i == kMaxNodes) return -1;
    char *p = dupPath(path, strlen(path));
    if (!p) return -1;
    gNodes[i] = {p, (uint32_t)strlen(path), nullptr, 0, 0, true, dir, true};
    if (i == gNodeCount) gNodeCount++;
    return (int)i;
}
void dropNode(int i) {
    Node &n = gNodes[i];
    if (n.owned && n.data) IOFreeData(n.data, n.cap);
    if (n.path) IOFreeData(n.path, n.pathLen + 1);
    n = {};
}
// Room for size bytes in a file node, copying a pack file into RAM on its first write.
bool reserve(Node &n, uint32_t size) {
    if (n.owned && size <= n.cap) return true;
    uint32_t cap = n.cap > 256 ? n.cap : 256;
    while (cap < size) cap *= 2;
    uint8_t *d = static_cast<uint8_t *>(IOMallocData(cap));
    if (!d) return false;
    if (n.size) memcpy(d, n.data, n.size);
    if (n.owned && n.data) IOFreeData(n.data, n.cap);
    n.data = d; n.cap = cap; n.owned = true;
    return true;
}

// Open files and directories, numbered as hexagonrpcd numbers them (0..2 are its root and two search directories).
constexpr unsigned kFds = 64, kFirstFd = 3;
struct Fd {bool used, dir, write; int node; uint32_t pos; char path[200]; uint32_t next; int dots;};
Fd gFd[kFds];
int allocFd(const char *path, bool dir, int node) {
    for (unsigned i = kFirstFd; i < kFds; ++i) {
        if (gFd[i].used) continue;
        gFd[i] = {};
        gFd[i].used = true; gFd[i].dir = dir; gFd[i].node = node;
        strlcpy(gFd[i].path, path, sizeof gFd[i].path);
        return (int)i;
    }
    return -2;
}
int openFd(const char *path, bool wantDir) {
    const int file = findFile(path);
    const bool dir = file < 0 && isDir(path);
    if (file < 0 && !dir) return -1;
    if (wantDir && !dir) return -1;
    return allocFd(path, dir, file);
}
// A file opened for writing ("w" truncates, "a" appends, "r+" keeps), created when missing.
int openWrite(const char *path, const char *mode) {
    int node = findFile(path);
    if (node < 0 && mode[0] == 'r') return -1;
    if (node < 0 && (node = newNode(path, false)) < 0) return -2;
    Node &n = gNodes[node];
    if (mode[0] == 'w') n.size = 0;
    const int fd = allocFd(path, false, node);
    if (fd >= 0) {gFd[fd].write = true; gFd[fd].pos = mode[0] == 'a' ? n.size : 0;}
    return fd;
}
Fd *fdAt(uint64_t n) {return n < kFds && n >= kFirstFd && gFd[n].used ? &gFd[n] : nullptr;}
// The child of dir that node i lies under, if any: its name in name (cap bytes); false otherwise.
bool childOf(const char *dir, size_t n, uint32_t i, char *name, size_t cap) {
    const Node &f = gNodes[i];
    if (!f.used || f.pathLen <= n + 1 || memcmp(f.path, dir, n) || f.path[n] != '/') return false;
    const char *child = f.path + n + 1;
    size_t len = 0;
    while (child + len < f.path + f.pathLen && child[len] != '/') ++len;
    if (!len || len + 1 > cap) return false;
    memcpy(name, child, len); name[len] = 0;
    return true;
}
// The next entry of a directory: ".", "..", then each child once (the first node under it names it).
bool readDir(Fd &d, char *name, size_t cap) {
    name[0] = 0;
    if (d.dots < 2) {strlcpy(name, d.dots ? ".." : ".", cap); d.dots++; return true;}
    const size_t n = strcmp(d.path, "/") ? strlen(d.path) : 0;
    char other[256];
    for (; d.next < gNodeCount; ++d.next) {
        if (!childOf(d.path, n, d.next, name, cap)) continue;
        bool seen = false;
        for (uint32_t j = 0; j < d.next && !seen; ++j) seen = childOf(d.path, n, j, other, sizeof other) && !strcmp(other, name);
        if (seen) continue;
        ++d.next;
        return true;
    }
    name[0] = 0;
    return true;  // empty name: the end
}

// ---- the SMMU: the SLPI's FastRPC streams pass through ----
uint32_t smmuRd(void *ctx, unsigned at) {return *reinterpret_cast<volatile uint32_t *>(reinterpret_cast<uintptr_t>(ctx) + at);}
void smmuWr(void *ctx, unsigned at, uint32_t value) {
    *reinterpret_cast<volatile uint32_t *>(reinterpret_cast<uintptr_t>(ctx) + at) = value;
    __asm__ volatile("dsb sy" ::: "memory");
}

// ---- FastRPC (one invoke in flight; the DSP thread delivers its answer) ----
constexpr uint32_t kTgid = 0x5d3, kPdSensors = 2, kSession = 1, kPage = 4096, kAlign = 128;
constexpr size_t kDmaBytes = 512 * 1024;
// The payload memory: 512 KiB of the PIL NPU carve-out below the WiFi arena (0x86900000..0x86c00000 is free; reserved in the UEFI memory
// map, and XNU's own memory is all above 4 GiB), mapped write-combined as the WiFi arena is.
constexpr uint64_t kDmaPhys = 0x86900000;
IOMemoryMap *gDma;
uint8_t *gDmaVa;
uint64_t gDmaPa;
ShengAdspChannel *gRpc, *gRouter;
IOLock *gLock;
// Event words, one per waiting thread: the listener's (bit 0 an RPC answer, bit 2 a channel state change) and the SSC thread's (bit 1 an
// SSC packet, bit 2 a router change).
volatile uint32_t gRpcEvents, gSscEvents;
volatile bool gRpcUp, gRouterUp;
uint32_t gCtxId = 1;
struct {volatile bool done; uint64_t ctx; int32_t retval;} gReply;

constexpr uint32_t sc(uint32_t method, uint32_t in, uint32_t out) {return (method & 0x1f) << 24 | (in & 0xff) << 16 | (out & 0xff) << 8;}
constexpr uint32_t scIn(uint32_t v) {return (v >> 16) & 0xff;}
constexpr uint32_t scOut(uint32_t v) {return (v >> 8) & 0xff;}
constexpr uint32_t scMethod(uint32_t v) {return (v >> 24) & 0x1f;}

void wake(volatile uint32_t &word, uint32_t bits) {
    __atomic_fetch_or(&word, bits, __ATOMIC_RELEASE);
    thread_wakeup((event_t)&word);
}
uint32_t waitEvents(volatile uint32_t &word, uint32_t ms) {
    uint32_t bits = __atomic_exchange_n(&word, 0, __ATOMIC_ACQ_REL);
    if (bits) return bits;
    assert_wait_timeout((event_t)&word, THREAD_UNINT, ms, kMillisecondScale);
    bits = __atomic_exchange_n(&word, 0, __ATOMIC_ACQ_REL);
    if (bits) {clear_wait(current_thread(), THREAD_AWAKENED); return bits;}
    thread_block(THREAD_CONTINUE_NULL);
    return __atomic_exchange_n(&word, 0, __ATOMIC_ACQ_REL);
}
// The payload is write-combined: a barrier is all it needs (and bzero's DC ZVA faults on it, so it is cleared byte-wise).
void clear(uint8_t *p, size_t n) {for (size_t i = 0; i < n; ++i) reinterpret_cast<volatile uint8_t *>(p)[i] = 0;}

void rpcState(void *, int up) {gRpcUp = up != 0; wake(gRpcEvents, 4);}
void rpcReceive(void *, const uint8_t *data, uint32_t length) {
    if (length < 12) return;
    const uint64_t ctx = le64(data);
    int32_t ret; memcpy(&ret, data + 8, 4);
    if (ctx == 0xabcdabcdull) return;  // a protection domain's exit notice
    gReply.ctx = ctx; gReply.retval = ret;
    __atomic_store_n(&gReply.done, true, __ATOMIC_RELEASE);
    wake(gRpcEvents, 1);
}

struct Buf {void *p; uint32_t len;};  // in buffers (copied out), then out buffers (copied back)
// One invoke on handle, waiting up to ms for the answer (0: forever). Returns the DSP's retval, or -1000.. for local failures.
int invoke(uint32_t handle, uint32_t scalars, Buf *bufs, uint32_t ms, bool kernel = false) {
    const uint32_t nin = scIn(scalars), nout = scOut(scalars), n = nin + nout;
    const size_t meta = (size_t)n * 40 + 16 * 8 + 64 * 4;
    clear(gDmaVa, meta);
    uint8_t *const rpra = gDmaVa, *const list = gDmaVa + 16 * n, *const pages = gDmaVa + 24 * n;
    const uint64_t iova = gDmaPa | (uint64_t)kSession << 32;
    size_t at = meta;
    for (uint32_t i = 0; i < n; ++i) {
        const uint32_t len = bufs[i].len;
        put64(rpra + 16 * i + 8, len);
        put32(list + 8 * i, len ? 1 : 0);
        put32(list + 8 * i + 4, i);
        if (!len) continue;
        at = (at + kAlign - 1) & ~(size_t)(kAlign - 1);
        if (at + len > kDmaBytes) return -1001;
        put64(rpra + 16 * i, gDmaPa + at);
        const uint64_t first = (gDmaPa + at) / kPage, last = (gDmaPa + at + len - 1) / kPage;
        put64(pages + 16 * i, (iova + at) & ~(uint64_t)(kPage - 1));
        put64(pages + 16 * i + 8, (last - first + 1) * kPage);
        if (i < nin) memcpy(gDmaVa + at, bufs[i].p, len);
        else clear(gDmaVa + at, len);
        at += len;
    }
    const size_t total = (at + kPage - 1) & ~(size_t)(kPage - 1);
    __asm__ volatile("dsb sy" ::: "memory");
    const uint32_t id = gCtxId;
    gCtxId = gCtxId % 255 + 1;
    uint8_t msg[40];
    put32(msg, kernel ? 0 : kTgid);  // Linux sends pid 0 for its own invokes (INIT_ATTACH)
    put32(msg + 4, kTgid);
    put64(msg + 8, (uint64_t)id << 4 | kPdSensors);
    put32(msg + 16, handle);
    put32(msg + 20, scalars);
    put64(msg + 24, iova);
    put64(msg + 32, total);
    __atomic_store_n(&gReply.done, false, __ATOMIC_RELEASE);
    if (sheng_adsp_send(gRpc, msg, sizeof msg)) return -1002;
    S[3] = S[3] + 1;
    uint64_t deadline = 0;
    if (ms) clock_interval_to_deadline(ms, kMillisecondScale, &deadline);
    for (;;) {
        if (__atomic_load_n(&gReply.done, __ATOMIC_ACQUIRE)) {
            if (((gReply.ctx >> 4) & 0xff) == id) break;
            __atomic_store_n(&gReply.done, false, __ATOMIC_RELEASE);  // a stale answer
        }
        if (deadline && mach_absolute_time() >= deadline) return -1003;
        waitEvents(gRpcEvents, 1000);
    }
    __asm__ volatile("dsb sy" ::: "memory");
    for (uint32_t i = nin; i < n; ++i) {
        if (!bufs[i].len) continue;
        const uint64_t pv = le64(rpra + 16 * i);
        memcpy(bufs[i].p, gDmaVa + (pv - gDmaPa), bufs[i].len);
    }
    S[15] = gReply.retval;
    return gReply.retval;
}

// ---- the reverse calls (hexagonrpcd's localctl + apps_std, read only) ----
constexpr uint32_t kEFailed = 1, kEBadParm = 14, kEUnsupported = 20;
constexpr unsigned kMaxIo = 8;
struct Io {uint8_t *p; uint32_t len;};
const char *kIfaces[] = {"remotectl", "apps_std", "apps_mem"};

// The calls the DSP makes are short; their arguments arrive in the next2 in-buffer (encoded) and the results go back in the next.
uint32_t serve(uint32_t handle, uint32_t scalars, const Io *in, Io *out, uint8_t *outStore, size_t storeCap) {
    const uint32_t nin = scIn(scalars), nout = scOut(scalars);
    uint32_t method = scMethod(scalars);
    if (method == 31 && nin && in[0].len >= 4) method = le32(in[0].p);
    if (scalars & 0xff) return kEBadParm;
    size_t used = 0;
    auto alloc = [&](unsigned i, uint32_t len) -> uint8_t * {
        if (i >= nout || used + len > storeCap) return nullptr;
        out[i].p = outStore + used; out[i].len = len;
        bzero(out[i].p, len);
        used += (len + 7) & ~7u;
        return out[i].p;
    };
    auto str = [&](unsigned i) -> const char * {return i < nin && in[i].len && in[i].p[in[i].len - 1] == 0 ? reinterpret_cast<const char *>(in[i].p) : nullptr;};
    const uint8_t *prim = nin ? in[0].p : nullptr;
    const uint32_t primLen = nin ? in[0].len : 0;
    char path[256];
    if (handle == 0) {  // remotectl
        if (method == 0 && primLen >= 8 && nin >= 2 && nout >= 2) {  // open(name) -> handle, error
            const char *name = str(1);
            uint8_t *o = alloc(0, 8);
            alloc(1, le32(prim + 4));
            if (!name || !o) return kEBadParm;
            for (uint32_t i = 0; i < sizeof kIfaces / sizeof kIfaces[0]; ++i)
                if (!strcmp(name, kIfaces[i])) {put32(o, i); return 0;}
            put32(o + 4, (uint32_t)-5);
            note("remotectl open %s: none", name);
            return (uint32_t)-5;
        }
        if (method == 1 && primLen >= 8) {alloc(0, 4); alloc(1, le32(prim + 4)); return 0;}  // close
        return kEUnsupported;
    }
    if (handle != 1) return kEUnsupported;  // apps_mem: the sensors domain has not been seen to map memory
    switch (method) {
    case 2: return 0;  // fflush
    case 3: {          // fclose
        Fd *f = primLen >= 4 ? fdAt(le32(prim)) : nullptr;
        if (!f) return kEFailed;
        f->used = false;
        return 0;
    }
    case 4: {          // fread(fd, size) -> written, eof, data
        if (primLen < 8) return kEBadParm;
        Fd *f = fdAt(le32(prim));
        const uint32_t want = le32(prim + 4);
        uint8_t *o = alloc(0, 8), *data = alloc(1, want);
        if (!o || !data) return kEBadParm;
        if (!f || f->dir) return kEFailed;
        const Node &v = gNodes[f->node];
        const uint32_t n = f->pos < v.size ? (v.size - f->pos < want ? v.size - f->pos : want) : 0;
        memcpy(data, v.data + f->pos, n);
        f->pos += n;
        put32(o, n); put32(o + 4, n < want ? 1 : 0);
        return 0;
    }
    case 9: {          // fseek(fd, pos, whence)
        if (primLen < 12) return kEBadParm;
        Fd *f = fdAt(le32(prim));
        if (!f || f->dir) return kEFailed;
        const int64_t pos = (int32_t)le32(prim + 4), whence = le32(prim + 8);
        const int64_t base = whence == 0 ? 0 : whence == 1 ? f->pos : gNodes[f->node].size;
        if (base + pos < 0) return kEFailed;
        f->pos = (uint32_t)(base + pos);
        return 0;
    }
    case 19: {         // fopen_with_env(envvar, delim, name, mode) -> fd
        const char *env = str(1), *name = str(3), *mode = str(4);
        uint8_t *o = alloc(0, 4);
        if (!env || !name || !mode || !o) return kEBadParm;
        const char *base = !strcmp(env, "ADSP_LIBRARY_PATH") ? "/usr/lib/qcom/adsp" : !strcmp(env, "ADSP_AVS_CFG_PATH") ? "/vendor/etc/acdbdata" : nullptr;
        if (!base) return kEBadParm;
        if (!canonical(base, name, path, sizeof path)) return kEFailed;
        const bool write = mode[0] == 'w' || mode[0] == 'a' || strchr(mode, '+');
        const int fd = write ? openWrite(path, mode) : openFd(path, false);
        if (fd < 0) {S[7] = S[7] + 1; note("missing %s", path); return kEFailed;}
        S[6] = S[6] + 1;
        put32(o, (uint32_t)fd);
        return 0;
    }
    case 26: {         // opendir(name) -> dir
        const char *name = str(1);
        uint8_t *o = alloc(0, 8);
        if (!name || !o) return kEBadParm;
        if (!canonical("/", name, path, sizeof path)) return kEFailed;
        const int fd = openFd(path, true);
        if (fd < 0) {S[7] = S[7] + 1; note("missing dir %s", path); return kEFailed;}
        put64(o, (uint64_t)fd);
        return 0;
    }
    case 27: {         // closedir(dir)
        Fd *f = primLen >= 8 ? fdAt(le64(prim)) : nullptr;
        if (!f) return kEFailed;
        f->used = false;
        return 0;
    }
    case 28: {         // readdir(dir) -> inode, name[256], eof
        Fd *f = primLen >= 8 ? fdAt(le64(prim)) : nullptr;
        uint8_t *o = alloc(0, 264);
        if (!o) return kEBadParm;
        if (!f || !f->dir) return kEFailed;
        readDir(*f, reinterpret_cast<char *>(o + 4), 255);
        put32(o + 260, o[4] ? 0 : 1);
        return 0;
    }
    case 31: {         // stat(name) -> {tsz, dev, ino, mode, nlink, rdev, size, atime, ns, mtime, ns, ctime, ns}
        const char *name = str(1);
        uint8_t *o = alloc(0, 96);
        if (!name || !o) return kEBadParm;
        if (!canonical("/usr/lib/qcom/adsp", name, path, sizeof path)) return kEFailed;
        const int file = findFile(path);
        if (file < 0 && !isDir(path)) {S[7] = S[7] + 1; note("stat missing %s", path); return kEFailed;}
        put32(o + 24, file >= 0 ? 0100644 : 040755);
        put32(o + 28, 1);
        put64(o + 40, file >= 0 ? gNodes[file].size : 0);
        return 0;
    }
    case 5: {          // fwrite(fd, data) -> written, eof
        if (primLen < 8 || nin < 2) return kEBadParm;
        Fd *f = fdAt(le32(prim));
        uint8_t *o = alloc(0, 8);
        if (!o) return kEBadParm;
        if (!f || f->dir || !f->write) return kEFailed;
        Node &v = gNodes[f->node];
        const uint32_t n = in[1].len;
        if (!reserve(v, f->pos + n)) return 2;
        if (f->pos > v.size) bzero(v.data + v.size, f->pos - v.size);
        memcpy(v.data + f->pos, in[1].p, n);
        f->pos += n;
        if (f->pos > v.size) v.size = f->pos;
        gWrites++;
        put32(o, n);
        return 0;
    }
    case 22: {         // fileExists(name) -> exists
        const char *name = str(1);
        uint8_t *o = alloc(0, nout ? 4 : 0);
        if (!name) return kEBadParm;
        const bool exists = canonical("/", name, path, sizeof path) && (findFile(path) >= 0 || isDir(path));
        if (o) o[0] = exists ? 1 : 0;
        return 0;
    }
    case 23: return 0;  // fsync
    case 24: {         // fremove(name)
        const char *name = str(1);
        if (!name || !canonical("/", name, path, sizeof path)) return kEBadParm;
        const int node = findFile(path);
        if (node < 0) return kEFailed;
        dropNode(node);
        gRemoves++;
        return 0;
    }
    case 29: {         // mkdir(name, mode)
        const char *name = str(1);
        if (!name || !canonical("/", name, path, sizeof path)) return kEBadParm;
        if (isDir(path)) return 0;
        return newNode(path, true) < 0 ? 2 : 0;
    }
    case 32: {         // ftrunc(fd, length)
        if (primLen < 12) return kEBadParm;
        Fd *f = fdAt(le32(prim + 4));
        if (!f || f->dir) return kEFailed;
        Node &v = gNodes[f->node];
        const uint32_t length = le32(prim + 8);
        if (length > v.size && !reserve(v, length)) return 2;
        if (length > v.size) bzero(v.data + v.size, length - v.size);
        v.size = length;
        return 0;
    }
    case 33: {         // frename(old, new)
        const char *from = str(1), *to = str(2);
        char target[256];
        if (!from || !to || !canonical("/", from, path, sizeof path) || !canonical("/", to, target, sizeof target)) return kEBadParm;
        const int node = findFile(path);
        if (node < 0) {note("rename missing %s", path); return kEFailed;}
        const int old = findFile(target);
        if (old >= 0 && old != node) dropNode(old);
        char *p = dupPath(target, strlen(target));
        if (!p) return 2;
        IOFreeData(gNodes[node].path, gNodes[node].pathLen + 1);
        gNodes[node].path = p; gNodes[node].pathLen = (uint32_t)strlen(target);
        for (unsigned i = kFirstFd; i < kFds; ++i) if (gFd[i].used && gFd[i].node == node) strlcpy(gFd[i].path, target, sizeof gFd[i].path);
        gRenames++;
        return 0;
    }
    default:
        note("apps_std method %u not served", method);
        return kEUnsupported;
    }
}

// next2's encoding of a call's buffers: per buffer a u32 size, zero padding to 8 (counted from the blob's start), the bytes.
size_t encodeOut(const Io *out, uint32_t n, uint8_t *to, size_t cap) {
    size_t at = 0;
    for (uint32_t i = 0; i < n; ++i) {
        if (at + 4 > cap) return 0;
        put32(to + at, out[i].len);
        at += 4;
        if (out[i].len && (at & 7)) {const size_t pad = 8 - (at & 7); if (at + pad > cap) return 0; bzero(to + at, pad); at += pad;}
        if (at + out[i].len > cap) return 0;
        if (out[i].len) memcpy(to + at, out[i].p, out[i].len);
        at += out[i].len;
    }
    return at;
}
bool decodeIn(const uint8_t *from, size_t length, uint32_t n, Io *in) {
    size_t at = 0;
    for (uint32_t i = 0; i < n; ++i) {
        if (at + 4 > length) return false;
        in[i].len = le32(from + at);
        at += 4;
        if (in[i].len && (at & 7)) at += 8 - (at & 7);
        if (at + in[i].len > length) return false;
        in[i].p = const_cast<uint8_t *>(from + at);
        at += in[i].len;
    }
    return true;
}

// The listener: attach to the sensors domain, register the default listener, then answer reverse calls forever (adsp_listener next2).
constexpr uint32_t kInCap = 4096;
uint8_t *gScratch;  // [0, 64K) the encoded results, [64K, 64K + kInCap) the next call's buffers, then the results' storage
bool listen() {
    // INIT_ATTACH to the sensors protection domain (handle 1, method 0, the process id)
    // the sensors domain comes up some time after the SLPI says ready: try for a minute
    uint32_t tgid = kTgid;
    int rc = -1;
    for (unsigned tries = 0; tries < 20; ++tries) {
        Buf attach[1] = {{&tgid, 4}};
        rc = invoke(1, sc(0, 1, 0), attach, 5000, true);
        if (!rc) break;
        note("attach %d (try %u)", rc, tries + 1);
        printf("PDX203 SNS %s\n", pdx203_sns_text);
        IOSleep(3000);
    }
    if (rc) {stage(-StAttached, rc); return false;}
    // remotectl_open("adsp_default_listener") -> handle; adsp_default_listener_register on it
    static const char name[] = "adsp_default_listener";
    uint32_t prim[2] = {sizeof name, 256}, primOut[2] = {0, 0};
    char err[256];
    Buf open[4] = {{prim, 8}, {const_cast<char *>(name), sizeof name}, {primOut, 8}, {err, 256}};
    rc = invoke(0, sc(0, 2, 2), open, 5000);
    if (rc || primOut[1]) {note("listener open %d/%d", rc, (int)primOut[1]); stage(-StAttached, rc ? rc : (int)primOut[1]); return false;}
    rc = invoke(primOut[0], sc(0, 0, 0), nullptr, 5000);
    if (rc) {note("register %d", rc); stage(-StAttached, rc); return false;}
    stage(StAttached);
    rc = invoke(3, sc(3, 0, 0), nullptr, 5000);  // adsp_listener_init2
    if (rc) {note("init2 %d", rc); stage(-StListening, rc); return false;}
    stage(StListening);
    uint32_t rctx = 0, result = 0xffffffffu, callSc = sc(0, 0, 0);
    size_t encoded = 0;
    uint8_t *const enc = gScratch, *const inBlob = gScratch + 65536, *const store = gScratch + 65536 + kInCap;
    const size_t storeCap = kDmaBytes - 65536 - kInCap;
    for (;;) {
        uint32_t pin[4] = {rctx, result, (uint32_t)encoded, kInCap}, pout[4] = {0, 0, 0, 0};
        Buf next[4] = {{pin, 16}, {enc, (uint32_t)encoded}, {pout, 16}, {inBlob, kInCap}};
        rc = invoke(3, sc(4, 2, 2), next, 0);
        if (rc) {note("next2 %d", rc); stage(-StListening, rc); return false;}
        rctx = pout[0];
        const uint32_t handle = pout[1];
        callSc = pout[2];
        Io in[kMaxIo] = {}, out[kMaxIo] = {};
        const uint32_t nin = scIn(callSc), nout = scOut(callSc);
        if (pout[3] > kInCap || nin > kMaxIo || nout > kMaxIo || !decodeIn(inBlob, pout[3], nin, in)) {
            note("bad call h%u sc %x len %u", handle, callSc, pout[3]);
            result = kEBadParm; encoded = 0;
            S[5] = S[5] + 1;
            continue;
        }
        result = serve(handle, callSc, in, out, store, storeCap);
        if (result) S[5] = S[5] + 1; else S[4] = S[4] + 1;
        if (S[4] + S[5] <= 60 || result || scMethod(callSc) == 5 || scMethod(callSc) >= 22)
            printf("PDX203 SNS call h%u m%u sc %08x -> %u | %s\n", handle, scMethod(callSc), callSc, result, result ? pdx203_sns_text : "");
        // the results go back with the next next2 (only on success: a failed call's outputs are not looked at)
        encoded = result ? 0 : encodeOut(out, nout, enc, 65536);
        if (!result && nout && !encoded) result = 2;  // AEE_ENOMEMORY
    }
}

// ---- QRTR on the SLPI's link, the service locator, and the SSC client ----
constexpr uint32_t kLocalNode = 1, kLocatorPort = 1, kSscPort = 0x4001, kSscService = 400;
constexpr float kRateHz = 25.0f;
QcomQrtr gQ;
const QcomPdDomain kSlpiDomains[] = {{"msm/slpi/root_pd", 90, nullptr}, {"msm/slpi/sensor_pd", 90, nullptr}};
constexpr unsigned kQueueSlots = 16, kMaxPacket = 0x4400;
struct Packet {uint32_t length; uint8_t data[kMaxPacket];};
Packet *gInbox;  // SSC packets for our client port, filled by the DSP thread
uint32_t gInHead, gInCount;

int qSend(void *, const uint8_t *packet, uint32_t length) {return sheng_adsp_send(gRouter, packet, length) ? -1 : 0;}
void locator(void *, QcomQrtr *q, uint32_t, uint32_t node, uint32_t from, const uint8_t *data, uint32_t length) {
    uint8_t answer[480];
    const uint32_t n = qcom_servloc_answer(kSlpiDomains, 2, data, length, answer, sizeof answer);
    if (n) qcom_qrtr_send_data(q, kLocatorPort, node, from, answer, n);
}
void sscPacket(void *, QcomQrtr *, uint32_t, uint32_t, uint32_t, const uint8_t *data, uint32_t length) {
    if (length > kMaxPacket) return;
    if (gInCount == kQueueSlots) return;  // our thread is behind; drop
    Packet &p = gInbox[(gInHead + gInCount) % kQueueSlots];
    memcpy(p.data, data, length);
    p.length = length;
    gInCount++;
    wake(gSscEvents, 2);
}
void routerState(void *, int up) {
    IOLockLock(gLock);
    gRouterUp = up != 0;
    if (up) {gQ.hello_sent = gQ.hello_received = 0; gQ.remote_count = 0; gQ.remote_node = QCOM_QRTR_NODE_BROADCAST; qcom_qrtr_start(&gQ);}
    IOLockUnlock(gLock);
    wake(gSscEvents, 4);
}
void routerReceive(void *, const uint8_t *data, uint32_t length) {
    IOLockLock(gLock);
    qcom_qrtr_receive(&gQ, data, length);
    IOLockUnlock(gLock);
    wake(gSscEvents, 4);
}
bool popPacket(Packet *to) {
    IOLockLock(gLock);
    const bool any = gInCount != 0;
    if (any) {
        const Packet &p = gInbox[gInHead];
        memcpy(to->data, p.data, p.length);
        to->length = p.length;
        gInHead = (gInHead + 1) % kQueueSlots;
        gInCount--;
    }
    IOLockUnlock(gLock);
    return any;
}

// protobuf, the few shapes the SSC client messages need
struct Pb {uint8_t *p; size_t at, cap; bool bad;};
void pbByte(Pb &b, uint8_t v) {if (b.at < b.cap) b.p[b.at++] = v; else b.bad = true;}
void pbVarint(Pb &b, uint64_t v) {do {pbByte(b, (uint8_t)((v & 0x7f) | (v > 0x7f ? 0x80 : 0))); v >>= 7;} while (v);}
void pbFixed64(Pb &b, unsigned field, uint64_t v) {pbVarint(b, field << 3 | 1); for (int i = 0; i < 8; ++i) pbByte(b, (uint8_t)(v >> (8 * i)));}
void pbFixed32(Pb &b, unsigned field, uint32_t v) {pbVarint(b, field << 3 | 5); for (int i = 0; i < 4; ++i) pbByte(b, (uint8_t)(v >> (8 * i)));}
void pbBytes(Pb &b, unsigned field, const void *data, size_t n) {
    pbVarint(b, field << 3 | 2); pbVarint(b, n);
    for (size_t i = 0; i < n; ++i) pbByte(b, static_cast<const uint8_t *>(data)[i]);
}
void pbBool(Pb &b, unsigned field, bool v) {pbVarint(b, field << 3); pbVarint(b, v ? 1 : 0);}
// The next field of a message: number, wire type, and its value (varint/fixed in *value, bytes in *data/*n). False at the end or on error.
bool pbNext(const uint8_t *p, size_t len, size_t &at, unsigned &field, unsigned &type, uint64_t &value, const uint8_t *&data, size_t &n) {
    auto varint = [&](uint64_t &v) {
        v = 0;
        for (unsigned shift = 0; shift < 64; shift += 7) {
            if (at >= len) return false;
            const uint8_t c = p[at++];
            v |= (uint64_t)(c & 0x7f) << shift;
            if (!(c & 0x80)) return true;
        }
        return false;
    };
    uint64_t key;
    if (at >= len || !varint(key)) return false;
    field = (unsigned)(key >> 3); type = (unsigned)(key & 7);
    switch (type) {
    case 0: return varint(value);
    case 1: if (at + 8 > len) return false; value = le64(p + at); at += 8; return true;
    case 5: if (at + 4 > len) return false; value = le32(p + at); at += 4; return true;
    case 2: if (!varint(value) || value > len - at) return false; data = p + at; n = (size_t)value; at += n; return true;
    default: return false;
    }
}

struct Suid {uint64_t low, high;};
constexpr Suid kLookup = {0xababababababababull, 0xababababababababull};
uint16_t gTxn = 1;
uint32_t gLookups, gSendFails;
// One SSC request (QMI 0x20: TLV 1 = u16 length + sns_client_request_msg, TLV 0x10 = 1 for large reports) to the SSC service.
bool sscRequest(const Suid &suid, uint32_t msgId, const uint8_t *payload, size_t payloadLen) {
    uint8_t body[160], req[200], frame[240];
    Pb inner = {body, 0, sizeof body, false};
    if (payloadLen) pbBytes(inner, 2, payload, payloadLen);
    Pb r = {req, 0, sizeof req, false};
    uint8_t uid[18];
    Pb u = {uid, 0, sizeof uid, false};
    pbFixed64(u, 1, suid.low); pbFixed64(u, 2, suid.high);
    pbBytes(r, 1, uid, u.at);
    pbFixed32(r, 2, msgId);
    static const uint8_t config[] = {0x08, 0x01, 0x10, 0x00};  // processor APSS, delivery wakeup
    pbBytes(r, 3, config, sizeof config);
    pbBytes(r, 4, body, inner.at);
    if (inner.bad || r.bad || u.bad) return false;
    const uint16_t txn = gTxn++;
    size_t at = 7;
    frame[at++] = 0x01; const uint16_t tl = (uint16_t)(r.at + 2); memcpy(frame + at, &tl, 2); at += 2;
    const uint16_t pl = (uint16_t)r.at; memcpy(frame + at, &pl, 2); at += 2;
    memcpy(frame + at, req, r.at); at += r.at;
    frame[at++] = 0x10; frame[at++] = 1; frame[at++] = 0; frame[at++] = 1;
    frame[0] = 0; memcpy(frame + 1, &txn, 2);
    const uint16_t mid = 0x20, len = (uint16_t)(at - 7);
    memcpy(frame + 3, &mid, 2); memcpy(frame + 5, &len, 2);
    IOLockLock(gLock);
    const int i = qcom_qrtr_find_remote(&gQ, kSscService);
    const int rc = i < 0 ? -1 : qcom_qrtr_send_data(&gQ, kSscPort, gQ.remote[i].node, gQ.remote[i].port, frame, (uint32_t)at);
    IOLockUnlock(gLock);
    if (rc) gSendFails++;
    return rc == 0;
}
bool lookup(const char *type) {
    uint8_t m[48];
    Pb b = {m, 0, sizeof m, false};
    pbBytes(b, 1, type, strlen(type));
    pbBool(b, 2, false);
    pbBool(b, 3, true);
    gLookups++;
    return !b.bad && sscRequest(kLookup, 512, m, b.at);
}

// ---- orientation from gravity ----
// Android's axes: x to the right edge, y to the top edge, z out of the screen; at rest the reading is +g along the edge pointing up.
// iOS (UIDeviceOrientation, as the orientation device takes it): 1 portrait, 2 upside down, 3 landscape left (top edge to the left, so
// the right edge up: +x), 4 landscape right. A new orientation needs a clear dominant axis (> 6.5 m/s^2, the phone not near flat) held
// for kHoldSamples.
constexpr unsigned kHoldSamples = 12;  // ~0.5 s at 25 Hz
int gCandidate, gHeld, gOrientation;
void accelSample(float x, float y, float z) {
    S[9] = S[9] + 1;
    S[12] = (int32_t)(x * 1000); S[13] = (int32_t)(y * 1000); S[14] = (int32_t)(z * 1000);
    const float ax = x < 0 ? -x : x, ay = y < 0 ? -y : y;
    int want = 0;
    if (ay > 6.5f && ay > ax * 1.5f) want = y > 0 ? 1 : 2;
    else if (ax > 6.5f && ax > ay * 1.5f) want = x > 0 ? 3 : 4;
    if (!want || want == gOrientation) {gCandidate = 0; gHeld = 0; return;}
    if (want != gCandidate) {gCandidate = want; gHeld = 0;}
    if (++gHeld < (int)kHoldSamples) return;
    gOrientation = want; gCandidate = 0; gHeld = 0;
    S[10] = want; S[11] = S[11] + 1;
    printf("PDX203 SNS orientation %d (accel %d %d %d mm/s^2)%s\n", want, (int)S[12], (int)S[13], (int)S[14],
           pdx203_knob_autorotate ? "" : " (autorotate off)");
    if (pdx203_knob_autorotate && native_hid_orientation_step) native_hid_orientation_step(-want);
}

// An SSC indication (QMI 0x21/0x22, TLV 2 = u16 length + sns_client_event_msg): each event of the SUID-lookup sensor (768: data type and
// SUIDs) and of the accelerometer (1025: the sample) is handled.
Suid gAccel;
bool gHaveAccel;
void sscIndication(const uint8_t *data, uint32_t length) {
    QcomQmiMessage m;
    if (qcom_qmi_parse(data, length, &m) || m.type != QCOM_QMI_INDICATION || (m.message != 0x21 && m.message != 0x22)) return;
    uint32_t size = 0;
    const uint8_t *tlv = qcom_qmi_tlv(&m, 0x02, &size);
    if (!tlv || size < 2) return;
    uint16_t pl; memcpy(&pl, tlv, 2);
    if (pl > size - 2) return;
    const uint8_t *p = tlv + 2;
    S[8] = S[8] + 1;
    size_t at = 0, n = 0; unsigned field, type; uint64_t value = 0; const uint8_t *d = nullptr;
    Suid from = {};
    while (pbNext(p, pl, at, field, type, value, d, n)) {
        if (field == 1 && type == 2) {  // the sender's SUID
            size_t a2 = 0, n2 = 0; unsigned f2, t2; uint64_t v2 = 0; const uint8_t *d2 = nullptr;
            while (pbNext(d, n, a2, f2, t2, v2, d2, n2)) {if (f2 == 1) from.low = v2; else if (f2 == 2) from.high = v2;}
        } else if (field == 2 && type == 2) {  // one event: msg_id, timestamp, payload
            size_t a2 = 0, n2 = 0, payLen = 0; unsigned f2, t2; uint64_t v2 = 0; const uint8_t *d2 = nullptr, *pay = nullptr;
            uint32_t msgId = 0;
            while (pbNext(d, n, a2, f2, t2, v2, d2, n2)) {if (f2 == 1) msgId = (uint32_t)v2; else if (f2 == 3 && t2 == 2) {pay = d2; payLen = n2;}}
            if (!pay) continue;
            if (msgId == 768 && from.low == kLookup.low && from.high == kLookup.high) {  // SUID event: data type, SUIDs
                size_t a3 = 0, n3 = 0; unsigned f3, t3; uint64_t v3 = 0; const uint8_t *d3 = nullptr;
                char dataType[24] = "";
                Suid first = {}; bool any = false;
                while (pbNext(pay, payLen, a3, f3, t3, v3, d3, n3)) {
                    if (f3 == 1 && t3 == 2) {const size_t c = n3 < sizeof dataType - 1 ? n3 : sizeof dataType - 1; memcpy(dataType, d3, c); dataType[c] = 0;}
                    else if (f3 == 2 && t3 == 2 && !any) {
                        size_t a4 = 0, n4 = 0; unsigned f4, t4; uint64_t v4 = 0; const uint8_t *d4 = nullptr;
                        while (pbNext(d3, n3, a4, f4, t4, v4, d4, n4)) {if (f4 == 1) first.low = v4; else if (f4 == 2) first.high = v4;}
                        any = true;
                    }
                }
                printf("PDX203 SNS suid %s: %s %016llx%016llx\n", dataType, any ? "found" : "none", (unsigned long long)first.high,
                       (unsigned long long)first.low);
                if (any && !strcmp(dataType, "accel")) {gAccel = first; gHaveAccel = true;}
            } else if (msgId == 1025 && gHaveAccel && from.low == gAccel.low && from.high == gAccel.high) {
                float xyz[3] = {}; unsigned got = 0;
                size_t a3 = 0, n3 = 0; unsigned f3, t3; uint64_t v3 = 0; const uint8_t *d3 = nullptr;
                while (pbNext(pay, payLen, a3, f3, t3, v3, d3, n3)) {
                    if (f3 != 1) continue;
                    if (t3 == 5 && got < 3) {const uint32_t w = (uint32_t)v3; memcpy(&xyz[got++], &w, 4);}
                    else if (t3 == 2) for (size_t k = 0; k + 4 <= n3 && got < 3; k += 4) memcpy(&xyz[got++], d3 + k, 4);
                }
                if (got == 3) accelSample(xyz[0], xyz[1], xyz[2]);
            }
        }
    }
}

// The SSC side: wait for the SSC service, look the accelerometer up (again each second until the registry is loaded and it shows up),
// enable it, then take its events. Runs alongside the listener on its own thread.
void sscThread(void *, wait_result_t) {
    thread_set_thread_name(current_thread(), "pdx203-ssc");
    Packet *pkt = static_cast<Packet *>(IOMallocData(sizeof(Packet)));
    if (!pkt) return;
    uint64_t lastLookup = 0, lastRow = 0, ns = 0;
    bool enabled = false, sawService = false;
    for (;;) {
        waitEvents(gSscEvents, 200);
        while (popPacket(pkt)) sscIndication(pkt->data, pkt->length);
        IOLockLock(gLock);
        const bool service = qcom_qrtr_find_remote(&gQ, kSscService) >= 0;
        IOLockUnlock(gLock);
        if (service && !sawService) {sawService = true; if (S[0] < StSsc) stage(StSsc);}
        const uint64_t now = mach_absolute_time();
        absolutetime_to_nanoseconds(now - lastLookup, &ns);
        if (service && !gHaveAccel && ns > 1000000000ull) {lookup("accel"); lastLookup = now;}
        if (gHaveAccel && !enabled) {
            uint8_t cfg[5] = {0x0d};
            memcpy(cfg + 1, &kRateHz, 4);
            if (sscRequest(gAccel, 513, cfg, sizeof cfg)) {enabled = true; stage(StAccel);}
        }
        if (enabled && S[9] && S[0] == StAccel) stage(StStreaming);
        absolutetime_to_nanoseconds(now - lastRow, &ns);
        if (ns > 10000000000ull) {
            lastRow = now;
            printf("PDX203 SNS row stage %d rpc %d calls %d/%d opens %d missing %d ind %d accel %d (%d %d %d mm/s^2) orient %d changes %d"
                   " | lookups %u fail %u qrtr in %u out %u refused %u resumes %u services %u | writes %u renames %u removes %u\n",
                   (int)S[0], (int)S[3], (int)S[4], (int)S[5], (int)S[6], (int)S[7], (int)S[8], (int)S[9], (int)S[12], (int)S[13],
                   (int)S[14], (int)S[10], (int)S[11], gLookups, gSendFails, gQ.rx_packets, gQ.tx_packets, gQ.tx_refused, gQ.resumes_sent, gQ.remote_count, gWrites, gRenames, gRemoves);
            if (pdx203_slpi_glink_summary) {
                char g[640];
                pdx203_slpi_glink_summary(g, sizeof g);
                printf("PDX203 SNS glink %s\n", g);
            }
        }
    }
}

void snsThread(void *, wait_result_t) {
    thread_set_thread_name(current_thread(), "pdx203-sns");
    stage(StStarted);
    // the add-on (and so the pack) is in RAM from boot; a few tries in case the RAM block publishes it late
    bool pack = false;
    for (unsigned i = 0; i < 30 && !(pack = loadPack()); ++i) IOSleep(1000);
    if (!pack) {stage(-StPack, -2); return;}
    if (!initNodes()) {stage(-StPack, -12); return;}
    S[2] = (int32_t)gFileCount;
    stage(StPack);
    IODeviceMemory *dma = IODeviceMemory::withRange(kDmaPhys, kDmaBytes);
    gDma = dma ? dma->map(kIOMapAnywhere | kIOMapWriteCombineCache) : nullptr;
    if (dma) dma->release();
    gScratch = static_cast<uint8_t *>(IOMallocData(kDmaBytes));
    gInbox = static_cast<Packet *>(IOMallocData(sizeof(Packet) * kQueueSlots));
    if (!gDma || !gScratch || !gInbox) {stage(-StPack, gDma ? -13 : -12); return;}
    gDmaVa = reinterpret_cast<uint8_t *>(gDma->getVirtualAddress());
    gDmaPa = kDmaPhys;
    // the FastRPC streams to physical addresses
    IODeviceMemory *memory = IODeviceMemory::withRange(0x15000000, 0x100000);
    IOMemoryMap *map = memory ? memory->map(kIOMapAnywhere | kIOMapInhibitCache) : nullptr;
    if (memory) memory->release();
    if (!map) {stage(-StSmmu, -12); return;}
    const smmu_io io = {reinterpret_cast<void *>(map->getVirtualAddress()), smmuRd, smmuWr};
    for (uint16_t stream = 0x541; stream <= 0x543; ++stream) {
        const smmu_plan plan = smmu_plan_bypass(&io, stream, gDmaPa, kDmaBytes);
        if (const char *error = smmu_configure_bypass(&io, &plan, stream)) {note("smmu %x: %s", stream, error); stage(-StSmmu, stream); return;}
    }
    stage(StSmmu);
    // the SLPI's channels: it opens both, we answer
    qcom_qrtr_init(&gQ, kLocalNode, qSend, nullptr);
    qcom_qrtr_add_service(&gQ, QCOM_SERVLOC_SERVICE, QCOM_SERVLOC_INSTANCE, kLocatorPort, locator, nullptr);
    qcom_qrtr_add_service(&gQ, 0, 0, kSscPort, sscPacket, nullptr);
    static const ShengAdspClient rpcClient = {rpcState, rpcReceive}, routerClient = {routerState, routerReceive};
    gRouter = pdx203_dsp_channel(1, "IPCRTR", kMaxPacket, 4, 0, &routerClient, nullptr);
    gRpc = pdx203_dsp_channel(1, "fastrpcglink-apps-dsp", 128, 16, 0, &rpcClient, nullptr);
    if (!gRouter || !gRpc) {stage(-StChannels, -12); return;}
    for (unsigned i = 0; i < 600 && !gRpcUp; ++i) waitEvents(gRpcEvents, 100);
    if (!gRpcUp) {note("fastrpc channel not up"); stage(-StChannels, -60); return;}
    stage(StChannels);
    thread_t ssc = nullptr;
    if (kernel_thread_start(sscThread, nullptr, &ssc) == KERN_SUCCESS) thread_deallocate(ssc);
    listen();
}
}  // namespace

extern "C" void pdx203_sensors_probe(void) {
    unsigned mode = 1;
    PE_parse_boot_argn("pdx203-sns", &mode, sizeof mode);
    if (!mode || !pdx203_dsp_channel || S[0]) return;
    gLock = IOLockAlloc();
    thread_t thread = nullptr;
    if (!gLock || kernel_thread_start(snsThread, nullptr, &thread) != KERN_SUCCESS) {stage(-StStarted, -7); return;}
    thread_deallocate(thread);
}
