// SPDX-License-Identifier: BSD-2-Clause-Patent
// pdx203: the unified log on the USB console. The board's only log channel is the kernel's (QcomXnuUsbDebug); os_log lines of user-space
// processes never reach it, so locationd, backboardd, SpringBoard, Safari and WebKit fail silently there. This program reads the unified
// log once a second (OSLogStore, system scope, the logging entitlements adrenoprobed uses) and writes the entries of the processes in
// /private/var/ngpu/oslog/filter.txt to /dev/console, one line each:  OSLOG <process>[<pid>] <level> <subsystem>:<category> <message>
//
// It runs as /usr/bin/IOMFB_FDR_Loader: launchd starts that job at boot from its cache (a new LaunchDaemons plist is not read on iOS 17),
// and the program it replaces only exits with status 1 on this board. The kernel overlays the addon's usrbin directory on /usr/bin
// (patch zzy-zoslog-addon-mount); the binary's CDHash is in the owner trust cache.
//
// filter.txt: one rule per line, '#' comments. A process name (or a prefix ending in '*') selects its entries; "level N" sets the lowest
// level forwarded (0 any, 1 debug, 2 info, 3 notice/default, 4 error, 5 fault; default 3); "rate N" caps lines per second (default 60);
// "all" forwards every process at that level. Without the file a built-in list is used.
#import <Foundation/Foundation.h>
#include <dlfcn.h>
#include <errno.h>
#include <string.h>
#include <fts.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/sysctl.h>
#include <fcntl.h>
#include <objc/message.h>
#include <objc/runtime.h>
#include <string>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <mach/mach_time.h>
#include <mach/mach.h>
#include <mach-o/dyld_images.h>
#include <pthread.h>
#include <time.h>

namespace {

constexpr const char *kFilter = "/private/var/ngpu/oslog/filter.txt";
const char *const kDefault[] = {"locationd", "backboardd", "SpringBoard", "MobileSafari", "com.apple.WebKit.*", "kbd", nullptr};

struct Filter {
    std::vector<std::string> exact, prefix;
    bool all = false;
    long level = 3, rate = 60;
};

Filter load() {
    Filter f;
    FILE *file = fopen(kFilter, "r");
    if (!file) {
        for (unsigned i = 0; kDefault[i]; ++i) {
            std::string s = kDefault[i];
            if (s.back() == '*') f.prefix.push_back(s.substr(0, s.size() - 1)); else f.exact.push_back(s);
        }
        return f;
    }
    char line[256];
    while (fgets(line, sizeof line, file)) {
        std::string s = line;
        while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ')) s.pop_back();
        if (s.empty() || s[0] == '#') continue;
        if (s == "all") f.all = true;
        else if (!s.compare(0, 6, "level ")) f.level = strtol(s.c_str() + 6, nullptr, 10);
        else if (!s.compare(0, 5, "rate ")) f.rate = strtol(s.c_str() + 5, nullptr, 10);
        else if (s.back() == '*') f.prefix.push_back(s.substr(0, s.size() - 1));
        else f.exact.push_back(s);
    }
    fclose(file);
    return f;
}

bool wanted(const Filter &f, const char *process) {
    if (f.all) return true;
    for (const auto &e : f.exact) if (e == process) return true;
    for (const auto &p : f.prefix) if (!strncmp(process, p.c_str(), p.size())) return true;
    return false;
}

// OSLogEntryLog levels: 0 undefined, 1 debug, 2 info, 3 notice, 4 error, 5 fault; activity and signpost entries have no level (3 here).
char levelLetter(long level) {
    static const char letters[] = "?dinef";
    return level >= 0 && level <= 5 ? letters[level] : '?';
}

int gConsole = -1;
void emit(const char *text) {
    if (gConsole < 0) gConsole = open("/dev/console", O_WRONLY | O_NOCTTY);
    if (gConsole >= 0) (void)write(gConsole, text, strlen(text));
}

}  // namespace

// The live stream `log stream` uses (LoggingSupport, private): OSLogEventLiveStore +liveLocalStore, -prepareWithCompletionHandler: gives an
// OSLogEventLiveSource, OSLogEventLiveStream -initWithLiveSource: with an event handler, -activate. OSLogStore (the stored log) answered nil
// without an error on pdx203, for both +localStoreAndReturnError: and +storeWithScope:error:.
namespace {
Filter gFilter;
long gSentThisSecond;
time_t gSecond;
unsigned long gDropped;
id gKeepStore, gKeepStream;   // the live store and stream stay alive for the process

// logType: 0x00 default (notice), 0x01 info, 0x02 debug, 0x10 error, 0x11 fault -> the OSLogEntryLog levels of filter.txt.
long levelOfType(long type) {
    switch (type) {
    case 0x02: return 1;
    case 0x01: return 2;
    case 0x10: return 4;
    case 0x11: return 5;
    default: return 3;
    }
}

id value(id object, NSString *key) {
    @try {return [object valueForKey:key];} @catch (NSException *) {return nil;}
}

void handle(id event) {
    NSString *process = value(event, @"process");
    const char *name = process.UTF8String ?: "?";
    if (!wanted(gFilter, name)) return;
    const long level = levelOfType([value(event, @"logType") integerValue]);
    if (level < gFilter.level) return;
    const time_t now = time(nullptr);
    if (now != gSecond) {gSecond = now; gSentThisSecond = 0;}
    if (gSentThisSecond >= gFilter.rate) {++gDropped; return;}
    ++gSentThisSecond;
    NSString *message = value(event, @"composedMessage");
    NSString *subsystem = value(event, @"subsystem"), *category = value(event, @"category");
    const int pid = [value(event, @"processIdentifier") intValue];
    std::string text = message.UTF8String ?: "";
    for (char &c : text) if (c == '\n' || c == '\r' || c == '\t') c = ' ';
    if (text.size() > 300) text.resize(300);
    char line[420];
    snprintf(line, sizeof line, "OSLOG %s[%d] %c %s:%s %s\n", name, pid, levelLetter(level), subsystem.UTF8String ?: "",
             category.UTF8String ?: "", text.c_str());
    emit(line);
}

bool startLiveStream() {
    char line[300];
    void *support = dlopen("/System/Library/PrivateFrameworks/LoggingSupport.framework/LoggingSupport", RTLD_NOW);
    Class storeClass = objc_getClass("OSLogEventLiveStore"), streamClass = objc_getClass("OSLogEventLiveStream");
    snprintf(line, sizeof line, "OSLOG LoggingSupport %s, live store %s, live stream %s\n", support ? "loaded" : (dlerror() ?: "not loaded"),
             storeClass ? "found" : "missing", streamClass ? "found" : "missing");
    emit(line);
    if (!storeClass || !streamClass || ![storeClass respondsToSelector:@selector(liveLocalStore)]) return false;
    id store = ((id (*)(Class, SEL))objc_msgSend)(storeClass, @selector(liveLocalStore));
    if (!store) {emit("OSLOG liveLocalStore answered nil\n"); return false;}
    gKeepStore = store;
    void (^prepared)(id, NSError *) = ^(id source, NSError *error) {
        if (!source) {
            char text[300];
            snprintf(text, sizeof text, "OSLOG live source refused: %s\n", (error ? error.description : @"nil, no error").UTF8String);
            emit(text);
            return;
        }
        id stream = ((id (*)(id, SEL, id))objc_msgSend)([streamClass alloc], @selector(initWithLiveSource:), source);
        if (!stream) {emit("OSLOG live stream: init failed\n"); return;}
        gKeepStream = stream;
        void (^events)(id) = ^(id event) {@autoreleasepool {handle(event);}};
        ((void (*)(id, SEL, id))objc_msgSend)(stream, @selector(setEventHandler:), events);
        if ([stream respondsToSelector:@selector(setInvalidationHandler:)]) {
            void (^invalid)(int, id) = ^(int reason, id) {
                char text[100];
                snprintf(text, sizeof text, "OSLOG live stream invalidated (%d)\n", reason);
                emit(text);
            };
            ((void (*)(id, SEL, id))objc_msgSend)(stream, @selector(setInvalidationHandler:), invalid);
        }
        ((void (*)(id, SEL))objc_msgSend)(stream, @selector(activate));
        emit("OSLOG live stream active\n");
    };
    ((void (*)(id, SEL, id))objc_msgSend)(store, @selector(prepareWithCompletionHandler:), prepared);
    return true;
}
}  // namespace

// The LaunchServices database (csstore) on Data: every one found is logged with its size. With /private/var/ngpu/oslog/lsrebuild (an addon
// marker) and no lsrebuild.done next to it, they are removed once and lsd is stopped: launchd starts it again on demand and it rebuilds
// the database from the system's bundles. pdx203: WebKit's WebContent extension (in the OS cryptex, reached through the addon's overlay)
// was not in the database ("Error launching process ... OSStatus error -10814"), so Safari never started a web process.
namespace {
void launchServicesDatabase() {
    char line[600];
    const bool rebuild = access("/private/var/ngpu/oslog/lsrebuild", F_OK) == 0 && access("/private/var/ngpu/oslog/lsrebuild2.done", F_OK) != 0;
    // every container tree (the system group containers included), the daemons' caches and /var/db; usb-128 found none in the narrower set
    char *roots[] = {(char *)"/private/var/containers", (char *)"/private/var/mobile/Containers", (char *)"/private/var/mobile/Library",
                     (char *)"/private/var/root/Library", (char *)"/private/var/db", nullptr};
    FTS *fts = fts_open(roots, FTS_PHYSICAL | FTS_NOCHDIR | FTS_XDEV, nullptr);
    unsigned found = 0, removed = 0;
    if (fts) {
        while (FTSENT *e = fts_read(fts)) {
            if (e->fts_level > 7 && e->fts_info == FTS_D) {fts_set(fts, e, FTS_SKIP); continue;}
            if (e->fts_info != FTS_F) continue;
            const size_t n = strlen(e->fts_name);
            if (n < 8 || strcmp(e->fts_name + n - 8, ".csstore")) continue;
            ++found;
            snprintf(line, sizeof line, "OSLOG csstore %s (%lld bytes)%s\n", e->fts_path, (long long)e->fts_statp->st_size, rebuild ? ", removing" : "");
            emit(line);
            if (rebuild && unlink(e->fts_path) == 0) ++removed;
        }
        fts_close(fts);
    }
    if (rebuild && !removed) {
        // No database file reachable from here (pdx203 usb-128/129: none found, likely the platform sandbox): LaunchServices' own rebuild,
        // the call uicache makes (entitlement com.apple.lsapplicationworkspace.rebuildappdatabases).
        Class workspaceClass = objc_getClass("LSApplicationWorkspace");
        id workspace = workspaceClass ? ((id (*)(Class, SEL))objc_msgSend)(workspaceClass, @selector(defaultWorkspace)) : nil;
        SEL rebuildSel = @selector(_LSPrivateRebuildApplicationDatabasesForSystemApps:internal:user:);
        bool ok = false;
        if (workspace && [workspace respondsToSelector:rebuildSel])
            ok = ((BOOL (*)(id, SEL, BOOL, BOOL, BOOL))objc_msgSend)(workspace, rebuildSel, YES, YES, YES);
        snprintf(line, sizeof line, "OSLOG csstore: %u found; LaunchServices rebuild through LSApplicationWorkspace: %s\n", found,
                 !workspace ? "no workspace" : ![workspace respondsToSelector:rebuildSel] ? "no such call" : ok ? "done" : "refused");
        emit(line);
        if (ok) {int fd = open("/private/var/ngpu/oslog/lsrebuild2.done", O_CREAT | O_WRONLY, 0644); if (fd >= 0) close(fd);}
        return;
    }
    if (!rebuild) {snprintf(line, sizeof line, "OSLOG csstore: %u found, %u removed\n", found, removed); emit(line); return;}
    int fd = open("/private/var/ngpu/oslog/lsrebuild2.done", O_CREAT | O_WRONLY, 0644);   // only once something was removed
    if (fd >= 0) close(fd);
    // lsd by name from the kernel's process table (sysctl kern.proc.all; proc_listallpids found nothing in usb-128)
    int mib[4] = {CTL_KERN, KERN_PROC, KERN_PROC_ALL, 0};
    size_t size = 0;
    unsigned killed = 0, seen = 0;
    if (sysctl(mib, 3, nullptr, &size, nullptr, 0) == 0 && size) {
        size += size / 4;
        struct kinfo_proc *procs = (struct kinfo_proc *)malloc(size);
        if (procs && sysctl(mib, 3, procs, &size, nullptr, 0) == 0) {
            seen = (unsigned)(size / sizeof *procs);
            for (unsigned i = 0; i < seen; ++i)
                if (!strcmp(procs[i].kp_proc.p_comm, "lsd") && kill(procs[i].kp_proc.p_pid, SIGKILL) == 0) ++killed;
        }
        free(procs);
    }
    snprintf(line, sizeof line, "OSLOG csstore: %u found, %u removed, lsd stopped %u times (%u processes): LaunchServices rebuilds its database\n",
             found, removed, killed, seen);
    emit(line);
}
}  // namespace

// The deletable system apps (Maps, Music, Stocks, ...): a restore leaves them as placeholders ("Waiting...") that the first-boot data
// migration fills from /private/var/staged_system_apps (kernel patch zzy-staged-system-apps copies them onto the owner Data volume). The
// migrator runs its plugins only when the build changes, so on this board it never did. With /private/var/ngpu/oslog/systemapps (an addon
// marker) and no systemapps.done next to it, the one plugin that installs them (SystemAppMigrator, through InstallCoordination with the
// entitlements com.apple.datamigrator has) runs here once, a minute after boot. A marker whose first line is "upgrade" runs it as after
// an OS update (didUpgrade).
namespace {
void installStagedSystemApps() {
    constexpr const char *kMarker = "/private/var/ngpu/oslog/systemapps", *kDone = "/private/var/ngpu/oslog/systemapps2.done";
    if (access(kMarker, F_OK) != 0 || access(kDone, F_OK) == 0) return;
    char first[32] = {};
    if (FILE *file = fopen(kMarker, "r")) {fgets(first, sizeof first, file); fclose(file);}
    const bool upgrade = !strncmp(first, "upgrade", 7);
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 60 * NSEC_PER_SEC), dispatch_get_global_queue(QOS_CLASS_UTILITY, 0), ^{
        @autoreleasepool {
            char line[400];
            struct stat staged;
            const bool present = stat("/private/var/staged_system_apps", &staged) == 0;
            void *framework = dlopen("/System/Library/PrivateFrameworks/DataMigration.framework/DataMigration", RTLD_NOW);
            Class migratorClass = objc_getClass("DataClassMigrator");
            id migrator = nil;
            if (migratorClass && [migratorClass respondsToSelector:@selector(dataClassMigratorForBundleAtPath:)])
                migrator = ((id (*)(Class, SEL, NSString *))objc_msgSend)(migratorClass, @selector(dataClassMigratorForBundleAtPath:),
                                                                          @"/System/Library/DataClassMigrators/SystemAppMigrator.migrator");
            snprintf(line, sizeof line, "OSLOG system apps: staged directory %s, DataMigration %s, migrator %s%s\n", present ? "present" : "missing",
                     framework ? "loaded" : (dlerror() ?: "not loaded"), migrator ? object_getClassName(migrator) : "none",
                     upgrade ? ", as an upgrade" : "");
            emit(line);
            // SystemAppMigrator checks every installed system app's code against the trust cache through MIS, which fails here ("Error from
            // MIS ... An unknown error"), and demotes the app to a placeholder (pdx203 usb-139: the apps the previous run installed lost their
            // bundles). Its own escape (a SystemAppMigratorIgnoreAppsNotInTrustCache file in installd's data container) needs
            // containermanagerd, which this root job cannot reach (usb-140: MIDaemonConfiguration aborted the forwarder). The plugin runs in
            // this process, so the two methods answer here instead: every installed app counts as in the trust cache, and failures are ignored.
            if (Class appMigrator = objc_getClass("MISystemAppMigrator")) {
                const bool a = class_replaceMethod(appMigrator, @selector(shouldIgnoreAppsNotInTrustCache),
                                                   imp_implementationWithBlock(^BOOL(id) { return YES; }), "B16@0:8") != nullptr;
                const bool b = class_replaceMethod(appMigrator, NSSelectorFromString(@"isCurrentlyInstalledAppInTrustCacheForBundleID:"),
                                                   imp_implementationWithBlock(^BOOL(id, id) { return YES; }), "B24@0:8@16") != nullptr;
                snprintf(line, sizeof line, "OSLOG system apps: trust cache checks answered here (replaced %s, %s)\n", a ? "ignore" : "-",
                         b ? "in-cache" : "-");
                emit(line);
            }
            // What the staged directory holds (pdx203 usb-136: "Did not have a staged app/patch" for every app after the first run)
            NSArray *names = [[NSFileManager defaultManager] contentsOfDirectoryAtPath:@"/private/var/staged_system_apps" error:nil];
            if (names) {
                NSString *joined = [names componentsJoinedByString:@","];
                snprintf(line, sizeof line, "OSLOG system apps: %lu staged: %.320s\n", (unsigned long)names.count, joined.UTF8String);
                emit(line);
            }
            // SystemAppMigrator removes the staged directory's content once it has run, installed or not (usb-137: empty after the run of
            // usb-135, which could not reach installcoordinationd). With nothing staged, the kernel's copy marker goes, so the next boot
            // copies the apps again (zzy-staged-system-apps, sheng-staged-apps=1), and the migrator runs then.
            if (!names.count) {
                const int removed = unlink("/private/var/db/.sheng-staged-apps");
                snprintf(line, sizeof line, "OSLOG system apps: nothing staged; kernel copy marker %s, the next boot copies them again\n",
                         removed == 0 ? "removed" : strerror(errno));
                emit(line);
                return;
            }
            if (!migrator) return;
            if (upgrade && [migrator respondsToSelector:@selector(setDidUpgrade:)])
                ((void (*)(id, SEL, BOOL))objc_msgSend)(migrator, @selector(setDidUpgrade:), YES);
            const BOOL ok = ((BOOL (*)(id, SEL))objc_msgSend)(migrator, @selector(performMigration));
            snprintf(line, sizeof line, "OSLOG system apps: SystemAppMigrator performMigration %s\n", ok ? "succeeded" : "failed");
            emit(line);
            if ([migrator respondsToSelector:@selector(itemsToInstall)]) {
                NSDictionary *items = ((id (*)(id, SEL))objc_msgSend)(migrator, @selector(itemsToInstall));
                NSString *text = [[items.description componentsSeparatedByCharactersInSet:NSCharacterSet.newlineCharacterSet]
                    componentsJoinedByString:@" "];
                for (NSUInteger at = 0; at < text.length && at < 6000; at += 300) {
                    NSString *part = [text substringWithRange:NSMakeRange(at, MIN((NSUInteger)300, text.length - at))];
                    snprintf(line, sizeof line, "OSLOG system apps: items %s\n", part.UTF8String);
                    emit(line);
                }
            }
            // Done after any run that had staged apps: the migrator reports failure for side issues (usb-142: "Failed to mark com.apple.Fitness
            // as uninstalled") with every app installed, and each rerun costs a staged copy of 231 MB two boots later.
            {int fd = open(kDone, O_CREAT | O_WRONLY, 0644); if (fd >= 0) close(fd);}
            (void)ok;
        }
    });
}
}  // namespace

// CPU use on the USB console, every 5 s while something is busy (and every 60 s regardless):
//   CPUTOP <total>% | <process>[pid] <pct> ...            the busiest processes, as % of one core (8 cores: 800 % at most)
//   CPUTHR <process>[pid] <pct>: <thread> <pct> ...       the busiest threads of the three busiest processes
// The pids come from sysctl kern.proc.all (proc_listallpids answered nothing here, usb-128); the times from proc_pidinfo (PROC_PIDTASKINFO in
// Mach ticks, PROC_PIDTHREADINFO per thread). A thread's share is its part of its process's threads' time, so the units of the two cancel.
// pdx203: Safari's pages draw at 24 fps where Settings scrolls at 37-47 (2026-11-03); this says which process and thread is the limit.
namespace {
struct TaskInfo {   // <sys/proc_info.h> struct proc_taskinfo (not in the iOS SDK)
    uint64_t virtual_size, resident_size, total_user, total_system, threads_user, threads_system;
    int32_t policy, faults, pageins, cow_faults, messages_sent, messages_received, syscalls_mach, syscalls_unix, csw, threadnum, numrunning,
        priority;
};
struct ThreadInfo {   // struct proc_threadinfo
    uint64_t user_time, system_time;
    int32_t cpu_usage, policy, run_state, flags, sleep_time, curpri, priority, maxpriority;
    char name[64];
};
constexpr int kPidTaskInfo = 4, kPidThreadInfo = 5, kPidListThreads = 6;
extern "C" int proc_pidinfo(int pid, int flavor, uint64_t arg, void *buffer, int size);

struct Sample { int pid; char name[20]; uint64_t ticks, delta; };

// A process's images for naming sampled addresses: "dsc:<unslid address>" in the shared cache (symbolize on the host against the
// shared cache's symbols), "<image>+<offset>" in another image, "jit" in JavaScriptCore's JIT region, else the raw address.
struct TaskImages {
    struct Image { uint64_t base; char name[40]; };
    std::vector<Image> images;
    uint64_t slide = 0, cacheBase = 0, jitStart = 0, jitEnd = 0;
    mach_port_t task = MACH_PORT_NULL;
    void load(mach_port_t t) {
        task = t;
        task_dyld_info_data_t dyld = {};
        mach_msg_type_number_t count = TASK_DYLD_INFO_COUNT;
        if (task_info(task, TASK_DYLD_INFO, (task_info_t)&dyld, &count) != KERN_SUCCESS || !dyld.all_image_info_addr) return;
        struct dyld_all_image_infos all = {};
        vm_size_t got = 0;
        if (vm_read_overwrite(task, dyld.all_image_info_addr, sizeof all, (vm_address_t)&all, &got) != KERN_SUCCESS) return;
        slide = all.sharedCacheSlide;
        cacheBase = all.sharedCacheBaseAddress;
        std::vector<struct dyld_image_info> infos(all.infoArrayCount < 2048 ? all.infoArrayCount : 2048);
        if (infos.empty() || vm_read_overwrite(task, (vm_address_t)all.infoArray, infos.size() * sizeof infos[0],
                                               (vm_address_t)infos.data(), &got) != KERN_SUCCESS) return;
        for (const auto &info : infos) {
            const uint64_t base = (uint64_t)info.imageLoadAddress;
            if (cacheBase && base >= cacheBase && base < cacheBase + 0x200000000ull) continue;
            char path[256] = {};
            vm_read_overwrite(task, (vm_address_t)info.imageFilePath, sizeof path - 1, (vm_address_t)path, &got);
            const char *name = strrchr(path, '/') ? strrchr(path, '/') + 1 : path;
            Image image = {base, {}};
            strlcpy(image.name, name, sizeof image.name);
            images.push_back(image);
        }
    }
    // The region holding pc, when it is JavaScriptCore's executable allocator (VM_MEMORY_JAVASCRIPT_JIT_EXECUTABLE_ALLOCATOR, 64).
    bool inJit(uint64_t pc) {
        if (pc >= jitStart && pc < jitEnd) return true;
        vm_address_t address = pc;
        vm_size_t size = 0;
        natural_t depth = 0;
        vm_region_submap_info_data_64_t info = {};
        mach_msg_type_number_t count = VM_REGION_SUBMAP_INFO_COUNT_64;
        if (vm_region_recurse_64(task, &address, &size, &depth, (vm_region_recurse_info_t)&info, &count) != KERN_SUCCESS) return false;
        if (address > pc || info.user_tag != 64) return false;
        jitStart = address; jitEnd = address + size;
        return true;
    }
    void where(uint64_t pc, char *out, size_t size) {
        if (cacheBase && pc >= cacheBase && pc < cacheBase + 0x200000000ull) {snprintf(out, size, "dsc:%llx", (unsigned long long)(pc - slide)); return;}
        const Image *best = nullptr;
        for (const Image &image : images) if (image.base <= pc && (!best || image.base > best->base)) best = &image;
        if (best && pc - best->base < 0x4000000ull) {snprintf(out, size, "%s+%llx", best->name, (unsigned long long)(pc - best->base)); return;}
        if (task != MACH_PORT_NULL && inJit(pc)) {strlcpy(out, "jit", size); return;}
        snprintf(out, size, "%llx", (unsigned long long)pc);
    }
};

// Rows of 10 "<where> <count>" pairs (the counts are samples, not percent), the first `limit` of a list sorted by count.
void emitRows(const char *tag, const char *process, int pid, unsigned samples, const std::vector<std::pair<std::string, unsigned>> &list,
              unsigned limit) {
    char line[480];
    for (unsigned first = 0; first < list.size() && first < limit; first += 10) {
        int at = snprintf(line, sizeof line, "%s %s[%d] n%u r%u:", tag, process, pid, samples, first / 10);
        for (unsigned i = first; i < list.size() && i < first + 10 && at < (int)sizeof line - 60; ++i)
            at += snprintf(line + at, sizeof line - at, " %s %u", list[i].first.c_str(), list[i].second);
        snprintf(line + at, sizeof line - at, "\n");
        emit(line);
    }
}

// kperf, XNU's own sampler (KPERF is in this kernel): a 1 ms timer on every CPU samples the thread running there when it belongs to the
// profiled process, and the kernel walks that thread's user stack when it next returns to user space (an AST on the same CPU). The leaf
// is where the thread really was, not its next system call as with thread_get_state (profile below). One second, then the events are
// read back from the kdebug buffer (class DBG_PERF only). It needs root and an unowned ktrace (nothing else tracing).
//   CPUKC <process>[pid] n<samples> r<row>: <where> <samples> ...   leaf PCs, 10 a row
//   CPUKI <process>[pid] n<samples> r<row>: <where> <samples> ...   every address in the stack once a sample (inclusive)
//   CPUKS <process>[pid] <samples>: <where> <where> ...             the most frequent stacks, leaf first, 7 frames
//   CPUKT <process>[pid] n<samples> e<events> w<wrapped>: <thread> <samples> ...
// Addresses as TaskImages::where. An error at setup is reported as "CPUKP <step> errno <n>" (the first three times).
#ifndef KERN_KDENABLE
#define KERN_KDENABLE 3
#endif
#ifndef KERN_KDSETBUF
#define KERN_KDSETBUF 4
#endif
#ifndef KERN_KDGETBUF
#define KERN_KDGETBUF 5
#endif
#ifndef KERN_KDSETUP
#define KERN_KDSETUP 6
#endif
#ifndef KERN_KDREMOVE
#define KERN_KDREMOVE 7
#endif
#ifndef KERN_KDREADTR
#define KERN_KDREADTR 10
#endif
#ifndef KERN_KDSET_TYPEFILTER
#define KERN_KDSET_TYPEFILTER 22
#endif
struct KdBuf { uint64_t timestamp, arg1, arg2, arg3, arg4, arg5; uint32_t debugid, cpuid; uint64_t unused; };
static_assert(sizeof(KdBuf) == 64, "kd_buf");
// kbufinfo_t for KERN_KDGETBUF: the flags say whether the buffer wrapped (KDBG_WRAPPED 0x08).
struct KdBufInfo { int nkdbufs, nolog, flags, nkdthreads, bufid; };
int kdebugOp(int op, int value, bool withValue) {
    int mib[4] = {CTL_KERN, KERN_KDEBUG, op, value};
    size_t zero = 0;
    return sysctl(mib, withValue ? 4 : 3, nullptr, &zero, nullptr, 0) ? errno : 0;
}
int kperfPair(const char *name, uint64_t a, uint64_t b) {
    uint64_t in[2] = {a, b};
    return sysctlbyname(name, nullptr, nullptr, in, sizeof in) ? errno : 0;
}
int kperfValue(const char *name, uint32_t value) {return sysctlbyname(name, nullptr, nullptr, &value, sizeof value) ? errno : 0;}
// The kernel's interrupt-time sampler (ArmCpu 0001, sysctl kern.armcpu_upc): for two seconds every interrupt taken on a core running
// this process's user code records the interrupted PC, the thread's LR and thread ID. Unlike thread_get_state (profile below), the PC is
// where the thread really was. Rows (counts are samples):
//   CPUIC <process>[pid] n<samples> r<row>: <where> <samples> ...   interrupted PCs, 10 a row
//   CPUIL <process>[pid] n<samples> r<row>: <where> <samples> ...   their LRs (the caller, for a leaf that has not saved it)
//   CPUIT <process>[pid] n<samples> irq<wanted> el1<in the kernel> other<other process> bad<saved PC != ELR>: <thread> <samples> ...
// Addresses as TaskImages::where. False when the sysctl is missing or refuses (then thread_get_state's profile runs).
bool irqProfile(int pid, const char *process, mach_port_t task) {
    static int failures;
    char line[480];
    int32_t value = pid;
    if (sysctlbyname("kern.armcpu_upc", nullptr, nullptr, &value, sizeof value)) {
        if (failures++ < 2) {snprintf(line, sizeof line, "CPUIT %s[%d]: kern.armcpu_upc errno %d\n", process, pid, errno); emit(line);}
        return false;
    }
    std::vector<std::pair<uint64_t, std::string>> names;
    thread_act_array_t threads = nullptr;
    mach_msg_type_number_t threadCount = 0;
    if (task_threads(task, &threads, &threadCount) == KERN_SUCCESS) {
        for (unsigned t = 0; t < threadCount; ++t) {
            thread_identifier_info_data_t ident = {};
            mach_msg_type_number_t n = THREAD_IDENTIFIER_INFO_COUNT;
            thread_extended_info_data_t ext = {};
            mach_msg_type_number_t m = THREAD_EXTENDED_INFO_COUNT;
            if (thread_info(threads[t], THREAD_IDENTIFIER_INFO, (thread_info_t)&ident, &n) == KERN_SUCCESS) {
                std::string name = thread_info(threads[t], THREAD_EXTENDED_INFO, (thread_info_t)&ext, &m) == KERN_SUCCESS && ext.pth_name[0]
                                       ? std::string(ext.pth_name) : "t" + std::to_string(t);
                for (char &c : name) if (c == ' ') c = '_';
                if (name.size() > 28) name.resize(28);
                names.push_back({ident.thread_id, name});
            }
            mach_port_deallocate(mach_task_self(), threads[t]);
        }
        vm_deallocate(mach_task_self(), (vm_address_t)threads, threadCount * sizeof threads[0]);
    }
    usleep(2000000);
    value = 0;
    sysctlbyname("kern.armcpu_upc", nullptr, nullptr, &value, sizeof value);
    struct Record { uint64_t pc, lr, tid; };
    std::vector<uint8_t> buffer(32 + 8192 * sizeof(Record));
    size_t size = buffer.size();
    if (sysctlbyname("kern.armcpu_upc", buffer.data(), &size, nullptr, 0) || size < 32) {
        if (failures++ < 2) {snprintf(line, sizeof line, "CPUIT %s[%d]: read errno %d\n", process, pid, errno); emit(line);}
        return false;
    }
    uint32_t header[8];
    memcpy(header, buffer.data(), sizeof header);
    const size_t records = std::min<size_t>(header[0], (size - 32) / sizeof(Record));
    const Record *r = reinterpret_cast<const Record *>(buffer.data() + 32);
    TaskImages images;
    images.load(task);
    std::vector<std::pair<uint64_t, unsigned>> pcs, lrs, perThread;
    auto bump = [](std::vector<std::pair<uint64_t, unsigned>> &list, uint64_t key) {
        for (auto &e : list) if (e.first == key) {++e.second; return;}
        list.push_back({key, 1});
    };
    for (size_t i = 0; i < records; ++i) {
        bump(pcs, r[i].pc & 0x0000000fffffffffull);
        bump(lrs, r[i].lr & 0x0000000fffffffffull);
        bump(perThread, r[i].tid);
    }
    auto named = [&](const std::vector<std::pair<uint64_t, unsigned>> &list) {
        std::vector<std::pair<std::string, unsigned>> out;
        for (const auto &e : list) {
            char where[64];
            images.where(e.first, where, sizeof where);
            bool hit = false;
            for (auto &o : out) if (o.first == where) {o.second += e.second; hit = true; break;}
            if (!hit) out.push_back({where, e.second});
        }
        std::sort(out.begin(), out.end(), [](const auto &a, const auto &b) { return a.second > b.second; });
        return out;
    };
    if (records) {
        emitRows("CPUIC", process, pid, (unsigned)records, named(pcs), 60);
        emitRows("CPUIL", process, pid, (unsigned)records, named(lrs), 40);
    }
    std::sort(perThread.begin(), perThread.end(), [](const auto &a, const auto &b) { return a.second > b.second; });
    int at = snprintf(line, sizeof line, "CPUIT %s[%d] n%zu irq%u el1 %u other %u bad %u:", process, pid, records, header[1], header[2], header[3], header[4]);
    for (unsigned i = 0; i < perThread.size() && i < 10 && at < (int)sizeof line - 40; ++i) {
        std::string name = "tid" + std::to_string(perThread[i].first);
        for (const auto &n : names) if (n.first == perThread[i].first) name = n.second;
        at += snprintf(line + at, sizeof line - at, " %s %u", name.c_str(), perThread[i].second);
    }
    snprintf(line + at, sizeof line - at, "\n");
    emit(line);
    return true;
}
bool kperfProfile(int pid, const char *process, mach_port_t task) {
    static int failures;
    char line[480];
    constexpr int kEvents = 65536;
    constexpr uint32_t kUstack = 1u << 3, kThInfo = 1u << 0;
    constexpr uint32_t kUhdr = (37u << 24) | (2u << 16) | (6u << 2), kUdata = (37u << 24) | (2u << 16) | (4u << 2);
    mach_timebase_info_data_t timebase;
    mach_timebase_info(&timebase);
    const uint64_t periodTicks = 1000000ull * timebase.denom / timebase.numer;   // 1 ms
    const char *step = nullptr;
    int error = 0;
    auto check = [&](const char *what, int e) {if (e && !step) {step = what; error = e;} return !e;};
    // A previous run's leftovers, then the buffer, the type filter (DBG_PERF's 256 subclasses: bits 37*256..37*256+255) and kperf's
    // action 1 (user stacks and thread info, this pid only) on timer 0.
    kdebugOp(KERN_KDREMOVE, 0, false);
    kperfValue("kperf.reset", 1);
    check("setbuf", kdebugOp(KERN_KDSETBUF, kEvents, true)) && check("setup", kdebugOp(KERN_KDSETUP, 0, false));
    if (!step) {
        std::vector<uint8_t> filter(8192, 0);
        memset(filter.data() + 37 * 32, 0xff, 32);
        int mib[3] = {CTL_KERN, KERN_KDEBUG, KERN_KDSET_TYPEFILTER};
        size_t size = filter.size();
        check("typefilter", sysctl(mib, 3, filter.data(), &size, nullptr, 0) ? errno : 0);
    }
    if (!step) check("action.count", kperfValue("kperf.action.count", 1)) &&
        check("action.samplers", kperfPair("kperf.action.samplers", 1, kUstack | kThInfo)) &&
        check("action.filter_by_pid", kperfPair("kperf.action.filter_by_pid", 1, (uint64_t)pid)) &&
        check("action.ucallstack_depth", kperfPair("kperf.action.ucallstack_depth", 1, 48)) &&
        check("timer.count", kperfValue("kperf.timer.count", 1)) &&
        check("timer.period", kperfPair("kperf.timer.period", 0, periodTicks)) &&
        check("timer.action", kperfPair("kperf.timer.action", 0, 1)) &&
        check("kdenable", kdebugOp(KERN_KDENABLE, 1, true)) &&
        check("sampling", kperfValue("kperf.sampling", 1));
    if (step) {
        kperfValue("kperf.reset", 1);
        kdebugOp(KERN_KDREMOVE, 0, false);
        if (failures++ < 3) {snprintf(line, sizeof line, "CPUKP %s errno %d\n", step, error); emit(line);}
        return false;
    }
    // Thread names by thread ID meanwhile (the samples carry the ID).
    std::vector<std::pair<uint64_t, std::string>> names;
    thread_act_array_t threads = nullptr;
    mach_msg_type_number_t threadCount = 0;
    if (task_threads(task, &threads, &threadCount) == KERN_SUCCESS) {
        for (unsigned t = 0; t < threadCount; ++t) {
            thread_identifier_info_data_t ident = {};
            mach_msg_type_number_t n = THREAD_IDENTIFIER_INFO_COUNT;
            thread_extended_info_data_t ext = {};
            mach_msg_type_number_t m = THREAD_EXTENDED_INFO_COUNT;
            if (thread_info(threads[t], THREAD_IDENTIFIER_INFO, (thread_info_t)&ident, &n) == KERN_SUCCESS) {
                std::string name = thread_info(threads[t], THREAD_EXTENDED_INFO, (thread_info_t)&ext, &m) == KERN_SUCCESS && ext.pth_name[0]
                                       ? std::string(ext.pth_name) : "t" + std::to_string(t);
                for (char &c : name) if (c == ' ') c = '_';
                if (name.size() > 28) name.resize(28);
                names.push_back({ident.thread_id, name});
            }
            mach_port_deallocate(mach_task_self(), threads[t]);
        }
        vm_deallocate(mach_task_self(), (vm_address_t)threads, threadCount * sizeof threads[0]);
    }
    usleep(1000000);
    kperfValue("kperf.sampling", 0);
    kdebugOp(KERN_KDENABLE, 0, true);
    KdBufInfo info = {};
    {
        int mib[3] = {CTL_KERN, KERN_KDEBUG, KERN_KDGETBUF};
        size_t size = sizeof info;
        sysctl(mib, 3, &info, &size, nullptr, 0);
    }
    std::vector<KdBuf> events(kEvents);
    size_t count = events.size() * sizeof(KdBuf);
    {
        int mib[3] = {CTL_KERN, KERN_KDEBUG, KERN_KDREADTR};
        if (sysctl(mib, 3, events.data(), &count, nullptr, 0)) count = 0;
    }
    kperfValue("kperf.reset", 1);
    kdebugOp(KERN_KDREMOVE, 0, false);
    // Each sample: a UHDR (flags, frames, async index, async frames) and then UDATA events of four frames, all on the sampled thread;
    // the last frame of the stack is kperf's fixup word (the caller of a leaf without a frame), not a frame.
    struct Pending { std::vector<uint64_t> frames; unsigned need; };
    std::vector<std::pair<uint64_t, Pending>> pending;
    std::vector<std::pair<uint64_t, unsigned>> leafs, inclusive, perThread;
    std::vector<std::pair<std::vector<uint64_t>, unsigned>> stacks;
    unsigned samples = 0;
    auto bump = [](std::vector<std::pair<uint64_t, unsigned>> &list, uint64_t key, size_t cap) {
        for (auto &e : list) if (e.first == key) {++e.second; return;}
        if (list.size() < cap) list.push_back({key, 1});
    };
    for (size_t i = 0; i < count; ++i) {
        const KdBuf &e = events[i];
        const uint32_t code = e.debugid & ~3u;
        if (code != kUhdr && code != kUdata) continue;
        Pending *p = nullptr;
        for (auto &q : pending) if (q.first == e.arg5) p = &q.second;
        if (code == kUhdr) {
            if (!p) {pending.push_back({e.arg5, {}}); p = &pending.back().second;}
            p->frames.clear();
            p->need = (e.arg1 & 1) ? (unsigned)(e.arg2 + e.arg4) : 0;   // CALLSTACK_VALID
            if (p->need > 256) p->need = 0;
            continue;
        }
        if (!p || !p->need) continue;
        const uint64_t words[4] = {e.arg1, e.arg2, e.arg3, e.arg4};
        for (unsigned k = 0; k < 4 && p->frames.size() < p->need; ++k) p->frames.push_back(words[k] & 0x0000000fffffffffull);
        if (p->frames.size() < p->need) continue;
        std::vector<uint64_t> frames = p->frames;
        p->need = 0;
        if (frames.size() >= 2) frames.pop_back();   // the fixup word
        if (frames.empty() || !frames[0]) continue;
        ++samples;
        bump(perThread, e.arg5, 256);
        bump(leafs, frames[0], 8192);
        for (unsigned c = 0; c < frames.size(); ++c) {
            bool dup = false;
            for (unsigned d = 0; d < c && !dup; ++d) dup = frames[d] == frames[c];
            if (!dup) bump(inclusive, frames[c], 16384);
        }
        std::vector<uint64_t> top(frames.begin(), frames.begin() + std::min<size_t>(frames.size(), 7));
        bool hit = false;
        for (auto &s : stacks) if (s.first == top) {++s.second; hit = true; break;}
        if (!hit && stacks.size() < 4096) stacks.push_back({top, 1});
    }
    const bool wrapped = info.flags & 0x08;
    TaskImages images;
    images.load(task);
    // Names, then merged by name: every JIT address is one "jit" entry.
    auto named = [&](std::vector<std::pair<uint64_t, unsigned>> &list) {
        std::vector<std::pair<std::string, unsigned>> out;
        for (const auto &e : list) {
            char where[64];
            images.where(e.first, where, sizeof where);
            bool hit = false;
            for (auto &o : out) if (o.first == where) {o.second += e.second; hit = true; break;}
            if (!hit) out.push_back({where, e.second});
        }
        std::sort(out.begin(), out.end(), [](const auto &a, const auto &b) { return a.second > b.second; });
        return out;
    };
    if (samples) {
        emitRows("CPUKC", process, pid, samples, named(leafs), 50);
        emitRows("CPUKI", process, pid, samples, named(inclusive), 100);
        std::sort(stacks.begin(), stacks.end(), [](const auto &a, const auto &b) { return a.second > b.second; });
        for (unsigned s = 0; s < stacks.size() && s < 16; ++s) {
            int at = snprintf(line, sizeof line, "CPUKS %s[%d] %u:", process, pid, stacks[s].second);
            for (uint64_t pc : stacks[s].first) {
                char where[64];
                images.where(pc, where, sizeof where);
                if (at < (int)sizeof line - 70) at += snprintf(line + at, sizeof line - at, " %s", where);
            }
            snprintf(line + at, sizeof line - at, "\n");
            emit(line);
        }
    }
    std::sort(perThread.begin(), perThread.end(), [](const auto &a, const auto &b) { return a.second > b.second; });
    int at = snprintf(line, sizeof line, "CPUKT %s[%d] n%u e%zu w%d:", process, pid, samples, count, wrapped);
    for (unsigned i = 0; i < perThread.size() && i < 10 && at < (int)sizeof line - 40; ++i) {
        std::string name = "tid" + std::to_string(perThread[i].first);
        for (const auto &n : names) if (n.first == perThread[i].first) name = n.second;
        at += snprintf(line + at, sizeof line - at, " %s %u", name.c_str(), perThread[i].second);
    }
    snprintf(line + at, sizeof line - at, "\n");
    emit(line);
    return true;
}

// A one-second PC profile of a busy process (see cpuTopPass: every 10 s one of the three busiest over 40 %, in turn): the user PC of
// every thread whose CPU time moved since its last look (thread_get_state), plus its callers (lr and the frame-pointer chain), the hottest
// as "dsc:<unslid address>" in the shared cache (symbolize on the host against the shared cache's symbols) or "<image>+<offset>" in
// another image; then the samples by thread name, and the MIBs (or name2oid strings) of threads caught in __sysctl:
//   CPUPC <process>[pid] n<samples> r<row>: <where> <samples> ...   leaf PCs, 10 a row
//   CPUPI <process>[pid] n<samples> r<row>: <where> <samples> ...   return addresses, each once a sample (inclusive)
//   CPUPT <process>[pid]: <thread> <samples> ...
//   CPUSC <process>[pid] n<samples>: <mib or n2o:name> <samples> ...
// Only threads running at the look are sampled (TH_STATE_RUNNING); the counts are of those looks, so a thread's share is of its
// running time. Caveat (pdx203, 2026-11-03): before that filter the leaf PCs leaned to system calls (blocked threads were sampled too).
// It needs a task read port (task_for_pid with the task-port grants in build-oslog-console.py); a refusal is reported once.
extern "C" kern_return_t task_read_for_pid(mach_port_t, int, mach_port_t *) __attribute__((weak_import));
void profile(int pid, const char *process) {
    static int refusals;
    mach_port_t task = MACH_PORT_NULL;
    kern_return_t kr = task_for_pid(mach_task_self(), pid, &task);
    if (kr != KERN_SUCCESS && task_read_for_pid) kr = task_read_for_pid(mach_task_self(), pid, &task);
    char line[480];
    if (kr != KERN_SUCCESS) {
        if (refusals++ < 2) {snprintf(line, sizeof line, "CPUPC %s[%d]: no task port (0x%x)\n", process, pid, kr); emit(line);}
        return;
    }
    // kperf's samples where it works; thread_get_state's below otherwise. Off: sa63 (usb-211) froze for good within a second of its
    // first kperf setup (no panic text, the USB log stopped), so it stays off until that is understood.
    constexpr bool kUseKperf = false;
    if (kUseKperf && kperfProfile(pid, process, task)) {mach_port_deallocate(mach_task_self(), task); return;}
    if (irqProfile(pid, process, task)) {mach_port_deallocate(mach_task_self(), task); return;}
    // The images: the shared cache's slide and base, and every other image's load address and name.
    struct Image { uint64_t base; char name[40]; };
    std::vector<Image> images;
    uint64_t slide = 0, cacheBase = 0;
    task_dyld_info_data_t dyld = {};
    mach_msg_type_number_t count = TASK_DYLD_INFO_COUNT;
    if (task_info(task, TASK_DYLD_INFO, (task_info_t)&dyld, &count) == KERN_SUCCESS && dyld.all_image_info_addr) {
        struct dyld_all_image_infos all = {};
        vm_size_t got = 0;
        if (vm_read_overwrite(task, dyld.all_image_info_addr, sizeof all, (vm_address_t)&all, &got) == KERN_SUCCESS) {
            slide = all.sharedCacheSlide;
            cacheBase = all.sharedCacheBaseAddress;
            std::vector<struct dyld_image_info> infos(all.infoArrayCount < 2048 ? all.infoArrayCount : 2048);
            if (!infos.empty() && vm_read_overwrite(task, (vm_address_t)all.infoArray, infos.size() * sizeof infos[0],
                                                         (vm_address_t)infos.data(), &got) == KERN_SUCCESS) {
                for (const auto &info : infos) {
                    const uint64_t base = (uint64_t)info.imageLoadAddress;
                    if (cacheBase && base >= cacheBase && base < cacheBase + 0x200000000ull) continue;
                    char path[256] = {};
                    vm_read_overwrite(task, (vm_address_t)info.imageFilePath, sizeof path - 1, (vm_address_t)path, &got);
                    const char *name = strrchr(path, '/') ? strrchr(path, '/') + 1 : path;
                    Image image = {base, {}};
                    strlcpy(image.name, name, sizeof image.name);
                    images.push_back(image);
                }
            }
        }
    }
    thread_act_array_t threads = nullptr;
    mach_msg_type_number_t threadCount = 0;
    if (task_threads(task, &threads, &threadCount) != KERN_SUCCESS) {mach_port_deallocate(mach_task_self(), task); return;}
    std::vector<std::string> names(threadCount);
    for (unsigned t = 0; t < threadCount; ++t) {
        thread_extended_info_data_t ext = {};
        mach_msg_type_number_t n = THREAD_EXTENDED_INFO_COUNT;
        if (thread_info(threads[t], THREAD_EXTENDED_INFO, (thread_info_t)&ext, &n) == KERN_SUCCESS && ext.pth_name[0]) names[t] = ext.pth_name;
        else names[t] = "t" + std::to_string(t);
        for (char &c : names[t]) if (c == ' ') c = '_';
        if (names[t].size() > 28) names[t].resize(28);
    }
    std::vector<std::pair<uint64_t, unsigned>> pcs, callers;
    std::vector<unsigned> perThread(threadCount);
    std::vector<uint64_t> cpuSeen(threadCount);
    std::vector<std::pair<std::string, unsigned>> sysctls;
    // __sysctl's syscall instruction in the target (the shared cache maps at the same address in every process of a boot).
    static uint64_t sysctlPc;
    if (!sysctlPc) if (void *f = dlsym(RTLD_DEFAULT, "__sysctl")) sysctlPc = (uint64_t)f & 0x0000000fffffffffull;
    unsigned samples = 0;
    const uint64_t end = clock_gettime_nsec_np(CLOCK_UPTIME_RAW) + 1000000000ull;
    while (clock_gettime_nsec_np(CLOCK_UPTIME_RAW) < end) {
        for (unsigned t = 0; t < threadCount; ++t) {
            thread_basic_info_data_t basic = {};
            mach_msg_type_number_t n = THREAD_BASIC_INFO_COUNT;
            if (thread_info(threads[t], THREAD_BASIC_INFO, (thread_info_t)&basic, &n) != KERN_SUCCESS) continue;
            // Only a thread whose CPU time moved since its last look (2026-11-03: a run loop asleep in mach_msg still showed as running
            // and made 70 % of WebKit's samples); the first look at each thread only records its time.
            const uint64_t cpu = (uint64_t)(basic.user_time.seconds + basic.system_time.seconds) * 1000000ull +
                                 (uint64_t)(basic.user_time.microseconds + basic.system_time.microseconds);
            const uint64_t before = cpuSeen[t];
            cpuSeen[t] = cpu + 1;
            if (!before || cpu + 1 == before) continue;
            // And only one running now: one that ran since the last look but is blocked again would be caught at its system call
            // (2026-11-03, sa62: WebKit's samples were 35 % mach_msg and 63 % workq_kernreturn, its busy main thread hardly seen).
            if (basic.run_state != TH_STATE_RUNNING) continue;
            arm_thread_state64_t state = {};
            n = ARM_THREAD_STATE64_COUNT;
            if (thread_get_state(threads[t], ARM_THREAD_STATE64, (thread_state_t)&state, &n) != KERN_SUCCESS) continue;
            const uint64_t pc = arm_thread_state64_get_pc(state) & 0x0000000fffffffffull;
            // A thread in __sysctl: which MIB (x0 the name, x1 its length); for name2oid ({0, 3}, what sysctlbyname does first) the
            // name string in x4.
            if (sysctlPc && pc >= sysctlPc && pc < sysctlPc + 16) {
                int mib[8] = {};
                const uint64_t nameAt = state.__x[0] & 0x0000000fffffffffull, len = state.__x[1];
                vm_size_t got = 0;
                char what[64] = {};
                if (len >= 1 && len <= 8 && vm_read_overwrite(task, (vm_address_t)nameAt, len * sizeof(int), (vm_address_t)mib, &got) == KERN_SUCCESS) {
                    if (len >= 2 && mib[0] == 0 && mib[1] == 3) {
                        char text[48] = {};
                        if (vm_read_overwrite(task, (vm_address_t)(state.__x[4] & 0x0000000fffffffffull), sizeof text - 1, (vm_address_t)text, &got) != KERN_SUCCESS) strlcpy(text, "?", sizeof text);
                        snprintf(what, sizeof what, "n2o:%.40s", text);
                    } else {
                        int at = 0;
                        for (unsigned i = 0; i < len && at < (int)sizeof what - 12; ++i) at += snprintf(what + at, sizeof what - at, i ? ".%d" : "%d", mib[i]);
                    }
                    bool hit = false;
                    for (auto &e : sysctls) if (e.first == what) {++e.second; hit = true; break;}
                    if (!hit && sysctls.size() < 32) sysctls.push_back({what, 1});
                }
            }
            ++samples; ++perThread[t];
            bool found = false;
            for (auto &e : pcs) if (e.first == pc) {++e.second; found = true; break;}
            if (!found && pcs.size() < 4096) pcs.push_back({pc, 1});
            // The callers: lr and the frame-pointer chain (16 frames), each return address once per sample (inclusive counts).
            uint64_t chain[18]; unsigned depth = 0;
            chain[depth++] = arm_thread_state64_get_lr(state) & 0x0000000fffffffffull;
            uint64_t fp = arm_thread_state64_get_fp(state) & 0x0000000fffffffffull;
            for (unsigned f = 0; f < 16 && fp && !(fp & 7); ++f) {
                uint64_t pair[2] = {};
                vm_size_t got = 0;
                if (vm_read_overwrite(task, (vm_address_t)fp, sizeof pair, (vm_address_t)pair, &got) != KERN_SUCCESS || got != sizeof pair) break;
                const uint64_t ret = pair[1] & 0x0000000fffffffffull, next = pair[0] & 0x0000000fffffffffull;
                if (!ret) break;
                chain[depth++] = ret;
                if (next <= fp) break;
                fp = next;
            }
            for (unsigned c = 0; c < depth; ++c) {
                bool dup = false;
                for (unsigned d = 0; d < c; ++d) if (chain[d] == chain[c]) dup = true;
                if (dup) continue;
                bool hit = false;
                for (auto &e : callers) if (e.first == chain[c]) {++e.second; hit = true; break;}
                if (!hit && callers.size() < 8192) callers.push_back({chain[c], 1});
            }
        }
        usleep(2000);
    }
    if (samples) {
        std::sort(pcs.begin(), pcs.end(), [](const auto &a, const auto &b) { return a.second > b.second; });
        std::sort(callers.begin(), callers.end(), [](const auto &a, const auto &b) { return a.second > b.second; });
        // Rows of 10: CPUPC the leaf PCs (top 40), CPUPI the return addresses in the stacks (top 80); counts are samples, not percent.
        auto rows = [&](const char *tag, const std::vector<std::pair<uint64_t, unsigned>> &list, unsigned limit) {
            for (unsigned first = 0; first < list.size() && first < limit; first += 10) {
                int at = snprintf(line, sizeof line, "%s %s[%d] n%u r%u:", tag, process, pid, samples, first / 10);
                for (unsigned i = first; i < list.size() && i < first + 10 && at < (int)sizeof line - 60; ++i) {
                    const uint64_t pc = list[i].first;
                    char where[64];
                    if (cacheBase && pc >= cacheBase && pc < cacheBase + 0x200000000ull) {
                        snprintf(where, sizeof where, "dsc:%llx", (unsigned long long)(pc - slide));
                    } else {
                        const Image *best = nullptr;
                        for (const Image &image : images) if (image.base <= pc && (!best || image.base > best->base)) best = &image;
                        if (best) snprintf(where, sizeof where, "%s+%llx", best->name, (unsigned long long)(pc - best->base));
                        else snprintf(where, sizeof where, "%llx", (unsigned long long)pc);
                    }
                    at += snprintf(line + at, sizeof line - at, " %s %u", where, list[i].second);
                }
                snprintf(line + at, sizeof line - at, "\n");
                emit(line);
            }
        };
        rows("CPUPC", pcs, 40);
        rows("CPUPI", callers, 80);
        if (!sysctls.empty()) {
            std::sort(sysctls.begin(), sysctls.end(), [](const auto &a, const auto &b) { return a.second > b.second; });
            int at = snprintf(line, sizeof line, "CPUSC %s[%d] n%u:", process, pid, samples);
            for (unsigned i = 0; i < sysctls.size() && i < 8 && at < (int)sizeof line - 70; ++i)
                at += snprintf(line + at, sizeof line - at, " %s %u", sysctls[i].first.c_str(), sysctls[i].second);
            snprintf(line + at, sizeof line - at, "\n");
            emit(line);
        }
        int at = 0;
        std::vector<unsigned> order(threadCount);
        for (unsigned t = 0; t < threadCount; ++t) order[t] = t;
        std::sort(order.begin(), order.end(), [&](unsigned a, unsigned b) { return perThread[a] > perThread[b]; });
        at = snprintf(line, sizeof line, "CPUPT %s[%d] %u threads:", process, pid, threadCount);
        for (unsigned i = 0; i < threadCount && i < 10 && perThread[order[i]] && at < (int)sizeof line - 40; ++i)
            at += snprintf(line + at, sizeof line - at, " %s %u", names[order[i]].c_str(), perThread[order[i]]);
        snprintf(line + at, sizeof line - at, "\n");
        emit(line);
    }
    for (unsigned t = 0; t < threadCount; ++t) mach_port_deallocate(mach_task_self(), threads[t]);
    vm_deallocate(mach_task_self(), (vm_address_t)threads, threadCount * sizeof threads[0]);
    mach_port_deallocate(mach_task_self(), task);
}


// What a WebKit GPU process's sandbox lets it do with the Metal plugin (WebKit.GPU reads MetalPluginName but never runs the plugin,
// usb-174/175): sandbox_check against that process, once per pid, for the plugin's files (file-read-data) and the accelerator's user
// client (iokit-open-user-client, filter types 12..16 tried as the numbering is private).  SBCHECK <pid> <check>=<0 allowed|1 denied|-1>
extern "C" int sandbox_check(pid_t, const char *, int, ...);
extern "C" int proc_pidpath(int, void *, uint32_t);
// sa39: also the plugin's real path behind the bindfs mount (/private/var/ngpu/addons), and the images the process has loaded outside
// the shared cache (whether dyld ever mapped the plugin), at its first sight, 30 s and 2 min later.
void sandboxCheck(int pid) {
    static std::vector<std::pair<int, int>> seen;
    int *passes = nullptr;
    for (auto &d : seen) if (d.first == pid) passes = &d.second;
    if (!passes) {seen.push_back({pid, 0}); passes = &seen.back().second;}
    const int pass = (*passes)++;
    if (pass != 0 && pass != 6 && pass != 24) return;
    char path[1024] = {};
    if (proc_pidpath(pid, path, sizeof path) <= 0 || !strstr(path, "com.apple.WebKit.GPU")) return;
    constexpr int kNoReport = 0x40000000, kFilterPath = 1;
    const char *const files[] = {"/System/Library/Extensions/NativeAdrenoMetal.bundle/NativeAdrenoMetal",
                                 "/System/Library/Extensions/NativeAdrenoMetal.bundle/Info.plist",
                                 "/System/Library/Extensions/NativeAdrenoMetal.bundle/libvulkan_freedreno.dylib",
                                 "/System/Library/Extensions/NativeAdrenoMetal.publish", "/System/Library/Extensions/NativeAdrenoMetal.clients",
                                 "/System/Library/Extensions/AGXMetalA10.bundle/AGXMetalA10",
                                 "/private/var/ngpu/addons/NativeAdrenoMetal.bundle/NativeAdrenoMetal", nullptr};
    char line[480];
    int at = snprintf(line, sizeof line, "SBCHECK %d", pid);
    for (unsigned i = 0; files[i]; ++i) {
        const char *base = strrchr(files[i], '/') + 1;
        at += snprintf(line + at, sizeof line - at, " %s=%d", base, sandbox_check(pid, "file-read-data", kFilterPath | kNoReport, files[i]));
    }
    at += snprintf(line + at, sizeof line - at, " map=%d", sandbox_check(pid, "file-map-executable", kFilterPath | kNoReport, files[0]));
    at += snprintf(line + at, sizeof line - at, " maprealpath=%d",
                   sandbox_check(pid, "file-map-executable", kFilterPath | kNoReport, "/private/var/ngpu/addons/NativeAdrenoMetal.bundle/NativeAdrenoMetal"));
    for (int type = 12; type <= 16; ++type)
        at += snprintf(line + at, sizeof line - at, " io%d=%d", type, sandbox_check(pid, "iokit-open-user-client", type | kNoReport, "AGXDeviceUserClient"));
    snprintf(line + at, sizeof line - at, "\n");
    emit(line);
    // The images outside the shared cache.
    mach_port_t task = MACH_PORT_NULL;
    kern_return_t kr = task_for_pid(mach_task_self(), pid, &task);
    if (kr != KERN_SUCCESS && task_read_for_pid) kr = task_read_for_pid(mach_task_self(), pid, &task);
    at = snprintf(line, sizeof line, "SBIMAGES %d pass %d:", pid, pass);
    if (kr != KERN_SUCCESS) {snprintf(line + at, sizeof line - at, " no task port 0x%x\n", kr); emit(line); return;}
    task_dyld_info_data_t dyld = {};
    mach_msg_type_number_t count = TASK_DYLD_INFO_COUNT;
    unsigned total = 0;
    struct Img { uint64_t base; char name[40]; };
    std::vector<Img> imgs;
    uint64_t cacheBase = 0, slide = 0;
    if (task_info(task, TASK_DYLD_INFO, (task_info_t)&dyld, &count) == KERN_SUCCESS && dyld.all_image_info_addr) {
        struct dyld_all_image_infos all = {};
        vm_size_t got = 0;
        if (vm_read_overwrite(task, dyld.all_image_info_addr, sizeof all, (vm_address_t)&all, &got) == KERN_SUCCESS) {
            total = all.infoArrayCount;
            cacheBase = all.sharedCacheBaseAddress;
            slide = all.sharedCacheSlide;
            std::vector<struct dyld_image_info> infos(all.infoArrayCount < 2048 ? all.infoArrayCount : 2048);
            if (!infos.empty() && vm_read_overwrite(task, (vm_address_t)all.infoArray, infos.size() * sizeof infos[0],
                                                         (vm_address_t)infos.data(), &got) == KERN_SUCCESS) {
                for (const auto &info : infos) {
                    const uint64_t base = (uint64_t)info.imageLoadAddress;
                    if (all.sharedCacheBaseAddress && base >= all.sharedCacheBaseAddress && base < all.sharedCacheBaseAddress + 0x200000000ull) continue;
                    char path[256] = {};
                    vm_read_overwrite(task, (vm_address_t)info.imageFilePath, sizeof path - 1, (vm_address_t)path, &got);
                    const char *name = strrchr(path, '/') ? strrchr(path, '/') + 1 : path;
                    if (at < (int)sizeof line - 40) at += snprintf(line + at, sizeof line - at, " %.32s", name);
                    Img img = {base, {}};
                    strlcpy(img.name, name, sizeof img.name);
                    imgs.push_back(img);
                }
            }
        }
    }
    snprintf(line + at, sizeof line - at, " (%u images)\n", total);
    emit(line);
    // sa43: every thread's stack (frame pointers, 12 frames), symbolized as image+offset or dsc:<unslid> (ipsw dyld a2s): a thread
    // held inside the plugin's load (its initializers) would show here, as the plugin is mapped but never reached Metal's init call.
    auto where = [&](uint64_t pc, char *out, size_t size) {
        pc &= 0x0000000fffffffffull;
        if (cacheBase && pc >= cacheBase && pc < cacheBase + 0x200000000ull) {snprintf(out, size, "dsc:%llx", (unsigned long long)(pc - slide)); return;}
        const Img *best = nullptr;
        for (const Img &img : imgs) if (img.base <= pc && (!best || img.base > best->base)) best = &img;
        if (best) snprintf(out, size, "%.20s+%llx", best->name, (unsigned long long)(pc - best->base));
        else snprintf(out, size, "%llx", (unsigned long long)pc);
    };
    thread_act_array_t threads = nullptr;
    mach_msg_type_number_t threadCount = 0;
    if (pass != 0 && task_threads(task, &threads, &threadCount) == KERN_SUCCESS) {
        for (unsigned t = 0; t < threadCount; ++t) {
            arm_thread_state64_t state = {};
            mach_msg_type_number_t n = ARM_THREAD_STATE64_COUNT;
            if (thread_get_state(threads[t], ARM_THREAD_STATE64, (thread_state_t)&state, &n) == KERN_SUCCESS) {
                thread_extended_info_data_t ext = {};
                mach_msg_type_number_t en = THREAD_EXTENDED_INFO_COUNT;
                const bool named = thread_info(threads[t], THREAD_EXTENDED_INFO, (thread_info_t)&ext, &en) == KERN_SUCCESS && ext.pth_name[0];
                at = snprintf(line, sizeof line, "SBSTACK %d t%u %.24s:", pid, t, named ? ext.pth_name : "-");
                char w[64];
                where(arm_thread_state64_get_pc(state), w, sizeof w);
                at += snprintf(line + at, sizeof line - at, " %s", w);
                where(arm_thread_state64_get_lr(state), w, sizeof w);
                at += snprintf(line + at, sizeof line - at, " %s", w);
                uint64_t fp = arm_thread_state64_get_fp(state) & 0x0000000fffffffffull;
                for (unsigned f = 0; f < 12 && fp && at < (int)sizeof line - 40; ++f) {
                    uint64_t frame[2] = {};
                    vm_size_t got = 0;
                    if (vm_read_overwrite(task, (vm_address_t)fp, sizeof frame, (vm_address_t)frame, &got) != KERN_SUCCESS) break;
                    if (!frame[1]) break;
                    where(frame[1], w, sizeof w);
                    at += snprintf(line + at, sizeof line - at, " %s", w);
                    const uint64_t next = frame[0] & 0x0000000fffffffffull;
                    if (next <= fp) break;
                    fp = next;
                }
                snprintf(line + at, sizeof line - at, "\n");
                emit(line);
            }
            mach_port_deallocate(mach_task_self(), threads[t]);
        }
        vm_deallocate(mach_task_self(), (vm_address_t)threads, threadCount * sizeof threads[0]);
    }
    mach_port_deallocate(mach_task_self(), task);
}
void cpuTopPass(std::vector<Sample> &previous, std::vector<std::pair<uint64_t, uint64_t>> &threadsBefore, uint64_t intervalNs,
                bool always, int &failures) {
    int mib[4] = {CTL_KERN, KERN_PROC, KERN_PROC_ALL, 0};
    size_t size = 0;
    if (sysctl(mib, 3, nullptr, &size, nullptr, 0) != 0 || !size) return;
    size += size / 4;
    std::vector<char> buffer(size);
    if (sysctl(mib, 3, buffer.data(), &size, nullptr, 0) != 0) return;
    const struct kinfo_proc *procs = (const struct kinfo_proc *)buffer.data();
    const unsigned count = (unsigned)(size / sizeof *procs);
    static mach_timebase_info_data_t timebase;
    if (!timebase.denom) mach_timebase_info(&timebase);
    std::vector<Sample> now;
    for (unsigned i = 0; i < count; ++i) {
        TaskInfo info = {};
        const int pid = procs[i].kp_proc.p_pid;
        if (proc_pidinfo(pid, kPidTaskInfo, 0, &info, sizeof info) != (int)sizeof info) {
            if (pid > 1 && failures < 3 && ++failures == 3) {
                char line[120];
                snprintf(line, sizeof line, "CPUTOP proc_pidinfo(%d) refused: %s\n", pid, strerror(errno));
                emit(line);
            }
            continue;
        }
        if (!strncmp(procs[i].kp_proc.p_comm, "com.apple.WebKi", 15)) sandboxCheck(pid);
        Sample sample = {pid, {}, info.total_user + info.total_system, 0};
        strlcpy(sample.name, procs[i].kp_proc.p_comm, sizeof sample.name);
        for (const Sample &old : previous)
            if (old.pid == pid && sample.ticks >= old.ticks) sample.delta = sample.ticks - old.ticks;
        now.push_back(sample);
    }
    const bool first = previous.empty();
    previous = now;
    if (first) return;
    std::sort(now.begin(), now.end(), [](const Sample &a, const Sample &b) { return a.delta > b.delta; });
    auto percent = [&](uint64_t ticks) { return (unsigned)(ticks * timebase.numer / timebase.denom * 100 / (intervalNs ? intervalNs : 1)); };
    uint64_t total = 0;
    for (const Sample &s : now) total += s.delta;
    if (!always && percent(total) < 15) return;
    char line[400];
    int at = snprintf(line, sizeof line, "CPUTOP %u%%", percent(total));
    for (unsigned i = 0; i < now.size() && i < 8 && now[i].delta; ++i)
        at += snprintf(line + at, sizeof line - at, " | %s[%d] %u", now[i].name, now[i].pid, percent(now[i].delta));
    snprintf(line + at, sizeof line - at, "\n");
    emit(line);
    static uint64_t lastProfile;
    const uint64_t nowNs = clock_gettime_nsec_np(CLOCK_UPTIME_RAW);
    // Every 10 s one of the three busiest processes over 40 % of a core, in turn (frame-time work: 2026-11-03, TestUFO at 50 fps with
    // backboardd, the WebKit content and GPU processes each at 50-65 %).
    static unsigned profileTurn;
    if (nowNs - lastProfile > 10000000000ull) {
        unsigned busy = 0;
        while (busy < now.size() && busy < 3 && now[busy].pid > 0 && percent(now[busy].delta) >= 40) ++busy;
        if (busy) {
            const Sample &pick = now[profileTurn++ % busy];
            if (pick.pid != getpid()) {lastProfile = nowNs; profile(pick.pid, pick.name);}
        }
    }
    // CPUTHR <process>[pid] <total>: <thread> <percent of a core>/k<of which in the kernel> ...
    static std::vector<std::pair<uint64_t, uint64_t>> sysBefore;
    std::vector<std::pair<uint64_t, uint64_t>> threadsNow, sysNow;
    for (unsigned i = 0; i < now.size() && i < 3 && percent(now[i].delta) >= 10; ++i) {
        uint64_t handles[256];
        const int bytes = proc_pidinfo(now[i].pid, kPidListThreads, 0, handles, sizeof handles);
        if (bytes <= 0) continue;
        struct Thread { uint64_t delta, sys; char name[24]; };
        std::vector<Thread> threads;
        uint64_t sum = 0;
        for (int t = 0; t < bytes / (int)sizeof(uint64_t); ++t) {
            ThreadInfo info = {};
            if (proc_pidinfo(now[i].pid, kPidThreadInfo, handles[t], &info, sizeof info) != (int)sizeof info) continue;
            const uint64_t time = info.user_time + info.system_time;
            threadsNow.push_back({handles[t], time});
            sysNow.push_back({handles[t], info.system_time});
            uint64_t delta = 0, sys = 0;
            for (const auto &old : threadsBefore)
                if (old.first == handles[t] && time >= old.second) delta = time - old.second;
            for (const auto &old : sysBefore)
                if (old.first == handles[t] && info.system_time >= old.second) sys = info.system_time - old.second;
            Thread thread = {delta, sys, {}};
            if (info.name[0]) strlcpy(thread.name, info.name, sizeof thread.name);
            else snprintf(thread.name, sizeof thread.name, "t%llx", (unsigned long long)(handles[t] & 0xfffff));
            for (char &c : thread.name) if (c == ' ') c = '_';
            threads.push_back(thread);
            sum += delta;
        }
        if (!sum) continue;
        std::sort(threads.begin(), threads.end(), [](const Thread &a, const Thread &b) { return a.delta > b.delta; });
        const unsigned whole = percent(now[i].delta);
        at = snprintf(line, sizeof line, "CPUTHR %s[%d] %u:", now[i].name, now[i].pid, whole);
        for (unsigned t = 0; t < threads.size() && t < 6 && threads[t].delta; ++t)
            at += snprintf(line + at, sizeof line - at, " %s %u/k%u", threads[t].name, (unsigned)(threads[t].delta * whole / sum),
                           (unsigned)(threads[t].sys * whole / sum));
        snprintf(line + at, sizeof line - at, "\n");
        emit(line);
    }
    threadsBefore = threadsNow;
    sysBefore = sysNow;
}


void *cpuTop(void *) {
    std::vector<Sample> previous;
    std::vector<std::pair<uint64_t, uint64_t>> threads;
    int failures = 0;
    uint64_t last = clock_gettime_nsec_np(CLOCK_UPTIME_RAW);
    for (unsigned pass = 0;; ++pass) {
        sleep(5);
        const uint64_t when = clock_gettime_nsec_np(CLOCK_UPTIME_RAW);
        @autoreleasepool { cpuTopPass(previous, threads, when - last, pass % 12 == 0, failures); }
        last = when;
    }
    return nullptr;
}
}  // namespace

int main() {
    @autoreleasepool {
        Filter filter = load();
        char line[400];
        snprintf(line, sizeof line, "OSLOG forwarder started (pid %d): %zu names, %zu prefixes%s, level %ld, rate %ld/s\n", getpid(),
                 filter.exact.size(), filter.prefix.size(), filter.all ? ", all processes" : "", filter.level, filter.rate);
        emit(line);
        gFilter = filter;
        pthread_t sampler;
        if (pthread_create(&sampler, nullptr, cpuTop, nullptr) == 0) pthread_detach(sampler);
        launchServicesDatabase();
        installStagedSystemApps();
        if (startLiveStream()) {
            for (;;) {
                sleep(10);
                if (gDropped) {snprintf(line, sizeof line, "OSLOG dropped %lu lines over the rate limit so far\n", gDropped); emit(line);}
            }
        }
        // OSLogStore lives in OSLog.framework (linked); load it by path as well and say what dyld answers if the class is still missing.
        void *oslog = dlopen("/System/Library/Frameworks/OSLog.framework/OSLog", RTLD_NOW);
        Class storeClass = NSClassFromString(@"OSLogStore") ?: objc_getClass("OSLogStore");
        snprintf(line, sizeof line, "OSLOG OSLog.framework %s%s, OSLogStore %s\n", oslog ? "loaded" : "not loaded: ", oslog ? "" : (dlerror() ?: "?"),
                 storeClass ? "found" : "missing");
        emit(line);
        NSDate *since = [NSDate dateWithTimeIntervalSinceNow:-120];   // the boot's last two minutes before this started
        unsigned long dropped = 0;
        for (unsigned pass = 0;; ++pass) {
            @autoreleasepool {
                NSError *error = nil;
                // iOS: +localStoreAndReturnError: (every process's entries with com.apple.logging.local-store); then the system scope (0),
                // which storeWithScope:error: (not in the iOS SDK headers) answered with nil on pdx203.
                id store = nil;
                const char *how = "none";
                if (storeClass && [storeClass respondsToSelector:@selector(localStoreAndReturnError:)]) {
                    store = ((id (*)(Class, SEL, NSError **))objc_msgSend)(storeClass, @selector(localStoreAndReturnError:), &error);
                    how = "localStoreAndReturnError:";
                }
                if (!store && storeClass && [storeClass respondsToSelector:@selector(storeWithScope:error:)]) {
                    NSError *second = nil;
                    store = ((id (*)(Class, SEL, NSInteger, NSError **))objc_msgSend)(storeClass, @selector(storeWithScope:error:), 0, &second);
                    if (!store && !error) error = second;
                    how = "storeWithScope:error:";
                }
                if (store && pass == 0) {snprintf(line, sizeof line, "OSLOG store from %s\n", how); emit(line);}
                if (!store) {
                    if (pass % 30 == 0) {
                        snprintf(line, sizeof line, "OSLOG no store (%s): %s\n", how, (error ? error.description : @"nil, no error").UTF8String);
                        emit(line);
                    }
                    sleep(1);
                    continue;
                }
                id position = ((id (*)(id, SEL, NSDate *))objc_msgSend)(store, @selector(positionWithDate:), since);
                id entries = ((id (*)(id, SEL, NSUInteger, id, id, NSError **))objc_msgSend)(
                    store, @selector(entriesEnumeratorWithOptions:position:predicate:error:), 0, position, nil, &error);
                NSDate *newest = since;
                long sent = 0;
                for (id entry in entries) {
                    @autoreleasepool {
                        NSDate *date = [entry valueForKey:@"date"];
                        if ([date compare:since] != NSOrderedDescending) continue;   // seen in an earlier pass
                        if ([date compare:newest] == NSOrderedDescending) newest = date;
                        if (![entry respondsToSelector:@selector(process)]) continue;
                        NSString *process = [entry valueForKey:@"process"];
                        const char *name = process.UTF8String ?: "?";
                        if (!wanted(filter, name)) continue;
                        const long level = [entry respondsToSelector:@selector(level)] ? [[entry valueForKey:@"level"] integerValue] : 3;
                        if (level && level < filter.level) continue;
                        if (sent >= filter.rate) {++dropped; continue;}
                        ++sent;
                        NSString *message = [entry valueForKey:@"composedMessage"];
                        NSString *subsystem = [entry respondsToSelector:@selector(subsystem)] ? [entry valueForKey:@"subsystem"] : @"";
                        NSString *category = [entry respondsToSelector:@selector(category)] ? [entry valueForKey:@"category"] : @"";
                        const int pid = [entry respondsToSelector:@selector(processIdentifier)] ? [[entry valueForKey:@"processIdentifier"] intValue] : 0;
                        std::string text = message.UTF8String ?: "";
                        for (char &c : text) if (c == '\n' || c == '\r' || c == '\t') c = ' ';
                        if (text.size() > 300) text.resize(300);
                        snprintf(line, sizeof line, "OSLOG %s[%d] %c %s:%s %s\n", name, pid, levelLetter(level), subsystem.UTF8String ?: "",
                                 category.UTF8String ?: "", text.c_str());
                        emit(line);
                    }
                }
                since = newest;
                if (dropped && pass % 10 == 0) {
                    snprintf(line, sizeof line, "OSLOG dropped %lu lines over the rate limit so far\n", dropped);
                    emit(line);
                }
            }
            sleep(1);
        }
    }
}
