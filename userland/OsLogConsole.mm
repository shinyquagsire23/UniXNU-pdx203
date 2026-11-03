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
    constexpr const char *kMarker = "/private/var/ngpu/oslog/systemapps", *kDone = "/private/var/ngpu/oslog/systemapps.done";
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
            if (!migrator) return;
            if (upgrade && [migrator respondsToSelector:@selector(setDidUpgrade:)])
                ((void (*)(id, SEL, BOOL))objc_msgSend)(migrator, @selector(setDidUpgrade:), YES);
            const BOOL ok = ((BOOL (*)(id, SEL))objc_msgSend)(migrator, @selector(performMigration));
            snprintf(line, sizeof line, "OSLOG system apps: SystemAppMigrator performMigration %s\n", ok ? "succeeded" : "failed");
            emit(line);
            if (ok) {int fd = open(kDone, O_CREAT | O_WRONLY, 0644); if (fd >= 0) close(fd);}
        }
    });
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
