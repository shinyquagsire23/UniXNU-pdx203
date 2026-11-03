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

int main() {
    @autoreleasepool {
        Filter filter = load();
        char line[400];
        snprintf(line, sizeof line, "OSLOG forwarder started (pid %d): %zu names, %zu prefixes%s, level %ld, rate %ld/s\n", getpid(),
                 filter.exact.size(), filter.prefix.size(), filter.all ? ", all processes" : "", filter.level, filter.rate);
        emit(line);
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
