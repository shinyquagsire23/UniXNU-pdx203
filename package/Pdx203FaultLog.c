/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/* The crash detail the exit log (the fork's zzzzzzzzz-exit-log.patch) appends for a process that dies of a signal: the exiting
 * thread is the one that took the signal, and its saved user state still holds the fault. PC, LR, fault address, ESR, the task's
 * shared-region slide (so a PC in the dyld cache can be resolved offline against the cryptex's cache) and a frame-pointer
 * backtrace of up to 12 return addresses read with copyin. After UniXNU-alioth's BootFaultProbe.c (boot_progress_exit_fault),
 * without its trap hooks: those need the boot-progress layer. Built with the osfmk flags (scripts/build-pdx203-stubs.py).
 *
 * An abort (the exiting thread entered the kernel by SVC: abort() -> pthread_kill) carries no reason, but a process that dies of an
 * uncaught exception holds the exception text in its malloc heap ("*** Terminating app due to uncaught exception 'X', reason: 'Y'"),
 * and the Metal plugin's handler (drivers/NativeMetal/ExceptionDiag.h) leaves "SHENGTHROW <name> | <reason> | <return addresses>"
 * there in every process that loaded it. Both are found by alioth's reason scan (BootFaultProbe.c boot_progress_exit_reason) over
 * the writable malloc regions of the exiting process and printed as EXITWHY and EXITTHROW rows right before its EXITLOG row
 * (MobileSafari and PosterBoard aborted with nothing else to go on, pdx203 usb-51..55). CoreFoundation's own report of the exception
 * keeps the throw's return addresses as "(0x... 0x... ...)" after "*** First throw call stack:"; that run is printed as EXITSTACK (its
 * addresses resolve with the EXITLOG row's slide). */
#include <kern/task.h>
#include <kern/thread.h>
#include <arm/thread.h>
#include <kern/misc_protos.h>
#include <vm/vm_map.h>
#include <mach/vm_region.h>
#include <string.h>

extern int proc_selfpid(void);
extern void proc_selfname(char *, int);

#define REASON_CHUNK 65536
static char reason_chunk[REASON_CHUNK + 1];
static char reason_text[300], throw_text[330], stack_text[400];
static volatile int reason_busy;

static void
pdx203_exit_reason(void)
{
	if (__atomic_exchange_n(&reason_busy, 1, __ATOMIC_ACQUIRE)) {
		return;
	}
	reason_text[0] = throw_text[0] = stack_text[0] = 0;
	vm_map_t map = current_map();
	vm_map_offset_t addr = 0;
	uint64_t scanned = 0;
	int best = 0, thrown = 0, stacked = 0;
	while (scanned < (160ull << 20) && (best < 2 || !stacked)) {
		vm_map_size_t size = 0;
		natural_t depth = 0;
		vm_region_submap_info_data_64_t info;
		mach_msg_type_number_t count = VM_REGION_SUBMAP_INFO_V2_COUNT_64;
		if (vm_map_region_recurse_64(map, &addr, &size, &depth, (vm_region_submap_info_64_t)&info, &count) != KERN_SUCCESS) {
			break;
		}
		vm_map_offset_t base = addr;
		addr += size;
		/* malloc's regions only (VM_MEMORY_MALLOC .. VM_MEMORY_MALLOC_NANO user tags) */
		if (info.is_submap || (info.protection & (VM_PROT_READ | VM_PROT_WRITE)) != (VM_PROT_READ | VM_PROT_WRITE) ||
		    info.user_tag < 1 || info.user_tag > 11) {
			continue;
		}
		for (vm_map_offset_t off = 0; off < size && (best < 2 || !stacked); off += REASON_CHUNK) {
			unsigned long len = (size - off) < REASON_CHUNK ? (unsigned long)(size - off) : REASON_CHUNK;
			if (copyin((user_addr_t)(base + off), reason_chunk, len)) {
				continue;
			}
			scanned += len;
			unsigned long start = 0;
			for (unsigned long i = 0; i <= len; ++i) {
				int ok = i < len && (unsigned char)reason_chunk[i] >= 0x20 && (unsigned char)reason_chunk[i] < 0x7f;
				if (ok) {
					continue;
				}
				unsigned long run = i - start;
				if (run >= 24) {
					char saved = reason_chunk[i];
					reason_chunk[i] = 0;
					if (!stacked && run >= 40 && !strncmp(reason_chunk + start, "(0x", 3)) {
						unsigned long m = run < sizeof(stack_text) - 1 ? run : sizeof(stack_text) - 1;
						memcpy(stack_text, reason_chunk + start, m);
						stack_text[m] = 0;
						stacked = 1;
					}
					if (!thrown && run >= 32 && !strncmp(reason_chunk + start, "SHENGTHROW ", 11)) {
						unsigned long m = run < sizeof(throw_text) - 1 ? run : sizeof(throw_text) - 1;
						memcpy(throw_text, reason_chunk + start, m);
						throw_text[m] = 0;
						thrown = 1;
					}
					/* rank 2: the whole uncaught-exception line; rank 1: any run with a reason clause */
					int rank = strnstr(reason_chunk + start, "uncaught exception", run) ? 2 :
					    strnstr(reason_chunk + start, "reason: ", run) ? 1 : 0;
					if (rank > best) {
						unsigned long m = run < sizeof(reason_text) - 1 ? run : sizeof(reason_text) - 1;
						memcpy(reason_text, reason_chunk + start, m);
						reason_text[m] = 0;
						best = rank;
					}
					reason_chunk[i] = saved;
					if (best >= 2 && stacked) {
						break;
					}
				}
				start = i + 1;
			}
		}
	}
	char name[32];
	proc_selfname(name, sizeof(name));
	if (reason_text[0]) {
		printf("EXITWHY %s[%d] %.299s\n", name, proc_selfpid(), reason_text);
	}
	if (stack_text[0]) {
		printf("EXITSTACK %s[%d] %.399s\n", name, proc_selfpid(), stack_text);
	}
	if (throw_text[0]) {
		printf("EXITTHROW %s[%d] %.329s\n", name, proc_selfpid(), throw_text);
	}
	__atomic_store_n(&reason_busy, 0, __ATOMIC_RELEASE);
}

/* Where a code address lies (pdx203 usb-137/138: backboardd faulted at addresses outside the dyld cache, likely in the Metal plugin
 * bundle): the start and size of the mapped region holding it and its offset there. A dylib's __TEXT is one region, so its start is the
 * image's load address and the offset matches the binary's own addresses (+ its __TEXT vmaddr). Printed as one EXITMAP row. */
static void
pdx203_exit_map(const uint64_t *addrs, unsigned count)
{
	char row[600];
	int at = snprintf(row, sizeof(row), "EXITMAP");
	vm_map_t map = current_map();
	for (unsigned i = 0; i < count && at > 0 && (unsigned)at + 64 < sizeof(row); ++i) {
		vm_map_offset_t addr = addrs[i];
		vm_map_size_t size = 0;
		natural_t depth = 0;
		vm_region_submap_info_data_64_t info;
		mach_msg_type_number_t infoCount = VM_REGION_SUBMAP_INFO_V2_COUNT_64;
		if (!addrs[i] || vm_map_region_recurse_64(map, &addr, &size, &depth, (vm_region_submap_info_64_t)&info, &infoCount) != KERN_SUCCESS ||
		    addrs[i] < addr || addrs[i] >= addr + size) {
			at += snprintf(row + at, sizeof(row) - at, " %llx:-", addrs[i]);
			continue;
		}
		at += snprintf(row + at, sizeof(row) - at, " %llx:%llx+%llx/%llx%s", addrs[i], (unsigned long long)addr,
		    (unsigned long long)(addrs[i] - addr), (unsigned long long)size, (info.protection & VM_PROT_EXECUTE) ? "x" : "");
	}
	printf("%s\n", row);
}

unsigned long
pdx203_exit_fault(char *out, unsigned long capacity)
{
	if (!out || capacity < 64) {
		return 0;
	}
	out[0] = 0;
	arm_saved_state_t *state = find_user_regs(current_thread());
	if (!state || !is_saved_state64(state)) {
		return 0;
	}
	uint64_t pc = (uint64_t)get_saved_state_pc(state), lr = (uint64_t)get_saved_state_lr(state);
	uint64_t fp = (uint64_t)get_saved_state_fp(state);
	uint64_t far = (uint64_t)get_saved_state_far(state);
	uint32_t esr = (uint32_t)get_saved_state_esr(state);
	uint64_t slide = current_task()->task_shared_region_slide;
	if (((esr >> 26) & 0x3f) == 0x15) {   /* SVC: a signal the thread raised itself (abort), not a fault */
		pdx203_exit_reason();
	}
	int n = snprintf(out, capacity, " pc 0x%llx lr 0x%llx far 0x%llx esr 0x%x slide 0x%llx bt", pc, lr, far, esr, slide);
	if (n < 0) {
		out[0] = 0;
		return 0;
	}
	unsigned long used = (unsigned long)n < capacity ? (unsigned long)n : capacity - 1;
	uint64_t frames[14] = {pc, lr};
	unsigned frameCount = 2;
	for (unsigned depth = 0; depth < 12 && fp && !(fp & 15) && used + 20 < capacity; ++depth) {
		uint64_t pair[2];
		if (copyin((user_addr_t)fp, pair, sizeof(pair))) {
			break;
		}
		if (frameCount < 14) {
			frames[frameCount++] = pair[1];
		}
		n = snprintf(out + used, capacity - used, " 0x%llx", pair[1]);
		if (n <= 0) {
			break;
		}
		used += (unsigned long)n < capacity - used ? (unsigned long)n : capacity - used - 1;
		if (pair[0] <= fp || pair[0] - fp > 1048576) {
			break;
		}
		fp = pair[0];
	}
	if (((esr >> 26) & 0x3f) != 0x15) {   /* a fault: name the regions of its code addresses */
		pdx203_exit_map(frames, frameCount);
	}
	return used;
}

/* The render server's freezes (pdx203 2026-11-03, usb-149..153): backboardd stops presenting with the GPU idle and every swap retired, and
 * something sends it SIGTERM a while later; it respawns and the screen comes back. Nothing in the log says what it waited on. When
 * backboardd or SpringBoard exits on a signal, every one of its threads is still where the freeze left it (the exit runs on the thread
 * that took the signal; the others are held): each thread's name, user PC and LR and a frame-pointer backtrace of up to 10 return
 * addresses, as FREEZETHR rows after a FREEZETRACE row with the slide, then one EXITMAP row of the distinct PCs (the regions name the
 * Metal plugin, Turnip or the dyld cache). Called from proc_prepareexit (the fork's zzzzzzzzzf-freeze-trace.patch) with the exiting
 * process current. */
#define FREEZE_THREADS 64
static struct {uint64_t pc, lr, fp, tid; char name[32];} freeze_threads[FREEZE_THREADS];
static volatile int freeze_busy;
extern void bsd_getthreadname(void *uth, char *buffer);

void
pdx203_exit_threads(const char *name, int pid, int rv, int current)
{
	const int sig = rv & 0x7f;
	if (!current || !name || sig == 0 || sig == 0x7f) {   /* not a signal exit, or not on the process's own thread */
		return;
	}
	if (strncmp(name, "backboardd", sizeof("backboardd")) != 0 && strncmp(name, "SpringBoard", sizeof("SpringBoard")) != 0) {
		return;
	}
	if (__atomic_exchange_n(&freeze_busy, 1, __ATOMIC_ACQUIRE)) {
		return;
	}
	task_t task = current_task();
	unsigned count = 0, total = 0;
	thread_t thread;
	char thread_name[64];
	task_lock(task);
	queue_iterate(&task->threads, thread, thread_t, task_threads) {
		++total;
		if (count == FREEZE_THREADS) {
			continue;
		}
		arm_saved_state_t *state = get_user_regs(thread);
		if (!state || !is_saved_state64(state)) {
			continue;
		}
		freeze_threads[count].pc = (uint64_t)get_saved_state_pc(state);
		freeze_threads[count].lr = (uint64_t)get_saved_state_lr(state);
		freeze_threads[count].fp = (uint64_t)get_saved_state_fp(state);
		freeze_threads[count].tid = thread_tid(thread);
		thread_name[0] = 0;
		void *uth = get_bsdthread_info(thread);
		if (uth) {
			bsd_getthreadname(uth, thread_name);
		}
		strlcpy(freeze_threads[count].name, thread_name, sizeof(freeze_threads[count].name));
		++count;
	}
	task_unlock(task);
	printf("FREEZETRACE %s[%d] sig %d threads %u slide 0x%llx\n", name, pid, sig, total, (unsigned long long)task->task_shared_region_slide);
	uint64_t pcs[24];
	unsigned pcCount = 0;
	for (unsigned i = 0; i < count; ++i) {
		char row[400];
		int at = snprintf(row, sizeof(row), "FREEZETHR %s[%d] %u %llu \"%.31s\" pc 0x%llx lr 0x%llx bt", name, pid, i,
		    (unsigned long long)freeze_threads[i].tid, freeze_threads[i].name, freeze_threads[i].pc, freeze_threads[i].lr);
		uint64_t fp = freeze_threads[i].fp;
		for (unsigned depth = 0; depth < 10 && fp && !(fp & 15) && at > 0 && (unsigned)at + 24 < sizeof(row); ++depth) {
			uint64_t pair[2];
			if (copyin((user_addr_t)fp, pair, sizeof(pair))) {
				break;
			}
			at += snprintf(row + at, sizeof(row) - at, " 0x%llx", pair[1]);
			if (pair[0] <= fp || pair[0] - fp > 1048576) {
				break;
			}
			fp = pair[0];
		}
		printf("%s\n", row);
		bool seen = false;
		for (unsigned k = 0; k < pcCount; ++k) {
			seen = seen || pcs[k] == freeze_threads[i].pc;
		}
		if (!seen && pcCount < 24) {
			pcs[pcCount++] = freeze_threads[i].pc;
		}
	}
	pdx203_exit_map(pcs, pcCount < 10 ? pcCount : 10);
	if (pcCount > 10) {
		pdx203_exit_map(pcs + 10, pcCount - 10 < 10 ? pcCount - 10 : 10);
	}
	__atomic_store_n(&freeze_busy, 0, __ATOMIC_RELEASE);
}
