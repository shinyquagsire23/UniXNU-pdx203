/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/* The crash detail the exit log (the fork's zzzzzzzzz-exit-log.patch) appends for a process that dies of a signal: the exiting
 * thread is the one that took the signal, and its saved user state still holds the fault. PC, LR, fault address, ESR, the task's
 * shared-region slide (so a PC in the dyld cache can be resolved offline against the cryptex's cache) and a frame-pointer
 * backtrace of up to 12 return addresses read with copyin. After UniXNU-alioth's BootFaultProbe.c (boot_progress_exit_fault),
 * without its trap hooks: those need the boot-progress layer. Built with the osfmk flags (scripts/build-pdx203-stubs.py). */
#include <kern/task.h>
#include <kern/thread.h>
#include <arm/thread.h>
#include <kern/misc_protos.h>

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
	int n = snprintf(out, capacity, " pc 0x%llx lr 0x%llx far 0x%llx esr 0x%x slide 0x%llx bt", pc, lr, far, esr, slide);
	if (n < 0) {
		out[0] = 0;
		return 0;
	}
	unsigned long used = (unsigned long)n < capacity ? (unsigned long)n : capacity - 1;
	for (unsigned depth = 0; depth < 12 && fp && !(fp & 15) && used + 20 < capacity; ++depth) {
		uint64_t pair[2];
		if (copyin((user_addr_t)fp, pair, sizeof(pair))) {
			break;
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
	return used;
}
