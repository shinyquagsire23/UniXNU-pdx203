// SPDX-License-Identifier: BSD-2-Clause-Patent
// pdx203 copy of UniXNU-sheng package/LegacyDisplayProbe.cpp at 978e3f7 (2026-10-22, the revision that matches the
// unixnu fork's drivers/NativeMobileDisplay), the sheng board probes of the late probe removed.
// Bounded offline original lifecycle instrument; no desktop claim.
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOUserClient.h>
#include <libkern/c++/OSMetaClass.h>
#include <libkern/c++/OSSerialize.h>
#include <libkern/c++/OSData.h>
#include <kern/task.h>
#include <sys/proc.h>
#include <pexpert/pexpert.h>
#include <mach/kmod.h>
#include <machine/machine_routines.h>
extern "C" kmod_info_t g_kernel_kmod_info;
extern "C" void native_legacy_display_static_init(void) __asm__("__GLOBAL__sub_I_NativeLegacyDisplay.cpp");
extern "C" IOService *native_legacy_display_allocate(void);
extern "C" void native_gop_presenter_static_init(void) __asm__("__GLOBAL__sub_I_NativeGOPPresenter.cpp");
extern "C" IOService *native_gop_presenter_create(void);
static IOService *pendingLegacyDisplay;
static IOService *activeGOPPresenter,*activeGOPNub;
// GPU hand-off presenter (sheng-fb-gpu=1, phase 4): lives beside the display driver, which offers it frames instead of copying them.
static IOService *gpuHandoffPresenter;
extern "C" bool native_gop_gpu_offer(IOService *,void *display,IOMemoryDescriptor *,uint64_t stride,uint32_t width,uint32_t height);
extern "C" void native_gop_presenter_set_source(IOService *presenter,IOService *nub);
volatile uint32_t native_legacy_gpu_fallbacks,native_legacy_gpu_last_status;
static IOService *activeLegacyDisplay;
// Keep one diagnostic mapping alive until reboot, even after backend stop.
// The pulse is permanently closed before any system-frame copy.
static IOMemoryMap *diagnosticAnchorMapping;
extern "C" int boot_progress_pulse_anchor(uintptr_t,uint64_t,unsigned,unsigned,unsigned) __attribute__((weak_import));
static IORegistryEntry *unitScaleChosen;
static bool unitScaleAdded,unitRotationAdded;
static void finishUnitScalePolicy(bool keep) {
    if(unitScaleChosen && !keep) {
        if(unitScaleAdded)unitScaleChosen->removeProperty("display-scale");
        if(unitRotationAdded)unitScaleChosen->removeProperty("display-rotation");
    }
    OSSafeReleaseNULL(unitScaleChosen);unitScaleAdded=false;unitRotationAdded=false;
}
static bool publishOrMatchWord(IORegistryEntry *entry,const char *key,uint32_t value,bool &added) {
    OSObject *previous=entry->getProperty(key);
    if(previous) {
        OSData *data=OSDynamicCast(OSData,previous);uint32_t word=0;
        if(!data || data->getLength()!=sizeof(word))return false;
        memcpy(&word,data->getBytesNoCopy(),sizeof(word));return word==value;
    }
    OSData *data=OSData::withBytes(&value,sizeof(value));
    bool result=data && entry->setProperty(key,data);OSSafeReleaseNULL(data);added=result;return result;
}
extern "C" {
volatile uint32_t native_legacy_stage;
volatile int native_legacy_meta_status=-1;
volatile uint32_t native_legacy_init_result,native_legacy_start_result,native_legacy_probe_match;
volatile uint32_t native_legacy_parent_match,native_legacy_object_size,native_legacy_released;
volatile uint64_t native_legacy_object,native_legacy_nub,native_legacy_workloop;
volatile uint32_t native_legacy_guard_slot;
volatile uint64_t native_legacy_guard_object,native_legacy_guard_caller;
volatile int native_legacy_surface_status=-1;
volatile uint32_t native_legacy_surface_found;
volatile uint64_t native_legacy_video_base,native_legacy_video_bytes;
volatile uint64_t native_legacy_gop_map,native_legacy_gop_virtual,native_legacy_gop_physical,native_legacy_gop_bytes;
volatile uint32_t native_legacy_gop_width,native_legacy_gop_height;
volatile uint32_t native_legacy_registration_calls,native_legacy_properties_length;
volatile uint32_t native_legacy_dt_attached;
volatile int native_legacy_unit_scale_status=-1;
volatile uint32_t native_legacy_unit_scale_calls,native_legacy_chosen_scale,native_legacy_chosen_rotation;
volatile uint32_t native_legacy_pm_stage,native_legacy_pm_calls,native_legacy_pm_callbacks,native_legacy_pm_ordinal;
volatile int native_legacy_pm_status=-1;
volatile uint32_t native_legacy_swap_validation_calls,native_legacy_swap_validation_result,native_legacy_swap_validation_reason;
volatile uint64_t native_legacy_swap_request,native_legacy_swap_surface;
volatile uint32_t native_legacy_transfer_calls;
volatile int native_legacy_transfer_status=-1;
volatile uint64_t native_legacy_transfer_bytes,native_legacy_transfer_request;
volatile uint32_t native_legacy_retirement_calls,native_legacy_retired_generation,native_legacy_presented_generation,native_legacy_retired_frames;
volatile uint64_t native_legacy_retired_current,native_legacy_retired_pending;
char native_legacy_properties[4096];
volatile uint32_t native_legacy_client_calls,native_legacy_client_type;
volatile int native_legacy_client_pid,native_legacy_client_status=-1;
volatile uint64_t native_legacy_client_task,native_legacy_client_object;
char native_legacy_client_class[64];
extern kern_return_t OSKextLoadKextWithIdentifier(const char *);
extern int boot_progress_prepare_scanout(void) __attribute__((weak_import));
extern void boot_progress_cancel_scanout(void) __attribute__((weak_import));
extern void boot_progress_stage(const char *) __attribute__((weak_import));
volatile uint32_t native_legacy_prepare_calls,native_legacy_prepare_reason,native_legacy_prepare_attempts;
volatile uint32_t native_legacy_prepare_failures,native_legacy_prepare_retried,native_legacy_prepare_max_attempts;
volatile uint64_t native_legacy_prepare_wait_us;
static uint64_t probeNowUs(void) {uint64_t ns;absolutetime_to_nanoseconds(mach_absolute_time(),&ns);return ns/1000;}
bool native_legacy_prepare_scanout(void) {
    native_legacy_prepare_calls=native_legacy_prepare_calls+1;native_legacy_prepare_reason=0;native_legacy_prepare_attempts=0;
    if (!boot_progress_prepare_scanout) return true;
    if (ml_at_interrupt_context()) {native_legacy_prepare_reason=1;return false;}
    if (!ml_get_interrupts_enabled()) {native_legacy_prepare_reason=2;return false;}
    // Yield to any in-flight diagnostic writer; never spin on a preempted
    // holder or claim a frame that the console can still overwrite.
    const uint64_t began=probeNowUs();
    for (unsigned attempt=0;attempt<100;++attempt) {
        native_legacy_prepare_attempts=attempt+1;
        if (boot_progress_prepare_scanout()) {
            if(attempt) {
                native_legacy_prepare_retried=native_legacy_prepare_retried+1;native_legacy_prepare_wait_us=native_legacy_prepare_wait_us+(probeNowUs()-began);
                if(attempt+1>native_legacy_prepare_max_attempts)native_legacy_prepare_max_attempts=attempt+1;
            }
            return true;
        }
        IOSleep(1);
    }
    native_legacy_prepare_failures=native_legacy_prepare_failures+1;native_legacy_prepare_wait_us=native_legacy_prepare_wait_us+(probeNowUs()-began);
    native_legacy_prepare_reason=3;
    if(boot_progress_cancel_scanout)boot_progress_cancel_scanout();
    return false;
}
__attribute__((noinline)) void native_legacy_display_checkpoint(void) {__asm__ volatile("" ::: "memory");}
__attribute__((noinline)) void native_legacy_gop_mapping_checkpoint(void) {__asm__ volatile("" ::: "memory");}
__attribute__((noinline)) void native_legacy_client_checkpoint(void) {__asm__ volatile("" ::: "memory");}
void native_legacy_pm_observed(unsigned stage,IOReturn status,unsigned long ordinal) {
    native_legacy_pm_stage=stage;native_legacy_pm_status=status;native_legacy_pm_ordinal=(uint32_t)ordinal;
    native_legacy_pm_calls=native_legacy_pm_calls+1;
    if(stage==3)native_legacy_pm_callbacks=native_legacy_pm_callbacks+1;
}
__attribute__((noinline)) void native_legacy_swap_validation_checkpoint(bool,unsigned,const void *,const void *) {__asm__ volatile("" ::: "memory");}
// Last rejected swap request, copied from the original request layout the validator reads (offsets
// qualified in NativeLegacyDisplay::validateSwap). Observation only; nothing here changes the verdict.
volatile uint32_t native_legacy_swap_accepted_count,native_legacy_swap_rejected_count,native_legacy_rej_reason;
volatile uint64_t native_legacy_rej_surfaces[4];
volatile uint32_t native_legacy_rej_state[3],native_legacy_rej_options[3],native_legacy_rej_size[2],native_legacy_rej_rects[2][4];
char native_legacy_rej_class[32];
volatile uint64_t native_legacy_rej_surface[7];   // width, height, format, planes, stride, allocation, descriptor bytes of the last rejected surface
void native_legacy_surface_rejected(uint64_t width,uint64_t height,uint32_t format,uint32_t planes,uint64_t stride,uint64_t allocation,uint64_t bytes) {
    native_legacy_rej_surface[0]=width;native_legacy_rej_surface[1]=height;native_legacy_rej_surface[2]=format;native_legacy_rej_surface[3]=planes;
    native_legacy_rej_surface[4]=stride;native_legacy_rej_surface[5]=allocation;native_legacy_rej_surface[6]=bytes;
}
static void recordRejectedSwap(unsigned reason,const void *request,const void *surface) {
    const unsigned char *bytes=reinterpret_cast<const unsigned char *>(request);
    if(!bytes)return;
    native_legacy_rej_reason=reason;
    uint64_t surfaces[4]={};memcpy(surfaces,bytes+2624,sizeof(surfaces));
    for(unsigned i=0;i<4;++i)native_legacy_rej_surfaces[i]=surfaces[i];
    const unsigned stateOffsets[3]={792,796,800},optionOffsets[3]={804,808,812};
    for(unsigned i=0;i<3;++i){uint32_t a=0,b=0;memcpy(&a,bytes+stateOffsets[i],4);memcpy(&b,bytes+optionOffsets[i],4);
        native_legacy_rej_state[i]=a;native_legacy_rej_options[i]=b;}
    uint32_t w=0,h=0;memcpy(&w,bytes+816,4);memcpy(&h,bytes+820,4);native_legacy_rej_size[0]=w;native_legacy_rej_size[1]=h;
    const unsigned rectOffsets[2]={17,81};
    for(unsigned r=0;r<2;++r){uint32_t rect[4]={};memcpy(rect,bytes+rectOffsets[r],sizeof(rect));
        for(unsigned i=0;i<4;++i)native_legacy_rej_rects[r][i]=rect[i];}
    char name[32]={};
    if(surface){const OSObject *object=reinterpret_cast<const OSObject *>(surface);
        const OSMetaClass *meta=object->getMetaClass();const char *cls=meta?meta->getClassName():nullptr;
        if(cls)strlcpy(name,cls,sizeof(name));}
    memcpy(native_legacy_rej_class,name,sizeof(name));
    // pdx203: this board has no boot-progress page to show the record, so the first eight refusals and every 512th after go to the
    // console (and the USB log) as GPUREJ. The surface fields are those NativeLegacyDisplay reported for this request (reason 4).
    const uint32_t count=native_legacy_swap_rejected_count;
    if(count<=8 || !(count%512)) {
        printf("GPUREJ n %u reason %u class %s size %ux%u rect %u,%u,%u,%u %u,%u,%u,%u state %x/%x/%x opt %x/%x/%x"
               " | surf %llux%llu fmt %08llx planes %llu stride %llu alloc %llu desc %llu\n",
               count,reason,name[0]?name:"-",native_legacy_rej_size[0],native_legacy_rej_size[1],
               native_legacy_rej_rects[0][0],native_legacy_rej_rects[0][1],native_legacy_rej_rects[0][2],native_legacy_rej_rects[0][3],
               native_legacy_rej_rects[1][0],native_legacy_rej_rects[1][1],native_legacy_rej_rects[1][2],native_legacy_rej_rects[1][3],
               native_legacy_rej_state[0],native_legacy_rej_state[1],native_legacy_rej_state[2],
               native_legacy_rej_options[0],native_legacy_rej_options[1],native_legacy_rej_options[2],
               (unsigned long long)native_legacy_rej_surface[0],(unsigned long long)native_legacy_rej_surface[1],
               (unsigned long long)native_legacy_rej_surface[2],(unsigned long long)native_legacy_rej_surface[3],
               (unsigned long long)native_legacy_rej_surface[4],(unsigned long long)native_legacy_rej_surface[5],
               (unsigned long long)native_legacy_rej_surface[6]);
    }
}
// Content statistics of presented frames (copy = kind 0, blank fill = kind 1): the first 32 are kept, with the time
// in seconds since the first frame. A per-frame record shows the animation cadence and when the OS blanks the panel.
#define NATIVE_LEGACY_FRAMES 32
volatile uint32_t native_legacy_frame_count;
volatile uint32_t native_legacy_frame_kind[NATIVE_LEGACY_FRAMES];
volatile uint32_t native_legacy_frame_time[NATIVE_LEGACY_FRAMES];
volatile uint64_t native_legacy_frame_stats[NATIVE_LEGACY_FRAMES][5];
static uint64_t frameEpoch;
void native_legacy_frame_observed(unsigned kind,uint64_t count,uint64_t minx,uint64_t miny,uint64_t maxx,uint64_t maxy) {
    uint32_t n=native_legacy_frame_count;
    uint64_t now,frequency;
    __asm__ volatile("isb\n\tmrs %0, cntpct_el0":"=r"(now));__asm__ volatile("mrs %0, cntfrq_el0":"=r"(frequency));
    if(!frameEpoch)frameEpoch=now;
    if(n<NATIVE_LEGACY_FRAMES) {
        native_legacy_frame_kind[n]=kind;
        native_legacy_frame_time[n]=frequency?(uint32_t)((now-frameEpoch)/frequency):0;
        native_legacy_frame_stats[n][0]=count;native_legacy_frame_stats[n][1]=minx;native_legacy_frame_stats[n][2]=miny;
        native_legacy_frame_stats[n][3]=maxx;native_legacy_frame_stats[n][4]=maxy;
    }
    native_legacy_frame_count=n+1;
}
volatile uint32_t native_legacy_frame_colors[8][4];
volatile uint64_t native_legacy_frame_color_counts[8][4],native_legacy_frame_color_other[8];
// Called right after the matching native_legacy_frame_observed(); fills slot count-1 (or the blank slot).
void native_legacy_frame_colors_observed(const uint32_t *colors,const uint64_t *counts,uint64_t other) {
    uint32_t n=native_legacy_frame_count;
    if(!n || n>8)return;   // dominant colours are kept for the first eight frames only
    n-=1;
    for(unsigned i=0;i<4;++i){native_legacy_frame_colors[n][i]=colors[i];native_legacy_frame_color_counts[n][i]=counts[i];}
    native_legacy_frame_color_other[n]=other;
}
// The life of every accepted swap, from the kernel's side: validated (the client submitted it), offered to the GPU copy (or started on the CPU),
// and retired. Times are microseconds since boot; the last 64 are kept.
// The display refresh tick (NativeLegacyDisplay, sheng-vsync): vsync messages sent, arms, idle stops, late ticks, starts, refresh
// properties reported at that rate, and the rate.
static volatile uint64_t vsyncEvents[6];
static volatile unsigned vsyncHz;
// Stage 3 is the swap's retirement (native_legacy_retirement_observed): with sheng-early-retire it comes at the offer, before the copy is done.
struct SwapEvent {uint64_t submitted,offered,done,retired;uint32_t number;};
static SwapEvent swapEvents[64];static uint32_t swapEventCount;
extern "C" {volatile uint32_t native_legacy_early_retires,native_legacy_early_failures;}
extern "C" void native_legacy_swap_stage(unsigned stage) {
    const uint64_t now=probeNowUs();
    uint32_t total=swapEventCount;
    if(stage==0) {
        SwapEvent &e=swapEvents[total%64];e={now,0,0,0,total+1};swapEventCount=total+1;return;
    }
    uint32_t first=total>64?total-64:0;
    for(uint32_t i=first;i<total;++i) {
        SwapEvent &e=swapEvents[i%64];
        if(stage==1 && !e.offered && !e.done){e.offered=now;return;}
        if(stage==2 && !e.done){e.done=now;if(!e.offered)e.offered=now;return;}
        if(stage==3 && !e.retired){e.retired=now;return;}
    }
}
// Summary of the last 40 finished swaps for the periodic DISPLAY row (UI frame rate, 2026-11-03: ~23.5 presents/s): the interval between
// submissions and the time from submission to the offer (hand-off start) and to retirement, average and maximum, in microseconds.
extern "C" unsigned native_legacy_swap_summary(char *out,unsigned cap) {
    uint32_t total=swapEventCount,first=total>40?total-40:0;
    // Also: submission to retirement (ret), retirement to the next submission (the compositor's turn, nx), and the gaps by display
    // refreshes (16.7 ms): at most 1, 2, 3, 4 and more (pdx203 usb-146: the gaps ran 32-44 ms; whole refreshes would mean the swaps wait
    // for the panel, anything between the compositor). Then the early retirements and the copies that failed after one.
    uint64_t n=0,gapSum=0,gapMax=0,offerSum=0,offerMax=0,doneSum=0,doneMax=0,prev=0,retSum=0,nextSum=0,nextN=0,prevRetired=0;
    uint32_t bins[5]={};
    for(uint32_t i=first;i<total;++i) {
        const SwapEvent &e=swapEvents[i%64];
        if(!e.done)continue;
        if(prev && e.submitted>prev) {
            uint64_t g=e.submitted-prev;gapSum+=g;if(g>gapMax)gapMax=g;
            const uint64_t r=(g+16666/2)/16667;++bins[r<1?0:r>4?4:r-1];
        }
        if(prevRetired && e.submitted>prevRetired){nextSum+=e.submitted-prevRetired;++nextN;}
        prev=e.submitted;prevRetired=e.retired;
        uint64_t o=e.offered-e.submitted,d=e.done-e.submitted;
        offerSum+=o;if(o>offerMax)offerMax=o;doneSum+=d;if(d>doneMax)doneMax=d;++n;
        if(e.retired>e.submitted)retSum+=e.retired-e.submitted;
    }
    if(!n)return snprintf(out,cap,"swaps none");
    return snprintf(out,cap,"swaps %llu gap avg/max %llu/%llu us offer %llu/%llu us done %llu/%llu us ret %llu nx %llu | v %u/%u/%u/%u/%u | er %u/%u | vs %u s%llu l%llu i%llu r%llu",
        (unsigned long long)n,(unsigned long long)(n>1?gapSum/(n-1):0),(unsigned long long)gapMax,(unsigned long long)(offerSum/n),
        (unsigned long long)offerMax,(unsigned long long)(doneSum/n),(unsigned long long)doneMax,(unsigned long long)(retSum/n),
        (unsigned long long)(nextN?nextSum/nextN:0),bins[0],bins[1],bins[2],bins[3],bins[4],native_legacy_early_retires,native_legacy_early_failures,
        // the vsync tick: its rate, messages sent to the compositor, late ticks, idle stops (no client), refresh properties reported
        vsyncHz,(unsigned long long)vsyncEvents[1],(unsigned long long)vsyncEvents[3],(unsigned long long)vsyncEvents[2],(unsigned long long)vsyncEvents[5]);
}
extern "C" unsigned native_legacy_swap_timeline(char *out,unsigned cap) {
    unsigned at=0;out[0]=0;
    uint32_t total=swapEventCount,first=total>40?total-40:0;
    for(uint32_t i=first;i<total && at+90<cap;++i) {
        const SwapEvent &e=swapEvents[i%64];
        uint64_t previous=i>first?swapEvents[(i-1)%64].submitted:e.submitted;
        at+=snprintf(out+at,cap-at,"W%u t=%llu.%03llus +%llums offer=%lldms done=%lldms\n",e.number,(unsigned long long)(e.submitted/1000000),
            (unsigned long long)((e.submitted/1000)%1000),(unsigned long long)((e.submitted-previous)/1000),
            e.offered?(long long)((e.offered-e.submitted)/1000):-1LL,e.done?(long long)((e.done-e.submitted)/1000):-1LL);
    }
    return at;
}
void native_legacy_vsync_observed(unsigned event,unsigned hz) {
    if(event<6)vsyncEvents[event]=vsyncEvents[event]+1;
    vsyncHz=hz;
}
extern "C" unsigned native_legacy_prepare_report(char *out,unsigned cap) {
    return snprintf(out,cap,"prepare calls=%u failures=%u retried=%u max_attempts=%u waited_ms=%llu vsync hz=%u sent=%llu arms=%llu idle=%llu late=%llu start=%llu refresh=%llu",
        native_legacy_prepare_calls,native_legacy_prepare_failures,native_legacy_prepare_retried,native_legacy_prepare_max_attempts,
        (unsigned long long)(native_legacy_prepare_wait_us/1000),vsyncHz,(unsigned long long)vsyncEvents[1],(unsigned long long)vsyncEvents[0],
        (unsigned long long)vsyncEvents[2],(unsigned long long)vsyncEvents[3],(unsigned long long)vsyncEvents[4],(unsigned long long)vsyncEvents[5]);
}
void native_legacy_swap_validation_observed(bool accepted,unsigned reason,const void *request,const void *surface) {
    if(accepted)native_legacy_swap_stage(0);
    if(accepted)native_legacy_swap_accepted_count=native_legacy_swap_accepted_count+1;
    // pdx203: the first accepted swap and every 4096th, with the refusals so far (GPUREJ prints those)
    // (Until 2026-11-03 the else below belonged to the print's condition: every accepted swap but the first and each 4096th counted as rejected.)
    if(accepted) {
        if(native_legacy_swap_accepted_count==1 || !(native_legacy_swap_accepted_count%4096))
            printf("GPUSWAP accepted %u reason %u rejected %u\n",native_legacy_swap_accepted_count,reason,native_legacy_swap_rejected_count);
    } else {native_legacy_swap_rejected_count=native_legacy_swap_rejected_count+1;recordRejectedSwap(reason,request,surface);}
    native_legacy_swap_validation_calls=native_legacy_swap_validation_calls+1;
    native_legacy_swap_validation_result=accepted;native_legacy_swap_validation_reason=reason;
    native_legacy_swap_request=(uintptr_t)request;native_legacy_swap_surface=(uintptr_t)surface;
    native_legacy_swap_validation_checkpoint(accepted,reason,request,surface);
}
// Time of the pixel copy into the scanout buffer (counter ticks): swaps completed, last/maximum/sum, for the banner.
volatile uint64_t native_legacy_copy_n,native_legacy_copy_last,native_legacy_copy_max,native_legacy_copy_sum,native_legacy_copy_bytes_last,native_legacy_copy_bytes_sum;
void native_legacy_copy_timing_observed(uint64_t ticks,uint64_t bytes) {
    native_legacy_copy_bytes_last=bytes;native_legacy_copy_bytes_sum=native_legacy_copy_bytes_sum+bytes;
    native_legacy_copy_n=native_legacy_copy_n+1;native_legacy_copy_last=ticks;native_legacy_copy_sum=native_legacy_copy_sum+ticks;
    if(ticks>native_legacy_copy_max)native_legacy_copy_max=ticks;
}
__attribute__((noinline)) void native_legacy_transfer_observed(IOReturn status,const void *request,uint64_t bytes) {
    native_legacy_transfer_calls=native_legacy_transfer_calls+1;
    if(status==kIOReturnSuccess)native_legacy_swap_stage(2);
    native_legacy_transfer_status=status;native_legacy_transfer_request=(uintptr_t)request;native_legacy_transfer_bytes=bytes;
    if(status!=kIOReturnSuccess && boot_progress_stage) {
        char message[120];snprintf(message,sizeof(message),"DISPLAY COPY FAILED status=%08x prepare=%u tries=%u",
          (unsigned)status,native_legacy_prepare_reason,native_legacy_prepare_attempts);
        boot_progress_stage(message);
    }
}
bool native_legacy_gpu_offer(void *display,IOMemoryDescriptor *memory,uint64_t stride,uint32_t width,uint32_t height) {
    return gpuHandoffPresenter && native_gop_gpu_offer(gpuHandoffPresenter,display,memory,stride,width,height);
}
void native_legacy_gpu_fallback_observed(int status) {
    native_legacy_gpu_last_status=(uint32_t)status;native_legacy_gpu_fallbacks=native_legacy_gpu_fallbacks+1;
}
__attribute__((noinline)) void native_legacy_retirement_observed(IOService *display) {
    const unsigned char *bytes=reinterpret_cast<const unsigned char *>(display);
    uint64_t current=0,pending=0;uint32_t completed=0,presented=0,frames=0;
    memcpy(&current,bytes+5472,8);memcpy(&pending,bytes+5480,8);
    memcpy(&completed,bytes+5820,4);memcpy(&presented,bytes+5824,4);memcpy(&frames,bytes+252,4);
    native_legacy_retired_current=current;native_legacy_retired_pending=pending;
    native_legacy_retired_generation=completed;native_legacy_presented_generation=presented;native_legacy_retired_frames=frames;
    native_legacy_retirement_calls=native_legacy_retirement_calls+1;
    native_legacy_swap_stage(3);
}
// The display object's properties as they are now (the registration copy above predates setDisplayRefreshProperties and the rest): for the
// UFS log. Returns the bytes written (NUL-terminated).
extern "C" unsigned native_legacy_properties_now(char *out,unsigned cap) {
    if(!out || !cap)return 0;
    out[0]=0;
    IOService *display=reinterpret_cast<IOService *>(native_legacy_object);
    if(!display)return 0;
    OSSerialize *xml=OSSerialize::withCapacity(8192);
    unsigned size=0;
    if(xml && display->serializeProperties(xml)) {
        size=xml->getLength();if(size>=cap)size=cap-1;
        memcpy(out,xml->text(),size);out[size]=0;
    }
    OSSafeReleaseNULL(xml);
    return size;
}
void native_legacy_registration_observed(IOService *display) {
    native_legacy_registration_calls=native_legacy_registration_calls+1;
    OSSerialize *xml=OSSerialize::withCapacity(2048);
    if(xml && display->serializeProperties(xml)) {
        unsigned size=xml->getLength();if(size>=sizeof(native_legacy_properties))size=sizeof(native_legacy_properties)-1;
        memcpy(native_legacy_properties,xml->text(),size);native_legacy_properties[size]=0;native_legacy_properties_length=size;
    }
    OSSafeReleaseNULL(xml);
}
void native_legacy_client_enter(task_t task,uint32_t type) {
    native_legacy_client_calls=native_legacy_client_calls+1;native_legacy_client_task=(uintptr_t)task;
    native_legacy_client_type=type;native_legacy_client_pid=task?proc_pid((proc_t)get_bsdtask_info(task)):0;
}
// Every framebuffer user client seen (object, pid), with what it does: the original client's vtable is copied and two entries (externalMethod, slot 168,
// and registerNotificationPort(port,type,refCon), slot 175) are pointed at wrappers that count and forward to the original function. The original code
// is not touched; only this object's vptr names the copy. XNU's ARM64 vptr points at slot 0 (the original code dispatches with ldr x8,[vptr,#slot*8]).
enum {kSeenClients=8,kSelectors=96,kRegEvents=8,kVtableSlots=256};
struct ClientStat {
    uintptr_t object;int pid;uint32_t calls,fails;uint32_t selector_calls[kSelectors],selector_fails[kSelectors],selector_status[kSelectors];
    unsigned regs;struct {uint32_t type;uint64_t port;uint32_t ref,status;} reg[kRegEvents];
    uint64_t sel6_first[3],sel6_last[3];uint32_t sel6_distinct;uint64_t sel6_seen[8][3];uint32_t sel6_calls[8],sel6_fails[8],sel6_status[8];
    uint32_t seq_count;struct {uint32_t a,b,c,status,done,presented;} seq[24];
    uintptr_t original_external,original_register;
};
static ClientStat seenClients[kSeenClients];static unsigned seenClientCount;
typedef IOReturn (*ExternalFn)(IOUserClient *,uint32_t,IOExternalMethodArguments *,IOExternalMethodDispatch *,OSObject *,void *);
typedef IOReturn (*RegisterFn)(IOUserClient *,mach_port_t,UInt32,UInt32);
static ClientStat *statFor(IOUserClient *client) {
    for(unsigned i=0;i<seenClientCount;++i)if(seenClients[i].object==(uintptr_t)client)return &seenClients[i];
    return nullptr;
}
// pdx203 (usb-154): the render server's freezes are its display thread blocked in kern_SwapWait (selector 6, from
// IOMFBDisplay::can_update_) while every swap had been validated, transferred and retired. The last 32 selector-6 calls of any
// client: arguments, when it began (ms since boot), how long it took (or that it is still waiting), its return, and the completed and
// presented generations (+5820, +5824) at entry and exit. Printed by native_legacy_swapwait_dump (FBWAIT rows) when backboardd or
// SpringBoard exits on a signal (the board's Pdx203FaultLog.c) and with the FBCLI rows.
struct SwapWaitCall {uint64_t args[3],beganUs,endedUs;int pid;uint32_t status,doneIn,presentedIn,doneOut,presentedOut;bool ended;};
static SwapWaitCall swapWaits[32];static uint32_t swapWaitCount;
static void swapGenerations(uint32_t *done,uint32_t *presented) {
    const unsigned char *object=reinterpret_cast<const unsigned char *>(native_legacy_object);
    *done=*presented=0;if(object){memcpy(done,object+5820,4);memcpy(presented,object+5824,4);}
}
extern "C" unsigned native_legacy_swapwait_dump(char *out,unsigned cap) {
    unsigned at=0;if(!out || !cap)return 0;out[0]=0;
    const uint64_t now=probeNowUs();
    uint32_t done=0,presented=0;swapGenerations(&done,&presented);
    at+=snprintf(out+at,cap-at,"FBWAIT now %llu ms gen %u/%u calls %u\n",(unsigned long long)(now/1000),done,presented,swapWaitCount);
    const uint32_t total=swapWaitCount,first=total>32?total-32:0;
    for(uint32_t i=first;i<total && at+140<cap;++i) {
        const SwapWaitCall &c=swapWaits[i%32];
        at+=snprintf(out+at,cap-at,"FBWAIT #%u pid %d args %llx %llx %llx at %llu ms %s %llu us -> %x gen %u/%u -> %u/%u\n",i,c.pid,
            (unsigned long long)c.args[0],(unsigned long long)c.args[1],(unsigned long long)c.args[2],(unsigned long long)(c.beganUs/1000),
            c.ended?"took":"WAITING",(unsigned long long)((c.ended?c.endedUs:now)-c.beganUs),c.status,c.doneIn,c.presentedIn,c.doneOut,c.presentedOut);
    }
    return at;
}
static IOReturn wrappedExternal(IOUserClient *client,uint32_t selector,IOExternalMethodArguments *args,IOExternalMethodDispatch *dispatch,OSObject *target,void *ref) {
    ClientStat *stat=statFor(client);
    SwapWaitCall *wait=nullptr;
    if(selector==6 && args && args->scalarInput && args->scalarInputCount>=3) {
        wait=&swapWaits[swapWaitCount%32];++swapWaitCount;
        memset(wait,0,sizeof *wait);memcpy(wait->args,args->scalarInput,sizeof wait->args);
        wait->pid=stat?stat->pid:-1;wait->beganUs=probeNowUs();swapGenerations(&wait->doneIn,&wait->presentedIn);
    }
    IOReturn status=stat?reinterpret_cast<ExternalFn>(stat->original_external)(client,selector,args,dispatch,target,ref):kIOReturnNotAttached;
    if(wait) {
        // a newer call may have reused the slot while this one waited (32 calls later): then only that one is kept
        if(!wait->ended && !memcmp(wait->args,args->scalarInput,sizeof wait->args)) {
            wait->endedUs=probeNowUs();wait->status=(uint32_t)status;swapGenerations(&wait->doneOut,&wait->presentedOut);wait->ended=true;
        }
    }
    if(stat) {
        if(selector==6 && args && args->scalarInput && args->scalarInputCount>=3) {
            const uint64_t *in=args->scalarInput;
            if(!stat->sel6_distinct && !stat->sel6_first[0] && !stat->sel6_first[1])memcpy(stat->sel6_first,in,sizeof stat->sel6_first);
            memcpy(stat->sel6_last,in,sizeof stat->sel6_last);
            int at=-1;
            for(unsigned k=0;k<stat->sel6_distinct && k<8;++k)if(!memcmp(stat->sel6_seen[k],in,24))at=(int)k;
            if(at<0 && stat->sel6_distinct<8){at=(int)stat->sel6_distinct++;memcpy(stat->sel6_seen[at],in,24);}
            if(stat->seq_count<24){
                auto &e=stat->seq[stat->seq_count++];e.a=(uint32_t)in[0];e.b=(uint32_t)in[1];e.c=(uint32_t)in[2];e.status=(uint32_t)status;
                const unsigned char *object=reinterpret_cast<const unsigned char *>(native_legacy_object);
                e.done=e.presented=0;if(object){memcpy(&e.done,object+5820,4);memcpy(&e.presented,object+5824,4);}
            }
            if(at>=0){++stat->sel6_calls[at];if(status)++stat->sel6_fails[at];stat->sel6_status[at]=(uint32_t)status;}
        }
        ++stat->calls;if(status)++stat->fails;
        if(selector<kSelectors){++stat->selector_calls[selector];if(status)++stat->selector_fails[selector];stat->selector_status[selector]=(uint32_t)status;}
    }
    return status;
}
static IOReturn wrappedRegister(IOUserClient *client,mach_port_t port,UInt32 type,UInt32 refCon) {
    ClientStat *stat=statFor(client);
    IOReturn status=stat?reinterpret_cast<RegisterFn>(stat->original_register)(client,port,type,refCon):kIOReturnNotAttached;
    if(stat && stat->regs<kRegEvents){auto &e=stat->reg[stat->regs++];e.type=type;e.port=(uint64_t)(uintptr_t)port;e.ref=refCon;e.status=(uint32_t)status;}
    return status;
}
static void interpose(IOUserClient *client,int pid) {
    if(seenClientCount>=kSeenClients)return;
    ClientStat &stat=seenClients[seenClientCount];
    memset(&stat,0,sizeof stat);
    uintptr_t *vtable=*reinterpret_cast<uintptr_t **>(client);
    uintptr_t *copy=static_cast<uintptr_t *>(IOMalloc(kVtableSlots*sizeof(uintptr_t)));
    if(!copy)return;
    memcpy(copy,vtable,kVtableSlots*sizeof(uintptr_t));
    stat.object=(uintptr_t)client;stat.pid=pid;stat.original_external=copy[168];stat.original_register=copy[175];
    copy[168]=(uintptr_t)&wrappedExternal;copy[175]=(uintptr_t)&wrappedRegister;
    ++seenClientCount;
    *reinterpret_cast<uintptr_t **>(client)=copy;
}
// The display object's state words the original wait selector tests (offsets 5832..5864, as dwords), for the log.
extern "C" unsigned native_legacy_state_words(char *out,unsigned cap) {
    out[0]=0;
    const unsigned char *object=reinterpret_cast<const unsigned char *>(native_legacy_object);
    if(!object)return 0;
    unsigned at=snprintf(out,cap,"d+5808:");
    for(unsigned o=5808;o<5872 && at+16<cap;o+=4){uint32_t v;memcpy(&v,object+o,4);at+=snprintf(out+at,cap-at," %x",v);}
    return at;
}
extern "C" unsigned native_legacy_clients_dump(char *out,unsigned cap) {
    unsigned at=0;out[0]=0;
    for(unsigned c=0;c<seenClientCount && at+120<cap;++c) {
        const ClientStat &s=seenClients[c];
        at+=snprintf(out+at,cap-at,"C%u pid=%d calls=%u fails=%u regs=%u",c,s.pid,s.calls,s.fails,s.regs);
        for(unsigned r=0;r<s.regs && r<kRegEvents && at+60<cap;++r)
            at+=snprintf(out+at,cap-at," reg(t%u,p%llx,r%x)=%x",s.reg[r].type,(unsigned long long)s.reg[r].port,s.reg[r].ref,s.reg[r].status);
        at+=snprintf(out+at,cap-at,"\n");
        for(unsigned k=0;k<s.sel6_distinct && k<8 && at+100<cap;++k)
            at+=snprintf(out+at,cap-at,"  sel6 args %llx %llx %llx: %u calls %u fails last %x\n",(unsigned long long)s.sel6_seen[k][0],(unsigned long long)s.sel6_seen[k][1],(unsigned long long)s.sel6_seen[k][2],s.sel6_calls[k],s.sel6_fails[k],s.sel6_status[k]);
        for(unsigned k=0;k<s.seq_count && at+90<cap;++k)
            at+=snprintf(out+at,cap-at,"  s6#%u %x %x %x -> %x [5820]=%x [5824]=%x\n",k,s.seq[k].a,s.seq[k].b,s.seq[k].c,s.seq[k].status,s.seq[k].done,s.seq[k].presented);
        unsigned n=0;
        for(unsigned sel=0;sel<kSelectors && at+40<cap;++sel) {
            if(!s.selector_calls[sel])continue;
            if(!n)at+=snprintf(out+at,cap-at,"  sel");
            at+=snprintf(out+at,cap-at," %u:%u/%u/%x",sel,s.selector_calls[sel],s.selector_fails[sel],s.selector_status[sel]);
            if(++n%6==0)at+=snprintf(out+at,cap-at,"\n  sel");
        }
        if(n)at+=snprintf(out+at,cap-at,"\n");
    }
    return at;
}
void native_legacy_client_observed(IOReturn status,IOUserClient *client) {
    if(client && !status)interpose(client,native_legacy_client_pid);
    native_legacy_client_status=status;native_legacy_client_object=(uintptr_t)client;
    if(client)strlcpy(native_legacy_client_class,client->getMetaClass()->getClassName(),sizeof(native_legacy_client_class));
    native_legacy_client_checkpoint();
}
__attribute__((noinline)) bool native_legacy_gop_mapping_observed(IOService *object,IOMemoryMap *mapping,uint32_t width,uint32_t height) {
    if((uintptr_t)object!=native_legacy_object)panic("NativeLegacyDisplay: wrong mapping owner");
    native_legacy_gop_map=(uintptr_t)mapping;native_legacy_gop_virtual=mapping->getVirtualAddress();
    native_legacy_gop_physical=mapping->getPhysicalAddress();native_legacy_gop_bytes=mapping->getLength();
    native_legacy_gop_width=width;native_legacy_gop_height=height;
    if(boot_progress_pulse_anchor && !diagnosticAnchorMapping) {
        PE_Video anchorVideo={};
        if(!PE_current_console(&anchorVideo) && anchorVideo.v_depth==32 &&
           anchorVideo.v_rowBytes<=65536 &&
           anchorVideo.v_width==width && anchorVideo.v_height==height &&
           anchorVideo.v_baseAddr==mapping->getPhysicalAddress() && !anchorVideo.v_rotate) {
            mapping->retain();
            if(boot_progress_pulse_anchor(mapping->getVirtualAddress(),mapping->getLength(),width,height,
                (unsigned)anchorVideo.v_rowBytes))diagnosticAnchorMapping=mapping;
            else mapping->release();
        }
    }
    native_legacy_gop_mapping_checkpoint();
    unsigned enabled=0;PE_parse_boot_argn("offline-legacy-unit-scale",&enabled,sizeof(enabled));
    if(!enabled)return true;
    native_legacy_unit_scale_calls=native_legacy_unit_scale_calls+1;native_legacy_unit_scale_status=1;
    unsigned phase=0;PE_parse_boot_argn("offline-legacy-phase",&phase,sizeof(phase));
    PE_Video video={};
    // The value is the UI scale: logical points = actual scanout pixels / scale (1..3). It changes
    // only the published display-scale; the scanout copy still moves one actual pixel per pixel.
    if(enabled<1 || enabled>3 || phase!=4 || unitScaleChosen || PE_current_console(&video) ||
       video.v_width!=width || video.v_height!=height || video.v_rotate ||
       mapping->getPhysicalAddress()!=video.v_baseAddr)return false;
    // Explicit offline UI policy: `enabled` actual GOP scanout pixels per logical
    // point in each direction (1 = native pixels). This is not physical pixel density,
    // panel timing, an Apple device identity or a synthesized mode list.
    unitScaleChosen=IORegistryEntry::fromPath("/chosen",gIODTPlane);
    if(!unitScaleChosen || !publishOrMatchWord(unitScaleChosen,"display-scale",enabled,unitScaleAdded) ||
       !publishOrMatchWord(unitScaleChosen,"display-rotation",video.v_rotate,unitRotationAdded)) {
        finishUnitScalePolicy(false);return false;
    }
    OSData *scale=OSDynamicCast(OSData,unitScaleChosen->getProperty("display-scale"));
    OSData *rotation=OSDynamicCast(OSData,unitScaleChosen->getProperty("display-rotation"));
    uint32_t actualScale=0,actualRotation=0;
    if(!scale || !rotation || scale->getLength()!=4 || rotation->getLength()!=4) {
        finishUnitScalePolicy(false);return false;
    }
    memcpy(&actualScale,scale->getBytesNoCopy(),4);memcpy(&actualRotation,rotation->getBytesNoCopy(),4);
    native_legacy_chosen_scale=actualScale;native_legacy_chosen_rotation=actualRotation;
    native_legacy_unit_scale_status=actualScale==enabled && actualRotation==video.v_rotate?0:2;
    if(native_legacy_unit_scale_status){finishUnitScalePolicy(false);return false;}
    return true;
}
__attribute__((noreturn,noinline)) void native_legacy_backend_guard(unsigned slot,void *object,void *caller) {
    native_legacy_guard_slot=slot;native_legacy_guard_object=(uintptr_t)object;native_legacy_guard_caller=(uintptr_t)caller;
    // The panic text has not reached the USB log on pdx203 (usb-157: the display thread entered slot 345 and the system ran on without a
    // panic line). Print the leaf first and give the log's drain a moment (a spin: the caller may hold a gate or run unpreemptible).
    printf("NativeLegacyDisplay: leaf=%u caller=%llx object=%llx\n",slot,(uint64_t)(uintptr_t)caller,(uint64_t)(uintptr_t)object);
    IODelay(300000);
    panic("NativeLegacyDisplay: leaf=%u caller=%llx stage=%u PM=%u swap=%u/%u copy=%08x bytes=%llu retire=%u handoff=%u/%u",
      slot,(uint64_t)(uintptr_t)caller,native_legacy_stage,native_legacy_pm_stage,
      native_legacy_swap_validation_result,native_legacy_swap_validation_reason,
      (unsigned)native_legacy_transfer_status,native_legacy_transfer_bytes,native_legacy_retirement_calls,
      native_legacy_prepare_reason,native_legacy_prepare_attempts);
}
void native_legacy_display_probe(void);
void native_legacy_display_probe(void) {
    // The registration path and optional original-userclient inspection both
    // reach this boot-thread entry. Run class constructors only once; a second
    // postModLoad would attempt to register the same metaclasses again.
    if(native_legacy_stage)return;
    native_legacy_stage=1;void *handle=OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if(!handle){native_legacy_display_checkpoint();return;}
    native_legacy_display_static_init();native_gop_presenter_static_init();native_legacy_meta_status=OSMetaClass::postModLoad(handle);
    if(native_legacy_meta_status){native_legacy_display_checkpoint();return;}
    PE_Video video={};native_legacy_stage=2;
    if(PE_current_console(&video) || !video.v_baseAddr || !video.v_width || !video.v_height || video.v_depth!=32
        || video.v_rotate || video.v_width>16384 || video.v_height>16384 || video.v_rowBytes<video.v_width*4
        || video.v_rowBytes>UINT64_MAX/video.v_height){native_legacy_display_checkpoint();return;}
    IOService *display=native_legacy_display_allocate();native_legacy_object=(uintptr_t)display;
    if(!display){native_legacy_display_checkpoint();return;}
    native_legacy_object_size=display->getMetaClass()->getClassSize();
    native_legacy_parent_match=(display->metaCast("IOMobileFramebufferLegacy") && display->metaCast("IOMobileFramebuffer") && display->metaCast("IOService"));
    native_legacy_stage=3;native_legacy_init_result=display->init();
    unsigned phase=1;PE_parse_boot_argn("offline-legacy-phase",&phase,sizeof(phase));
    if(native_legacy_init_result && (phase==2 || phase==3 || phase==4)) {
        // IOSurface matches the real IOBSD resource, published by bsd_autoconf.
        // Retain this actual object until that normal lifecycle boundary.
        pendingLegacyDisplay=display;return;
    }
    native_legacy_stage=7;display->release();native_legacy_released=1;native_legacy_stage=8;
    native_legacy_display_checkpoint();
}
void native_legacy_display_late_probe(void);
// pdx203: sheng's late probe also starts its board probes here (keystore, cpufreq, gpu power, touch, wifi, buttons,
// backlight, hid input). this board has the hid input device (package/HidInputProbe.cpp, offline-hid-input=1) and the sec_ts
// touch probe (package/Pdx203TouchProbe.cpp, pdx203-touch=1); both start before the display part, which returns early when no
// display is pending, as alioth orders them.
extern "C" int native_hid_input_probe(void);
extern "C" bool pdx203_touch_start(void);
// the AppleKeyStore stand-in (package/KeyStoreProbe.cpp, drivers/NativeKeyStoreEvents; offline-key-events=1, sheng-aks=1):
// with no SEP, it answers backboardd and SpringBoard's state queries as "no passcode, nothing locked"
extern "C" void native_keystore_probe(void);
// the cpu clocks (package/CpuFreqProbe.cpp, alioth's epss voter; sheng-cpufreq=<mode>): first, so the rest of the boot runs faster
extern "C" void pdx203_cpufreq_start(void);
// the adreno 650 (package/GpuProbe.cpp, alioth's; pipa-gpu=<mode>, nothing without it): before the display part, as alioth
extern "C" void pdx203_gpu_start(void);
// the power and volume buttons (package/ButtonProbe.cpp, alioth's with this board's keys; pdx203-bt=<mode>): their own thread,
// which waits for the interrupt controller, and they feed the hid input device's report 2
extern "C" void pdx203_buttons_probe(void);
// the battery as an IOPMPowerSource (package/Pdx203Battery.cpp; pdx203-battery=1): PM8150B fuel gauge and charger over the
// arbiter the button thread maps
extern "C" void pdx203_battery_probe(void);
// the time of day (package/Pdx203Rtc.cpp; pdx203-rtc=0 turns it off): the PM8150 RTC plus an offset kept in the RTC alarm registers,
// published as the IORTC resource AppleARMPE reads the calendar from; needs the arbiter the button thread maps
extern "C" void pdx203_rtc_probe(void);
// the audio DSP (package/AdspProbe.cpp, sheng's for SM8250; pdx203-adsp=<mode>) and the speaker path on top of it
// (package/Pdx203Audio.cpp; pdx203-audio=<bits>): their own threads, nothing without the arguments
extern "C" void sheng_adsp_probe(void);
extern "C" void pdx203_audio_probe(void);
// the QCA6390 WiFi chip on PCIe0 (package/Pdx203Wifi.cpp; pdx203-wifi=<stage>): its rails go through the ADSP thread's RPMh helper
extern "C" void pdx203_wifi_probe(void);
void native_legacy_display_late_probe(void) {
    pdx203_cpufreq_start();
    pdx203_gpu_start();
    native_keystore_probe();
    (void)native_hid_input_probe();
    pdx203_buttons_probe();
    pdx203_battery_probe();
    pdx203_rtc_probe();
    sheng_adsp_probe();
    pdx203_audio_probe();
    pdx203_wifi_probe();
    unsigned touch=0;
    if(PE_parse_boot_argn("pdx203-touch",&touch,sizeof(touch)) && touch==1)(void)pdx203_touch_start();
    IOService *display=pendingLegacyDisplay;if(!display)return;pendingLegacyDisplay=nullptr;
    native_legacy_surface_status=OSKextLoadKextWithIdentifier("com.apple.iokit.IOSurface");
    OSDictionary *matching=IOService::nameMatching("IOCoreSurfaceRoot");
    IOService *surface=matching?IOService::waitForMatchingService(matching,5000000000ULL):nullptr;
    native_legacy_surface_found=surface!=nullptr;OSSafeReleaseNULL(surface);OSSafeReleaseNULL(matching);
    PE_Video video={};
    if(!native_legacy_surface_status && native_legacy_surface_found && !PE_current_console(&video)
        && video.v_baseAddr && video.v_width && video.v_height && video.v_depth==32
        && !video.v_rotate && video.v_width<=16384 && video.v_height<=16384
        && video.v_rowBytes>=video.v_width*4 && video.v_rowBytes<=UINT64_MAX/video.v_height) {
        native_legacy_video_base=video.v_baseAddr;native_legacy_video_bytes=video.v_rowBytes*video.v_height;
        // A real firmware framebuffer resource, with addresses and extent
        // obtained from the actual GOP handoff. It has no invented compatible,
        // display list, panel timing, GPU/fence completion or vendor registers.
        native_legacy_stage=4;IOPlatformDevice *nub=new IOPlatformDevice;
        native_legacy_nub=(uintptr_t)nub;bool nubAttached=false,nubStarted=false,attached=false,started=false;
        IORegistryEntry *displayTreeParent=nullptr;bool dtAttached=false;
        IOService *parent=IOService::getResourceService();
        IODeviceMemory *memory=IODeviceMemory::withRange(video.v_baseAddr,video.v_rowBytes*video.v_height);
        OSArray *ranges=OSArray::withCapacity(1);
        if(nub && memory && ranges && parent && nub->init()) {
            ranges->setObject(memory);nub->setDeviceMemory(ranges);nub->setName("NativeGOPFramebuffer");
            nubAttached=nub->attach(parent);if(nubAttached)nubStarted=nub->start(parent);
            if(nubStarted) {
                unsigned phase=0;PE_parse_boot_argn("offline-legacy-phase",&phase,sizeof(phase));
                if(phase==4) {
                    // This is the actually implemented GOP display subsystem,
                    // attached to its real IOPlatformDevice memory resource.
                    // No Apple/vendor compatible or panel timing is supplied.
                    displayTreeParent=IORegistryEntry::fromPath("/arm-io",gIODTPlane);
                    const char type[]="display-subsystem";OSData *data=OSData::withBytes(type,sizeof(type));
                    if(displayTreeParent && data && nub->setProperty("device_type",data)) {
                        nub->setName("gop-display",gIODTPlane);dtAttached=nub->attachToParent(displayTreeParent,gIODTPlane);
                    }
                    OSSafeReleaseNULL(data);native_legacy_dt_attached=dtAttached;
                }
                if(phase==3) {
                    IOService *presenter=native_gop_presenter_create();bool presenterAttached=false;
                    if(presenter && presenter->init() && (presenterAttached=presenter->attach(nub)) && presenter->start(nub)) {
                        // Explicit own transport service only. The incomplete
                        // original framebuffer backend is never published.
                        activeGOPPresenter=presenter;activeGOPNub=nub;nub=nullptr;nubAttached=false;nubStarted=false;
                    } else {if(presenterAttached)presenter->detach(nub);OSSafeReleaseNULL(presenter);}
                } else {
                    SInt32 score=0;native_legacy_probe_match=(phase!=4 || dtAttached) && display->probe(nub,&score)==display;
                    if(native_legacy_probe_match)attached=display->attach(nub);
                    if(attached){native_legacy_stage=5;started=display->start(nub);native_legacy_start_result=started;}
                    if(started && phase==4) {
                        activeLegacyDisplay=display;activeGOPNub=nub;native_legacy_stage=10;
                        {
                            unsigned gpu=0;PE_parse_boot_argn("sheng-fb-gpu",&gpu,sizeof gpu);
                            if(gpu==1) {
                                IOService *handoff=native_gop_presenter_create();
                                // Not a child of the display subsystem's node: the original display code enumerates that node's children (int-075/077:
                                // SpringBoard found no display and crashed with the presenter attached there, registered or not).
                                IOService *home=IOService::getResourceService();
                                bool handoffAttached=false;
                                if(handoff && home)native_gop_presenter_set_source(handoff,nub);
                                if(handoff && home && handoff->init() && (handoffAttached=handoff->attach(home)) && handoff->start(home))gpuHandoffPresenter=handoff;
                                else {if(handoffAttached)handoff->detach(home);OSSafeReleaseNULL(handoff);}
                            }
                        }
                        finishUnitScalePolicy(true);
                        OSSafeReleaseNULL(displayTreeParent);
                        native_legacy_display_checkpoint();return;
                    }
                }
            }
        }
        native_legacy_stage=6;finishUnitScalePolicy(false);
        if(started)display->stop(nub);if(attached)display->detach(nub);
        if(dtAttached)nub->detachFromParent(displayTreeParent,gIODTPlane);
        if(nubStarted)nub->stop(parent);if(nubAttached)nub->detach(parent);
        OSSafeReleaseNULL(ranges);OSSafeReleaseNULL(memory);OSSafeReleaseNULL(nub);
        OSSafeReleaseNULL(displayTreeParent);
    }
    native_legacy_stage=7;display->release();native_legacy_released=1;native_legacy_stage=8;
    native_legacy_display_checkpoint();
}
}

// pdx203: alioth's stride wrapper (UniXNU-alioth package/LinkStubs.c). A compositor surface can report its row pitch in pixels in
// the IOSurface field the display reads (alioth: 1088 for a 1080-wide window of four bytes a pixel), while
// native_legacy_surface_bounds compares it with width*4, so every GPU-composited swap was refused (GPUREJ: stride 1088, need 4320).
// The field is taken as pixels only when the allocation and the row count prove it; a getter that returns bytes is left alone.
// scripts/build-pdx203-legacy-display.py binds the original getter as native_legacy_surface_stride_raw.
extern "C" uint64_t native_legacy_surface_stride_raw(OSObject *), native_legacy_surface_allocation(OSObject *),
    native_legacy_surface_height(OSObject *);
extern "C" uint32_t native_legacy_surface_format(OSObject *), native_legacy_surface_planes(OSObject *);
extern "C" uint64_t native_legacy_surface_stride(OSObject *surface) {
    const uint64_t stride = native_legacy_surface_stride_raw(surface);
    const uint64_t allocation = native_legacy_surface_allocation(surface);
    const uint64_t height = native_legacy_surface_height(surface);
    // 'b3a8' (the render server's deep framebuffer, two planes): the field is plane 1's (alpha bytes) row pitch, and plane 0 (the
    // color words the display narrows, NativeLegacyRequest.h native_legacy_copy_xr10) fills the rest of each row's share of the
    // allocation. usb-31: stride 1152, allocation 14254080, height 2560 -> plane 0 at 4416 bytes a row (1096 x 4, rounded to 64).
    if (native_legacy_surface_format(surface) == 0x62336138U && native_legacy_surface_planes(surface) == 2 && stride && height &&
        !(allocation % height) && allocation / height > stride) return allocation / height - stride;
    if (stride && allocation && height && stride <= allocation / height / 4) return stride * 4;
    return stride;
}

// pdx203: a read-only dump of the display engine as the boot loader left it, for the page flip (DPU 6.0, SM8250; offsets of the stock
// device tree's qcom,mdss_mdp from 0xae00000: CTL 0x2000 + 0x200 n, VIG 0x5000 + 0x2000 n, DMA 0x25000 + 0x2000 n, mixers 0x45000 +
// 0x1000 n, pingpong 0x71000 + 0x800 n), and the apps SMMU's stream entries for the display's stream 0x820 (mask 0x402,
// qcom,smmu_sde_unsec, iommu-earlymap): whether the pipes fetch physical addresses or the boot loader's IOVAs. Called once.
extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern "C" void pdx203_dpu_dump(void) {
    static bool done;
    if (done) return;
    done = true;
    volatile uint32_t *mdss = reinterpret_cast<volatile uint32_t *>(ml_io_map(0xae00000, 0x80000));
    if (!mdss) {printf("DPU map failed\n"); return;}
    auto rd = [&](uint32_t off) {return mdss[off / 4];};
    printf("DPU hw %08x mdp-top %08x | gop %llx bytes %llx\n", rd(0x0), rd(0x1000), (unsigned long long)native_legacy_gop_physical,
           (unsigned long long)native_legacy_gop_bytes);
    static const uint32_t pipes[8] = {0x5000, 0x7000, 0x9000, 0xb000, 0x25000, 0x27000, 0x29000, 0x2b000};
    static const char *const names[8] = {"vig0", "vig1", "vig2", "vig3", "dma0", "dma1", "dma2", "dma3"};
    for (unsigned i = 0; i < 8; ++i) {
        const uint32_t b = pipes[i];
        if (!rd(b + 0x14) && !rd(b + 0x00)) continue;
        printf("DPU %s src %08x xy %08x out %08x oxy %08x addr %08x %08x stride %08x %08x fmt %08x unpack %08x op %08x\n", names[i],
               rd(b + 0x00), rd(b + 0x08), rd(b + 0x0c), rd(b + 0x10), rd(b + 0x14), rd(b + 0x18), rd(b + 0x24), rd(b + 0x28),
               rd(b + 0x30), rd(b + 0x34), rd(b + 0x38));
    }
    for (unsigned i = 0; i < 6; ++i) {
        const uint32_t b = 0x2000 + 0x200 * i;
        const uint32_t top = rd(b + 0x14), intf = rd(b + 0xf4);
        if (!top && !intf && !rd(b + 0x00) && !rd(b + 0x04)) continue;
        printf("DPU ctl%u layer %08x %08x %08x %08x %08x %08x top %08x flush %08x start %08x intf %08x ext %08x %08x\n", i, rd(b + 0x00),
               rd(b + 0x04), rd(b + 0x08), rd(b + 0x0c), rd(b + 0x10), rd(b + 0x24), top, rd(b + 0x18), rd(b + 0x1c), intf, rd(b + 0x40),
               rd(b + 0x44));
    }
    for (unsigned i = 0; i < 2; ++i) {
        const uint32_t b = 0x45000 + 0x1000 * i;
        printf("DPU lm%u op %08x out %08x bg %08x\n", i, rd(b + 0x00), rd(b + 0x04), rd(b + 0x08));
    }
    for (unsigned i = 0; i < 2; ++i) {
        const uint32_t b = 0x71000 + 0x800 * i;
        printf("DPU pp%u te %08x vsync %08x height %08x wr %08x init %08x thresh %08x start %08x line %08x autorefresh %08x\n", i,
               rd(b + 0x00), rd(b + 0x04), rd(b + 0x08), rd(b + 0x0c), rd(b + 0x10), rd(b + 0x18), rd(b + 0x1c), rd(b + 0x2c), rd(b + 0x30));
    }
    volatile uint32_t *smmu = reinterpret_cast<volatile uint32_t *>(ml_io_map(0x15000000, 0x1000));
    if (!smmu) {printf("DPU smmu map failed\n"); return;}
    const uint32_t idr0 = smmu[0x20 / 4], idr1 = smmu[0x24 / 4], cr0 = smmu[0];
    const unsigned smrs = idr0 & 0xff;
    printf("DPU smmu cr0 %08x idr0 %08x idr1 %08x smrs %u\n", cr0, idr0, idr1, smrs);
    const uint32_t page = (idr1 >> 31) ? 0x10000 : 0x1000, numpage = 1u << (((idr1 >> 28) & 7) + 1);
    for (unsigned n = 0; n < smrs && n < 128; ++n) {
        const uint32_t smr = smmu[(0x800 + 4 * n) / 4], s2cr = smmu[(0xc00 + 4 * n) / 4];
        if (!(smr >> 31)) continue;
        const uint32_t id = smr & 0xffff, mask = (smr >> 16) & 0x7fff;
        if (((id ^ 0x820) & ~(mask | 0x402) & 0x7fff)) continue;
        printf("DPU smr %u id %04x mask %04x s2cr %08x (type %u cb %u)\n", n, id, mask, s2cr, (s2cr >> 16) & 3, s2cr & 0xff);
        if (((s2cr >> 16) & 3) == 0) {
            volatile uint32_t *cb = reinterpret_cast<volatile uint32_t *>(ml_io_map(0x15000000 + numpage * page + (s2cr & 0xff) * page, 0x1000));
            if (cb) printf("DPU cb%u sctlr %08x ttbr0 %08x%08x tcr %08x\n", s2cr & 0xff, cb[0], cb[0x24 / 4], cb[0x20 / 4], cb[0x30 / 4]);
        }
    }
}

// pdx203: the page flip (pdx203-flip=1). The boot loader's display engine as the DPU dump read it (usb-40): VIG 0 and VIG 1 each fetch
// half of the 1096x2560 ARGB8888 splash at 0x9c000000 (stride 4384, VIG 1 through its source x 548), CTL 0 drives both (flush bits 0 and
// 1), pingpong 0 refreshes the command-mode panel by itself (autorefresh every frame) and the display's SMMU context has translation off,
// so the pipes fetch physical addresses. Three buffers in the boot loader's 35 MiB "Display Reserved" region (0x9c000000, the boot one
// first): a frame is stored into one neither shown nor waiting to latch, then both pipes' source address moves to it and CTL 0 is
// flushed; the hardware latches the new address at a frame start, so a frame is never shown while it is written. Before the next flip the
// previous one must have latched (its flush bits clear). native_legacy_flip_stats: [0] state (0 off, 1 on, else why it is off), [1]
// flips, [2] flips that waited for the previous latch, [3] latch waits that timed out, [4] longest wait (us), [5] buffer shown.
volatile uint32_t native_legacy_flip_stats[6];
namespace {
constexpr uint64_t kReservedBase = 0x9c000000ull, kReservedBytes = 0x2300000ull;
constexpr uint32_t kVig0 = 0x5000, kVig1 = 0x7000, kCtl0Flush = 0x2018, kStride = 4384, kFormat = 0x000237ff;
volatile uint32_t *gFlipMdss;
uint64_t gFlipPhys[3];
volatile void *gFlipVa[3];
uint64_t gFlipBytes;
int gFlipState = -1, gFlipShown = 0, gFlipPending = -1, gFlipBack = -1;
// The USB control channel's SCREENSHOT (drivers/QcomXnuUsbDebug): the buffer the panel shows (the flip's shown one, else the GOP
// buffer), BGRA read with aligned 32-bit loads (write-combined device memory), downscaled by pdx203-shot-scale (default 2, 1..4) so
// the 192-byte SHOTNEXT chunks move it in seconds. Called on the USB worker thread: no locks, no logging; a flip latched meanwhile
// can tear the picture, which only matters for a screenshot.
extern "C" int qcom_usb_debug_register_screenshot(int (*)(void *, uint8_t *, size_t, uint32_t *, uint32_t *), void *) __attribute__((weak_import));
int shotCapture(void *, uint8_t *rgb, size_t capacity, uint32_t *width, uint32_t *height) {
    PE_Video video = {};
    if (PE_current_console(&video) || !video.v_width || !video.v_height || video.v_rowBytes < video.v_width * 4) return 0;
    const volatile uint8_t *base = gFlipState == 1 && gFlipShown >= 0 && gFlipShown < 3 && gFlipVa[gFlipShown]
        ? reinterpret_cast<const volatile uint8_t *>(gFlipVa[gFlipShown]) : reinterpret_cast<const volatile uint8_t *>(native_legacy_gop_virtual);
    if (!base) return 0;
    unsigned scale = 2;
    PE_parse_boot_argn("pdx203-shot-scale", &scale, sizeof scale);
    if (scale < 1 || scale > 4) scale = 2;
    const uint32_t w = (uint32_t)video.v_width / scale, h = (uint32_t)video.v_height / scale;
    if ((uint64_t)w * h * 3 > capacity) return 0;
    uint8_t *out = rgb;
    for (uint32_t y = 0; y < h; ++y) {
        const volatile uint32_t *row = reinterpret_cast<const volatile uint32_t *>(base + (uint64_t)y * scale * video.v_rowBytes);
        for (uint32_t x = 0; x < w; ++x) {
            const uint32_t p = row[x * scale];
            *out++ = (uint8_t)(p >> 16); *out++ = (uint8_t)(p >> 8); *out++ = (uint8_t)p;
        }
    }
    *width = w; *height = h;
    return 1;
}
bool flipReady() {
    if (gFlipState >= 0) return gFlipState == 1;
    static bool shotRegistered;
    if (!shotRegistered && &qcom_usb_debug_register_screenshot) {shotRegistered = true; qcom_usb_debug_register_screenshot(shotCapture, nullptr);}
    unsigned on = 0;
    PE_parse_boot_argn("pdx203-flip", &on, sizeof on);
    auto off = [](int why) {gFlipState = why; native_legacy_flip_stats[0] = (uint32_t)why; return false;};
    if (on != 1) return off(0);
    gFlipBytes = native_legacy_gop_bytes;
    if (native_legacy_gop_physical != kReservedBase || !native_legacy_gop_virtual || !gFlipBytes || (gFlipBytes & 0xfff) ||
        3 * gFlipBytes > kReservedBytes) return off(2);
    gFlipMdss = reinterpret_cast<volatile uint32_t *>(ml_io_map(0xae00000, 0x80000));
    if (!gFlipMdss) return off(3);
    volatile uint32_t *m = gFlipMdss;
    if (m[(kVig0 + 0x14) / 4] != (uint32_t)kReservedBase || m[(kVig1 + 0x14) / 4] != (uint32_t)kReservedBase ||
        (m[(kVig0 + 0x24) / 4] & 0xffff) != kStride || m[(kVig0 + 0x30) / 4] != kFormat || m[(kVig1 + 0x30) / 4] != kFormat) return off(4);
    for (unsigned i = 0; i < 3; ++i) gFlipPhys[i] = kReservedBase + i * gFlipBytes;
    gFlipVa[0] = reinterpret_cast<volatile void *>(native_legacy_gop_virtual);
    for (unsigned i = 1; i < 3; ++i) {
        IODeviceMemory *memory = IODeviceMemory::withRange(gFlipPhys[i], gFlipBytes);
        IOMemoryMap *map = memory ? memory->map(kIOMapAnywhere | kIOMapWriteCombineCache) : nullptr;
        if (memory) memory->release();
        if (!map) return off(5);
        gFlipVa[i] = reinterpret_cast<volatile void *>(map->getVirtualAddress());   // kept for the life of the system
    }
    gFlipState = 1;
    native_legacy_flip_stats[0] = 1;
    // pdx203-hwte=1: frames go to the command-mode panel on its own tear-effect signal (GPIO 66, function mdp_vsync, TE source GPIO 0)
    // instead of pingpong 0's free-running counter that the UEFI shim enabled (PlatformSec.c: PP_VSYNC_COUNTER_ENABLE | 123, no external
    // TE), which writes the panel's memory out of step with its scan and tears whatever buffer is shown. Linux dpu_hw_pingpong
    // connect_external_te: PP_SYNC_CONFIG_VSYNC bit 20; dpu_hw_top setup_vsync_sel: MDP_VSYNC_SEL (MDP top 0x414) bits 15:12 for
    // pingpong 0. If the signal never comes the autorefresh stops and a flip's latch times out: native_legacy_flip_commit then clears
    // bit 20 again (native_legacy_flip_stats[0] = 6).
    unsigned hwte = 0;
    PE_parse_boot_argn("pdx203-hwte", &hwte, sizeof hwte);
    if (hwte == 1) {
        volatile uint32_t *gpio = reinterpret_cast<volatile uint32_t *>(ml_io_map(0xf942000, 0x1000));   // north tile 0xf900000, gpio 66
        const uint32_t gpioBefore = gpio ? gpio[0] : 0xffffffffu, selBefore = m[0x1414 / 4], ppBefore = m[0x71004 / 4];
        if (gpio && ((gpioBefore >> 2) & 0xf) != 1) gpio[0] = (gpioBefore & ~(0xfu << 2)) | (1u << 2);
        m[0x1414 / 4] = selBefore & ~(0xfu << 12);
        __asm__ volatile("dsb sy" ::: "memory");
        m[0x71004 / 4] = ppBefore | (1u << 20);
        __asm__ volatile("dsb sy" ::: "memory");
        printf("DPU hwte gpio66 %08x>%08x vsync-sel %08x>%08x pp0 sync %08x>%08x\n", gpioBefore, gpio ? gpio[0] : 0, selBefore, m[0x1414 / 4],
               ppBefore, m[0x71004 / 4]);
        native_legacy_flip_stats[0] = 2;   // on, with the panel's TE
    }
    return true;
}
}  // namespace
extern "C" uint32_t pdx203_gpu_wait_idle(uint32_t timeout_us) __attribute__((weak_import));
extern "C" volatile void *native_legacy_flip_target(uint64_t *bytes) {
    // the frame about to be copied must be finished: wait for the render server's GPU work (package/GpuProbe.cpp), at most 50 ms
    if (&pdx203_gpu_wait_idle) pdx203_gpu_wait_idle(50000);
    if (!flipReady()) return nullptr;
    for (int i = 0; i < 3; ++i)
        if (i != gFlipShown && i != gFlipPending) {gFlipBack = i; break;}
    if (bytes) *bytes = gFlipBytes;
    return gFlipVa[gFlipBack];
}
extern "C" void native_legacy_flip_commit(void) {
    if (gFlipState != 1 || gFlipBack < 0) return;   // gFlipState stays 1 with or without the TE
    volatile uint32_t *m = gFlipMdss;
    if (gFlipPending >= 0) {
        // the previous flip latches at a frame start (16.7 ms at most); wait up to 40 ms for its flush bits to clear
        uint64_t start = mach_absolute_time(), ns = 0;
        bool waited = false;
        while ((m[kCtl0Flush / 4] & 0x3) && ns < 40000000ull) {
            waited = true;
            IODelay(100);
            absolutetime_to_nanoseconds(mach_absolute_time() - start, &ns);
        }
        if (waited) native_legacy_flip_stats[2] = native_legacy_flip_stats[2] + 1;
        if (m[kCtl0Flush / 4] & 0x3) {
            native_legacy_flip_stats[3] = native_legacy_flip_stats[3] + 1;
            if (native_legacy_flip_stats[0] == 2) {   // no TE arrives: back to pingpong 0's own counter
                m[0x71004 / 4] = m[0x71004 / 4] & ~(1u << 20);
                __asm__ volatile("dsb sy" ::: "memory");
                native_legacy_flip_stats[0] = 6;
                printf("DPU hwte reverted: the flip did not latch in 40 ms\n");
            }
        }
        if (ns / 1000 > native_legacy_flip_stats[4]) native_legacy_flip_stats[4] = (uint32_t)(ns / 1000);
        gFlipShown = gFlipPending;
        gFlipPending = -1;
    }
    const uint32_t address = (uint32_t)gFlipPhys[gFlipBack];
    m[(kVig0 + 0x14) / 4] = address;
    m[(kVig1 + 0x14) / 4] = address;
    __asm__ volatile("dsb sy" ::: "memory");
    m[kCtl0Flush / 4] = 0x3;
    __asm__ volatile("dsb sy" ::: "memory");
    gFlipPending = gFlipBack;
    gFlipBack = -1;
    native_legacy_flip_stats[1] = native_legacy_flip_stats[1] + 1;
    native_legacy_flip_stats[5] = (uint32_t)gFlipShown;
}
// The GPU hand-off through the flip (sheng-fb-gpu=1): the display picks the back buffer before it offers the frame (no CPU wait, the
// GPU's copy runs after the frame's own work on the same GPU), the presenter tells the GPU client which buffer to copy into and exposes
// all three buffers as its scanout, and the display commits the flip when the client reports the copy done.
extern "C" int native_legacy_flip_pick(void) {
    if (!flipReady()) return -1;
    for (int i = 0; i < 3; ++i)
        if (i != gFlipShown && i != gFlipPending) {gFlipBack = i; break;}
    return gFlipBack;
}
extern "C" int native_legacy_flip_back_index(void) {return gFlipState == 1 ? gFlipBack : -1;}
extern "C" IOMemoryDescriptor *native_legacy_flip_memory(void) {
    static IOMemoryDescriptor *whole;
    if (!flipReady()) return nullptr;
    if (!whole) whole = IODeviceMemory::withRange(kReservedBase, 3 * gFlipBytes);   // kept for the life of the system
    return whole;
}
