#!/usr/bin/env python3
"""Build the pdx203 GOP legacy display archive (build/native-legacy-display/libNativeLegacyDisplay.late.a).

UniXNU-sheng scripts/build-native-legacy-display.py at 978e3f7 (2026-10-22, the revision that matches the unixnu fork's
drivers/NativeMobileDisplay), cut to the display: the generated ABI of the original IOMobileFramebufferLegacy, the owned
NativeLegacyDisplay bound to it, the board probe (package/LegacyDisplayProbe.cpp, the late probe without sheng's board
probes) and NativeGOPPresenter. The input side is this board's: the HID start-up (package/HidInputProbe.cpp, alioth's),
the sec_ts touch probe (package/Pdx203TouchProbe.cpp) and NativeHidInput with its original bindings
(scripts/build-native-hid-input.py), and the AppleKeyStore stand-in (package/KeyStoreProbe.cpp,
drivers/NativeKeyStoreEvents). sheng's boot log/reset/trace, cpufreq, gpu power, wifi, buttons and
backlight objects are not built. build-xnu-4k-baseline.sh links the archive with CONFIG_XNU_NATIVE_LEGACY_DISPLAY and
checks it against build-report.json.
"""
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'build/native-legacy-display';OUT.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(ROOT/'kits/XnuPrelink'))
from native_prelink import macho,backed_offset,exports
def run(cmd,**kw):return subprocess.check_output([str(x) for x in cmd],timeout=90,**kw)
def require(v,why):
    if not v:raise SystemExit(why)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
# The fork's changes to the NativeMobileDisplay module ride as patches (patches/modules/NativeMobileDisplay/*.patch, in name order) rather
# than as commits of the submodule, as NativeMetal's do (UniXNU scripts/build-adreno-metal-plugin.py). They are applied as a stack, in
# order: a later patch may change lines an earlier one added (then the earlier one no longer reverses alone), so the applied part is the
# longest prefix whose last patch reverses; the rest is applied. One that applies neither way stops the build.
module_patches=sorted((ROOT/'patches/modules/NativeMobileDisplay').glob('*.patch'))
module=(ROOT/'drivers/GopFramebuffer').resolve()
applied=0
for k in range(len(module_patches),0,-1):
    if subprocess.run(['git','-C',module,'apply','--reverse','--check',module_patches[k-1].resolve()],capture_output=True).returncode==0:
        applied=k;break
for module_patch in module_patches[applied:]:
    subprocess.run(['git','-C',module,'apply',module_patch.resolve()],check=True)
    print('applied',module_patch.name,'to drivers/NativeMobileDisplay')
# the audit accepts the original class identity (IOMobileFramebufferLegacy, parent 0xfffffff006b31400, 0x18a0 bytes) from a live
# qemu observation of the unchanged kernelcache, or, when that observation was pruned, from its recorded hash in an earlier
# output. a fresh tree has neither, so the output starts as that recorded hash alone, as the sheng and uke trees carry it
abi_record=ROOT/'build/display-analysis/mobile-graphics-legacy-abi.json'
if not abi_record.exists():
    abi_record.parent.mkdir(parents=True,exist_ok=True)
    abi_record.write_text(json.dumps(dict(runtime_metadata_checkpoint_sha256='3044c2beff552911605994c0dc7aa73182066cbb5aed0b396999ceeb5a96da2b'))+'\n')
print(run(['python3',ROOT/'scripts/audit-mobile-graphics-legacy.py'],text=True),end='')
audit=json.loads((ROOT/'build/display-analysis/mobile-graphics-legacy-abi.json').read_text())
pure=audit['abstract_slots'];methods=audit['methods']
header='''// SPDX-License-Identifier: BSD-2-Clause-Patent
// Opaque ABI storage for the exact original class, not source-level access
// to its private fields. The actual original metadata supplies its full
// Framebuffer/Service/IOService ancestry. Only qualified base prototypes may
// be called from source; opaque entries are original dispatch targets only.
#pragma once
#include <IOKit/IOService.h>
#include <libkern/libkern.h>
struct OriginalLegacySwapRequest;
class IOMobileFramebufferLegacy : public IOService {
 OSDeclareAbstractStructors(IOMobileFramebufferLegacy)
 unsigned char originalPrivate[0x18a0-sizeof(IOService)];
public:
 // Both fields are qualified by the complete original slot-249 getter.
 // This modifies this object's own mode state, never a vtable or executable.
 void installBootDimensions(uint32_t width,uint32_t height) {
  *reinterpret_cast<uint32_t *>(reinterpret_cast<unsigned char *>(this)+5808)=width;
  *reinterpret_cast<uint32_t *>(reinterpret_cast<unsigned char *>(this)+5812)=height;
 }
 const OriginalLegacySwapRequest *pendingSwapRequest() const {
  const OriginalLegacySwapRequest *request=nullptr;
  memcpy(&request,reinterpret_cast<const unsigned char *>(this)+5480,sizeof(request));return request;
 }
 // The real normal scanout backend removes its pending head before calling
 // slot 295. Slot 312 is a discard path (terminal state 8), not presentation.
 // The queue is a tail queue (head at +5480, tail pointer at +5488, each request's next at +2960 and previous-link at +2968): taking the head
 // moves the next request, if a client already queued one behind it (int-132: swap 3 arrived during swap 2's GPU copy and the old
 // single-entry form refused to retire swap 2, so every wait on it stayed busy), up to the head. The original queue's reference
 // moves to the current-swap slot for the original completion body.
 bool activateOnlyPendingSwap(const OriginalLegacySwapRequest *request) {
  auto self=reinterpret_cast<unsigned char *>(this);
  auto bytes=reinterpret_cast<const unsigned char *>(request);
  const void *current=nullptr,*head=nullptr,*tail=nullptr,*next=nullptr,*previous=nullptr;
  memcpy(&current,self+5472,8);memcpy(&head,self+5480,8);memcpy(&tail,self+5488,8);
  if(!request || current || head!=request)return false;
  memcpy(&next,bytes+2960,8);memcpy(&previous,bytes+2968,8);
  if(previous!=self+5480)return false;
  if(next) {
   const void *link=self+5480;
   memcpy(const_cast<unsigned char *>(reinterpret_cast<const unsigned char *>(next))+2968,&link,8);
  } else if(tail!=bytes+2960)return false;
  memcpy(self+5472,&request,8);
  if(next) {memcpy(self+5480,&next,8);}
  else {const void *empty=nullptr,*emptyTail=self+5480;memcpy(self+5480,&empty,8);memcpy(self+5488,&emptyTail,8);}
  return true;
 }
 void free() override;
 bool init(OSDictionary *properties=nullptr) override;
 IOReturn setProperties(OSObject *) override;
 void systemWillShutdown(IOOptionBits) override;
 IOService *probe(IOService *,SInt32 *) override;
 bool start(IOService *) override;
 void stop(IOService *) override;
 IOWorkLoop *getWorkLoop() const override;
 IOReturn newUserClient(task_t,void *,UInt32,IOUserClient **) override;
 IOReturn setPowerState(unsigned long,IOService *) override;
'''
for i in range(168,358):
    if i==184:header+=' virtual bool isMainDisplay();\n'
    elif i==188:header+=' virtual IOReturn getDotPitch(uint32_t *);\n'
    elif i==213:header+=' virtual IOReturn supportedFrameInfo(uint32_t,uint32_t *,char *,uint32_t *)=0;\n'
    elif i==249:header+=' virtual void getDisplayDimensions(uint32_t *,uint32_t *);\n'
    elif i==290:header+=' virtual void kickSwapQueue();\n'
    elif i==296:header+=' virtual bool validateSwap(const OriginalLegacySwapRequest *);\n'
    elif i==295:header+=' virtual void completeCurrentSwap();\n'
    elif i==337:header+=' virtual IOReturn collectCompletionReports(uint32_t,void *,uint32_t *,uint32_t)=0;\n'
    elif i==288:header+=' virtual bool startHardware();\n'
    elif i==301:header+=' virtual bool getRefreshAttributes(uint64_t *,bool);\n'
    # Slot 345 (the family's swap timeout) stays a guard leaf with the upstream GopFramebuffer pinned in drivers/; the fork's
    # cancelSwapsAfterTimeout() (its NativeMobileDisplay 0002 patch) returns with the GPU path.
    elif i in pure:header+=f' virtual __attribute__((noreturn)) void opaque{i}()=0;\n'
    else:header+=f' virtual __attribute__((deprecated("Unclassified ABI: never call from source"))) void opaque{i}();\n'
header+='};\nstatic_assert(sizeof(IOMobileFramebufferLegacy)==0x18a0,"Actual original instance extent");\n'
(OUT/'OriginalLegacyFramebufferABI.h').write_text(header)
print(run(['python3',ROOT/'scripts/build-arm-driver.py','NativeLegacyDisplay'],text=True),end='')
obj=OUT/'NativeLegacyDisplay.o';raw=obj.read_bytes();image=macho(raw)
st=next(p for c,p,_ in image['commands'] if c==2);sy,count,strings,_=struct.unpack_from('<4I',raw,st+8);symbols=[]
for i in range(count):
    ix,kind,section,_,va=struct.unpack_from('<IBBHQ',raw,sy+i*16)
    symbols.append((raw[strings+ix:raw.index(b'\0',strings+ix)].decode(),va,kind))
vtable=next(va for name,va,_ in symbols if name=='__ZTV19NativeLegacyDisplay');relocations={}
for segment in image['segments']:
    for sec in segment['sections']:
        if sec['va']<=vtable<sec['va']+sec['size']:
            ro,rc=struct.unpack_from('<2I',raw,sec['command_offset']+56)
            for i in range(rc):
                offset,bits=struct.unpack_from('<2I',raw,ro+i*8)
                if bits>>27&1 and bits>>28==0 and bits>>25&3==3:relocations[sec['va']+offset]=symbols[bits&0xffffff][0]
compiled=[relocations.get(vtable+16+i*8) for i in range(358)]
require(all(compiled) and vtable+16+358*8 not in relocations,'Exact owned 358-entry vtable')
candidate=ROOT/'build/native-prelink/native-owner-mobile-graphics-candidate.macho'
if not candidate.exists():  # pruned: same fallback as scripts/audit-mobile-graphics-legacy.py (identical original family executable)
    later=sorted((ROOT/'build/native-prelink').glob('native-owner-ufs-cpu-desktop-*-candidate.macho'),key=lambda f:f.stat().st_mtime)
    require(later,'No reference candidate available');candidate=later[-1]
old=candidate.read_bytes();old_image=macho(old)
names={n:v for v,group in exports(old,old_image).items() for n in group}
def read(va,n):
    at=backed_offset(old_image['segments'],va,n,len(old));return old[at:at+n]
bindings={'__ZN25IOMobileFramebufferLegacyC2EPK11OSMetaClass':0xfffffff0057865cc,
          '__ZN25IOMobileFramebufferLegacyD2Ev':0xfffffff005786618,
          '__ZN25IOMobileFramebufferLegacy10gMetaClassE':0xfffffff006b35940,
          '__ZTV25IOMobileFramebufferLegacy':int(audit['legacy_vptr'],16)-16}
owned=[];native=[]
for i,name in enumerate(compiled):
    expected=int(methods[i]['address'],16)
    if name.startswith(('__ZN19NativeLegacyDisplay','__ZNK19NativeLegacyDisplay')):
        owned.append(i);require(i in (0,1,7,14,84,86,87,141,155,184,188,288,290,296,301) or i in pure,'Only qualified own methods and guarded leaves override')
    elif '25IOMobileFramebufferLegacy' in name:
        require(i not in pure,'No unresolved original abstract backend target');bindings[name]=expected
    else:
        require(i<168 and names.get(name)==expected,'Actual inherited native base symbol '+str(i));native.append(i)
require(owned==sorted([0,1,7,14,84,86,87,141,155,184,188,288,290,296,301]+pure),'Exact owned method set')
undefined={n for n,_,k in symbols if k==1 and 'IOMobileFramebufferLegacy' in n}
require(undefined==set(bindings),'All and only actual original-class imports qualified')
# Exact original IOSurface getters, independently observed through the real
# original swap request. These aliases call original code; no object layout
# is substituted and no original method/vtable is patched.
for name,address,field,wide in [
    ('descriptor',0xfffffff004b9ead0,48,True),('width',0xfffffff004b9ebbc,88,True),
    ('height',0xfffffff004b9ebc4,96,True),('stride',0xfffffff004b9ebd4,104,True),
    ('allocation',0xfffffff004b9ebfc,144,True),('format',0xfffffff004b9ebac,128,False),
    ('planes',0xfffffff004b9ebf4,152,False)]:
    expected=(0xf9400000 if wide else 0xb9400000)|((field//(8 if wide else 4))<<10)
    require(read(address,8)==struct.pack('<2I',expected,0xd65f03c0),'Exact original surface getter '+name)
    # the stride field can hold pixels (alioth GPUREJ), so the board wraps the raw getter (package/LegacyDisplayProbe.cpp) instead of
    # binding the name the driver calls
    bindings['_native_legacy_surface_stride_raw' if name=='stride' else '_native_legacy_surface_'+name]=address
# The original display-refresh notification sender (self, timestamp, continuous host time, frame interval): it sends the six-word
# vsync message (type 5) to every client entry queued at +5592, the tail queue that selector 72's registration (slot 208) fills at
# +5512 + type*16. The platform subclass that normally calls it on each panel vblank is not part of this port.
require(read(0xfffffff005796720,16)==bytes.fromhex('ff0302d1fd7b07a9fdc30191a0831ff8') and
        read(0xfffffff005796830,4)==bytes.fromhex('08ed4af9') and
        read(0xfffffff00579685c,16)==bytes.fromhex('e2230091c3008052240080521684ff97') and
        read(0xfffffff005795fbc,4)==bytes.fromhex('0ab182d2'),
        'Original vsync sender, its +5592 queue and six-word drop-allowed async send')
bindings['_native_legacy_vsync_notify']=0xfffffff005796720
# Slot 301 fills the six refresh words that setDisplayRefreshProperties publishes as IOMFBDisplayRefresh (minimum interval, its mach
# time, maximum interval, its mach time, step, its mach time; intervals in 32.32 seconds): the generic body fixes the minimum at
# 0x04444444 (1/60 s), and the render server paces its display timer by it.
require(int(methods[301]['address'],16)==0xfffffff00579ea5c and read(0xfffffff00579ea68,8)==bytes.fromhex('888888d28888a0f2') and
        read(0xfffffff00579ec54,4)==bytes.fromhex('08b544f9'),
        'Original slot-301 refresh words with the 1/60 s minimum, called by setDisplayRefreshProperties')
# setDisplayRefreshProperties itself (self): fills the six words through slot 301 and publishes them as IOMFBDisplayRefresh. Nothing in
# the family calls it (the platform subclass does), so without it the render server falls back to its own 1/60 s heartbeat.
require(read(0xfffffff00579ebb8,16)==bytes.fromhex('ff0302d1fd7b07a9fdc30191a0031ff8') and
        read(0xfffffff00579ed54,16)==bytes.fromhex('086540f90173ff90218c0c9100013fd6'),
        'Original setDisplayRefreshProperties publishing IOMFBDisplayRefresh through setProperty')
bindings['_native_legacy_set_refresh_properties']=0xfffffff00579ebb8
# Both protected destructors/constructors must follow the exact original chain.
def branch(pc):
    word=struct.unpack('<I',read(pc,4))[0];require(word&0xfc000000==0x94000000,'Actual protected base call')
    value=word&0x3ffffff;return pc+4*(value-(1<<26) if value&(1<<25) else value)
require(branch(0xfffffff0057865fc)==0xfffffff00577b510 and branch(0xfffffff005786630)==0xfffffff00577b55c,
        'Original protected Legacy C2/D2 really call their original Service bases')
# Actual caller passes only self at slot 288 and tests bit zero of w0.
require(read(0xfffffff005787ef8,16)==bytes.fromhex('080040f9088144f900013fd6e0000037'),
        'Original bool start_hardware() call and result check')
getter=int(methods[249]['address'],16)
for offset,field in [(0x30,5808),(0x58,5812)]:
    require(struct.unpack('<I',read(getter+offset,4))[0]==0xb9400000|((field//4)<<10)|(8<<5)|8,
            'Actual original width/height getter fields')
require(int(methods[295]['address'],16)==0xfffffff005795b88 and
        read(0xfffffff005795b98,16)==bytes.fromhex('e00740f9280080520101001272d3ff97'),
        'Actual original slot-295 passes true to the normal current completion body')
require(read(0xfffffff00578aa7c,24)==bytes.fromhex('080040f9084545f9030080d2e20303aa0400805200013fd6'),
        'Original completion calls optional report slot 337 with null outputs and zero limit')
require(read(0xfffffff005777e9c,16)==bytes.fromhex('080040f908e142f900013fd6c0000036'),
        'Original isMainDisplay dispatch uses only self and tests boolean bit zero')
asm=OUT/'original-bindings.S';asm.write_text(''.join(f'.globl {name}\n.set {name}, {va:#x}\n' for name,va in sorted(bindings.items())))
sdk=run(['xcrun','--sdk','iphoneos','--show-sdk-path'],text=True).strip()
run(['xcrun','clang','-target','arm64-apple-ios17.0','-isysroot',sdk,'-c',asm,'-o',OUT/'original-bindings.o'])
llvm=Path(run(['brew','--prefix','llvm'],text=True).strip())/'bin';late=OUT/'NativeLegacyDisplay.late.o'
run([llvm/'llvm-objcopy','--remove-section=__DATA,__mod_init_func','--globalize-symbol=__GLOBAL__sub_I_NativeLegacyDisplay.cpp',obj,late])
def code(data,info):
    sec=next(s for g in info['segments'] for s in g['sections'] if s['name']=='__text');return data[sec['fileoff']:sec['fileoff']+sec['size']]
require(code(raw,image)==code(late.read_bytes(),macho(late.read_bytes())),'Only owned initializer section registration changes')
r=json.loads((ROOT/'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/iokit/DEVELOPMENT/IOPlatformExpert.cpo.json').read_text().strip().rstrip(','))
def board_cpp(name,extra=()):
    """compile a board C++ source of kits/pdx203 with the kernel's IOKit C++ line into OUT/<stem>.o"""
    cmd=[];args=r['arguments'];i=0;src=ROOT/'kits/pdx203'/name;obj=OUT/(Path(name).stem+'.o')
    while i<len(args):
        value=args[i]
        if value==r['file']:cmd.append(str(src))
        elif value=='-o':cmd+=['-o',str(obj)];i+=1
        elif value=='-Xclang' and args[i+1].startswith('-W'):i+=1
        elif not value.startswith('-W'):cmd.append(value)
        i+=1
    cmd+=['-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-unnecessary-virtual-specifier','-Wno-c99-designator','-Wno-reorder-init-list',
          '-Wno-deprecated-enum-enum-conversion','-Wno-missing-designated-field-initializers','-I',str(ROOT/'.cache/xnu-work/osfmk'),*extra]
    run(cmd,cwd=r['directory'])
    return src,obj
source,probe=board_cpp('LegacyDisplayProbe.cpp')
# the input side, started from the late probe: alioth's HID start-up (offline-hid-input=1) and the sec_ts touch probe
# (pdx203-touch=1). NativeHidInput is a class nothing references by symbol, so its late object has to sit in this force-loaded
# archive with them (alioth boot-usb70: in a plain archive the linker dropped it and no IOHIDDevice existed).
hid_src,hid_probe=board_cpp('HidInputProbe.cpp')
touch_src,touch_probe=board_cpp('Pdx203TouchProbe.cpp')
print(run(['python3',ROOT/'scripts/build-native-hid-input.py'],text=True),end='')
hid=ROOT/'build/sheng-hid-input'
# the AppleKeyStore stand-in, as sheng builds it: an owned IOService (no original ABI to bind) whose static initialiser is
# globalised so KeyStoreProbe.cpp can register the class itself
keystore_src,keystore_probe=board_cpp('KeyStoreProbe.cpp')
cpufreq_src,cpufreq_probe=board_cpp('CpuFreqProbe.cpp')
# the power, volume and camera keys over SPMI (pdx203-bt=<mode>), started from the late probe
buttons_src,buttons_probe=board_cpp('ButtonProbe.cpp')
def board_c(name,json_rel):
    """compile a board C source of kits/pdx203 with the kernel C line of json_rel into OUT/<stem>.o"""
    j=json.loads((ROOT/'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837'/json_rel).read_text().strip().rstrip(','))
    src=ROOT/'kits/pdx203'/name;obj=OUT/(Path(name).stem+'.o');cmd=[];args=j['arguments'];i=0
    while i<len(args):
        value=args[i]
        if value==j['file']:cmd.append(str(src))
        elif value=='-o':cmd+=['-o',str(obj)];i+=1
        elif value=='-Xclang' and args[i+1].startswith('-W'):i+=1
        elif not value.startswith('-W'):cmd.append(value)
        i+=1
    cmd+=['-Wall','-Wextra','-Werror','-Wno-unused-parameter','-mgeneral-regs-only','-I',str(ROOT/'.cache/xnu-work/osfmk')]
    run(cmd,cwd=j['directory'])
    return src,obj
# the GENI I2C core the touch probe drives (with the kernel's osfmk C line). The audio DSP, battery, RTC and Wi-Fi objects of the
# full pdx203 image return one at a time once the minimal format 2 image is stable.
board_c_objs=[board_c('GeniI2c.c','osfmk/DEVELOPMENT/startup.o.json')]
print(run(['python3',ROOT/'scripts/build-arm-driver.py','AppleKeyStoreState'],text=True),end='')
keyevents=ROOT/'build/apple-key-store-state';keyevents_late=keyevents/'NativeKeyStoreEvents.late.o'
run([llvm/'llvm-objcopy','--remove-section=__DATA,__mod_init_func','--globalize-symbol=__GLOBAL__sub_I_NativeKeyStoreEvents.cpp',
     keyevents/'NativeKeyStoreEvents.o',keyevents_late])
require(code((keyevents/'NativeKeyStoreEvents.o').read_bytes(),macho((keyevents/'NativeKeyStoreEvents.o').read_bytes()))==
        code(keyevents_late.read_bytes(),macho(keyevents_late.read_bytes())),'Owned keystore instructions unchanged by late init staging')
archive=OUT/'libNativeLegacyDisplay.late.a';archive.unlink(missing_ok=True)
run(['xcrun','ar','rcs',archive,late,OUT/'original-bindings.o',probe,hid_probe,touch_probe,hid/'NativeHidInput.late.o',hid/'hid-original-bindings.o',
     keystore_probe,keyevents_late,cpufreq_probe,buttons_probe,*[o for _,o in board_c_objs]])
sources=[ROOT/'drivers/GopFramebuffer/NativeLegacyDisplay.cpp',ROOT/'drivers/GopFramebuffer/NativeLegacyRequest.h',source,
         hid_src,touch_src,ROOT/'drivers/ShengHidInput/NativeHidInput.cpp',keystore_src,ROOT/'drivers/AppleKeyStoreState/NativeKeyStoreEvents.cpp',cpufreq_src,buttons_src,
         ROOT/'kits/pdx203/ButtonsCore.h',*[c for c,_ in board_c_objs]]
print(run(['python3',ROOT/'scripts/build-arm-driver.py','NativeGOPPresenter'],text=True),end='')
gop=ROOT/'build/native-gop-presenter';gop_late=gop/'NativeGOPPresenter.late.o'
run([llvm/'llvm-objcopy','--remove-section=__DATA,__mod_init_func','--globalize-symbol=__GLOBAL__sub_I_NativeGOPPresenter.cpp',
     gop/'NativeGOPPresenter.o',gop_late])
require(code((gop/'NativeGOPPresenter.o').read_bytes(),macho((gop/'NativeGOPPresenter.o').read_bytes()))==
        code(gop_late.read_bytes(),macho(gop_late.read_bytes())),'Owned presenter instructions unchanged by late init staging')
run(['xcrun','ar','rcs',archive,gop_late])
sources.extend([ROOT/'drivers/GopFramebuffer/NativeGOPPresenter.cpp',ROOT/'drivers/GopFramebuffer/NativeGOPWire.h'])
(OUT/'build-report.json').write_text(json.dumps(dict(
    source_sha256={str(p.relative_to(ROOT)):sha(p) for p in sources},generated_abi_sha256=sha(OUT/'OriginalLegacyFramebufferABI.h'),
    original_candidate_sha256=sha(candidate),original_audit_sha256=sha(ROOT/'build/display-analysis/mobile-graphics-legacy-abi.json'),
    compiled_object_sha256=sha(obj),late_object_sha256=sha(late),archive_sha256=sha(archive),
    bindings={n:hex(v) for n,v in bindings.items()},virtual_entries=358,owned_entries=owned,inherited_native_entries=native,
    guarded_backend_leaves=[i for i in pure if i not in (213,337)],original_instructions_modified=False,driver_initialized=False,display_presented=False,desktop=False),indent=2)+'\n')
print('pdx203: built the GOP legacy display archive (NativeLegacyDisplay, its original bindings, the board probe, NativeGOPPresenter,'
      ' the HID start-up, the sec_ts touch probe, NativeHidInput, the AppleKeyStore stand-in)')
