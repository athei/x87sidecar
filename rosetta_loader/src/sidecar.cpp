#include "sidecar.hpp"

#include <dirent.h>
#include <libproc.h>
#include <mach/mach.h>
#include <mach/mach_port.h>
#include <mach/mach_vm.h>
#include <pthread.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <map>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dyld_process_info.hpp"
#include "guest_pc_map.hpp"
#include "rosetta_core/Config.h"
#include "rosetta_core/ConfigEnv.h"
#include "rosetta_core/CoreConfig.h"
#include "rosetta_core/Fixup.h"
#include "rosetta_core/IRInstr.h"
#include "rosetta_core/IRModuleData.h"
#include "rosetta_core/Opcode.h"
#include "rosetta_core/ProfileFormat.h"
#include "rosetta_core/ProfileRuntime.h"
#include "rosetta_core/ThreadContextOffsets.h"
#include "rosetta_core/TransactionalList.h"
#include "rosetta_core/TranslationResult.h"
#include "rosetta_core/Translator.h"
#include "rosetta_core/X87Cache.h"

namespace sidecar {

namespace {

// Largest message we expect to receive — header (24) + 5×8 args (40) +
// trailer + future payload. 4 KB is generous.
constexpr size_t kRecvBufferSize = 4096;

struct ThreadArgs {
    mach_port_t servicePort;
    mach_port_t parentTaskPort;
};

// Bumped once per processed translate_insn request.  Read by the reporter
// thread to log throughput so we can tell "stuck" from "just slow" while
// big workloads (e.g. WoW world-load) churn through cold translation.
std::atomic<uint64_t> g_hits{0};

// Translate-path syscall-optimization counters, read by the reporter.  All
// writers are the single receive thread; atomics only because the reporter
// thread reads them.
std::atomic<uint64_t> g_statVmSyscalls{0};  // mach_vm_* traps on the translate path
std::atomic<uint64_t> g_statTcoHits{0};     // TCO served from cache
std::atomic<uint64_t> g_statIrHits{0};      // IR array served from cache (probe read only)
std::atomic<uint64_t> g_statIrMisses{0};    // IR array read in full

// X87_PROFILE state.  Opened once at sidecar startup; closed when the
// receive thread exits (i.e. parent process death).  Block-id assignment
// and de-dup live in profile::register_block (rosetta_core) so the JIT
// counter-bump emit and this dumper agree on ids by construction.
//
// The counter array is allocated in our address space and mach_vm_remapped
// (copy=FALSE) into the parent at a parent VA — so JIT-emitted LDADDAL on
// the parent VA writes the SAME backing pages we read at exit through
// our local VA.  No mach_vm_read needed at any point; no race with
// parent's death.
struct ProfileState {
    std::FILE* file = nullptr;
    std::mutex io_mu;
    std::unordered_set<uint32_t> dumped;  // block_ids whose IR we've written
};
ProfileState g_profile;

// Defined below alongside the other parent-memory helpers; forward-declared so
// the block dumper can read the translation module's record to recover the
// block's absolute guest VA.
bool readParent(mach_port_t task, uint64_t addr, void* dst, size_t size);

void dumpBlockIfNew(mach_port_t parentTask, uint64_t module_data_ptr, uint64_t block_ptr,
                    const IRInstr* ir, uint64_t num_instrs) {
    if (g_profile.file == nullptr) {
        return;
    }
    const uint64_t ir_hash = profile::hash_ir_stream(ir, static_cast<size_t>(num_instrs));
    const uint32_t bid =
        profile::register_block(reinterpret_cast<const IRBlock*>(block_ptr), ir_hash);
    if (bid == profile::kOverflowId) {
        return;
    }

    std::scoped_lock lock(g_profile.io_mu);
    if (!g_profile.dumped.insert(bid).second) {
        return;  // already wrote this block's IR stream
    }

    // Recover the block's absolute guest x86 VA.  IRInstr::pc (== IRBlock::start_pc)
    // is only an offset within the current translation module; the module's
    // absolute base is IRModuleData::text_vmaddr_range.  Read that module record
    // from the parent here — past the profiler-off and first-seen guards above —
    // so it costs nothing when profiling is off and runs at most once per block,
    // on the cold translation path, never in steady state.  (Guest is 32-bit, so
    // base + offset fits in u32.)
    uint64_t module_base = 0;
    if (module_data_ptr != 0) {
        IRModuleData mod{};
        if (readParent(parentTask, module_data_ptr, &mod, sizeof(mod))) {
            module_base = mod.text_vmaddr_range;
        }
    }
    const uint32_t guest_va = static_cast<uint32_t>(module_base + ir[0].pc);

    profile::BlockHeader hdr{
        .block_id = bid,
        .num_instrs = static_cast<uint32_t>(num_instrs),
        .start_pc = guest_va,
        ._reserved = 0,
    };
    std::fwrite(&hdr, sizeof(hdr), 1, g_profile.file);

    // Stream full IRInstr values; analyzer feeds them straight to
    // Translator::translate_instruction so it gets real disp/index/imm
    // and emit counts match production exactly.  PC is per-run and the
    // analyzer never reads it — zero it so identical patterns hash
    // identically across captures.
    std::vector<IRInstr> stable(ir, ir + num_instrs);
    for (auto& rec : stable) {
        rec.pc = 0;
    }
    std::fwrite(stable.data(), sizeof(IRInstr), num_instrs, g_profile.file);
    std::fflush(g_profile.file);
}

// ── Cross-process marshalling helpers ───────────────────────────────────────
// Translator + its helpers grow `insn_buf` (mmap/calloc) and append to six
// fixup lists (`::operator new`). Both allocators land in *this* process, so
// we can't simply hand parent's pointers to Translator — the resulting
// pointers would be unreachable from the parent.
//
// Strategy:
//   1. Read parent's TR, ThreadContextOffsets, and IR array into locals.
//      sizeof(tr) bytes are read in full — the loader's M2 init patched
//      stock's TR allocator to allocate sizeof(TranslationResult) per TR
//      so parent's heap has the full extended struct (including our
//      appended x87_cache, OPT-1).
//   2. RESET TR's mutable buffers to empty (data=null, end=0, end_cap=0,
//      use_heap=1) and lists to nullptr. With use_heap=1 grow uses calloc
//      (no munmap of foreign pointers); with empty lists push_back_slow's
//      `delete old_begin` is `delete nullptr`, which is a no-op.
//   3. Run Translator on `tr`. Its growth/pushes allocate fresh sidecar-
//      local heap; the local TR's data/list pointers now name those.
//   4. APPEND the locally-produced bytes/fixups to parent's existing
//      buffers. If the append fits within parent's capacity we just
//      mach_vm_write the delta. If it doesn't, allocate a parent-side
//      replacement via `mach_vm_allocate`, copy parent's existing
//      contents over, then append the new tail. Update TR's pointers to
//      the parent VA.
//   5. mach_vm_write the patched TR back in full (sizeof(TranslationResult))
//      — including x87_cache so OPT-1's cross-instruction state persists.
//      Free our local allocations.
//
// Parent's old buffer (when we replace it on grow) becomes orphaned in
// parent's heap — we can't `free()` parent-side from here. The leak is
// per-grow only; capacity doubles each time so growths are logarithmic.

constexpr size_t kListCount = 6;

// ── OPT-1 cache storage (mode-independent) ──────────────────────────────────
// x87_cache is per-thread, cross-instruction Translator state. It used to be
// appended to the tracee's TranslationResult and round-tripped through the
// tracee's heap — which forced the M2 install to enlarge every TR (the TR-size
// MOVZ patch), and that patch is only safe before the first TR is allocated
// (pre-Rosetta-init). We now keep the cache HERE, keyed by the per-thread TR
// address the tracee passes in each request, so the tracee's TR stays
// stock-sized and no TR-size patch is needed. This is what lets the JIT-hook
// install work identically for default (task_for_pid+ptrace, stopped pre-init)
// and cooperative (task-port handshake, attached post-init) attach.
// Stock's real TranslationResult is 0x268 bytes — the size stock's own
// allocator MOVZ passes (offset_finder's translation_result_size_pattern; the
// AOT path patches this same MOVZ to 0x400 to make room for the appended
// x87_cache).  Our struct definition carries reverse-engineered tail fields
// past that (segments_*, field_280) up to offsetof(x87_cache)=0x288 — fields
// stock never allocates.  We read/write back exactly stock's real size: stock
// places each block's code buffer in the adjacent heap chunk (observed at
// tr+0x280), so writing even the extra bytes past 0x268 overran into and
// zeroed the block's first emitted ARM word — a 0x00000000 UDF that crashed
// WoW on nearly every fld block.  The translator only ever touches fields well
// below 0x268 (insn_buf, fixup lists, free_gpr/fpr masks, thread_context_
// offsets), so trimming to the real size loses no state.
// kStockTRSize lives in sidecar.hpp; the loader checks it against the
// immediate translator_translate hands its allocator at startup.
static_assert(kStockTRSize <= offsetof(TranslationResult, x87_cache),
              "stock TR size must not exceed our struct's pre-x87_cache prefix");
std::mutex g_x87CacheMu;
std::unordered_map<uint64_t, X87Cache> g_x87Cache;

// ThreadContextOffsets cache, keyed by the tracee-side pointer.  The struct
// describes stock's thread-context layout (process-lifetime constants that
// stock re-materialises on the calling thread's STACK, so the same pointer
// recurs per thread with the same contents).  Each distinct pointer is read
// from the tracee once; every miss cross-checks the new read against the
// first cached copy and logs loudly if the constants assumption ever breaks.
// Receive thread only, no lock.  X87_NO_TCO_CACHE restores the
// read-every-request behaviour.
std::unordered_map<uint64_t, ThreadContextOffsets> g_tcoCache;

// Per-block IR array cache, keyed by TR address (one in-flight block per TR,
// same keying rationale as g_x87Cache above).  The translator never mutates
// the tracee's IR array (TranslatorX87Fusion works on a local copy), so it is
// read-only cross-task and constant while stock walks the block.  But stock
// walks it with one request per x87 run, and re-reading the whole array
// (80 B × num_instrs) on every request was the single largest read on the
// translate path.  Reuse requires the identity triple to match, the walk to
// stay monotonic (a new translation pass restarts at a lower-or-equal index),
// and an 80-byte probe of the current IRInstr to compare equal.  Residual
// ABA needs recycled block/array pointers with a byte-identical current
// IRInstr (incl. guest pc) but different lookahead entries, i.e. guest
// self-modifying code re-decoded at the same pc: accepted, X87_NO_IR_CACHE
// is the hatch.  Entry storage is reused across requests either way, which
// also drops the old per-request vector malloc.  Receive thread only.
struct IRCacheEntry {
    uint64_t block = 0;
    uint64_t instr_array = 0;
    uint64_t num_instrs = 0;
    uint64_t next_idx = 0;  // where the previous reply told stock to continue
    uint64_t hash = 0;      // profile::hash_ir_stream of ir, when hash_valid
    bool hash_valid = false;
    std::vector<IRInstr> ir;
};
std::unordered_map<uint64_t, IRCacheEntry> g_irCache;

// X87_DIAG_DIR=<dir>: append diagnostic lines to <dir>/x87diag.<pid>.log.
// Some hosts (CrossOver respawning the game process, for one) lose the
// process's stdout entirely; a per-pid file is the one channel out of that
// process's sidecar that survives.  Opened on first use; nullptr, and every
// caller a no-op, when the knob is unset.
FILE* diagLog() {
    static FILE* f = []() -> FILE* {
        if (g_rosetta_config == nullptr || g_rosetta_config->diag_dir.empty()) {
            return nullptr;
        }
        char path[1024];
        std::snprintf(path, sizeof(path), "%s/x87diag.%d.log", g_rosetta_config->diag_dir.c_str(),
                      getpid());
        return std::fopen(path, "a");
    }();
    return f;
}

// NOTE on the road not taken: mach_vm_remap(copy=FALSE) of TRACEE-owned pages
// into the sidecar (to turn the TR / insn-buf / fixup reads and writes below
// into memcpys) does NOT work.  The tracee is an x86_64-translated task with
// 4 KB VM pages; remapping its private anonymous memory into our 16 KB arm64
// map silently degrades to copy semantics, so the two views diverge in BOTH
// directions (verified empirically 2026-07: reads return remap-time bytes,
// writes through the local view never reach the tracee).  Sharing only works
// in the other direction, for objects WE allocate and remap INTO the tracee
// (the X87_PROFILE counter array in main.cpp).  Don't re-attempt.

struct TranslateRequest {
    uint64_t tr_addr;
    uint64_t block;  // opaque IRBlock* — Translator only compares as ptr
    uint64_t instr_array;
    uint64_t num_instrs;
    uint64_t insn_idx;
};

struct TranslateOutcome {
    bool reply_some;  // true → reply Some(value), else None.
    int64_t value;
};

bool readParent(mach_port_t task, uint64_t addr, void* dst, size_t size) {
    if (size == 0) {
        return true;
    }
    mach_vm_size_t got = 0;
    kern_return_t kr =
        mach_vm_read_overwrite(task, addr, size, reinterpret_cast<mach_vm_address_t>(dst), &got);
    return kr == KERN_SUCCESS && got == size;
}

// ── Guest-pc sampler ────────────────────────────────────────────────────────
// Samples the tracee without stopping it and resolves each host ARM pc to a
// guest x86 pc through guest_pc::resolve.  Neither thread_get_state nor
// mach_vm_read suspends the target, so this costs the tracee nothing beyond
// memory-read traffic; the cost is our own CPU, which bounds the sample rate.
//
// It latches onto the thread caught executing guest code inside the configured
// guest range and then samples only that thread, which is what makes a high rate
// affordable: a sweep of every thread pays a fragment-tree walk per thread even
// for the ones parked outside translated code.  Output goes to the single file
// named by X87_SAMPLE.
// One loaded image of the target, as the profile reports it.  A guest pc means
// nothing without the map of the run it came from, so the sampler builds that
// map itself (see scanModules) rather than leaving the reader to find a matching
// wine +loaddll log.
struct GuestModule {
    uint64_t base = 0;
    uint64_t size = 0;
    bool macho = false;  // else a PE mapped by wine
    // Executable, but nothing on disk or in dyld accounts for it: JIT output.
    // `path` is then a description of the mapping rather than a file, and there
    // is nothing to symbolise against.
    bool anon = false;
    // A mapping of PART of a file, with no image header of its own: the kernel
    // names the file, but nothing can walk back to a Mach-O header and the
    // mapping's offset into the file is NOT recoverable (mach_vm_region reports
    // an offset within some VM object of Rosetta's own, measured: the runtime's
    // own image reports 0x0, 0x1000, 0x2000 for regions at +0x0, +0xd000,
    // +0xe000).  So the file names the row and nothing may be symbolised against
    // it.  Rosetta maps a 16 KB slice of its runtime next to every guest image
    // this way, holding the guest-syscall dispatcher a blocked thread parks in.
    bool slice = false;
    // Cleared when a later scan no longer finds the image.  Kept rather than
    // dropped: samples taken while it was mapped still need its range.
    bool loaded = true;
    std::string path;  // on disk, and may contain spaces, so it goes last
};

// How often the module map is rebuilt while the target runs.  Flat, and short.
// A backoff was tried and is wrong for this target: the game keeps loading for
// far longer than it takes to start — wow_turbo.dll, the addons and wined3d all
// arrive well after ten seconds — so an interval that grows starves exactly the
// window that matters.  Measured on the client: a 14 s run got two scans and
// missed a third of its samples' modules.
constexpr double kScanEveryS = 5.0;

// The walk pauses this long every this many regions.  See scanModules: the cost
// that matters is not our CPU, it is the target's vm_map lock.
constexpr uint64_t kScanYieldEvery = 128;
constexpr long kScanYieldNs = 1000000;  // 1 ms

// Counters for the profile header.  The scanner thread writes them, the sampler
// thread reads them, and they are the ONLY thing the two share: the map itself
// travels as a file.
struct ModuleMapStats {
    std::atomic<uint64_t> learned{0};    // images the map gained
    std::atomic<uint64_t> searches{0};   // header searches, the expensive path
    std::atomic<uint64_t> dyld_asks{0};  // times dyld's image list was consulted
    std::atomic<uint64_t> scan_us{0};    // time spent reading the target for both
};
ModuleMapStats g_modmap;

struct SamplerCtx {
    // True once the guest range means something: either it was pinned, or the
    // main image has been found.  Until then nothing can be judged in range,
    // because the fallback range is "all 32-bit guest code" and would let any
    // thread that touched a wine dll win the latch.
    [[nodiscard]] bool rangeKnown() const;

    mach_port_t task;
    uint64_t base;
    SamplerConfig cfg;
    int reads;
    // Read once while the target is alive.  The last profile is written after it
    // has exited, and pid_for_task on a dead task reports -1, which loses the
    // one field that ties a profile to the run's log.
    pid_t pid = 0;
    // Main-image detection, used only until it succeeds.
    bool image_found = false;
    int image_attempts = 0;
    std::unordered_set<uint64_t> image_rejected;
};

// Give up after this many probes and keep the default range: a target whose
// main image never shows up in a sample is one the walk-back cannot help with.
constexpr int kMaxImageAttempts = 64;

// Exit handshake with the sampler thread, which is detached and would otherwise
// be killed mid-interval by process exit.  One relaxed load per tick.
std::atomic<bool> g_sampler_running{false};
std::atomic<bool> g_sampler_flush_request{false};
std::atomic<bool> g_sampler_flush_done{false};

bool SamplerCtx::rangeKnown() const {
    return cfg.guest_range_pinned || image_found;
}

bool samplerRead(void* raw, uint64_t addr, void* dst, size_t len) {
    auto* ctx = static_cast<SamplerCtx*>(raw);
    ctx->reads++;
    return readParent(ctx->task, addr, dst, len);
}

double nowUs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (static_cast<double>(ts.tv_sec) * 1e6) + (static_cast<double>(ts.tv_nsec) / 1e3);
}

// Unlatch after this long without seeing the latched thread in the range, so a
// thread that stops being the interesting one is given up.  Wall-clock, not
// ticks: at 10 kHz a tick count would make this window a fifth of a second and
// every loading screen would unlatch.
constexpr double kUnlatchAfterS = 2.0;

// Sweep every thread for this many sweeps before latching, and latch onto the
// one seen in the range most often rather than the first one seen at all: more
// than one thread can run guest code and the busiest is the one worth having.
constexpr uint64_t kDiscoverySweeps = 200;

// If nothing ever runs inside the guest range, discovery would otherwise sweep
// every thread forever, which is the most expensive mode there is.  After this
// many sweeps drop to one sweep per kIdleSweepS seconds.  It never stops
// looking: a target that starts slowly, or whose interesting thread is replaced
// by another one, still gets picked up, just with a longer wait.  Latching
// clears the throttle again.
constexpr uint64_t kDiscoveryGiveUp = 30000;
constexpr double kIdleSweepS = 1.0;

// Frame-pointer walk of the GUEST stack.  Return addresses on that stack are
// already guest x86 addresses (the translation keeps guest state in guest
// memory), so unwinding needs no resolver at all: only the leaf pc comes from
// the ARM pc.  Frame pointer omission means the chain can be short or break
// early; that costs depth, never a wrong leaf.
constexpr int kMaxFrames = 48;

// A caller's frame sits above the callee's and not absurdly far above it.
constexpr uint64_t kMaxFrameSpan = 1U << 20;

// Nothing is mapped in the low 64 KB of a Windows address space, so a "return
// address" below this is a misread slot, not a frame.  Without this the walk
// happily appends roots like 0x1, 0x4, 0xa0 and 0x800, which then show up as
// garbage at the base of a stack.
constexpr uint64_t kMinCodeAddr = 0x10000;

int unwindGuestStack(mach_port_t task, uint64_t framePtr, bool guest32, uint64_t* out, int max) {
    const uint64_t mask = guest32 ? 0xFFFFFFFFULL : ~0ULL;
    const size_t width = guest32 ? 4 : 8;
    uint64_t fp = framePtr & mask;
    int depth = 0;
    while (depth < max) {
        if (fp == 0 || (fp & (width - 1)) != 0) {
            break;
        }
        uint8_t frame[16];
        if (!readParent(task, fp, frame, width * 2)) {
            break;
        }
        uint64_t saved = 0;
        uint64_t ret = 0;
        memcpy(&saved, frame, width);
        memcpy(&ret, frame + width, width);
        if (ret < kMinCodeAddr || (guest32 && ret >= 0x100000000ULL)) {
            break;
        }
        out[depth++] = ret;
        // The stack grows down, so the caller's frame is above ours.
        if (saved <= fp || saved - fp > kMaxFrameSpan) {
            break;
        }
        fp = saved;
    }
    return depth;
}

// ── Finding the guest's main image ──────────────────────────────────────────
// The interesting thread is the one running the program, not a worker running
// library code, so the latch needs the main executable's address range.  Rather
// than be told it, read it out of the guest: every resolved pc lies inside some
// mapped PE image, Windows maps images on 64 KB boundaries, and an image's own
// header carries both its size and whether it is an EXE or a DLL.  Walking back
// from a pc to the nearest such header therefore names the image the pc belongs
// to, and the IMAGE_FILE_DLL bit says whether it is the one we want.
constexpr uint64_t kImageGranularity = 0x10000;

// WoW.exe is ~9.5 MB; this bounds the walk-back for any pc we probe.
constexpr int kMaxImageSteps = 512;

constexpr uint16_t kDosMagic = 0x5A4D;     // "MZ"
constexpr uint32_t kPeSignature = 0x4550;  // "PE\0\0"
constexpr uint16_t kPe32Magic = 0x10B;
constexpr uint16_t kPe32PlusMagic = 0x20B;
constexpr uint16_t kFileDll = 0x2000;  // IMAGE_FILE_DLL

struct GuestImage {
    uint64_t base = 0;
    uint32_t size = 0;
    bool is_exe = false;
    bool macho = false;  // else a PE
};

// Parse a PE image header at `base`, if one is there.  `allow64` also accepts a
// PE32+ one: the module map wants every image the process has, while the
// main-image search below is looking for a 32-bit program and would rather not
// consider the 64-bit side of a wow64 process at all.  SizeOfImage sits 56 bytes
// into the optional header either way — PE32+ drops BaseOfData and widens
// ImageBase, which cancels out.
bool readGuestImage(mach_port_t task, uint64_t base, GuestImage& out, bool allow64 = false) {
    uint16_t dos = 0;
    if (!readParent(task, base, &dos, sizeof(dos)) || dos != kDosMagic) {
        return false;
    }
    int32_t lfanew = 0;
    if (!readParent(task, base + 0x3C, &lfanew, sizeof(lfanew)) || lfanew < 0 ||
        lfanew > (1 << 20)) {
        return false;
    }
    const uint64_t pe = base + static_cast<uint64_t>(lfanew);
    uint32_t sig = 0;
    if (!readParent(task, pe, &sig, sizeof(sig)) || sig != kPeSignature) {
        return false;
    }
    uint16_t characteristics = 0;
    uint16_t magic = 0;
    uint32_t sizeOfImage = 0;
    // COFF header follows the signature; Characteristics is its last field.
    // The optional header follows that, with SizeOfImage 56 bytes in.
    if (!readParent(task, pe + 4 + 18, &characteristics, sizeof(characteristics)) ||
        !readParent(task, pe + 24, &magic, sizeof(magic)) ||
        !readParent(task, pe + 24 + 56, &sizeOfImage, sizeof(sizeOfImage))) {
        return false;
    }
    const bool bitnessWanted = magic == kPe32Magic || (allow64 && magic == kPe32PlusMagic);
    if (!bitnessWanted) {
        return false;
    }
    if (sizeOfImage == 0 || sizeOfImage > (256U << 20)) {
        return false;
    }
    out.base = base;
    out.size = sizeOfImage;
    out.is_exe = (characteristics & kFileDll) == 0;
    out.macho = false;
    return true;
}

// A Mach-O guest (an ordinary x86-64 program under Rosetta, rather than a
// Windows one under Wine) carries the same two facts in its own header:
// MH_EXECUTE says it is a program and not a library, and the segment table
// gives its extent.  Unlike a PE, its base is not on a coarse boundary that can
// be walked back to, but it does not need to be: the region the pc lives in
// starts at the __TEXT segment, which is where the header sits.
constexpr uint32_t kMachMagic64 = 0xFEEDFACFU;
constexpr uint32_t kMachMagic32 = 0xFEEDFACEU;
constexpr uint32_t kMachExecute = 2;  // MH_EXECUTE
constexpr uint32_t kLcSegment64 = 0x19;
constexpr uint32_t kLcSegment32 = 0x1;

// `codeOnly` restricts the extent to the executable segments.  A dylib in the
// dyld shared cache has its __TEXT, __DATA and __LINKEDIT split across the
// cache's own regions, gigabytes apart, so its full extent is meaningless as a
// module range; the executable part is contiguous and is the only part a pc can
// land in.  The guest-range use below wants the whole image and leaves it off.
constexpr uint32_t kLcIdDylib = 0xD;

bool readGuestMachO(mach_port_t task, uint64_t base, GuestImage& out, bool codeOnly = false,
                    std::string* installName = nullptr) {
    uint32_t header[8];
    if (!readParent(task, base, header, sizeof(header))) {
        return false;
    }
    const bool is64 = header[0] == kMachMagic64;
    if (!is64 && header[0] != kMachMagic32) {
        return false;
    }
    const uint32_t filetype = header[3];
    const uint32_t ncmds = header[4];
    if (ncmds == 0 || ncmds > 4096) {
        return false;
    }

    uint64_t lo = UINT64_MAX;
    uint64_t hi = 0;
    uint64_t cmd = base + (is64 ? 32 : 28);
    for (uint32_t i = 0; i < ncmds; i++) {
        uint32_t head[2];  // cmd, cmdsize
        if (!readParent(task, cmd, head, sizeof(head)) || head[1] < 8) {
            return false;
        }
        // A dylib's own name, which is the only way to name one inside the dyld
        // shared cache: every mapping there reports the cache file rather than
        // the library, so the kernel cannot tell us and only the image itself
        // knows.  Free here — these load commands are already being walked.
        if (head[0] == kLcIdDylib && installName != nullptr && head[1] >= 12) {
            uint32_t nameOff = 0;
            if (readParent(task, cmd + 8, &nameOff, sizeof(nameOff)) && nameOff >= 12 &&
                nameOff < head[1]) {
                char buf[512];
                const size_t want = std::min<size_t>(sizeof(buf) - 1, head[1] - nameOff);
                if (readParent(task, cmd + nameOff, buf, want)) {
                    buf[want] = '\0';
                    *installName = buf;
                }
            }
        }
        if (head[0] == (is64 ? kLcSegment64 : kLcSegment32)) {
            if (head[1] < (is64 ? 72U : 56U)) {
                return false;  // too short to be the segment command it claims
            }
            char name[16];
            if (!readParent(task, cmd + 8, name, sizeof(name))) {
                return false;
            }
            // __PAGEZERO is a multi-gigabyte hole below the image, not part of
            // it, and would swallow the whole address space if counted.
            if (memcmp(name, "__PAGEZERO", 10) != 0) {
                uint64_t vmaddr = 0;
                uint64_t vmsize = 0;
                uint32_t initprot = 0;
                if (is64) {
                    if (!readParent(task, cmd + 24, &vmaddr, 8) ||
                        !readParent(task, cmd + 32, &vmsize, 8) ||
                        !readParent(task, cmd + 60, &initprot, 4)) {
                        return false;
                    }
                } else {
                    uint32_t a = 0;
                    uint32_t sz = 0;
                    if (!readParent(task, cmd + 24, &a, 4) || !readParent(task, cmd + 28, &sz, 4) ||
                        !readParent(task, cmd + 44, &initprot, 4)) {
                        return false;
                    }
                    vmaddr = a;
                    vmsize = sz;
                }
                if (!codeOnly || (initprot & VM_PROT_EXECUTE) != 0) {
                    lo = vmaddr < lo ? vmaddr : lo;
                    hi = vmaddr + vmsize > hi ? vmaddr + vmsize : hi;
                }
            }
        }
        cmd += head[1];
    }
    if (lo == UINT64_MAX || hi <= lo || hi - lo > (256ULL << 20)) {
        return false;
    }

    out.base = base;
    out.size = static_cast<uint32_t>(hi - lo);
    out.is_exe = filetype == kMachExecute;
    out.macho = true;
    return true;
}

// The start of the mapping `pc` lives in.  For a Mach-O that is the image
// header; for a PE it is usually a section, which is why the PE path walks.
bool guestRegionStart(mach_port_t task, uint64_t pc, uint64_t& start) {
    mach_vm_address_t address = pc;
    mach_vm_size_t size = 0;
    vm_region_basic_info_data_64_t info;
    mach_msg_type_number_t count = VM_REGION_BASIC_INFO_COUNT_64;
    mach_port_t object = MACH_PORT_NULL;
    if (mach_vm_region(task, &address, &size, VM_REGION_BASIC_INFO_64,
                       reinterpret_cast<vm_region_info_t>(&info), &count,
                       &object) != KERN_SUCCESS) {
        return false;
    }
    if (pc < address || pc >= address + size) {
        return false;  // nothing mapped at pc; the returned region is above it
    }
    start = address;
    return true;
}

// The image containing `pc`: a Mach-O at the start of its mapping, or a PE
// found by walking back to the nearest header.  `forMap` asks the two readers
// for what the module map wants rather than what main-image detection wants:
// PE32+ accepted, and a Mach-O measured by its executable segments only.
bool findGuestImage(mach_port_t task, uint64_t pc, GuestImage& out, bool forMap = false,
                    std::string* installName = nullptr) {
    uint64_t regionStart = 0;
    if (guestRegionStart(task, pc, regionStart) &&
        readGuestMachO(task, regionStart, out, forMap, installName) && pc < out.base + out.size) {
        return true;
    }

    uint64_t candidate = pc & ~(kImageGranularity - 1);
    for (int step = 0; step < kMaxImageSteps; step++) {
        GuestImage img;
        if (readGuestImage(task, candidate, img, forMap) && pc < img.base + img.size) {
            out = img;
            return true;
        }
        if (candidate < kImageGranularity) {
            break;
        }
        candidate -= kImageGranularity;
    }
    return false;
}

// The module holding `addr`, or nullptr.
const GuestModule* moduleAt(const std::map<uint64_t, GuestModule>& modules, uint64_t addr) {
    auto it = modules.upper_bound(addr);
    if (it == modules.begin()) {
        return nullptr;
    }
    --it;
    return addr < it->second.base + it->second.size ? &it->second : nullptr;
}

// Build the module map that goes into the profile.  It takes two sources,
// because neither sees the whole address space: dyld knows every Mach-O image by
// name, including those in the shared cache, and knows nothing about the PE
// images wine maps; the kernel knows which file backs any mapping, which names
// the PEs, but reports the cache file rather than the dylib for anything in the
// shared cache.
//
// NOTHING here walks the address space.  A walk is ~12000 region queries plus a
// read apiece, and every one of them takes the TARGET's vm_map lock, which its
// own mmap and page faults need too — measured cost on the client: the game took
// 13 s to reach its login screen against 8 s without the scanner.  Instead the
// sampler hands over the addresses it actually saw and only those are looked up,
// so once a module has been resolved the scanner touches the target no further.

// Record a named image, dropping any `anon@` row it turns out to cover.  Order
// alone decides otherwise, and gets it wrong: a region inside an image that no
// header walk-back can find becomes an anon row first, dyld names the image
// afterwards, and because a lookup takes the greatest base at or below the
// address the row then shadows the image for the rest of the run.  Measured on
// the client: two rows inside libRosettaRuntime hid 4200 samples that its export
// table can name.  Only rows wholly inside the image go; one that straddles its
// end also describes memory the image does not.
void registerImage(std::map<uint64_t, GuestModule>& modules, const GuestModule& mod) {
    const uint64_t limit = mod.base + mod.size;
    for (auto it = modules.lower_bound(mod.base); it != modules.end() && it->first < limit;) {
        if (it->second.anon && it->second.base + it->second.size <= limit) {
            it = modules.erase(it);
        } else {
            ++it;
        }
    }
    modules[mod.base] = mod;
}

// Ask dyld where its images are.  This is the ONLY way to name anything in the
// shared cache: a dylib's __TEXT there sits hundreds of megabytes inside a
// single ~2 GB region that covers a thousand libraries, so there is no header
// for a per-address lookup to walk back to.  Reads nothing for an image already
// known at the same base under the same name — a Mach-O's segments are fixed
// once it is loaded.
void refreshDyldImages(mach_port_t task, std::map<uint64_t, GuestModule>& modules) {
    const double started = nowUs();
    __block std::map<uint64_t, GuestModule>* known = &modules;
    __block uint64_t learned = 0;
    kern_return_t kr = KERN_SUCCESS;
    DyldProcessInfo info = _dyld_process_info_create(task, 0, &kr);
    if (info == nullptr) {
        return;
    }
    _dyld_process_info_for_each_image(
        info, ^(uint64_t header, const uuid_t /*uuid*/, const char* path) {
          if (path == nullptr) {
              return;
          }
          const auto it = known->find(header);
          if (it != known->end() && it->second.macho && it->second.path == path) {
              return;
          }
          GuestImage img;
          if (readGuestMachO(task, header, img, true)) {
              const GuestModule mod{.base = header, .size = img.size, .macho = true, .path = path};
              registerImage(*known, mod);
              learned++;
          }
        });
    _dyld_process_info_release(info);
    g_modmap.dyld_asks.fetch_add(1, std::memory_order_relaxed);
    g_modmap.learned.fetch_add(learned, std::memory_order_relaxed);
    g_modmap.scan_us.fetch_add(static_cast<uint64_t>(nowUs() - started), std::memory_order_relaxed);
}

// Does `addr` point at executable memory?  One region query, and the only cheap
// way to tell code the map has not named yet from the ordinary data a
// frame-pointer walk mistakes for a return address.
// `regionBase`/`regionSize`, when given, report the mapping the address fell
// in, so a caller that cannot name it any other way can still describe it by
// its extent rather than discarding it.
bool looksLikeCode(mach_port_t task, uint64_t addr, uint64_t* regionBase = nullptr,
                   uint64_t* regionSize = nullptr) {
    mach_vm_address_t address = addr;
    mach_vm_size_t size = 0;
    vm_region_submap_info_data_64_t info;
    mach_msg_type_number_t count = VM_REGION_SUBMAP_INFO_COUNT_64;
    natural_t depth = 1000;
    // _recurse, not mach_vm_region: the shared cache is a submap, and the plain
    // call reports the submap entry (~2 GB, r--) instead of the mapping inside
    // it (r-x).  Measured — with the plain call every cache address looks
    // non-executable and this test rejects exactly what it is meant to admit.
    if (mach_vm_region_recurse(task, &address, &size, &depth,
                               reinterpret_cast<vm_region_recurse_info_t>(&info),
                               &count) != KERN_SUCCESS) {
        return false;
    }
    if (regionBase != nullptr) {
        *regionBase = address;
    }
    if (regionSize != nullptr) {
        *regionSize = size;
    }
    return addr >= address && addr < address + size && (info.protection & VM_PROT_EXECUTE) != 0;
}

// One sampled address the map cannot explain: find the image holding it, and
// nothing else.  A PE is found by walking back to its header on the 64 KB grid,
// a Mach-O at the start of its mapping; then one name lookup.  Tens of syscalls
// for a whole module, paid once, against thousands for a walk paid every time.
bool resolveAddress(mach_port_t task, pid_t pid, uint64_t addr,
                    std::map<uint64_t, GuestModule>& modules) {
    const double started = nowUs();
    g_modmap.searches.fetch_add(1, std::memory_order_relaxed);
    GuestImage img;
    std::string installName;
    if (!findGuestImage(task, addr, img, true, &installName)) {
        return false;
    }
    // What the kernel says backs the mapping, which is right for everything wine
    // maps and for an ordinary dylib on disk.
    char path[MAXPATHLEN];
    path[0] = '\0';
    const int len = proc_regionfilename(pid, img.base, path, sizeof(path));
    if (len > 0) {
        path[std::min<size_t>(static_cast<size_t>(len), sizeof(path) - 1)] = '\0';
    }
    // Inside the shared cache every mapping reports the cache itself, so the
    // image's own install name is the only thing that names the library.
    std::string name = path;
    if (name.empty() || name.find("dyld_shared_cache") != std::string::npos) {
        if (installName.empty()) {
            return false;  // nothing can name it; not worth reporting
        }
        name = installName;
    }
    registerImage(
        modules, GuestModule{.base = img.base, .size = img.size, .macho = img.macho, .path = name});
    g_modmap.learned.fetch_add(1, std::memory_order_relaxed);
    g_modmap.scan_us.fetch_add(static_cast<uint64_t>(nowUs() - started), std::memory_order_relaxed);
    return true;
}

// The sampler's handoff to the aggregator: a single-producer, single-consumer
// ring of 64-bit words.  Measuring a sample costs a few stores and an index
// publish — no lookup, no allocation, no lock — and everything else (the
// histograms, the module map, the profile) belongs to the thread that drains it.
//
// Records are variable length because stacks are:
//     [0] mach thread id
//     [1] status | no-guest reason | in_range | depth
//     [2] leaf guest pc, or the host pc when there is no guest one
//     [3] x16                      (only when the status is NotTranslated)
//     [3..] `depth` frames, root first (only when the status is Resolved)
constexpr size_t kRingWords = 1U << 18;  // 2 MB, seconds of slack at 10 kHz
constexpr size_t kRingMask = kRingWords - 1;

enum class SampleKind : uint8_t {
    Resolved = 0,
    NotTranslated = 1,
    Unavailable = 2,
};

struct SampleRing {
    std::array<std::atomic<uint64_t>, kRingWords> slots;
    std::atomic<uint64_t> head{0};  // words published by the sampler
    std::atomic<uint64_t> tail{0};  // words consumed by the aggregator
    std::atomic<uint64_t> dropped{0};

    // Producer.  Never blocks and never waits: a record that does not fit is
    // dropped whole and counted, because stalling the sampler would distort the
    // very timeline it is measuring.
    void push(const uint64_t* words, size_t n) {
        const uint64_t h = head.load(std::memory_order_relaxed);
        const uint64_t t = tail.load(std::memory_order_acquire);
        if (h - t + n > kRingWords) {
            dropped.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        for (size_t i = 0; i < n; i++) {
            slots[(h + i) & kRingMask].store(words[i], std::memory_order_relaxed);
        }
        // Publishing last is what makes a partly written record unreadable.
        head.store(h + n, std::memory_order_release);
    }
};
SampleRing g_ring;

// The reason rides in bits 24..31, which the header already had spare, so a
// record does not grow to carry it.
uint64_t sampleHeader(SampleKind kind, bool inRange, uint64_t depth,
                      guest_pc::NoGuestReason reason = guest_pc::NoGuestReason::Unknown) {
    return (static_cast<uint64_t>(kind) << 32) | (static_cast<uint64_t>(reason) << 24) |
           (static_cast<uint64_t>(inRange) << 16) | depth;
}

// What the sampler measures about itself.  Counters rather than records because
// they describe the sampling, not any one sample.
struct SamplerCounters {
    std::atomic<uint64_t> samples{0};
    std::atomic<uint64_t> total_ns{0};
    std::atomic<uint64_t> missed_ticks{0};
    // Of those, the ticks where OUR work alone outran the period.  The rest
    // were late for reasons outside this thread: a sleep that returned late, or
    // preemption.  At a 100 us period either is easy to hit.
    std::atomic<uint64_t> overrun_ticks{0};
    // The guest range, once main-image detection settles it.
    std::atomic<uint64_t> guest_lo{0};
    std::atomic<uint64_t> guest_hi{0};
    std::atomic<bool> image_found{false};
    // A snapshot of the resolver cache, refreshed periodically: the cache itself
    // belongs to the sampler thread and may not be read from another.
    std::atomic<uint64_t> cache_pc_hits{0};
    std::atomic<uint64_t> cache_fragment_hits{0};
    std::atomic<uint64_t> cache_negative_hits{0};
    std::atomic<uint64_t> cache_misses{0};
    std::atomic<uint64_t> cache_stale{0};
};
SamplerCounters g_counters;

// Latch state, which the profile reports and only the sampler knows.  A mutex is
// right here and nowhere else: it is taken on a latch transition and once per
// profile write, never per sample.
struct LatchState {
    std::mutex mu;
    bool latched = false;
    uint64_t tid = 0;
    std::string name;
    uint64_t sweeps = 0;
    double started = 0;
    // Time actually spent latched, which is the only time samples accrue.  The
    // rate must be divided by THIS and not by the sampler's lifetime: discovery
    // takes ~2 s on the client, and counting it made a 99% run read as 88%.
    double latched_at = 0;  // 0 when not latched
    double latched_us = 0;  // completed latched stretches
    std::vector<std::string> events;

    void add(double at, const char* fmt, ...) __attribute__((format(printf, 3, 4))) {
        char detail[256];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(detail, sizeof(detail), fmt, ap);
        va_end(ap);
        char line[320];
        snprintf(line, sizeof(line), "%.1f %s", at, detail);
        const std::scoped_lock lock(mu);
        events.emplace_back(line);
    }
};
LatchState g_latch;

// Everything is kept per guest thread, so the profile says which thread each
// address came from rather than leaving it implicit.
struct ThreadStats {
    uint64_t samples = 0;
    uint64_t resolved = 0;
    uint64_t in_range = 0;
    std::unordered_map<uint64_t, uint64_t> pcs;        // leaf guest pc -> hits
    std::map<std::vector<uint64_t>, uint64_t> stacks;  // root-first stack -> hits
    // Host ARM pcs, for the samples no x86 was translated to.  Leaves only: the
    // guest frame-pointer chain says nothing about native frames, and an empty
    // answer is better than a walk of whatever x5 happened to hold.
    std::unordered_map<uint64_t, uint64_t> host_pcs;
    // Why each of those pcs had no guest pc, as a set of 1 << NoGuestReason
    // bits.  A set rather than one value because a fragment can be freed and the
    // same address reused, so the same pc can legitimately answer differently at
    // different times; in practice one bit is set and it says whether the
    // address is Rosetta's own code or translated output the map does not reach.
    std::unordered_map<uint64_t, uint8_t> host_reasons;
    // For the host pcs that are the instruction after `svc #0x80`: which syscalls
    // the thread was seen inside there, and how often.  One park pc serves every
    // blocking call the guest makes, so this is the difference between "18% of the
    // thread is in one unnameable instruction" and knowing what it waits on.
    std::unordered_map<uint64_t, std::map<int32_t, uint64_t>> host_syscalls;
};

struct SamplerStats {
    uint64_t samples = 0;           // recorded, i.e. what the histograms hold
    uint64_t samples_measured = 0;  // taken, including any the ring had to drop
    uint64_t in_range = 0;
    uint64_t resolved = 0;
    uint64_t not_translated = 0;
    uint64_t unavailable = 0;
    uint64_t frames = 0;   // total frames unwound, for the average depth
    uint64_t unwound = 0;  // samples an unwind was attempted for
    double total_us = 0;
    std::map<uint64_t, ThreadStats> threads;  // mach thread id -> its samples
};

// Fold one set of counters into another, so the run total can be accumulated
// from the windows rather than kept alongside them.  The aggregator drains into
// the window set and merges here once per report, which keeps the drain path
// single-target and makes the two views agree by construction: the cumulative
// profile IS the sum of every window written.
void mergeStats(SamplerStats& into, const SamplerStats& from) {
    into.samples += from.samples;
    into.samples_measured += from.samples_measured;
    into.in_range += from.in_range;
    into.resolved += from.resolved;
    into.not_translated += from.not_translated;
    into.unavailable += from.unavailable;
    into.frames += from.frames;
    into.unwound += from.unwound;
    into.total_us += from.total_us;
    for (const auto& [tid, src] : from.threads) {
        ThreadStats& dst = into.threads[tid];
        dst.samples += src.samples;
        dst.resolved += src.resolved;
        dst.in_range += src.in_range;
        for (const auto& [pc, n] : src.pcs) {
            dst.pcs[pc] += n;
        }
        for (const auto& [stack, n] : src.stacks) {
            dst.stacks[stack] += n;
        }
        for (const auto& [pc, n] : src.host_pcs) {
            dst.host_pcs[pc] += n;
        }
        for (const auto& [pc, bits] : src.host_reasons) {
            dst.host_reasons[pc] |= bits;
        }
        for (const auto& [pc, calls] : src.host_syscalls) {
            for (const auto& [call, n] : calls) {
                dst.host_syscalls[pc][call] += n;
            }
        }
    }
}

// A thread's name, if it set one.  Empty for most, but a named thread is the
// clearest way to say which one a profile is of.
std::string machThreadName(mach_port_t thread) {
    thread_extended_info_data_t info{};
    mach_msg_type_number_t count = THREAD_EXTENDED_INFO_COUNT;
    if (thread_info(thread, THREAD_EXTENDED_INFO, reinterpret_cast<thread_info_t>(&info), &count) !=
        KERN_SUCCESS) {
        return {};
    }
    info.pth_name[sizeof(info.pth_name) - 1] = '\0';
    return info.pth_name;
}

// The mach thread id, which is stable for the life of the thread, unlike the
// port name a task_threads sweep hands back.
uint64_t machThreadId(mach_port_t thread) {
    thread_identifier_info_data_t id{};
    mach_msg_type_number_t count = THREAD_IDENTIFIER_INFO_COUNT;
    if (thread_info(thread, THREAD_IDENTIFIER_INFO, reinterpret_cast<thread_info_t>(&id), &count) !=
        KERN_SUCCESS) {
        return 0;
    }
    return id.thread_id;
}

// `record` false makes this a probe: it still resolves, so the caller can tell
// whether the thread is in the guest range, but nothing enters the profile.
// Discovery uses that, because a latched profile must contain the latched
// thread's samples and nothing else.
// One sample: read the thread's ARM pc, resolve it to a guest pc, unwind, and
// hand the result to the aggregator.  Nothing is accumulated here and nothing is
// looked up — this thread's only job is to be on time.  `record=false` makes it
// a discovery probe, which scores a thread without entering the profile.
void sampleThread(SamplerCtx* ctx, const guest_pc::Reader& reader, guest_pc::Cache& cache,
                  mach_port_t thread, uint64_t tid, bool record, bool& inRange) {
    inRange = false;
    arm_thread_state64_t state{};
    mach_msg_type_number_t count = ARM_THREAD_STATE64_COUNT;
    if (thread_get_state(thread, ARM_THREAD_STATE64, reinterpret_cast<thread_state_t>(&state),
                         &count) != KERN_SUCCESS) {
        return;
    }
    if (record) {
        g_counters.samples.fetch_add(1, std::memory_order_relaxed);
    }

    guest_pc::Resolution res{};
    const double t0 = nowUs();
    const guest_pc::Status status =
        guest_pc::resolve(reader, ctx->base, arm_thread_state64_get_pc(state), res, cache);

    // Room for the header words, the leaf and a full stack.
    uint64_t rec[kMaxFrames + 3];
    size_t n = 0;

    switch (status) {
        case guest_pc::Status::Resolved: {
            if (!ctx->cfg.guest_range_pinned && !ctx->image_found &&
                ctx->image_attempts < kMaxImageAttempts &&
                !ctx->image_rejected.contains(res.x86_pc & ~(kImageGranularity - 1))) {
                GuestImage img;
                if (findGuestImage(ctx->task, res.x86_pc, img)) {
                    if (img.is_exe) {
                        ctx->cfg.guest_lo = img.base;
                        ctx->cfg.guest_hi = img.base + img.size;
                        ctx->image_found = true;
                        g_counters.guest_lo.store(img.base, std::memory_order_relaxed);
                        g_counters.guest_hi.store(img.base + img.size, std::memory_order_relaxed);
                        g_counters.image_found.store(true, std::memory_order_relaxed);
                        fprintf(stdout,
                                "[rosettax87] X87_SAMPLE: main image found at "
                                "0x%llx-0x%llx; looking for the thread that runs it\n",
                                static_cast<unsigned long long>(img.base),
                                static_cast<unsigned long long>(img.base + img.size));
                        fflush(stdout);
                    } else {
                        // A DLL: remember it so its pcs stop costing a walk.
                        for (uint64_t p = img.base; p < img.base + img.size;
                             p += kImageGranularity) {
                            ctx->image_rejected.insert(p);
                        }
                    }
                } else {
                    // Only a walk that found no image at all counts against the
                    // budget.  Identifying a DLL is progress: it is cached, so
                    // it never costs again, and a client can easily run code in
                    // dozens of libraries before it reaches its own.
                    ctx->image_attempts++;
                    ctx->image_rejected.insert(res.x86_pc & ~(kImageGranularity - 1));
                }
            }
            inRange = ctx->rangeKnown() && res.x86_pc >= ctx->cfg.guest_lo &&
                      res.x86_pc < ctx->cfg.guest_hi;
            if (!record) {
                break;
            }
            uint64_t depth = 0;
            if (ctx->cfg.unwind) {
                // x5 holds the guest frame pointer while translated code runs,
                // which the resolver has just confirmed is the case.
                const bool guest32 = res.x86_pc < 0x100000000ULL;
                uint64_t frames[kMaxFrames];
                const int walked =
                    unwindGuestStack(ctx->task, state.__x[5], guest32, frames, kMaxFrames);
                depth = static_cast<uint64_t>(walked);
                rec[3] = 0;  // filled below, root first
                for (int i = walked - 1; i >= 0; i--) {
                    rec[3 + (walked - 1 - i)] = frames[i];
                }
            }
            rec[0] = tid;
            rec[1] = sampleHeader(SampleKind::Resolved, inRange, depth);
            rec[2] = res.x86_pc;
            n = 3 + depth;
            break;
        }
        case guest_pc::Status::NotTranslated:
            if (record) {
                rec[0] = tid;
                rec[1] = sampleHeader(SampleKind::NotTranslated, false, 0, res.reason);
                // The host pc, which is what the thread was actually running: no
                // x86 was translated to this address because none is involved.
                // Everything the guest runs is x86_64 and gets translated, so
                // this is Rosetta's own arm64 code and nothing else: its runtime
                // image, libRosettaRuntime, and the regions of generated
                // routines a blocked thread parks in.  On the client that is
                // ~30% of the samples on the game's own thread, and counting
                // them without keeping the address made the largest single slice
                // of the thread the one nothing could be said about.
                rec[2] = arm_thread_state64_get_pc(state);
                // x16, because a thread parked immediately after `svc #0x80` has
                // the syscall number still in it, and the aggregator can tell
                // which pcs those are.  Rosetta's guest-syscall dispatcher takes
                // the number from the guest's own eax, so nothing static can name
                // that call site: this register is the only witness.
                rec[3] = state.__x[16];
                n = 4;
            }
            break;
        case guest_pc::Status::Unavailable:
            if (record) {
                rec[0] = tid;
                rec[1] = sampleHeader(SampleKind::Unavailable, false, 0);
                n = 2;
            }
            break;
    }
    if (n != 0) {
        g_ring.push(rec, n);
    }
    // Timed across resolve AND unwind: the unwind is the larger half once the
    // cache is warm, so timing only the resolve would flatter the sampler.
    if (record) {
        g_counters.total_ns.fetch_add(static_cast<uint64_t>((nowUs() - t0) * 1000.0),
                                      std::memory_order_relaxed);
    }
}

// Everything the profile is made of, owned by one thread: the histograms, the
// module map and the file itself.  The sampler thread touches none of it.
struct AggCtx {
    mach_port_t task;
    pid_t pid;
    uint64_t runtime_base;
    std::string path;
    uint64_t interval_us;
    uint64_t sweep_interval_us;
    double report_s;
    bool unwind;
    bool range_pinned;
    // Per-window delta profiles: how many have been written, and where the one
    // being filled started, in run seconds and in latched seconds.  A window's
    // rate has to divide by the latched time inside THAT window, not the run's.
    bool windows = false;
    unsigned window_seq = 0;
    double window_started_s = 0;
    double window_latched_us = 0;
    // The whole series lives in one appended file, <path>.windows, opened on the
    // first window written rather than at startup so a run that never latches
    // leaves nothing behind.  One file per window was what this used to be, and
    // at a ten-second interval an hour of play left 360 of them next to the
    // profile.
    FILE* window_file = nullptr;
    std::map<uint64_t, GuestModule> modules;
    // 64 KB slots holding an address no search could explain: a frame-pointer
    // walk invents return addresses out of ordinary data, and those must cost
    // one lookup, not one per sighting.
    std::unordered_set<uint64_t> unmapped;
    // Host pcs already tested for being the instruction after `svc #0x80`.  One
    // entry per distinct address, so a run pays one read per park point rather
    // than anything per sample.
    std::unordered_map<uint64_t, bool> host_svc_return;
    double last_dyld_us = 0;
    bool asked_dyld = false;
    // The fullest the ring has been seen.  Owned by this thread alone, so no
    // atomic: it is read once, when this same thread writes the profile.
    uint64_t ring_peak = 0;
};

// How often dyld's image list may be consulted, at most.
constexpr double kDyldAskEveryS = 1.0;

// How the profile spells one NoGuestReason bit.  A pc normally carries exactly
// one; more than one means the address was reused, and the reader joins them.
const char* noGuestReasonName(guest_pc::NoGuestReason reason) {
    switch (reason) {
        case guest_pc::NoGuestReason::NoFragment:
            return "nofrag";
        case guest_pc::NoGuestReason::RuntimeRoutines:
            return "routines";
        case guest_pc::NoGuestReason::BeforeFirstBoundary:
            return "preboundary";
        case guest_pc::NoGuestReason::Unknown:
            break;
    }
    return "unknown";
}

// The reason bits of one host pc as a `|`-joined field, never empty.
std::string noGuestReasonField(uint8_t bits) {
    std::string out;
    for (int i = 0; i < 8; i++) {
        if ((bits & (1U << i)) == 0) {
            continue;
        }
        if (!out.empty()) {
            out += "|";
        }
        out += noGuestReasonName(static_cast<guest_pc::NoGuestReason>(i));
    }
    return out.empty() ? "unknown" : out;
}

// One profile record, written into `fh`.  Written by the aggregator, which owns
// everything in it; the sampler is not involved and does not pause, so sampling
// continues through a write.  A window passes its own span and the latched time
// inside it; windowStart below zero means this is the cumulative profile and the
// whole-run figures apply.
void emitProfile(FILE* fh, const AggCtx* ctx, const SamplerStats& st, double elapsed,
                 double windowStart, double windowProfiled) {
    const std::scoped_lock latch(g_latch.mu);
    const bool windowed = windowStart >= 0;
    // The time samples actually accrued over: latched stretches only.  Dividing
    // by the sampler's lifetime instead counts the discovery phase, which
    // records nothing and takes ~2 s on the client — enough to make a run that
    // achieved 99% of its requested rate read as 88%.
    const double profiled =
        windowed
            ? windowProfiled
            : (g_latch.latched_us + (g_latch.latched_at != 0 ? nowUs() - g_latch.latched_at : 0)) /
                  1e6;
    const uint64_t latchedTid = g_latch.tid;
    const std::string& latchedName = g_latch.name;
    const uint64_t sweeps = g_latch.sweeps;
    pid_t targetPid = ctx->pid;
    char when[64] = "unknown";
    const time_t now = time(nullptr);
    struct tm tmv;
    if (gmtime_r(&now, &tmv) != nullptr) {
        strftime(when, sizeof(when), "%Y-%m-%dT%H:%M:%SZ", &tmv);
    }

    fprintf(fh, "# x87sidecar guest-pc sample profile\n");
    fprintf(fh, "# Addresses are guest x86 pcs. The [modules] section below is the map of the\n");
    fprintf(fh, "# run they came from, so symbolising needs nothing but this file.\n");
    // 2 adds the `slice` module kind, the [host_leaves] reason column and the
    // [host_syscalls] section.
    fprintf(fh, "version 2\n");
    fprintf(fh, "pid %d\n", targetPid);
    fprintf(fh, "written %s\n", when);
    fprintf(fh, "runtime_base 0x%llx\n", static_cast<unsigned long long>(ctx->runtime_base));
    fprintf(fh, "rate_hz %.1f\n",
            ctx->interval_us != 0 ? 1e6 / static_cast<double>(ctx->interval_us) : 0.0);
    fprintf(fh, "interval_us %llu\n", static_cast<unsigned long long>(ctx->interval_us));
    fprintf(fh, "sweep_hz %.1f\n",
            ctx->sweep_interval_us != 0 ? 1e6 / static_cast<double>(ctx->sweep_interval_us) : 0.0);
    fprintf(fh, "elapsed_s %.1f\n", elapsed);
    // A window holds only the samples taken between these two run timestamps,
    // so a reader can pick the stretch it cares about and sum those files.
    // Absent means cumulative: every sample since the sampler started.
    if (windowed) {
        fprintf(fh, "window_seq %u\n", ctx->window_seq);
        fprintf(fh, "window_start_s %.3f\n", windowStart);
        fprintf(fh, "window_end_s %.3f\n", windowStart + elapsed);
    }
    // The rate that was actually achieved.  It is not rate_hz: the sample work,
    // the profile writes below and the kernel's timer leeway all come out of it,
    // so a reader weighting samples by time needs this one.
    fprintf(fh, "profiled_s %.1f\n", profiled);
    fprintf(fh, "effective_hz %.1f\n",
            profiled > 0 ? static_cast<double>(st.samples_measured) / profiled : 0.0);
    fprintf(
        fh, "missed_ticks %llu\n",
        static_cast<unsigned long long>(g_counters.missed_ticks.load(std::memory_order_relaxed)));
    fprintf(
        fh, "overrun_ticks %llu\n",
        static_cast<unsigned long long>(g_counters.overrun_ticks.load(std::memory_order_relaxed)));
    // Samples measured but never recorded, because the aggregator fell behind
    // and the ring was full.  Zero unless the machine is badly overloaded; it is
    // here so a run that lost some says so instead of quietly under-counting.
    fprintf(fh, "samples_dropped %llu\n",
            static_cast<unsigned long long>(g_ring.dropped.load(std::memory_order_relaxed)));
    // What the margin was: how close the handoff came to being full.
    fprintf(fh, "ring_peak_words %llu\n", static_cast<unsigned long long>(ctx->ring_peak));
    fprintf(fh, "ring_peak_pct %.1f\n",
            100.0 * static_cast<double>(ctx->ring_peak) / static_cast<double>(kRingWords));
    fprintf(fh, "mode latched\n");
    fprintf(fh, "guest_range 0x%llx-0x%llx\n",
            static_cast<unsigned long long>(g_counters.guest_lo.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(g_counters.guest_hi.load(std::memory_order_relaxed)));
    const char* rangeFrom = "default (no main image found)";
    if (ctx->range_pinned) {
        rangeFrom = "X87_GUEST_RANGE";
    } else if (g_counters.image_found.load(std::memory_order_relaxed)) {
        rangeFrom = "main image (detected)";
    }
    fprintf(fh, "guest_range_from %s\n", rangeFrom);
    // A failed latch writes no profile at all, so a profile always has a
    // subject.  `latched_thread` is the CURRENT one, not the only one: a run
    // that re-latched has every thread it followed in [threads], each holding
    // the samples taken while it was the subject.  [latch_history] says when
    // each took over.
    fprintf(fh, "latch_state latched\n");
    fprintf(fh, "latched_thread 0x%llx\n", static_cast<unsigned long long>(latchedTid));
    if (!latchedName.empty()) {
        fprintf(fh, "latched_thread_name %s\n", latchedName.c_str());
    }
    fprintf(fh, "threads_seen %zu\n", st.threads.size());
    fprintf(fh, "discovery_sweeps %llu\n", static_cast<unsigned long long>(sweeps));
    fprintf(fh, "latch_events %zu\n", g_latch.events.size());
    fprintf(fh, "unwind %s\n", ctx->unwind ? "yes" : "no");
    fprintf(fh, "samples %llu\n", static_cast<unsigned long long>(st.samples_measured));
    fprintf(fh, "resolved %llu\n", static_cast<unsigned long long>(st.resolved));
    fprintf(fh, "in_range %llu\n", static_cast<unsigned long long>(st.in_range));
    fprintf(fh, "not_translated %llu\n", static_cast<unsigned long long>(st.not_translated));
    fprintf(fh, "unavailable %llu\n", static_cast<unsigned long long>(st.unavailable));
    fprintf(
        fh, "avg_us %.2f\n",
        st.samples_measured != 0 ? st.total_us / static_cast<double>(st.samples_measured) : 0.0);
    fprintf(
        fh, "avg_depth %.2f\n",
        st.unwound != 0 ? static_cast<double>(st.frames) / static_cast<double>(st.unwound) : 0.0);
    fprintf(
        fh, "cache_pc_hits %llu\n",
        static_cast<unsigned long long>(g_counters.cache_pc_hits.load(std::memory_order_relaxed)));
    fprintf(fh, "cache_fragment_hits %llu\n",
            static_cast<unsigned long long>(
                g_counters.cache_fragment_hits.load(std::memory_order_relaxed)));
    fprintf(fh, "cache_negative_hits %llu\n",
            static_cast<unsigned long long>(
                g_counters.cache_negative_hits.load(std::memory_order_relaxed)));
    fprintf(
        fh, "cache_misses %llu\n",
        static_cast<unsigned long long>(g_counters.cache_misses.load(std::memory_order_relaxed)));
    fprintf(
        fh, "cache_stale %llu\n",
        static_cast<unsigned long long>(g_counters.cache_stale.load(std::memory_order_relaxed)));
    fprintf(fh, "modules %zu\n", ctx->modules.size());
    // What the map cost the target: header searches (a walk-back apiece) and
    // whole-image-list asks.  Both should stay small; if searches tracks
    // modules_unmapped, the cheap executability test has stopped filtering.
    fprintf(fh, "modules_searches %llu\n",
            static_cast<unsigned long long>(g_modmap.searches.load(std::memory_order_relaxed)));
    fprintf(fh, "modules_dyld_asks %llu\n",
            static_cast<unsigned long long>(g_modmap.dyld_asks.load(std::memory_order_relaxed)));
    fprintf(fh, "modules_learned %llu\n",
            static_cast<unsigned long long>(g_modmap.learned.load(std::memory_order_relaxed)));
    // 64 KB slots holding an address no image explains: overwhelmingly the
    // frame-pointer walk mistaking data for a return address.
    fprintf(fh, "modules_unmapped %zu\n", ctx->unmapped.size());
    fprintf(fh, "modules_scan_s %.2f\n",
            static_cast<double>(g_modmap.scan_us.load(std::memory_order_relaxed)) / 1e6);

    // Per-thread totals, so a reader can tell one thread's profile from a blend
    // of several without parsing the sample sections.
    // Anything but a single "latched" line means the run was not one
    // continuous observation of one thread.
    fprintf(fh, "\n[latch_history]\n# elapsed_s event detail\n");
    for (const auto& event : g_latch.events) {
        fprintf(fh, "%s\n", event.c_str());
    }

    fprintf(fh, "\n[threads]\n# tid samples resolved in_range latched\n");
    for (const auto& [tid, ts] : st.threads) {
        fprintf(fh, "0x%llx %llu %llu %llu %s\n", static_cast<unsigned long long>(tid),
                static_cast<unsigned long long>(ts.samples),
                static_cast<unsigned long long>(ts.resolved),
                static_cast<unsigned long long>(ts.in_range), tid == latchedTid ? "yes" : "no");
    }

    // What every address below means.  "gone" is an image that was mapped when
    // it was sampled and has since been unloaded; its samples are still real.
    // The path is last because it can contain spaces.
    //
    // Copied straight out of the scanner thread's file rather than built here,
    // which is what keeps a scan of the whole address space off this thread
    // entirely.  Worst case the map is one scan interval old.
    // A `slice` row is part of a file mapped with no header of its own: the file
    // names it, but an offset into the row relates to the mapping only, not to the
    // file, so nothing can be symbolised against it.
    fprintf(fh, "\n[modules]\n# base size kind state path\n");
    for (const auto& [base, mod] : ctx->modules) {
        const char* kind = "pe";
        if (mod.anon) {
            kind = "anon";
        } else if (mod.slice) {
            kind = "slice";
        } else if (mod.macho) {
            kind = "macho";
        }
        fprintf(fh, "0x%llx 0x%llx %s %s %s\n", static_cast<unsigned long long>(mod.base),
                static_cast<unsigned long long>(mod.size), kind, mod.loaded ? "loaded" : "gone",
                mod.path.c_str());
    }

    fprintf(fh, "\n[leaves]\n# tid pc count   (exclusive: execution was AT this address)\n");
    for (const auto& [tid, ts] : st.threads) {
        std::vector<std::pair<uint64_t, uint64_t>> all(ts.pcs.begin(), ts.pcs.end());
        std::sort(all.begin(), all.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });
        for (const auto& [pc, hits] : all) {
            fprintf(fh, "0x%llx 0x%llx %llu\n", static_cast<unsigned long long>(tid),
                    static_cast<unsigned long long>(pc), static_cast<unsigned long long>(hits));
        }
    }

    // Host ARM pcs: the samples with no guest pc because no x86 was involved.
    // Counted against `samples`, not `resolved`, so a reader must weight them
    // against the whole to see how much of the thread they are.
    //
    // `reason` is Rosetta's own answer for why there is no guest pc: `routines`
    // is a kind-0 fragment, i.e. runtime code, `nofrag` is an address in no
    // fragment at all, and `preboundary` is translated output before its map's
    // first boundary.
    fprintf(fh,
            "\n[host_leaves]\n# tid pc count reason   (host arm pcs, no guest "
            "translation)\n");
    for (const auto& [tid, ts] : st.threads) {
        std::vector<std::pair<uint64_t, uint64_t>> all(ts.host_pcs.begin(), ts.host_pcs.end());
        std::sort(all.begin(), all.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });
        for (const auto& [pc, hits] : all) {
            const auto reason = ts.host_reasons.find(pc);
            fprintf(
                fh, "0x%llx 0x%llx %llu %s\n", static_cast<unsigned long long>(tid),
                static_cast<unsigned long long>(pc), static_cast<unsigned long long>(hits),
                noGuestReasonField(reason == ts.host_reasons.end() ? 0 : reason->second).c_str());
        }
    }

    // Which syscall a blocked thread was inside, for the host pcs that sit right
    // after a trap.  Taken from x16 per sample rather than from the code, because
    // Rosetta's guest-syscall dispatcher takes the number from the guest's own
    // register: one pc serves every blocking call the guest makes, and only the
    // register tells them apart.  Negative numbers are the mach traps.
    fprintf(fh, "\n[host_syscalls]\n# tid pc svc count   (pc is the insn after `svc #0x80`)\n");
    for (const auto& [tid, ts] : st.threads) {
        for (const auto& [pc, calls] : ts.host_syscalls) {
            for (const auto& [call, hits] : calls) {
                fprintf(fh, "0x%llx 0x%llx %d %llu\n", static_cast<unsigned long long>(tid),
                        static_cast<unsigned long long>(pc), call,
                        static_cast<unsigned long long>(hits));
            }
        }
    }

    fprintf(fh, "\n[stacks]\n# tid root;...;leaf count   (inclusive: these were ON the stack)\n");
    for (const auto& [tid, ts] : st.threads) {
        std::vector<std::pair<const std::vector<uint64_t>*, uint64_t>> all;
        all.reserve(ts.stacks.size());
        for (const auto& [stack, hits] : ts.stacks) {
            all.emplace_back(&stack, hits);
        }
        std::sort(all.begin(), all.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });
        for (const auto& [stack, hits] : all) {
            fprintf(fh, "0x%llx ", static_cast<unsigned long long>(tid));
            for (size_t i = 0; i < stack->size(); i++) {
                fprintf(fh, "%s0x%llx", i != 0 ? ";" : "",
                        static_cast<unsigned long long>((*stack)[i]));
            }
            fprintf(fh, " %llu\n", static_cast<unsigned long long>(hits));
        }
    }

    // Closes the record.  Only a window needs it: records are concatenated into
    // one file, and a SIGKILL during a write leaves a partial one at the end,
    // which a reader has to be able to drop rather than sum.
    if (windowStart >= 0) {
        fprintf(fh, "\nend_window %u\n", ctx->window_seq);
    }
}

// The cumulative profile: written whole every report interval, via a temporary
// and a rename, so a reader mid-run always sees a complete file.
void writeProfile(const AggCtx* ctx, const SamplerStats& st, double elapsed,
                  const std::string& path) {
    const std::string tmp = path + ".tmp";
    FILE* fh = fopen(tmp.c_str(), "wb");
    if (fh == nullptr) {
        return;
    }
    emitProfile(fh, ctx, st, elapsed, -1, 0);
    fclose(fh);
    if (rename(tmp.c_str(), path.c_str()) != 0) {
        unlink(tmp.c_str());
    }
}

// One window, appended to the series file.  Built in memory first so the record
// reaches the file in a single write: it is appended rather than renamed into
// place, so a reader can be part-way through the file at any moment.
void appendWindow(AggCtx* ctx, const SamplerStats& st, double elapsed, double windowStart,
                  double windowProfiled) {
    if (ctx->window_file == nullptr) {
        ctx->window_file = fopen((ctx->path + ".windows").c_str(), "wb");
        if (ctx->window_file == nullptr) {
            return;
        }
    }
    char* buf = nullptr;
    size_t len = 0;
    FILE* mem = open_memstream(&buf, &len);
    if (mem == nullptr) {
        return;
    }
    emitProfile(mem, ctx, st, elapsed, windowStart, windowProfiled);
    fclose(mem);
    fwrite(buf, 1, len, ctx->window_file);
    fflush(ctx->window_file);
    free(buf);
}

// Take one address the map does not explain and try to name its image.  Only
// ever called for addresses the target actually executed, so a module costs one
// lookup and everything else in it is free.
void learnAddress(AggCtx* ctx, uint64_t addr) {
    if (addr < kMinCodeAddr || moduleAt(ctx->modules, addr) != nullptr) {
        return;
    }
    const uint64_t slot = addr & ~(kImageGranularity - 1);
    if (ctx->unmapped.contains(slot)) {
        return;
    }
    // CHEAPEST TEST FIRST, and it matters more than it looks: most addresses
    // that reach here are not code at all but slots a frame-pointer walk
    // mistook for return addresses — 4347 of them in one 33 s client run.  Real
    // code is EXECUTABLE; an invented address points at the heap, the stack or
    // nothing.  One region query settles it, where the image search below costs
    // a 512-step header walk-back before it can fail.  Ordered the other way
    // round this cost the target ~2 million cross-task reads in a startup.
    //
    // Not "is it backed by a file": measured, the guest x86_64 shared cache
    // under Rosetta reports NO file for its text at all, so that test rejects
    // exactly the addresses dyld is needed for.
    uint64_t regionBase = 0;
    uint64_t regionSize = 0;
    if (!looksLikeCode(ctx->task, addr, &regionBase, &regionSize)) {
        ctx->unmapped.insert(slot);
        return;
    }
    if (resolveAddress(ctx->task, ctx->pid, addr, ctx->modules)) {
        return;
    }
    // Executable, but no header to walk back to: an image in the shared cache,
    // or one whose sampled address lies in a region that is not its first.  dyld
    // is the remaining authority.  Rate limited as a backstop: several new
    // images sampled at once need one ask between them, not one each.  Whatever
    // it still cannot explain is memoised below and never asks again.
    const double now = nowUs();
    if (ctx->asked_dyld && now - ctx->last_dyld_us < kDyldAskEveryS * 1e6) {
        return;  // deliberately not memoised: the next sighting gets a full try
    }
    ctx->last_dyld_us = now;
    ctx->asked_dyld = true;
    refreshDyldImages(ctx->task, ctx->modules);
    if (moduleAt(ctx->modules, addr) != nullptr) {
        return;
    }
    // Executable, no image header, unknown to dyld — but the kernel still knows
    // which file backs the mapping, and for these it is not JIT output at all:
    // Rosetta maps a 16 KB slice of /usr/libexec/rosetta/runtime (file offset
    // 0x20000, its guest-syscall dispatcher) next to every guest image, and a
    // thread blocked in a guest syscall is observed in exactly that slice.  On
    // the client it is the single largest host address in the profile.  Only the
    // mapping's own file offset can relate an address in it to the file, since
    // there is no header to walk back to, so it is recorded with the row.
    if (regionSize != 0) {
        char path[MAXPATHLEN];
        path[0] = '\0';
        const int len = proc_regionfilename(ctx->pid, regionBase, path, sizeof(path));
        if (len > 0) {
            path[std::min<size_t>(static_cast<size_t>(len), sizeof(path) - 1)] = '\0';
            if (path[0] != '\0' && !std::string_view(path).contains("dyld_shared_cache")) {
                registerImage(
                    ctx->modules,
                    GuestModule{
                        .base = regionBase, .size = regionSize, .slice = true, .path = path});
                g_modmap.learned.fetch_add(1, std::memory_order_relaxed);
                return;
            }
        }
    }
    // Executable, no image, and no file either: generated code, and
    // on this target that is most of what runs.  Recording the mapping itself
    // keeps it in the profile as its own row instead of collapsing into one
    // "unmapped" total, which is the difference between "44% of the thread is
    // somewhere we cannot name" and "44% of the thread is in this 11 KB of
    // it".  Nothing can be symbolised inside it, but an offset from a stable
    // base is still an identity that a disassembly can be pointed at.
    //
    // It starts after any image already known inside the region rather than at
    // the region's base.  One mapping can hold a small image and a large
    // unnamed remainder: wine's own loader is 8 KB of __TEXT at the bottom of
    // the region the hot generated code sits in, and keying this on the region
    // base would have replaced `wine` in the map with an entry spanning it.
    const uint64_t regionEnd = regionBase + regionSize;
    uint64_t lo = regionBase;
    if (const auto it = ctx->modules.upper_bound(addr); it != ctx->modules.begin()) {
        const auto& prev = std::prev(it)->second;
        if (prev.base >= regionBase && prev.base + prev.size > lo) {
            lo = prev.base + prev.size;
        }
    }
    if (regionSize != 0 && lo <= addr && addr < regionEnd) {
        char label[64];
        snprintf(label, sizeof(label), "anon@0x%llx", static_cast<unsigned long long>(lo));
        ctx->modules[lo] = GuestModule{
            .base = lo, .size = regionEnd - lo, .macho = false, .anon = true, .path = label};
        g_modmap.learned.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    ctx->unmapped.insert(slot);
}

// Is this host pc the instruction after a syscall, i.e. where a thread INSIDE a
// syscall is observed from outside?  That is the one thing worth knowing about an
// address in Rosetta's own code: a blocked thread parks on such a pc, and x16
// then still holds the number, which is the only way to name a call site whose
// number is dynamic (the guest-syscall dispatcher takes it from the guest's eax).
//
// One read per distinct address, once, on this thread; an address already tested
// is never read again, including the ones that are not syscall returns.
bool isSyscallReturn(AggCtx* ctx, uint64_t pc) {
    if (pc < 4) {
        return false;
    }
    const auto known = ctx->host_svc_return.find(pc);
    if (known != ctx->host_svc_return.end()) {
        return known->second;
    }
    uint32_t insn = 0;
    const bool is = readParent(ctx->task, pc - 4, &insn, sizeof(insn)) && insn == 0xD4001001;
    ctx->host_svc_return[pc] = is;
    return is;
}

// Consume everything the sampler has published.  Pure local work: no syscall
// touches the target unless an address turns up that no module explains.
// Samples land in `st`, the window being filled.  Every one is keyed by the
// thread it came from, so a run that re-latched keeps both threads rather than
// choosing between them.
void drainRing(AggCtx* ctx, SamplerStats& st) {
    while (true) {
        const uint64_t tail = g_ring.tail.load(std::memory_order_relaxed);
        const uint64_t head = g_ring.head.load(std::memory_order_acquire);
        // How full the handoff has ever been.  With no drops this is the only
        // thing that says whether the run had margin or was one hiccup from
        // losing samples.  Measured per record, not once per drain: this thread
        // is the only consumer, so the level only rises between drains and the
        // next drain would see that peak anyway — but a drain can BLOCK in the
        // middle, on a header walk-back or an ask to dyld, while the sampler
        // keeps pushing, and that is the growth a once-per-drain reading misses.
        // The loads are already here, so it costs nothing.
        ctx->ring_peak = std::max(ctx->ring_peak, head - tail);
        if (head - tail < 2) {
            return;
        }
        const uint64_t tid = g_ring.slots[tail & kRingMask].load(std::memory_order_relaxed);
        const uint64_t hdr = g_ring.slots[(tail + 1) & kRingMask].load(std::memory_order_relaxed);
        const auto kind = static_cast<SampleKind>((hdr >> 32) & 0xFF);
        const auto reason = static_cast<guest_pc::NoGuestReason>((hdr >> 24) & 0xFF);
        const bool inRange = ((hdr >> 16) & 1) != 0;
        const uint64_t depth = hdr & 0xFFFF;
        const size_t words = kind == SampleKind::Resolved        ? 3 + depth
                             : kind == SampleKind::NotTranslated ? 4
                                                                 : 2;

        {
            ThreadStats& ts = st.threads[tid];
            ts.samples++;
            st.samples++;
            switch (kind) {
                case SampleKind::Resolved: {
                    const uint64_t leaf =
                        g_ring.slots[(tail + 2) & kRingMask].load(std::memory_order_relaxed);
                    st.resolved++;
                    ts.resolved++;
                    ts.pcs[leaf]++;
                    learnAddress(ctx, leaf);
                    if (inRange) {
                        st.in_range++;
                        ts.in_range++;
                    }
                    if (ctx->unwind) {
                        st.unwound++;
                        st.frames += depth;
                        std::vector<uint64_t> stack;
                        stack.reserve(depth + 1);
                        for (uint64_t i = 0; i < depth; i++) {
                            const uint64_t frame = g_ring.slots[(tail + 3 + i) & kRingMask].load(
                                std::memory_order_relaxed);
                            stack.push_back(frame);
                            // A module can appear only as a CALLER, so frames
                            // are learned too, not just leaves.
                            learnAddress(ctx, frame);
                        }
                        stack.push_back(leaf);  // leaf last
                        ts.stacks[stack]++;
                    }
                    break;
                }
                case SampleKind::NotTranslated: {
                    const uint64_t host =
                        g_ring.slots[(tail + 2) & kRingMask].load(std::memory_order_relaxed);
                    const uint64_t x16 =
                        g_ring.slots[(tail + 3) & kRingMask].load(std::memory_order_relaxed);
                    st.not_translated++;
                    ts.host_pcs[host]++;
                    ts.host_reasons[host] |= static_cast<uint8_t>(1U << static_cast<int>(reason));
                    // Same lookup as a guest address: the map holds Mach-O
                    // images too, and dyld is the only thing that can name one
                    // inside the shared cache, which is where most of these are.
                    learnAddress(ctx, host);
                    // A pc right after a trap says the thread is inside a
                    // syscall, and x16 says which.  Bounded so a register that is
                    // not a syscall number cannot invent an entry.
                    const auto call = static_cast<int64_t>(x16);
                    if (call >= -1024 && call <= 4096 && isSyscallReturn(ctx, host)) {
                        ts.host_syscalls[host][static_cast<int32_t>(call)]++;
                    }
                    break;
                }
                case SampleKind::Unavailable:
                    st.unavailable++;
                    break;
            }
        }
        g_ring.tail.store(tail + words, std::memory_order_release);
    }
}

// How often the aggregator wakes.  Short, because draining is local work and a
// short interval keeps the ring shallow and a newly loaded module named within
// a fraction of a second.
constexpr long kDrainEveryNs = 100000000;  // 100 ms

// How often the sampler publishes its resolver-cache counters.
constexpr uint64_t kCacheSnapshotEvery = 1024;

void* aggregatorMain(void* raw) {
    auto* ctx = static_cast<AggCtx*>(raw);
    // Samples drain into the window; the run total is the windows merged.  Kept
    // this way round rather than as two independent tallies so the cumulative
    // profile is the sum of the window records by construction, which is the one
    // property a reader adding them up depends on.
    SamplerStats win;
    SamplerStats run;
    uint64_t baseSamples = 0;
    uint64_t baseNs = 0;
    double lastReport = nowUs();
    while (true) {
        const bool running = g_sampler_running.load(std::memory_order_relaxed);
        const bool flushing = g_sampler_flush_request.load(std::memory_order_relaxed) &&
                              !g_sampler_flush_done.load(std::memory_order_relaxed);
        drainRing(ctx, win);

        double startedAt = 0;
        double latchedUs = 0;
        bool latched = false;
        {
            const std::scoped_lock lock(g_latch.mu);
            latched = g_latch.latched || g_latch.tid != 0;
            startedAt = g_latch.started;
            latchedUs =
                g_latch.latched_us + (g_latch.latched_at != 0 ? nowUs() - g_latch.latched_at : 0);
        }
        const bool due = ctx->report_s > 0 && (nowUs() - lastReport) / 1e6 >= ctx->report_s;
        // A profile is only written once there IS a subject: an unlatched one
        // would be a blend of whatever the discovery sweeps saw.
        if (latched && (due || flushing || !running)) {
            lastReport = nowUs();
            const double elapsed = (nowUs() - startedAt) / 1e6;
            const uint64_t samplesNow = g_counters.samples.load(std::memory_order_relaxed);
            const uint64_t nsNow = g_counters.total_ns.load(std::memory_order_relaxed);
            win.samples_measured = samplesNow - baseSamples;
            win.total_us = static_cast<double>(nsNow - baseNs) / 1000.0;
            baseSamples = samplesNow;
            baseNs = nsNow;

            if (ctx->windows) {
                appendWindow(ctx, win, elapsed - ctx->window_started_s, ctx->window_started_s,
                             (latchedUs - ctx->window_latched_us) / 1e6);
                ctx->window_seq++;
            }
            ctx->window_started_s = elapsed;
            ctx->window_latched_us = latchedUs;

            mergeStats(run, win);
            win = SamplerStats{};
            writeProfile(ctx, run, elapsed, ctx->path);
        }
        if (flushing) {
            g_sampler_flush_done.store(true, std::memory_order_release);
        }
        if (!running) {
            if (ctx->window_file != nullptr) {
                fclose(ctx->window_file);
                ctx->window_file = nullptr;
            }
            return nullptr;
        }
        struct timespec ts{.tv_sec = 0, .tv_nsec = kDrainEveryNs};
        nanosleep(&ts, nullptr);
    }
}

void* samplerMain(void* raw) {
    auto* ctx = static_cast<SamplerCtx*>(raw);
    const guest_pc::Reader reader{samplerRead, ctx};
    guest_pc::Cache cache;

    mach_port_t latched = MACH_PORT_NULL;
    uint64_t latchedTid = 0;
    std::string latchedName;
    // Survives an unlatch, so a re-latch can tell "the same thread went quiet
    // and came back" from "the subject changed".
    uint64_t previousTid = 0;
    std::unordered_map<uint64_t, uint64_t> scores;  // mach thread id -> in-range hits
    double lastInRange = 0;
    double lastSweep = 0;
    uint64_t sweeps = 0;
    bool gaveUpLatching = false;

    // The only thing this ever prints: everything else belongs in the profile,
    // not in the game's log.
    const double sampleHz =
        ctx->cfg.interval_us != 0 ? 1e6 / static_cast<double>(ctx->cfg.interval_us) : 0.0;
    const double sweepHz = ctx->cfg.sweep_interval_us != 0
                               ? 1e6 / static_cast<double>(ctx->cfg.sweep_interval_us)
                               : 0.0;
    if (ctx->cfg.guest_range_pinned) {
        fprintf(stdout,
                "[rosettax87] X87_SAMPLE: sampling to '%s' at %.0f Hz (sweeps %.0f Hz), latching "
                "onto the thread that runs guest-range=[0x%llx,0x%llx)\n",
                ctx->cfg.path.c_str(), sampleHz, sweepHz,
                static_cast<unsigned long long>(ctx->cfg.guest_lo),
                static_cast<unsigned long long>(ctx->cfg.guest_hi));
    } else {
        fprintf(stdout,
                "[rosettax87] X87_SAMPLE: sampling to '%s' at %.0f Hz (sweeps %.0f Hz), latching "
                "onto the thread that runs the main image\n",
                ctx->cfg.path.c_str(), sampleHz, sweepHz);
    }
    fflush(stdout);

    double started = nowUs();
    double nextTick = started;
    uint64_t ticks = 0;
    for (;;) {
        const double tickStart = nowUs();
        if (latched != MACH_PORT_NULL) {
            // Steady state: one thread, one thread_get_state, no task_threads,
            // no port churn and no thread_info.
            bool inRange = false;
            sampleThread(ctx, reader, cache, latched, latchedTid, true, inRange);
            const double now = nowUs();
            if (inRange) {
                lastInRange = now;
            } else if (now - lastInRange > kUnlatchAfterS * 1e6) {
                const double quiet = (now - lastInRange) / 1e6;
                fprintf(stdout,
                        "[rosettax87] X87_SAMPLE: unlatched from thread 0x%llx, %.1f s with "
                        "nothing in the main image; searching again\n",
                        static_cast<unsigned long long>(latchedTid), quiet);
                fflush(stdout);
                g_latch.add((now - started) / 1e6,
                            "unlatched 0x%llx after %.1f s outside the image",
                            static_cast<unsigned long long>(latchedTid), quiet);
                {
                    const std::scoped_lock lock(g_latch.mu);
                    g_latch.latched = false;
                    if (g_latch.latched_at != 0) {
                        g_latch.latched_us += nowUs() - g_latch.latched_at;
                        g_latch.latched_at = 0;
                    }
                }
                latchedName.clear();
                mach_port_deallocate(mach_task_self(), latched);
                latched = MACH_PORT_NULL;
                previousTid = latchedTid;
                latchedTid = 0;
                scores.clear();
                sweeps = 0;
                lastSweep = 0;
            }
        } else if (!gaveUpLatching || nowUs() - lastSweep >= kIdleSweepS * 1e6) {
            lastSweep = nowUs();
            thread_act_array_t threads = nullptr;
            mach_msg_type_number_t count = 0;
            if (task_threads(ctx->task, &threads, &count) != KERN_SUCCESS) {
                break;
            }
            if (sweeps >= kDiscoveryGiveUp && !gaveUpLatching) {
                gaveUpLatching = true;
                // Two different failures reach here and they need different
                // fixes, so name the one that happened rather than just the
                // range: without a main image the range is only a fallback.
                if (!ctx->cfg.guest_range_pinned && !ctx->image_found) {
                    fprintf(stdout,
                            "[rosettax87] X87_SAMPLE: FAILED to latch after %llu sweeps: no "
                            "main image was found in the guest (no PE or Mach-O header above "
                            "any sampled pc), and nothing ran in the fallback range "
                            "[0x%llx,0x%llx). NO PROFILE WILL BE WRITTEN. Set "
                            "X87_GUEST_RANGE=lo-hi to say where to look. Still searching, one "
                            "sweep per %.0f s.\n",
                            static_cast<unsigned long long>(sweeps),
                            static_cast<unsigned long long>(ctx->cfg.guest_lo),
                            static_cast<unsigned long long>(ctx->cfg.guest_hi), kIdleSweepS);
                } else {
                    fprintf(
                        stdout,
                        "[rosettax87] X87_SAMPLE: FAILED to latch after %llu sweeps: no "
                        "thread ran in [0x%llx,0x%llx), which came from %s. NO PROFILE WILL "
                        "BE WRITTEN. Still searching, one sweep per %.0f s.\n",
                        static_cast<unsigned long long>(sweeps),
                        static_cast<unsigned long long>(ctx->cfg.guest_lo),
                        static_cast<unsigned long long>(ctx->cfg.guest_hi),
                        ctx->cfg.guest_range_pinned ? "X87_GUEST_RANGE" : "the detected main image",
                        kIdleSweepS);
                }
                fflush(stdout);
            }
            sweeps++;
            mach_port_t best = MACH_PORT_NULL;
            uint64_t bestTid = 0;
            uint64_t bestScore = 0;
            for (mach_msg_type_number_t i = 0; i < count; i++) {
                const uint64_t tid = machThreadId(threads[i]);
                bool inRange = false;
                // record=false: a discovery sample is a probe for "which thread
                // is worth having", never part of the profile.
                sampleThread(ctx, reader, cache, threads[i], tid, false, inRange);
                if (inRange && tid != 0) {
                    const uint64_t score = ++scores[tid];
                    if (score >= bestScore) {
                        bestScore = score;
                        best = threads[i];
                        bestTid = tid;
                    }
                }
            }
            if (best != MACH_PORT_NULL && sweeps >= kDiscoverySweeps) {
                // Re-latching onto a different thread used to throw away
                // everything captured so far, because a cumulative profile had
                // to describe one thread or it described nothing.  Every sample
                // is keyed by its thread and every report interval is now its
                // own file, so neither is true: the old thread's samples stay
                // under the old thread, in the windows they were taken in, and
                // the run reads as a timeline of which thread was the subject
                // when.  Throwing them away was only ever a way to avoid
                // blending two threads into one number.
                if (previousTid != 0 && bestTid != previousTid) {
                    fprintf(stdout,
                            "[rosettax87] X87_SAMPLE: subject changed, thread 0x%llx takes over "
                            "from 0x%llx; both stay in the profile under their own thread\n",
                            static_cast<unsigned long long>(bestTid),
                            static_cast<unsigned long long>(previousTid));
                    g_latch.add((nowUs() - started) / 1e6, "subject changed from 0x%llx",
                                static_cast<unsigned long long>(previousTid));
                }
                latched = best;
                latchedTid = bestTid;
                latchedName = machThreadName(best);
                // The unlatch window is measured from the last in-range sample,
                // so it has to start now rather than at whatever the previous
                // latch left behind.
                lastInRange = nowUs();
                // Latching disproves "nothing ever runs there", so drop the
                // throttle: if this thread later goes quiet, re-discovery has
                // to run at full speed, not at one sweep per kIdleSweepS.
                gaveUpLatching = false;

                {
                    const std::scoped_lock lock(g_latch.mu);
                    g_latch.latched = true;
                    g_latch.tid = latchedTid;
                    g_latch.name = latchedName;
                    g_latch.started = started;
                    g_latch.latched_at = nowUs();
                }
                g_latch.add((nowUs() - started) / 1e6,
                            "latched 0x%llx%s%s%s seen_in_range=%llu candidates=%zu sweeps=%llu",
                            static_cast<unsigned long long>(bestTid),
                            latchedName.empty() ? "" : " (", latchedName.c_str(),
                            latchedName.empty() ? "" : ")",
                            static_cast<unsigned long long>(bestScore), scores.size(),
                            static_cast<unsigned long long>(sweeps));
                fprintf(stdout,
                        "[rosettax87] X87_SAMPLE: LATCHED onto thread 0x%llx%s%s%s after %llu "
                        "sweeps (seen in range %llu times, %zu candidates); profiling only "
                        "that thread from here\n",
                        static_cast<unsigned long long>(bestTid), latchedName.empty() ? "" : " (\"",
                        latchedName.c_str(), latchedName.empty() ? "" : "\")",
                        static_cast<unsigned long long>(sweeps),
                        static_cast<unsigned long long>(bestScore), scores.size());
                fflush(stdout);
            }
            for (mach_msg_type_number_t i = 0; i < count; i++) {
                if (threads[i] != latched) {
                    mach_port_deallocate(mach_task_self(), threads[i]);
                }
            }
            vm_deallocate(mach_task_self(), reinterpret_cast<vm_address_t>(threads),
                          count * sizeof(thread_act_t));
        }

        // Publish what the resolver cache is doing, for the profile header.
        // Once in a while rather than per sample: it is a diagnostic, and the
        // cache itself belongs to this thread.
        if (++ticks % kCacheSnapshotEvery == 0) {
            const auto& cs = cache.stats();
            g_counters.cache_pc_hits.store(cs.pc_hits, std::memory_order_relaxed);
            g_counters.cache_fragment_hits.store(cs.fragment_hits, std::memory_order_relaxed);
            g_counters.cache_negative_hits.store(cs.negative_hits, std::memory_order_relaxed);
            g_counters.cache_misses.store(cs.misses, std::memory_order_relaxed);
            g_counters.cache_stale.store(cs.stale, std::memory_order_relaxed);
        }

        // Pace against a deadline instead of sleeping the whole interval after
        // the work.  A sample costs tens of microseconds and the kernel adds a
        // leeway proportional to the requested wait (measured on this machine:
        // 1000 us -> 1263, 100 us -> 132), so sleeping the full interval lands
        // well below the requested rate: a 1 kHz run realized 754 Hz.  Asking
        // only for what is left until the next deadline converges on the rate
        // that was actually asked for.
        const uint64_t period =
            latched != MACH_PORT_NULL ? ctx->cfg.interval_us : ctx->cfg.sweep_interval_us;
        if (nowUs() - tickStart > static_cast<double>(period)) {
            g_counters.overrun_ticks.fetch_add(1, std::memory_order_relaxed);
        }
        nextTick += static_cast<double>(period);
        const double now = nowUs();
        if (now >= nextTick) {
            // The work outran the period.  Sample again immediately, but move
            // the deadline up: catching up in a burst would sample one part of
            // the timeline harder than the rest.
            g_counters.missed_ticks.fetch_add(1, std::memory_order_relaxed);
            nextTick = now;
        } else {
            const auto waitUs = static_cast<uint64_t>(nextTick - now);
            struct timespec ts{static_cast<time_t>(waitUs / 1000000),
                               static_cast<long>((waitUs % 1000000) * 1000)};
            nanosleep(&ts, nullptr);
        }
    }

    return nullptr;
}

bool writeParent(mach_port_t task, uint64_t addr, const void* src, size_t size) {
    if (size == 0) {
        return true;
    }
    return mach_vm_write(task, addr, reinterpret_cast<vm_offset_t>(const_cast<void*>(src)), size) ==
           KERN_SUCCESS;
}

// Translate-path wrappers around readParent/writeParent, kept separate so
// the reporter's vm/req metric counts only this path (the sampler thread
// calls readParent directly).
bool readTranslate(mach_port_t task, uint64_t addr, void* dst, size_t size) {
    if (size == 0) {
        return true;
    }
    g_statVmSyscalls.fetch_add(1, std::memory_order_relaxed);
    return readParent(task, addr, dst, size);
}

bool writeTranslate(mach_port_t task, uint64_t addr, const void* src, size_t size) {
    if (size == 0) {
        return true;
    }
    g_statVmSyscalls.fetch_add(1, std::memory_order_relaxed);
    return writeParent(task, addr, src, size);
}

// Allocate a parent-side replacement buffer of size `newCap`, copy parent's
// existing live bytes, then append `tailSize` bytes from `tail`. On success
// returns the parent VA of the new buffer. On any failure deallocates and
// returns 0.
mach_vm_address_t allocAndAppendInParent(mach_port_t parentTask, uint64_t origAddr,
                                         uint64_t origLive, uint64_t newCap, const void* tail,
                                         uint64_t tailSize) {
    // Round up to page granularity.
    newCap = (newCap + 0xFFF) & ~static_cast<uint64_t>(0xFFF);
    mach_vm_address_t parentNew = 0;
    g_statVmSyscalls.fetch_add(1, std::memory_order_relaxed);
    if (mach_vm_allocate(parentTask, &parentNew, newCap, VM_FLAGS_ANYWHERE) != KERN_SUCCESS) {
        return 0;
    }
    if (origLive > 0) {
        std::vector<uint8_t> stash(origLive);
        if (!readTranslate(parentTask, origAddr, stash.data(), origLive) ||
            !writeTranslate(parentTask, parentNew, stash.data(), origLive)) {
            mach_vm_deallocate(parentTask, parentNew, newCap);
            return 0;
        }
    }
    if (tailSize > 0) {
        if (!writeTranslate(parentTask, parentNew + origLive, tail, tailSize)) {
            mach_vm_deallocate(parentTask, parentNew, newCap);
            return 0;
        }
    }
    return parentNew;
}

// Run Translator and write its output back to parent's TR. Returns Some(N)
// when translation produced a result and the write-back path completed;
// otherwise returns None (the stub falls through to stock translate_insn).
TranslateOutcome processTranslateRequest(mach_port_t parentTask, const TranslateRequest& req) {
    TranslateOutcome out{.reply_some = false, .value = 0};

    // X87_ALWAYS_NONE: short-circuit before any cross-process I/O.  The stub
    // sees a None reply, falls through to STASH, and stock translates the
    // op.  Hook + IPC mechanics still exercise (so we can A/B it against
    // X87_DISABLE_HOOK=1, which skips the hook entirely).  If a real
    // freeze repros under DISABLE_HOOK=0 + ALWAYS_NONE=1, the bug is in
    // the marshalling itself; if not, it's in our emitted code.
    if (g_rosetta_config != nullptr && g_rosetta_config->loader_always_none != 0U) {
        return out;
    }

    constexpr uint64_t kMaxNumInstrs = 0x10000;
    if (req.num_instrs == 0 || req.num_instrs > kMaxNumInstrs) {
        return out;
    }
    if (req.insn_idx >= req.num_instrs) {
        return out;
    }

    // Read parent's TR. Value-initialised local; we sterilise its list
    // pointers before scope end so `~TransactionalList` runs `::operator
    // delete(nullptr)` (a no-op) instead of freeing arbitrary parent VAs.
    // Value-init (not plain default-init) so the reverse-engineered tail past
    // stock's real 0x268 size — which we deliberately do NOT read from the
    // tracee — is deterministic zero rather than uninitialised.
    TranslationResult tr{};
    // Read only the stock-sized TR from the tracee; x87_cache lives in our own
    // per-thread map (keyed by TR address), not in the tracee's heap.
    if (!readTranslate(parentTask, req.tr_addr, &tr, kStockTRSize)) {
        return out;
    }
    {
        std::scoped_lock lk(g_x87CacheMu);
        tr.x87_cache = g_x87Cache[req.tr_addr];  // default-constructs on first use
    }

    // Any None reply produced WITHOUT running the translator (the early
    // failure returns below) must not leave the persisted per-TR X87Cache
    // trusting mid-run registers: stock translates the op itself and
    // clobbers the GPRs the cache thinks are holding TOP/base, so the next
    // sidecar-translated op would miscompile.  The translator's own None
    // paths invalidate in its default case; this guard covers every bypass
    // in one place.
    struct CacheBypassGuard {
        uint64_t tr_addr;
        bool ran_translator = false;
        ~CacheBypassGuard() {
            if (!ran_translator) {
                std::scoped_lock lk(g_x87CacheMu);
                auto it = g_x87Cache.find(tr_addr);
                if (it != g_x87Cache.end()) {
                    it->second.invalidate();
                    it->second.prev_block = nullptr;
                }
            }
        }
    } _bypass_guard{.tr_addr = req.tr_addr};

    TransactionalList<Fixup>* lists[kListCount] = {
        &tr.external_fixups, &tr.internal_fixups,  &tr._fixups,
        &tr.field_B0,        &tr.dyld_stub_fixups, &tr.field_1A8,
    };

    struct Sterilizer {
        TransactionalList<Fixup>** ls;
        ~Sterilizer() {
            for (size_t i = 0; i < kListCount; i++) {
                ls[i]->begin = ls[i]->end = ls[i]->end_cap = nullptr;
                ls[i]->_size = 0;
            }
        }
    } _sterilizer{lists};

    // Snapshot parent-side state we need for write-back.
    uint32_t* const origInsnData = tr.insn_buf.data;
    uint64_t const origInsnEnd = tr.insn_buf.end;
    uint64_t const origInsnCap = tr.insn_buf.end_cap;
    uint32_t const origInsnUseHeap = tr.insn_buf.use_heap;
    ThreadContextOffsets* const origTCO = tr.thread_context_offsets;

    struct ListBackup {
        Fixup* begin;
        Fixup* end;
        Fixup* end_cap;
        uint64_t _size;
    } origLists[kListCount];
    for (size_t i = 0; i < kListCount; i++) {
        origLists[i] = {.begin = lists[i]->begin,
                        .end = lists[i]->end,
                        .end_cap = lists[i]->end_cap,
                        ._size = lists[i]->_size};
    }

    // IR array: serve from the per-block cache when possible (see the
    // g_irCache comment for the reuse conditions and the ABA argument);
    // otherwise read it in full from the tracee.
    IRCacheEntry& irc = g_irCache[req.tr_addr];
    {
        const bool noIrCache =
            g_rosetta_config != nullptr && g_rosetta_config->loader_no_ir_cache != 0U;
        // Reuse only for a request at or past the frontier the previous
        // reply set: stock walks a block front to back, so anything behind
        // it is a pass started over, whose IR may have been re-decoded into
        // the same array.
        bool reuse = !noIrCache && irc.block == req.block && irc.instr_array == req.instr_array &&
                     irc.num_instrs == req.num_instrs && req.insn_idx >= irc.next_idx;
        if (reuse) {
            // Probe a window, not one instruction: the translator reads the
            // whole run's lookahead (fusions, the IR pipeline, bridging via
            // flag_liveness), and flag_liveness is recomputed from the
            // block's successors on every decode, so the cached tail can go
            // stale without the guest code changing.
            constexpr uint64_t kProbeWindow = 32;
            IRInstr probe[kProbeWindow];
            uint64_t win = req.num_instrs - req.insn_idx;
            if (win > kProbeWindow) {
                win = kProbeWindow;
            }
            if (!readTranslate(parentTask, req.instr_array + req.insn_idx * sizeof(IRInstr), probe,
                         win * sizeof(IRInstr))) {
                return out;
            }
            reuse = std::memcmp(probe, &irc.ir[req.insn_idx], win * sizeof(IRInstr)) == 0;
        }
        if (reuse) {
            g_statIrHits.fetch_add(1, std::memory_order_relaxed);
        } else {
            irc.ir.resize(req.num_instrs);
            if (!readTranslate(parentTask, req.instr_array, irc.ir.data(),
                         req.num_instrs * sizeof(IRInstr))) {
                // Don't leave a half-valid entry behind: the vector was
                // already resized, so stale keys could pair with the wrong
                // length and index out of bounds on a later request.
                irc = IRCacheEntry{};
                return out;
            }
            irc.block = req.block;
            irc.instr_array = req.instr_array;
            irc.num_instrs = req.num_instrs;
            irc.hash_valid = false;
            g_statIrMisses.fetch_add(1, std::memory_order_relaxed);
        }
        // Conservative until the reply is known; the Some path below moves it.
        irc.next_idx = req.insn_idx + 1;
    }
    IRInstr* const localIR = irc.ir.data();

    if (origTCO == nullptr) {
        return out;
    }
    ThreadContextOffsets localTCO{};
    {
        const bool noTcoCache =
            g_rosetta_config != nullptr && g_rosetta_config->loader_no_tco_cache != 0U;
        const uint64_t tcoAddr = reinterpret_cast<uint64_t>(origTCO);
        auto it = noTcoCache ? g_tcoCache.end() : g_tcoCache.find(tcoAddr);
        if (it != g_tcoCache.end()) {
            localTCO = it->second;
            g_statTcoHits.fetch_add(1, std::memory_order_relaxed);
        } else {
            if (!readTranslate(parentTask, tcoAddr, &localTCO, sizeof(localTCO))) {
                return out;
            }
            if (!noTcoCache) {
                // Canary for the constants assumption: all TCOs in a process
                // must describe the same layout.
                if (!g_tcoCache.empty() &&
                    std::memcmp(&g_tcoCache.begin()->second, &localTCO, sizeof(localTCO)) != 0) {
                    fprintf(stdout,
                            "[rosettax87] WARNING: ThreadContextOffsets at 0x%llx differs from "
                            "cached layout; TCO cache assumption broken, rerun with "
                            "X87_NO_TCO_CACHE=1\n",
                            static_cast<unsigned long long>(tcoAddr));
                    fflush(stdout);
                }
                g_tcoCache.emplace(tcoAddr, localTCO);
            }
        }
    }

    // x87_cache (OPT-1) was loaded from g_x87Cache above, not from the tracee.
    // On the first request for a TR address it default-constructs; Translator's
    // cache.invalidate() converges it on the first block mismatch.

    // Set up local insn_buf with capacity ≥ parent's. Critical: end starts at
    // origInsnEnd so Translator's emit/fixup offsets count in the SAME
    // coordinate space the parent uses (`data + insn_offset`). If we started
    // end at 0, fixups referencing emitted bytes would patch into parent's
    // pre-existing prologue bytes when stock later applies them — corruption
    // that crashes parent with EXC_BAD_INSTRUCTION.
    //
    // use_heap=1 ensures grow() picks calloc and skips its munmap-of-old-
    // pointer branch (the "old" pointer would otherwise be foreign memory).
    std::vector<uint8_t> localInsnVec(std::max<uint64_t>(origInsnCap, 0x4000));
    tr.insn_buf.data = reinterpret_cast<uint32_t*>(localInsnVec.data());
    tr.insn_buf.end = origInsnEnd;
    tr.insn_buf.end_cap = localInsnVec.size();
    tr.insn_buf.use_heap = 1;
    for (auto& list : lists) {
        list->begin = list->end = list->end_cap = nullptr;
        list->_size = 0;
    }
    tr.thread_context_offsets = &localTCO;

    if (g_rosetta_config != nullptr && g_rosetta_config->loader_log_ops != 0U) {
        const uint16_t op = localIR[req.insn_idx].opcode();
        const char* name = (op < kOpcodeNames.size()) ? kOpcodeNames[op] : "?";
        fprintf(stdout,
                "[rosettax87] op %s (0x%x) idx=%lld/%lld gpr=%08x fpr=%08x "
                "unocc=%08x pinned=%08x\n",
                name, static_cast<unsigned>(op), static_cast<long long>(req.insn_idx),
                static_cast<long long>(req.num_instrs), tr.free_gpr_mask, tr.free_fpr_mask,
                tr._unoccupied_temporary_fprs_for_xmm_scalars, tr._pinned_temporary_scalars);
        fflush(stdout);
    }

    dumpBlockIfNew(parentTask, reinterpret_cast<uint64_t>(tr.ir_module_data), req.block,
                   localIR, req.num_instrs);

    // X87_STOCK_HASH_LIST / X87_STOCK_OPS hand whole blocks to stock, by
    // IR-content hash (profile::hash_ir_stream, the key the bridge and
    // rollback lists use) or by contained opcode.  Under wow64 the sidecar
    // sees host-side PCs only, so content is the one per-block selector
    // that can target a guest module's code.  Requests arrive on
    // (re)translation, never per execution, so this is cold path; the hash
    // is still computed once per IR-cache fill rather than per request.
    // X87_LOG_HASH_LIST writes an uptime-stamped line per request for the
    // listed blocks (the clock WINEDEBUG=+timestamp prints), so a crash
    // moment in a wine log can be put next to the block's last translation.
    bool stock_hash_hit = false;
    const bool haveStockHashes =
        g_rosetta_config != nullptr && !g_rosetta_config->x87_stock_hash_list.empty();
    const bool haveStockOps =
        g_rosetta_config != nullptr && !g_rosetta_config->x87_stock_ops.empty();
    const bool haveLogHashes =
        g_rosetta_config != nullptr && !g_rosetta_config->x87_log_hash_list.empty();
    if (haveStockHashes || haveStockOps || haveLogHashes) {
        if (!irc.hash_valid) {
            irc.hash = profile::hash_ir_stream(localIR, req.num_instrs);
            irc.hash_valid = true;
        }
        const uint64_t block_hash = irc.hash;
        if (haveLogHashes &&
            std::ranges::binary_search(g_rosetta_config->x87_log_hash_list, block_hash)) {
            static int log_hits = 0;
            ++log_hits;
            if (log_hits <= 1000 || log_hits % 100 == 0) {
                const double up =
                    static_cast<double>(clock_gettime_nsec_np(CLOCK_UPTIME_RAW)) / 1e9;
                if (FILE* df = diagLog()) {
                    fprintf(df, "[x87trace] up=%.3f hit#%d hash=0x%016llx idx=%llu/%llu tr=%llx\n",
                            up, log_hits, static_cast<unsigned long long>(block_hash),
                            static_cast<unsigned long long>(req.insn_idx),
                            static_cast<unsigned long long>(req.num_instrs),
                            static_cast<unsigned long long>(req.tr_addr));
                    fflush(df);
                }
            }
        }
        if (haveStockHashes) {
            stock_hash_hit =
                std::ranges::binary_search(g_rosetta_config->x87_stock_hash_list, block_hash);
        }
        if (!stock_hash_hit && haveStockOps) {
            for (uint64_t i = 0; i < req.num_instrs && !stock_hash_hit; ++i) {
                stock_hash_hit = std::ranges::binary_search(g_rosetta_config->x87_stock_ops,
                                                            localIR[i].opcode());
            }
        }
        if (stock_hash_hit) {
            static int stock_hits = 0;
            ++stock_hits;
            if (stock_hits <= 5 || stock_hits % 1000 == 0) {
                char line[160];
                snprintf(line, sizeof(line), "[x87stock] hit #%d hash=0x%016llx idx=%llu/%llu\n",
                         stock_hits, static_cast<unsigned long long>(block_hash),
                         static_cast<unsigned long long>(req.insn_idx),
                         static_cast<unsigned long long>(req.num_instrs));
                fputs(line, stdout);
                fflush(stdout);
                if (FILE* df = diagLog()) {
                    fputs(line, df);
                    fflush(df);
                }
            }
        }
    }

    // On a stock hit ran_translator stays false and the bypass guard
    // invalidates the persisted X87Cache, which is what a None reply that
    // skipped the translator needs.
    std::optional<int64_t> result;
    if (!stock_hash_hit) {
        _bypass_guard.ran_translator = true;
        result = Translator::translate_instruction(
            &tr, reinterpret_cast<IRBlock*>(req.block), localIR,
            static_cast<int64_t>(req.num_instrs), static_cast<int64_t>(req.insn_idx));
        if (result.has_value()) {
            irc.next_idx = result.value();
        }
    }

    // Capture growth state. If insn_buf grew, Translator's grow() abandoned
    // localInsnVec for a calloc'd buffer (we own that and must free it).
    auto* const localInsnData = reinterpret_cast<uint8_t*>(tr.insn_buf.data);
    bool const insnGrew = (localInsnData != localInsnVec.data());
    uint64_t const insnEmitted = tr.insn_buf.end - origInsnEnd;
    Fixup* localPushed[kListCount];
    uint64_t localPushedBytes[kListCount];
    for (size_t i = 0; i < kListCount; i++) {
        localPushed[i] = lists[i]->begin;
        localPushedBytes[i] = static_cast<uint64_t>(reinterpret_cast<uint8_t*>(lists[i]->end) -
                                                    reinterpret_cast<uint8_t*>(lists[i]->begin));
    }
    struct LocalCleanup {
        uint8_t* insn_buf;  // null if Translator never grew (vec owns)
        Fixup* lists[kListCount];
        ~LocalCleanup() {
            if (insn_buf) {
                free(insn_buf);
            }
            for (auto& list : lists) {
                if (list) {
                    ::operator delete(list);
                }
            }
        }
    } _cleanup{.insn_buf = insnGrew ? localInsnData : nullptr,
               .lists = {localPushed[0], localPushed[1], localPushed[2], localPushed[3],
                         localPushed[4], localPushed[5]}};

    if (g_rosetta_config != nullptr && g_rosetta_config->loader_dump_emit != 0U &&
        result.has_value() && insnEmitted > 0) {
        // Hexdump the emitted AArch64 words of this request (guest pc +
        // opcode + tracee buffer VA), so a bad encoding can be found by
        // disassembling the dump offline.  EXTREMELY high volume.
        const uint16_t op = localIR[req.insn_idx].opcode();
        const char* name = (op < kOpcodeNames.size()) ? kOpcodeNames[op] : "?";
        fprintf(stdout, "[rosettax87] EMIT pc=%08x op=%s idx=%lld/%lld at=%llx words=%llu:",
                localIR[req.insn_idx].pc, name, static_cast<long long>(req.insn_idx),
                static_cast<long long>(req.num_instrs),
                reinterpret_cast<unsigned long long>(origInsnData) + origInsnEnd,
                static_cast<unsigned long long>(insnEmitted / 4));
        const auto* words = reinterpret_cast<const uint32_t*>(localInsnData + origInsnEnd);
        for (uint64_t w = 0; w < insnEmitted / 4; w++) {
            fprintf(stdout, " %08x", words[w]);
        }
        fprintf(stdout, "\n");
        fflush(stdout);
    }

    // We always write the TR back, even on None — Translator's default case
    // (and other unhandled paths) calls cache.invalidate() and resets the
    // scratch register masks; if we don't propagate those, parent ends up
    // with stale `cache.gprs_valid=1` from the previous Some translation
    // while stock's now-running translate_insn (for the unhandled opcode)
    // happily clobbers the GPRs the cache claims are still holding TOP /
    // base. The next x87 op would then trust the cache and emit wrong code.
    // Restore parent VAs in TR before any conditional data writes below;
    // the data-write path will re-pivot insn_buf/list pointers if grow
    // happened.
    tr.insn_buf.data = origInsnData;
    tr.insn_buf.end = origInsnEnd;
    tr.insn_buf.end_cap = origInsnCap;
    tr.insn_buf.use_heap = origInsnUseHeap;
    tr.thread_context_offsets = origTCO;
    for (size_t i = 0; i < kListCount; i++) {
        lists[i]->begin = origLists[i].begin;
        lists[i]->end = origLists[i].end;
        lists[i]->end_cap = origLists[i].end_cap;
        lists[i]->_size = origLists[i]._size;
    }

    if (result.has_value()) {
        // Write insn_buf delta bytes (the region Translator emitted, at
        // offsets [origInsnEnd .. origInsnEnd+emitted]) back to parent.
        // Two cases:
        //  - No grow + fits in parent's cap → mach_vm_write the tail
        //    in place.
        //  - Grow OR doesn't fit → allocate a parent-side replacement,
        //    copy parent's existing [0..origInsnEnd] bytes over, then
        //    append our emitted slice, and pivot TR.insn_buf.data onto
        //    it.
        uint64_t finalInsnEnd = origInsnEnd + insnEmitted;
        uint32_t* finalInsnData = origInsnData;
        uint64_t finalInsnCap = origInsnCap;
        if (!insnGrew && finalInsnEnd <= origInsnCap) {
            if (insnEmitted > 0) {
                if (!writeTranslate(parentTask, reinterpret_cast<uint64_t>(origInsnData) + origInsnEnd,
                              localInsnData + origInsnEnd, insnEmitted)) {
                    return out;
                }
            }
        } else {
            uint64_t newCap = std::max(origInsnCap * 2, finalInsnEnd);
            mach_vm_address_t parentNew = allocAndAppendInParent(
                parentTask, reinterpret_cast<uint64_t>(origInsnData), origInsnEnd, newCap,
                localInsnData + origInsnEnd, insnEmitted);
            if (parentNew == 0) {
                return out;
            }
            finalInsnData = reinterpret_cast<uint32_t*>(parentNew);
            finalInsnCap = (newCap + 0xFFF) & ~static_cast<uint64_t>(0xFFF);
        }
        tr.insn_buf.data = finalInsnData;
        tr.insn_buf.end = finalInsnEnd;
        tr.insn_buf.end_cap = finalInsnCap;

        // Append each list's pushed entries to parent — same fits-or-grow
        // split.
        for (size_t i = 0; i < kListCount; i++) {
            const auto& orig = origLists[i];
            uint64_t parentLive =
                reinterpret_cast<uint8_t*>(orig.end) - reinterpret_cast<uint8_t*>(orig.begin);
            uint64_t parentCap =
                reinterpret_cast<uint8_t*>(orig.end_cap) - reinterpret_cast<uint8_t*>(orig.begin);
            uint64_t added = localPushedBytes[i];
            uint64_t newLive = parentLive + added;

            if (newLive <= parentCap) {
                if (added > 0) {
                    if (!writeTranslate(parentTask, reinterpret_cast<uint64_t>(orig.end), localPushed[i],
                                  added)) {
                        return out;
                    }
                }
                lists[i]->end =
                    reinterpret_cast<Fixup*>(reinterpret_cast<uint8_t*>(orig.begin) + newLive);
            } else {
                uint64_t newCap = std::max(parentCap * 2, newLive);
                mach_vm_address_t parentNew =
                    allocAndAppendInParent(parentTask, reinterpret_cast<uint64_t>(orig.begin),
                                           parentLive, newCap, localPushed[i], added);
                if (parentNew == 0) {
                    return out;
                }
                uint64_t roundedCap = (newCap + 0xFFF) & ~static_cast<uint64_t>(0xFFF);
                lists[i]->begin = reinterpret_cast<Fixup*>(parentNew);
                lists[i]->end = reinterpret_cast<Fixup*>(parentNew + newLive);
                lists[i]->end_cap = reinterpret_cast<Fixup*>(parentNew + roundedCap);
            }
        }
    }

    // Persist OPT-1's cross-instruction cache in our own per-thread map (not the
    // tracee's TR). Always — even on None, Translator's default case calls
    // cache.invalidate() and resets scratch masks; dropping that would leave the
    // next call trusting stale gprs_valid state.
    {
        std::scoped_lock lk(g_x87CacheMu);
        g_x87Cache[req.tr_addr] = tr.x87_cache;
    }

    // Write back only the stock-sized TR (propagates scratch-mask updates and
    // any pivoted buffer pointers from the Some path above). x87_cache is NOT
    // written to the tracee — it lives in g_x87Cache — so the tracee's TR needs
    // no enlargement and the M2 install needs no TR-size patch.
    if (!writeTranslate(parentTask, req.tr_addr, &tr, kStockTRSize)) {
        return out;
    }

    if (result.has_value()) {
        out.reply_some = true;
        out.value = result.value();
    } else {
        // The stub's FILTER prologue routes only x87 opcodes to us, so
        // reaching nullopt means the dispatcher returned nullopt for an
        // x87 op.  Two cases:
        //   1. Deliberate fall-through (fxsave/fxrstor): explicit `case`
        //      in Translator.cpp; we don't inline because of the 8 × f80
        //      ST slots that would inherit frstor's eager-conversion
        //      regression.  Stock translates them via shared x22.
        //   2. Forgot-to-handle: an x87 op without a translate_* and
        //      without an entry in kKnownFallThrough below.  This is the
        //      discoverability signal — emit a loud UNHANDLED line so a
        //      future helper-using opcode (e.g. a new transcendental)
        //      doesn't silently compose with stock's {x22, w23} ABI and
        //      produce wrong code.
        static constexpr std::array<uint16_t, 6> kKnownFallThrough = {
            kOpcodeName_fclex,    // metadata-only; inline parity → no win
            kOpcodeName_finit,    // metadata-only; inline 0.95× → no win
            kOpcodeName_fldenv,   // metadata-only; inline parity → no win
            kOpcodeName_fstenv,   // metadata-only; inline 0.66× regression
            kOpcodeName_fxsave,   // SSE-era extended (8×f80 ST + 16×XMM)
            kOpcodeName_fxrstor,  // SSE-era extended
        };
        const uint16_t op = localIR[req.insn_idx].opcode();
        // stock_hash_hit: the None reply is a deliberate whole-block
        // exclusion, not a missing handler.
        const bool deliberate = stock_hash_hit ||
                                std::ranges::find(kKnownFallThrough, op) != kKnownFallThrough.end();
        if (!deliberate) {
            const char* name = (op < kOpcodeNames.size()) ? kOpcodeNames[op] : "?";
            fprintf(stdout,
                    "[rosettax87] UNHANDLED x87 opcode %s (0x%x) at "
                    "insn_idx=%lld; falling through to stock — add a "
                    "translate_* and dispatch case, or extend "
                    "kKnownFallThrough if this is a deliberate-stock op\n",
                    name, static_cast<unsigned>(op), static_cast<long long>(req.insn_idx));
            fflush(stdout);
        }
    }
    return out;
}

// True when `kr` is a receive-side mach_msg error (0x10004xxx, bit 14 set;
// send-side errors are 0x10000xxx).  A combined SEND|RCV call can fail on
// either half: a send-half failure means the reply was NOT queued (we still
// own the SEND_ONCE right) and the receive half never ran; a receive-half
// failure means the reply went out fine.
constexpr bool machMsgRcvError(kern_return_t kr) {
    return (kr & 0x00004000) != 0;
}

void runReceiveLoop(mach_port_t servicePort, mach_port_t parentTaskPort) {
    struct alignas(8) {
        uint8_t bytes[kRecvBufferSize];
    } buf;

    struct ReplyMsg {
        mach_msg_header_t hdr;
        uint64_t result;
        uint64_t some_flag;  // 1 = Some(result), 0 = None (fall through)
    };

    uint64_t send_failures = 0;
    bool replyPending = false;  // buf.bytes holds a ReplyMsg not yet sent
    for (;;) {
        auto* hdr = reinterpret_cast<mach_msg_header_t*>(buf.bytes);
        kern_return_t kr;
        if (replyPending) {
            // MIG-server shape: send the pending reply and block for the
            // next request in ONE trap (the received message reuses the
            // same buffer).  Halves the per-request mach_msg count vs the
            // old separate SEND + RCV calls.
            replyPending = false;
            kr = mach_msg(hdr, MACH_SEND_MSG | MACH_RCV_MSG, sizeof(ReplyMsg), sizeof(buf),
                          servicePort, MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
            if (kr != KERN_SUCCESS && !machMsgRcvError(kr)) {
                if (++send_failures <= 5) {
                    fprintf(stdout, "sidecar: reply send failed 0x%x %s\n", kr,
                            mach_error_string(kr));
                }
                // mach_msg consumes the SEND_ONCE on success; on send
                // failure we must drop it ourselves, then retry as a
                // plain receive.
                mach_port_deallocate(mach_task_self(), hdr->msgh_remote_port);
                continue;
            }
        } else {
            hdr->msgh_local_port = servicePort;
            hdr->msgh_size = sizeof(buf);
            kr = mach_msg(hdr, MACH_RCV_MSG, 0, sizeof(buf), servicePort, MACH_MSG_TIMEOUT_NONE,
                          MACH_PORT_NULL);
        }
        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "sidecar: mach_msg(RCV) returned 0x%x (%s)\n", kr,
                    mach_error_string(kr));
            return;
        }

        g_hits.fetch_add(1, std::memory_order_relaxed);

        // M3 payload: header (24 B) + 5 × 8-byte args (40 B) = 64 B.
        // Args are TR*, IRBlock*, IRInstr*, num_instrs, insn_idx — the
        // five translate_insn parameters in register order (x0..x4).
        // Reply path: stub provided a SEND_ONCE on msgh_remote_port (via
        // MAKE_SEND_ONCE on the local-port disposition).  We echo the
        // request's msgh_id back so the stub can detect cross-talk
        // (Step 1b) and put the Some/None signal into a dedicated body
        // word (some_flag) so msgh_id is purely a transaction tag.
        //
        // Dispatch on the top-byte sentinel (0x10) rather than an exact
        // msgh_id match — the bottom 24 bits now carry per-call data.
        mach_port_t replyPort = hdr->msgh_remote_port;
        if (hdr->msgh_size >= 24 + 40 && (hdr->msgh_id & 0xFF000000U) == 0x10000000U &&
            replyPort != MACH_PORT_NULL) {
            const uint32_t reqId = hdr->msgh_id;
            TranslateRequest req{};
            std::memcpy(&req, buf.bytes + 24, sizeof(req));

            TranslateOutcome outcome = processTranslateRequest(parentTaskPort, req);

            // Build the reply in place over the request bytes; it goes out
            // fused with the next receive at the top of the loop.  Every
            // header field is set explicitly since the buffer holds the
            // stale request header.
            auto* reply = reinterpret_cast<ReplyMsg*>(buf.bytes);
            reply->hdr.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_MOVE_SEND_ONCE, 0);
            reply->hdr.msgh_size = sizeof(ReplyMsg);
            reply->hdr.msgh_remote_port = replyPort;
            reply->hdr.msgh_local_port = MACH_PORT_NULL;
            reply->hdr.msgh_voucher_port = MACH_PORT_NULL;
            reply->hdr.msgh_id = reqId;  // echo for transaction match
            reply->result = outcome.reply_some ? static_cast<uint64_t>(outcome.value) : 0;
            reply->some_flag = outcome.reply_some ? 1U : 0U;
            replyPending = true;
        } else if (replyPort != MACH_PORT_NULL &&
                   (hdr->msgh_bits & MACH_MSGH_BITS_REMOTE_MASK) == MACH_MSG_TYPE_MOVE_SEND_ONCE) {
            // Other / malformed message — discard the SEND_ONCE.
            mach_port_deallocate(mach_task_self(), replyPort);
        }
    }
}

void* threadEntry(void* arg) {
    auto* a = reinterpret_cast<ThreadArgs*>(arg);
    runReceiveLoop(a->servicePort, a->parentTaskPort);
    delete a;
    return nullptr;
}

// Periodic throughput reporter — every kReporterPeriodSec seconds, log
// requests-per-period and the running total.  Quiet during idle: print
// one transition line when the sidecar goes idle (delta=0 after an
// active period), then suppress further "0 req/s" lines until activity
// resumes.  Long-running games at steady-state shouldn't spam the log.
constexpr unsigned kReporterPeriodSec = 2;

void* reporterEntry(void* /*arg*/) {
    pthread_setname_np("rosettax87-reporter");
    uint64_t prev_total = 0;
    uint64_t prev_vm = 0;
    bool printed_idle = false;
    for (;;) {
        const struct timespec ts = {.tv_sec = kReporterPeriodSec, .tv_nsec = 0};
        nanosleep(&ts, nullptr);
        const uint64_t cur = g_hits.load(std::memory_order_relaxed);
        const uint64_t delta = cur - prev_total;
        prev_total = cur;
        const uint64_t vm = g_statVmSyscalls.load(std::memory_order_relaxed);
        const uint64_t vm_delta = vm - prev_vm;
        prev_vm = vm;
        if (delta == 0) {
            if (!printed_idle && cur > 0) {
                fprintf(stdout, "[rosettax87] sidecar: idle (total %llu)\n",
                        static_cast<unsigned long long>(cur));
                fflush(stdout);
                printed_idle = true;
            }
            continue;
        }
        printed_idle = false;
        // vm/req counts mach_vm_* traps on the translate path this period
        // (mach_msg excluded: 1/request by construction).  Cache columns are
        // cumulative hits/misses so rates are readable at a glance.
        fprintf(stdout,
                "[rosettax87] sidecar: %llu req/s (total %llu) vm/req=%.2f "
                "ir=%llu/%llu tco=%llu\n",
                static_cast<unsigned long long>(delta / kReporterPeriodSec),
                static_cast<unsigned long long>(cur),
                static_cast<double>(vm_delta) / static_cast<double>(delta),
                static_cast<unsigned long long>(g_statIrHits.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(g_statIrMisses.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(g_statTcoHits.load(std::memory_order_relaxed)));
        fflush(stdout);
    }
}

}  // namespace

bool installPortInParent(mach_port_t parentTaskPort, mach_port_t* outServicePort,
                         uint32_t* outParentReqName, uint32_t* outParentReplyName) {
    // Allocate a fresh receive port in this process.
    mach_port_t servicePort = MACH_PORT_NULL;
    kern_return_t kr = mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_RECEIVE, &servicePort);
    if (kr != KERN_SUCCESS) {
        fprintf(stdout, "sidecar: mach_port_allocate(RECEIVE) failed (0x%x %s)\n", kr,
                mach_error_string(kr));
        return false;
    }

    // Insert a send right (derived from our receive right) so we can
    // hand it across the task boundary.
    kr =
        mach_port_insert_right(mach_task_self(), servicePort, servicePort, MACH_MSG_TYPE_MAKE_SEND);
    if (kr != KERN_SUCCESS) {
        fprintf(stdout, "sidecar: mach_port_insert_right(SELF, MAKE_SEND) failed (0x%x %s)\n", kr,
                mach_error_string(kr));
        return false;
    }

    // Allocate a fresh name in the parent task's namespace, then plant
    // our send right under that name. The parent process can mach_msg
    // to that name and the kernel will route to our servicePort.
    mach_port_name_t parentName = MACH_PORT_NULL;
    kr = mach_port_allocate(parentTaskPort, MACH_PORT_RIGHT_DEAD_NAME, &parentName);
    if (kr != KERN_SUCCESS) {
        fprintf(stdout, "sidecar: mach_port_allocate(parent, DEAD_NAME) failed (0x%x %s)\n", kr,
                mach_error_string(kr));
        return false;
    }
    // Drop the placeholder so the name slot is free.
    kr = mach_port_deallocate(parentTaskPort, parentName);
    if (kr != KERN_SUCCESS) {
        fprintf(stdout, "sidecar: mach_port_deallocate(parent placeholder) failed (0x%x %s)\n", kr,
                mach_error_string(kr));
        return false;
    }

    // Plant our send right under that freed name. The kernel resolves
    // (mach_task_self(), servicePort) to the underlying port object and
    // installs a send right at `parentName` in `parentTaskPort`'s ns.
    kr = mach_port_insert_right(parentTaskPort, parentName, servicePort, MACH_MSG_TYPE_COPY_SEND);
    if (kr != KERN_SUCCESS) {
        fprintf(stdout,
                "sidecar: mach_port_insert_right(parent, COPY_SEND) failed "
                "(0x%x %s)\n",
                kr, mach_error_string(kr));
        return false;
    }

    // Verify what's actually at parentName in parent's namespace.
    mach_port_type_t parentType = 0;
    kr = mach_port_type(parentTaskPort, parentName, &parentType);
    if (kr != KERN_SUCCESS) {
        fprintf(stdout, "sidecar: mach_port_type(parent,0x%x) failed 0x%x %s\n", parentName, kr,
                mach_error_string(kr));
    }

    *outServicePort = servicePort;
    *outParentReqName = static_cast<uint32_t>(parentName);

    // Allocate the parent-owned reply port. Stub names it as
    // msgh_local_port + MAKE_SEND_ONCE, so the kernel hands the sidecar
    // a fresh SEND_ONCE per call. Sidecar replies via that send-once
    // and the reply lands here in parent's space; the stub's
    // mach_msg(RCV) drains it.
    mach_port_name_t parentReplyName = MACH_PORT_NULL;
    kr = mach_port_allocate(parentTaskPort, MACH_PORT_RIGHT_RECEIVE, &parentReplyName);
    if (kr != KERN_SUCCESS) {
        fprintf(stdout,
                "sidecar: mach_port_allocate(parent, RECEIVE) for reply "
                "failed (0x%x %s)\n",
                kr, mach_error_string(kr));
        return false;
    }
    *outParentReplyName = static_cast<uint32_t>(parentReplyName);
    return true;
}

// A launcher can start several independently wrapped processes with the same
// profiler environment.  Expand against the process whose task port the
// sidecar received, not the sidecar's own pid, so they cannot truncate one
// another's output.
static bool expandTargetPid(std::string& path, pid_t targetPid, const char* variable) {
    if (path.find("%p") == std::string::npos) {
        return true;
    }
    if (targetPid <= 0) {
        fprintf(stdout,
                "[rosettax87] %s: cannot expand %%p because the target pid is unavailable; "
                "profiling disabled\n",
                variable);
        return false;
    }

    const std::string replacement = std::to_string(targetPid);
    size_t offset = 0;
    while ((offset = path.find("%p", offset)) != std::string::npos) {
        path.replace(offset, 2, replacement);
        offset += replacement.size();
    }
    return true;
}

bool spawnReceiveThread(mach_port_t servicePort, mach_port_t parentTaskPort) {
    if (g_rosetta_config != nullptr && !g_rosetta_config->profile_path.empty()) {
        std::string path = g_rosetta_config->profile_path;
        pid_t targetPid = 0;
        pid_for_task(parentTaskPort, &targetPid);
        if (!expandTargetPid(path, targetPid, "X87_PROFILE")) {
            path.clear();
        }
        g_profile.file = path.empty() ? nullptr : std::fopen(path.c_str(), "wb");
        if (g_profile.file == nullptr) {
            if (!path.empty()) {
                fprintf(stdout, "[rosettax87] X87_PROFILE: failed to open '%s' for writing\n",
                        path.c_str());
            }
        } else {
            fprintf(stdout, "[rosettax87] X87_PROFILE: dumping IR streams to '%s'\n",
                    path.c_str());
        }
        fflush(stdout);
    }

    pthread_t thr;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    auto* args = new ThreadArgs{.servicePort = servicePort, .parentTaskPort = parentTaskPort};
    int rc = pthread_create(&thr, &attr, threadEntry, args);
    pthread_attr_destroy(&attr);
    if (rc != 0) {
        delete args;
        fprintf(stdout, "sidecar: pthread_create failed (%d)\n", rc);
        return false;
    }

    // Throughput reporter is opt-in via X87_LOG_THROUGHPUT=1 — useful for
    // bisecting hangs (tells "stuck" apart from "just slow" on big
    // workloads), but noisy enough that we don't want it on by default.
    // Same detached lifetime as the receive thread; both die with the
    // loader process when the parent exits.  Failure to spawn the
    // reporter is non-fatal.
    if (g_rosetta_config != nullptr && g_rosetta_config->loader_log_throughput != 0U) {
        pthread_t rthr;
        pthread_attr_t rattr;
        pthread_attr_init(&rattr);
        pthread_attr_setdetachstate(&rattr, PTHREAD_CREATE_DETACHED);
        int rrc = pthread_create(&rthr, &rattr, reporterEntry, nullptr);
        pthread_attr_destroy(&rattr);
        if (rrc != 0) {
            fprintf(stdout,
                    "sidecar: reporter pthread_create failed (%d) — "
                    "throughput logging disabled\n",
                    rrc);
        }
    }
    return true;
}

void samplerConfigFromEnv(SamplerConfig& cfg) {
    // X87_SAMPLE names the profile and enables the sampler, exactly as
    // X87_PROFILE does for the block profiler.
    if (const char* path = getenv("X87_SAMPLE"); path != nullptr && path[0] != '\0') {
        cfg.path = path;
    }
    if (const char* hz = getenv("X87_SAMPLE_HZ")) {
        const double rate = strtod(hz, nullptr);
        if (rate > 0) {
            cfg.interval_us = static_cast<uint64_t>(1e6 / rate);
        }
    }
    if (const char* hz = getenv("X87_SAMPLE_SWEEP_HZ")) {
        const double rate = strtod(hz, nullptr);
        if (rate > 0) {
            cfg.sweep_interval_us = static_cast<uint64_t>(1e6 / rate);
        }
    }
    if (const char* secs = getenv("X87_SAMPLE_REPORT")) {
        cfg.report_s = strtod(secs, nullptr);
    }
    // Defaults ON, so unset and empty both read as on and only an explicit "0"
    // turns it off, matching every other knob that ships enabled.
    if (const char* w = getenv("X87_SAMPLE_WINDOWS"); w != nullptr && w[0] != '\0') {
        cfg.windows = strcmp(w, "0") != 0;
    }
    if (env_truthy("X87_NO_UNWIND")) {
        cfg.unwind = false;
    }
    if (const char* range = getenv("X87_GUEST_RANGE")) {
        const char* dash = strchr(range[0] == '0' && range[1] == 'x' ? range + 2 : range, '-');
        if (dash != nullptr) {
            cfg.guest_lo = strtoull(range, nullptr, 0);
            cfg.guest_hi = strtoull(dash + 1, nullptr, 0);
            cfg.guest_range_pinned = true;
        }
    }
}

// Delete every <path>.NNNN left by an earlier run.  The series is one appended
// file now, so this only clears what a build before that wrote: those files are
// still a valid series to a reader, and left in place they would be summed into
// a capture they have nothing to do with.  Scanned rather than counted up from
// zero, because that run may have written more windows than this one will and
// stopping at the first gap would leave its tail behind.
void unlinkWindowSeries(const std::string& path) {
    const size_t slash = path.find_last_of('/');
    const std::string dir = slash == std::string::npos ? "." : path.substr(0, slash);
    const std::string stem = slash == std::string::npos ? path : path.substr(slash + 1);
    DIR* d = opendir(dir.c_str());
    if (d == nullptr) {
        return;
    }
    while (const struct dirent* e = readdir(d)) {
        const std::string name = e->d_name;
        if (name.size() != stem.size() + 5 || name.compare(0, stem.size(), stem) != 0 ||
            name[stem.size()] != '.') {
            continue;
        }
        if (std::all_of(name.begin() + static_cast<long>(stem.size()) + 1, name.end(),
                        [](char c) { return c >= '0' && c <= '9'; })) {
            unlink((dir + "/" + name).c_str());
        }
    }
    closedir(d);
}

void startSampler(mach_port_t parentTaskPort, uint64_t runtimeBase, const SamplerConfig& in) {
    if (in.path.empty()) {
        return;
    }
    SamplerConfig cfg = in;
    // Sweeping every thread faster than the sample rate is never what was
    // meant: the sweep is the expensive mode and the rate is the cheap one.
    cfg.sweep_interval_us = std::max(cfg.sweep_interval_us, cfg.interval_us);
    // The target's pid is needed by both threads and pid_for_task only answers
    // while it is alive, so read it before touching the output path.
    pid_t targetPid = 0;
    pid_for_task(parentTaskPort, &targetPid);
    if (!expandTargetPid(cfg.path, targetPid, "X87_SAMPLE")) {
        return;
    }
    // Nothing from a previous run may survive into this one: a run that never
    // latches writes no profile, and the absence has to be the answer rather
    // than the last run's file still sitting there to be read as this one's.
    // The windows go too, and matter more: a reader sums the series, so a stale
    // window would be added into this run's totals rather than merely misread.
    // The series file is reopened truncating anyway, but only if this run gets
    // as far as a window, and an old one sitting next to a fresh profile is
    // exactly the confusion this is here to prevent.
    unlink(cfg.path.c_str());
    unlink((cfg.path + ".windows").c_str());
    unlinkWindowSeries(cfg.path);
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    auto* ctx = new SamplerCtx{parentTaskPort, runtimeBase, cfg, 0};
    ctx->pid = targetPid;
    pthread_t thr;
    if (pthread_create(&thr, &attr, samplerMain, ctx) != 0) {
        delete ctx;
        fprintf(stdout, "[rosettax87] X87_SAMPLE: sampler thread creation failed\n");
        pthread_attr_destroy(&attr);
        return;
    }
    g_sampler_running.store(true, std::memory_order_release);

    // The aggregator owns the histograms, the module map and the file.  If it
    // cannot start there is nothing to write the profile, so the sampler is told
    // to stop rather than left measuring into a ring nobody drains.
    auto* agg = new AggCtx{.task = parentTaskPort,
                           .pid = targetPid,
                           .runtime_base = runtimeBase,
                           .path = cfg.path,
                           .interval_us = cfg.interval_us,
                           .sweep_interval_us = cfg.sweep_interval_us,
                           .report_s = cfg.report_s,
                           .unwind = cfg.unwind,
                           .range_pinned = cfg.guest_range_pinned,
                           .windows = cfg.windows};
    pthread_t aggThr;
    if (pthread_create(&aggThr, &attr, aggregatorMain, agg) != 0) {
        delete agg;
        g_sampler_running.store(false, std::memory_order_release);
        fprintf(stdout, "[rosettax87] X87_SAMPLE: aggregator thread creation failed\n");
    }
    pthread_attr_destroy(&attr);
}

// Async-signal-safe: an atomic store, nanosleep and write(2).  It is called from
// a SIGTERM handler as well as from the normal exit path, because the sidecar is
// usually killed rather than allowed to return from main.
void flushSamplerIfEnabled() {
    if (!g_sampler_running.load(std::memory_order_acquire)) {
        return;
    }
    g_sampler_flush_request.store(true, std::memory_order_release);
    // The sampler notices within one tick; the write itself is a whole-file
    // rewrite, tens of MB on a long high-rate run.  Bounded so a wedged sampler
    // cannot hold up exit, and short enough to fit inside the grace period a
    // process manager gives between SIGTERM and SIGKILL.
    constexpr int kFlushWaitMs = 3000;
    for (int i = 0; i < kFlushWaitMs; i++) {
        if (g_sampler_flush_done.load(std::memory_order_acquire)) {
            return;
        }
        struct timespec ms{0, 1000000};
        nanosleep(&ms, nullptr);
    }
    static constexpr char kLate[] =
        "[rosettax87] X87_SAMPLE: final write did not finish in time; the profile holds "
        "everything up to the last report interval only\n";
    (void)write(STDOUT_FILENO, kLate, sizeof(kLate) - 1);
}

void dumpCountersIfEnabled(mach_port_t /*parentTaskPort*/) {
    if (g_profile.file == nullptr) {
        return;
    }
    const uint64_t local_addr = profile::counter_array_local_addr();
    const uint32_t count = profile::block_count();
    if (local_addr == 0 || count == 0) {
        fprintf(stdout,
                "[rosettax87] X87_PROFILE: no counters to dump (count=%u local_addr=0x%llx); "
                "closing without a counter section; analyzer will reject it\n",
                count, local_addr);
        std::fclose(g_profile.file);
        g_profile.file = nullptr;
        return;
    }

    // The counter array's backing pages are shared with parent via
    // mach_vm_remap; reading from local_addr observes whatever parent's
    // LDADDAL has written, with no IPC and no race against parent's
    // death.
    const auto* counts = reinterpret_cast<const uint64_t*>(local_addr);

    std::scoped_lock lock(g_profile.io_mu);
    profile::CounterSectionHeader chdr{
        .magic = profile::kCounterSectionMagic,
        .count = count,
    };
    std::fwrite(&chdr, sizeof(chdr), 1, g_profile.file);
    std::fwrite(counts, sizeof(uint64_t), count, g_profile.file);

    // Translation-path tally section: per block_id 0..count-1, snapshot the
    // accumulated (ir, peephole, single, fallthrough) op counts.  Written
    // here at exit time because translate_instruction's bumps can keep
    // landing right up until parent exit; dumping inline with BlockHeader
    // would race the bumps (dumpBlockIfNew runs *before* the first
    // translate_instruction call — see sidecar.cpp:~360).
    profile::TallySectionHeader thdr{
        .magic = profile::kTallySectionMagic,
        .count = count,
    };
    std::fwrite(&thdr, sizeof(thdr), 1, g_profile.file);
    for (uint32_t bid = 0; bid < count; ++bid) {
        const profile::BlockTally t = profile::get_block_tally(bid);
        profile::BlockTallyEntry entry{
            .ir_ops = t.ir_ops,
            .peephole_ops = t.peephole_ops,
            .single_ops = t.single_ops,
            .fallthrough_ops = t.fallthrough_ops,
            .ir_build_fail_ops = t.ir_build_fail_ops,
            .ir_fpr_fail_ops = t.ir_fpr_fail_ops,
            .ir_gpr_fail_ops = t.ir_gpr_fail_ops,
            .max_gpr_peak = t.max_gpr_peak,
            .ir_split_runs = t.ir_split_runs,
            .ir_remat_runs = t.ir_remat_runs,
            .bridge_ops = t.bridge_ops,
            .bridge_fail_runs = t.bridge_fail_runs,
        };
        std::fwrite(&entry, sizeof(entry), 1, g_profile.file);
    }

    // Build-bail-opcode side-table: per block_id 0..count-1, the opcode at
    // which X87IR::build()'s default arm bailed (or 0xFFFF sentinel).  The
    // analyzer combines this with the counter section to produce an exec-
    // weighted "which opcodes are blocking IR coverage" histogram.  Always
    // written when profiling is enabled; entries are 0xFFFF for blocks that
    // never tripped a bail.
    profile::BuildFailOpSectionHeader bhdr{
        .magic = profile::kBuildFailOpSectionMagic,
        .count = count,
    };
    std::fwrite(&bhdr, sizeof(bhdr), 1, g_profile.file);
    for (uint32_t bid = 0; bid < count; ++bid) {
        const uint16_t op = profile::get_block_build_fail_op(bid);
        std::fwrite(&op, sizeof(op), 1, g_profile.file);
    }

    // IR-gate per-reason refusal counter side-table (IRG1): per block_id
    // 0..count-1, 5 uint16 counters indexed by kIRGateReason*.  Pinpoints
    // the silent "ir%=0 with all-zero failure tallies" cohort the BFO0
    // histogram can't see; per-reason counts (vs a single sentinel) avoid
    // trailing-tail short_run records masking longer-run refusals.
    profile::IRGateRefuseSectionHeader ihdr{
        .magic = profile::kIRGateRefuseSectionMagic,
        .count = count,
    };
    std::fwrite(&ihdr, sizeof(ihdr), 1, g_profile.file);
    for (uint32_t bid = 0; bid < count; ++bid) {
        const profile::BlockIRGateCounters c = profile::get_block_ir_gate_counters(bid);
        std::fwrite(&c, sizeof(c), 1, g_profile.file);
    }

    // Top-dirty predecessor side-table (TDP0): per block_id, the last x87
    // opcode translated before the most-recent top_dirty gate refusal,
    // or 0xFFFF if no top_dirty refusal was observed.  Used by the
    // analyzer to render a "Top opcodes preceding top_dirty refusal"
    // histogram, pinpointing which op leaves top_dirty=1.
    profile::TopDirtyPredSectionHeader tdhdr{
        .magic = profile::kTopDirtyPredSectionMagic,
        .count = count,
    };
    std::fwrite(&tdhdr, sizeof(tdhdr), 1, g_profile.file);
    for (uint32_t bid = 0; bid < count; ++bid) {
        const uint16_t op = profile::get_block_top_dirty_predecessor(bid);
        std::fwrite(&op, sizeof(op), 1, g_profile.file);
    }

    // Per-reason max cache.run_remaining at refusal (RRR0).
    profile::MaxRunAtRefuseSectionHeader mrhdr{
        .magic = profile::kMaxRunAtRefuseSectionMagic,
        .count = count,
    };
    std::fwrite(&mrhdr, sizeof(mrhdr), 1, g_profile.file);
    for (uint32_t bid = 0; bid < count; ++bid) {
        const profile::BlockMaxRunAtRefuse mr = profile::get_block_max_run_at_refuse(bid);
        std::fwrite(&mr, sizeof(mr), 1, g_profile.file);
    }

    std::fflush(g_profile.file);
    std::fclose(g_profile.file);
    g_profile.file = nullptr;

    const uint64_t mx = *std::max_element(counts, counts + count);
    // Leading \n: this fires from the sidecar's kqueue NOTE_EXIT handler
    // *after* the parent process has already terminated and the shell
    // has redrawn its prompt.  Without the leading \n the message glues
    // onto the prompt line.
    fprintf(stdout, "\n[rosettax87] X87_PROFILE: wrote %u block counters; max=%llu\n", count,
            static_cast<unsigned long long>(mx));
    fflush(stdout);
}

}  // namespace sidecar
