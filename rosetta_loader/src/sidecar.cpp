#include "sidecar.hpp"

#include <mach/mach.h>
#include <mach/mach_port.h>
#include <mach/mach_vm.h>
#include <pthread.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <mutex>
#include <unordered_set>
#include <vector>

#include "rosetta_core/Config.h"
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

bool writeParent(mach_port_t task, uint64_t addr, const void* src, size_t size) {
    if (size == 0) {
        return true;
    }
    return mach_vm_write(task, addr, reinterpret_cast<vm_offset_t>(const_cast<void*>(src)), size) ==
           KERN_SUCCESS;
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
    if (mach_vm_allocate(parentTask, &parentNew, newCap, VM_FLAGS_ANYWHERE) != KERN_SUCCESS) {
        return 0;
    }
    if (origLive > 0) {
        std::vector<uint8_t> stash(origLive);
        if (!readParent(parentTask, origAddr, stash.data(), origLive) ||
            !writeParent(parentTask, parentNew, stash.data(), origLive)) {
            mach_vm_deallocate(parentTask, parentNew, newCap);
            return 0;
        }
    }
    if (tailSize > 0) {
        if (!writeParent(parentTask, parentNew + origLive, tail, tailSize)) {
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

    // Read parent's TR. Default-constructed local; we sterilise its list
    // pointers before scope end so `~TransactionalList` runs `::operator
    // delete(nullptr)` (a no-op) instead of freeing arbitrary parent VAs.
    TranslationResult tr;
    if (!readParent(parentTask, req.tr_addr, &tr, sizeof(tr))) {
        return out;
    }

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

    // Read IR array + ThreadContextOffsets.
    std::vector<IRInstr> localIR(req.num_instrs);
    if (!readParent(parentTask, req.instr_array, localIR.data(),
                    req.num_instrs * sizeof(IRInstr))) {
        return out;
    }

    if (origTCO == nullptr) {
        return out;
    }
    ThreadContextOffsets localTCO{};
    if (!readParent(parentTask, reinterpret_cast<uint64_t>(origTCO), &localTCO, sizeof(localTCO))) {
        return out;
    }

    // x87_cache is OUR addition (OPT-1) — see comment in TranslationResult.h.
    // The loader's M2 init patched stock's TR allocator to allocate
    // sizeof(TranslationResult) bytes per TR, so the cache field lives
    // inside parent's TR allocation and persists across calls. Trust
    // parent's bytes. (`cache.invalidate()` will fire automatically on
    // first call when prev_block doesn't match the just-passed block,
    // converging junk-initialised cache state to a sane baseline.)

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
        fprintf(stdout, "[rosettax87] op %s (0x%x) idx=%lld/%lld\n", name,
                static_cast<unsigned>(op), static_cast<long long>(req.insn_idx),
                static_cast<long long>(req.num_instrs));
        fflush(stdout);
    }

    dumpBlockIfNew(parentTask, reinterpret_cast<uint64_t>(tr.ir_module_data), req.block,
                   localIR.data(), req.num_instrs);

    auto result = Translator::translate_instruction(
        &tr, reinterpret_cast<IRBlock*>(req.block), localIR.data(),
        static_cast<int64_t>(req.num_instrs), static_cast<int64_t>(req.insn_idx));

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
                if (!writeParent(parentTask, reinterpret_cast<uint64_t>(origInsnData) + origInsnEnd,
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
                    if (!writeParent(parentTask, reinterpret_cast<uint64_t>(orig.end),
                                     localPushed[i], added)) {
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

    // Write back the full TR (always — propagates cache + scratch-mask
    // updates from Translator's run, plus any pivoted buffer pointers from
    // the Some path above). The loader's M2 init patched stock's TR
    // allocator to sizeof(TranslationResult), so parent's allocation has
    // room for our appended x87_cache field — persisting it across calls
    // is what restores OPT-1's cross-instruction reuse.
    if (!writeParent(parentTask, req.tr_addr, &tr, sizeof(tr))) {
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
        const bool deliberate = std::ranges::find(kKnownFallThrough, op) != kKnownFallThrough.end();
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

void runReceiveLoop(mach_port_t servicePort, mach_port_t parentTaskPort) {
    struct alignas(8) {
        uint8_t bytes[kRecvBufferSize];
    } buf;

    uint64_t send_failures = 0;
    for (;;) {
        auto* hdr = reinterpret_cast<mach_msg_header_t*>(buf.bytes);
        hdr->msgh_local_port = servicePort;
        hdr->msgh_size = sizeof(buf);

        kern_return_t kr = mach_msg(hdr, MACH_RCV_MSG, 0, sizeof(buf), servicePort,
                                    MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "sidecar: mach_msg(RCV) returned 0x%x (%s)\n", kr,
                    mach_error_string(kr));
            return;
        }

        const uint64_t hits = g_hits.fetch_add(1, std::memory_order_relaxed) + 1;

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

            struct ReplyMsg {
                mach_msg_header_t hdr;
                uint64_t result;
                uint64_t some_flag;  // 1 = Some(result), 0 = None (fall through)
            } reply{};
            reply.hdr.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_MOVE_SEND_ONCE, 0);
            reply.hdr.msgh_size = sizeof(reply);
            reply.hdr.msgh_remote_port = replyPort;
            reply.hdr.msgh_local_port = MACH_PORT_NULL;
            reply.hdr.msgh_id = reqId;  // echo for transaction match
            reply.result = outcome.reply_some ? static_cast<uint64_t>(outcome.value) : 0;
            reply.some_flag = outcome.reply_some ? 1U : 0U;

            kern_return_t kr_send = mach_msg(&reply.hdr, MACH_SEND_MSG, sizeof(reply), 0,
                                             MACH_PORT_NULL, MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
            if (kr_send != KERN_SUCCESS) {
                if (++send_failures <= 5) {
                    fprintf(stdout, "sidecar: #%llu reply send failed 0x%x %s\n",
                            static_cast<unsigned long long>(hits), kr_send,
                            mach_error_string(kr_send));
                }
                // mach_msg consumes the SEND_ONCE on success; on failure
                // we must drop it ourselves.
                mach_port_deallocate(mach_task_self(), replyPort);
            }
            // mach_msg(SEND) on success consumed replyPort. Don't drop it.
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
    bool printed_idle = false;
    for (;;) {
        const struct timespec ts = {.tv_sec = kReporterPeriodSec, .tv_nsec = 0};
        nanosleep(&ts, nullptr);
        const uint64_t cur = g_hits.load(std::memory_order_relaxed);
        const uint64_t delta = cur - prev_total;
        prev_total = cur;
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
        fprintf(stdout, "[rosettax87] sidecar: %llu req/s (total %llu)\n",
                static_cast<unsigned long long>(delta / kReporterPeriodSec),
                static_cast<unsigned long long>(cur));
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

bool spawnReceiveThread(mach_port_t servicePort, mach_port_t parentTaskPort) {
    if (g_rosetta_config != nullptr && !g_rosetta_config->profile_path.empty()) {
        const char* path = g_rosetta_config->profile_path.c_str();
        g_profile.file = std::fopen(path, "wb");
        if (g_profile.file == nullptr) {
            fprintf(stdout, "[rosettax87] X87_PROFILE: failed to open '%s' for writing\n", path);
        } else {
            fprintf(stdout, "[rosettax87] X87_PROFILE: dumping IR streams to '%s'\n", path);
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
