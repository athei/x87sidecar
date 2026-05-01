#include "sidecar.hpp"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <pthread.h>
#include <vector>

#include <mach/mach.h>
#include <mach/mach_port.h>
#include <mach/mach_vm.h>

#include "rosetta_core/Fixup.h"
#include "rosetta_core/IRInstr.h"
#include "rosetta_core/Opcode.h"
#include "rosetta_core/ThreadContextOffsets.h"
#include "rosetta_core/TranscendentalHelper.h"
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
    uint64_t block;          // opaque IRBlock* — Translator only compares as ptr
    uint64_t instr_array;
    uint64_t num_instrs;
    uint64_t insn_idx;
};

struct TranslateOutcome {
    bool reply_some;       // true → reply Some(value), else None.
    int64_t value;
};

bool readParent(mach_port_t task, uint64_t addr, void* dst, size_t size) {
    if (size == 0) { return true;
}
    mach_vm_size_t got = 0;
    kern_return_t kr = mach_vm_read_overwrite(task, addr, size,
                                              reinterpret_cast<mach_vm_address_t>(dst), &got);
    return kr == KERN_SUCCESS && got == size;
}

bool writeParent(mach_port_t task, uint64_t addr, const void* src, size_t size) {
    if (size == 0) { return true;
}
    return mach_vm_write(task, addr, reinterpret_cast<vm_offset_t>(const_cast<void*>(src)), size) ==
           KERN_SUCCESS;
}

// Allocate a parent-side replacement buffer of size `newCap`, copy parent's
// existing live bytes, then append `tailSize` bytes from `tail`. On success
// returns the parent VA of the new buffer. On any failure deallocates and
// returns 0.
mach_vm_address_t allocAndAppendInParent(mach_port_t parentTask,
                                          uint64_t origAddr, uint64_t origLive,
                                          uint64_t newCap,
                                          const void* tail, uint64_t tailSize) {
    // Round up to page granularity.
    newCap = (newCap + 0xFFF) & ~static_cast<uint64_t>(0xFFF);
    mach_vm_address_t parentNew = 0;
    if (mach_vm_allocate(parentTask, &parentNew, newCap, VM_FLAGS_ANYWHERE) !=
        KERN_SUCCESS) {
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
TranslateOutcome processTranslateRequest(mach_port_t parentTask,
                                         const TranslateRequest& req) {
    TranslateOutcome out{.reply_some=false, .value=0};

    constexpr uint64_t kMaxNumInstrs = 0x10000;
    if (req.num_instrs == 0 || req.num_instrs > kMaxNumInstrs) { return out;
}
    if (req.insn_idx >= req.num_instrs) { return out;
}

    // Read parent's TR. Default-constructed local; we sterilise its list
    // pointers before scope end so `~TransactionalList` runs `::operator
    // delete(nullptr)` (a no-op) instead of freeing arbitrary parent VAs.
    TranslationResult tr;
    if (!readParent(parentTask, req.tr_addr, &tr, sizeof(tr))) { return out;
}

    TransactionalList<Fixup>* lists[kListCount] = {
        &tr.external_fixups, &tr.internal_fixups, &tr._fixups,
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
    uint32_t* const origInsnData    = tr.insn_buf.data;
    uint64_t  const origInsnEnd     = tr.insn_buf.end;
    uint64_t  const origInsnCap     = tr.insn_buf.end_cap;
    uint32_t  const origInsnUseHeap = tr.insn_buf.use_heap;
    ThreadContextOffsets* const origTCO = tr.thread_context_offsets;

    struct ListBackup {
        Fixup* begin;
        Fixup* end;
        Fixup* end_cap;
        uint64_t _size;
    } origLists[kListCount];
    for (size_t i = 0; i < kListCount; i++) {
        origLists[i] = {.begin=lists[i]->begin, .end=lists[i]->end, .end_cap=lists[i]->end_cap,
                        ._size=lists[i]->_size};
    }

    // Read IR array + ThreadContextOffsets.
    std::vector<IRInstr> localIR(req.num_instrs);
    if (!readParent(parentTask, req.instr_array, localIR.data(),
                    req.num_instrs * sizeof(IRInstr))) { return out;
}

    if (origTCO == nullptr) { return out;
}
    ThreadContextOffsets localTCO{};
    if (!readParent(parentTask, reinterpret_cast<uint64_t>(origTCO), &localTCO,
                    sizeof(localTCO))) { return out;
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
    tr.insn_buf.data     = reinterpret_cast<uint32_t*>(localInsnVec.data());
    tr.insn_buf.end      = origInsnEnd;
    tr.insn_buf.end_cap  = localInsnVec.size();
    tr.insn_buf.use_heap = 1;
    for (auto & list : lists) {
        list->begin = list->end = list->end_cap = nullptr;
        list->_size = 0;
    }
    tr.thread_context_offsets = &localTCO;

    auto result = Translator::translate_instruction(
        &tr, reinterpret_cast<IRBlock*>(req.block),
        localIR.data(), static_cast<int64_t>(req.num_instrs), static_cast<int64_t>(req.insn_idx));

    // Capture growth state. If insn_buf grew, Translator's grow() abandoned
    // localInsnVec for a calloc'd buffer (we own that and must free it).
    auto* const localInsnData = reinterpret_cast<uint8_t*>(tr.insn_buf.data);
    bool     const insnGrew      = (localInsnData != localInsnVec.data());
    uint64_t const insnEmitted   = tr.insn_buf.end - origInsnEnd;
    Fixup*   localPushed[kListCount];
    uint64_t localPushedBytes[kListCount];
    for (size_t i = 0; i < kListCount; i++) {
        localPushed[i] = lists[i]->begin;
        localPushedBytes[i] =
            static_cast<uint64_t>(reinterpret_cast<uint8_t*>(lists[i]->end) - reinterpret_cast<uint8_t*>(lists[i]->begin));
    }
    struct LocalCleanup {
        uint8_t* insn_buf;       // null if Translator never grew (vec owns)
        Fixup* lists[kListCount];
        ~LocalCleanup() {
            if (insn_buf) { free(insn_buf);
}
            for (auto & list : lists) {
                if (list) { ::operator delete(list);
}
            }
        }
    } _cleanup{.insn_buf=insnGrew ? localInsnData : nullptr,
               .lists={localPushed[0], localPushed[1], localPushed[2],
                localPushed[3], localPushed[4], localPushed[5]}};

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
    tr.insn_buf.data     = origInsnData;
    tr.insn_buf.end      = origInsnEnd;
    tr.insn_buf.end_cap  = origInsnCap;
    tr.insn_buf.use_heap = origInsnUseHeap;
    tr.thread_context_offsets = origTCO;
    for (size_t i = 0; i < kListCount; i++) {
        lists[i]->begin   = origLists[i].begin;
        lists[i]->end     = origLists[i].end;
        lists[i]->end_cap = origLists[i].end_cap;
        lists[i]->_size   = origLists[i]._size;
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
        uint64_t finalInsnEnd  = origInsnEnd + insnEmitted;
        uint32_t* finalInsnData = origInsnData;
        uint64_t finalInsnCap   = origInsnCap;
        if (!insnGrew && finalInsnEnd <= origInsnCap) {
            if (insnEmitted > 0) {
                if (!writeParent(parentTask,
                                 reinterpret_cast<uint64_t>(origInsnData) + origInsnEnd,
                                 localInsnData + origInsnEnd,
                                 insnEmitted)) { return out;
}
            }
        } else {
            uint64_t newCap = std::max(origInsnCap * 2, finalInsnEnd);
            mach_vm_address_t parentNew = allocAndAppendInParent(
                parentTask, reinterpret_cast<uint64_t>(origInsnData), origInsnEnd, newCap,
                localInsnData + origInsnEnd, insnEmitted);
            if (parentNew == 0) { return out;
}
            finalInsnData = reinterpret_cast<uint32_t*>(parentNew);
            finalInsnCap  = (newCap + 0xFFF) & ~static_cast<uint64_t>(0xFFF);
        }
        tr.insn_buf.data    = finalInsnData;
        tr.insn_buf.end     = finalInsnEnd;
        tr.insn_buf.end_cap = finalInsnCap;

        // Append each list's pushed entries to parent — same fits-or-grow
        // split.
        for (size_t i = 0; i < kListCount; i++) {
            const auto& orig    = origLists[i];
            uint64_t parentLive = reinterpret_cast<uint8_t*>(orig.end) - reinterpret_cast<uint8_t*>(orig.begin);
            uint64_t parentCap  = reinterpret_cast<uint8_t*>(orig.end_cap) - reinterpret_cast<uint8_t*>(orig.begin);
            uint64_t added      = localPushedBytes[i];
            uint64_t newLive    = parentLive + added;

            if (newLive <= parentCap) {
                if (added > 0) {
                    if (!writeParent(parentTask, reinterpret_cast<uint64_t>(orig.end),
                                     localPushed[i], added)) { return out;
}
                }
                lists[i]->end = reinterpret_cast<Fixup*>(reinterpret_cast<uint8_t*>(orig.begin) + newLive);
            } else {
                uint64_t newCap = std::max(parentCap * 2, newLive);
                mach_vm_address_t parentNew = allocAndAppendInParent(
                    parentTask, reinterpret_cast<uint64_t>(orig.begin), parentLive, newCap,
                    localPushed[i], added);
                if (parentNew == 0) { return out;
}
                uint64_t roundedCap = (newCap + 0xFFF) & ~static_cast<uint64_t>(0xFFF);
                lists[i]->begin   = reinterpret_cast<Fixup*>(parentNew);
                lists[i]->end     = reinterpret_cast<Fixup*>(parentNew + newLive);
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
    if (!writeParent(parentTask, req.tr_addr, &tr, sizeof(tr))) { return out;
}

    if (result.has_value()) {
        out.reply_some = true;
        out.value      = result.value();
    } else {
        // The stub's FILTER prologue routes only x87 opcodes to us, so
        // reaching nullopt means an x87 op we deliberately route to
        // stock.  The stub's NONE-reply path falls through to stock's
        // translate_insn — safe today because the unhandled set is
        // limited to memory-block / NOP-class opcodes (fxsave, fxrstor,
        // fnop, fdisi, feni, fclex, finit, fldenv, fstenv)
        // where stock's emit reads coherent X87State from memory via
        // the shared x22 (or emits zero ARM instructions for the NOP
        // family).  Log the opcode anyway: a future helper-using
        // opcode hitting this path would silently compose with stock's
        // {x22, w23} ABI and produce wrong code, and this line is the
        // discoverability signal.
        const uint16_t op = localIR[req.insn_idx].opcode;
        const char* name = (op < kOpcodeNames.size()) ? kOpcodeNames[op] : "?";
        fprintf(stdout,
                "[rosettax87] unhandled x87 opcode %s (0x%x) at "
                "insn_idx=%lld; falling through to stock\n",
                name, static_cast<unsigned>(op),
                static_cast<long long>(req.insn_idx));
        fflush(stdout);
    }
    return out;
}

void runReceiveLoop(mach_port_t servicePort, mach_port_t parentTaskPort) {
    struct alignas(8) {
        uint8_t bytes[kRecvBufferSize];
    } buf;

    uint64_t hits = 0;
    for (;;) {
        auto* hdr = reinterpret_cast<mach_msg_header_t*>(buf.bytes);
        hdr->msgh_local_port = servicePort;
        hdr->msgh_size = sizeof(buf);

        kern_return_t kr =
            mach_msg(hdr, MACH_RCV_MSG, 0, sizeof(buf), servicePort,
                     MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "sidecar: mach_msg(RCV) returned 0x%x (%s)\n", kr,
                    mach_error_string(kr));
            return;
        }

        hits++;

        // M3 payload: header (24 B) + 5 × 8-byte args (40 B) = 64 B.
        // Args are TR*, IRBlock*, IRInstr*, num_instrs, insn_idx — the
        // five translate_insn parameters in register order (x0..x4).
        // Reply path: stub provided a SEND_ONCE on msgh_remote_port (via
        // MAKE_SEND_ONCE on the local-port disposition). msgh_id=1 +
        // body[0]=result returns Some(N) to the stub; msgh_id=0 falls
        // through to stock translate_insn.
        mach_port_t replyPort = hdr->msgh_remote_port;
        if (hdr->msgh_size >= 24 + 40 && hdr->msgh_id == 0x10000001 &&
            replyPort != MACH_PORT_NULL) {
            TranslateRequest req{};
            std::memcpy(&req, buf.bytes + 24, sizeof(req));

            TranslateOutcome outcome =
                processTranslateRequest(parentTaskPort, req);

            struct ReplyMsg {
                mach_msg_header_t hdr;
                uint64_t result;
            } reply{};
            reply.hdr.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_MOVE_SEND_ONCE, 0);
            reply.hdr.msgh_size = sizeof(reply);
            reply.hdr.msgh_remote_port = replyPort;
            reply.hdr.msgh_local_port = MACH_PORT_NULL;
            reply.hdr.msgh_id = outcome.reply_some ? 1 : 0;
            reply.result      = outcome.reply_some
                                    ? static_cast<uint64_t>(outcome.value)
                                    : 0;

            kern_return_t kr_send = mach_msg(
                &reply.hdr, MACH_SEND_MSG, sizeof(reply), 0, MACH_PORT_NULL,
                MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
            if (kr_send != KERN_SUCCESS) {
                if (hits <= 5) {
                    fprintf(stdout,
                            "sidecar: #%llu reply send failed 0x%x %s\n",
                            hits, kr_send, mach_error_string(kr_send));
                }
                // mach_msg consumes the SEND_ONCE on success; on failure
                // we must drop it ourselves.
                mach_port_deallocate(mach_task_self(), replyPort);
            }
            // mach_msg(SEND) on success consumed replyPort. Don't drop it.
        } else if (replyPort != MACH_PORT_NULL &&
                   (hdr->msgh_bits & MACH_MSGH_BITS_REMOTE_MASK) ==
                       MACH_MSG_TYPE_MOVE_SEND_ONCE) {
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

}  // namespace

bool installPortInParent(mach_port_t parentTaskPort,
                         mach_port_t* outServicePort,
                         uint32_t* outParentReqName,
                         uint32_t* outParentReplyName) {
    // Allocate a fresh receive port in this process.
    mach_port_t servicePort = MACH_PORT_NULL;
    kern_return_t kr =
        mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_RECEIVE, &servicePort);
    if (kr != KERN_SUCCESS) {
        fprintf(stdout,
                "sidecar: mach_port_allocate(RECEIVE) failed (0x%x %s)\n",
                kr, mach_error_string(kr));
        return false;
    }

    // Insert a send right (derived from our receive right) so we can
    // hand it across the task boundary.
    kr = mach_port_insert_right(mach_task_self(), servicePort, servicePort,
                                MACH_MSG_TYPE_MAKE_SEND);
    if (kr != KERN_SUCCESS) {
        fprintf(stdout,
                "sidecar: mach_port_insert_right(SELF, MAKE_SEND) failed (0x%x %s)\n",
                kr, mach_error_string(kr));
        return false;
    }

    // Allocate a fresh name in the parent task's namespace, then plant
    // our send right under that name. The parent process can mach_msg
    // to that name and the kernel will route to our servicePort.
    mach_port_name_t parentName = MACH_PORT_NULL;
    kr = mach_port_allocate(parentTaskPort, MACH_PORT_RIGHT_DEAD_NAME, &parentName);
    if (kr != KERN_SUCCESS) {
        fprintf(stdout,
                "sidecar: mach_port_allocate(parent, DEAD_NAME) failed (0x%x %s)\n",
                kr, mach_error_string(kr));
        return false;
    }
    // Drop the placeholder so the name slot is free.
    kr = mach_port_deallocate(parentTaskPort, parentName);
    if (kr != KERN_SUCCESS) {
        fprintf(stdout,
                "sidecar: mach_port_deallocate(parent placeholder) failed (0x%x %s)\n",
                kr, mach_error_string(kr));
        return false;
    }

    // Plant our send right under that freed name. The kernel resolves
    // (mach_task_self(), servicePort) to the underlying port object and
    // installs a send right at `parentName` in `parentTaskPort`'s ns.
    kr = mach_port_insert_right(parentTaskPort, parentName, servicePort,
                                MACH_MSG_TYPE_COPY_SEND);
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
        fprintf(stdout,
                "sidecar: mach_port_type(parent,0x%x) failed 0x%x %s\n",
                parentName, kr, mach_error_string(kr));
    }

    *outServicePort = servicePort;
    *outParentReqName = static_cast<uint32_t>(parentName);

    // Allocate the parent-owned reply port. Stub names it as
    // msgh_local_port + MAKE_SEND_ONCE, so the kernel hands the sidecar
    // a fresh SEND_ONCE per call. Sidecar replies via that send-once
    // and the reply lands here in parent's space; the stub's
    // mach_msg(RCV) drains it.
    mach_port_name_t parentReplyName = MACH_PORT_NULL;
    kr = mach_port_allocate(parentTaskPort, MACH_PORT_RIGHT_RECEIVE,
                            &parentReplyName);
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
    pthread_t thr;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    auto* args = new ThreadArgs{.servicePort=servicePort, .parentTaskPort=parentTaskPort};
    int rc = pthread_create(&thr, &attr, threadEntry, args);
    pthread_attr_destroy(&attr);
    if (rc != 0) {
        delete args;
        fprintf(stdout, "sidecar: pthread_create failed (%d)\n", rc);
        return false;
    }
    return true;
}

}  // namespace sidecar
