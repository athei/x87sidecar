#include "rosetta_core/Translator.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>

#include "TranslatorX87Internal.hpp"
#include "rosetta_core/AssemblerHelpers.hpp"
#include "rosetta_core/Config.h"
#include "rosetta_core/CoreConfig.h"
#include "rosetta_core/IRInstr.h"
#include "rosetta_core/Opcode.h"
#include "rosetta_core/ProfileFormat.h"
#include "rosetta_core/ProfileRuntime.h"
#include "rosetta_core/Register.h"
#include "rosetta_core/TranslationResult.h"
#include "rosetta_core/TranslatorCustom.h"
#include "rosetta_core/TranslatorHelpers.hpp"
#include "rosetta_core/TranslatorX87.h"
#include "rosetta_core/TranslatorX87F80.hpp"
#include "rosetta_core/TranslatorX87Fusion.h"
#include "rosetta_core/TranslatorX87Helpers.hpp"
#include "rosetta_core/X87Cache.h"
#include "rosetta_core/X87IR.h"
#include "rosetta_core/X87Trace.h"

// True for opcode IDs in the two x87 ranges that Rosetta assigns to x87
// instructions.  Mirrors the JIT stub's FILTER prologue range check at
// rosetta_loader/src/stub_asm.cpp:464-467.  Used by the dispatch default
// case so we only complain about *x87* ops we forgot to handle — the AOT
// path (CustomTranslationHook) sees every non-x87 op too and would
// otherwise spam stdout.
static bool is_x87_opcode(uint16_t op) {
    return (op >= kOpcodeName_fcmovb && op <= kOpcodeName_fucomip) ||
           (op >= kOpcodeName_f2xm1 && op <= kOpcodeName_fyl2xp1);
}

// Real body: one x87 instruction (or one fusion / one IR run).  The public
// translate_instruction wraps it so the reply's frontier (last_next_idx) is
// recorded once, after every exit path, and so that a run is one reply.
static auto translate_instruction_impl(TranslationResult* translation_result, IRBlock* block,
                                       IRInstr* instr_array, int64_t num_instrs, int64_t insn_idx)
    -> std::optional<int64_t>;

static profile::BlockTally block_tally(const X87Cache& cache) {
    return profile::BlockTally{
        .ir_ops = cache.tally_ir,
        .peephole_ops = cache.tally_peep,
        .single_ops = cache.tally_single,
        .fallthrough_ops = cache.tally_ft,
        .ir_build_fail_ops = cache.tally_ir_build_fail,
        .ir_fpr_fail_ops = cache.tally_ir_fpr_fail,
        .ir_gpr_fail_ops = cache.tally_ir_gpr_fail,
        .max_gpr_peak = cache.tally_max_gpr_peak,
        .ir_split_runs = cache.tally_ir_split,
        .ir_remat_runs = cache.tally_ir_remat,
        .bridge_ops = cache.tally_bridge,
        .bridge_fail_runs = cache.tally_bridge_fail,
    };
}

// The IRG1 per-reason gate refusal counters and the RRR0 max-run-at-refusal
// side table, both mirrored from the cache.
static void mirror_block_gate_counters(uint32_t bid, const X87Cache& cache) {
    profile::BlockIRGateCounters c{};
    c.counts[profile::kIRGateReasonShortRun] = cache.tally_ir_gate_short_run;
    c.counts[profile::kIRGateReasonTopDirty] = cache.tally_ir_gate_top_dirty;
    c.counts[profile::kIRGateReasonTagPush] = cache.tally_ir_gate_tag_push;
    c.counts[profile::kIRGateReasonDeferredPop] = cache.tally_ir_gate_deferred_pop;
    c.counts[profile::kIRGateReasonPermDirty] = cache.tally_ir_gate_perm_dirty;
    profile::set_block_ir_gate_counters(bid, c);

    profile::BlockMaxRunAtRefuse mr{};
    for (uint32_t r = 0; r < profile::kIRGateReasonCount; ++r) {
        mr.max_run[r] = cache.max_run_at_gate[r];
    }
    profile::set_block_max_run_at_refuse(bid, mr);
}

// What a request can change in the result before it is known to succeed:
// the emitted words, the fixup lists, the register masks and the x87 cache.
// A request whose allocator ran dry is put back to this and declined.
struct RequestEntry {
    uint64_t buf_end;
    uint64_t fixup_count[6];
    uint32_t free_gpr_mask;
    uint32_t free_fpr_mask;
    uint32_t pinned_temporary_scalars;
    X87Cache cache;
};

static constexpr TransactionalList<Fixup> TranslationResult::* kFixupLists[6] = {
    &TranslationResult::external_fixups, &TranslationResult::internal_fixups,
    &TranslationResult::_fixups,         &TranslationResult::field_B0,
    &TranslationResult::dyld_stub_fixups, &TranslationResult::field_1A8,
};

static RequestEntry capture_request_entry(const TranslationResult& tr) {
    RequestEntry e{.buf_end = tr.insn_buf.end,
                   .fixup_count = {},
                   .free_gpr_mask = tr.free_gpr_mask,
                   .free_fpr_mask = tr.free_fpr_mask,
                   .pinned_temporary_scalars = tr._pinned_temporary_scalars,
                   .cache = tr.x87_cache};
    for (size_t i = 0; i < 6; i++) {
        e.fixup_count[i] = (tr.*kFixupLists[i]).element_count();
    }
    return e;
}

// Undo a request that ran out of scratch registers, so that stock, which
// translates the instruction instead, finds the result as the request found
// it.  This is sound at any point inside the request because a request is
// a whole run (see translate_instruction): it starts with the cache released
// and the x87 state complete in memory, and nothing it emitted is kept.  The
// cache goes back to its entry state with the run released, as after any
// declined request.  When the request had moved the cache onto a new block,
// the entry state is the previous block's, so the block is entered afresh
// the next time it is translated.  A request that found stock translating
// the block again is treated the same way, by forgetting the block.  The
// rest of the block goes to stock as well (X87Cache::declined_block).
static void decline_after_alloc_failure(TranslationResult& tr, const RequestEntry& entry,
                                        const IRBlock* block, const IRInstr* instr_array,
                                        int64_t num_instrs, int64_t insn_idx) {
    tr.insn_buf.end = entry.buf_end;
    for (size_t i = 0; i < 6; i++) {
        auto& list = tr.*kFixupLists[i];
        list.end = list.begin + entry.fixup_count[i];
    }
    tr.free_gpr_mask = entry.free_gpr_mask;
    tr.free_fpr_mask = entry.free_fpr_mask;
    tr._pinned_temporary_scalars = entry.pinned_temporary_scalars;

    const uint32_t attempted_bid = tr.x87_cache.profile_bid;
    auto& cache = tr.x87_cache;
    cache = entry.cache;
    cache.invalidate();
    cache.bridge_pending_total = 0;
    cache.bridge_pending_idx = -1;
    if (cache.prev_block == block && insn_idx <= cache.last_insn_idx) {
        cache.prev_block = nullptr;
    }
    const bool same_block = cache.prev_block == block;
    if (same_block) {
        cache.last_insn_idx = static_cast<int32_t>(insn_idx);
    }
    cache.last_next_idx = -1;
    cache.declined_block = block;
    cache.declined_idx = static_cast<int32_t>(insn_idx);
    cache.declined_num_instrs = static_cast<int32_t>(num_instrs);
    cache.declined_start_pc = instr_array[0].pc;
    // The attempt mirrored profile side tables for ops whose code was just
    // dropped.  Within one translation of the block the cache holds what they
    // said before, so mirror that back.  An attempt that had entered the
    // block anew is left alone rather than zeroed, since the block id may
    // carry an earlier translation's counts: its entries, and the
    // latest-write build-fail opcode and top_dirty predecessor in any case,
    // can then reflect the declined attempt.
    if (attempted_bid != profile::kOverflowId && same_block) {
        profile::set_block_tally(attempted_bid, block_tally(cache));
        mirror_block_gate_counters(attempted_bid, cache);
    }
}

// X87_NATIVE_SLOTS: sets the native layout and the slot tracker for one
// reply and clears both on every way out.  The tracker starts with TOP where
// the reply finds it and every slot holding Rosetta's 80-bit value.
struct NativeLayoutScope {
    NativeLayoutScope(bool on, TranslationResult& tr) {
        g_x87_native_layout = on;
        g_x87_slot_tracker = on ? &tr : nullptr;
        if (on) {
            tr.x87_cache.top_rel = 0;
            for (auto& f : tr.x87_cache.slot_fmt) {
                f = kSlotNative;
            }
        }
    }
    ~NativeLayoutScope() {
        g_x87_native_layout = false;
        g_x87_slot_tracker = nullptr;
    }
    NativeLayoutScope(const NativeLayoutScope&) = delete;
    NativeLayoutScope& operator=(const NativeLayoutScope&) = delete;
};

// X87_NATIVE_SLOTS: the slots, as positions relative to TOP at the start of
// the reply (X87Cache::slot_fmt), that still hold an 80-bit value when the
// next `count` instructions from idx read them as binary64.  Simulated from
// the tracker's current state with X87Cache::slot_effect.  Deferred FXCH
// permutes depths, and the permutation is flushed through binary64
// registers at every push, pop, fincstp, fdecstp and fsave, at the start
// when flush_first, and at the end when to_run_end (x87_end flushes it at
// the end of a run), so those flushes read every slot they permute.
//
// The emitters convert nothing themselves: the dispatcher converts this set
// before a fusion or a direct emitter runs, over a window that covers the
// longest fusion (six instructions), and before the IR lowers a run, over
// the whole run.  Converting a slot the instruction ends up not reading, or
// an empty one, costs code but changes no result.
static uint8_t slots_to_convert(const X87Cache& c, const IRInstr* instrs, int64_t num, int64_t idx,
                                int count, bool flush_first, bool to_run_end) {
    int top = c.top_rel;
    int8_t perm[8];
    uint8_t fmt[8];
    for (int i = 0; i < 8; i++) {
        perm[i] = c.perm[i];
        fmt[i] = c.slot_fmt[i];
    }
    uint8_t need = 0;
    const auto read = [&](int phys) {
        const int r = (top + phys) & 7;
        if (fmt[r] == kSlotNative) {
            need = static_cast<uint8_t>(need | (1U << r));
            fmt[r] = kSlotF64;
        }
    };
    const auto flush = [&] {
        for (int i = 0; i < 8; i++) {
            if (perm[i] != i) {
                read(i);
            }
            perm[i] = static_cast<int8_t>(i);
        }
    };
    if (flush_first) {
        flush();
    }
    for (int k = 0; k < count && idx + k < num; k++) {
        const auto e = X87Cache::slot_effect(instrs[idx + k]);
        for (int d = 0; d < 8; d++) {
            if ((e.reads >> d) & 1U) {
                read(perm[d]);
            }
            if ((e.writes >> d) & 1U) {
                fmt[(top + perm[d]) & 7] = kSlotF64;
            }
        }
        switch (e.kind) {
            case X87Cache::kEffectFxch:
                std::swap(perm[0], perm[e.arg]);
                break;
            case X87Cache::kEffectFree:
                fmt[(top + perm[e.arg]) & 7] = kSlotDead;
                break;
            case X87Cache::kEffectTopInc:
            case X87Cache::kEffectTopDec:
                flush();
                top = (top + (e.kind == X87Cache::kEffectTopInc ? 1 : -1)) & 7;
                break;
            case X87Cache::kEffectSave:
            case X87Cache::kEffectRestore:
                for (int i = 0; i < 8; i++) {
                    perm[i] = static_cast<int8_t>(i);
                    fmt[i] = e.kind == X87Cache::kEffectSave ? kSlotDead : kSlotF64;
                }
                top = 0;
                break;
            case X87Cache::kEffectPlain:
                break;
        }
        if (e.pushes != 0 || e.pops != 0) {
            flush();
        }
        for (int p = 0; p < e.pops; p++) {
            fmt[top] = kSlotDead;
            top = (top + 1) & 7;
        }
        for (int p = 0; p < e.pushes; p++) {
            top = (top - 1) & 7;
            fmt[top] = kSlotF64;
        }
    }
    if (to_run_end) {
        flush();
    }
    return need;
}

// Converts each slot in `mask` (positions as in slots_to_convert) from its
// 80-bit value to binary64, in place, and marks it binary64 in the tracker.
// Uses the run cache's base, TOP and Xst_base when it holds them, otherwise
// loads base and TOP into registers of its own; either way it leaves the
// allocator as it found it.  Four scratch GPRs on top of those, so a run
// leaves one register of the eight-register pool to stock; the slot address
// is computed again after the conversion, which needs all four.  No NZCV.
static void emit_slot_conversions(TranslationResult& tr, uint8_t mask) {
    if (mask == 0) {
        return;
    }
    auto& c = tr.x87_cache;
    AssemblerBuffer& buf = tr.insn_buf;
    int Xbase = c.base_gpr;
    int Wtop = c.top_gpr;
    int Xst = (c.gprs_valid && c.st_base_valid) ? c.st_base_gpr : -1;
    const bool own = c.gprs_valid == 0;
    if (own) {
        Xbase = alloc_free_gpr(tr);
        Wtop = alloc_free_gpr(tr);
        emit_x87_base(buf, tr, Xbase);
        emit_load_top(buf, tr, Xbase, Wtop);
        Xst = -1;
    }
    const int Xm = alloc_free_gpr(tr);
    const int Wx = alloc_free_gpr(tr);
    const int Xt1 = alloc_free_gpr(tr);
    const int Xt2 = alloc_free_gpr(tr);
    const int Xfrom = Xst >= 0 ? Xst : Xbase;
    const int16_t off = Xst >= 0 ? 0 : kX87NativeSlotOff;
    for (int r = 0; r < 8; r++) {
        if (((mask >> r) & 1U) == 0) {
            continue;
        }
        const int depth = (r - c.top_rel) & 7;
        emit_native_slot_key(buf, Wtop, depth, Xt1);
        emit_add_sub_shifted_reg(buf, 1, 0, 0, 0, Xt1, 0, Xfrom, Xt1);
        emit_ldur_stur(buf, 3, /*LDUR*/ 1, off, Xt1, Xm);
        emit_ldr_str_imm(buf, 1, 0, /*LDRH*/ 1, static_cast<int16_t>((off + 8) / 2), Xt1, Wx);
        TranslatorX87::emit_f80_bits_to_f64(buf, Xm, Wx, Xt1, Xt2);
        emit_native_slot_key(buf, Wtop, depth, Xt1);
        emit_add_sub_shifted_reg(buf, 1, 0, 0, 0, Xt1, 0, Xfrom, Xt1);
        emit_ldur_stur(buf, 3, /*STUR*/ 0, off, Xt1, Xm);
        c.slot_fmt[r] = kSlotF64;
    }
    free_gpr(tr, Xt2);
    free_gpr(tr, Xt1);
    free_gpr(tr, Wx);
    free_gpr(tr, Xm);
    if (own) {
        free_gpr(tr, Wtop);
        free_gpr(tr, Xbase);
    }
}

// Flush every deferred piece of x87 state to memory and release the run.
// Only for the path that must not happen: an instruction inside a run the
// lookahead counted as handled came back declined.  The ops already emitted
// stay, stock takes over at the declined one, and it must see a complete
// X87State when it does.
static void flush_run_and_release(TranslationResult& tr) {
    auto& cache = tr.x87_cache;
    AssemblerBuffer& buf = tr.insn_buf;
    // x87_end below flushes a deferred FXCH permutation through binary64
    // registers; the slots it moves have to be converted first.
    if (cache.native_slots && cache.perm_dirty) {
        emit_slot_conversions(tr, slots_to_convert(cache, nullptr, 0, 0, 0, /*flush_first=*/true,
                                                   /*to_run_end=*/false));
    }
    cache.run_remaining = 1;
    auto [Xbase, Wd_top] = TranslatorX87::x87_begin(tr, buf);
    const int Wd_tmp = alloc_gpr(tr, 2);
    TranslatorX87::x87_end(tr, buf, Xbase, Wd_top, Wd_tmp, /*consumed=*/1);
    free_gpr(tr, Wd_tmp);
    cache.tick();
    tr.free_gpr_mask = cache.stock_free_gpr_mask;
}

// Bytes an x87 instruction reads or writes through its memory operand.
static int x87_access_bytes(const IRInstr& insn, const IROperand& op) {
    switch (insn.opcode()) {
        case Opcode::kOpcodeName_fbld:
        case Opcode::kOpcodeName_fbstp:
            return 10;
        case Opcode::kOpcodeName_fsave:
        case Opcode::kOpcodeName_frstor:
            return 108;  // the 32-bit layout, the only one the emitters write
        default:
            break;
    }
    switch (op.mem.size) {
        case IROperandSize::S16:
            return 2;
        case IROperandSize::S32:
            return 4;
        case IROperandSize::S64:
            return 8;
        case IROperandSize::S80:
            return 10;
        default:
            return 1;
    }
}

static bool x87_writes_memory(uint16_t opcode) {
    switch (opcode) {
        case Opcode::kOpcodeName_fst:
        case Opcode::kOpcodeName_fstp:
        case Opcode::kOpcodeName_fist:
        case Opcode::kOpcodeName_fistp:
        case Opcode::kOpcodeName_fisttp:
        case Opcode::kOpcodeName_fbstp:
        case Opcode::kOpcodeName_fnstcw:
        case Opcode::kOpcodeName_fstsw:
        case Opcode::kOpcodeName_fsave:
            return true;
        default:
            return false;
    }
}

// A memory fault in translated code is recovered by rolling back to the
// start of the instruction-map entry the faulting pc belongs to, and there
// is one entry per reply.  The runtime only rolls back when nothing before
// the faulting instruction in that entry changed state: no store, no NZCV
// or FPCR write, no write to a register that holds guest state, and no
// floating-point arithmetic.  Otherwise it aborts the process with
// "unexpectedly need to EmulateForward on a synchronous exception".  A reply
// that converts the register file changes state before its first guest
// access (the entry conversion alone does), so with X87_PRECISE_FAULTS=1
// the access of the reply's first instruction is probed here, before
// anything else.  The knob is off by default, since a store probe into
// write-combined memory is uncached and slow (docs/internals.md).
//
// A load is probed with a load of the access width into the zero register,
// or of its first and last byte for 10 and 108 bytes; one load also covers
// an unaligned access that crosses a page.  A store is probed by loading
// those bytes and writing them back unchanged: the full access width for 2,
// 4 and 8 bytes, otherwise the first byte after a load of the last.  The
// write-back is the reply's only state change before the probe completes,
// so a fault in it still rolls back, and it faults where the store would,
// including on a read-only page.  An atomic would leave no window, but LSE
// atomics take a bus error on write-combined and uncached mappings (Metal
// buffers, IOSurfaces), where plain loads and stores work.  The window: a
// write by another thread to those bytes between the load and the
// write-back is undone, and the real store then overwrites the same bytes,
// so only that transient value is lost, which is an order the x86 store
// could have produced.
//
// Once the probes pass, the real access later in the reply cannot fault,
// and a fault in a probe reaches the guest handler with the reply's first
// instruction as rip and the state from before it.  A store into a
// PROT_NONE page faults in the probe's load, so the handler sees a read
// fault where stock reports a write.  Accesses further into a reply are
// not probed: rolling back to the reply start would report the wrong
// instruction.
static void emit_fault_probe(TranslationResult& tr, IRInstr& insn) {
    IROperand* mem = nullptr;
    for (int i = 0; i < insn.num_operands && i < 4; i++) {
        const auto kind = insn.operands[i].kind;
        if (kind == IROperandKind::MemRef || kind == IROperandKind::AbsMem) {
            mem = &insn.operands[i];
            break;
        }
    }
    if (mem == nullptr) {
        return;
    }
    const auto saved_mask = tr.free_gpr_mask;
    tr.free_gpr_mask &= kGprScratchMask;
    AssemblerBuffer& buf = tr.insn_buf;
    const int bytes = x87_access_bytes(insn, *mem);
    const int Xaddr = compute_operand_address(tr, /*is_64bit=*/1, mem, GPR::XZR);
    const bool store = x87_writes_memory(insn.opcode());
    const int Xval = store ? alloc_free_gpr(tr) : GPR::XZR;
    if (bytes == 2 || bytes == 4 || bytes == 8) {
        const int size = bytes == 2 ? 1 : bytes == 4 ? 2 : 3;
        emit_ldr_str_imm(buf, size, /*is_fp=*/0, /*LDR*/ 1, /*imm12=*/0, Xaddr, Xval);
        if (store) {
            emit_ldr_str_imm(buf, size, 0, /*STR*/ 0, 0, Xaddr, Xval);
        }
    } else {
        if (bytes > 1) {
            emit_ldr_str_imm(buf, /*size=*/0, 0, /*LDR*/ 1, bytes - 1, Xaddr, GPR::XZR);
        }
        emit_ldr_str_imm(buf, 0, 0, /*LDR*/ 1, /*imm12=*/0, Xaddr, Xval);
        if (store) {
            emit_ldr_str_imm(buf, 0, 0, /*STR*/ 0, 0, Xaddr, Xval);
        }
    }
    if (store) {
        free_gpr(tr, Xval);
    }
    free_gpr(tr, Xaddr);
    tr.free_gpr_mask = saved_mask;
}

auto Translator::translate_instruction(TranslationResult* translation_result, IRBlock* block,
                                       IRInstr* instr_array, int64_t num_instrs, int64_t insn_idx)
    -> std::optional<int64_t> {
    auto& cache = translation_result->x87_cache;
    const auto start = translation_result->insn_buf.end;
    const auto op = instr_array[insn_idx].opcode();
    translation_result->alloc_failure = nullptr;
    // The rest of a block in which a request ran out of registers goes to
    // stock (X87Cache::declined_block).  ARPL cannot, and reads no x87 flags.
    if (cache.declined_block != nullptr) {
        if (cache.declined_block != block || insn_idx <= cache.declined_idx ||
            num_instrs != cache.declined_num_instrs ||
            instr_array[0].pc != cache.declined_start_pc) {
            cache.declined_block = nullptr;
        } else if (op != kOpcodeName_arpl) {
            cache.last_next_idx = -1;
            return std::nullopt;
        }
    }
    // Metadata-only ops and FXSAVE/FXRSTOR go to stock, which consumes
    // native state directly. Every handled x87 reply uses compact state
    // internally and must export native f80 after its cache is flushed.
    const bool native_boundary = is_x87_opcode(op) && op != kOpcodeName_fclex &&
                                 op != kOpcodeName_finit && op != kOpcodeName_fldenv &&
                                 op != kOpcodeName_fstenv && op != kOpcodeName_fxsave &&
                                 op != kOpcodeName_fxrstor;
    // A reply made only of integer loads and stores, control word moves and
    // no-ops does no arithmetic, so it can work on the native f80 slots
    // directly and needs no conversion at either end.  That keeps an int64
    // held on the x87 stack across replies exact, which the binary64 round
    // trip of the boundary cannot (Delphi's System.Move holds fild values
    // across its branches).  The run lookahead decides it, so the predicate
    // covers every op the reply below can consume: bridging is off for such
    // a run and an IR run never extends past it.
    const bool transport =
        native_boundary &&
        (g_rosetta_config == nullptr || g_rosetta_config->native_transport != 0) &&
        X87Cache::is_native_transport_run(instr_array, num_instrs, insn_idx);
    // X87_NATIVE_SLOTS: the reply keeps its working binary64 values inside
    // the native slots (+6, stride 10).  A native transport reply needs no
    // binary64 values at all and keeps its own emitters.
    const bool slots = native_boundary && !transport && g_rosetta_config != nullptr &&
                       g_rosetta_config->native_slots != 0;
    // Captured before native_transport and native_slots are set, so a
    // declined request leaves both clear, as every request that ends does.
    const RequestEntry entry = capture_request_entry(*translation_result);
    cache.native_transport = transport ? 1 : 0;
    cache.native_slots = slots ? 1 : 0;
    const NativeLayoutScope layout(slots, *translation_result);
    // The probe has to stay the first thing in the reply: before the entry
    // conversion, before any NZCV save or FPCR write, before anything an op
    // emitter does.  A native transport reply needs none: it is one
    // instruction (or a fused fild m64 / fistp m64 pair), and its emitters
    // touch guest memory before they change any state, so its own access
    // rolls back and reports the fault kind stock does.  X87_PROFILE's block
    // counter and X87_TRACE_BLOCK's records are stores emitted ahead of the
    // op, so with either on a fault in a transport reply that carries them
    // aborts.
    if (native_boundary && !transport && precise_faults_enabled())
        emit_fault_probe(*translation_result, instr_array[insn_idx]);
    const bool traced =
        native_boundary && x87trace::matches(instr_array, static_cast<size_t>(num_instrs));
    const uint64_t trace_site = uint64_t(instr_array[insn_idx].pc) << 32;
    if (traced)
        x87trace::emit_boundary(*translation_result, trace_site | (uint64_t(insn_idx) << 1));
    // With X87_NATIVE_SLOTS nothing is converted on entry: each slot is
    // converted where the reply first reads it (emit_slot_conversions).
    if (native_boundary && !transport && !slots)
        TranslatorX87::emit_native_state_boundary(*translation_result, true);
    auto ret = translation_result->alloc_failure != nullptr
                   ? std::nullopt
                   : translate_instruction_impl(translation_result, block, instr_array,
                                                num_instrs, insn_idx);

    // A run is one reply.  Stock records one instruction-map entry per
    // translate_insn reply, and when an asynchronous signal lands in a
    // translation Rosetta's runtime steps the ARM code forward to the next
    // entry, takes the guest state from the thread context there, and resumes
    // from that state after the handler.  Inside a run the architectural TOP
    // lives in a register, tag updates are pending and FXCH is a compile-time
    // permutation, none of which is in memory at an intermediate entry, so
    // each such entry was a point where a signal silently shifted the x87
    // stack (issue #23).  With the whole run in one reply the only entries
    // are the run's start and its end, where x87_end has flushed everything.
    while (ret.has_value() && cache.active() && *ret < num_instrs &&
           translation_result->alloc_failure == nullptr) {
        cache.last_next_idx = static_cast<int32_t>(*ret);
        const auto more =
            translate_instruction_impl(translation_result, block, instr_array, num_instrs, *ret);
        if (!more.has_value() && translation_result->alloc_failure != nullptr) {
            break;
        }
        if (!more.has_value()) {
            static int declined_count = 0;
            if (++declined_count <= 10) {
                std::fprintf(stdout,
                             "[rosettax87] x87 run: instruction %lld of block hash=0x%016llx "
                             "declined mid-run, flushing (#%d)\n",
                             static_cast<long long>(*ret),
                             static_cast<unsigned long long>(cache.profile_hash), declined_count);
                std::fflush(stdout);
            }
            flush_run_and_release(*translation_result);
            break;
        }
        ret = more;
    }
    cache.native_transport = 0;
    cache.native_slots = 0;
    if (native_boundary && translation_result->alloc_failure == nullptr) {
        if (ret) {
            if (slots)
                TranslatorX87::emit_native_slots_export(*translation_result);
            else if (!transport)
                TranslatorX87::emit_native_state_boundary(*translation_result, false);
            if (traced)
                x87trace::emit_boundary(*translation_result, trace_site | (uint64_t(*ret) << 1) | 1,
                                        *ret == num_instrs);
        } else {
            // Declined: drop the boundary and the fault probe, including
            // any fixup the probe's address computation recorded.
            translation_result->insn_buf.end = start;
            for (size_t i = 0; i < 6; i++) {
                auto& list = translation_result->*kFixupLists[i];
                list.end = list.begin + entry.fixup_count[i];
            }
        }
    }
    if (translation_result->alloc_failure != nullptr) {
        // ARPL is the one opcode stock cannot be handed: it never had an id
        // for it (see translate_instruction_impl).  The register form, the
        // only one seen in practice, needs four scratch GPRs, so this is not
        // expected to happen.
        if (op == kOpcodeName_arpl) {
            std::fprintf(stdout,
                         "[rosettax87] %s failed while translating ARPL, which stock cannot "
                         "translate; aborting\n",
                         translation_result->alloc_failure);
            std::fflush(stdout);
            std::abort();
        }
        decline_after_alloc_failure(*translation_result, entry, block, instr_array, num_instrs,
                                    insn_idx);
        return std::nullopt;
    }
    cache.last_next_idx = ret.has_value() ? static_cast<int32_t>(*ret) : -1;
    return ret;
}

static auto translate_instruction_impl(TranslationResult* translation_result, IRBlock* block,
                                       IRInstr* instr_array, int64_t num_instrs, int64_t insn_idx)
    -> std::optional<int64_t> {
    auto* const cur_instr = &instr_array[insn_idx];
    const auto opcode = cur_instr->opcode();
    auto& cache = translation_result->x87_cache;

    // ── OPT-1: x87 cross-instruction cache management ───────────────────────
    // Invalidate the cache if we've moved to a different block (between blocks,
    // branches may have executed non-x87 code that clobbers scratch GPRs).
    // Then, if no cache is active, scan ahead to find the length of the current
    // consecutive x87 run.  x87_cache_set_run only activates if run >= 2.
    {
        // Same block, but not a continuation: stock started the block's
        // translation over.  Either it asks at or before an index we already
        // handled, or a run is active and it asks anywhere but the frontier
        // the previous reply set (a re-ask inside a span a fusion or IR run
        // consumed).  In both cases the emitted code that holds base/TOP in
        // registers belongs to a pass stock discarded, so the cache is reset
        // as on a block change and this op is emitted from memory state.
        // Neither has been observed in a real run so far; the line below is
        // there so a run that does hit one says so.
        const bool midspan_reask = block == cache.prev_block && cache.active() &&
                                   cache.last_next_idx >= 0 &&
                                   insn_idx != static_cast<int64_t>(cache.last_next_idx);
        const bool restart_same_block =
            (block == cache.prev_block && insn_idx <= cache.last_insn_idx) || midspan_reask;
        if (restart_same_block) {
            static int restart_count = 0;
            ++restart_count;
            if (restart_count <= 10 || restart_count % 1000 == 0) {
                std::fprintf(stdout,
                             "[rosettax87] x87 cache: block hash=0x%016llx translated again "
                             "(ask=%lld last_in=%d frontier=%d run=%d), resetting cache (#%d)\n",
                             static_cast<unsigned long long>(cache.profile_hash),
                             static_cast<long long>(insn_idx), cache.last_insn_idx,
                             cache.last_next_idx, static_cast<int>(cache.run_remaining),
                             restart_count);
                std::fflush(stdout);
            }
        }
        if (block != cache.prev_block || restart_same_block) {
            cache.invalidate();
            cache.prev_block = block;
            // Stock carries its register-allocator state across the block
            // in free_gpr_mask (TR+0x210) — capture the incoming word (the
            // exit paths below write it back) and restrict our working
            // mask to the scratch pool.  Done before the block-counter
            // bump so its scratch allocation draws from the restricted
            // mask.  Never overwrite the field with kGprScratchMask: stock
            // would then reuse regs it still holds live (miscompiles), and
            // our emits would clobber stock-live values at runtime.
            cache.stock_free_gpr_mask = translation_result->free_gpr_mask;
            translation_result->free_gpr_mask &= kGprScratchMask;
            cache.last_insn_idx = -1;
            cache.last_next_idx = -1;
            cache.tally_ir = 0;
            cache.tally_peep = 0;
            cache.tally_single = 0;
            cache.tally_ft = 0;
            cache.tally_ir_build_fail = 0;
            cache.tally_ir_fpr_fail = 0;
            cache.tally_ir_gpr_fail = 0;
            cache.tally_ir_split = 0;
            cache.tally_ir_remat = 0;
            cache.tally_bridge = 0;
            cache.tally_bridge_fail = 0;
            cache.tally_max_gpr_peak = 0;
            cache.tally_ir_gate_short_run = 0;
            cache.tally_ir_gate_top_dirty = 0;
            cache.tally_ir_gate_tag_push = 0;
            cache.tally_ir_gate_deferred_pop = 0;
            cache.tally_ir_gate_perm_dirty = 0;
            for (auto& v : cache.max_run_at_gate) {
                v = 0;
            }
            cache.prev_x87_opcode = 0xFFFFU;
            // X87_FAST_ROUND=2: one scan per block for control-word writers;
            // x87_fast_round_active keeps the full RC dispatch in blocks
            // that contain one.
            cache.block_has_cw_write = 0;
            if (g_rosetta_config && g_rosetta_config->fast_round == 2) {
                for (int64_t i = 0; i < num_instrs; i++) {
                    switch (instr_array[i].opcode()) {
                        case kOpcodeName_fldcw:
                        case kOpcodeName_fldenv:
                        case kOpcodeName_frstor:
                        case kOpcodeName_fxrstor:
                        case kOpcodeName_finit:
                        case kOpcodeName_fsave:
                            cache.block_has_cw_write = 1;
                            break;
                        default:
                            break;
                    }
                    if (cache.block_has_cw_write) {
                        break;
                    }
                }
            }
            cache.profile_bid = profile::kOverflowId;
            // Compute the IR hash unconditionally so the X87_*_HASH_LIST
            // rollback gate works without X87_PROFILE.  The hash is
            // FNV-1a over the IR stream with each instr's `pc` zeroed,
            // so it identifies the block by content and is stable across
            // launches (unlike profile_bid, which is registration order).
            cache.profile_hash =
                profile::hash_ir_stream(instr_array, static_cast<size_t>(num_instrs));
            if (profile::counter_array_addr() != 0) {
                const uint32_t bid = profile::register_block(block, cache.profile_hash);
                if (bid != profile::kOverflowId) {
                    emit_block_counter_bump(*translation_result, bid);
                    cache.profile_bid = bid;
                }
            }
        } else {
            // Same block, new x87 entry.  Off-run the incoming word is
            // stock's current allocator state (possibly advanced by stock
            // since our last writeback) — recapture it.  Mid-run it is our
            // own previous writeback (captured & ~pins); keep the earlier
            // capture, which still holds the pinned bits.  Either way the
            // working mask is pool-restricted.
            if (!cache.active()) {
                cache.stock_free_gpr_mask = translation_result->free_gpr_mask;
            }
            translation_result->free_gpr_mask &= kGprScratchMask;
        }
        cache.last_insn_idx = static_cast<int32_t>(insn_idx);
        if (!cache.active()) {
            const bool cache_disabled = g_rosetta_config && g_rosetta_config->disable_x87_cache;
            if (!cache_disabled) {
                // A native transport run is answered one instruction per
                // reply, so every integer load and store is the first
                // instruction of its reply, and its own access, which comes
                // before any state change, faults precisely (see the probe
                // call in translate_instruction).  Transport replies have no entry or
                // exit conversion, so the extra replies cost little.  An
                // adjacent fild m64 / fistp m64 pair stays one reply (see
                // try_fuse_fld_group).
                const int run = cache.native_transport && precise_faults_enabled()
                                    ? 1
                                    : X87Cache::lookahead(instr_array, num_instrs, insn_idx);
                cache.set_run(run);
                // Run bridging: at a fresh run start, check whether short v1
                // gaps join this run to following x87 segments.  Stash the
                // descriptor; the IR dispatch below fires the all-or-nothing
                // attempt when it reaches this index with no deferred state.
                cache.bridge_pending_total = 0;
                cache.bridge_pending_idx = -1;
                // Per-block bisect: include list restricts bridging to the
                // listed IR-content hashes; exclude list vetoes them
                // (exclude wins).  cache.profile_hash is populated on block
                // transition, before any run start in the block.
                bool bridge_allowed =
                    g_rosetta_config != nullptr && g_rosetta_config->enable_bridge != 0;
                if (bridge_allowed) {
                    const auto& cfg = *g_rosetta_config;
                    if (!cfg.x87_no_bridge_hash_list.empty() &&
                        std::ranges::binary_search(cfg.x87_no_bridge_hash_list,
                                                   cache.profile_hash)) {
                        bridge_allowed = false;
                    } else if (!cfg.x87_bridge_hash_list.empty() &&
                               !std::ranges::binary_search(cfg.x87_bridge_hash_list,
                                                           cache.profile_hash)) {
                        bridge_allowed = false;
                    }
                }
                // A native transport run is not bridged: the bridged region
                // would be lowered by the IR into the compact layout.
                if (bridge_allowed && run >= 1 && !cache.native_transport) {
                    const auto br = X87Cache::lookahead_bridged(
                        instr_array, num_instrs, insn_idx, g_rosetta_config->bridge_max_gap,
                        g_rosetta_config->bridge_max_total,
                        g_rosetta_config->enable_bridge_v2 != 0);
                    if (br.total > run) {
                        cache.bridge_pending_total = br.total;
                        cache.bridge_pending_x87 = br.x87_count;
                        cache.bridge_pending_plain = static_cast<int16_t>(run);
                        cache.bridge_pending_idx = static_cast<int16_t>(insn_idx);
                    }
                }
            }
        }
    }

    // ── ARPL: the one non-x87 opcode we claim ───────────────────────────────
    // Synthetic, produced by the decode hook for `63 /r`, which stock cannot
    // decode at all (0x63 is MOVSXD in 64-bit mode, so its tables have no ARPL)
    // and therefore cannot translate either — falling through to stock would
    // hand it an opcode id it has never heard of.  Handled here, above the x87
    // dispatch, because none of the x87 run / cache / bridging machinery
    // applies: a run is a span of consecutive x87 opcodes and bridging only
    // crosses a fixed set of mov/lea/ALU shapes, neither of which includes
    // ARPL, so an ARPL can never land mid-run.  The mask handling mirrors the
    // dispatch's own exit paths: allocate from the pool-restricted working
    // mask (already restricted above), then hand stock back its own word.
    if (opcode == kOpcodeName_arpl) {
        TranslatorCustom::translate_arpl(translation_result, cur_instr);
        translation_result->free_gpr_mask = cache.stock_free_gpr_mask;
        return insn_idx + 1;
    }

    // X87_PROFILE — mirror the in-cache tally to ProfileRuntime so the sidecar
    // dump (kqueue NOTE_EXIT) reads the final count.  No-op when profiling is
    // disabled or this block hit kMaxBlocks.
    const auto mirror_tally = [&] {
        if (cache.profile_bid != profile::kOverflowId) {
            profile::set_block_tally(cache.profile_bid, block_tally(cache));
        }
    };

    // Mirror the per-reason gate-refusal counters (separate side-table from
    // BlockTally; see ProfileFormat.h IRG1 section).  Called whenever any of
    // the 5 gate counters increments.  Also mirrors the max-run-at-refusal
    // side-table (RRR0) — same trigger.
    const auto mirror_gate_counters = [&] {
        if (cache.profile_bid != profile::kOverflowId) {
            mirror_block_gate_counters(cache.profile_bid, cache);
        }
    };

    // Update max_run_at_gate[reason] = max(prev, current run_remaining).
    // Called inline at each gate refusal branch — captures the run length
    // at the moment of refusal (before any tick() decrement).
    const auto bump_max_run = [&](uint16_t reason) {
        const auto run = static_cast<uint16_t>(cache.run_remaining);
        cache.max_run_at_gate[reason] = std::max(run, cache.max_run_at_gate[reason]);
    };

    // Consolidate the rollback-eligibility check used by the top_dirty /
    // deferred_pop gates.  Returns true when the per-branch enable knob is
    // set AND the hash include/exclude lists allow this block.  Hash exclude
    // wins over hash include.  Empty hash lists are no-ops.
    const auto rollback_allowed = [&](uint8_t branch_enable) -> bool {
        if (g_rosetta_config == nullptr || branch_enable == 0) {
            return false;
        }
        const auto& cfg = *g_rosetta_config;
        if (!cfg.x87_no_rollback_hash_list.empty() &&
            std::ranges::binary_search(cfg.x87_no_rollback_hash_list, cache.profile_hash)) {
            return false;
        }
        if (!cfg.x87_rollback_hash_list.empty() &&
            !std::ranges::binary_search(cfg.x87_rollback_hash_list, cache.profile_hash)) {
            return false;
        }
        return true;
    };

    // ── IR pipeline: try whole-run optimization for runs of 3+ ─────────────
    // The IR fires once at the start of a fresh run (no deferred cache state).
    // It builds an SSA-like IR, runs optimization passes (DSE, FMA, FCOM+FSTSW
    // fusion), and lowers directly to AArch64.
    //
    // X87_PROFILE diagnostic: when the cache is active (i.e. an x87 run is
    // in flight) but one of the eligibility conditions below refuses entry
    // to compile_run, record which condition fired into the IRG0 side-table.
    // Latest-write-wins per block — analyzer aggregates exec-weighted across
    // blocks to surface the dominant refusal cause.  Was previously a single
    // combined &&-chain whose failures were invisible to the profiler.
    // ── Run bridging: all-or-nothing bridged IR attempt ─────────────────────
    // Fires exactly once per stashed descriptor, at the region's start
    // instruction, and only with no deferred cache state (a fresh run start
    // has none; the guard is belt-and-braces).  On success the whole region
    // — x87 segments plus the mov/lea bridges joining them — lowers as ONE
    // IR run, so the FP stack never round-trips through memory at the gaps.
    // On any failure nothing was emitted (build/gate failures precede
    // lower()); restore the plain run and fall through to the normal paths.
    {
        const bool ir_disabled = g_rosetta_config && g_rosetta_config->disable_x87_ir;
        if (!ir_disabled && cache.bridge_pending_total > 0 &&
            cache.bridge_pending_idx == static_cast<int16_t>(insn_idx) && cache.top_dirty == 0 &&
            cache.tag_push_pending == 0 && cache.deferred_pop_count == 0 && cache.perm_dirty == 0) {
            const int total = cache.bridge_pending_total;
            const int x87_count = cache.bridge_pending_x87;
            const int plain_run = cache.bridge_pending_plain;
            cache.bridge_pending_total = 0;
            cache.bridge_pending_idx = -1;

            cache.set_run(total);  // pin GPRs across the whole region
            // X87_NATIVE_SLOTS: convert what the region reads before it is
            // lowered, and take it back if it falls back.
            const uint64_t pre_convert_end = translation_result->insn_buf.end;
            uint8_t pre_convert_fmt[8];
            std::copy(std::begin(cache.slot_fmt), std::end(cache.slot_fmt), pre_convert_fmt);
            if (cache.native_slots) {
                emit_slot_conversions(*translation_result,
                                      slots_to_convert(cache, instr_array, num_instrs, insn_idx,
                                                       total, /*flush_first=*/false,
                                                       /*to_run_end=*/true));
            }
            X87IR::IRFailReason bridge_reason = X87IR::IRFailReason::kNone;
            const int consumed =
                X87IR::compile_run(translation_result, instr_array, num_instrs, insn_idx, total,
                                   &bridge_reason, nullptr, nullptr, /*bridged=*/true,
                                   /*bridges_v2=*/g_rosetta_config->enable_bridge_v2 != 0);
            if (consumed == total) {
                if (cache.tally_bridge != 0xFFFFU) {
                    cache.tally_bridge =
                        static_cast<uint16_t>(cache.tally_bridge + (total - x87_count));
                }
                cache.tally_ir = static_cast<uint16_t>(cache.tally_ir + x87_count);
                mirror_tally();
                if (g_rosetta_config->log_bridge) {
                    std::fprintf(stderr,
                                 "[x87-bridge] joined region: total=%d x87=%d bridges=%d "
                                 "hash=0x%016llx idx=%lld\n",
                                 total, x87_count, total - x87_count,
                                 static_cast<unsigned long long>(cache.profile_hash),
                                 static_cast<long long>(insn_idx));
                }
                for (int i = 0; i < total; i++) {
                    cache.tick();
                }
                if (cache.active()) {
                    translation_result->free_gpr_mask =
                        cache.stock_free_gpr_mask & ~cache.pinned_mask();
                } else {
                    translation_result->free_gpr_mask = cache.stock_free_gpr_mask;
                }
                translation_result->free_fpr_mask =
                    translation_result->_unoccupied_temporary_fprs_for_xmm_scalars;
                translation_result->_pinned_temporary_scalars = 0;
                return insn_idx + total;
            }
            // Fallback: nothing emitted but the conversions above, which are
            // taken back; restore the plain x87 run.  Direct field write:
            // set_run() ignores lengths < 2, which would leave run_remaining
            // at the bridged total and keep the cache active across the gap's
            // stock-translated instructions.
            translation_result->insn_buf.end = pre_convert_end;
            std::copy(std::begin(pre_convert_fmt), std::end(pre_convert_fmt),
                      std::begin(cache.slot_fmt));
            cache.run_remaining = static_cast<int16_t>(plain_run >= 2 ? plain_run : 0);
            if (cache.tally_bridge_fail != 0xFFFFU) {
                cache.tally_bridge_fail++;
            }
            if (g_rosetta_config->log_bridge) {
                std::fprintf(stderr,
                             "[x87-bridge] fallback: total=%d reason=%d hash=0x%016llx idx=%lld\n",
                             total, static_cast<int>(bridge_reason),
                             static_cast<unsigned long long>(cache.profile_hash),
                             static_cast<long long>(insn_idx));
            }
        }
    }

    {
        // The IR lowers into the compact layout, which a native transport
        // reply does not have.
        const bool ir_disabled =
            (g_rosetta_config && g_rosetta_config->disable_x87_ir) || cache.native_transport != 0;
        if (!ir_disabled && cache.active()) {
            // Measurement-only override: profile_analyze can ask for compile_run
            // to fire regardless of the gate so we can score "what would IR emit
            // for this pattern if the gate were lifted?".  The emitted code is
            // not necessarily correct under the dirty conditions; the analyzer
            // discards it and only reads insn_buf.end / 4.
            const bool force_gate =
                g_rosetta_config != nullptr && g_rosetta_config->force_x87_ir_gate != 0;
            bool gate_refused = false;
            // Speculative-flush rollback state.  The gate-refusal cascade
            // may emit a small flush (3-14 ARM) and clear a deferred cache
            // field, then fall through to compile_run.  If compile_run
            // subsequently bails on kFprPressure / kGprPressure, the
            // flush ARM is wasted — the next dispatch path (peephole /
            // single-op) handles the deferred state itself.  Rollback
            // rewinds insn_buf.end and restores the cleared cache field.
            //
            // perm_dirty participates unconditionally.  top_dirty and
            // deferred_pop default ON since 2026-05-06: the
            // X87IRLower::lower() prologue flush (`855a424`) closed the
            // cascade hole that previously corrupted WoW geom + weapon
            // when these branches rolled back.  X87_ENABLE_ROLLBACK_*=0
            // remains as a per-branch kill-switch for bisect / future
            // diagnostic work.  X87_LOG_ROLLBACK=1 prints one stdout
            // line per firing.
            //
            // The standalone tag_push_pending refusal arm was removed
            // 2026-05-06: lower()'s prologue at X87IRLower.cpp:343-350
            // emits the tag clear when it runs, and compile_run's
            // FPR/GPR pressure check (X87IRLower.cpp:1536-1564) bails
            // before lower(), so cache.tag_push_pending is preserved on
            // bail.  The cascade absorbs the common (top_dirty,
            // tag_push) co-set case at the top_dirty arm; the rare
            // tag_push-only case falls through with no gate-side flush.
            const uint64_t saved_buf_end = translation_result->insn_buf.end;
            const int8_t saved_perm_dirty = cache.perm_dirty;
            int8_t saved_perm[8];
            for (int i = 0; i < 8; i++) {
                saved_perm[i] = cache.perm[i];
            }
            // Captured unconditionally (cheap); restored only when the
            // matching X87_ENABLE_ROLLBACK_* knob is set.
            const int8_t saved_top_dirty = cache.top_dirty;
            const int8_t saved_tag_push_pending = cache.tag_push_pending;
            const int8_t saved_deferred_pop_count = cache.deferred_pop_count;
            // X87_NATIVE_SLOTS: slots converted after this point are taken
            // back with the code that converted them.
            uint8_t saved_slot_fmt[8];
            std::copy(std::begin(cache.slot_fmt), std::end(cache.slot_fmt), saved_slot_fmt);
            bool flushed = false;
            // Track which branch set `flushed` so the X87_LOG_ROLLBACK
            // diagnostic can name it.  kNone when no branch fired.
            enum RollbackBranch : uint8_t {
                kRollbackNone = 0,
                kRollbackTopDirty,
                kRollbackDeferredPop,
                kRollbackPermDirty,
            } flushed_branch = kRollbackNone;
            if (!force_gate) {
                if (cache.run_remaining < 3) {
                    cache.tally_ir_gate_short_run =
                        static_cast<uint16_t>(cache.tally_ir_gate_short_run + 1);
                    bump_max_run(profile::kIRGateReasonShortRun);
                    gate_refused = true;
                } else if (cache.top_dirty != 0) {
                    // FLUSH top_dirty only (3 ARM).  tag_push_pending is
                    // commonly set alongside (x87_push set both); we leave
                    // it untouched here.  If compile_run consumes the run
                    // and lower() runs, lower's prologue
                    // (X87IRLower.cpp:343-350) emits the tag clear.  If
                    // compile_run bails before lower (FPR / GPR pressure),
                    // cache.tag_push_pending is still set; the next call
                    // hits this branch again or the deferred_pop /
                    // perm_dirty arms.  The standalone tag_push refusal arm
                    // was removed 2026-05-06: lower()'s prologue makes the
                    // fall-through correct, and the cascade absorbs the
                    // common (top_dirty, tag_push) co-set case here.
                    // Override via X87_GATE_FLUSH_THRESHOLD; 0 = default 3.
                    const int kMinRunForFlush =
                        (g_rosetta_config != nullptr &&
                         g_rosetta_config->x87_ir_gate_flush_threshold_top_dirty != 0)
                            ? g_rosetta_config->x87_ir_gate_flush_threshold_top_dirty
                            : 3;
                    if (cache.gprs_valid && cache.run_remaining >= kMinRunForFlush) {
                        AssemblerBuffer& buf = translation_result->insn_buf;
                        const int Wd_tmp = alloc_free_gpr(*translation_result);
                        emit_store_top(buf, cache.base_gpr, cache.top_gpr, Wd_tmp);
                        free_gpr(*translation_result, Wd_tmp);
                        cache.top_dirty = 0;
                        bump_max_run(profile::kIRGateReasonTopDirty);
                        if (cache.profile_bid != profile::kOverflowId &&
                            cache.prev_x87_opcode != 0xFFFFU) {
                            profile::set_block_top_dirty_predecessor(cache.profile_bid,
                                                                     cache.prev_x87_opcode);
                        }
                        mirror_gate_counters();
                        // Per-branch kill-switch (default on).  Composes
                        // hash-list bounds via rollback_allowed(); with
                        // empty hash lists this is equivalent to
                        // "branch enable knob set?".
                        if (rollback_allowed(g_rosetta_config != nullptr
                                                 ? g_rosetta_config->x87_enable_rollback_top_dirty
                                                 : 0)) {
                            flushed = true;
                            flushed_branch = kRollbackTopDirty;
                        }
                        // gate_refused stays false — fall through to compile_run.
                    } else {
                        cache.tally_ir_gate_top_dirty =
                            static_cast<uint16_t>(cache.tally_ir_gate_top_dirty + 1);
                        bump_max_run(profile::kIRGateReasonTopDirty);
                        gate_refused = true;
                    }
                } else if (cache.deferred_pop_count != 0) {
                    // FLUSH deferred_pop_count alone (2 + 6*count ARM).  Reaches
                    // here when top_dirty=0 (the top_dirty branch fired earlier
                    // in the run + IR partial-consumed leaving deferred_pop_count
                    // behind).  Cap on count<=2 keeps the flush cost under
                    // ~14 ARM; count>=3 is rare.
                    // Override via X87_GATE_FLUSH_THRESHOLD_DEFERRED_POP; 0 = default 3.
                    const int kMinRunForFlush =
                        (g_rosetta_config != nullptr &&
                         g_rosetta_config->x87_ir_gate_flush_threshold_deferred_pop != 0)
                            ? g_rosetta_config->x87_ir_gate_flush_threshold_deferred_pop
                            : 3;
                    if (cache.gprs_valid && cache.run_remaining >= kMinRunForFlush &&
                        cache.deferred_pop_count <= 2) {
                        AssemblerBuffer& buf = translation_result->insn_buf;
                        const int Wd_tmp = alloc_free_gpr(*translation_result);
                        const int Wd_tmp2 = alloc_free_gpr(*translation_result);
                        const int Wd_tagw = alloc_free_gpr(*translation_result);
                        emit_x87_tag_set_empty_batch(buf, cache.base_gpr, cache.top_gpr, Wd_tmp,
                                                     Wd_tmp2, Wd_tagw, cache.deferred_pop_count);
                        free_gpr(*translation_result, Wd_tagw);
                        free_gpr(*translation_result, Wd_tmp2);
                        free_gpr(*translation_result, Wd_tmp);
                        cache.deferred_pop_count = 0;
                        bump_max_run(profile::kIRGateReasonDeferredPop);
                        mirror_gate_counters();
                        // Per-branch kill-switch (default on).  Composes
                        // hash-list bounds via rollback_allowed().
                        if (rollback_allowed(
                                g_rosetta_config != nullptr
                                    ? g_rosetta_config->x87_enable_rollback_deferred_pop
                                    : 0)) {
                            flushed = true;
                            flushed_branch = kRollbackDeferredPop;
                        }
                    } else {
                        cache.tally_ir_gate_deferred_pop =
                            static_cast<uint16_t>(cache.tally_ir_gate_deferred_pop + 1);
                        bump_max_run(profile::kIRGateReasonDeferredPop);
                        gate_refused = true;
                    }
                } else if (cache.perm_dirty) {
                    // FLUSH perm_dirty alone (~6 ARM for the typical 2-cycle
                    // FXCH-ST(1) case).  By design invariant (every push/pop
                    // path calls perm_flush_before_stack_change first),
                    // perm_dirty=1 implies all other deferred flags=0, so this
                    // branch always fires alone in the cascade.
                    // Override via X87_GATE_FLUSH_THRESHOLD_PERM_DIRTY; 0 = default 3.
                    const int kMinRunForFlush =
                        (g_rosetta_config != nullptr &&
                         g_rosetta_config->x87_ir_gate_flush_threshold_perm_dirty != 0)
                            ? g_rosetta_config->x87_ir_gate_flush_threshold_perm_dirty
                            : 3;
                    if (cache.gprs_valid && cache.run_remaining >= kMinRunForFlush) {
                        AssemblerBuffer& buf = translation_result->insn_buf;
                        // The flush moves the permuted slots through binary64
                        // registers.
                        if (cache.native_slots) {
                            emit_slot_conversions(
                                *translation_result,
                                slots_to_convert(cache, instr_array, num_instrs, insn_idx, 0,
                                                 /*flush_first=*/true, /*to_run_end=*/false));
                        }
                        const int Wd_tmp = alloc_free_gpr(*translation_result);
                        const int Dd_save = alloc_free_fpr(*translation_result);
                        const int Dd_chain = alloc_free_fpr(*translation_result);
                        const int Xst_base =
                            (cache.gprs_valid && cache.st_base_valid) ? cache.st_base_gpr : -1;
                        emit_x87_perm_flush(buf, cache.base_gpr, cache.top_gpr, Wd_tmp, cache.perm,
                                            Xst_base, Dd_save, Dd_chain);
                        free_fpr(*translation_result, Dd_chain);
                        free_fpr(*translation_result, Dd_save);
                        free_gpr(*translation_result, Wd_tmp);
                        cache.reset_perm();
                        flushed = true;
                        flushed_branch = kRollbackPermDirty;
                        bump_max_run(profile::kIRGateReasonPermDirty);
                        mirror_gate_counters();
                    } else {
                        cache.tally_ir_gate_perm_dirty =
                            static_cast<uint16_t>(cache.tally_ir_gate_perm_dirty + 1);
                        bump_max_run(profile::kIRGateReasonPermDirty);
                        gate_refused = true;
                    }
                }
            }
            if (gate_refused) {
                mirror_gate_counters();
            } else {
                X87IR::IRFailReason ir_reason = X87IR::IRFailReason::kNone;
                int ir_peak_gprs = 0;
                // out_fail_opcode is only useful when we'll record it into the
                // profile side-table.  Pass nullptr in non-profile builds so
                // compile_run skips the conditional store entirely.
                const bool profiling_block = cache.profile_bid != profile::kOverflowId;
                uint16_t ir_fail_opcode = profile::kNoBuildFailOpcode;
                // X87_NATIVE_SLOTS: convert every slot the run reads before
                // it is lowered, including the ones lower() flushes a
                // leftover permutation through.  Taken back below if the
                // run is not lowered.
                const uint64_t pre_convert_end = translation_result->insn_buf.end;
                uint8_t pre_convert_fmt[8];
                std::copy(std::begin(cache.slot_fmt), std::end(cache.slot_fmt), pre_convert_fmt);
                if (cache.native_slots) {
                    emit_slot_conversions(
                        *translation_result,
                        slots_to_convert(cache, instr_array, num_instrs, insn_idx,
                                         cache.run_remaining, cache.perm_dirty != 0,
                                         /*to_run_end=*/true));
                }
                const int ir_consumed = X87IR::compile_run(
                    translation_result, instr_array, num_instrs, insn_idx, cache.run_remaining,
                    &ir_reason, &ir_peak_gprs, profiling_block ? &ir_fail_opcode : nullptr);
                if (profiling_block && ir_fail_opcode != profile::kNoBuildFailOpcode) {
                    profile::set_block_build_fail_op(cache.profile_bid, ir_fail_opcode);
                }
                if (ir_peak_gprs > 0) {
                    const auto peak_unsigned = static_cast<uint32_t>(ir_peak_gprs);
                    if (peak_unsigned > cache.tally_max_gpr_peak) {
                        cache.tally_max_gpr_peak = static_cast<uint16_t>(peak_unsigned);
                    }
                }
                if (ir_consumed == 0) {
                    switch (ir_reason) {
                        case X87IR::IRFailReason::kBuildFail:
                            cache.tally_ir_build_fail =
                                static_cast<uint16_t>(cache.tally_ir_build_fail + 1);
                            break;
                        case X87IR::IRFailReason::kFprPressure:
                            cache.tally_ir_fpr_fail =
                                static_cast<uint16_t>(cache.tally_ir_fpr_fail + 1);
                            break;
                        case X87IR::IRFailReason::kGprPressure:
                            cache.tally_ir_gpr_fail =
                                static_cast<uint16_t>(cache.tally_ir_gpr_fail + 1);
                            break;
                        case X87IR::IRFailReason::kNone:
                        case X87IR::IRFailReason::kBridgePartial:
                            // kBridgePartial never reaches this switch (the
                            // bridged attempt handles its own fallback), but
                            // keep -Wswitch exhaustive.
                            break;
                    }
                    mirror_tally();
                    // Speculative-flush rollback: if the gate-cascade flush
                    // emitted ARM and compile_run bailed, the flush ARM is
                    // wasted — peephole / single-op handles the deferred
                    // state itself.  Rewind insn_buf.end and restore the
                    // cleared cache field(s) to pre-flush state.  The
                    // perm_dirty branch always restores; top_dirty /
                    // deferred_pop only when flushed_branch indicates
                    // the matching opt-in enable knob fired.  Diagnostic
                    // counters (bump_max_run, mirror_gate_counters) stay
                    // — they record that an attempt was made.
                    if (!flushed) {
                        // Only the slot conversions for the run to take back.
                        translation_result->insn_buf.end = pre_convert_end;
                        std::copy(std::begin(pre_convert_fmt), std::end(pre_convert_fmt),
                                  std::begin(cache.slot_fmt));
                    }
                    if (flushed) {
                        const uint64_t pre_rewind_end = translation_result->insn_buf.end;
                        const int8_t pre_top_dirty = cache.top_dirty;
                        const int8_t pre_tag_push_pending = cache.tag_push_pending;
                        const int8_t pre_deferred_pop_count = cache.deferred_pop_count;
                        const int8_t pre_perm_dirty = cache.perm_dirty;
                        translation_result->insn_buf.end = saved_buf_end;
                        std::copy(std::begin(saved_slot_fmt), std::end(saved_slot_fmt),
                                  std::begin(cache.slot_fmt));
                        // perm_dirty rollback is unconditional (not gated by
                        // an opt-in enable); restores cache.perm[] + perm_dirty.
                        if (flushed_branch == kRollbackPermDirty) {
                            cache.perm_dirty = saved_perm_dirty;
                            for (int i = 0; i < 8; i++) {
                                cache.perm[i] = saved_perm[i];
                            }
                        }
                        // Branch-specific cache restores for the opt-in
                        // top_dirty / deferred_pop rollback paths.
                        if (flushed_branch == kRollbackTopDirty) {
                            cache.top_dirty = saved_top_dirty;
                        }
                        if (flushed_branch == kRollbackDeferredPop) {
                            cache.deferred_pop_count = saved_deferred_pop_count;
                        }
                        if (g_rosetta_config != nullptr &&
                            g_rosetta_config->x87_log_rollback != 0) {
                            const char* branch_name = "none";
                            switch (flushed_branch) {
                                case kRollbackTopDirty:
                                    branch_name = "top_dirty";
                                    break;
                                case kRollbackDeferredPop:
                                    branch_name = "deferred_pop";
                                    break;
                                case kRollbackPermDirty:
                                    branch_name = "perm_dirty";
                                    break;
                                case kRollbackNone:
                                    break;
                            }
                            const char* fail_name = "None";
                            switch (ir_reason) {
                                case X87IR::IRFailReason::kBuildFail:
                                    fail_name = "BuildFail";
                                    break;
                                case X87IR::IRFailReason::kFprPressure:
                                    fail_name = "FprPressure";
                                    break;
                                case X87IR::IRFailReason::kGprPressure:
                                    fail_name = "GprPressure";
                                    break;
                                case X87IR::IRFailReason::kBridgePartial:
                                    fail_name = "BridgePartial";
                                    break;
                                case X87IR::IRFailReason::kNone:
                                    break;
                            }
                            // block_id = cache.profile_bid when X87_PROFILE is on
                            // (kOverflowId otherwise).  hash is FNV-1a IR-content
                            // hash, populated unconditionally — stable across
                            // launches and the right key for cross-run bisect.
                            // insn_idx + run_remaining pin the rollback to a
                            // position inside the IR stream so an offline tool
                            // can lift the surrounding instructions from the
                            // .prof file.
                            std::printf(
                                "[rollback] branch=%s ir_fail=%s buf_end_delta=%lld "
                                "td %d->%d tp %d->%d dpc %d->%d pd %d->%d "
                                "opcode=0x%04x pc=0x%08x "
                                "block_id=%u hash=0x%016llx insn_idx=%lld "
                                "run_remaining=%d\n",
                                branch_name, fail_name,
                                static_cast<long long>(pre_rewind_end -
                                                       translation_result->insn_buf.end),
                                pre_top_dirty, cache.top_dirty, pre_tag_push_pending,
                                cache.tag_push_pending, pre_deferred_pop_count,
                                cache.deferred_pop_count, pre_perm_dirty, cache.perm_dirty,
                                cur_instr->opcode(), cur_instr->pc,
                                static_cast<unsigned>(cache.profile_bid),
                                static_cast<unsigned long long>(cache.profile_hash),
                                static_cast<long long>(insn_idx),
                                static_cast<int>(cache.run_remaining));
                        }
                    }
                }
                if (ir_consumed > 0) {
                    cache.tally_ir = static_cast<uint16_t>(cache.tally_ir + ir_consumed);
                    mirror_tally();
                    for (int i = 0; i < ir_consumed; i++) {
                        cache.tick();
                    }
                    if (cache.active()) {
                        translation_result->free_gpr_mask =
                            cache.stock_free_gpr_mask & ~cache.pinned_mask();
                    } else {
                        translation_result->free_gpr_mask = cache.stock_free_gpr_mask;
                    }
                    translation_result->free_fpr_mask =
                        translation_result->_unoccupied_temporary_fprs_for_xmm_scalars;
                    translation_result->_pinned_temporary_scalars = 0;
                    return insn_idx + ir_consumed;
                }
            }
        }
    }

    // ── X87_NATIVE_SLOTS: convert what the next emitter reads ───────────────
    // A fusion consumes up to six instructions, also when the run cache is
    // off, and finishes the run's deferred FXCH when it reaches the run's
    // end, so the window covers that much.  With the cache off a reply holds
    // one instruction unless a fusion takes more, so a slot only a later
    // instruction reads is converted here, back at the end of this reply,
    // and again in its own: code that a debugging knob can afford.
    if (cache.native_slots) {
        const int run = cache.active() ? cache.run_remaining
                                       : X87Cache::lookahead(instr_array, num_instrs, insn_idx);
        const int window = std::clamp(run, 1, 6);
        const bool to_run_end = run <= 6;
        emit_slot_conversions(*translation_result,
                              slots_to_convert(cache, instr_array, num_instrs, insn_idx, window,
                                               /*flush_first=*/false, to_run_end));
    }

    // ── Peephole: try 2-instruction fusion patterns ─────────────────────────
    // A native transport reply keeps only the fild m64 + fistp m64 copies,
    // which touch no slot; every other fusion works on compact slots.
    uint64_t fusions_mask = g_rosetta_config ? g_rosetta_config->disabled_fusions_mask : 0;
    if (cache.native_transport) {
        fusions_mask |= ~(1ULL << static_cast<int>(FusionId::fild_fistp));
    }
    const auto fused = TranslatorX87::try_peephole(translation_result, instr_array, num_instrs,
                                                   insn_idx, fusions_mask);

    // ── Single-op fast path: fused emitters for isolated fld/fst/fstp ──────
    // Only when the cache is inactive (run==1, or cache disabled): no pinned
    // GPRs and no deferred TOP/tag/perm state can exist then — x87_end flushes
    // everything at run end.  Counts into tally_single so profile_analyze
    // stays comparable across configs.
    bool single_fast = false;
    if (!fused && !cache.active() && !cache.native_transport &&
        !(g_rosetta_config && g_rosetta_config->disable_x87_single_fast)) {
        single_fast = TranslatorX87::try_translate_single_fast(translation_result, cur_instr);
    }

    // Native transport: the integer loads and stores work on the native
    // slots.  fldcw, fnstcw and the no-ops touch no slot and take their
    // usual emitters below.
    bool native = false;
    if (!fused && cache.native_transport) {
        native = true;
        switch (opcode) {
            case Opcode::kOpcodeName_fild:
                TranslatorX87::translate_fild_native(translation_result, cur_instr);
                break;
            case Opcode::kOpcodeName_fist:
                TranslatorX87::translate_fist_native(translation_result, cur_instr);
                break;
            case Opcode::kOpcodeName_fistp:
                TranslatorX87::translate_fistp_native(translation_result, cur_instr);
                break;
            case Opcode::kOpcodeName_fisttp:
                TranslatorX87::translate_fisttp_native(translation_result, cur_instr);
                break;
            default:
                native = false;
                break;
        }
    }

    if (!fused && !single_fast && !native) {
        switch (opcode) {
            case Opcode::kOpcodeName_fldz:
                TranslatorX87::translate_fldz(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fld1:
                TranslatorX87::translate_fld1(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fldl2e:
                TranslatorX87::translate_fldl2e(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fldl2t:
                TranslatorX87::translate_fldl2t(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fldlg2:
                TranslatorX87::translate_fldlg2(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fldln2:
                TranslatorX87::translate_fldln2(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fldpi:
                TranslatorX87::translate_fldpi(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fld:
                TranslatorX87::translate_fld(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fild:
                TranslatorX87::translate_fild(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fbld:
                TranslatorX87::translate_fbld(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fadd:
                TranslatorX87::translate_fadd(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_faddp:
                TranslatorX87::translate_faddp(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fiadd:
                TranslatorX87::translate_fiadd(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fsub:
            case Opcode::kOpcodeName_fsubr:
                TranslatorX87::translate_fsub(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fsubp:
            case Opcode::kOpcodeName_fsubrp:
                TranslatorX87::translate_fsubp(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fdiv:
            case Opcode::kOpcodeName_fdivr:
                TranslatorX87::translate_fdiv(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fdivp:
            case Opcode::kOpcodeName_fdivrp:
                TranslatorX87::translate_fdivp(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fmul:
            case Opcode::kOpcodeName_fmulp:
                TranslatorX87::translate_fmul(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fst:
            case Opcode::kOpcodeName_fst_stack:
            case Opcode::kOpcodeName_fstp:
            case Opcode::kOpcodeName_fstp_stack:
                TranslatorX87::translate_fst(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fstsw:
                TranslatorX87::translate_fstsw(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fxam:
                TranslatorX87::translate_fxam(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fdecstp:
                TranslatorX87::translate_fdecstp(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fincstp:
                TranslatorX87::translate_fincstp(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_ffree:
                TranslatorX87::translate_ffree(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fxtract:
                TranslatorX87::translate_fxtract(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fscale:
                TranslatorX87::translate_fscale(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fbstp:
                TranslatorX87::translate_fbstp(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_frstor:
                TranslatorX87::translate_frstor(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fsave:
                TranslatorX87::translate_fsave(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fcom:
            case Opcode::kOpcodeName_fcomp:
            case Opcode::kOpcodeName_fcompp:
            case Opcode::kOpcodeName_fucom:
            case Opcode::kOpcodeName_fucomp:
            case Opcode::kOpcodeName_fucompp:
                TranslatorX87::translate_fcom(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fxch:
                TranslatorX87::translate_fxch(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fchs:
                TranslatorX87::translate_fchs(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fabs:
                TranslatorX87::translate_fabs(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fsqrt:
                TranslatorX87::translate_fsqrt(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fistp:
                TranslatorX87::translate_fistp(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fisttp:
                TranslatorX87::translate_fisttp(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fidiv:
                TranslatorX87::translate_fidiv(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fimul:
                TranslatorX87::translate_fimul(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fisub:
                TranslatorX87::translate_fisub(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fidivr:
                TranslatorX87::translate_fidivr(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_frndint:
                TranslatorX87::translate_frndint(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fcomi:
            case Opcode::kOpcodeName_fcomip:
            case Opcode::kOpcodeName_fucomi:
            case Opcode::kOpcodeName_fucomip:
                TranslatorX87::translate_fcomi(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_ftst:
                TranslatorX87::translate_ftst(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fist:
                TranslatorX87::translate_fist(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fisubr:
                TranslatorX87::translate_fisubr(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_ficom:
            case Opcode::kOpcodeName_ficomp:
                TranslatorX87::translate_ficom(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fcmovb:
            case Opcode::kOpcodeName_fcmovbe:
            case Opcode::kOpcodeName_fcmove:
            case Opcode::kOpcodeName_fcmovnb:
            case Opcode::kOpcodeName_fcmovnbe:
            case Opcode::kOpcodeName_fcmovne:
            case Opcode::kOpcodeName_fcmovu:
            case Opcode::kOpcodeName_fcmovnu:
                TranslatorX87::translate_fcmov(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fldcw:
                TranslatorX87::translate_fldcw(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fnstcw:
                TranslatorX87::translate_fnstcw(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fnop:
            case Opcode::kOpcodeName_fdisi:  // 8087 FPU-int disable; NOP on 80287+
            case Opcode::kOpcodeName_feni:   // 8087 FPU-int enable; NOP on 80287+
                TranslatorX87::translate_fnop(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fsin:
                TranslatorX87::translate_fsin(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fcos:
                TranslatorX87::translate_fcos(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_f2xm1:
                TranslatorX87::translate_f2xm1(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fpatan:
                TranslatorX87::translate_fpatan(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fsincos:
                TranslatorX87::translate_fsincos(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fptan:
                TranslatorX87::translate_fptan(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fyl2x:
                TranslatorX87::translate_fyl2x(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fyl2xp1:
                TranslatorX87::translate_fyl2xp1(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fprem:
                TranslatorX87::translate_fprem(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fprem1:
                TranslatorX87::translate_fprem1(translation_result, cur_instr);
                break;

            // Identical body to default: is intentional — the explicit
            // case documents the "deliberate fall-through" set and pairs
            // with the sidecar's kKnownFallThrough allowlist that
            // suppresses the UNHANDLED warning for these opcodes only.
            // Don't merge with default:.
            //
            // Deliberate fall-through to stock:
            //   • fclex/finit/fldenv/fstenv: metadata-only ops with no
            //     stack-aware path; inlining buys nothing measurable
            //     (parity at best, fstenv was a 0.66× regression).
            //     Test coverage in test_*_compose.c confirms no
            //     m108-style internal-offset bug — see
            //     project_native_rosetta_lazy_f80.md.
            //   • fxsave/fxrstor: SSE-era extended save/restore
            //     (512 B incl. 8 ST f80 slots + 16 XMM/YMM + MXCSR).
            //     Inlining would inherit frstor's 0.38× eager-f80
            //     regression; we don't translate SSE.
            //
            // For all six: is_handled_x87 returns false, so the run
            // terminates before this point and the preceding handled
            // op's x87_end has already flushed deferred state to
            // memory — stock reads coherent X87State via x22.
            case Opcode::kOpcodeName_fclex:
            case Opcode::kOpcodeName_finit:
            case Opcode::kOpcodeName_fldenv:
            case Opcode::kOpcodeName_fstenv:
            case Opcode::kOpcodeName_fxsave:
            case Opcode::kOpcodeName_fxrstor:
                cache.tally_ft = static_cast<uint16_t>(cache.tally_ft + 1);
                mirror_tally();
                // The entry prologue pool-restricted the working mask —
                // hand stock back its own word before falling through.
                translation_result->free_gpr_mask = cache.stock_free_gpr_mask;
                return std::nullopt;

            default:
                // Returning nullopt means "I have no handler for this
                // opcode."  In the runtime/sidecar flow, the stub's FILTER
                // prologue routes only x87 opcodes to us, so reaching here
                // with an x87 opcode means an op we *forgot* to handle —
                // log a loud line so we notice.  Add a translate_* and
                // dispatch case, or list it as deliberate fall-through
                // above (and extend kKnownFallThrough on the sidecar side
                // for a new stock-only op like fxsave/fxrstor).
                // In the offline AOT flow (CustomTranslationHook), every
                // non-x87 op also reaches here; that path falls back to
                // stock translate_insn unchanged.  is_x87_opcode gates the
                // log so the AOT path isn't spammed for non-x87 ops.
                //
                // Critical: hand stock back its own free_gpr_mask word
                // (the entry prologue pool-restricted the working copy).
                // Stock's translate_insn (run by the stub's STASH abs-jump
                // in JIT mode, or by original_translate_insn in AOT mode)
                // carries its register-allocation state across the basic
                // block in that field — overwriting it with our scratch
                // values makes stock reuse registers it still holds live
                // (miscompiles), on top of redoing allocation as if every
                // instruction started a fresh block.
                if (is_x87_opcode(opcode)) {
                    const char* name = (opcode < kOpcodeNames.size()) ? kOpcodeNames[opcode] : "?";
                    fprintf(stdout,
                            "[rosettax87] unexpected x87 opcode %s (0x%x) reached "
                            "translator default — add a translate_* and dispatch "
                            "case, or list it as deliberate fall-through above\n",
                            name, static_cast<unsigned>(opcode));
                    fflush(stdout);
                }
                cache.tally_ft = static_cast<uint16_t>(cache.tally_ft + 1);
                mirror_tally();
                translation_result->free_gpr_mask = cache.stock_free_gpr_mask;
                return std::nullopt;
        }
    }

    // OPT-1: Tick the cache (decrements run counter; releases on expiry).
    // Then write stock's captured mask word back, excluding any GPRs still
    // pinned by the cache.  Fused pairs consumed 2 instructions — tick twice.
    const int consumed = fused.value_or(1);
    if (fused.has_value()) {
        cache.tally_peep = static_cast<uint16_t>(cache.tally_peep + consumed);
    } else {
        cache.tally_single = static_cast<uint16_t>(cache.tally_single + 1);
    }
    mirror_tally();
    for (int i = 0; i < consumed; i++) {
        cache.tick();
    }

    if (cache.active()) {
        translation_result->free_gpr_mask = cache.stock_free_gpr_mask & ~cache.pinned_mask();
    } else {
        translation_result->free_gpr_mask = cache.stock_free_gpr_mask;
    }
    translation_result->free_fpr_mask =
        translation_result->_unoccupied_temporary_fprs_for_xmm_scalars;
    translation_result->_pinned_temporary_scalars = 0;

    // Track the most-recent x87 op translated in this block via the
    // peep+single fallback — the IR-gate top_dirty diagnostic reads this
    // to attribute which preceding op left top_dirty=1.  IR success path
    // doesn't update this (it returns earlier at line 225); IR-consumed
    // ops can't be the immediate predecessor of a NEXT-call top_dirty
    // refusal because IR resets all dirty state on success.
    cache.prev_x87_opcode = opcode;

    return insn_idx + consumed;
}
