#include "rosetta_core/X87Cache.h"

#include <cstdint>

#include "rosetta_core/IRInstr.h"
#include "rosetta_core/Opcode.h"
#include "rosetta_core/X87Bridge.h"

// =============================================================================
// is_handled_x87 — returns true for opcodes that have a translate_* handler.
// Used by lookahead to determine consecutive x87 run lengths.
//
// A handful of metadata-only and memory-block x87 opcodes are
// deliberately *excluded* even though they're "x87" in the broad
// sense:
//   • fclex / finit / fldenv / fstenv — metadata-only ops touching
//     CW/SW/TW (16-bit halfwords at offsets 0x00/0x02/0x04 in
//     X87State).  Inlining gave parity at best (fstenv was a 0.66×
//     regression).  Composition tests in tests/test_*_compose.c
//     confirm stock's emit uses the same internal offsets we do —
//     no m108-style internal-offset bug for these ops.
//   • fxsave / fxrstor — SSE-era extended save/restore (512 B incl.
//     8 ST f80 slots + 16 XMM/YMM + MXCSR).  Inlining would inherit
//     frstor's 0.38× eager-f80 regression; we don't translate SSE.
//
// For all six: the run breaks before them, x87_end flushes deferred
// state, the stub abs-jumps to STASH, and stock translates them in
// isolation reading coherent X87State via x22.  There is deliberately no
// general per-opcode fallback: transcendentals would clash on stock's
// {x22, w23} helper-call ABI, which is why this list stays short.
//
// fsave / frstor stay inline (translate_fsave / translate_frstor).
// Rosetta keeps the register file as packed f80 at +0x06, stride 10, in
// every one of its own paths.  The sidecar's binary64 slots at +0x08,
// stride 8, exist only inside one reply: Translator converts at the reply's
// entry and exit (emit_native_state_boundary), so stock never sees them.
// With X87_NATIVE_SLOTS the binary64 values sit inside the native slots
// instead, and only the slots a reply reads are converted (slot_effect).
// =============================================================================

static bool is_handled_x87(uint16_t op) {
    switch (op) {
        case kOpcodeName_fldz:
        case kOpcodeName_fld1:
        case kOpcodeName_fldl2e:
        case kOpcodeName_fldl2t:
        case kOpcodeName_fldlg2:
        case kOpcodeName_fldln2:
        case kOpcodeName_fldpi:
        case kOpcodeName_fld:
        case kOpcodeName_fild:
        case kOpcodeName_fadd:
        case kOpcodeName_faddp:
        case kOpcodeName_fiadd:
        case kOpcodeName_fsub:
        case kOpcodeName_fsubr:
        case kOpcodeName_fsubp:
        case kOpcodeName_fsubrp:
        case kOpcodeName_fdiv:
        case kOpcodeName_fdivr:
        case kOpcodeName_fdivp:
        case kOpcodeName_fdivrp:
        case kOpcodeName_fmul:
        case kOpcodeName_fmulp:
        case kOpcodeName_fst:
        case kOpcodeName_fst_stack:
        case kOpcodeName_fstp:
        case kOpcodeName_fstp_stack:
        case kOpcodeName_fstsw:
        case kOpcodeName_fcom:
        case kOpcodeName_fcomp:
        case kOpcodeName_fcompp:
        case kOpcodeName_fucom:
        case kOpcodeName_fucomp:
        case kOpcodeName_fucompp:
        case kOpcodeName_fxch:
        case kOpcodeName_fchs:
        case kOpcodeName_fabs:
        case kOpcodeName_fsqrt:
        case kOpcodeName_fistp:
        case kOpcodeName_fisttp:
        case kOpcodeName_fidiv:
        case kOpcodeName_fimul:
        case kOpcodeName_fisub:
        case kOpcodeName_fidivr:
        case kOpcodeName_frndint:
        case kOpcodeName_fcomi:
        case kOpcodeName_fcomip:
        case kOpcodeName_fucomi:
        case kOpcodeName_fucomip:
        case kOpcodeName_ftst:
        case kOpcodeName_fist:
        case kOpcodeName_fisubr:
        case kOpcodeName_fcmovb:
        case kOpcodeName_fcmovbe:
        case kOpcodeName_fcmove:
        case kOpcodeName_fcmovnb:
        case kOpcodeName_fcmovnbe:
        case kOpcodeName_fcmovne:
        case kOpcodeName_fcmovu:
        case kOpcodeName_fcmovnu:
        case kOpcodeName_ficom:
        case kOpcodeName_ficomp:
        case kOpcodeName_fldcw:
        case kOpcodeName_fnstcw:
        case kOpcodeName_fnop:
        case kOpcodeName_fdisi:
        case kOpcodeName_feni:
        case kOpcodeName_fxam:
        case kOpcodeName_fbld:
        case kOpcodeName_fdecstp:
        case kOpcodeName_fincstp:
        case kOpcodeName_ffree:
        case kOpcodeName_fxtract:
        case kOpcodeName_fscale:
        case kOpcodeName_fbstp:
        case kOpcodeName_frstor:
        case kOpcodeName_fsave:
        case kOpcodeName_fsin:
        case kOpcodeName_fcos:
        case kOpcodeName_f2xm1:
        case kOpcodeName_fpatan:
        case kOpcodeName_fsincos:
        case kOpcodeName_fptan:
        case kOpcodeName_fyl2x:
        case kOpcodeName_fyl2xp1:
        case kOpcodeName_fprem:
        case kOpcodeName_fprem1:
            return true;
        default:
            return false;
    }
}

// The stack register an operand names, or -1 for a memory or other operand.
// fstsw AX is the only handled x87 instruction with a register operand that
// is not a stack register, and slot_effect does not look at its operands.
static int st_operand(const IRInstr& insn, int k) {
    if (k >= insn.num_operands || insn.operands[k].kind != IROperandKind::Register) {
        return -1;
    }
    return insn.operands[k].reg.reg.index() & 7;
}

// Every stack register named by an operand, as a depth mask.  Rosetta puts
// ST(0) in the operands of the two-register and the memory forms of the
// arithmetic, compare, fcmov and fxch instructions, so for those this is the
// set of registers read.
static uint8_t st_operand_mask(const IRInstr& insn) {
    uint8_t mask = 0;
    for (int k = 0; k < insn.num_operands && k < 4; k++) {
        const int d = st_operand(insn, k);
        if (d >= 0) {
            mask |= static_cast<uint8_t>(1U << d);
        }
    }
    return mask;
}

// The table behind X87_NATIVE_SLOTS: which stack registers each handled
// instruction reads as binary64, which it writes without reading, and how it
// moves TOP.  It has to cover every read the emitters make: a read the table
// misses finds a slot still in the 80-bit format, which emit_load_st reports
// as a translation failure (the request is declined, or the X87_STRICT_ALLOC
// harness asserts).
X87Cache::SlotEffect X87Cache::slot_effect(const IRInstr& insn) {
    SlotEffect e;
    switch (insn.opcode()) {
        case kOpcodeName_fldz:
        case kOpcodeName_fld1:
        case kOpcodeName_fldl2e:
        case kOpcodeName_fldl2t:
        case kOpcodeName_fldlg2:
        case kOpcodeName_fldln2:
        case kOpcodeName_fldpi:
        case kOpcodeName_fild:
        case kOpcodeName_fbld:
            e.pushes = 1;
            break;
        case kOpcodeName_fld: {
            // fld st(i) is [ST(0) destination, ST(i) source].
            const int src = st_operand(insn, 1);
            if (st_operand(insn, 0) >= 0 && src >= 0) {
                e.reads = static_cast<uint8_t>(1U << src);
                e.soft = e.reads;  // an 80-bit ST(i) is copied as ten bytes
            }
            e.pushes = 1;
            break;
        }
        case kOpcodeName_fst:
        case kOpcodeName_fstp:
        case kOpcodeName_fst_stack:
        case kOpcodeName_fstp_stack: {
            // The register forms copy ST(0) into operands[0].  fst st(0)
            // copies nothing, but the IR and the fusions still load ST(0)
            // for it.  fstp m80 copies an 80-bit ST(0) as it is and converts
            // a binary64 one, so it needs no conversion.
            const int dst = st_operand(insn, 0);
            const bool m80 = dst < 0 && insn.num_operands > 0 &&
                             insn.operands[0].mem.size == IROperandSize::S80;
            if (!m80) {
                // An 80-bit ST(0) is copied to a register as ten bytes, and
                // converted into an FPR for m32/m64.
                e.reads = 1;
                e.soft = 1;
            }
            if (dst > 0) {
                e.writes = static_cast<uint8_t>(1U << dst);
            }
            if (insn.opcode() == kOpcodeName_fstp || insn.opcode() == kOpcodeName_fstp_stack) {
                e.pops = 1;
            }
            break;
        }
        case kOpcodeName_fadd:
        case kOpcodeName_fsub:
        case kOpcodeName_fsubr:
        case kOpcodeName_fdiv:
        case kOpcodeName_fdivr:
        case kOpcodeName_fmul:
        case kOpcodeName_fcom:
        case kOpcodeName_fucom:
        case kOpcodeName_fcomi:
        case kOpcodeName_fucomi:
        case kOpcodeName_fcmovb:
        case kOpcodeName_fcmovbe:
        case kOpcodeName_fcmove:
        case kOpcodeName_fcmovnb:
        case kOpcodeName_fcmovnbe:
        case kOpcodeName_fcmovne:
        case kOpcodeName_fcmovu:
        case kOpcodeName_fcmovnu:
            e.reads = static_cast<uint8_t>(st_operand_mask(insn) | 1U);
            break;
        case kOpcodeName_faddp:
        case kOpcodeName_fsubp:
        case kOpcodeName_fsubrp:
        case kOpcodeName_fdivp:
        case kOpcodeName_fdivrp:
        case kOpcodeName_fmulp:
        case kOpcodeName_fcomp:
        case kOpcodeName_fucomp:
        case kOpcodeName_fcomip:
        case kOpcodeName_fucomip:
            e.reads = static_cast<uint8_t>(st_operand_mask(insn) | 1U);
            e.pops = 1;
            break;
        case kOpcodeName_fcompp:
        case kOpcodeName_fucompp:
            e.reads = 0x3;
            e.pops = 2;
            break;
        case kOpcodeName_fist:
            // The integer stores convert an 80-bit ST(0) exactly
            // (emit_f80_slot_to_int) when the GPRs allow.
            e.reads = 1;
            e.soft = 1;
            break;
        case kOpcodeName_fistp:
        case kOpcodeName_fisttp:
            e.reads = 1;
            e.soft = 1;
            e.pops = 1;
            break;
        case kOpcodeName_fiadd:
        case kOpcodeName_fisub:
        case kOpcodeName_fisubr:
        case kOpcodeName_fimul:
        case kOpcodeName_fidiv:
        case kOpcodeName_fidivr:
        case kOpcodeName_ficom:
        case kOpcodeName_ftst:
        case kOpcodeName_fxam:
        case kOpcodeName_fchs:
        case kOpcodeName_fabs:
        case kOpcodeName_fsqrt:
        case kOpcodeName_frndint:
        case kOpcodeName_fsin:
        case kOpcodeName_fcos:
        case kOpcodeName_f2xm1:
            e.reads = 1;
            break;
        case kOpcodeName_ficomp:
        case kOpcodeName_fbstp:
            e.reads = 1;
            e.pops = 1;
            break;
        case kOpcodeName_fsincos:
        case kOpcodeName_fptan:
        case kOpcodeName_fxtract:
            e.reads = 1;
            e.pushes = 1;
            break;
        case kOpcodeName_fscale:
        case kOpcodeName_fprem:
        case kOpcodeName_fprem1:
            e.reads = 0x3;
            break;
        case kOpcodeName_fpatan:
        case kOpcodeName_fyl2x:
        case kOpcodeName_fyl2xp1:
            e.reads = 0x3;
            e.pops = 1;
            break;
        case kOpcodeName_fxch: {
            const int d = st_operand(insn, 1);
            if (d > 0) {
                // A deferred FXCH reads nothing; a swap in memory moves
                // ten-byte images when an 80-bit slot takes part.
                e.reads = static_cast<uint8_t>(1U | (1U << d));
                e.soft = e.reads;
                e.kind = kEffectFxch;
                e.arg = static_cast<int8_t>(d);
            }
            break;
        }
        case kOpcodeName_ffree:
            e.kind = kEffectFree;
            e.arg = static_cast<int8_t>(st_operand(insn, 0) < 0 ? 0 : st_operand(insn, 0));
            break;
        case kOpcodeName_fincstp:
            e.kind = kEffectTopInc;
            break;
        case kOpcodeName_fdecstp:
            e.kind = kEffectTopDec;
            break;
        case kOpcodeName_fsave:
            // 80-bit slots go into the save area as their ten bytes.
            e.reads = 0xFF;
            e.soft = 0xFF;
            e.kind = kEffectSave;
            break;
        case kOpcodeName_frstor:
            e.kind = kEffectRestore;
            break;
        default:
            // fstsw, fldcw, fnstcw, fnop, fdisi, feni, and any non-x87
            // instruction.
            break;
    }
    return e;
}

// The ops a native transport reply may contain.  None of them does
// arithmetic on a register value: fild writes an integer into a slot, which
// a 64-bit significand holds exactly, the integer stores read one back, and
// the rest touch only the control word or nothing.  So the reply can work on
// the native 80-bit slots directly instead of converting the register file
// to binary64 and back, which rounds every int64 of 2^53 or more held on the
// stack across a reply (Delphi's System.Move keeps fild values on the stack
// across its branches).
static bool is_native_transport_x87(uint16_t op) {
    switch (op) {
        case kOpcodeName_fild:
        case kOpcodeName_fist:
        case kOpcodeName_fistp:
        case kOpcodeName_fisttp:
        case kOpcodeName_fldcw:
        case kOpcodeName_fnstcw:
        case kOpcodeName_fnop:
        case kOpcodeName_fdisi:
        case kOpcodeName_feni:
            return true;
        default:
            return false;
    }
}

// =============================================================================
// X87Cache member functions
// =============================================================================

bool X87Cache::active() const {
    return run_remaining > 0;
}

void X87Cache::invalidate() {
    gprs_valid = 0;
    st_base_valid = 0;
    top_dirty = 0;
    tag_push_pending = 0;
    deferred_pop_count = 0;
    run_remaining = 0;
    reset_perm();
}

void X87Cache::set_run(int run_length) {
    if (run_length >= 2) {
        run_remaining = static_cast<int16_t>(run_length);
    }
}

void X87Cache::tick() {
    if (run_remaining > 0) {
        run_remaining--;
        if (run_remaining == 0) {
            gprs_valid = 0;
            st_base_valid = 0;
            top_dirty = 0;
            tag_push_pending = 0;
            deferred_pop_count = 0;
        }
    }
}

void X87Cache::reset_perm() {
    for (int i = 0; i < 8; i++) {
        perm[i] = static_cast<int8_t>(i);
    }
    perm_dirty = 0;
}

bool X87Cache::perm_is_identity() const {
    for (int i = 0; i < 8; i++) {
        if (perm[i] != i) {
            return false;
        }
    }
    return true;
}

uint32_t X87Cache::pinned_mask() const {
    uint32_t mask = 0;
    if (gprs_valid) {
        mask |= (1U << base_gpr);
        mask |= (1U << top_gpr);
        if (st_base_valid) {
            mask |= (1U << st_base_gpr);
        }
    }
    return mask;
}

int X87Cache::lookahead(IRInstr* instr_array, int64_t num_instrs, int64_t insn_idx) {
    int count = 0;
    for (int64_t i = insn_idx; i < num_instrs; i++) {
        if (!is_handled_x87(instr_array[i].opcode())) {
            break;
        }
        count++;
    }
    return count;
}

bool X87Cache::is_native_transport_run(IRInstr* instr_array, int64_t num_instrs, int64_t insn_idx) {
    int64_t i = insn_idx;
    for (; i < num_instrs && is_handled_x87(instr_array[i].opcode()); i++) {
        if (!is_native_transport_x87(instr_array[i].opcode())) {
            return false;
        }
    }
    return i > insn_idx;
}

X87Cache::BridgedRun X87Cache::lookahead_bridged(IRInstr* instr_array, int64_t num_instrs,
                                                 int64_t insn_idx, int max_gap,
                                                 int max_total_bridges, bool allow_v2) {
    // v2 (flag-dead ALU) gaps are only trusted when the block demonstrably
    // carries Rosetta's flag-liveness analysis — an unpopulated (all-zero)
    // field would otherwise read as "every flag dead".  One scan of the
    // whole block decides this for all gap instructions below.
    const bool v2 = allow_v2 && x87bridge::block_has_flag_liveness(instr_array, num_instrs);

    // Walk forward from insn_idx (an x87 instruction by contract): x87 ops
    // extend the region; a bridge instruction extends it tentatively as
    // long as the consecutive gap stays <= max_gap and the region's bridge
    // budget holds.  `last_x87_end` tracks the trailing-bridge trim point:
    // the region is only ever committed up to the last x87 instruction.
    int total = 0;        // committed region length (ends on x87)
    int x87_count = 0;    // x87 ops within the committed region
    int bridges = 0;      // bridges within the committed region
    int pending_gap = 0;  // bridge instructions since the last x87 op
    int joins = 0;        // gaps that got x87 on both sides

    for (int64_t i = insn_idx; i < num_instrs; i++) {
        IRInstr& ins = instr_array[i];
        if (is_handled_x87(ins.opcode())) {
            if (pending_gap > 0) {
                joins++;
                bridges += pending_gap;
                pending_gap = 0;
            }
            x87_count++;
            total = static_cast<int>(i - insn_idx) + 1;
            continue;
        }
        if ((v2 ? x87bridge::is_bridge_v2_proven(ins) : x87bridge::is_bridge_v1(ins)) &&
            pending_gap < max_gap && bridges + pending_gap < max_total_bridges) {
            pending_gap++;  // tentative — committed only if x87 follows
            continue;
        }
        break;  // ineligible instruction (or budget exhausted) ends the scan
    }

    // Worth bridging only if at least one gap was joined and the merged run
    // clears the IR gate.  (pending_gap > 0 leftovers are trailing bridges,
    // already excluded from `total` by construction.)
    if (joins == 0 || x87_count < 3) {
        return BridgedRun{};
    }
    return BridgedRun{
        .total = static_cast<int16_t>(total),
        .x87_count = static_cast<int16_t>(x87_count),
        .bridges = static_cast<int16_t>(bridges),
    };
}
