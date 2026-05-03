#include "rosetta_core/X87Cache.h"

#include <cstdint>

#include "rosetta_core/IRInstr.h"
#include "rosetta_core/Opcode.h"

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
// isolation reading coherent X87State via x22.  See
// feedback_no_per_opcode_fallback.md (transcendentals would clash on
// stock's {x22, w23} helper-call ABI — that's why this list stays
// short).
//
// fsave / frstor stay INLINE — see translate_fsave / translate_frstor
// for the full story.  TL;DR: stock's m108 path uses an incompatible
// stride-10/0x06 raw-f80 ST layout that doesn't interoperate with
// our (and stock's modern m512-path) stride-8/0x08 f64 layout.
// Composing them was empirically validated to corrupt all 8 ST slots.
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

void X87Cache::invalidate(uint32_t& free_gpr_mask, uint32_t scratch_mask) {
    invalidate();
    free_gpr_mask = scratch_mask;
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
        if (!is_handled_x87(instr_array[i].opcode)) {
            break;
        }
        count++;
    }
    return count;
}
