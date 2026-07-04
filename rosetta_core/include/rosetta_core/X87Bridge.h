#pragma once

#include "rosetta_core/IRInstr.h"
#include "rosetta_core/IROperand.h"
#include "rosetta_core/Opcode.h"

// ── X87Bridge.h ──────────────────────────────────────────────────────────────
// Bridge-instruction predicates, shared between the offline analyzer
// (profile_analyze's FP-run-fragmentation report) and — if that report
// justifies building it — the runtime bridging lookahead.  Keeping the
// predicate in one place guarantees the measurement and the runtime agree
// on what counts as bridgeable.
//
// A "bridge" is a non-x87 instruction sitting in a short gap between two
// x87 runs inside one IRBlock that the sidecar could translate itself,
// letting a single IR run span the gap instead of spilling/reloading the
// whole x87 stack around it.
//
// v1 scope: instructions that neither read nor write EFLAGS and whose
// lowering is trivial under stock's register convention (guest GPRs live
// 1:1 in ARM x0–x15): 32/64-bit `mov r,r` / `mov r,imm` / `mov r,[m]` /
// `mov [m],r` and `lea`.  Excluded: 8/16-bit operands (sub-register merge
// semantics), segment overrides, rep prefixes, fixup-carrying immediates,
// absolute-memory operands (need external fixups), `mov [m],imm`
// (deferred, easy v1.1).
//
// v2 scope: v1 plus the simple flag-WRITING ALU ops add/sub/inc/dec/and/
// or/xor, bridgeable only when their written flags are provably dead.
// The proof is Rosetta's own per-instruction liveness analysis,
// IRInstr::flag_liveness — decoded empirically (2026-07): on a flag
// writer the byte is the set of written flags that are live-out
// (observed bits: CF=0x20, ZF=0x40, SF=0x80; 0xfe = conservative
// all-live seeded at jcc terminators and block exits; refined masks
// appear for intra-block readers like adc/cmovcc).  flag_liveness == 0
// therefore means "no written flag is ever read" — the same fact stock's
// own lazy-flags codegen relies on.  Bridged ALU lowers to NON-flag-
// setting ARM, so NZCV is never touched; flags passing THROUGH a partial
// writer (a cmp's CF surviving an inc/dec) stay correct for free.
// Pure flag producers (cmp, test) are never bridges — their flags are
// live by construction.  Deferred v2 forms: `alu [m],imm` and
// `inc/dec [m]` (need an imm alongside mem_operand in the 16-byte node).
// ─────────────────────────────────────────────────────────────────────────────

namespace x87bridge {

// Widest gap (consecutive non-x87 instructions) v1 bridging would join.
inline constexpr int kMaxGapV1 = 2;

inline bool gpr_32_or_64(const IROperand& op) {
    if (op.kind != IROperandKind::Register) {
        return false;
    }
    if (op.reg.size != IROperandSize::S32 && op.reg.size != IROperandSize::S64) {
        return false;
    }
    return op.reg.reg.is_gpr();
}

inline bool mem_32_or_64(const IROperand& op) {
    if (op.kind != IROperandKind::MemRef) {
        return false;
    }
    if (op.mem.seg_override != 0) {
        return false;
    }
    return op.mem.size == IROperandSize::S32 || op.mem.size == IROperandSize::S64;
}

inline bool plain_immediate(const IROperand& op) {
    // Rosetta encodes plain mov/ALU immediates with kind == BranchOffset
    // (value in .branch.value) — verified against real captures; the
    // Immediate kind carries fixup targets (x86-abs relocations), which
    // need external_fixups plumbing and are out of v1 scope.
    return op.kind == IROperandKind::BranchOffset;
}

inline bool is_bridge_v1(const IRInstr& ins) {
    if (ins.rep_prefix != 0) {
        return false;
    }
    const uint16_t op = ins.opcode();
    if (op == kOpcodeName_mov) {
        if (ins.num_operands != 2) {
            return false;
        }
        const IROperand& dst = ins.operands[0];
        const IROperand& src = ins.operands[1];
        if (gpr_32_or_64(dst)) {
            return gpr_32_or_64(src) || plain_immediate(src) || mem_32_or_64(src);
        }
        if (mem_32_or_64(dst)) {
            return gpr_32_or_64(src);
        }
        return false;
    }
    if (op == kOpcodeName_lea) {
        if (ins.num_operands != 2) {
            return false;
        }
        // lea never dereferences, but keep the seg-override guard: the
        // runtime address materializer asserts seg_override == 0.
        return gpr_32_or_64(ins.operands[0]) && ins.operands[1].kind == IROperandKind::MemRef &&
               ins.operands[1].mem.seg_override == 0;
    }
    return false;
}

// v2 = v1 ∪ simple flag-writing ALU.  Eligibility here means "bridgeable
// IF its written flags are provably dead" (see flags_dead below); this
// predicate only classifies shape, and covers exactly the shapes the
// runtime can build (X87IRBuild.cpp build_bridge).
inline bool is_bridge_v2(const IRInstr& ins) {
    if (is_bridge_v1(ins)) {
        return true;
    }
    if (ins.rep_prefix != 0) {
        return false;
    }
    const uint16_t op = ins.opcode();
    switch (op) {
        case kOpcodeName_add:
        case kOpcodeName_sub:
        case kOpcodeName_and:
        case kOpcodeName_or:
        case kOpcodeName_xor: {
            if (ins.num_operands != 2) {
                return false;
            }
            const IROperand& dst = ins.operands[0];
            const IROperand& src = ins.operands[1];
            if (gpr_32_or_64(dst)) {
                return gpr_32_or_64(src) || plain_immediate(src) || mem_32_or_64(src);
            }
            if (mem_32_or_64(dst)) {
                return gpr_32_or_64(src);  // [m],imm deferred (node layout)
            }
            return false;
        }
        case kOpcodeName_inc:
        case kOpcodeName_dec: {
            if (ins.num_operands != 1) {
                return false;
            }
            return gpr_32_or_64(ins.operands[0]);  // mem forms deferred
        }
        default:
            return false;
    }
}

// Flag-deadness proof for a v2 (flag-writing) bridge candidate: Rosetta's
// liveness byte says none of this instruction's written flags is live-out.
inline bool flags_dead(const IRInstr& ins) {
    return ins.flag_liveness == 0;
}

// Guard against a Rosetta build that leaves flag_liveness unpopulated
// (all-zero would silently read as "everything dead").  Any real block
// containing a flag writer has at least one nonzero byte: the LAST writer
// before a jcc terminator / block exit is seeded conservative (0xfe), and
// flag readers (adc, cmovcc) carry their read mask.  No nonzero byte
// anywhere in the block ⇒ don't trust the field, no v2 bridging here.
inline bool block_has_flag_liveness(const IRInstr* instr_array, int64_t num_instrs) {
    for (int64_t i = 0; i < num_instrs; i++) {
        if (instr_array[i].flag_liveness != 0) {
            return true;
        }
    }
    return false;
}

// Full v2 eligibility for one instruction (caller must separately check
// block_has_flag_liveness once per block): v1 shapes need no proof; v2-only
// shapes need their written flags dead.
inline bool is_bridge_v2_proven(const IRInstr& ins) {
    if (is_bridge_v1(ins)) {
        return true;
    }
    return is_bridge_v2(ins) && flags_dead(ins);
}

}  // namespace x87bridge
