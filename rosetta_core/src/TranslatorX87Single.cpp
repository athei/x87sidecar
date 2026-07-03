#include <cstdint>

#include "TranslatorX87Internal.hpp"
#include "rosetta_core/AssemblerHelpers.hpp"
#include "rosetta_core/IRInstr.h"
#include "rosetta_core/IROperand.h"
#include "rosetta_core/Opcode.h"
#include "rosetta_core/Register.h"
#include "rosetta_core/TranslationResult.h"
#include "rosetta_core/TranslatorHelpers.hpp"
#include "rosetta_core/TranslatorX87.h"
#include "rosetta_core/TranslatorX87Helpers.hpp"

// =============================================================================
// Single-op fast path — fused emitters for isolated fld / fst / fstp (m32/m64)
//
// A length-1 x87 run is structurally excluded from every amortization path:
// the IR needs a run of >= 3, the peephole needs >= 2 ops, and the register
// cache only arms at run >= 2.  So an isolated fld/fstp — the ABI-bridge shape
// produced at every f32/f64 call boundary (ST0 return / spill) — pays the full
// uncached cost: ~21 ARM for fld m32, of which only ~4 do FP work.  The rest
// is three separate halfword RMW round-trips on ADJACENT X87State fields
// (status_word TOP load, TOP store-back RMW, tag_word RMW) plus uncached
// ST-slot address math.
//
// This path fuses that bookkeeping for the exact single-op shapes:
//
//   * status_word (+0x02) and tag_word (+0x04) are loaded and stored as ONE
//     32-bit unaligned access at [Xbase, #2] — status in bits [15:0], tag in
//     bits [31:16] (little-endian).  6 halfword memory ops become 2 word ops.
//     The x87 state is thread-local, so the wider RMW cannot lose updates.
//   * The ST-slot store folds the +0x08 st[] base into the scaled index:
//     st[i] lives at (i + 1) * 8, so [Xbase, Widx, LSL #3] with Widx = i + 1.
//   * TOP decrement borrows in place: SUB #0x800 may borrow into bits >= 14,
//     but the following 3-bit UBFX discards those, yielding (TOP - 1) & 7.
//     The pop increment must NOT use the same trick (ADD #0x800 carries into
//     C3 at TOP=7); it uses a 3-bit BFI of TOP+1 instead, which is mod-8 free.
//   * Emission order is scheduled for M-series 4-wide issue: both loads
//     (state word + FP operand) issue back-to-back, the dependent state math
//     sinks into the load shadow, and the FP convert runs in parallel on the
//     FP domain.
//
// Result: fld m32 ~15 ARM (m64 ~14), fstp m64 ~13 (m32 ~14), fst ~8.
//
// Correctness guardrails (must match the generic translate_fld/translate_fst
// path bit-for-bit):
//   * Status flags C0..C3/B pass through untouched — BFI only writes [13:11].
//     The generic path never sets C1 on push; replicated, not "improved".
//   * Tag semantics identical: push marks the new slot kValid (bits = 00),
//     pop marks the old slot kEmpty (bits = 11).  Tag maintenance is
//     mandatory (see OPT-2 note in TranslatorX87Helpers.cpp).
//   * Only fires when the cross-instruction cache is inactive (gate in
//     Translator.cpp) — no pinned GPRs or deferred TOP/tag/perm state exists.
// =============================================================================

namespace TranslatorX87 {

namespace {

// tag-word field mask seed within the combined 32-bit status+tag word:
// tag bits live in [31:16], so a slot's 2-bit field is at 16 + 2*slot.
// MOVZ Wm, #3, LSL #16 seeds 0x30000; LSLV by (2*slot) selects the field.
constexpr uint16_t kTagSeed = 3;

// TOP field LSB (bit 11) as a subtractable immediate: one TOP unit.
constexpr uint16_t kTopUnit = 1U << kX87TopShift;  // 0x800

// LSL #1 (32-bit) via UBFM: immr=31, imms=30 — same idiom as emit_x87_push.
void emit_lsl1(AssemblerBuffer& buf, int Wd, int Wn) {
    emit_bitfield(buf, /*is_64bit=*/0, /*opc=*/2 /*UBFM*/, /*N=*/0,
                  /*immr=*/31, /*imms=*/30, Wn, Wd);
}

// BFI Wd[13:11] <- Wn[2:0] — insert a (possibly unmasked) TOP value into the
// status half of the combined word.  The 3-bit width makes the insert mod-8.
void emit_bfi_top(AssemblerBuffer& buf, int Wd, int Wn) {
    emit_bitfield(buf, /*is_64bit=*/0, /*opc=*/1 /*BFM*/,
                  /*N=*/0, /*immr=*/(32 - kX87TopShift) % 32, /*imms=*/2, Wn, Wd);
}

// UBFX Wd <- Wn[13:11] — extract TOP (masked to 3 bits).
void emit_ubfx_top(AssemblerBuffer& buf, int Wd, int Wn) {
    emit_bitfield(buf, /*is_64bit=*/0, /*opc=*/2 /*UBFM*/,
                  /*N=*/0, /*immr=*/kX87TopShift, /*imms=*/kX87TopShift + 2, Wn, Wd);
}

// Accepted operand shape: memory (MemRef/AbsMem) of f32/f64 size.
bool is_mem_f32_f64(const IROperand& op) {
    if (op.kind != IROperandKind::MemRef && op.kind != IROperandKind::AbsMem) {
        return false;
    }
    return op.mem.size == IROperandSize::S32 || op.mem.size == IROperandSize::S64;
}

// ── fld m32/m64 — push + store-to-ST(0), fused state RMW ────────────────────
void translate_fld_single(TranslationResult& a1, IRInstr* a2) {
    AssemblerBuffer& buf = a1.insn_buf;
    const bool is_f32 = (a2->operands[0].mem.size == IROperandSize::S32);

    // Fixed-pool GPRs FIRST (before compute_operand_address — free-pool picks
    // must not collide with the fixed indices; same order as translate_fld).
    const int Xbase = alloc_gpr(a1, 0);
    const int Wsw = alloc_gpr(a1, 1);   // combined status(15:0) + tag(31:16)
    const int Wtop = alloc_gpr(a1, 2);  // new TOP = (TOP - 1) & 7
    const int Wm = alloc_gpr(a1, 3);    // tag field mask
    const int Dd = alloc_free_fpr(a1);

    // Operand address first — guest regs only, independent of Xbase, so the
    // two loads below can issue back-to-back (Bug 4: addr_size from operand).
    const bool addr_is_64 = (a2->operands[0].mem.addr_size == IROperandSize::S64);
    const int Xaddr = compute_operand_address(a1, addr_is_64, &a2->operands[0], GPR::XZR);

    emit_x87_base(buf, a1, Xbase);
    // LDUR Wsw, [Xbase, #2] — one 32-bit load of status_word + tag_word.
    emit_ldur_stur(buf, /*size=*/2, /*opc=*/1 /*LDUR*/, kX87StatusWordOff, Xbase, Wsw);
    // LDR S/D Dd, [Xaddr] — FP operand load pipelines with the LDUR.
    emit_fldr_imm(buf, is_f32 ? 2 : 3, Dd, Xaddr, /*imm12=*/0);
    free_gpr(a1, Xaddr);

    // MOVZ Wm, #3, LSL #16 — no deps, fills a load-shadow slot.
    emit_movn(buf, /*is_64bit=*/0, /*opc=MOVZ*/ 2, /*hw=*/1, kTagSeed, Wm);

    // newTop = (TOP - 1) & 7: the SUB may borrow into bits >= 14; the 3-bit
    // UBFX discards them.  Wsw itself stays unmodified.
    emit_add_imm(buf, /*is_64bit=*/0, /*is_sub=*/1, /*is_set_flags=*/0,
                 /*shift=*/0, kTopUnit, Wsw, Wtop);
    emit_ubfx_top(buf, Wtop, Wtop);

    if (is_f32) {
        emit_fcvt_s_to_d(buf, Dd, Dd);  // FP domain, parallel with GPR math
    }

    const int Widx = alloc_free_gpr(a1);  // newTop + 1 folds the st[] base
    emit_add_imm(buf, /*is_64bit=*/0, /*is_sub=*/0, /*is_set_flags=*/0,
                 /*shift=*/0, 1, Wtop, Widx);
    const int Wsh = alloc_free_gpr(a1);  // tag bit position = 2 * newTop
    emit_lsl1(buf, Wsh, Wtop);

    emit_bfi_top(buf, Wsw, Wtop);   // status half: TOP <- newTop
    emit_lslv(buf, 0, Wsh, Wm, Wm); // mask = 0x30000 << (2 * newTop)

    // STR Dd, [Xbase, Widx, LSL #3] — st[newTop] at (newTop + 1) * 8.
    emit_ldr_str_reg(buf, /*size=*/3, /*is_fp=*/1, /*opc=*/0 /*STR*/, Widx, /*shift=*/1, Xbase,
                     Dd);

    // BIC Wsw, Wsw, Wm — tag half: mark newTop kValid (bits = 00).
    emit_logical_shifted_reg(buf, /*is_64bit=*/0, /*opc=*/0 /*AND*/, /*N=invert*/ 1,
                             /*shift_type=*/0, Wm, /*shift_amt=*/0, Wsw, Wsw);
    // STUR Wsw, [Xbase, #2] — single combined status+tag write-back.
    emit_ldur_stur(buf, /*size=*/2, /*opc=*/0 /*STUR*/, kX87StatusWordOff, Xbase, Wsw);

    free_fpr(a1, Dd);
    free_gpr(a1, Wsh);
    free_gpr(a1, Widx);
    free_gpr(a1, Wm);
    free_gpr(a1, Wtop);
    free_gpr(a1, Wsw);
    free_gpr(a1, Xbase);
}

// ── fst/fstp m32/m64 — load-from-ST(0) + store, pop fused when fstp ─────────
void translate_fst_single(TranslationResult& a1, IRInstr* a2, bool is_fstp) {
    AssemblerBuffer& buf = a1.insn_buf;
    const bool is_f32 = (a2->operands[0].mem.size == IROperandSize::S32);

    const int Xbase = alloc_gpr(a1, 0);
    const int Wtop = alloc_gpr(a1, 2);
    const int Dd = alloc_free_fpr(a1);

    emit_x87_base(buf, a1, Xbase);

    if (!is_fstp) {
        // Non-popping fst: no state change at all — TOP read only.
        emit_load_top(buf, a1, Xbase, Wtop);  // LDRH + UBFX (Opt-6 X18-direct)
        const int Widx = alloc_free_gpr(a1);
        emit_add_imm(buf, /*is_64bit=*/0, /*is_sub=*/0, /*is_set_flags=*/0,
                     /*shift=*/0, 1, Wtop, Widx);
        // LDR Dd, [Xbase, Widx, LSL #3] — st[TOP] at (TOP + 1) * 8.
        emit_ldr_str_reg(buf, /*size=*/3, /*is_fp=*/1, /*opc=*/1 /*LDR*/, Widx, /*shift=*/1,
                         Xbase, Dd);
        free_gpr(a1, Widx);
        // Address convention matches translate_fst (hardcoded 64-bit).
        const int Xaddr =
            compute_operand_address(a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);
        if (is_f32) {
            emit_fcvt_d_to_s(buf, Dd, Dd);
        }
        emit_fstr_imm(buf, is_f32 ? 2 : 3, Dd, Xaddr, /*imm12=*/0);
        free_gpr(a1, Xaddr);

        free_fpr(a1, Dd);
        free_gpr(a1, Wtop);
        free_gpr(a1, Xbase);
        return;
    }

    // fstp: fused pop — one combined status+tag RMW.
    const int Wsw = alloc_gpr(a1, 1);  // combined status(15:0) + tag(31:16)
    const int Wm = alloc_gpr(a1, 3);
    emit_ldur_stur(buf, /*size=*/2, /*opc=*/1 /*LDUR*/, kX87StatusWordOff, Xbase, Wsw);

    // Address math overlaps the LDUR latency (matches translate_fst: 64-bit).
    const int Xaddr = compute_operand_address(a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);

    emit_ubfx_top(buf, Wtop, Wsw);  // oldTop
    emit_movn(buf, /*is_64bit=*/0, /*opc=MOVZ*/ 2, /*hw=*/1, kTagSeed, Wm);

    // Widx = oldTop + 1: DUAL USE — st[] scaled index AND unmasked newTop.
    const int Widx = alloc_free_gpr(a1);
    emit_add_imm(buf, /*is_64bit=*/0, /*is_sub=*/0, /*is_set_flags=*/0,
                 /*shift=*/0, 1, Wtop, Widx);
    const int Wsh = alloc_free_gpr(a1);  // tag bit position = 2 * oldTop
    emit_lsl1(buf, Wsh, Wtop);

    // LDR Dd, [Xbase, Widx, LSL #3] — st[oldTop] at (oldTop + 1) * 8.
    emit_ldr_str_reg(buf, /*size=*/3, /*is_fp=*/1, /*opc=*/1 /*LDR*/, Widx, /*shift=*/1, Xbase,
                     Dd);

    emit_lslv(buf, 0, Wsh, Wm, Wm);  // mask = 0x30000 << (2 * oldTop)
    // ORR Wsw, Wsw, Wm — tag half: mark oldTop kEmpty (bits = 11).
    emit_logical_shifted_reg(buf, /*is_64bit=*/0, /*opc=*/1 /*ORR*/, /*N=*/0,
                             /*shift_type=*/0, Wm, /*shift_amt=*/0, Wsw, Wsw);
    // Status half: TOP <- (oldTop + 1) mod 8 via 3-bit BFI of Widx.  An
    // in-place ADD #0x800 would carry into C3 at TOP=7 — do not use it.
    emit_bfi_top(buf, Wsw, Widx);

    if (is_f32) {
        emit_fcvt_d_to_s(buf, Dd, Dd);
    }
    emit_fstr_imm(buf, is_f32 ? 2 : 3, Dd, Xaddr, /*imm12=*/0);
    free_gpr(a1, Xaddr);

    // STUR Wsw, [Xbase, #2] — single combined status+tag write-back.
    emit_ldur_stur(buf, /*size=*/2, /*opc=*/0 /*STUR*/, kX87StatusWordOff, Xbase, Wsw);

    free_fpr(a1, Dd);
    free_gpr(a1, Wsh);
    free_gpr(a1, Widx);
    free_gpr(a1, Wm);
    free_gpr(a1, Wtop);
    free_gpr(a1, Wsw);
    free_gpr(a1, Xbase);
}

}  // namespace

auto try_translate_single_fast(TranslationResult* a1, IRInstr* a2) -> bool {
    if (!is_mem_f32_f64(a2->operands[0])) {
        return false;  // fld st(i), fst(p)_stack, m80, BCD → generic path
    }

    switch (a2->opcode()) {
        case kOpcodeName_fld:
            translate_fld_single(*a1, a2);
            return true;
        case kOpcodeName_fst:
            translate_fst_single(*a1, a2, /*is_fstp=*/false);
            return true;
        case kOpcodeName_fstp:
            translate_fst_single(*a1, a2, /*is_fstp=*/true);
            return true;
        default:
            return false;
    }
}

}  // namespace TranslatorX87
