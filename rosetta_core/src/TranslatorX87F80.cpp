#include "rosetta_core/TranslatorX87F80.hpp"

#include <cstdint>

#include "rosetta_core/AssemblerBuffer.h"
#include "rosetta_core/AssemblerHelpers.hpp"

namespace TranslatorX87 {

// ── shared CSEL emitter ─────────────────────────────────────────────────────
// AssemblerHelpers exposes emit_fcsel_f64 but no GPR CSEL helper.  The original
// inline sites in translate_fld / translate_fst encoded CSEL via a local
// lambda; reproduce it here so we don't pollute the public header just for
// these helpers.
static inline void emit_csel(AssemblerBuffer& buf, int is_64bit, int Rd, int Rn, int Rm, int cond) {
    uint32_t insn = 0x1A800000U;
    insn |= static_cast<uint32_t>(is_64bit != 0) << 31;
    insn |= static_cast<uint32_t>(Rm & 0x1F) << 16;
    insn |= static_cast<uint32_t>(cond & 0xF) << 12;
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Rd & 0x1F);
    buf.emit(insn);
}

// =============================================================================
// f80 -> f64 (read).
//
// f80 layout (10 bytes):
//   bytes 0-7  64-bit mantissa (bit 63 = explicit integer bit)
//   bytes 8-9  16-bit word: bit 15 = sign, bits 14:0 = 15-bit exp (bias 16383)
//
// Conversion (round-half-up via pre-add 0x400, with carry-out compensation
// for the all-ones-mantissa wrap case):
//   sign     = exp_word[15]
//   exp_low  = exp_word[14:0]
//   mant_lo  = (mantissa >> 11) & 0x000F_FFFF_FFFF_FFFF
//   if exp_low == 0:        f64 = sign << 63           (zero/denormal -> ±0)
//   if exp_low == 0x7FFF:   f64 = (sign<<63)|(0x7FF<<52)|mant_lo  (Inf/NaN)
//   else:                   f64 = (sign<<63)|((exp_low-15360)<<52)|mant_lo
//
// 15360 = 16383 (f80 bias) - 1023 (f64 bias).  Doesn't fit in imm12/imm12<<12,
// so we do SUB #16384 + ADD #1024 to subtract it without burning a register.
// =============================================================================
void emit_f80_to_f64_convert(AssemblerBuffer& buf, int Xmant_inout, int Wexp, int Xsign, int Wd_aux,
                             int Wd_tmp) {
    // Wd_aux holds the rounding-carry first, then is reused for the
    // 0x7FFF/0x7FF constants — the carry is consumed before the override
    // constants are needed.

    // Sign bit -> Xsign[0]; clear sign in Wexp so it holds exp_low.
    emit_bitfield(buf, /*is_64bit=*/1, /*opc=*/2 /*UBFM*/, /*N=*/1,
                  /*immr=*/15, /*imms=*/15, Wexp, Xsign);
    LogicalImmEncoding enc_15bits;
    is_bitmask_immediate(/*is_64bit=*/false, 0x7FFFU, enc_15bits);
    emit_and_imm(buf, /*is_64bit=*/0, Wexp, enc_15bits.N, enc_15bits.immr, enc_15bits.imms, Wexp);

    // Pre-round: add 0x400 (= half of the 11-bit slack we'll truncate)
    // before LSR 11 to implement round-half-up.  Without this, sqrt(2)
    // and similar values fail by 1 ULP because the sticky bits get
    // dropped by pure truncation.
    //
    // ADDS captures the carry-out: when the mantissa is close enough
    // to all-ones, the addition wraps past bit 63 and the integer bit
    // is lost.  Compensate by incrementing the f64 exponent (e.g.,
    // 1.999... -> 2.0).
    emit_add_imm(buf, /*is_64bit=*/1, /*is_sub=*/0, /*is_set_flags=*/1,
                 /*shift=*/0, /*imm12=*/0x400, Xmant_inout, Xmant_inout);
    emit_cset(buf, /*is_64bit=*/0, /*cond=*/2 /*CS*/, Wd_aux);

    // Mantissa: drop integer bit + low 11 fractional bits -> 52-bit value.
    emit_bitfield(buf, /*is_64bit=*/1, /*opc=*/2 /*UBFM*/, /*N=*/1,
                  /*immr=*/11, /*imms=*/63, Xmant_inout, Xmant_inout);
    LogicalImmEncoding enc_mant52;
    is_bitmask_immediate(/*is_64bit=*/true, 0x000FFFFFFFFFFFFFULL, enc_mant52);
    emit_and_imm(buf, /*is_64bit=*/1, Xmant_inout, enc_mant52.N, enc_mant52.immr, enc_mant52.imms,
                 Xmant_inout);

    // If exp_low == 0 (zero/denormal): zero out mantissa.
    // CMP Wexp, #0; CSEL Xmant_inout, XZR, Xmant_inout, EQ.
    emit_add_imm(buf, /*is_64bit=*/0, /*is_sub=*/1, /*is_set_flags=*/1,
                 /*shift=*/0, /*imm12=*/0, Wexp, /*Rd=*/31);
    emit_csel(buf, /*is_64bit=*/1, Xmant_inout, /*Rn=*/31, /*Rm=*/Xmant_inout, /*cond=*/0 /*EQ*/);

    // Compute exp_adj normal case in Wd_tmp:
    //   SUB Wd_tmp, Wexp, #0x4000, LSL #12   (-16384)
    //   ADD Wd_tmp, Wd_tmp, #0x400           (+1024 -> -15360)
    emit_add_imm(buf, /*is_64bit=*/0, /*is_sub=*/1, /*is_set_flags=*/0,
                 /*shift=*/1, /*imm12=*/4, Wexp, Wd_tmp);
    emit_add_imm(buf, /*is_64bit=*/0, /*is_sub=*/0, /*is_set_flags=*/0,
                 /*shift=*/0, /*imm12=*/0x400, Wd_tmp, Wd_tmp);

    // Apply round-overflow carry from the +0x400 ADDS at the top:
    // exp_adj += Wd_aux.
    emit_add_sub_shifted_reg(buf, /*is_64bit=*/0, /*is_sub=*/0, /*is_set_flags=*/0,
                             /*shift_type=*/0, /*Rm=*/Wd_aux, /*shift_amount=*/0,
                             /*Rn=*/Wd_tmp, /*Rd=*/Wd_tmp);

    // exp_adj override for exp == 0: CMP Wexp, #0; CSEL exp_adj, WZR, exp_adj, EQ
    emit_add_imm(buf, /*is_64bit=*/0, /*is_sub=*/1, /*is_set_flags=*/1,
                 /*shift=*/0, /*imm12=*/0, Wexp, /*Rd=*/31);
    emit_csel(buf, /*is_64bit=*/0, Wd_tmp, /*Rn=*/31, /*Rm=*/Wd_tmp, /*cond=*/0 /*EQ*/);

    // exp_adj override for exp == 0x7FFF: load 0x7FFF into Wd_aux for
    // the CMP, then 0x7FF for the override value.
    emit_movn(buf, /*is_64bit=*/0, /*opc=*/2 /*MOVZ*/, /*hw=*/0, /*imm16=*/0x7FFF, Wd_aux);
    emit_subs_reg(buf, /*is_64bit=*/0, /*Rn=*/Wexp, /*Rm=*/Wd_aux, /*Rd=*/31);
    emit_movn(buf, /*is_64bit=*/0, /*opc=*/2 /*MOVZ*/, /*hw=*/0, /*imm16=*/0x7FF, Wd_aux);
    emit_csel(buf, /*is_64bit=*/0, Wd_tmp, /*Rn=*/Wd_aux, /*Rm=*/Wd_tmp, /*cond=*/0 /*EQ*/);

    // Build f64 raw bits in Xmant_inout via two BFIs.
    //   BFI Xmant_inout, Xd_tmp, #52, #11   -> bits [62:52] = exp_adj[10:0]
    //   BFI Xmant_inout, Xsign,  #63, #1    -> bit 63 = sign[0]
    // BFI is BFM with opc=01.  For lsb,width: immr=(64-lsb)%64, imms=width-1.
    emit_bitfield(buf, /*is_64bit=*/1, /*opc=*/1 /*BFM*/, /*N=*/1,
                  /*immr=*/12, /*imms=*/10, Wd_tmp, Xmant_inout);
    emit_bitfield(buf, /*is_64bit=*/1, /*opc=*/1 /*BFM*/, /*N=*/1,
                  /*immr=*/1, /*imms=*/0, Xsign, Xmant_inout);
}

// =============================================================================
// f64 -> f80 (write).
//
// IEEE 754 double:  [63] sign | [62:52] exp (11-bit, bias 1023) | [51:0] mant
// x87 f80:          bytes 0-7 mantissa (64-bit, explicit integer bit at 63)
//                   bytes 8-9 [15] sign | [14:0] exp (15-bit, bias 16383)
//
// 22 instructions, 3 paths:
//   [0-4]   extract sign/exp/mantissa from double bits
//   [5-7]   dispatch: CBZ -> zero/denorm, B.EQ -> inf/nan
//   [8-14]  normal: set integer bit, bias-adjust exp, store, B .done
//   [15-17] zero/denorm: store zero mantissa + sign, B .done
//   [18-21] inf/nan: set integer bit, store, sign|0x7FFF, store
//
// All branch displacements are PC-relative (instruction units), so this
// helper is safe to invoke any number of times back-to-back in one block.
// =============================================================================
void emit_f64_to_f80(AssemblerBuffer& buf, int Xaddr_slot, int Dd_src, int Xbits, int Wexp,
                     int Wd_tmp) {
    // [0] FMOV Xbits, Dd_src — raw double bits to GPR
    emit_fmov_d_to_x(buf, Xbits, Dd_src);

    // [1] UBFX Xexp, Xbits, #52, #11 — extract 11-bit exponent
    emit_bitfield(buf, /*is_64bit=*/1, /*opc=*/2, /*N=*/1, /*immr=*/52, /*imms=*/62, Xbits, Wexp);

    // [2] LSR Wd_tmp, Xbits, #48 — shift sign from bit 63 to bit 15
    emit_bitfield(buf, 1, 2, 1, 48, 63, Xbits, Wd_tmp);

    // [3] AND Wd_tmp, Wd_tmp, #0x8000 — isolate sign at bit 15
    LogicalImmEncoding enc_sign;
    is_bitmask_immediate(/*is_64bit=*/false, 0x8000, enc_sign);
    emit_and_imm(buf, 0, Wd_tmp, enc_sign.N, enc_sign.immr, enc_sign.imms, Wd_tmp);

    // [4] LSL Xbits, Xbits, #11 — position 52-bit mantissa for f80 (bits [63:12])
    //     UBFM Xd, Xn, #53, #52 is the alias for LSL Xd, Xn, #11
    emit_bitfield(buf, 1, 2, 1, 53, 52, Xbits, Xbits);

    // [5] CBZ Wexp, .zero_denorm (+10 insns = +40 bytes)
    emit_cbz(buf, /*is_64bit=*/0, /*is_nz=*/0, Wexp, 10);

    // [6] CMP Wexp, #0x7FF  (SUBS WZR, Wexp, #2047)
    emit_add_imm(buf, /*is_64bit=*/0, /*is_sub=*/1, /*is_set_flags=*/1,
                 /*shift=*/0, /*imm12=*/0x7FF, Wexp, /*Rd=*/31);

    // [7] B.EQ .inf_nan (+11 insns = +44 bytes)
    emit_b_cond(buf, /*cond=*/0 /*EQ*/, 11);

    // ── Normal number ──
    // [8] ORR Xbits, Xbits, #0x8000000000000000 — set explicit integer bit
    LogicalImmEncoding enc_intbit;
    is_bitmask_immediate(/*is_64bit=*/true, 0x8000000000000000ULL, enc_intbit);
    emit_orr_imm(buf, 1, Xbits, Xbits, enc_intbit.N, enc_intbit.immr, enc_intbit.imms);

    // [9]  ADD Wexp, Wexp, #3, LSL#12  (+12288)
    emit_add_imm(buf, 0, 0, 0, /*shift=*/1, /*imm12=*/3, Wexp, Wexp);
    // [10] ADD Wexp, Wexp, #0xC00      (+3072; total +15360 = 16383−1023)
    emit_add_imm(buf, 0, 0, 0, 0, 0xC00, Wexp, Wexp);

    // [11] ORR Wexp, Wexp, Wd_tmp — combine biased exponent + sign
    emit_logical_shifted_reg(buf, 0, /*opc=*/1, /*n=*/0, /*shift_type=*/0, Wd_tmp,
                             /*shift_amount=*/0, Wexp, Wexp);

    // [12] STR Xbits, [Xaddr_slot] — 8-byte mantissa
    emit_str_imm(buf, /*size=*/3, Xbits, Xaddr_slot, /*imm12=*/0);
    // [13] STRH Wexp, [Xaddr_slot, #8] — 2-byte exponent (imm12=4, scaled by 2)
    emit_str_imm(buf, /*size=*/1, Wexp, Xaddr_slot, /*imm12=*/4);

    // [14] B .done (+8 insns = +32 bytes)
    emit_b(buf, 8);

    // ── Zero / denormal (treated as ±0.0) ──
    // [15] STR XZR, [Xaddr_slot] — mantissa = 0
    emit_str_imm(buf, 3, /*Rt=*/31, Xaddr_slot, 0);
    // [16] STRH Wd_tmp, [Xaddr_slot, #8] — exponent = sign only
    emit_str_imm(buf, 1, Wd_tmp, Xaddr_slot, 4);
    // [17] B .done (+5 insns = +20 bytes)
    emit_b(buf, 5);

    // ── Infinity / NaN ──
    // [18] ORR Xbits, Xbits, #0x8000000000000000 — set explicit integer bit
    emit_orr_imm(buf, 1, Xbits, Xbits, enc_intbit.N, enc_intbit.immr, enc_intbit.imms);
    // [19] STR Xbits, [Xaddr_slot] — mantissa
    emit_str_imm(buf, 3, Xbits, Xaddr_slot, 0);
    // [20] ORR Wexp, Wd_tmp, #0x7FFF — sign | max exponent
    LogicalImmEncoding enc_7fff;
    is_bitmask_immediate(/*is_64bit=*/false, 0x7FFF, enc_7fff);
    emit_orr_imm(buf, 0, Wexp, Wd_tmp, enc_7fff.N, enc_7fff.immr, enc_7fff.imms);
    // [21] STRH Wexp, [Xaddr_slot, #8] — exponent
    emit_str_imm(buf, 1, Wexp, Xaddr_slot, 4);
}

};  // namespace TranslatorX87
