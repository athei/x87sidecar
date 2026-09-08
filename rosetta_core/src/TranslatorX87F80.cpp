#include "rosetta_core/TranslatorX87F80.hpp"

#include <cstdint>

#include "rosetta_core/AssemblerBuffer.h"
#include "rosetta_core/AssemblerHelpers.hpp"
#include "rosetta_core/TranslatorX87Helpers.hpp"

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
// Narrow to binary64 using round-to-nearest-even. Handle subnormals and
// exponent overflow explicitly; native state may have been supplied by a
// signal handler or a stock x87 instruction.
//
// 15360 = 16383 (f80 bias) - 1023 (f64 bias).  Doesn't fit in imm12/imm12<<12,
// so we do SUB #16384 + ADD #1024 to subtract it without burning a register.
// =============================================================================
void emit_f80_to_f64_convert(AssemblerBuffer& buf, int Xmant_inout, int Wexp, int Xsign, int Wd_aux,
                             int Wd_tmp) {
    // Wd_aux holds the rounding-carry first, then is reused for the
    // 0x7FFF/0x7FF constants; the carry is consumed before the override
    // constants are needed.

    // Sign bit -> Xsign[0]; clear sign in Wexp so it holds exp_low.
    emit_bitfield(buf, /*is_64bit=*/1, /*opc=*/2 /*UBFM*/, /*N=*/1,
                  /*immr=*/15, /*imms=*/15, Wexp, Xsign);
    LogicalImmEncoding enc_15bits;
    is_bitmask_immediate(/*is_64bit=*/false, 0x7FFFU, enc_15bits);
    emit_and_imm(buf, /*is_64bit=*/0, Wexp, enc_15bits.N, enc_15bits.immr, enc_15bits.imms, Wexp);

    // Small extended exponents produce binary64 subnormals. Handle them
    // separately so native state can cross a reply boundary losslessly.
    emit_movn(buf, 0, 2, 0, 0x3c00, Wd_aux);
    emit_subs_reg(buf, 0, Wexp, Wd_aux, 31);
    const auto small_branch = buf.end;
    emit_b_cond(buf, 9 /*LS*/, 0);

    // Handle Inf/NaN before rounding so a NaN payload cannot carry into
    // the exponent or disappear. All branches stay inside this reply.
    emit_movn(buf, 0, 2, 0, 0x7fff, Wd_aux);
    emit_subs_reg(buf, 0, Wexp, Wd_aux, 31);
    const auto special_branch = buf.end;
    emit_b_cond(buf, 0 /*EQ*/, 0);

    // Add half minus one plus the retained low bit, then truncate 11 bits.
    // ADDS records a carry out of the explicit integer bit, which increments
    // the binary64 exponent when rounding 1.999... to 2.0.
    emit_bitfield(buf, 1, 2, 1, 11, 11, Xmant_inout, Wd_aux);
    emit_add_imm(buf, 1, 0, 0, 0, 0x3ff, Wd_aux, Wd_aux);
    emit_add_sub_shifted_reg(buf, 1, 0, 1, 0, Wd_aux, 0, Xmant_inout, Xmant_inout);
    emit_cset(buf, /*is_64bit=*/0, /*cond=*/2 /*CS*/, Wd_aux);

    // Mantissa: drop integer bit + low 11 fractional bits -> 52-bit value.
    emit_bitfield(buf, /*is_64bit=*/1, /*opc=*/2 /*UBFM*/, /*N=*/1,
                  /*immr=*/11, /*imms=*/63, Xmant_inout, Xmant_inout);
    LogicalImmEncoding enc_mant52;
    is_bitmask_immediate(/*is_64bit=*/true, 0x000FFFFFFFFFFFFFULL, enc_mant52);
    emit_and_imm(buf, /*is_64bit=*/1, Xmant_inout, enc_mant52.N, enc_mant52.immr, enc_mant52.imms,
                 Xmant_inout);

    // Compute exp_adj normal case in Wd_tmp:
    //   SUB Wd_tmp, Wexp, #0x4000, LSL #12   (-16384)
    //   ADD Wd_tmp, Wd_tmp, #0x400           (+1024 -> -15360)
    emit_add_imm(buf, /*is_64bit=*/0, /*is_sub=*/1, /*is_set_flags=*/0,
                 /*shift=*/1, /*imm12=*/4, Wexp, Wd_tmp);
    emit_add_imm(buf, /*is_64bit=*/0, /*is_sub=*/0, /*is_set_flags=*/0,
                 /*shift=*/0, /*imm12=*/0x400, Wd_tmp, Wd_tmp);

    // Apply the rounding carry from ADDS:
    // exp_adj += Wd_aux.
    emit_add_sub_shifted_reg(buf, /*is_64bit=*/0, /*is_sub=*/0, /*is_set_flags=*/0,
                             /*shift_type=*/0, /*Rm=*/Wd_aux, /*shift_amount=*/0,
                             /*Rn=*/Wd_tmp, /*Rd=*/Wd_tmp);

    // A finite extended value beyond binary64's range becomes infinity.
    emit_add_imm(buf, 0, 1, 1, 0, 0x7ff, Wd_tmp, 31);
    emit_csel(buf, 1, Xmant_inout, 31, Xmant_inout, 2 /*HS*/);
    emit_movn(buf, 0, 2, 0, 0x7ff, Wd_aux);
    emit_csel(buf, 0, Wd_tmp, Wd_aux, Wd_tmp, 2 /*HS*/);

    // Build f64 raw bits in Xmant_inout via two BFIs.
    //   BFI Xmant_inout, Xd_tmp, #52, #11   -> bits [62:52] = exp_adj[10:0]
    //   BFI Xmant_inout, Xsign,  #63, #1    -> bit 63 = sign[0]
    // BFI is BFM with opc=01.  For lsb,width: immr=(64-lsb)%64, imms=width-1.
    emit_bitfield(buf, /*is_64bit=*/1, /*opc=*/1 /*BFM*/, /*N=*/1,
                  /*immr=*/12, /*imms=*/10, Wd_tmp, Xmant_inout);
    emit_bitfield(buf, /*is_64bit=*/1, /*opc=*/1 /*BFM*/, /*N=*/1,
                  /*immr=*/1, /*imms=*/0, Xsign, Xmant_inout);
    const auto normal_done = buf.end;
    emit_b(buf, 0);

    buf.data[small_branch / 4] =
        0x54000009U | (static_cast<uint32_t>((buf.end - small_branch) / 4) << 5);
    // shift = 15372 - exp. A shift of 64 is the half-min-subnormal case.
    emit_movn(buf, 0, 2, 0, 15372, Wd_tmp);
    emit_subs_reg(buf, 0, Wd_tmp, Wexp, Wd_tmp);
    emit_add_imm(buf, 0, 1, 1, 0, 64, Wd_tmp, 31);
    const auto tiny_branch = buf.end;
    emit_b_cond(buf, 2 /*HS*/, 0);
    emit_add_sub_shifted_reg(buf, 1, 1, 0, 0, Wd_tmp, 0, 31, Wd_aux);
    emit_lslv(buf, 1, Wd_aux, Xmant_inout, Wd_aux);
    // LSRV Xmant, Xmant, Xshift.
    buf.emit(0x9ac02400U | (Wd_tmp << 16) | (Xmant_inout << 5) | Xmant_inout);
    emit_movn(buf, 1, 2, 3, 0x8000, Wexp);
    emit_subs_reg(buf, 1, Wd_aux, Wexp, 31);
    emit_cset(buf, 0, 8 /*HI*/, Wd_tmp);
    emit_cset(buf, 0, 0 /*EQ*/, Wd_aux);
    emit_logical_shifted_reg(buf, 1, 0, 0, 0, Xmant_inout, 0, Wd_aux, Wd_aux);
    emit_add_sub_shifted_reg(buf, 1, 0, 0, 0, Wd_tmp, 0, Xmant_inout, Xmant_inout);
    emit_add_sub_shifted_reg(buf, 1, 0, 0, 0, Wd_aux, 0, Xmant_inout, Xmant_inout);
    const auto rounded_done = buf.end;
    emit_b(buf, 0);

    buf.data[tiny_branch / 4] =
        0x54000002U | (static_cast<uint32_t>((buf.end - tiny_branch) / 4) << 5);
    emit_cset(buf, 0, 0 /*EQ*/, Wd_tmp);
    emit_movn(buf, 1, 2, 3, 0x8000, Wd_aux);
    emit_subs_reg(buf, 1, Xmant_inout, Wd_aux, 31);
    emit_cset(buf, 0, 8 /*HI*/, Xmant_inout);
    emit_logical_shifted_reg(buf, 1, 0, 0, 0, Wd_tmp, 0, Xmant_inout, Xmant_inout);
    buf.data[rounded_done / 4] = 0x14000000U | static_cast<uint32_t>((buf.end - rounded_done) / 4);
    emit_bitfield(buf, 1, 1, 1, 1, 0, Xsign, Xmant_inout);
    const auto small_done = buf.end;
    emit_b(buf, 0);

    buf.data[special_branch / 4] =
        0x54000000U | (static_cast<uint32_t>((buf.end - special_branch) / 4) << 5);
    // Remove the explicit integer bit. Preserve representable payload bits,
    // and quiet any NaN, including one with only low extended payload bits.
    emit_bitfield(buf, 1, 2, 1, 63, 62, Xmant_inout, Wd_aux);
    emit_bitfield(buf, 1, 2, 1, 11, 62, Xmant_inout, Xmant_inout);
    const auto infinity = buf.end;
    emit_cbz(buf, 1, 0, Wd_aux, 0);
    LogicalImmEncoding quiet_nan;
    is_bitmask_immediate(true, 0x0008000000000000ULL, quiet_nan);
    emit_orr_imm(buf, 1, Xmant_inout, Xmant_inout, quiet_nan.N, quiet_nan.immr, quiet_nan.imms);
    buf.data[infinity / 4] |= static_cast<uint32_t>((buf.end - infinity) / 4) << 5;
    emit_movn(buf, 0, 2, 0, 0x7ff, Wd_tmp);
    emit_bitfield(buf, 1, 1, 1, 12, 10, Wd_tmp, Xmant_inout);
    emit_bitfield(buf, 1, 1, 1, 1, 0, Xsign, Xmant_inout);
    buf.data[small_done / 4] = 0x14000000U | static_cast<uint32_t>((buf.end - small_done) / 4);
    buf.data[normal_done / 4] = 0x14000000U | static_cast<uint32_t>((buf.end - normal_done) / 4);
}

// =============================================================================
// f64 -> f80 (write).
//
// IEEE 754 double:  [63] sign | [62:52] exp (11-bit, bias 1023) | [51:0] mant
// x87 f80:          bytes 0-7 mantissa (64-bit, explicit integer bit at 63)
//                   bytes 8-9 [15] sign | [14:0] exp (15-bit, bias 16383)
//
// Four paths: normal, subnormal, zero, and Inf/NaN. Subnormals are
// normalized with CLZ and retain every binary64 significand bit.
//
// All branch displacements are PC-relative (instruction units), so this
// helper is safe to invoke any number of times back-to-back in one block.
// =============================================================================
void emit_f64_to_f80(AssemblerBuffer& buf, int Xaddr_slot, int Dd_src, int Xbits, int Wexp,
                     int Wd_tmp) {
    // FMOV Xbits, Dd_src ; raw double bits to GPR
    emit_fmov_d_to_x(buf, Xbits, Dd_src);

    // UBFX Xexp, Xbits, #52, #11 ; extract 11-bit exponent
    emit_bitfield(buf, /*is_64bit=*/1, /*opc=*/2, /*N=*/1, /*immr=*/52, /*imms=*/62, Xbits, Wexp);

    // LSR Wd_tmp, Xbits, #48 ; shift sign from bit 63 to bit 15
    emit_bitfield(buf, 1, 2, 1, 48, 63, Xbits, Wd_tmp);

    // AND Wd_tmp, Wd_tmp, #0x8000 ; isolate sign at bit 15
    LogicalImmEncoding enc_sign;
    is_bitmask_immediate(/*is_64bit=*/false, 0x8000, enc_sign);
    emit_and_imm(buf, 0, Wd_tmp, enc_sign.N, enc_sign.immr, enc_sign.imms, Wd_tmp);

    // LSL Xbits, Xbits, #11 ; position 52-bit mantissa for f80 (bits [62:11])
    //     UBFM Xd, Xn, #53, #52 is the alias for LSL Xd, Xn, #11
    emit_bitfield(buf, 1, 2, 1, 53, 52, Xbits, Xbits);

    // CBZ Wexp, .zero_denorm
    emit_cbz(buf, /*is_64bit=*/0, /*is_nz=*/0, Wexp, 10);

    // CMP Wexp, #0x7FF  (SUBS WZR, Wexp, #2047)
    emit_add_imm(buf, /*is_64bit=*/0, /*is_sub=*/1, /*is_set_flags=*/1,
                 /*shift=*/0, /*imm12=*/0x7FF, Wexp, /*Rd=*/31);

    // B.EQ .inf_nan
    emit_b_cond(buf, /*cond=*/0 /*EQ*/, 21);

    // ── Normal number ──
    // ORR Xbits, Xbits, #0x8000000000000000 ; set explicit integer bit
    LogicalImmEncoding enc_intbit;
    is_bitmask_immediate(/*is_64bit=*/true, 0x8000000000000000ULL, enc_intbit);
    emit_orr_imm(buf, 1, Xbits, Xbits, enc_intbit.N, enc_intbit.immr, enc_intbit.imms);

    // ADD Wexp, Wexp, #3, LSL#12  (+12288)
    emit_add_imm(buf, 0, 0, 0, /*shift=*/1, /*imm12=*/3, Wexp, Wexp);
    // ADD Wexp, Wexp, #0xC00      (+3072; total +15360 = 16383−1023)
    emit_add_imm(buf, 0, 0, 0, 0, 0xC00, Wexp, Wexp);

    // ORR Wexp, Wexp, Wd_tmp ; combine biased exponent + sign
    emit_logical_shifted_reg(buf, 0, /*opc=*/1, /*n=*/0, /*shift_type=*/0, Wd_tmp,
                             /*shift_amount=*/0, Wexp, Wexp);

    // STR Xbits, [Xaddr_slot] ; 8-byte mantissa
    emit_str_imm(buf, /*size=*/3, Xbits, Xaddr_slot, /*imm12=*/0);
    // STRH Wexp, [Xaddr_slot, #8] ; 2-byte exponent (imm12=4, scaled by 2)
    emit_str_imm(buf, /*size=*/1, Wexp, Xaddr_slot, /*imm12=*/4);

    // B .done
    emit_b(buf, 18);

    // ── Zero / subnormal ──
    emit_cbz(buf, 1, 0, Xbits, 10);
    // CLZ and LSLV normalize a nonzero binary64 subnormal significand.
    buf.emit(0xdac01000U | (Xbits << 5) | Wexp);
    emit_lslv(buf, 1, Wexp, Xbits, Xbits);
    emit_add_sub_shifted_reg(buf, 0, 1, 0, 0, Wexp, 0, 31, Wexp);
    emit_add_imm(buf, 0, 0, 0, 1, 3, Wexp, Wexp);
    emit_add_imm(buf, 0, 0, 0, 0, 0xc01, Wexp, Wexp);
    emit_logical_shifted_reg(buf, 0, 1, 0, 0, Wd_tmp, 0, Wexp, Wexp);
    emit_str_imm(buf, 3, Xbits, Xaddr_slot, 0);
    emit_str_imm(buf, 1, Wexp, Xaddr_slot, 4);
    emit_b(buf, 8);
    // Zero: STR XZR, [Xaddr_slot] ; mantissa = 0
    emit_str_imm(buf, 3, /*Rt=*/31, Xaddr_slot, 0);
    // STRH Wd_tmp, [Xaddr_slot, #8] ; exponent = sign only
    emit_str_imm(buf, 1, Wd_tmp, Xaddr_slot, 4);
    // B .done
    emit_b(buf, 5);

    // ── Infinity / NaN ──
    // ORR Xbits, Xbits, #0x8000000000000000 ; set explicit integer bit
    emit_orr_imm(buf, 1, Xbits, Xbits, enc_intbit.N, enc_intbit.immr, enc_intbit.imms);
    // STR Xbits, [Xaddr_slot] ; mantissa
    emit_str_imm(buf, 3, Xbits, Xaddr_slot, 0);
    // ORR Wexp, Wd_tmp, #0x7FFF ; sign | max exponent
    LogicalImmEncoding enc_7fff;
    is_bitmask_immediate(/*is_64bit=*/false, 0x7FFF, enc_7fff);
    emit_orr_imm(buf, 0, Wexp, Wd_tmp, enc_7fff.N, enc_7fff.immr, enc_7fff.imms);
    // STRH Wexp, [Xaddr_slot, #8] ; exponent
    emit_str_imm(buf, 1, Wexp, Xaddr_slot, 4);
}

// Rosetta imports/exports physical x87 slots as packed f80 at +6, stride 10.
// Our emitters use compact doubles at +8, stride 8, only within one reply.
// Convert in place from low to high when shrinking, high to low when expanding,
// so a store never overwrites another live source slot. Empty slots have no
// architectural value and must not be decoded as if they contained f80.
void emit_native_state_boundary(TranslationResult& tr, bool entering) {
    const auto saved_mask = tr.free_gpr_mask;
    tr.free_gpr_mask &= kGprScratchMask;
    auto& buf = tr.insn_buf;
    const int base = alloc_free_gpr(tr);
    const int flags = alloc_free_gpr(tr);
    const int mant = alloc_free_gpr(tr);
    const int exp = alloc_free_gpr(tr);
    const int sign = alloc_free_gpr(tr);
    const int aux = alloc_free_gpr(tr);
    const int tmp = alloc_free_gpr(tr);
    const int fp = alloc_free_fpr(tr);
    emit_x87_base(buf, tr, base);
    emit_ldr_str_imm(buf, 1, 0, 1, 2, base, exp);
    // The empty-stack path leaves NZCV untouched. Adding one to a loaded
    // halfword sets bit 16 exactly when every tag is empty (0xffff).
    emit_add_imm(buf, 0, 0, 0, 0, 1, exp, exp);
    const auto empty = buf.end;
    buf.emit(0x37000000U | (16U << 19) | exp);  // TBNZ Wexp, #16, .done
    emit_mrs_nzcv(buf, flags);
    for (int j = 0; j < 8; ++j) {
        const int i = entering ? j : 7 - j;
        emit_ldr_str_imm(buf, 1, 0, 1, 2, base, exp);
        emit_bitfield(buf, 0, 2, 0, 2 * i, 2 * i + 1, exp, exp);
        emit_add_imm(buf, 0, 1, 1, 0, 3, exp, 31);
        const auto skip = buf.end;
        emit_b_cond(buf, 0, 0);
        if (entering) {
            emit_ldur_stur(buf, 3, 1, 6 + 10 * i, base, mant);
            emit_ldr_str_imm(buf, 1, 0, 1, (14 + 10 * i) / 2, base, exp);
            TranslatorX87::emit_f80_to_f64_convert(buf, mant, exp, sign, aux, tmp);
            emit_str_imm(buf, 3, mant, base, 1 + i);
        } else {
            emit_add_imm(buf, 1, 0, 0, 0, 6 + 10 * i, base, sign);
            emit_ldr_str_imm(buf, 3, 1, 1, 1 + i, base, fp);
            TranslatorX87::emit_f64_to_f80(buf, sign, fp, mant, exp, tmp);
        }
        buf.data[skip / 4] = 0x54000000U | (static_cast<uint32_t>((buf.end - skip) / 4) << 5);
    }
    emit_msr_nzcv(buf, flags);
    buf.data[empty / 4] |= static_cast<uint32_t>((buf.end - empty) / 4) << 5;
    free_fpr(tr, fp);
    tr.free_gpr_mask = saved_mask;
}

}  // namespace TranslatorX87
