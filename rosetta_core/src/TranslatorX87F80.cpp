#include "rosetta_core/TranslatorX87F80.hpp"

#include <cassert>
#include <cstdint>

#include "rosetta_core/AssemblerBuffer.h"
#include "rosetta_core/AssemblerHelpers.hpp"
#include "rosetta_core/Config.h"
#include "rosetta_core/CoreConfig.h"
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

// X87_IGNORE_FTZ_DAZ=1. Real x87 has no flush-to-zero: MXCSR.FTZ and DAZ only
// apply to SSE. On FEAT_AFP hardware Rosetta maps them to FPCR.FZ (bit 24)
// and FPCR.FIZ (bit 0); without FEAT_AFP to bits 1 and 0 of the Apple register
// S3_6_C15_C2_5 (see flush_control below). The emitted x87 code does its
// arithmetic and f32/f64 conversions under those settings, so with either bit
// set a denormal input or result becomes zero, down to a plain
// `fld m32; fstp m32` copy. With the knob set, the reply clears both bits in
// whichever register holds them on entry and puts them back on exit. With it
// unset (the default) the boundary emits exactly what it emits without this
// code, so programs that never set FTZ or DAZ pay nothing.
//
// FPCR is only written when a bit has to change. On an M5 Max a MSR to FPCR
// costs about 3.5 ns even when it writes the value already there, and about
// 10 ns when it changes FZ or FIZ, against under 1 ns for a short run. A
// taken branch per boundary was measurable on short runs too, so in the usual
// case (no flush bit set, nothing to convert) both bits are folded into the
// empty-stack test and the boundary still takes one branch. With values on
// the stack, entry and exit each take one more branch.
//
// A signal handler that interrupts a run starts with the flush bits the run
// cleared, so it reads FTZ and DAZ as clear; the MXCSR in its signal context
// is correct, and the interrupted code gets both bits back when it returns.
//
// Stash, the halfword at +6: bit 0 set when the exit has to restore, bit 1
// the original FZ, bit 2 the original FIZ. Every entry writes it, after the
// entry conversion has read native slot 0, and the exit reads it before the
// exit conversion rewrites that slot. It is free only because the compact
// layout keeps doubles at +8, stride 8, and so never touches +6..+7 between
// the two conversions. Native transport replies do work on the native slots
// (+6, stride 10), and are still safe: they skip this boundary, so they
// neither write nor read the stash, and their integer conversions and
// LDR D / STR D copies do no floating-point arithmetic, so FZ and FIZ do not
// affect them. X87_NATIVE_SLOTS replies keep values inside the native slots,
// where this halfword holds slot 0's data, so they stash at +86 instead
// (emit_native_slots_fpcr_enter).
//
// Nothing here touches NZCV and every branch goes forward.
static constexpr int16_t kFpcrStashHalfword = 3;  // +6, in halfwords

// Older Apple hardware without FEAT_AFP has no FPCR.FIZ, and FPCR.FZ
// there flushes inputs as well as outputs, so Rosetta does not use FPCR for
// MXCSR.FTZ/DAZ at all: it keeps FPCR.FZ clear and sets the Apple register
// S3_6_C15_C2_5 instead, DAZ in bit 0 and FTZ in bit 1. The runtime picks
// between the two from the commpage CPU capabilities (bit 34, FEAT_AFP; else
// bit 38, the Apple register), and so does flush_control(). The register is
// not accessible at EL0 on FEAT_AFP hardware, so the choice has to be right.
FlushControl flush_control() {
    static const FlushControl fc = [] {
        // _COMM_PAGE_CPU_CAPABILITIES64 in the arm64 commpage.
        const uint64_t caps = *reinterpret_cast<const volatile uint64_t*>(0x0000000FFFFFC010ULL);
        if ((caps >> 34) & 1U)
            return FlushControl::kFpcr;
        if ((caps >> 38) & 1U)
            return FlushControl::kAfpcr;
        return FlushControl::kNone;
    }();
    return fc;
}

const char* flush_control_name(FlushControl fc) {
    switch (fc) {
        case FlushControl::kFpcr:
            return "FPCR.FZ/FIZ (FEAT_AFP)";
        case FlushControl::kAfpcr:
            return "S3_6_C15_C2_5 bits 1/0 (no FEAT_AFP)";
        default:
            return "none";
    }
}

namespace {
// The register holding the flush bits, and their positions in it. DAZ is
// bit 0 in both registers.
struct FlushReg {
    uint32_t mrs;  // MRS X0, <reg>
    uint32_t msr;  // MSR <reg>, X0
    int ftz;
    int daz;
};
constexpr FlushReg kFpcrReg{0xD53B4400U, 0xD51B4400U, 24, 0};
constexpr FlushReg kAfpcrReg{0xD53EF2A0U, 0xD51EF2A0U, 1, 0};
static_assert(kFpcrReg.daz == 0 && kAfpcrReg.daz == 0, "the stash code assumes DAZ in bit 0");

const FlushReg& flush_reg() {
    return flush_control() == FlushControl::kAfpcr ? kAfpcrReg : kFpcrReg;
}

void emit_mrs_flush(AssemblerBuffer& buf, int Xd) {
    buf.emit(flush_reg().mrs | static_cast<uint32_t>(Xd & 0x1F));
}

void emit_msr_flush(AssemblerBuffer& buf, int Xd) {
    buf.emit(flush_reg().msr | static_cast<uint32_t>(Xd & 0x1F));
}
}  // namespace

static bool ignore_ftz_daz() {
    return g_rosetta_config != nullptr && g_rosetta_config->ignore_ftz_daz != 0 &&
           flush_control() != FlushControl::kNone;
}

static void patch_forward(AssemblerBuffer& buf, size_t at) {
    buf.data[at / 4] |= static_cast<uint32_t>((buf.end - at) / 4) << 5;
}

static void emit_tbnz(AssemblerBuffer& buf, int Wt, int bit) {
    buf.emit(0x37000000U | (static_cast<uint32_t>(bit) << 19) | static_cast<uint32_t>(Wt));
}

// Wquiet = Wtags1 with bit 16 kept only when no flush bit is set in Wfpcr:
//   BIC Wquiet, Wtags1, Wfpcr, LSL #16        (DAZ, bit 0)
//   BIC Wquiet, Wquiet, Wfpcr, LSR #8 / LSL #15  (FTZ, bit 24 / bit 1)
static void emit_quiet_entry_test(AssemblerBuffer& buf, int Wtags1, int Wfpcr, int Wquiet) {
    const FlushReg& r = flush_reg();
    emit_logical_shifted_reg(buf, 0, /*AND*/ 0, /*n=*/1, /*LSL*/ 0, Wfpcr, 16 - r.daz, Wtags1,
                             Wquiet);
    if (r.ftz > 16)
        emit_logical_shifted_reg(buf, 0, /*AND*/ 0, /*n=*/1, /*LSR*/ 1, Wfpcr, r.ftz - 16, Wquiet,
                                 Wquiet);
    else
        emit_logical_shifted_reg(buf, 0, /*AND*/ 0, /*n=*/1, /*LSL*/ 0, Wfpcr, 16 - r.ftz, Wquiet,
                                 Wquiet);
}

// Entry path for a non-empty stack or a set flush bit. Reads the flush
// register again, since the conversion reused the registers. With neither
// bit set it branches to `to_quiet`; otherwise it stores the stash, clears
// both bits and branches to `to_done`. The caller patches both branches.
// `stash` is the stash's offset from base in halfwords.
static void emit_fpcr_enter_slow(AssemblerBuffer& buf, int base, int Xfpcr, int Wbits,
                                 int16_t stash, size_t& to_quiet, size_t& to_done) {
    const FlushReg& r = flush_reg();
    emit_mrs_flush(buf, Xfpcr);
    emit_bitfield(buf, 0, /*UBFM*/ 2, 0, r.ftz, r.ftz, Xfpcr, Wbits);            // FTZ -> bit 0
    emit_bitfield(buf, 0, /*BFM*/ 1, 0, /*immr=*/31, /*imms=*/0, Xfpcr, Wbits);  // DAZ -> bit 1
    to_quiet = buf.end;
    emit_cbz(buf, 0, 0, Wbits, 0);
    emit_bitfield(buf, 0, /*UBFM*/ 2, 0, 31, 30, Wbits, Wbits);  // LSL #1
    LogicalImmEncoding one;
    is_bitmask_immediate(false, 1, one);
    emit_orr_imm(buf, 0, Wbits, Wbits, one.N, one.immr, one.imms);
    emit_ldr_str_imm(buf, 1, 0, /*STR*/ 0, stash, base, Wbits);
    LogicalImmEncoding no_fz;
    LogicalImmEncoding no_fiz;
    is_bitmask_immediate(false, ~(uint64_t(1) << r.ftz) & 0xFFFFFFFFULL, no_fz);
    is_bitmask_immediate(false, ~(uint64_t(1) << r.daz) & 0xFFFFFFFFULL, no_fiz);
    emit_and_imm(buf, 0, Xfpcr, no_fz.N, no_fz.immr, no_fz.imms, Xfpcr);
    emit_and_imm(buf, 0, Xfpcr, no_fiz.N, no_fiz.immr, no_fiz.imms, Xfpcr);
    emit_msr_flush(buf, Xfpcr);
    to_done = buf.end;
    buf.emit(0x14000000U);  // B .done
}

// Exit restore, entered with the stash in Wstash (bit 0 known set).
static void emit_fpcr_restore(AssemblerBuffer& buf, int Xfpcr, int Wstash) {
    const FlushReg& r = flush_reg();
    emit_mrs_flush(buf, Xfpcr);
    emit_bitfield(buf, 0, /*UBFM*/ 2, 0, 1, 31, Wstash, Wstash);  // LSR #1
    emit_bitfield(buf, 0, /*BFM*/ 1, 0, static_cast<int8_t>((32 - r.ftz) & 31), 0, Wstash,
                  Xfpcr);                                          // BFI #ftz, #1
    emit_bitfield(buf, 0, /*UBFM*/ 2, 0, 1, 31, Wstash, Wstash);   // LSR #1
    emit_bitfield(buf, 0, /*BFM*/ 1, 0, r.daz, 0, Wstash, Xfpcr);  // BFXIL #0, #1
    emit_msr_flush(buf, Xfpcr);
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
    // Without X87_IGNORE_FTZ_DAZ the boundary is the plain conversion: no
    // FPCR access, no stash, and `quiet` is never emitted.
    const bool fpcr = ignore_ftz_daz();
    // Entry: Xmant = the flush register. Exit: Wsign = stash.
    if (fpcr && entering)
        emit_mrs_flush(buf, mant);
    else if (fpcr)
        emit_ldr_str_imm(buf, 1, 0, /*LDR*/ 1, kFpcrStashHalfword, base, sign);
    emit_ldr_str_imm(buf, 1, 0, 1, 2, base, exp);
    // The empty-stack path leaves NZCV untouched. Adding one to a loaded
    // halfword sets bit 16 exactly when every tag is empty (0xffff). Waux
    // keeps that bit only when FPCR needs no change either.
    emit_add_imm(buf, 0, 0, 0, 0, 1, exp, exp);
    size_t quiet = 0;
    if (fpcr) {
        if (entering)
            emit_quiet_entry_test(buf, exp, mant, aux);
        else  // BIC Waux, Wexp, Wsign, LSL #16 (restore flag)
            emit_logical_shifted_reg(buf, 0, /*AND*/ 0, /*n=*/1, /*LSL*/ 0, sign, 16, exp, aux);
        quiet = buf.end;
        emit_tbnz(buf, aux, 16);  // .quiet
        if (!entering) {
            const auto keep = buf.end;
            emit_cbz(buf, 0, 0, sign, 0);  // CBZ Wsign, .convert
            emit_fpcr_restore(buf, mant, sign);
            patch_forward(buf, keep);
        }
    }
    const auto empty = buf.end;
    emit_tbnz(buf, exp, 16);  // entry: .fpcr, exit: .quiet (knob off: .done)
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
    if (!fpcr) {
        patch_forward(buf, empty);
    } else if (entering) {
        // .fpcr, after the conversion has read native slot 0 at +6.
        patch_forward(buf, empty);
        size_t to_quiet = 0;
        size_t to_done = 0;
        emit_fpcr_enter_slow(buf, base, mant, sign, kFpcrStashHalfword, to_quiet, to_done);
        // .quiet: nothing to restore on exit.
        patch_forward(buf, to_quiet);
        patch_forward(buf, quiet);
        emit_ldr_str_imm(buf, 1, 0, /*STR*/ 0, kFpcrStashHalfword, base, GPR::XZR);
        // .done
        buf.data[to_done / 4] |= static_cast<uint32_t>((buf.end - to_done) / 4) & 0x3FFFFFFU;
    } else {
        patch_forward(buf, empty);
        patch_forward(buf, quiet);
    }
    free_fpr(tr, fp);
    tr.free_gpr_mask = saved_mask;
}

// ── Native transport ────────────────────────────────────────────────────────
//
// Branches below are emitted with a zero offset and patched to the current
// end of the buffer once the target is reached.  Every target lies ahead of
// its branch.

// TBZ/TBNZ Rt, #bit, <patched>.  Bits 32..63 select the X form.
static uint64_t emit_tbz_fwd(AssemblerBuffer& buf, int is_nz, int Rt, int bit) {
    const uint64_t at = buf.end;
    buf.emit((static_cast<uint32_t>(bit >> 5) << 31) | 0x36000000U |
             (static_cast<uint32_t>(is_nz != 0) << 24) | (static_cast<uint32_t>(bit & 31) << 19) |
             static_cast<uint32_t>(Rt & 31));
    return at;
}

static uint64_t emit_cbz_fwd(AssemblerBuffer& buf, int is_64bit, int is_nz, int Rt) {
    const uint64_t at = buf.end;
    emit_cbz(buf, is_64bit, is_nz, Rt, 0);
    return at;
}

static uint64_t emit_b_fwd(AssemblerBuffer& buf) {
    const uint64_t at = buf.end;
    emit_b(buf, 0);
    return at;
}

static void bind_imm14(AssemblerBuffer& buf, uint64_t at) {
    buf.data[at / 4] |= (static_cast<uint32_t>((buf.end - at) / 4) & 0x3FFFU) << 5;
}

static void bind_imm19(AssemblerBuffer& buf, uint64_t at) {
    buf.data[at / 4] |= (static_cast<uint32_t>((buf.end - at) / 4) & 0x7FFFFU) << 5;
}

static void bind_imm26(AssemblerBuffer& buf, uint64_t at) {
    buf.data[at / 4] |= static_cast<uint32_t>((buf.end - at) / 4) & 0x3FFFFFFU;
}

static void and_mask(AssemblerBuffer& buf, int is_64bit, int Rd, int Rn, uint64_t mask) {
    LogicalImmEncoding enc;
    const bool ok = is_bitmask_immediate(is_64bit != 0, mask, enc);
    assert(ok);
    (void)ok;
    emit_and_imm(buf, is_64bit, Rd, enc.N, enc.immr, enc.imms, Rn);
}

static void orr_mask(AssemblerBuffer& buf, int is_64bit, int Rd, int Rn, uint64_t mask) {
    LogicalImmEncoding enc;
    const bool ok = is_bitmask_immediate(is_64bit != 0, mask, enc);
    assert(ok);
    (void)ok;
    emit_orr_imm(buf, is_64bit, Rd, Rn, enc.N, enc.immr, enc.imms);
}

// ── Flag-free f80 <-> binary64 ──────────────────────────────────────────────
//
// The results of emit_f80_to_f64_convert (but see the unnormals below) and
// emit_f64_to_f80, written for code that holds the x87 registers in the
// native layout and converts one slot at a time inside a run: four GPRs, no
// NZCV write, and control flow through CBZ/CBNZ/TBZ/TBNZ and B only, all
// forward.

// With e the biased exponent and m the significand, a normal result is
// built as (hi << 52) + r, where hi = sign << 11 | (e - 15361) and r is m
// rounded to its top 53 bits, the integer bit included.  r = 2^53 after a
// rounding carry adds one more to the exponent field, and an exponent field
// of 0x7FF that way is infinity with a zero fraction, so neither needs a
// case of its own.  e - 15361 >= 0x7FE overflows before rounding and goes
// to the path that also takes infinities and NaNs.
//
// One difference from emit_f80_to_f64_convert: an encoding with the integer
// bit clear under a nonzero exponent (unnormal, pseudo-infinity, pseudo-NaN)
// becomes the default NaN, as on the x87 and in stock Rosetta, where the
// flag-based version reads an unnormal as if the bit were set.  Its callers
// hold Rosetta's own 80-bit registers, so they meet these encodings as
// stock does.
void emit_f80_bits_to_f64(AssemblerBuffer& buf, int Xm, int Wx, int Xt1, int Xt2,
                          bool round_to_odd) {
    // Integer bit clear under a nonzero exponent: an unnormal, pseudo-
    // infinity or pseudo-NaN, which the x87 and stock Rosetta read as the
    // default NaN.
    const auto integer_bit = emit_tbz_fwd(buf, 1, Xm, 63);
    and_mask(buf, 0, Xt1, Wx, 0x7fff);
    const auto exponent_zero = emit_cbz_fwd(buf, 0, 0, Xt1);
    emit_movn(buf, 1, /*MOVZ*/ 2, 3, 0xfff8, Xm);  // 0xFFF8000000000000
    const auto done_invalid = emit_b_fwd(buf);
    bind_imm14(buf, integer_bit);
    bind_imm19(buf, exponent_zero);

    and_mask(buf, 0, Xt1, Wx, 0x7fff);
    emit_add_imm(buf, 0, /*sub*/ 1, 0, /*LSL 12*/ 1, 4, Xt1, Xt1);  // e - 16384
    emit_add_imm(buf, 0, 0, 0, 0, 1023, Xt1, Xt1);                 // e - 15361
    const auto small = emit_tbz_fwd(buf, 1, Xt1, 31);
    emit_add_imm(buf, 0, /*sub*/ 1, 0, 0, 0x7fe, Xt1, Xt2);
    const auto big = emit_tbz_fwd(buf, 0, Xt2, 31);

    emit_bitfield(buf, 0, /*UBFM*/ 2, 0, 15, 15, Wx, Xt2);                 // sign
    emit_add_sub_shifted_reg(buf, 0, 0, 0, /*LSL*/ 0, Xt2, 11, Xt1, Xt1);  // hi
    if (round_to_odd) {
        // Truncate at bit 11 and set the result's lowest bit when any of the
        // low 11 bits is set: (low 11 bits + 0x7FF) reaches bit 11 then.
        and_mask(buf, 0, Wx, Xm, 0x7ff);
        emit_add_imm(buf, 0, 0, 0, 0, 0x7ff, Wx, Xt2);
        emit_bitfield(buf, 1, /*UBFM*/ 2, 1, 11, 63, Xm, Xm);                // LSR #11
        emit_logical_shifted_reg(buf, 1, /*ORR*/ 1, 0, /*LSR*/ 1, Xt2, 11, Xm, Xm);
    } else {
        // Round to nearest even at bit 11: the carry of (bit 11 + 0x3FF +
        // the low 11 bits) out of bit 11 is the increment.
        emit_bitfield(buf, 1, /*UBFM*/ 2, 1, 11, 11, Xm, Xt2);
        emit_add_imm(buf, 0, 0, 0, 0, 0x3ff, Xt2, Xt2);
        and_mask(buf, 0, Wx, Xm, 0x7ff);
        emit_add_sub_shifted_reg(buf, 0, 0, 0, 0, Wx, 0, Xt2, Xt2);
        emit_bitfield(buf, 1, /*UBFM*/ 2, 1, 11, 63, Xm, Xm);                // LSR #11
        emit_add_sub_shifted_reg(buf, 1, 0, 0, /*LSR*/ 1, Xt2, 11, Xm, Xm);  // r
    }
    emit_add_sub_shifted_reg(buf, 1, 0, 0, /*LSL*/ 0, Xt1, 52, Xm, Xm);   // + hi << 52
    const auto done_normal = emit_b_fwd(buf);

    // e <= 15360: a subnormal or zero result, m >> (15372 - e) rounded to
    // nearest even.  The bits shifted out, aligned at bit 63 in Wx, round
    // up when the top one is set and either another one is or the result is
    // odd.
    bind_imm14(buf, small);
    emit_add_sub_shifted_reg(buf, 0, /*sub*/ 1, 0, 0, Xt1, 0, 31, Xt2);  // 15361 - e
    emit_add_imm(buf, 0, 0, 0, 0, 11, Xt2, Xt2);                       // shift
    emit_bitfield(buf, 0, 2, 0, 15, 15, Wx, Xt1);                      // sign
    emit_add_imm(buf, 0, /*sub*/ 1, 0, 0, 64, Xt2, Wx);
    const auto tiny = emit_tbz_fwd(buf, 0, Wx, 31);
    emit_add_sub_shifted_reg(buf, 0, 1, 0, 0, Xt2, 0, 31, Wx);  // 64 - shift, modulo 64
    emit_lslv(buf, 1, Wx, Xm, Wx);
    buf.emit(0x9ac02400U | (static_cast<uint32_t>(Xt2) << 16) | (static_cast<uint32_t>(Xm) << 5) |
             static_cast<uint32_t>(Xm));  // LSRV Xm, Xm, Xt2
    if (round_to_odd) {
        const auto exact = emit_cbz_fwd(buf, 1, 0, Wx);
        orr_mask(buf, 1, Xm, Xm, 1);
        bind_imm19(buf, exact);
    } else {
        const auto below_half = emit_tbz_fwd(buf, 0, Wx, 63);
        emit_bitfield(buf, 1, 2, 1, 63, 62, Wx, Wx);  // LSL #1
        emit_bitfield(buf, 1, /*BFM*/ 1, 1, 0, 0, Xm, Wx);  // BFXIL Wx, Xm, #0, #1
        const auto even_half = emit_cbz_fwd(buf, 1, 0, Wx);
        emit_add_imm(buf, 1, 0, 0, 0, 1, Xm, Xm);
        bind_imm14(buf, below_half);
        bind_imm19(buf, even_half);
    }
    emit_logical_shifted_reg(buf, 1, /*ORR*/ 1, 0, 0, Xt1, 63, Xm, Xm);
    const auto done_small = emit_b_fwd(buf);

    // shift >= 64: half the smallest subnormal or less.  Only shift == 64
    // with m above 2^63 rounds up, to that subnormal.
    bind_imm14(buf, tiny);
    if (round_to_odd) {
        // Any nonzero m truncates to zero with a sticky bit: the smallest
        // subnormal.
        const auto zero = emit_cbz_fwd(buf, 1, 0, Xm);
        emit_movn(buf, 1, /*MOVZ*/ 2, 0, 1, Xm);
        bind_imm19(buf, zero);
    } else {
        const auto zero_far = emit_cbz_fwd(buf, 0, 1, Wx);
        const auto zero_low = emit_tbz_fwd(buf, 0, Xm, 63);
        emit_bitfield(buf, 1, 2, 1, 63, 62, Xm, Xm);  // LSL #1
        const auto exact_half = emit_cbz_fwd(buf, 1, 0, Xm);
        emit_movn(buf, 1, /*MOVZ*/ 2, 0, 1, Xm);
        const auto to_sign = emit_b_fwd(buf);
        bind_imm19(buf, zero_far);
        bind_imm14(buf, zero_low);
        emit_movn(buf, 1, /*MOVZ*/ 2, 0, 0, Xm);
        bind_imm19(buf, exact_half);
        bind_imm26(buf, to_sign);
    }
    emit_logical_shifted_reg(buf, 1, 1, 0, 0, Xt1, 63, Xm, Xm);
    const auto done_tiny = emit_b_fwd(buf);

    // e - 15361 >= 0x7FE: e == 0x7FFF is an infinity or NaN; anything else
    // overflows binary64 and becomes infinity.  A NaN keeps the top 52
    // fraction bits below the integer bit and is quieted.
    bind_imm14(buf, big);
    orr_mask(buf, 0, Xt2, Wx, 0x8000);
    emit_add_imm(buf, 0, 0, 0, 0, 1, Xt2, Xt2);
    const auto finite = emit_tbz_fwd(buf, 0, Xt2, 16);
    emit_bitfield(buf, 1, 2, 1, 63, 62, Xm, Xt2);  // fraction, LSL #1
    emit_bitfield(buf, 1, 2, 1, 11, 62, Xm, Xm);   // UBFX Xm, Xm, #11, #52
    const auto infinity = emit_cbz_fwd(buf, 1, 0, Xt2);
    orr_mask(buf, 1, Xm, Xm, uint64_t(1) << 51);
    const auto nan_done = emit_b_fwd(buf);
    bind_imm14(buf, finite);
    emit_movn(buf, 1, 2, 0, 0, Xm);
    bind_imm19(buf, infinity);
    bind_imm26(buf, nan_done);
    orr_mask(buf, 1, Xm, Xm, 0x7ff0000000000000ULL);
    emit_bitfield(buf, 0, 2, 0, 15, 15, Wx, Xt2);
    emit_logical_shifted_reg(buf, 1, 1, 0, 0, Xt2, 63, Xm, Xm);

    bind_imm26(buf, done_normal);
    bind_imm26(buf, done_small);
    bind_imm26(buf, done_tiny);
    bind_imm26(buf, done_invalid);
}

// Normal, zero, subnormal (normalised with CLZ, every bit kept) and
// infinity or NaN (integer bit set, exponent 0x7FFF), as emit_f64_to_f80.
// The infinity test is exponent + 1 reaching bit 11, where that version
// compares.
void emit_f64_bits_to_f80(AssemblerBuffer& buf, int Xaddr, int Xb, int We, int Wt) {
    emit_bitfield(buf, 1, 2, 1, 52, 62, Xb, We);  // UBFX We, Xb, #52, #11
    emit_bitfield(buf, 1, 2, 1, 48, 63, Xb, Wt);  // LSR Wt, Xb, #48
    and_mask(buf, 0, Wt, Wt, 0x8000);             // sign at bit 15
    emit_bitfield(buf, 1, 2, 1, 53, 52, Xb, Xb);  // LSL Xb, Xb, #11
    const auto zero_sub = emit_cbz_fwd(buf, 0, 0, We);
    emit_add_imm(buf, 0, 0, 0, 0, 1, We, We);
    const auto inf_nan = emit_tbz_fwd(buf, 1, We, 11);
    orr_mask(buf, 1, Xb, Xb, uint64_t(1) << 63);
    emit_add_imm(buf, 0, 0, 0, 1, 3, We, We);      // +12288
    emit_add_imm(buf, 0, 0, 0, 0, 0xbff, We, We);  // +3071: exp + 15360 in all
    emit_logical_shifted_reg(buf, 0, 1, 0, 0, Wt, 0, We, We);
    emit_str_imm(buf, 3, Xb, Xaddr, 0);
    emit_str_imm(buf, 1, We, Xaddr, 4);
    const auto done_normal = emit_b_fwd(buf);

    bind_imm19(buf, zero_sub);
    const auto zero = emit_cbz_fwd(buf, 1, 0, Xb);
    buf.emit(0xdac01000U | (static_cast<uint32_t>(Xb) << 5) | static_cast<uint32_t>(We));  // CLZ
    emit_lslv(buf, 1, We, Xb, Xb);
    emit_add_sub_shifted_reg(buf, 0, 1, 0, 0, We, 0, 31, We);
    emit_add_imm(buf, 0, 0, 0, 1, 3, We, We);
    emit_add_imm(buf, 0, 0, 0, 0, 0xc01, We, We);  // 15361 - clz
    emit_logical_shifted_reg(buf, 0, 1, 0, 0, Wt, 0, We, We);
    emit_str_imm(buf, 3, Xb, Xaddr, 0);
    emit_str_imm(buf, 1, We, Xaddr, 4);
    const auto done_sub = emit_b_fwd(buf);
    bind_imm19(buf, zero);
    emit_str_imm(buf, 3, 31, Xaddr, 0);
    emit_str_imm(buf, 1, Wt, Xaddr, 4);
    const auto done_zero = emit_b_fwd(buf);

    bind_imm14(buf, inf_nan);
    orr_mask(buf, 1, Xb, Xb, uint64_t(1) << 63);
    emit_str_imm(buf, 3, Xb, Xaddr, 0);
    orr_mask(buf, 0, We, Wt, 0x7fff);
    emit_str_imm(buf, 1, We, Xaddr, 4);

    bind_imm26(buf, done_normal);
    bind_imm26(buf, done_sub);
    bind_imm26(buf, done_zero);
}

// X87_NATIVE_SLOTS keeps each working binary64 value inside its own native
// slot, and the slot tracker knows statically which slots the reply left in
// binary64 (X87Cache::slot_fmt).  Those are converted back in place; slots
// the reply never read, and slots it popped or freed, are left as they are,
// and a reply that left none in binary64 gets no code at all.  TOP is read
// from memory, where the end of the run has put it, and the slot of tracker
// position r is at depth r - top_rel from it.  Tags are not consulted:
// converting an empty slot back is harmless.  Nothing here writes NZCV.
void emit_native_slots_export(TranslationResult& tr) {
    const auto& c = tr.x87_cache;
    int count = 0;
    for (const auto f : c.slot_fmt) {
        count += f == kSlotF64 ? 1 : 0;
    }
    if (count == 0) {
        return;
    }
    const auto saved_mask = tr.free_gpr_mask;
    tr.free_gpr_mask &= kGprScratchMask;
    auto& buf = tr.insn_buf;
    const int base = alloc_free_gpr(tr);
    const int top = alloc_free_gpr(tr);
    const int addr = alloc_free_gpr(tr);
    const int bits = alloc_free_gpr(tr);
    const int exp = alloc_free_gpr(tr);
    const int tmp = alloc_free_gpr(tr);
    emit_x87_base(buf, tr, base);
    emit_load_top(buf, tr, base, top);
    for (int r = 0; r < 8; ++r) {
        if (c.slot_fmt[r] != kSlotF64) {
            continue;
        }
        emit_native_slot_key(buf, top, (r - c.top_rel) & 7, addr);
        emit_add_imm(buf, 0, 0, 0, 0, kX87NativeSlotOff, addr, addr);
        emit_add_sub_shifted_reg(buf, 1, 0, 0, 0, addr, 0, base, addr);
        emit_ldr_str_imm(buf, 3, 0, /*LDR*/ 1, 0, addr, bits);
        emit_f64_bits_to_f80(buf, addr, bits, exp, tmp);
    }
    tr.free_gpr_mask = saved_mask;
}

// X87_IGNORE_FTZ_DAZ with X87_NATIVE_SLOTS.  The 80-bit values fill all ten
// bytes of every slot, so the stash cannot use +6 as the boundary does; it
// is the halfword right after slot 7, +86.  Rosetta's x87 register file and
// its exchanges with the signal context and the thread state
// (x86_FLOAT_STATE64) cover bytes +0 to +85.  The stash lives only from a
// reply's entry to its exit, so it needs no value between replies.  Entry
// and exit are the boundary's FPCR handling without the conversions: the
// exit restores FZ and FIZ when the entry cleared them.  Nothing here
// touches NZCV and every branch goes forward.
static constexpr int16_t kNativeSlotsStashHalfword = 43;  // +86, in halfwords

void emit_native_slots_fpcr_enter(TranslationResult& tr) {
    if (!ignore_ftz_daz()) {
        return;
    }
    const auto saved_mask = tr.free_gpr_mask;
    tr.free_gpr_mask &= kGprScratchMask;
    auto& buf = tr.insn_buf;
    const int base = alloc_free_gpr(tr);
    const int fpcr = alloc_free_gpr(tr);
    const int bits = alloc_free_gpr(tr);
    emit_x87_base(buf, tr, base);
    size_t to_quiet = 0;
    size_t to_done = 0;
    emit_fpcr_enter_slow(buf, base, fpcr, bits, kNativeSlotsStashHalfword, to_quiet, to_done);
    // .quiet: neither bit set, nothing to restore on exit.
    patch_forward(buf, to_quiet);
    emit_ldr_str_imm(buf, 1, 0, /*STR*/ 0, kNativeSlotsStashHalfword, base, GPR::XZR);
    // .done
    buf.data[to_done / 4] |= static_cast<uint32_t>((buf.end - to_done) / 4) & 0x3FFFFFFU;
    tr.free_gpr_mask = saved_mask;
}

void emit_native_slots_fpcr_exit(TranslationResult& tr) {
    if (!ignore_ftz_daz()) {
        return;
    }
    const auto saved_mask = tr.free_gpr_mask;
    tr.free_gpr_mask &= kGprScratchMask;
    auto& buf = tr.insn_buf;
    const int base = alloc_free_gpr(tr);
    const int fpcr = alloc_free_gpr(tr);
    const int stash = alloc_free_gpr(tr);
    emit_x87_base(buf, tr, base);
    emit_ldr_str_imm(buf, 1, 0, /*LDR*/ 1, kNativeSlotsStashHalfword, base, stash);
    const auto keep = buf.end;
    emit_cbz(buf, 0, 0, stash, 0);  // CBZ Wstash, .done
    emit_fpcr_restore(buf, fpcr, stash);
    patch_forward(buf, keep);
    tr.free_gpr_mask = saved_mask;
}

void emit_copy_f80(AssemblerBuffer& buf, int Xdst, int dst_off, int Xsrc, int src_off, int Xt1,
                   int Xt2) {
    emit_ldur_stur(buf, 3, /*LDUR*/ 1, static_cast<int16_t>(src_off), Xsrc, Xt1);
    emit_ldur_stur(buf, 1, /*LDURH*/ 1, static_cast<int16_t>(src_off + 8), Xsrc, Xt2);
    emit_ldur_stur(buf, 3, /*STUR*/ 0, static_cast<int16_t>(dst_off), Xdst, Xt1);
    emit_ldur_stur(buf, 1, /*STURH*/ 0, static_cast<int16_t>(dst_off + 8), Xdst, Xt2);
}

// 6 + 10 * phys as two shifted adds and an immediate, the same address the
// boundary's LDUR/STR use for a constant slot.
void emit_native_slot_addr(AssemblerBuffer& buf, int Xaddr, int Xbase, int Wphys) {
    emit_add_sub_shifted_reg(buf, 1, 0, 0, 0 /*LSL*/, Wphys, 3, Xbase, Xaddr);
    emit_add_sub_shifted_reg(buf, 1, 0, 0, 0 /*LSL*/, Wphys, 1, Xaddr, Xaddr);
    emit_add_imm(buf, 1, 0, 0, 0, 6, Xaddr, Xaddr);
}

// fild: every int64 fits the 64-bit significand, so the conversion is a
// normalisation.  |v| is formed without flags (v XOR s minus s, s = v >> 63
// arithmetic); INT64_MIN gives 2^63, which is right as an unsigned
// magnitude.  The biased exponent is 16383 + 63 - clz(|v|), with the sign
// at bit 15.  Zero is stored as +0.
void emit_i64_to_f80_slot(AssemblerBuffer& buf, int Xslot, int Xint, int Xs, int Xe) {
    emit_bitfield(buf, 1, 0 /*SBFM*/, 1, 63, 63, Xint, Xs);                // ASR Xs, Xint, #63
    emit_logical_shifted_reg(buf, 1, 2 /*EOR*/, 0, 0, Xs, 0, Xint, Xint);  // EOR Xint, Xint, Xs
    emit_add_sub_shifted_reg(buf, 1, 1, 0, 0, Xs, 0, Xint, Xint);          // SUB Xint, Xint, Xs
    const auto zero = emit_cbz_fwd(buf, 1, 0, Xint);
    buf.emit(0xdac01000U | (static_cast<uint32_t>(Xint) << 5) | Xe);  // CLZ Xe, Xint
    emit_lslv(buf, 1, Xe, Xint, Xint);
    LogicalImmEncoding sign;
    is_bitmask_immediate(false, 0x8000, sign);
    emit_and_imm(buf, 0, Xs, sign.N, sign.immr, sign.imms, Xs);
    // 16446 = 16383 + 63 does not fit one imm12: 4 << 12, then 0x3e.
    emit_add_imm(buf, 0, 0, 0, 1, 4, Xs, Xs);
    emit_add_imm(buf, 0, 0, 0, 0, 0x3e, Xs, Xs);
    emit_add_sub_shifted_reg(buf, 0, 1, 0, 0, Xe, 0, Xs, Xe);  // SUB We, Ws, We
    emit_str_imm(buf, 3, Xint, Xslot, 0);
    emit_str_imm(buf, 1, Xe, Xslot, 4);
    const auto done = emit_b_fwd(buf);
    bind_imm19(buf, zero);
    emit_str_imm(buf, 3, 31, Xslot, 0);
    emit_str_imm(buf, 1, 31, Xslot, 4);
    bind_imm26(buf, done);
}

// fist / fistp / fisttp from an f80 slot.  With e the biased exponent and
// d = e - 16446, |v| = m * 2^d:
//   d >= 0           |v| >= 2^63, infinity or NaN: indefinite.  For m64 the
//                    indefinite is INT64_MIN, so -2^63 needs no case of its
//                    own.
//   e != 0, J == 0   unnormal, an invalid operand: indefinite.
//   -64 <= d < 0     shift sh = -d in [1, 64].  integer = (m >> 1) >> (sh-1),
//                    fraction = m << (64 - sh) with the binary point above
//                    bit 63 (LSLV by d, modulo 64, is 0 for sh == 64).
//   d < -64          |v| < 1/2 (zero and denormals too): integer 0, and a
//                    fraction below one half that is nonzero iff m is.
// The magnitude is then rounded up by one when the mode asks for it, which
// can only reach 2^63, negated for a negative sign, and checked against the
// destination width.  No instruction here writes NZCV.
void emit_f80_slot_to_int(AssemblerBuffer& buf, int Xbase, int Xslot, int Xint, int Xm, int Wx,
                          int We, NativeIntRound round, int bits) {
    const int Xf = Xslot;  // the address is dead once both halves are loaded
    const bool need_frac = round != NativeIntRound::kChop;
    uint64_t to_indef[3];
    int n_indef = 0;

    emit_ldr_str_imm(buf, 3, 0, 1, 0, Xslot, Xm);  // LDR  Xm, [Xslot]
    emit_ldr_str_imm(buf, 1, 0, 1, 4, Xslot, Wx);  // LDRH Wx, [Xslot, #8]
    LogicalImmEncoding exp15;
    is_bitmask_immediate(false, 0x7fff, exp15);
    emit_and_imm(buf, 0, We, exp15.N, exp15.immr, exp15.imms, Wx);
    const auto j_set = emit_tbz_fwd(buf, 1, Xm, 63);
    to_indef[n_indef++] = emit_cbz_fwd(buf, 0, 1, We);
    bind_imm14(buf, j_set);
    emit_add_imm(buf, 0, 1, 0, 1, 4, We, We);     // d = e - 16446
    emit_add_imm(buf, 0, 1, 0, 0, 0x3e, We, We);  //
    to_indef[n_indef++] = emit_tbz_fwd(buf, 0, We, 31);
    emit_add_imm(buf, 0, 0, 0, 0, 64, We, Xint);
    const auto tiny = emit_tbz_fwd(buf, 1, Xint, 31);
    emit_add_sub_shifted_reg(buf, 0, 1, 0, 0, We, 0, 31, Xint);  // sh = -d
    emit_add_imm(buf, 0, 1, 0, 0, 1, Xint, Xint);                // sh - 1
    emit_bitfield(buf, 1, 2 /*UBFM*/, 1, 1, 63, Xm, Xf);         // LSR Xf, Xm, #1
    buf.emit(0x9ac02400U | (static_cast<uint32_t>(Xint) << 16) | (static_cast<uint32_t>(Xf) << 5) |
             static_cast<uint32_t>(Xint));  // LSRV Xint, Xf, Xint
    if (need_frac) {
        emit_lslv(buf, 1, We, Xm, Xf);
    }
    const auto general_done = emit_b_fwd(buf);
    bind_imm14(buf, tiny);
    if (need_frac) {
        LogicalImmEncoding one;
        is_bitmask_immediate(true, 1, one);
        emit_bitfield(buf, 1, 2 /*UBFM*/, 1, 1, 63, Xm, Xf);  // LSR Xf, Xm, #1
        emit_and_imm(buf, 1, Xint, one.N, one.immr, one.imms, Xm);
        emit_logical_shifted_reg(buf, 1, 1 /*ORR*/, 0, 0, Xint, 0, Xf, Xf);
    }
    emit_movn(buf, 1, 2 /*MOVZ*/, 0, 0, Xint);
    bind_imm26(buf, general_done);

    if (round == NativeIntRound::kNearest || round == NativeIntRound::kControlWord) {
        const int Wc = We;
        uint64_t to_rounded[6];
        uint64_t to_inc[2];
        int n_rounded = 0;
        int n_inc = 0;
        uint64_t up_or_chop = 0;
        uint64_t down = 0;
        if (round == NativeIntRound::kControlWord) {
            emit_ldr_str_imm(buf, 1, 0, 1, 0, Xbase, Wc);  // LDRH Wc, [Xbase] (control word)
            up_or_chop = emit_tbz_fwd(buf, 1, Wc, 11);
            down = emit_tbz_fwd(buf, 1, Wc, 10);
        }
        // Nearest even: up when the fraction is above one half, or exactly
        // one half with an odd integer.
        to_rounded[n_rounded++] = emit_tbz_fwd(buf, 0, Xf, 63);
        emit_bitfield(buf, 1, 2 /*UBFM*/, 1, 63, 62, Xf, Xm);  // LSL Xm, Xf, #1
        to_inc[n_inc++] = emit_cbz_fwd(buf, 1, 1, Xm);
        to_rounded[n_rounded++] = emit_tbz_fwd(buf, 0, Xint, 0);
        if (round == NativeIntRound::kControlWord) {
            to_inc[n_inc++] = emit_b_fwd(buf);
            // RC=10 up, RC=11 chop.  Up raises the magnitude of a positive
            // value with a fraction; down that of a negative one.
            bind_imm14(buf, up_or_chop);
            to_rounded[n_rounded++] = emit_tbz_fwd(buf, 1, Wc, 10);
            to_rounded[n_rounded++] = emit_tbz_fwd(buf, 1, Wx, 15);
            const auto any_frac = emit_b_fwd(buf);
            bind_imm14(buf, down);
            to_rounded[n_rounded++] = emit_tbz_fwd(buf, 0, Wx, 15);
            bind_imm26(buf, any_frac);
            to_rounded[n_rounded++] = emit_cbz_fwd(buf, 1, 0, Xf);
        }
        for (int i = 0; i < n_inc; i++) {
            if ((buf.data[to_inc[i] / 4] & 0xFC000000U) == 0x14000000U) {
                bind_imm26(buf, to_inc[i]);
            } else {
                bind_imm19(buf, to_inc[i]);
            }
        }
        emit_add_imm(buf, 1, 0, 0, 0, 1, Xint, Xint);
        for (int i = 0; i < n_rounded; i++) {
            if ((buf.data[to_rounded[i] / 4] & 0x7E000000U) == 0x36000000U) {
                bind_imm14(buf, to_rounded[i]);
            } else {
                bind_imm19(buf, to_rounded[i]);
            }
        }
    }

    const auto positive = emit_tbz_fwd(buf, 0, Wx, 15);
    emit_add_sub_shifted_reg(buf, 1, 1, 0, 0, Xint, 0, 31, Xint);  // NEG Xint, Xint
    bind_imm14(buf, positive);
    uint64_t to_store;
    if (bits == 64) {
        // The magnitude is at most 2^63, whose bit pattern is the
        // indefinite either way.
        to_store = emit_b_fwd(buf);
    } else {
        // In range iff the value equals the sign extension of its low half.
        emit_add_sub_ext_reg(buf, 1, 1, 0, bits == 16 ? 5 /*SXTH*/ : 6 /*SXTW*/, 0, Xint, Xint, Xm);
        to_store = emit_cbz_fwd(buf, 1, 0, Xm);
    }
    for (int i = 0; i < n_indef; i++) {
        if ((buf.data[to_indef[i] / 4] & 0x7E000000U) == 0x36000000U) {
            bind_imm14(buf, to_indef[i]);
        } else {
            bind_imm19(buf, to_indef[i]);
        }
    }
    if (bits == 16) {
        emit_movn(buf, 0, 2 /*MOVZ*/, 0, 0x8000, Xint);
    } else if (bits == 32) {
        emit_movn(buf, 0, 2 /*MOVZ*/, 1, 0x8000, Xint);
    } else {
        emit_movn(buf, 1, 2 /*MOVZ*/, 3, 0x8000, Xint);
    }
    if (bits == 64) {
        bind_imm26(buf, to_store);
    } else {
        bind_imm19(buf, to_store);
    }
}

}  // namespace TranslatorX87
