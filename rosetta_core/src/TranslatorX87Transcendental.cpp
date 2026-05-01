#include "rosetta_core/TranslatorX87Transcendental.hpp"

#include <cassert>
#include <cstddef>

#include "rosetta_core/AssemblerHelpers.hpp"
#include "rosetta_core/TranscendentalHelper.h"
#include "rosetta_core/TranslatorX87Helpers.hpp"
#include "TranslatorX87Internal.hpp"

namespace TranslatorX87 {

namespace {

// Per-field offsets into TranscendentalConstants, expressed as the
// imm12 used by `LDR Dt, [Xconst, #imm12*8]` (so byte-offset/8).  Pinned
// by static_asserts below; updating the struct without updating these
// is a build error.
struct ConstOff {
    // Trig (sin/cos)
    static constexpr int InvPi    = 0;
    static constexpr int Pi1      = 1;
    static constexpr int Pi2      = 2;
    static constexpr int Pi3      = 3;
    static constexpr int SinC0    = 4;
    static constexpr int SinC1    = 5;
    static constexpr int SinC2    = 6;
    static constexpr int SinC3    = 7;
    static constexpr int SinC4    = 8;
    static constexpr int SinC5    = 9;
    static constexpr int SinC6    = 10;
    static constexpr int RangeVal = 11;
    static constexpr int Half     = 12;
    // f2xm1 (exp2m1)
    static constexpr int Exp2m1Log2Hi    = 13;
    static constexpr int Exp2m1Log2Lo    = 14;
    static constexpr int Exp2m1C1        = 15;
    static constexpr int Exp2m1C2        = 16;
    static constexpr int Exp2m1C3        = 17;
    static constexpr int Exp2m1C4        = 18;
    static constexpr int Exp2m1C5        = 19;
    static constexpr int Exp2m1C6        = 20;
    static constexpr int Exp2m1Shift     = 21;
    static constexpr int Exp2m1Rnd2Zero  = 22;
    static constexpr int Exp2m1TableBound= 23;
    static constexpr int One             = 24;
    // Tables (byte offsets, used directly with imm12+reg-offset LDR)
    static constexpr int ExpTableByteOff   = 25 * 8;     // 200
    static constexpr int ExpScalem1ByteOff = (25 + 128) * 8;  // 1224
    // log2 / fyl2x / fyl2xp1
    static constexpr int Log2Off          = 25 + 128 + 88 + 0;  // = 241
    static constexpr int Log2SignExpMask  = 25 + 128 + 88 + 1;  // = 242
    static constexpr int Log2InvLn2       = 25 + 128 + 88 + 2;  // = 243
    static constexpr int Log2C0           = 25 + 128 + 88 + 3;  // = 244
    static constexpr int Log2C1           = 25 + 128 + 88 + 4;  // = 245
    static constexpr int Log2C2           = 25 + 128 + 88 + 5;  // = 246
    static constexpr int Log2C3           = 25 + 128 + 88 + 6;  // = 247
    static constexpr int Log2C4           = 25 + 128 + 88 + 7;  // = 248
    static constexpr int Log2InvcByteOff   = (25 + 128 + 88 + 8) * 8;  // 1992
    static constexpr int Log2Log2cByteOff  = (25 + 128 + 88 + 8 + 128) * 8;  // 3016
    // fpatan / atan2 (after log2_log2c, double offset 25+128+88+8+128+128=505)
    static constexpr int Atan2NegTwo       = 505;
    static constexpr int Atan2PiOver2      = 506;
    static constexpr int Atan2C0           = 507;  // c0..c19 contiguous → C(i) = 507 + i
    // fptan (after atan2_c[20])
    static constexpr int TanTwoOverPi      = 527;
    static constexpr int TanHalfPiHi       = 528;
    static constexpr int TanHalfPiLo       = 529;
    static constexpr int TanShift          = 530;
    static constexpr int TanPoly0          = 531;  // poly[0..8] contiguous → P(i) = 531 + i
};

static_assert(offsetof(rosetta_core::TranscendentalConstants, inv_pi)            == ConstOff::InvPi          * 8, "ConstOff::InvPi drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, pi_1)              == ConstOff::Pi1            * 8, "ConstOff::Pi1 drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, pi_2)              == ConstOff::Pi2            * 8, "ConstOff::Pi2 drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, pi_3)              == ConstOff::Pi3            * 8, "ConstOff::Pi3 drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, sin_c)             == ConstOff::SinC0          * 8, "ConstOff::SinC0 drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, range_val)         == ConstOff::RangeVal       * 8, "ConstOff::RangeVal drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, half)              == ConstOff::Half           * 8, "ConstOff::Half drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, exp2m1_log2_hi)    == ConstOff::Exp2m1Log2Hi   * 8, "ConstOff::Exp2m1Log2Hi drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, exp2m1_log2_lo)    == ConstOff::Exp2m1Log2Lo   * 8, "ConstOff::Exp2m1Log2Lo drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, exp2m1_c1)         == ConstOff::Exp2m1C1       * 8, "ConstOff::Exp2m1C1 drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, exp2m1_c6)         == ConstOff::Exp2m1C6       * 8, "ConstOff::Exp2m1C6 drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, exp2m1_shift)      == ConstOff::Exp2m1Shift    * 8, "ConstOff::Exp2m1Shift drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, exp2m1_rnd2zero)   == ConstOff::Exp2m1Rnd2Zero * 8, "ConstOff::Exp2m1Rnd2Zero drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, exp2m1_tablebound) == ConstOff::Exp2m1TableBound * 8, "ConstOff::Exp2m1TableBound drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, one)               == ConstOff::One            * 8, "ConstOff::One drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, exp_table)         == ConstOff::ExpTableByteOff,     "ConstOff::ExpTableByteOff drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, exp_scalem1)       == ConstOff::ExpScalem1ByteOff,   "ConstOff::ExpScalem1ByteOff drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, log2_off)          == ConstOff::Log2Off          * 8, "ConstOff::Log2Off drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, log2_sign_exp_mask)== ConstOff::Log2SignExpMask  * 8, "ConstOff::Log2SignExpMask drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, log2_invln2)       == ConstOff::Log2InvLn2       * 8, "ConstOff::Log2InvLn2 drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, log2_c0)           == ConstOff::Log2C0           * 8, "ConstOff::Log2C0 drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, log2_c4)           == ConstOff::Log2C4           * 8, "ConstOff::Log2C4 drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, log2_invc)         == ConstOff::Log2InvcByteOff,     "ConstOff::Log2InvcByteOff drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, log2_log2c)        == ConstOff::Log2Log2cByteOff,    "ConstOff::Log2Log2cByteOff drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, atan2_neg_two)     == ConstOff::Atan2NegTwo      * 8, "ConstOff::Atan2NegTwo drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, atan2_pi_over_2)   == ConstOff::Atan2PiOver2     * 8, "ConstOff::Atan2PiOver2 drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, atan2_c)           == ConstOff::Atan2C0          * 8, "ConstOff::Atan2C0 drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, tan_two_over_pi)   == ConstOff::TanTwoOverPi     * 8, "ConstOff::TanTwoOverPi drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, tan_half_pi_hi)    == ConstOff::TanHalfPiHi      * 8, "ConstOff::TanHalfPiHi drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, tan_half_pi_lo)    == ConstOff::TanHalfPiLo      * 8, "ConstOff::TanHalfPiLo drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, tan_shift)         == ConstOff::TanShift         * 8, "ConstOff::TanShift drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, tan_poly)          == ConstOff::TanPoly0         * 8, "ConstOff::TanPoly0 drift");

enum class TrigReduceMode { Sin, Cos };

// 3-step Cody-Waite range reduction shared by fsin / fcos / fsincos.
//
// Sin mode:  n  = round(x * inv_pi);              qn = (s64) n
// Cos mode:  n' = round(x * inv_pi + 0.5);        qn = (s64) n'   (pre-offset)
//            n  = n' - 0.5
//
// Both then compute  r = x - n*pi_1 - n*pi_2 - n*pi_3   into Dr.
//
// Caller pre-allocates Dn, Xqn, Dr, Dtmp from the scratch pools and
// retains ownership.  On return Dn is dead; Xqn holds the sign-flip
// qn; Dr holds the reduced argument.
void emit_trig_range_reduce(AssemblerBuffer& buf, int Xconst, int Dx,
                             int Dn, int Xqn, int Dr, int Dtmp,
                             TrigReduceMode mode) {
    emit_fldr_imm(buf, /*size=*/3, Dn, Xconst, ConstOff::InvPi);

    if (mode == TrigReduceMode::Sin) {
        emit_fmul_f64(buf, Dn, Dx, Dn);                          // n = x*inv_pi
    } else {
        emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::Half);
        emit_fmadd_f64(buf, Dn, Dx, Dn, Dtmp);                   // n = x*inv_pi + 0.5
    }

    emit_frinta_f64(buf, Dn, Dn);                                // n = round(n)
    emit_fcvtzs(buf, /*ftype=*/1, /*is_64bit_int=*/1, Xqn, Dn);  // qn = (s64) n

    if (mode == TrigReduceMode::Cos) {
        emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::Half);
        emit_fsub_f64(buf, Dn, Dn, Dtmp);                        // n -= 0.5 (post-qn)
    }

    emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::Pi1);
    emit_fmsub_f64(buf, Dr, Dn, Dtmp, Dx);                       // r  = x - n*pi_1
    emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::Pi2);
    emit_fmsub_f64(buf, Dr, Dn, Dtmp, Dr);                       // r -= n*pi_2
    emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::Pi3);
    emit_fmsub_f64(buf, Dr, Dn, Dtmp, Dr);                       // r -= n*pi_3
}

// Order-6 Estrin polynomial in r²: y = r + r³ * P(r²) where the
// coefficients live at Xconst[ConstOff::SinC0..SinC6].  This is the
// `sin(r)` approximation from advsimd/sin.c (and reused by cos.c, which
// lists identical c0..c6 coefficients).
//
// Allocates 4 scratch FPRs (Dr2, Dr4, Dp2, Dp3) plus reuses Dy_out
// across the chain.  Dr (the reduced argument) survives — caller keeps
// ownership and must free it.
void emit_sin_poly_estrin(TranslationResult& a1, AssemblerBuffer& buf,
                           int Dy_out, int Dr, int Xconst) {
    const int Dr2  = alloc_free_fpr(a1);
    const int Dr4  = alloc_free_fpr(a1);
    const int Dp2  = alloc_free_fpr(a1);
    const int Dp3  = alloc_free_fpr(a1);
    const int Dtmp = alloc_free_fpr(a1);

    emit_fmul_f64(buf, Dr2, Dr, Dr);                             // r2 = r*r
    emit_fmul_f64(buf, Dr4, Dr2, Dr2);                           // r4 = r2*r2

    // p01 = c0 + r2 * c1
    emit_fldr_imm(buf, 3, Dy_out, Xconst, ConstOff::SinC0);
    emit_fldr_imm(buf, 3, Dtmp,   Xconst, ConstOff::SinC1);
    emit_fmadd_f64(buf, Dy_out, Dr2, Dtmp, Dy_out);

    // p23 = c2 + r2 * c3
    emit_fldr_imm(buf, 3, Dp2,  Xconst, ConstOff::SinC2);
    emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::SinC3);
    emit_fmadd_f64(buf, Dp2, Dr2, Dtmp, Dp2);

    // p45 = c4 + r2 * c5
    emit_fldr_imm(buf, 3, Dp3,  Xconst, ConstOff::SinC4);
    emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::SinC5);
    emit_fmadd_f64(buf, Dp3, Dr2, Dtmp, Dp3);

    // p46 = p45 + r4 * c6
    emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::SinC6);
    emit_fmadd_f64(buf, Dp3, Dr4, Dtmp, Dp3);
    free_fpr(a1, Dtmp);

    // p26 = p23 + r4 * p46
    emit_fmadd_f64(buf, Dp2, Dr4, Dp3, Dp2);
    free_fpr(a1, Dp3);

    // p06 = p01 + r4 * p26   (Dy_out := p06)
    emit_fmadd_f64(buf, Dy_out, Dr4, Dp2, Dy_out);
    free_fpr(a1, Dp2);
    free_fpr(a1, Dr4);

    // r3 = r2 * r  (reuse Dr2's slot)
    emit_fmul_f64(buf, Dr2, Dr2, Dr);                            // Dr2 := r3

    // y = r + r3 * p06   (Dy_out := y)
    emit_fmadd_f64(buf, Dy_out, Dr2, Dy_out, Dr);
    free_fpr(a1, Dr2);
}

// Final sign flip for sin/cos: result_bits = bits(Dy_in) ^ (Xqn << 63).
// Result lands in Dd_out.  Allocates and frees one GPR scratch.
void emit_apply_qn_sign(TranslationResult& a1, AssemblerBuffer& buf,
                         int Dy_in, int Xqn, int Dd_out) {
    const int Xy = alloc_free_gpr(a1);
    emit_fmov_d_to_x(buf, Xy, Dy_in);
    emit_logical_shifted_reg(buf, /*is_64=*/1, /*EOR=*/2, /*n=*/0,
                             /*shift_type=LSL*/0, /*Rm=*/Xqn,
                             /*shift_amount=*/63, /*Rn=*/Xy, /*Rd=*/Xy);
    emit_fmov_x_to_d(buf, Dd_out, Xy);
    free_gpr(a1, Xy);
}

// Body of fsin/fcos given a pre-loaded Dx and a pre-materialised Xconst.
// Runs Cody-Waite reduction in `mode`, evaluates the sin polynomial,
// applies the qn sign flip.  Result lands in Dd_out (caller-specified,
// must not overlap Dx or any FPR in kFprScratchPool that's currently
// live in the cache).  Caller still owns Dx and Xconst — this function
// neither frees nor clobbers them (other than Dx surviving the read).
void emit_inline_trig_body(TranslationResult& a1, AssemblerBuffer& buf,
                            int Dx, int Xconst, int Dd_out,
                            TrigReduceMode mode) {
    // 1. Range reduction → Dr, Xqn.
    const int Dn   = alloc_free_fpr(a1);
    const int Xqn  = alloc_free_gpr(a1);
    const int Dr   = alloc_free_fpr(a1);
    const int Dtmp = alloc_free_fpr(a1);
    emit_trig_range_reduce(buf, Xconst, Dx, Dn, Xqn, Dr, Dtmp, mode);
    free_fpr(a1, Dtmp);
    free_fpr(a1, Dn);

    // 2. y = r + r³ · P(r²).
    const int Dy = alloc_free_fpr(a1);
    emit_sin_poly_estrin(a1, buf, Dy, Dr, Xconst);
    free_fpr(a1, Dr);

    // 3. Sign flip: y ^= qn << 63;  result lands in Dd_out.
    emit_apply_qn_sign(a1, buf, Dy, Xqn, Dd_out);
    free_gpr(a1, Xqn);
    free_fpr(a1, Dy);
}

// Single-output wrapper used by fsin / fcos.  Loads ST(0), materialises
// Xconst, runs one trig body, then frees.  Result lands in d0 (matching
// the IPC convention so the caller's emit_store_st(..., Dd=0, ...)
// Just Works).
void emit_inline_sin_or_cos(TranslationResult& a1, AssemblerBuffer& buf,
                             int Xbase, int Wd_top, int Wd_tmp,
                             TrigReduceMode mode) {
    const int Xst_base  = x87_get_st_base(a1);
    const int depth_st0 = resolve_depth(a1, 0);

    const int Dx = alloc_free_fpr(a1);
    emit_load_st(buf, Xbase, Wd_top, depth_st0, Wd_tmp, Dx, Xst_base);

    const int Xconst = alloc_free_gpr(a1);
    const uint64_t consts_addr = rosetta_core::get_transcendental_constants_addr();
    assert(consts_addr != 0 && "transcendental constants not installed; loader should have set address");
    emit_movz_movk_abs64(buf, Xconst, consts_addr);

    emit_inline_trig_body(a1, buf, Dx, Xconst, /*Dd_out=*/0, mode);

    free_gpr(a1, Xconst);
    free_fpr(a1, Dx);
}

}  // namespace

void emit_inline_fsin(TranslationResult& a1, AssemblerBuffer& buf,
                      int Xbase, int Wd_top, int Wd_tmp) {
    emit_inline_sin_or_cos(a1, buf, Xbase, Wd_top, Wd_tmp,
                           TrigReduceMode::Sin);
}

void emit_inline_fcos(TranslationResult& a1, AssemblerBuffer& buf,
                      int Xbase, int Wd_top, int Wd_tmp) {
    emit_inline_sin_or_cos(a1, buf, Xbase, Wd_top, Wd_tmp,
                           TrigReduceMode::Cos);
}

// f2xm1 — replace ST(0) with 2^ST(0) - 1.  Port of optimized-routines'
// AdvSIMD `exp2m1` (math/aarch64/advsimd/exp2m1.c) scalarised to ARM64.
//
// Algorithm:
//   z = x + shift; n = z - shift           (round x to k/N, N=128)
//   u = bits(z)
//   r = x - n
//   scale = bits_to_double(exp_table[u & 127] + (u << 45))
//   poly  = r·log2_hi + r·log2_lo + r²·(c1 + r·c2 + r²·(c3 + r·c4 + r²·(c5 + r·c6)))
//   if (|x| < TableBound):
//       idx = (u & 0x3f) + (x < rnd2zero ? 24 : 0)
//       scalem1 = scalem1_table[idx]
//   else:
//       scalem1 = scale - 1
//   y = scalem1 + scale * poly
//
// Spec input is |x| <= 1; out-of-range special-case (|x| > ~1023) is
// not emitted because x87 leaves that range undefined.
void emit_inline_f2xm1(TranslationResult& a1, AssemblerBuffer& buf,
                       int Xbase, int Wd_top, int Wd_tmp) {
    const int Xst_base  = x87_get_st_base(a1);
    const int depth_st0 = resolve_depth(a1, 0);

    // 1. Load Dx = ST(0)
    const int Dx = alloc_free_fpr(a1);
    emit_load_st(buf, Xbase, Wd_top, depth_st0, Wd_tmp, Dx, Xst_base);

    // 2. Materialise Xconst = &TranscendentalConstants
    const int Xconst = alloc_free_gpr(a1);
    const uint64_t consts_addr = rosetta_core::get_transcendental_constants_addr();
    assert(consts_addr != 0 && "transcendental constants not installed");
    emit_movz_movk_abs64(buf, Xconst, consts_addr);

    // 3. Range reduction via shift trick: z = x + shift; n = z - shift; u = bits(z); r = x - n
    const int Dz_then_n = alloc_free_fpr(a1);
    const int Dshift    = alloc_free_fpr(a1);
    emit_fldr_imm(buf, 3, Dshift, Xconst, ConstOff::Exp2m1Shift);
    emit_fadd_f64(buf, Dz_then_n, Dx, Dshift);                  // Dz_then_n := z = x + shift

    const int Xu = alloc_free_gpr(a1);
    emit_fmov_d_to_x(buf, Xu, Dz_then_n);                       // Xu = bits(z)

    emit_fsub_f64(buf, Dz_then_n, Dz_then_n, Dshift);           // Dz_then_n := n = z - shift
    free_fpr(a1, Dshift);

    const int Dr = alloc_free_fpr(a1);
    emit_fsub_f64(buf, Dr, Dx, Dz_then_n);                      // Dr = r = x - n
    free_fpr(a1, Dz_then_n);

    // 4. r2 = r*r
    const int Dr2 = alloc_free_fpr(a1);
    emit_fmul_f64(buf, Dr2, Dr, Dr);

    // 5. Polynomial in r,r2:
    //    p56 = c5 + r*c6
    //    p36 = c3 + r*c4 + r²·p56
    //    p16 = c1 + r*c2 + r²·p36
    //    poly_lin = r·log2_hi + r·log2_lo
    //    poly = poly_lin + r²·p16
    {
        const int Dp_a = alloc_free_fpr(a1);   // accumulator (p56 → p36 → p16 → poly)
        const int Dtmp = alloc_free_fpr(a1);

        // p56 = c5 + r*c6
        emit_fldr_imm(buf, 3, Dp_a, Xconst, ConstOff::Exp2m1C5);
        emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::Exp2m1C6);
        emit_fmadd_f64(buf, Dp_a, Dr, Dtmp, Dp_a);

        // p36 = (c3 + r*c4) + r²·p56
        const int Dp_b = alloc_free_fpr(a1);
        emit_fldr_imm(buf, 3, Dp_b, Xconst, ConstOff::Exp2m1C3);
        emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::Exp2m1C4);
        emit_fmadd_f64(buf, Dp_b, Dr, Dtmp, Dp_b);              // c3 + r*c4
        emit_fmadd_f64(buf, Dp_a, Dr2, Dp_a, Dp_b);             // p36 = (c3 + r*c4) + r²·p56  (Dp_a := p36)
        free_fpr(a1, Dp_b);

        // p16 = (c1 + r*c2) + r²·p36
        const int Dp_c = alloc_free_fpr(a1);
        emit_fldr_imm(buf, 3, Dp_c, Xconst, ConstOff::Exp2m1C1);
        emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::Exp2m1C2);
        emit_fmadd_f64(buf, Dp_c, Dr, Dtmp, Dp_c);              // c1 + r*c2
        emit_fmadd_f64(buf, Dp_a, Dr2, Dp_a, Dp_c);             // p16 = (c1 + r*c2) + r²·p36
        free_fpr(a1, Dp_c);

        // poly_lin = r*log2_hi + r*log2_lo  (in Dtmp)
        emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::Exp2m1Log2Hi);
        emit_fmul_f64(buf, Dtmp, Dr, Dtmp);
        const int Dt2 = alloc_free_fpr(a1);
        emit_fldr_imm(buf, 3, Dt2, Xconst, ConstOff::Exp2m1Log2Lo);
        emit_fmadd_f64(buf, Dtmp, Dr, Dt2, Dtmp);               // Dtmp := r·(log2_hi+log2_lo)
        free_fpr(a1, Dt2);

        // poly = poly_lin + r²·p16
        emit_fmadd_f64(buf, Dp_a, Dr2, Dp_a, Dtmp);             // Dp_a := poly
        free_fpr(a1, Dtmp);

        // Dp_a now holds `poly`; remaining lifetime extends to step 9.
        // Stash it via a stable name by leaking out of the block — caller
        // tracks it as Dpoly below.
        // (Just rename: from here on, the holder of "poly" is Dp_a.)
        // We free Dr2/Dr after step 9's lookup.
        // Dpoly = Dp_a  (alias, no extra register).
        // Re-bind:
        // ... (using Dp_a as Dpoly below)
        // We need to free Dr2 and Dr before step 9 to free FPR slots.
        free_fpr(a1, Dr2);
        free_fpr(a1, Dr);

        // 6. scale = bits_to_double(exp_table[u & 127] + (u << 45))
        //    GPR plan:
        //      Xtmp1 = u & 0x7f
        //      Xtmp2 = Xconst + ExpTableByteOff
        //      Xtmp1 = LDR [Xtmp2, Xtmp1, LSL #3]      ; scale_bits
        //      free Xtmp2
        //      Xtmp1 += u << 45                         ; ADD shifted-reg
        //      Dscale = bits_to_double(Xtmp1)
        //      free Xtmp1
        const int Xtmp1 = alloc_free_gpr(a1);
        emit_and_imm(buf, /*is_64=*/1, Xtmp1, /*N=*/1, /*immr=*/0, /*imms=*/6, Xu);  // Xtmp1 = Xu & 0x7f
        const int Xtmp2 = alloc_free_gpr(a1);
        emit_add_imm(buf, /*is_64=*/1, /*is_sub=*/0, /*is_set_flags=*/0, /*shift=*/0,
                     /*imm12=*/ConstOff::ExpTableByteOff, Xconst, Xtmp2);            // Xtmp2 = Xconst + ExpTableByteOff
        emit_ldr_str_reg(buf, /*size=*/3, /*is_fp=*/0, /*opc=LDR*/1, /*Rm=*/Xtmp1,
                         /*shift=*/1, /*Rn=*/Xtmp2, /*Rt=*/Xtmp1);                   // LDR Xtmp1, [Xtmp2, Xtmp1, LSL #3]
        free_gpr(a1, Xtmp2);
        emit_add_sub_shifted_reg(buf, /*is_64=*/1, /*is_sub=*/0, /*is_set_flags=*/0,
                                  /*shift_type=LSL*/0, /*Rm=*/Xu,
                                  /*shift_amount=*/45, /*Rn=*/Xtmp1, /*Rd=*/Xtmp1);  // Xtmp1 += Xu << 45
        const int Dscale = alloc_free_fpr(a1);
        emit_fmov_x_to_d(buf, Dscale, Xtmp1);
        free_gpr(a1, Xtmp1);

        // 7. scalem1_a = scale - 1.0
        const int Dscalem1_a = alloc_free_fpr(a1);
        const int Dtmp2 = alloc_free_fpr(a1);
        emit_fldr_imm(buf, 3, Dtmp2, Xconst, ConstOff::One);
        emit_fsub_f64(buf, Dscalem1_a, Dscale, Dtmp2);
        // (Dtmp2 reused below for FCMP.)

        // 8. is_neg = x < rnd2zero
        emit_fldr_imm(buf, 3, Dtmp2, Xconst, ConstOff::Exp2m1Rnd2Zero);
        emit_fcmp_f64(buf, Dx, Dtmp2);
        free_fpr(a1, Dtmp2);

        //    idx = (u & 0x3f) + 24·is_neg
        const int Xidx = alloc_free_gpr(a1);
        emit_and_imm(buf, /*is_64=*/1, Xidx, /*N=*/1, /*immr=*/0, /*imms=*/5, Xu);   // Xidx = Xu & 0x3f
        free_gpr(a1, Xu);

        const int Xneg = alloc_free_gpr(a1);
        emit_cset(buf, /*is_64=*/1, /*cond=LT*/11, Xneg);                            // Xneg = (x < rnd2zero) ? 1 : 0
        emit_add_sub_shifted_reg(buf, 1, 0, 0, /*LSL*/0, Xneg, /*amount=*/4, Xidx, Xidx);  // Xidx += 16·neg
        emit_add_sub_shifted_reg(buf, 1, 0, 0, /*LSL*/0, Xneg, /*amount=*/3, Xidx, Xidx);  // Xidx +=  8·neg → +24·neg
        free_gpr(a1, Xneg);

        // 9. scalem1_b = scalem1_table[idx]
        const int Xtbl = alloc_free_gpr(a1);
        emit_add_imm(buf, 1, 0, 0, 0, ConstOff::ExpScalem1ByteOff, Xconst, Xtbl);
        const int Dscalem1_b = alloc_free_fpr(a1);
        emit_fldr_reg(buf, /*size=*/3, Dscalem1_b, Xtbl, Xidx, /*shift=*/1);         // LDR Dscalem1_b, [Xtbl, Xidx, LSL #3]
        free_gpr(a1, Xtbl);
        free_gpr(a1, Xidx);

        // 10. is_small = |x| < TableBound;  scalem1 = is_small ? Dscalem1_b : Dscalem1_a
        const int Dabs = alloc_free_fpr(a1);
        emit_fabs_f64(buf, Dabs, Dx);
        const int Dbound = alloc_free_fpr(a1);
        emit_fldr_imm(buf, 3, Dbound, Xconst, ConstOff::Exp2m1TableBound);
        emit_fcmp_f64(buf, Dabs, Dbound);
        free_fpr(a1, Dbound);
        free_fpr(a1, Dabs);

        const int Dscalem1 = alloc_free_fpr(a1);
        emit_fcsel_f64(buf, Dscalem1, Dscalem1_b, Dscalem1_a, /*cond=LT*/11);
        free_fpr(a1, Dscalem1_b);
        free_fpr(a1, Dscalem1_a);

        // 11. y = scalem1 + poly·scale  →  d0
        emit_fmadd_f64(buf, /*Dd=*/0, /*Dn=*/Dp_a, /*Dm=*/Dscale, /*Da=*/Dscalem1);

        free_fpr(a1, Dscalem1);
        free_fpr(a1, Dscale);
        free_fpr(a1, Dp_a);
    }

    free_gpr(a1, Xconst);
    free_fpr(a1, Dx);
}

// Body of inline_log2: given Din (a positive double, 0/inf/NaN excluded
// per x87 spec) and a materialised Xconst, computes log2(Din) into
// Dd_out.  Port of optimized-routines' AdvSIMD inline_log2
// (math/aarch64/advsimd/log2.c).
//
// Allocates internal scratch.  Caller still owns Din and Xconst — this
// function neither frees nor modifies them.
//
// GPR pressure: peaks at 4 scratch (Xu, Xu_off, Xtmp, plus Xconst).
// FPR pressure: peaks at ~6 (Dz, Dr, Dr2, Dinvc, Dlog2c, Dkd, Dhi, Dy_poly).
void emit_inline_log2(TranslationResult& a1, AssemblerBuffer& buf,
                       int Din, int Xconst, int Dd_out) {
    // 1. u = bits(Din);  u_off = u - off
    const int Xu = alloc_free_gpr(a1);
    emit_fmov_d_to_x(buf, Xu, Din);

    const int Xu_off = alloc_free_gpr(a1);
    {
        const int Xtmp = alloc_free_gpr(a1);
        emit_ldr_imm(buf, /*size=*/3, Xtmp, Xconst, ConstOff::Log2Off);
        emit_subs_reg(buf, /*is_64=*/1, Xu, Xtmp, Xu_off);   // Xu_off = Xu - off
        free_gpr(a1, Xtmp);
    }

    // 2. kd = (double)((s64) u_off >> 52)
    //    Compute k in a scratch GPR, convert to FPR, then free GPR.
    const int Dkd = alloc_free_fpr(a1);
    {
        const int Xk = alloc_free_gpr(a1);
        // ASR Xk, Xu_off, #52  — encoded as SBFM Xk, Xu_off, #52, #63.
        emit_bitfield(buf, /*is_64=*/1, /*opc=SBFM*/0, /*N=*/1,
                      /*immr=*/52, /*imms=*/63, Xu_off, Xk);
        emit_scvtf_x_to_d(buf, Dkd, Xk);
        free_gpr(a1, Xk);
    }

    // 3. iz = u - (u_off & sign_exp_mask);  z = bits_to_double(iz)
    //    Compute in-place in Xu (Xu's original value is no longer needed
    //    after this sequence), then FMOV Dz, Xu and free Xu.
    {
        const int Xtmp = alloc_free_gpr(a1);
        emit_ldr_imm(buf, /*size=*/3, Xtmp, Xconst, ConstOff::Log2SignExpMask);
        emit_logical_shifted_reg(buf, /*is_64=*/1, /*AND=*/0, /*n=*/0,
                                  /*shift_type=LSL*/0, /*Rm=*/Xtmp,
                                  /*shift_amount=*/0, /*Rn=*/Xu_off, /*Rd=*/Xtmp);  // Xtmp = u_off & mask
        emit_subs_reg(buf, /*is_64=*/1, Xu, Xtmp, Xu);                              // Xu = Xu - Xtmp = iz
        free_gpr(a1, Xtmp);
    }
    const int Dz = alloc_free_fpr(a1);
    emit_fmov_x_to_d(buf, Dz, Xu);
    free_gpr(a1, Xu);

    // 4. idx = (u_off >> 45) & 0x7f
    const int Xidx = alloc_free_gpr(a1);
    // UBFM Xidx, Xu_off, #45, #51  — extracts bits 45..51 right-aligned.
    emit_bitfield(buf, /*is_64=*/1, /*opc=UBFM*/2, /*N=*/1,
                  /*immr=*/45, /*imms=*/51, Xu_off, Xidx);
    free_gpr(a1, Xu_off);

    // 5. Lookup invc[idx], log2c[idx] from the two split tables.
    const int Dinvc = alloc_free_fpr(a1);
    {
        const int Xtbl = alloc_free_gpr(a1);
        emit_add_imm(buf, /*is_64=*/1, /*is_sub=*/0, /*is_set_flags=*/0, /*shift=*/0,
                     /*imm12=*/ConstOff::Log2InvcByteOff, Xconst, Xtbl);
        emit_fldr_reg(buf, /*size=*/3, Dinvc, Xtbl, Xidx, /*shift=*/1);
        free_gpr(a1, Xtbl);
    }
    const int Dlog2c = alloc_free_fpr(a1);
    {
        const int Xtbl = alloc_free_gpr(a1);
        emit_add_imm(buf, /*is_64=*/1, /*is_sub=*/0, /*is_set_flags=*/0, /*shift=*/0,
                     /*imm12=*/ConstOff::Log2Log2cByteOff, Xconst, Xtbl);
        emit_fldr_reg(buf, /*size=*/3, Dlog2c, Xtbl, Xidx, /*shift=*/1);
        free_gpr(a1, Xtbl);
    }
    free_gpr(a1, Xidx);

    // 6. r = -1 + z * invc   (FMADD with Da = -1.0)
    const int Dr = alloc_free_fpr(a1);
    {
        const int Dneg_one = alloc_free_fpr(a1);
        emit_fmov_d_one(buf, Dneg_one);
        emit_fneg_f64(buf, Dneg_one, Dneg_one);
        emit_fmadd_f64(buf, Dr, Dz, Dinvc, Dneg_one);
        free_fpr(a1, Dneg_one);
    }
    free_fpr(a1, Dz);
    free_fpr(a1, Dinvc);

    // 7. r2 = r * r
    const int Dr2 = alloc_free_fpr(a1);
    emit_fmul_f64(buf, Dr2, Dr, Dr);

    // 8. hi = (log2c + kd) + r * invln2
    const int Dhi = alloc_free_fpr(a1);
    emit_fadd_f64(buf, Dhi, Dlog2c, Dkd);                             // Dhi = log2c + kd
    free_fpr(a1, Dlog2c);
    free_fpr(a1, Dkd);
    {
        const int Dtmp = alloc_free_fpr(a1);
        emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::Log2InvLn2);
        emit_fmadd_f64(buf, Dhi, Dr, Dtmp, Dhi);                      // Dhi += r * invln2
        free_fpr(a1, Dtmp);
    }

    // 9. Polynomial:
    //    y = c2 + r * c3
    //    p = c0 + r * c1
    //    y = y + r² * c4
    //    y = p + r² * y_prev
    const int Dy = alloc_free_fpr(a1);
    {
        const int Dtmp = alloc_free_fpr(a1);
        emit_fldr_imm(buf, 3, Dy,   Xconst, ConstOff::Log2C2);
        emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::Log2C3);
        emit_fmadd_f64(buf, Dy, Dr, Dtmp, Dy);                        // y = c2 + r*c3

        const int Dp = alloc_free_fpr(a1);
        emit_fldr_imm(buf, 3, Dp,   Xconst, ConstOff::Log2C0);
        emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::Log2C1);
        emit_fmadd_f64(buf, Dp, Dr, Dtmp, Dp);                        // p = c0 + r*c1

        emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::Log2C4);
        emit_fmadd_f64(buf, Dy, Dr2, Dtmp, Dy);                       // y += r²*c4
        emit_fmadd_f64(buf, Dy, Dr2, Dy, Dp);                         // y = p + r²*y_prev
        free_fpr(a1, Dp);
        free_fpr(a1, Dtmp);
    }
    free_fpr(a1, Dr);

    // 10. result = hi + y * r²   →  Dd_out
    emit_fmadd_f64(buf, Dd_out, Dy, Dr2, Dhi);

    free_fpr(a1, Dy);
    free_fpr(a1, Dr2);
    free_fpr(a1, Dhi);
}

void emit_inline_fyl2x(TranslationResult& a1, AssemblerBuffer& buf,
                        int Xbase, int Wd_top, int Wd_tmp) {
    // x86 fyl2x: ST(0) := ST(1) * log2(ST(0)); pop.
    // Caller (emit_2in_pop_translate) handles the writeback at depth 1
    // and the x87_pop afterwards — we just leave the result in d0.
    const int Xst_base  = x87_get_st_base(a1);
    const int depth_st0 = resolve_depth(a1, 0);
    const int depth_st1 = resolve_depth(a1, 1);

    const int Dx = alloc_free_fpr(a1);
    emit_load_st(buf, Xbase, Wd_top, depth_st0, Wd_tmp, Dx, Xst_base);

    const int Dy = alloc_free_fpr(a1);
    emit_load_st(buf, Xbase, Wd_top, depth_st1, Wd_tmp, Dy, Xst_base);

    const int Xconst = alloc_free_gpr(a1);
    const uint64_t consts_addr = rosetta_core::get_transcendental_constants_addr();
    assert(consts_addr != 0 && "transcendental constants not installed");
    emit_movz_movk_abs64(buf, Xconst, consts_addr);

    emit_inline_log2(a1, buf, Dx, Xconst, /*Dd_out=*/0);              // d0 = log2(x)
    free_fpr(a1, Dx);
    free_gpr(a1, Xconst);

    emit_fmul_f64(buf, /*Dd=*/0, /*Dn=*/0, /*Dm=*/Dy);                // d0 = y * log2(x)
    free_fpr(a1, Dy);
}

// fpatan: replace ST(1) with atan2(ST(1), ST(0)); pop.  Port of
// optimized-routines' AdvSIMD atan2 (math/aarch64/advsimd/atan2.c)
// scalarised to ARM64.  Order-19 polynomial in z² over |z| ≤ 1, with
// range reduction via numerator/denominator swap and quadrant shift:
//
//   sign_xy = (bits(x) ^ bits(y)) & sign_mask
//   ax = |x|;  ay = |y|
//   pred_aygtax = (ay > ax)
//   num = pred_aygtax ? -ax : ay
//   den = pred_aygtax ?  ay : ax
//   z = num / den
//   shift = (x<0 ? -2 : 0) + (pred_aygtax ? 1 : 0)
//   z2 = z·z;  z3 = z2·z
//   poly (Horner): start from c19, iterate poly = c[i] + z²·poly down to c0
//   ret = z + shift·(π/2) + z³·poly
//   result = bits_to_double(bits(ret) ^ sign_xy)
//
// Special cases (zero/inf/NaN) follow whatever IEEE FP arithmetic does;
// no scalar fallback as in the AdvSIMD source (real game workloads
// don't feed degenerate inputs to fpatan).
void emit_inline_fpatan(TranslationResult& a1, AssemblerBuffer& buf,
                         int Xbase, int Wd_top, int Wd_tmp) {
    const int Xst_base  = x87_get_st_base(a1);
    const int depth_st0 = resolve_depth(a1, 0);
    const int depth_st1 = resolve_depth(a1, 1);

    // 1. Load Dx = ST(0), Dy = ST(1).
    const int Dx = alloc_free_fpr(a1);
    emit_load_st(buf, Xbase, Wd_top, depth_st0, Wd_tmp, Dx, Xst_base);
    const int Dy = alloc_free_fpr(a1);
    emit_load_st(buf, Xbase, Wd_top, depth_st1, Wd_tmp, Dy, Xst_base);

    const int Xconst = alloc_free_gpr(a1);
    const uint64_t consts_addr = rosetta_core::get_transcendental_constants_addr();
    assert(consts_addr != 0 && "transcendental constants not installed");
    emit_movz_movk_abs64(buf, Xconst, consts_addr);

    // 2. sign_xy = (bits(x) ^ bits(y)) & 0x8000000000000000
    const int Xsign_xy = alloc_free_gpr(a1);
    {
        const int Xtmp = alloc_free_gpr(a1);
        emit_fmov_d_to_x(buf, Xsign_xy, Dx);
        emit_fmov_d_to_x(buf, Xtmp, Dy);
        emit_logical_shifted_reg(buf, /*is_64=*/1, /*EOR=*/2, /*n=*/0,
                                  /*shift_type=LSL*/0, /*Rm=*/Xtmp,
                                  /*shift_amount=*/0, /*Rn=*/Xsign_xy, /*Rd=*/Xsign_xy);
        // AND Xsign_xy, Xsign_xy, #0x8000000000000000  (sign bit only).
        // Bitmask immediate: N=1, immr=1, imms=0  (1 one, rotated right by 1).
        emit_and_imm(buf, /*is_64=*/1, Xsign_xy, /*N=*/1, /*immr=*/1, /*imms=*/0, Xsign_xy);
        free_gpr(a1, Xtmp);
    }

    // 3. ax = |x|, ay = |y|
    const int Dax = alloc_free_fpr(a1);
    const int Day = alloc_free_fpr(a1);
    emit_fabs_f64(buf, Dax, Dx);
    emit_fabs_f64(buf, Day, Dy);
    free_fpr(a1, Dy);
    // Dx is still needed for the sign-of-x test below — keep it.

    // 4. pred_aygtax = (ay > ax).  FCMP Day, Dax sets NZCV.
    //    Then build num, den, shift_b via FCSEL on the same flags.
    emit_fcmp_f64(buf, Day, Dax);
    const int Dneg_ax = alloc_free_fpr(a1);
    emit_fneg_f64(buf, Dneg_ax, Dax);
    const int Dnum = alloc_free_fpr(a1);
    const int Dden = alloc_free_fpr(a1);
    emit_fcsel_f64(buf, Dnum, Dneg_ax, Day, /*cond=GT*/12);
    emit_fcsel_f64(buf, Dden, Day,    Dax, /*cond=GT*/12);
    free_fpr(a1, Dneg_ax);
    free_fpr(a1, Dax);
    free_fpr(a1, Day);

    const int Dshift_b = alloc_free_fpr(a1);
    {
        const int Dzero = alloc_free_fpr(a1);
        const int Done  = alloc_free_fpr(a1);
        emit_movi_d_zero(buf, Dzero);
        emit_fmov_d_one(buf, Done);
        emit_fcsel_f64(buf, Dshift_b, Done, Dzero, /*cond=GT*/12);
        free_fpr(a1, Dzero);
        free_fpr(a1, Done);
    }

    // 5. z = num / den
    const int Dz = alloc_free_fpr(a1);
    emit_fdiv_f64(buf, Dz, Dnum, Dden);
    free_fpr(a1, Dnum);
    free_fpr(a1, Dden);

    // 6. shift_a = (x < 0) ? -2.0 : 0.0;  shift = shift_a + shift_b
    emit_fcmp_zero_f64(buf, Dx);
    const int Dshift = alloc_free_fpr(a1);
    {
        const int Dneg_two = alloc_free_fpr(a1);
        const int Dzero    = alloc_free_fpr(a1);
        emit_fldr_imm(buf, 3, Dneg_two, Xconst, ConstOff::Atan2NegTwo);
        emit_movi_d_zero(buf, Dzero);
        emit_fcsel_f64(buf, Dshift, Dneg_two, Dzero, /*cond=MI*/4);
        free_fpr(a1, Dneg_two);
        free_fpr(a1, Dzero);
    }
    emit_fadd_f64(buf, Dshift, Dshift, Dshift_b);
    free_fpr(a1, Dshift_b);
    free_fpr(a1, Dx);

    // 7. z2 = z·z, z3 = z2·z
    const int Dz2 = alloc_free_fpr(a1);
    emit_fmul_f64(buf, Dz2, Dz, Dz);
    const int Dz3 = alloc_free_fpr(a1);
    emit_fmul_f64(buf, Dz3, Dz2, Dz);

    // 8. Horner polynomial in z²: poly = c19; poly = c[i] + z²·poly for i=18..0
    const int Dpoly = alloc_free_fpr(a1);
    {
        const int Dtmp = alloc_free_fpr(a1);
        emit_fldr_imm(buf, 3, Dpoly, Xconst, ConstOff::Atan2C0 + 19);
        for (int i = 18; i >= 0; --i) {
            emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::Atan2C0 + i);
            emit_fmadd_f64(buf, Dpoly, Dz2, Dpoly, Dtmp);
        }
        free_fpr(a1, Dtmp);
    }
    free_fpr(a1, Dz2);

    // 9. ret = z + shift·(π/2) + z³·poly
    {
        const int Dpi2 = alloc_free_fpr(a1);
        emit_fldr_imm(buf, 3, Dpi2, Xconst, ConstOff::Atan2PiOver2);
        emit_fmadd_f64(buf, Dz, Dshift, Dpi2, Dz);                 // Dz := z + shift·(π/2)
        free_fpr(a1, Dpi2);
    }
    emit_fmadd_f64(buf, Dz, Dz3, Dpoly, Dz);                       // Dz += z³·poly
    free_fpr(a1, Dpoly);
    free_fpr(a1, Dz3);
    free_fpr(a1, Dshift);

    // 10. result = bits_to_double(bits(ret) ^ sign_xy) → d0
    {
        const int Xret = alloc_free_gpr(a1);
        emit_fmov_d_to_x(buf, Xret, Dz);
        emit_logical_shifted_reg(buf, /*is_64=*/1, /*EOR=*/2, /*n=*/0,
                                  /*shift_type=LSL*/0, /*Rm=*/Xsign_xy,
                                  /*shift_amount=*/0, /*Rn=*/Xret, /*Rd=*/Xret);
        emit_fmov_x_to_d(buf, /*Dd=*/0, Xret);
        free_gpr(a1, Xret);
    }
    free_gpr(a1, Xsign_xy);
    free_gpr(a1, Xconst);
    free_fpr(a1, Dz);
}

// fptan: replace ST(0) with tan(ST(0)); push 1.0.  Port of
// optimized-routines' AdvSIMD tan (math/aarch64/advsimd/tan.c).
//
// Algorithm:
//   q = round(x · 2/π) via shift trick (FMA + subtract)
//   qi = (s64) q
//   r = (x - q·π/2_hi - q·π/2_lo) / 2
//   r2 = r·r;  r4 = r2·r2;  r8 = r4·r4
//   Order-7 Estrin polynomial in r² over poly[1..8]:
//     p07 = (poly[1] + r²·poly[2]) + r⁴·(poly[3] + r²·poly[4])
//         + r⁸·((poly[5] + r²·poly[6]) + r⁴·(poly[7] + r²·poly[8]))
//   p = poly[0] + r²·p07
//   p = r + r²·(p · r)            // tan(r) approximation, |r| ≤ π/8
//   n = p² - 1;  d = 2·p
//   Recombine via tan(2x) = 2·tan(x)/(1 - tan²(x)) and
//   tan(x) = 1/tan(π/2 - x), selecting numerator/denominator on
//   parity of qi:
//     even qi (qi&1 == 0): result = -d / n  (tan(2r))
//     odd  qi (qi&1 == 1): result =  n / d  ((p²-1)/(2p) = -cot(2r))
//
// Caller (translate_fptan) then pushes 1.0 onto the stack post-tan.
void emit_inline_fptan(TranslationResult& a1, AssemblerBuffer& buf,
                        int Xbase, int Wd_top, int Wd_tmp) {
    const int Xst_base  = x87_get_st_base(a1);
    const int depth_st0 = resolve_depth(a1, 0);

    // 1. Load Dx = ST(0)
    const int Dx = alloc_free_fpr(a1);
    emit_load_st(buf, Xbase, Wd_top, depth_st0, Wd_tmp, Dx, Xst_base);

    const int Xconst = alloc_free_gpr(a1);
    const uint64_t consts_addr = rosetta_core::get_transcendental_constants_addr();
    assert(consts_addr != 0 && "transcendental constants not installed");
    emit_movz_movk_abs64(buf, Xconst, consts_addr);

    // 2. q = round(x · 2/π) via FMA shift trick
    const int Dq = alloc_free_fpr(a1);
    {
        const int Dshift = alloc_free_fpr(a1);
        const int Dtwopi = alloc_free_fpr(a1);
        emit_fldr_imm(buf, 3, Dshift, Xconst, ConstOff::TanShift);
        emit_fldr_imm(buf, 3, Dtwopi, Xconst, ConstOff::TanTwoOverPi);
        emit_fmadd_f64(buf, Dq, Dx, Dtwopi, Dshift);                  // Dq = shift + x·2/π
        emit_fsub_f64(buf, Dq, Dq, Dshift);                           // Dq -= shift  → Dq = round(x·2/π)
        free_fpr(a1, Dshift);
        free_fpr(a1, Dtwopi);
    }

    // 3. qi = (s64) Dq
    const int Xqi = alloc_free_gpr(a1);
    emit_fcvtzs(buf, /*ftype=*/1, /*is_64bit_int=*/1, Xqi, Dq);

    // 4. r = x - q·half_pi_hi - q·half_pi_lo
    const int Dr = alloc_free_fpr(a1);
    {
        const int Dtmp = alloc_free_fpr(a1);
        emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::TanHalfPiHi);
        emit_fmsub_f64(buf, Dr, Dq, Dtmp, Dx);                        // r = x - q·hpi_hi
        emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::TanHalfPiLo);
        emit_fmsub_f64(buf, Dr, Dq, Dtmp, Dr);                        // r -= q·hpi_lo
        // r *= 0.5
        emit_fldr_imm(buf, 3, Dtmp, Xconst, ConstOff::Half);
        emit_fmul_f64(buf, Dr, Dr, Dtmp);
        free_fpr(a1, Dtmp);
    }
    free_fpr(a1, Dx);
    free_fpr(a1, Dq);

    // 5. r2, r4, r8
    const int Dr2 = alloc_free_fpr(a1);
    const int Dr4 = alloc_free_fpr(a1);
    const int Dr8 = alloc_free_fpr(a1);
    emit_fmul_f64(buf, Dr2, Dr, Dr);
    emit_fmul_f64(buf, Dr4, Dr2, Dr2);
    emit_fmul_f64(buf, Dr8, Dr4, Dr4);

    // 6. Order-7 Estrin in r²:
    //    p01 = poly[1] + r²·poly[2]
    //    p23 = poly[3] + r²·poly[4]
    //    p03 = p01 + r⁴·p23
    //    p45 = poly[5] + r²·poly[6]
    //    p67 = poly[7] + r²·poly[8]
    //    p47 = p45 + r⁴·p67
    //    p07 = p03 + r⁸·p47
    const int Dp_acc = alloc_free_fpr(a1);
    {
        const int Dtmp = alloc_free_fpr(a1);
        const int Dp_b = alloc_free_fpr(a1);

        // p01 → Dp_acc
        emit_fldr_imm(buf, 3, Dp_acc, Xconst, ConstOff::TanPoly0 + 1);
        emit_fldr_imm(buf, 3, Dtmp,   Xconst, ConstOff::TanPoly0 + 2);
        emit_fmadd_f64(buf, Dp_acc, Dr2, Dtmp, Dp_acc);

        // p23 → Dp_b
        emit_fldr_imm(buf, 3, Dp_b,   Xconst, ConstOff::TanPoly0 + 3);
        emit_fldr_imm(buf, 3, Dtmp,   Xconst, ConstOff::TanPoly0 + 4);
        emit_fmadd_f64(buf, Dp_b, Dr2, Dtmp, Dp_b);

        // p03 = p01 + r⁴·p23
        emit_fmadd_f64(buf, Dp_acc, Dr4, Dp_b, Dp_acc);

        // p45 → Dp_b
        emit_fldr_imm(buf, 3, Dp_b,   Xconst, ConstOff::TanPoly0 + 5);
        emit_fldr_imm(buf, 3, Dtmp,   Xconst, ConstOff::TanPoly0 + 6);
        emit_fmadd_f64(buf, Dp_b, Dr2, Dtmp, Dp_b);

        // p67 → Dtmp (after using it for poly[6] load, reuse slot)
        const int Dp_c = alloc_free_fpr(a1);
        emit_fldr_imm(buf, 3, Dp_c,   Xconst, ConstOff::TanPoly0 + 7);
        emit_fldr_imm(buf, 3, Dtmp,   Xconst, ConstOff::TanPoly0 + 8);
        emit_fmadd_f64(buf, Dp_c, Dr2, Dtmp, Dp_c);

        // p47 = p45 + r⁴·p67
        emit_fmadd_f64(buf, Dp_b, Dr4, Dp_c, Dp_b);
        free_fpr(a1, Dp_c);

        // p07 = p03 + r⁸·p47
        emit_fmadd_f64(buf, Dp_acc, Dr8, Dp_b, Dp_acc);
        free_fpr(a1, Dp_b);

        // 7. p = poly[0] + r²·p07
        emit_fldr_imm(buf, 3, Dtmp,   Xconst, ConstOff::TanPoly0 + 0);
        emit_fmadd_f64(buf, Dp_acc, Dr2, Dp_acc, Dtmp);
        free_fpr(a1, Dtmp);
    }
    free_fpr(a1, Dr8);
    free_fpr(a1, Dr4);

    // 8. p = r + r²·(p · r)
    {
        const int Dpr = alloc_free_fpr(a1);
        emit_fmul_f64(buf, Dpr, Dp_acc, Dr);                          // pr = p · r
        emit_fmadd_f64(buf, Dp_acc, Dr2, Dpr, Dr);                    // p = r + r²·pr
        free_fpr(a1, Dpr);
    }
    free_fpr(a1, Dr);
    free_fpr(a1, Dr2);

    // 9. n = p² - 1   (FMADD with Da = -1)
    const int Dn = alloc_free_fpr(a1);
    {
        const int Dneg_one = alloc_free_fpr(a1);
        emit_fmov_d_one(buf, Dneg_one);
        emit_fneg_f64(buf, Dneg_one, Dneg_one);
        emit_fmadd_f64(buf, Dn, Dp_acc, Dp_acc, Dneg_one);            // n = -1 + p·p
        free_fpr(a1, Dneg_one);
    }

    // 10. d = 2·p
    const int Dd = alloc_free_fpr(a1);
    emit_fadd_f64(buf, Dd, Dp_acc, Dp_acc);
    free_fpr(a1, Dp_acc);

    // 11. Recombine: TST Xqi, #1.  If odd → tan = n/d;  if even → tan = -d/n.
    const int Dneg_d = alloc_free_fpr(a1);
    emit_fneg_f64(buf, Dneg_d, Dd);

    // TST Xqi, #1   (encoded as ANDS XZR, Xqi, #1)
    emit_logical_imm(buf, /*is_64=*/1, /*opc=ANDS*/3, /*N=*/1,
                     /*immr=*/0, /*imms=*/0, /*Rn=*/Xqi, /*Rd=*/31);
    free_gpr(a1, Xqi);

    // Numerator:   ne (qi odd) → n;     eq (qi even) → -d
    // Denominator: ne          → d;     eq            →  n
    const int Dnum_final = alloc_free_fpr(a1);
    const int Dden_final = alloc_free_fpr(a1);
    emit_fcsel_f64(buf, Dnum_final, Dn,    Dneg_d, /*cond=NE*/1);
    emit_fcsel_f64(buf, Dden_final, Dd,    Dn,     /*cond=NE*/1);
    free_fpr(a1, Dneg_d);
    free_fpr(a1, Dn);
    free_fpr(a1, Dd);

    // 12. result = num / den → d0
    emit_fdiv_f64(buf, /*Dd=*/0, Dnum_final, Dden_final);

    free_fpr(a1, Dnum_final);
    free_fpr(a1, Dden_final);
    free_gpr(a1, Xconst);
}

// fprem: ST(0) := fmod(ST(0), ST(1)).  No pop.  Single-shot impl
// (one call covers what stock x87 does iteratively in k≤64 quotient
// bits); same simplification the prior IPC sidecar used.  C0/C1/C2/C3
// flag bits in status_word are not updated (we don't track them).
//
// Algorithm:
//   q = trunc(a / b)        (FDIV + FRINTZ)
//   result = a - q · b      (FMSUB)
void emit_inline_fprem(TranslationResult& a1, AssemblerBuffer& buf,
                        int Xbase, int Wd_top, int Wd_tmp) {
    const int Xst_base  = x87_get_st_base(a1);
    const int depth_st0 = resolve_depth(a1, 0);
    const int depth_st1 = resolve_depth(a1, 1);

    const int Da = alloc_free_fpr(a1);
    emit_load_st(buf, Xbase, Wd_top, depth_st0, Wd_tmp, Da, Xst_base);
    const int Db = alloc_free_fpr(a1);
    emit_load_st(buf, Xbase, Wd_top, depth_st1, Wd_tmp, Db, Xst_base);

    const int Dq = alloc_free_fpr(a1);
    emit_fdiv_f64(buf, Dq, Da, Db);            // q = a / b
    emit_frintz_f64(buf, Dq, Dq);              // q = trunc(q)
    emit_fmsub_f64(buf, /*Dd=*/0, Dq, Db, Da); // d0 = a - q · b

    free_fpr(a1, Dq);
    free_fpr(a1, Db);
    free_fpr(a1, Da);
}

// fprem1: ST(0) := IEEE-remainder(ST(0), ST(1)).  Same emit shape as
// fprem; differs only in FRINTN (round-to-nearest-even) instead of
// FRINTZ.  std::remainder uses round-to-nearest-even rounding for the
// quotient, which FRINTN gives directly.
void emit_inline_fprem1(TranslationResult& a1, AssemblerBuffer& buf,
                         int Xbase, int Wd_top, int Wd_tmp) {
    const int Xst_base  = x87_get_st_base(a1);
    const int depth_st0 = resolve_depth(a1, 0);
    const int depth_st1 = resolve_depth(a1, 1);

    const int Da = alloc_free_fpr(a1);
    emit_load_st(buf, Xbase, Wd_top, depth_st0, Wd_tmp, Da, Xst_base);
    const int Db = alloc_free_fpr(a1);
    emit_load_st(buf, Xbase, Wd_top, depth_st1, Wd_tmp, Db, Xst_base);

    const int Dq = alloc_free_fpr(a1);
    emit_fdiv_f64(buf, Dq, Da, Db);            // q = a / b
    emit_frintn_f64(buf, Dq, Dq);              // q = round(q) (ties-to-even)
    emit_fmsub_f64(buf, /*Dd=*/0, Dq, Db, Da); // d0 = a - q · b

    free_fpr(a1, Dq);
    free_fpr(a1, Db);
    free_fpr(a1, Da);
}

void emit_inline_fyl2xp1(TranslationResult& a1, AssemblerBuffer& buf,
                          int Xbase, int Wd_top, int Wd_tmp) {
    // x86 fyl2xp1: ST(1) := ST(1) * log2(ST(0) + 1); pop.  x87 spec
    // restricts the input to -1+ε ≤ ST(0) ≤ 1-√2/2 ≈ 0.293; in that
    // range, simple add-then-log2 is accurate enough (no need for
    // log1p's special-case for cancellation).
    const int Xst_base  = x87_get_st_base(a1);
    const int depth_st0 = resolve_depth(a1, 0);
    const int depth_st1 = resolve_depth(a1, 1);

    const int Dx = alloc_free_fpr(a1);
    emit_load_st(buf, Xbase, Wd_top, depth_st0, Wd_tmp, Dx, Xst_base);

    const int Dy = alloc_free_fpr(a1);
    emit_load_st(buf, Xbase, Wd_top, depth_st1, Wd_tmp, Dy, Xst_base);

    // Dx := Dx + 1.0   (in-place; the "+1" of fyl2xp1).
    {
        const int Done = alloc_free_fpr(a1);
        emit_fmov_d_one(buf, Done);
        emit_fadd_f64(buf, Dx, Dx, Done);
        free_fpr(a1, Done);
    }

    const int Xconst = alloc_free_gpr(a1);
    const uint64_t consts_addr = rosetta_core::get_transcendental_constants_addr();
    assert(consts_addr != 0 && "transcendental constants not installed");
    emit_movz_movk_abs64(buf, Xconst, consts_addr);

    emit_inline_log2(a1, buf, Dx, Xconst, /*Dd_out=*/0);
    free_fpr(a1, Dx);
    free_gpr(a1, Xconst);

    emit_fmul_f64(buf, /*Dd=*/0, /*Dn=*/0, /*Dm=*/Dy);
    free_fpr(a1, Dy);
}

void emit_inline_fsincos(TranslationResult& a1, AssemblerBuffer& buf,
                          int Xbase, int Wd_top, int Wd_tmp) {
    // fsincos: replace ST(0) with sin(ST(0)) and push cos(ST(0)).
    // Convention matches the prior IPC path — d0 = sin, d1 = cos.
    // Both share the input load and the Xconst pointer; their range
    // reductions and qn values differ (sin uses round(x*inv_pi), cos
    // uses round(x*inv_pi + 0.5) - 0.5) so there's nothing to share
    // beyond Dx / Xconst between the two pipelines.
    const int Xst_base  = x87_get_st_base(a1);
    const int depth_st0 = resolve_depth(a1, 0);

    const int Dx = alloc_free_fpr(a1);
    emit_load_st(buf, Xbase, Wd_top, depth_st0, Wd_tmp, Dx, Xst_base);

    const int Xconst = alloc_free_gpr(a1);
    const uint64_t consts_addr = rosetta_core::get_transcendental_constants_addr();
    assert(consts_addr != 0 && "transcendental constants not installed; loader should have set address");
    emit_movz_movk_abs64(buf, Xconst, consts_addr);

    emit_inline_trig_body(a1, buf, Dx, Xconst, /*Dd_out=*/0, TrigReduceMode::Sin);
    emit_inline_trig_body(a1, buf, Dx, Xconst, /*Dd_out=*/1, TrigReduceMode::Cos);

    free_gpr(a1, Xconst);
    free_fpr(a1, Dx);
}

}  // namespace TranslatorX87
