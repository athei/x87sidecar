#include "rosetta_core/TranslatorX87Transcendental.hpp"

#include <cassert>
#include <cstddef>

#include "rosetta_core/AssemblerHelpers.hpp"
#include "rosetta_core/TranscendentalHelper.h"
#include "rosetta_core/TranslatorX87Helpers.hpp"
#include "TranslatorX87Internal.hpp"

namespace TranslatorX87 {

namespace {

// BLR Xn — Branch with Link to Register.
// Encoding: 1101_0110_0011_1111_0000_00_Rn_00000  →  0xD63F0000 | (Rn << 5)
inline uint32_t encode_blr(int Rn) {
    return 0xD63F0000u | (uint32_t(Rn & 0x1F) << 5);
}

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

void emit_transcendental_ipc(TranslationResult& a1, AssemblerBuffer& buf,
                              int Xbase, int Wd_top, int Wd_tmp,
                              uint8_t opcode_tag, int num_inputs) {
    assert((num_inputs == 1 || num_inputs == 2) && "num_inputs must be 1 or 2");

    const int Xst_base = x87_get_st_base(a1);

    // Load ST(0) → d0.  Cache-aware: emit_load_st honours the deferred
    // permutation map via resolve_depth (0 normally, perm[0] when dirty).
    const int depth_st0 = resolve_depth(a1, 0);
    emit_load_st(buf, Xbase, Wd_top, depth_st0, Wd_tmp, /*Dd=*/0, Xst_base);

    if (num_inputs == 2) {
        const int depth_st1 = resolve_depth(a1, 1);
        emit_load_st(buf, Xbase, Wd_top, depth_st1, Wd_tmp, /*Dd=*/1, Xst_base);
    }

    // x0 = opcode_tag (small immediate, single MOVZ).
    emit_movn(buf, /*is_64=*/1, /*opc=MOVZ*/2, /*hw=*/0,
              static_cast<uint16_t>(opcode_tag), /*Rd=*/0);

    // x16 = absolute address of the trampoline blob in the parent's
    // libRosettaRuntime trailing pad.
    const uint64_t addr = rosetta_core::get_transcendental_helper_addr();
    assert(addr != 0 && "transcendental helper not installed; loader should have set address");
    emit_movz_movk_abs64(buf, /*Rd=*/16, addr);

    // BLR x16 — caller-saved regs (x0..x18, x30, d0..d7, d16..d31)
    // are clobbered.  Cache GPRs (Xbase=x22, Wd_top=x23, Wd_tmp=x24,
    // Xst_base=x28 if cached) are callee-saved by AAPCS64 and survive.
    buf.emit(encode_blr(/*Rn=*/16));

    // After return: d0 = out0 (first result), d1 = out1 (only
    // meaningful for fsincos).  Caller pulls them via emit_store_st(0)
    // and/or x87_push as appropriate for its op.
}

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
