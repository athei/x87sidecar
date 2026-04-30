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
};

static_assert(offsetof(rosetta_core::TranscendentalConstants, inv_pi)    == ConstOff::InvPi    * 8, "ConstOff::InvPi drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, pi_1)      == ConstOff::Pi1      * 8, "ConstOff::Pi1 drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, pi_2)      == ConstOff::Pi2      * 8, "ConstOff::Pi2 drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, pi_3)      == ConstOff::Pi3      * 8, "ConstOff::Pi3 drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, sin_c)     == ConstOff::SinC0    * 8, "ConstOff::SinC0 drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, range_val) == ConstOff::RangeVal * 8, "ConstOff::RangeVal drift");
static_assert(offsetof(rosetta_core::TranscendentalConstants, half)      == ConstOff::Half     * 8, "ConstOff::Half drift");

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
