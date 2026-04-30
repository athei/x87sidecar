#include "rosetta_core/TranslatorX87Transcendental.hpp"

#include <cassert>

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
    // Source: ARM-software/optimized-routines math/aarch64/advsimd/sin.c,
    // scalarised.  Worst-case 3.3 ULP in [-pi/2, pi/2], 2.73 ULP in
    // [-2^23, 2^23].  Beyond |x| >= 2^23 the 3-step Cody-Waite reduction
    // loses precision; result is junk.  Accepted (no fallback).
    //
    // Layout (imm12 = byte-offset / 8, since LDR D uses scaled offsets):
    //   [0]=inv_pi  [1..3]=pi_1..3  [4..10]=sin_c[0..6]  [11]=range_val.

    const int Xst_base = x87_get_st_base(a1);
    const int depth_st0 = resolve_depth(a1, 0);

    // 1. Load ST(0) into a scratch FPR.
    const int Dx = alloc_free_fpr(a1);
    emit_load_st(buf, Xbase, Wd_top, depth_st0, Wd_tmp, Dx, Xst_base);

    // 2. Materialise Xconst = &TranscendentalConstants in the trailing pad.
    const int Xconst = alloc_free_gpr(a1);
    const uint64_t consts_addr = rosetta_core::get_transcendental_constants_addr();
    assert(consts_addr != 0 && "transcendental constants not installed; loader should have set address");
    emit_movz_movk_abs64(buf, Xconst, consts_addr);

    // 3. n = round_to_nearest_away(x * inv_pi).
    const int Dn = alloc_free_fpr(a1);
    emit_fldr_imm(buf, /*size=64*/3, Dn, Xconst, /*imm12=*/0);  // Dn = inv_pi
    emit_fmul_f64(buf, Dn, Dx, Dn);                              // Dn = x * inv_pi
    emit_frinta_f64(buf, Dn, Dn);                                // Dn = round(Dn)

    // 4. qn = (int64) n  — kept until the final sign-flip XOR.
    const int Xqn = alloc_free_gpr(a1);
    emit_fcvtzs(buf, /*ftype=*/1, /*is_64bit_int=*/1, Xqn, Dn);

    // 5. r = x - n*pi_1 - n*pi_2 - n*pi_3   (3-step Cody-Waite reduction).
    const int Dr = alloc_free_fpr(a1);
    const int Dtmp = alloc_free_fpr(a1);
    emit_fldr_imm(buf, 3, Dtmp, Xconst, /*pi_1=*/1);
    emit_fmsub_f64(buf, Dr, Dn, Dtmp, Dx);                       // r = x - n*pi_1
    emit_fldr_imm(buf, 3, Dtmp, Xconst, /*pi_2=*/2);
    emit_fmsub_f64(buf, Dr, Dn, Dtmp, Dr);                       // r -= n*pi_2
    emit_fldr_imm(buf, 3, Dtmp, Xconst, /*pi_3=*/3);
    emit_fmsub_f64(buf, Dr, Dn, Dtmp, Dr);                       // r -= n*pi_3

    free_fpr(a1, Dn);
    free_fpr(a1, Dx);

    // 6. r2, r4 (defer r3 until after the polynomial — saves an FPR slot).
    const int Dr2 = alloc_free_fpr(a1);
    const int Dr4 = alloc_free_fpr(a1);
    emit_fmul_f64(buf, Dr2, Dr, Dr);                             // r2 = r*r
    emit_fmul_f64(buf, Dr4, Dr2, Dr2);                           // r4 = r2*r2

    // 7. Estrin polynomial accumulators p1=p01->p06, p2=p23->p26, p3=p45->p46.
    const int Dp1 = alloc_free_fpr(a1);
    const int Dp2 = alloc_free_fpr(a1);
    const int Dp3 = alloc_free_fpr(a1);

    // p01 = c0 + r2 * c1
    emit_fldr_imm(buf, 3, Dp1, Xconst, /*c0=*/4);
    emit_fldr_imm(buf, 3, Dtmp, Xconst, /*c1=*/5);
    emit_fmadd_f64(buf, Dp1, Dr2, Dtmp, Dp1);

    // p23 = c2 + r2 * c3
    emit_fldr_imm(buf, 3, Dp2, Xconst, /*c2=*/6);
    emit_fldr_imm(buf, 3, Dtmp, Xconst, /*c3=*/7);
    emit_fmadd_f64(buf, Dp2, Dr2, Dtmp, Dp2);

    // p45 = c4 + r2 * c5
    emit_fldr_imm(buf, 3, Dp3, Xconst, /*c4=*/8);
    emit_fldr_imm(buf, 3, Dtmp, Xconst, /*c5=*/9);
    emit_fmadd_f64(buf, Dp3, Dr2, Dtmp, Dp3);

    // p46 = p45 + r4 * c6
    emit_fldr_imm(buf, 3, Dtmp, Xconst, /*c6=*/10);
    emit_fmadd_f64(buf, Dp3, Dr4, Dtmp, Dp3);
    free_fpr(a1, Dtmp);

    // p26 = p23 + r4 * p46
    emit_fmadd_f64(buf, Dp2, Dr4, Dp3, Dp2);
    free_fpr(a1, Dp3);

    // p06 = p01 + r4 * p26
    emit_fmadd_f64(buf, Dp1, Dr4, Dp2, Dp1);
    free_fpr(a1, Dp2);
    free_fpr(a1, Dr4);

    // 8. r3 = r2 * r  (reuse Dr2's slot now that r2 is no longer needed).
    emit_fmul_f64(buf, Dr2, Dr2, Dr);                            // Dr2 := r3

    // 9. y = r + r3 * p06   (Dp1 := y, reusing the p06 slot).
    emit_fmadd_f64(buf, Dp1, Dr2, Dp1, Dr);
    free_fpr(a1, Dr2);
    free_fpr(a1, Dr);

    // 10. Sign flip: y = bit_xor(y, qn << 63)
    const int Xy = alloc_free_gpr(a1);
    emit_fmov_d_to_x(buf, Xy, Dp1);
    emit_logical_shifted_reg(buf, /*is_64=*/1, /*EOR=*/2, /*n=*/0,
                             /*shift_type=LSL*/0, /*Rm=*/Xqn,
                             /*shift_amount=*/63, /*Rn=*/Xy, /*Rd=*/Xy);

    // 11. Result lands in d0 — matches the post-IPC convention so
    //     translate_fsin's emit_store_st(..., Dd=0, ...) just works.
    emit_fmov_x_to_d(buf, /*Dd=*/0, Xy);

    free_gpr(a1, Xy);
    free_gpr(a1, Xqn);
    free_gpr(a1, Xconst);
    free_fpr(a1, Dp1);
}

}  // namespace TranslatorX87
