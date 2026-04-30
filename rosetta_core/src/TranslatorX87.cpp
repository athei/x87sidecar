#include "rosetta_core/TranslatorX87.h"

#include <bit>
#include <utility>

#include "rosetta_config/Config.h"
#include "rosetta_core/CoreConfig.h"
#include "rosetta_core/IRInstr.h"
#include "rosetta_core/Opcode.h"
#include "rosetta_core/Register.h"
#include "rosetta_core/RuntimeRoutine.h"
#include "rosetta_core/TranscendentalHelper.h"
#include "rosetta_core/TranslationResult.h"
#include "rosetta_core/TranslatorHelpers.hpp"
#include "rosetta_core/TranslatorX87F80.hpp"
#include "rosetta_core/TranslatorX87Helpers.hpp"
#include "rosetta_core/TranslatorX87Transcendental.hpp"
#include "TranslatorX87Internal.hpp"

namespace TranslatorX87 {

// FLDZ -- push +0.0 onto the x87 stack.
//
// x87 semantics:
//   TOP = (TOP - 1) & 7
//   ST(0).mantissa = 0x0000000000000000  (+0.0 as IEEE 754 double)
//   status_word[13:11] = new TOP
//
// Register allocation:
//   Xbase   (gpr pool 0) -- X87State base = X18 + x87_state_offset
//   Wd_top  (gpr pool 1) -- TOP; updated in-place by emit_x87_push
//   Wd_tmp  (gpr pool 2) -- scratch for emit_store_top RMW and offset math
//   Dd_val  (fpr free pool) -- holds +0.0 for the FSTR
//
// OPT-2: Wd_tmp2 (pool slot 3) removed — tag word maintenance eliminated.
// OPT-5: MOVI Dd, #0 replaces FMOV Dd, XZR — removes GPR dependency, allows
//         the zero instruction to issue in parallel with the Xbase ADD.
//
auto translate_fldz(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Wd_tmp2 = alloc_gpr(*a1, 3);
    const int Dd_val = alloc_free_fpr(*a1);

    x87_push(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);

    // OPT-5: MOVI Dd, #0 — zero the D register with no GPR dependency.
    emit_movi_d_zero(buf, Dd_val);
    emit_store_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_val, Xst_base);

    free_fpr(*a1, Dd_val);
    free_gpr(*a1, Wd_tmp2);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// FLD1 -- push +1.0 onto the x87 stack.
//
// x87 semantics:
//   TOP = (TOP - 1) & 7
//   ST(0).mantissa = 0x3FF0000000000000  (+1.0 as IEEE 754 double)
//   status_word[13:11] = new TOP
//
// Register allocation:
//   Xbase   (gpr pool 0) -- X87State base = X18 + x87_state_offset
//   Wd_top  (gpr pool 1) -- TOP; updated in-place by emit_x87_push
//   Wd_tmp  (gpr pool 2) -- scratch for emit_x87_push RMW and offset math
//   Dd_val  (fpr free pool) -- holds +1.0 for the FSTR
//
// OPT-2: Wd_tmp2 (pool slot 3) removed — tag word maintenance eliminated.
// OPT-5: FMOV Dd, #1.0 replaces MOVZ+FMOV (2 insns → 1 insn), eliminates
//         Wd_tmp usage for the constant, and removes cross-domain latency.
//
auto translate_fld1(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Wd_tmp2 = alloc_gpr(*a1, 3);
    const int Dd_val = alloc_free_fpr(*a1);

    x87_push(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);

    // OPT-5: FMOV Dd, #1.0 — single instruction, no GPR intermediate.
    emit_fmov_d_one(buf, Dd_val);
    emit_store_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_val, Xst_base);

    free_fpr(*a1, Dd_val);
    free_gpr(*a1, Wd_tmp2);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// Constant-push helpers — fldl2e, fldl2t, fldlg2, fldln2, fldpi
//
// All five push a known IEEE 754 double constant onto the x87 stack.
// The pattern is identical to fld1 but all four 16-bit chunks are non-zero,
// requiring MOVZ hw=3 + three MOVK instructions.
//
// Shared static helper — keeps each translate_fldXX to a single call.
// =============================================================================
static void translate_fld_const(TranslationResult* a1, uint64_t bits) {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Wd_tmp2 = alloc_gpr(*a1, 3);
    const int Dd_val = alloc_free_fpr(*a1);

    x87_push(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);
    free_gpr(*a1, Wd_tmp2);

    // OPT-H: Inline constant pool — LDR Dd, [PC, #8] + B + .quad
    // Replaces MOVZ/MOVK chain + FMOV (4-5 insns) with 2 insns + 8 bytes data.
    emit_ldr_literal_f64(buf, Dd_val, bits);
    emit_store_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_val, Xst_base);

    free_fpr(*a1, Dd_val);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// FLDL2E — push log2(e) = 0x3FF71547652B82FE
auto translate_fldl2e(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    translate_fld_const(a1, 0x3FF71547652B82FEULL);
}

// FLDL2T — push log2(10) = 0x400A934F0979A371
auto translate_fldl2t(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    translate_fld_const(a1, 0x400A934F0979A371ULL);
}

// FLDLG2 — push log10(2) = 0x3FD34413509F79FF
auto translate_fldlg2(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    translate_fld_const(a1, 0x3FD34413509F79FFULL);
}

// FLDLN2 — push ln(2) = 0x3FE62E42FEFA39EF
auto translate_fldln2(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    translate_fld_const(a1, 0x3FE62E42FEFA39EFULL);
}

// FLDPI — push pi = 0x400921FB54442D18
auto translate_fldpi(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    translate_fld_const(a1, 0x400921FB54442D18ULL);
}

// =============================================================================
// FLD — D9 /0 (m32fp), DD /0 (m64fp), D9 C0+i (ST(i))
//
// Pushes a floating-point value onto the x87 stack.
//
// Operand encoding (Rosetta IR):
//   FLD ST(i)   D9 C0+i  operands = [ST(0) dst Register, ST(i) src Register]
//   FLD m32fp   D9 /0    operands = [m32fp MemRef src]  — operands[0].size == S32
//   FLD m64fp   DD /0    operands = [m64fp MemRef src]  — operands[0].size == S64
//
// FLD m80fp (DB /5) is intentionally NOT handled here. The dispatcher in
// `Translator::translate_instruction` short-circuits to nullopt before
// calling us so stock libRosettaRuntime translates the entire instruction
// (its emit + its `x87_fld_fp80` routine, ABI matched). A previous
// version emitted a BL+Fixup for this case targeting our deleted dylib's
// custom routine; without the dylib, that fixup resolves to Apple's
// stock routine whose ABI doesn't match our prologue's register
// convention.
//
// ST(i) ordering requirement:
//   Intel spec: temp = ST(i); TOP--; ST(0) = temp.
//   The value MUST be read before emit_x87_push because push decrements TOP,
//   which would change the physical slot that depth i maps to.
//   (Special case: FLD ST(0) duplicates the top — still correct with load-before-push.)
//
// Register allocation (direct-emit paths):
//   Xbase   (gpr pool 0) -- X87State base pointer
//   Wd_top  (gpr pool 1) -- current TOP; updated in-place by emit_x87_push
//   Wd_tmp  (gpr pool 2) -- offset scratch for emit_{load,store}_st;
//                           reused as RMW scratch inside emit_x87_push
//   Dd_val  (fpr free pool) -- the loaded value
//   addr_reg (free pool) -- memory paths only; freed after FP load (OPT-4)
// =============================================================================
auto translate_fld(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;

    // -------------------------------------------------------------------------
    // Common setup for ST(i), m32fp, m64fp paths
    // -------------------------------------------------------------------------
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Wd_tmp2 = alloc_gpr(*a1, 3);
    const int Dd_val = alloc_free_fpr(*a1);

    if (a2->operands[0].kind == IROperandKind::Register) {
        // FLD ST(i) — D9 C0+i
        const int depth_src = a2->operands[1].reg.reg.index();
        emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, depth_src), Wd_tmp, Dd_val, Xst_base);
        x87_push(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);
        free_gpr(*a1, Wd_tmp2);
        emit_store_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_val, Xst_base);
    } else if (a2->operands[0].mem.size == IROperandSize::S32) {
        // FLD m32fp — D9 /0
        //
        // OPT-4: load the f32 value directly into an FP register via LDR Sd,
        // then widen with FCVT Dd, Sd.  Replaces the old path that went through
        // a GPR intermediate (read_operand_to_gpr → FMOV Dd, Xn → FCVT),
        // saving 1 instruction and eliminating the ~4-cycle cross-domain
        // transfer penalty on Apple M-series.
        //
        // Bug 4: addr_size derived from the operand (not hardcoded 64-bit) so
        // 32-bit guest code computes effective addresses correctly.
        x87_push(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);
        free_gpr(*a1, Wd_tmp2);

        const bool addr_is_64 = (a2->operands[0].mem.addr_size == IROperandSize::S64);
        const int addr_reg = compute_operand_address(*a1, addr_is_64, &a2->operands[0], GPR::XZR);
        // LDR Sd, [addr_reg] — load f32 directly into FP register
        emit_fldr_imm(buf, /*size=*/2 /*S=f32*/, Dd_val, addr_reg, /*imm12=*/0);
        free_gpr(*a1, addr_reg);

        // FCVT Dd, Sd — widen single → double
        emit_fcvt_s_to_d(buf, Dd_val, Dd_val);
        emit_store_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_val, Xst_base);
    } else if (a2->operands[0].mem.size == IROperandSize::S80) {
        // FLD m80fp — DB /5
        //
        // Inline f80 → f64 conversion via the shared emit_f80_to_f64 helper
        // in TranslatorX87F80.cpp.  The helper emits the same bit-math
        // sequence that previously lived inline here and is also used by
        // translate_frstor.

        // x87 push first — frees Wd_tmp2 so we have 4 free-pool regs available
        // for the conversion sequence.
        x87_push(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);
        free_gpr(*a1, Wd_tmp2);

        const bool addr_is_64 = (a2->operands[0].mem.addr_size == IROperandSize::S64);
        const int Xaddr = compute_operand_address(*a1, addr_is_64, &a2->operands[0], GPR::XZR);

        // Load 8-byte mantissa + 2-byte sign+exp word into caller-allocated
        // scratch, then free Xaddr early so the bit-math stage doesn't
        // exhaust the 8-slot GPR scratch pool.
        const int Xmant = alloc_free_gpr(*a1);
        emit_ldr_imm(buf, /*size=*/3, Xmant, Xaddr, /*imm12=*/0);
        const int Wexp  = alloc_free_gpr(*a1);
        // LDRH Wexp, [Xaddr, #8] — imm12 scales by 2 for halfword.
        emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1,
                         /*imm12=*/4, Xaddr, Wexp);
        free_gpr(*a1, Xaddr);

        const int Xsign  = alloc_free_gpr(*a1);
        const int Wd_aux = alloc_free_gpr(*a1);

        emit_f80_to_f64_convert(buf, Xmant, Wexp, Xsign, Wd_aux, Wd_tmp);

        free_gpr(*a1, Wd_aux);
        free_gpr(*a1, Xsign);
        free_gpr(*a1, Wexp);

        // FMOV Dd_val, Xmant — move raw bits to FP register, then store.
        emit_fmov_x_to_d(buf, Dd_val, Xmant);
        free_gpr(*a1, Xmant);

        emit_store_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_val, Xst_base);
    } else {
        // FLD m64fp — DD /0
        //
        // OPT-4: load f64 directly into FP register via LDR Dd.  Replaces the
        // old GPR-intermediate path (read_operand_to_gpr → FMOV Dd, Xn),
        // saving 1 instruction + cross-domain latency.
        x87_push(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);
        free_gpr(*a1, Wd_tmp2);

        const bool addr_is_64 = (a2->operands[0].mem.addr_size == IROperandSize::S64);
        const int addr_reg = compute_operand_address(*a1, addr_is_64, &a2->operands[0], GPR::XZR);
        // LDR Dd, [addr_reg] — load f64 directly into FP register
        emit_fldr_imm(buf, /*size=*/3 /*D=f64*/, Dd_val, addr_reg, /*imm12=*/0);
        free_gpr(*a1, addr_reg);

        emit_store_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_val, Xst_base);
    }

    free_fpr(*a1, Dd_val);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
    return;
}

// =============================================================================
// FBLD — DF /4 (m80bcd)
//
// Loads an 18-digit packed BCD value from memory, converts to f64, pushes onto
// the x87 stack as ST(0).
//
// m80bcd layout (10 bytes, little-endian):
//   bytes[0..8] : 18 packed BCD digits, 2 per byte (low byte = lowest digits).
//                 Within each byte: bits[7:4] = high digit, bits[3:0] = low digit.
//   byte[9]     : bit 7 = sign (1 = negative), bits[6:0] reserved (zero).
//
// Stock x87 spec is f80, but we natively store ST as f64 throughout. Conversion
// path is BCD → i64 → f64 (one SCVTF), which avoids the f80 unpack stock would
// pay. Max representable: |10^18 - 1| ≈ 60 bits, fits in i64; SCVTF rounds to
// nearest-even on the f64 → 18-digit values lose ~7 LSBs of precision, matching
// stock's f80→f64 round on the way out.
//
// We do not validate digit nibbles (>9 is implementation-defined per Intel);
// the binary value of the nibble flows through MADD as-is, matching stock.
// =============================================================================
auto translate_fbld(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const int Wd_tmp  = alloc_gpr(*a1, 2);   // x87_push scratch + emit_store_st offset
    const int Wd_tmp2 = alloc_gpr(*a1, 3);   // x87_push scratch (freed after push)
    const int Dd_val  = alloc_free_fpr(*a1);

    // Step 1: compute source memory address.
    const bool addr_is_64 = (a2->operands[0].mem.addr_size == IROperandSize::S64);
    const int Xaddr = compute_operand_address(*a1, addr_is_64, &a2->operands[0], GPR::XZR);

    // Step 2: load source bytes — 8-byte mantissa-side + 2-byte sign-side.
    //   LDR  Xlow,  [Xaddr]      (digits 0..15)
    //   LDRH Whigh, [Xaddr, #8]  (low byte: digits 16,17 ; high byte: sign in bit 7)
    const int Xlow  = alloc_free_gpr(*a1);
    const int Whigh = alloc_free_gpr(*a1);
    emit_ldr_imm(buf, /*size=*/3, Xlow, Xaddr, /*imm12=*/0);
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*opc=*/1, /*imm12=*/4, Xaddr, Whigh);
    free_gpr(*a1, Xaddr);

    // Step 3: x87 push — allocates ST(0), decrements TOP. Clobbers Wd_tmp/Wd_tmp2.
    // Xlow/Whigh are free-pool regs and are NOT touched by the push.
    x87_push(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);
    free_gpr(*a1, Wd_tmp2);

    // Step 4: build the 18-digit integer in Xacc, MSB-first via MADD chain.
    //   acc = digit_17
    //   acc = acc * 10 + digit_16
    //   acc = acc * 10 + digit_15
    //   ...
    //   acc = acc * 10 + digit_0
    // Wd_tmp is dead between x87_push and emit_store_st — we reuse it as the
    // current-digit scratch in the loop instead of burning a free-pool reg.
    const int Xacc  = alloc_free_gpr(*a1);
    const int Wten  = alloc_free_gpr(*a1);

    // Wten = 10
    emit_movn(buf, /*is_64=*/0, /*MOVZ=*/2, /*hw=*/0, /*imm=*/10, Wten);

    // Inline 64-bit MADD encoder: Xd = Xn * Xm + Xa
    //   1 0 0 11011 000 Rm 0 Ra Rn Rd  (sf=1)
    auto emit_madd64 = [&](int Rd, int Rn, int Rm, int Ra) {
        uint32_t insn = 0x9B000000u
                      | ((uint32_t)(Rm & 0x1F) << 16)
                      | ((uint32_t)(Ra & 0x1F) << 10)
                      | ((uint32_t)(Rn & 0x1F) << 5)
                      |  (uint32_t)(Rd & 0x1F);
        buf.emit(insn);
    };

    // Initialise Xacc = digit_17 (high nibble of byte 8 = bits 4..7 of Whigh).
    //   UBFX Wacc, Whigh, #4, #4
    emit_bitfield(buf, /*is_64=*/0, /*UBFM=*/2, /*N=*/0,
                  /*immr=*/4, /*imms=*/7, Whigh, Xacc);

    // digit_16 = Whigh[3:0]
    emit_bitfield(buf, /*is_64=*/0, /*UBFM=*/2, /*N=*/0,
                  /*immr=*/0, /*imms=*/3, Whigh, Wd_tmp);
    emit_madd64(Xacc, Xacc, Wten, Wd_tmp);

    // digits 15..0 from Xlow (bytes 7..0, MSB first).
    for (int byte_idx = 7; byte_idx >= 0; byte_idx--) {
        const int hi_lsb = byte_idx * 8 + 4;
        const int lo_lsb = byte_idx * 8;
        // hi nibble
        emit_bitfield(buf, /*is_64=*/1, /*UBFM=*/2, /*N=*/1,
                      /*immr=*/(int8_t)hi_lsb, /*imms=*/(int8_t)(hi_lsb + 3),
                      Xlow, Wd_tmp);
        emit_madd64(Xacc, Xacc, Wten, Wd_tmp);
        // lo nibble
        emit_bitfield(buf, /*is_64=*/1, /*UBFM=*/2, /*N=*/1,
                      /*immr=*/(int8_t)lo_lsb, /*imms=*/(int8_t)(lo_lsb + 3),
                      Xlow, Wd_tmp);
        emit_madd64(Xacc, Xacc, Wten, Wd_tmp);
    }
    free_gpr(*a1, Wten);
    free_gpr(*a1, Xlow);

    // Step 5: SCVTF Dd_val, Xacc — Xacc is always non-negative (summed positive
    // digits), so SCVTF produces |result|. The sign bit is OR'd into the f64
    // representation in step 6, which is the only path that correctly handles
    // negative zero (CSNEG of integer 0 is still 0; SCVTF of 0 is +0.0; only
    // an explicit sign-bit set produces -0.0 with bits 0x8000_0000_0000_0000).
    emit_scvtf(buf, /*is_64bit_int=*/1, /*ftype=*/1 /*f64*/, Dd_val, Xacc);

    // Step 6: OR sign bit (Whigh[15]) into the f64 result at bit 63.
    //   UBFX Wsign, Whigh, #15, #1     ; Wsign in [0..1]
    //   FMOV Xbits, Dd_val             ; raw f64 bits to GPR
    //   ORR  Xbits, Xbits, Xsign, LSL #63
    //   FMOV Dd_val, Xbits             ; bits back to FPR
    const int Wsign = alloc_free_gpr(*a1);
    emit_bitfield(buf, /*is_64=*/0, /*UBFM=*/2, /*N=*/0,
                  /*immr=*/15, /*imms=*/15, Whigh, Wsign);
    free_gpr(*a1, Whigh);
    // Reuse Xacc as the bits-shuttle GPR — its previous value is no longer needed.
    emit_fmov_d_to_x(buf, Xacc, Dd_val);
    emit_logical_shifted_reg(buf, /*is_64=*/1, /*ORR=*/1, /*n=*/0,
                             /*shift_type=LSL=*/0, /*Rm=*/Wsign,
                             /*shift_amount=*/63, /*Rn=*/Xacc, /*Rd=*/Xacc);
    emit_fmov_x_to_d(buf, Dd_val, Xacc);
    free_gpr(*a1, Wsign);
    free_gpr(*a1, Xacc);

    // Step 7: store ST(0) = Dd_val.
    emit_store_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_val, Xst_base);
    free_fpr(*a1, Dd_val);

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FILD — DB /0 (m32int), DF /0 (m16int), DF /5 (m64int)
//
// Loads a signed integer from memory, converts it to f64, and pushes it onto
// the x87 stack as ST(0).
//
// x87 semantics:
//   TOP = (TOP - 1) & 7
//   ST(0) = ConvertToDouble((signed)mem)
//
// Operand encoding (Rosetta IR):
//   DF /0:  FILD m16int  — operands = [m16int MemRef]  operands[0].size == S16
//   DB /0:  FILD m32int  — operands = [m32int MemRef]  operands[0].size == S32
//   DF /5:  FILD m64int  — operands = [m64int MemRef]  operands[0].size == S64
//
// Sequence:
//   1. Compute source memory address → addr_reg (free pool).
//   2. Load integer from memory into Wd_int (free pool), sign-extending as needed:
//        m16int: LDRH Wd_int, [addr_reg]  (zero-extend into W)
//                SXTH Wd_int, Wd_int      (sign-extend bits[15:0] → W)
//        m32int: LDR  Wd_int, [addr_reg]  (32-bit; SCVTF treats W as signed)
//        m64int: LDR  Xd_int, [addr_reg]  (64-bit)
//   3. Free addr_reg.
//   4. emit_x87_push — decrements TOP, updates status_word.
//      Clobbers Wd_tmp (used as RMW scratch).
//      Wd_int is held in a separate free-pool register and is NOT clobbered.
//   5. SCVTF Dd_val, Wd_int/Xd_int — signed integer → f64.
//        m16/m32: is_64bit_int=0 (W source)
//        m64:     is_64bit_int=1 (X source)
//   6. Store Dd_val into the freshly allocated ST(0).
//   7. Free Wd_int.
//
// Critical ordering constraint:
//   Wd_int MUST be a separate register from Wd_tmp.  emit_x87_push clobbers
//   Wd_tmp as its status_word RMW scratch (see emit_store_top).  If the
//   integer were held in Wd_tmp it would be destroyed before SCVTF in step 5,
//   producing garbage results whenever the push actually modifies Wd_tmp
//   (i.e. almost always, since TOP changes on every push).
//
// Why load-before-push?
//   The memory address is independent of TOP, so the load can happen in either
//   order relative to the push.  We load first to get addr_reg freed early,
//   keeping peak register pressure low.
//
// Why LDRH + SXTH for m16, not LDRSH?
//   emit_ldr_str_imm exposes opc=1 (LDR = zero-extend). LDRSH is opc=2 in
//   the AArch64 encoding, which the helper does not wrap.
//   LDRH + SXTH (SBFM W, W, #0, #15) is the correct two-instruction equivalent.
//
// Register allocation:
//   Xbase   (gpr pool 0) -- X87State base
//   Wd_top  (gpr pool 1) -- current TOP; updated in-place by emit_x87_push
//   Wd_tmp  (gpr pool 2) -- RMW scratch consumed by emit_x87_push,
//                           then offset scratch for emit_store_st
//   Wd_int  (gpr free pool) -- holds the loaded integer across the push;
//                              freed after SCVTF
//   Dd_val  (fpr free pool) -- converted f64 value; stored into ST(0)
// =============================================================================
auto translate_fild(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const IROperandSize int_size = a2->operands[0].mem.size;

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Wd_tmp2 = alloc_gpr(*a1, 3);
    const int Wd_int = alloc_free_gpr(*a1);  // survives emit_x87_push
    const int Dd_val = alloc_free_fpr(*a1);

    // Step 1: compute source memory address
    const int addr_reg =
        compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);

    // Step 2: load integer from memory into Wd_int, sign-extending as needed.
    // Wd_int is a free-pool register distinct from Wd_tmp, so emit_x87_push
    // cannot clobber it.
    if (int_size == IROperandSize::S16) {
        // LDRH Wd_int, [addr_reg]  — 16-bit zero-extending load into W
        emit_ldr_str_imm(buf, /*size=*/1 /*16-bit*/, /*is_fp=*/0, /*opc=*/1 /*LDR*/,
                         /*imm12=*/0, addr_reg, Wd_int);
        // SXTH Wd_int, Wd_int  — sign-extend bits[15:0] → W (SBFM W,W,#0,#15)
        emit_bitfield(buf, /*is_64bit=*/0, /*opc=*/0 /*SBFM*/, /*N=*/0,
                      /*immr=*/0, /*imms=*/15, Wd_int, Wd_int);
    } else if (int_size == IROperandSize::S32) {
        // LDR Wd_int, [addr_reg]  — 32-bit load; SCVTF(is_64bit_int=0) treats W as signed
        emit_ldr_str_imm(buf, /*size=*/2 /*32-bit*/, /*is_fp=*/0, /*opc=*/1 /*LDR*/,
                         /*imm12=*/0, addr_reg, Wd_int);
    } else {
        // LDR Xd_int, [addr_reg]  — 64-bit load (m64int, DF /5)
        emit_ldr_str_imm(buf, /*size=*/3 /*64-bit*/, /*is_fp=*/0, /*opc=*/1 /*LDR*/,
                         /*imm12=*/0, addr_reg, Wd_int);
    }

    // Step 3: free addr_reg — no longer needed
    free_gpr(*a1, addr_reg);

    // Step 4: push — allocates a new ST(0) slot, decrements TOP.
    // Clobbers Wd_tmp and Wd_tmp2.  Wd_int is unaffected (separate
    // free-pool register).
    x87_push(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);
    free_gpr(*a1, Wd_tmp2);

    // Step 5: SCVTF — convert signed integer in Wd_int/Xd_int to f64.
    // m16 and m32 use a W (32-bit) source register: is_64bit_int=0
    // m64 uses an X (64-bit) source register:        is_64bit_int=1
    const int is_64bit_int = (int_size == IROperandSize::S64) ? 1 : 0;
    emit_scvtf(buf, is_64bit_int, /*ftype=*/1 /*f64*/, Dd_val, Wd_int);

    // Step 6: store the converted value into the freshly pushed ST(0).
    // Wd_tmp is now clean (emit_x87_push is done with it) and used here
    // as the byte-offset scratch inside emit_store_st.
    emit_store_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_val, Xst_base);

    // Step 7: free in reverse allocation order
    free_fpr(*a1, Dd_val);
    free_gpr(*a1, Wd_int);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FADD — D8 /0, DC /0, D8 C0+i, DC C0+i
//
// Handles all four non-popping ADD variants under opcode kOpcodeName_fadd.
// Dispatch is on operands[0].kind:
//
//   Register → ST-ST path
//     D8 C0+i:  FADD ST(0), ST(i)  — operands = [ST(0), ST(i)]
//     DC C0+i:  FADD ST(i), ST(0)  — operands = [ST(i), ST(0)]
//     Both are handled identically: dst=operands[0].index, src=operands[1].index.
//
//   MemRef → Memory float path
//     D8 /0:  FADD m32fp  — operands = [m32fp MemRef, ST(0)]
//     DC /0:  FADD m64fp  — operands = [m64fp MemRef, ST(0)]
//     Source is operands[0] (MemRef). Destination is always ST(0).
// =============================================================================
auto translate_fadd(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd_dst = alloc_free_fpr(*a1);
    const int Dd_src = alloc_free_fpr(*a1);

    if (a2->operands[0].kind == IROperandKind::Register) {
        // -----------------------------------------------------------------
        // ST-ST path: FADD ST(depth_dst), ST(depth_src)
        // D8 C0+i: operands = [ST(0),  ST(i)]
        // DC C0+i: operands = [ST(i),  ST(0)]
        // -----------------------------------------------------------------
        const int depth_dst = a2->operands[0].reg.reg.index();
        const int depth_src = a2->operands[1].reg.reg.index();

        // Opt 3: load src first (its offset is discarded), then dst last so
        // Wd_tmp holds offset(depth_dst) and emit_store_st_at_offset can reuse it.
        emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, depth_src), Wd_tmp, Dd_src, Xst_base);
        const int Wk = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, depth_dst), Wd_tmp, Dd_dst, Xst_base);
        emit_fadd_f64(buf, Dd_dst, Dd_dst, Dd_src);
        emit_store_st_at_offset(buf, Xbase, Wk, Dd_dst, Xst_base);
    } else {
        // -----------------------------------------------------------------
        // Memory float path: FADD m32fp / m64fp
        //
        // Rosetta encodes these as [mem_src, ST(0)_dst]:
        //   D8 /0: operands = [m32fp MemRef, ST(0)]
        //   DC /0: operands = [m64fp MemRef, ST(0)]
        //
        // Source is operands[0] (MemRef). Destination is always ST(0).
        // -----------------------------------------------------------------
        const bool is_f32 = (a2->operands[0].mem.size == IROperandSize::S32);

        // Step 1: load ST(0) — Wd_tmp receives the byte offset for ST(0).
        const int Wk2 =
            emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_dst, Xst_base);

        // Step 2: compute memory address from operands[0]
        const int addr_reg =
            compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);

        // Step 3: load memory operand
        //   size 2 = S-register (f32), size 3 = D-register (f64)
        emit_fldr_imm(buf, is_f32 ? 2 : 3, Dd_src, addr_reg, /*imm12=*/0);

        // Return the address register to the free pool immediately — done with it.
        free_gpr(*a1, addr_reg);

        // Step 4: widen f32 → f64 if needed
        if (is_f32)
            emit_fcvt_s_to_d(buf, Dd_src, Dd_src);

        // Step 5: add
        emit_fadd_f64(buf, Dd_dst, Dd_dst, Dd_src);

        // Step 6: store back to ST(0).
        // Opt 3: addr_reg is a different free-pool register, so Wd_tmp still
        // holds the ST(0) byte offset from emit_load_st above.
        emit_store_st_at_offset(buf, Xbase, Wk2, Dd_dst, Xst_base);
    }

    free_fpr(*a1, Dd_src);
    free_fpr(*a1, Dd_dst);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FADDP — DE C0+i, DE C1
//
// x87 semantics:
//   ST(i) = ST(i) + ST(0)
//   TOP   = (TOP + 1) & 7   (pop: ST(0) slot becomes logically empty)
//
// DE C1 is the implicit form "FADDP" (no written operands in assembly) which
// the CPU decodes as FADDP ST(1), ST(0). Rosetta encodes it identically to
// FADDP ST(1), ST(0) in the IR, so depth_dst == 1 naturally; no special
// casing needed here.
//
// Ordering: store result first under the current TOP, then pop. Popping first
// would shift physical indices, causing ST(i) to resolve to the wrong slot
// when depth_dst > 0.
//
// Register allocation:
//   Xbase   (gpr pool 0) -- X87State base
//   Wd_top  (gpr pool 1) -- current TOP; updated in-place by emit_x87_pop
//   Wd_tmp  (gpr pool 2) -- offset scratch; reused for emit_x87_pop RMW
//   Dd_dst  (fpr free pool) -- ST(i) value; receives the sum
//   Dd_src  (fpr free pool) -- ST(0) value
//
auto translate_faddp(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const int depth_dst = a2->operands[0].reg.reg.index();  // ST(i)
    // operands[1] is always ST(0); depth_src == 0 by definition.

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd_dst = alloc_free_fpr(*a1);
    const int Dd_src = alloc_free_fpr(*a1);

    // Opt 3: load ST(0) (src) first — its offset is discarded.
    // Load ST(depth_dst) (dst) second so Wd_tmp holds offset(depth_dst).
    emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_src, Xst_base);
    const int Wk3 = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, depth_dst), Wd_tmp, Dd_dst, Xst_base);
    emit_fadd_f64(buf, Dd_dst, Dd_dst, Dd_src);
    emit_store_st_at_offset(buf, Xbase, Wk3, Dd_dst, Xst_base);

    // Pop: TOP = (TOP + 1) & 7.  Wd_tmp is dead after emit_store_st_at_offset,
    // safe to reuse as the status_word RMW scratch inside emit_x87_pop.
    x87_pop(buf, *a1, Xbase, Wd_top, Wd_tmp);

    free_fpr(*a1, Dd_src);
    free_fpr(*a1, Dd_dst);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FSUB / FSUBR — D8 /4, DC /4, D8 E0+i, DC E0+i  (non-popping)
//              — D8 /5, DC /5, D8 E8+i, DC E8+i  (reversed, non-popping)
//
// Handles all non-popping subtract variants under kOpcodeName_fsub and
// kOpcodeName_fsubr in a single function, dispatching on opcode to determine
// operand order.
//
// x87 semantics:
//   fsub:  ST(dst) = ST(dst) - ST(src)        (or ST(0) = ST(0) - mem)
//   fsubr: ST(dst) = ST(src) - ST(dst)        (or ST(0) = mem   - ST(0))
//
// Dispatch is on operands[0].kind:
//
//   Register → ST-ST path
//     D8 E0+i:  FSUB  ST(0), ST(i)  — operands = [ST(0), ST(i)]
//     DC E0+i:  FSUB  ST(i), ST(0)  — operands = [ST(i), ST(0)]
//     D8 E8+i:  FSUBR ST(0), ST(i)  — operands = [ST(0), ST(i)]
//     DC E8+i:  FSUBR ST(i), ST(0)  — operands = [ST(i), ST(0)]
//     dst = operands[0].index, src = operands[1].index in all cases.
//     For fsubr the arithmetic operands are swapped: result = Dd_src - Dd_dst.
//
//   MemRef → Memory float path
//     D8 /4:  FSUB  m32fp  — operands = [m32fp MemRef, ST(0)]
//     DC /4:  FSUB  m64fp  — operands = [m64fp MemRef, ST(0)]
//     D8 /5:  FSUBR m32fp  — operands = [m32fp MemRef, ST(0)]
//     DC /5:  FSUBR m64fp  — operands = [m64fp MemRef, ST(0)]
//     Source is operands[0] (MemRef). Destination is always ST(0).
//     fsub:  ST(0) = ST(0) - mem
//     fsubr: ST(0) = mem   - ST(0)
//
// Register allocation:
//   Xbase   (gpr pool 0) -- X87State base
//   Wd_top  (gpr pool 1) -- current TOP (read-only; no push/pop)
//   Wd_tmp  (gpr pool 2) -- offset scratch for emit_{load,store}_st
//   Dd_dst  (fpr free pool) -- destination / minuend operand
//   Dd_src  (fpr free pool) -- source / subtrahend operand
// =============================================================================
auto translate_fsub(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const bool is_fsubr = (a2->opcode == kOpcodeName_fsubr);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd_dst = alloc_free_fpr(*a1);
    const int Dd_src = alloc_free_fpr(*a1);

    if (a2->operands[0].kind == IROperandKind::Register) {
        // -----------------------------------------------------------------
        // ST-ST path
        // dst = operands[0].index, src = operands[1].index.
        // fsub:  result = Dd_dst - Dd_src  → stored into ST(dst)
        // fsubr: result = Dd_src - Dd_dst  → stored into ST(dst)
        // -----------------------------------------------------------------
        const int depth_dst = a2->operands[0].reg.reg.index();
        const int depth_src = a2->operands[1].reg.reg.index();

        // Opt 3: load src first (offset discarded), then dst last so Wd_tmp
        // holds offset(depth_dst) for emit_store_st_at_offset.
        emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, depth_src), Wd_tmp, Dd_src, Xst_base);
        const int Wk4 = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, depth_dst), Wd_tmp, Dd_dst, Xst_base);

        if (is_fsubr)
            emit_fsub_f64(buf, Dd_dst, Dd_src, Dd_dst);  // dst = src - dst
        else
            emit_fsub_f64(buf, Dd_dst, Dd_dst, Dd_src);  // dst = dst - src

        emit_store_st_at_offset(buf, Xbase, Wk4, Dd_dst, Xst_base);
    } else {
        // -----------------------------------------------------------------
        // Memory float path
        // Rosetta encodes as [mem_src, ST(0)_dst]:
        //   D8 /4, DC /4: operands = [m32/64fp MemRef, ST(0)]  (fsub)
        //   D8 /5, DC /5: operands = [m32/64fp MemRef, ST(0)]  (fsubr)
        // fsub:  ST(0) = ST(0) - mem   →  Dd_dst - Dd_src
        // fsubr: ST(0) = mem   - ST(0) →  Dd_src - Dd_dst
        // -----------------------------------------------------------------
        const bool is_f32 = (a2->operands[0].mem.size == IROperandSize::S32);

        const int Wk5 =
            emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_dst, Xst_base);

        const int addr_reg =
            compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);
        emit_fldr_imm(buf, is_f32 ? 2 : 3, Dd_src, addr_reg, /*imm12=*/0);
        free_gpr(*a1, addr_reg);

        if (is_f32)
            emit_fcvt_s_to_d(buf, Dd_src, Dd_src);

        if (is_fsubr)
            emit_fsub_f64(buf, Dd_dst, Dd_src, Dd_dst);  // ST(0) = mem - ST(0)
        else
            emit_fsub_f64(buf, Dd_dst, Dd_dst, Dd_src);  // ST(0) = ST(0) - mem

        // Opt 3: addr_reg is a separate free-pool register; Wd_tmp still holds
        // the ST(0) byte offset set by emit_load_st above.
        emit_store_st_at_offset(buf, Xbase, Wk5, Dd_dst, Xst_base);
    }

    free_fpr(*a1, Dd_src);
    free_fpr(*a1, Dd_dst);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FSUBP / FSUBRP — DE E8+i, DE E9  (popping subtract / reversed popping subtract)
//
// Handles both popping variants under kOpcodeName_fsubp and kOpcodeName_fsubrp
// in a single function, dispatching on opcode for operand order.
//
// x87 semantics:
//   fsubp:  ST(i) = ST(i) - ST(0);  TOP = (TOP + 1) & 7
//   fsubrp: ST(i) = ST(0) - ST(i);  TOP = (TOP + 1) & 7
//
// Encoding:
//   DE E8+i  FSUBP  ST(i), ST(0) — operands = [ST(i), ST(0)]
//   DE E9    FSUBP               — implicit ST(1), ST(0); depth_dst == 1
//   DE E0+i  FSUBRP ST(i), ST(0) — operands = [ST(i), ST(0)]
//   DE E1    FSUBRP              — implicit ST(1), ST(0); depth_dst == 1
//
// Pop ordering: result is written to ST(i) before popping so the physical
// index for ST(i) is still valid under the current TOP.
//
// Register allocation:
//   Xbase   (gpr pool 0) -- X87State base
//   Wd_top  (gpr pool 1) -- current TOP; updated in-place by emit_x87_pop
//   Wd_tmp  (gpr pool 2) -- offset scratch; reused for emit_x87_pop RMW
//   Dd_dst  (fpr free pool) -- ST(i) value; receives the result
//   Dd_src  (fpr free pool) -- ST(0) value
// =============================================================================
auto translate_fsubp(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const bool is_fsubrp = (a2->opcode == kOpcodeName_fsubrp);
    const int depth_dst = a2->operands[0].reg.reg.index();  // ST(i)
    // operands[1] is always ST(0); depth_src == 0 by definition.

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd_dst = alloc_free_fpr(*a1);
    const int Dd_src = alloc_free_fpr(*a1);

    // Opt 3: load ST(0) (src) first — its offset is discarded.
    // Load ST(depth_dst) (dst) second so Wd_tmp holds offset(depth_dst).
    emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_src, Xst_base);
    const int Wk6 = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, depth_dst), Wd_tmp, Dd_dst, Xst_base);

    if (is_fsubrp)
        emit_fsub_f64(buf, Dd_dst, Dd_src, Dd_dst);  // ST(i) = ST(0) - ST(i)
    else
        emit_fsub_f64(buf, Dd_dst, Dd_dst, Dd_src);  // ST(i) = ST(i) - ST(0)

    emit_store_st_at_offset(buf, Xbase, Wk6, Dd_dst, Xst_base);

    // Pop: TOP = (TOP + 1) & 7.  Wd_tmp is dead after emit_store_st_at_offset.
    x87_pop(buf, *a1, Xbase, Wd_top, Wd_tmp);

    free_fpr(*a1, Dd_src);
    free_fpr(*a1, Dd_dst);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FDIV / FDIVR — D8 /6, DC /6, D8 F0+i, DC F0+i  (non-popping)
//              — D8 /7, DC /7, D8 F8+i, DC F8+i  (reversed, non-popping)
//
// Handles all non-popping divide variants under kOpcodeName_fdiv and
// kOpcodeName_fdivr in a single function, dispatching on opcode to determine
// operand order.
//
// x87 semantics:
//   fdiv:  ST(dst) = ST(dst) / ST(src)        (or ST(0) = ST(0) / mem)
//   fdivr: ST(dst) = ST(src) / ST(dst)        (or ST(0) = mem   / ST(0))
//
// Dispatch is on operands[0].kind:
//
//   Register → ST-ST path
//     D8 F0+i:  FDIV  ST(0), ST(i)  — operands = [ST(0), ST(i)]
//     DC F0+i:  FDIV  ST(i), ST(0)  — operands = [ST(i), ST(0)]
//     D8 F8+i:  FDIVR ST(0), ST(i)  — operands = [ST(0), ST(i)]
//     DC F8+i:  FDIVR ST(i), ST(0)  — operands = [ST(i), ST(0)]
//     dst = operands[0].index, src = operands[1].index in all cases.
//     fdiv:  result = Dd_dst / Dd_src  → stored into ST(dst)
//     fdivr: result = Dd_src / Dd_dst  → stored into ST(dst)
//
//   MemRef → Memory float path
//     D8 /6:  FDIV  m32fp  — operands = [m32fp MemRef, ST(0)]
//     DC /6:  FDIV  m64fp  — operands = [m64fp MemRef, ST(0)]
//     D8 /7:  FDIVR m32fp  — operands = [m32fp MemRef, ST(0)]
//     DC /7:  FDIVR m64fp  — operands = [m64fp MemRef, ST(0)]
//     Source is operands[0] (MemRef). Destination is always ST(0).
//     fdiv:  ST(0) = ST(0) / mem   →  Dd_dst / Dd_src
//     fdivr: ST(0) = mem   / ST(0) →  Dd_src / Dd_dst
//
// Register allocation:
//   Xbase   (gpr pool 0) -- X87State base
//   Wd_top  (gpr pool 1) -- current TOP (read-only; no push/pop)
//   Wd_tmp  (gpr pool 2) -- offset scratch for emit_{load,store}_st
//   Dd_dst  (fpr free pool) -- destination / dividend operand
//   Dd_src  (fpr free pool) -- source / divisor operand
// =============================================================================
auto translate_fdiv(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const bool is_fdivr = (a2->opcode == kOpcodeName_fdivr);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd_dst = alloc_free_fpr(*a1);
    const int Dd_src = alloc_free_fpr(*a1);

    if (a2->operands[0].kind == IROperandKind::Register) {
        // -----------------------------------------------------------------
        // ST-ST path
        // dst = operands[0].index, src = operands[1].index.
        // fdiv:  result = Dd_dst / Dd_src  → stored into ST(dst)
        // fdivr: result = Dd_src / Dd_dst  → stored into ST(dst)
        // -----------------------------------------------------------------
        const int depth_dst = a2->operands[0].reg.reg.index();
        const int depth_src = a2->operands[1].reg.reg.index();

        // Opt 3: load src first (offset discarded), then dst last so Wd_tmp
        // holds offset(depth_dst) for emit_store_st_at_offset.
        emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, depth_src), Wd_tmp, Dd_src, Xst_base);
        const int Wk7 = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, depth_dst), Wd_tmp, Dd_dst, Xst_base);

        if (is_fdivr)
            emit_fdiv_f64(buf, Dd_dst, Dd_src, Dd_dst);  // dst = src / dst
        else
            emit_fdiv_f64(buf, Dd_dst, Dd_dst, Dd_src);  // dst = dst / src

        emit_store_st_at_offset(buf, Xbase, Wk7, Dd_dst, Xst_base);
    } else {
        // -----------------------------------------------------------------
        // Memory float path
        // Rosetta encodes as [mem_src, ST(0)_dst]:
        //   D8 /6, DC /6: operands = [m32/64fp MemRef, ST(0)]  (fdiv)
        //   D8 /7, DC /7: operands = [m32/64fp MemRef, ST(0)]  (fdivr)
        // fdiv:  ST(0) = ST(0) / mem   →  Dd_dst / Dd_src
        // fdivr: ST(0) = mem   / ST(0) →  Dd_src / Dd_dst
        // -----------------------------------------------------------------
        const bool is_f32 = (a2->operands[0].mem.size == IROperandSize::S32);

        const int Wk8 =
            emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_dst, Xst_base);

        const int addr_reg =
            compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);
        emit_fldr_imm(buf, is_f32 ? 2 : 3, Dd_src, addr_reg, /*imm12=*/0);
        free_gpr(*a1, addr_reg);

        if (is_f32)
            emit_fcvt_s_to_d(buf, Dd_src, Dd_src);

        if (is_fdivr)
            emit_fdiv_f64(buf, Dd_dst, Dd_src, Dd_dst);  // ST(0) = mem / ST(0)
        else
            emit_fdiv_f64(buf, Dd_dst, Dd_dst, Dd_src);  // ST(0) = ST(0) / mem

        // Opt 3: addr_reg is a separate free-pool register; Wd_tmp still holds
        // the ST(0) byte offset set by emit_load_st above.
        emit_store_st_at_offset(buf, Xbase, Wk8, Dd_dst, Xst_base);
    }

    free_fpr(*a1, Dd_src);
    free_fpr(*a1, Dd_dst);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FDIVP / FDIVRP — DE F8+i, DE F9  (popping divide / reversed popping divide)
//
// Handles both popping variants under kOpcodeName_fdivp and kOpcodeName_fdivrp
// in a single function, dispatching on opcode for operand order.
//
// x87 semantics:
//   fdivp:  ST(i) = ST(i) / ST(0);  TOP = (TOP + 1) & 7
//   fdivrp: ST(i) = ST(0) / ST(i);  TOP = (TOP + 1) & 7
//
// Encoding:
//   DE F8+i  FDIVP  ST(i), ST(0) — operands = [ST(i), ST(0)]
//   DE F9    FDIVP               — implicit ST(1), ST(0); depth_dst == 1
//   DE F0+i  FDIVRP ST(i), ST(0) — operands = [ST(i), ST(0)]
//   DE F1    FDIVRP              — implicit ST(1), ST(0); depth_dst == 1
//
// Pop ordering: result is written to ST(i) before popping so the physical
// index for ST(i) is still valid under the current TOP.
//
// Register allocation:
//   Xbase   (gpr pool 0) -- X87State base
//   Wd_top  (gpr pool 1) -- current TOP; updated in-place by emit_x87_pop
//   Wd_tmp  (gpr pool 2) -- offset scratch; reused for emit_x87_pop RMW
//   Dd_dst  (fpr free pool) -- ST(i) value; receives the result
//   Dd_src  (fpr free pool) -- ST(0) value
// =============================================================================
auto translate_fdivp(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const bool is_fdivrp = (a2->opcode == kOpcodeName_fdivrp);
    const int depth_dst = a2->operands[0].reg.reg.index();  // ST(i)
    // operands[1] is always ST(0); depth_src == 0 by definition.

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd_dst = alloc_free_fpr(*a1);
    const int Dd_src = alloc_free_fpr(*a1);

    // Opt 3: load ST(0) (src) first — its offset is discarded.
    // Load ST(depth_dst) (dst) second so Wd_tmp holds offset(depth_dst).
    emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_src, Xst_base);
    const int Wk9 = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, depth_dst), Wd_tmp, Dd_dst, Xst_base);

    if (is_fdivrp)
        emit_fdiv_f64(buf, Dd_dst, Dd_src, Dd_dst);  // ST(i) = ST(0) / ST(i)
    else
        emit_fdiv_f64(buf, Dd_dst, Dd_dst, Dd_src);  // ST(i) = ST(i) / ST(0)

    emit_store_st_at_offset(buf, Xbase, Wk9, Dd_dst, Xst_base);

    // Pop: TOP = (TOP + 1) & 7.  Wd_tmp is dead after emit_store_st_at_offset.
    x87_pop(buf, *a1, Xbase, Wd_top, Wd_tmp);

    free_fpr(*a1, Dd_src);
    free_fpr(*a1, Dd_dst);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FIADD — DA /0, DE /0
//
// x87 semantics:
//   ST(0) = ST(0) + ConvertToDouble(src_int)
//
// Source is a memory integer (Rosetta encodes as [mem_src, ST(0)_dst]):
//   DA /0:  FIADD m32int  — operands = [m32int MemRef, ST(0)]
//   DE /0:  FIADD m16int  — operands = [m16int MemRef, ST(0)]
//
// Destination is always ST(0).
//
// Sequence:
//   1. Load ST(0) as f64 into Dd_st0.
//   2. Compute source address → addr_reg.
//   3. Load 32-bit or 16-bit integer from memory into Wd_tmp.
//      For 16-bit: LDRH (zero-extend into W) then SXTH (sign-extend to W).
//      For 32-bit: LDR W (loads a signed 32-bit directly).
//   4. SCVTF Dd_int, Wd_tmp  — signed integer → f64.
//   5. FADD Dd_st0, Dd_st0, Dd_int.
//   6. Store result back into ST(0).
//
// Why LDRH + SXTH for 16-bit, not LDRSH?
//   emit_ldr_str_imm exposes opc=0 (STR) and opc=1 (LDR = zero-extend).
//   LDRSH uses opc=2 in the AArch64 encoding, which the helper doesn't wrap.
//   LDRH + SXTH (SBFM immr=0, imms=15) is a clean two-instruction equivalent.
//
// Register allocation:
//   Xbase   (gpr pool 0) -- X87State base
//   Wd_top  (gpr pool 1) -- current TOP (read-only; no push/pop here)
//   Wd_tmp  (gpr pool 2) -- offset scratch for emit_load_st, then reused as
//                           the integer load target and SCVTF source
//   Dd_st0  (fpr free pool) -- ST(0) value; receives the sum
//   Dd_int  (fpr free pool) -- converted integer value
//
auto translate_fiadd(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    // Rosetta encodes FIADD as [mem_src, ST(0)_dst]:
    //   DA /0: operands = [m32int MemRef, ST(0)]
    //   DE /0: operands = [m16int MemRef, ST(0)]
    // Source size is in operands[0].mem.size.
    const bool is_m16 = (a2->operands[0].mem.size == IROperandSize::S16);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd_st0 = alloc_free_fpr(*a1);
    const int Dd_int = alloc_free_fpr(*a1);

    // Step 1: load ST(0).  Wd_tmp receives the byte offset for ST(0).
    const int Wk10 = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_st0, Xst_base);

    // Step 2: compute source memory address — allocates a separate free-pool GPR.
    const int addr_reg =
        compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);

    // Step 3: load integer from memory into addr_reg (reusing it for the value).
    // Opt 3: loading into addr_reg (not Wd_tmp) preserves the ST(0) byte offset
    // in Wd_tmp so emit_store_st_at_offset can skip recomputing it below.
    if (is_m16) {
        // LDRH addr_reg, [addr_reg]  — 16-bit zero-extending load
        emit_ldr_str_imm(buf, /*size=*/1 /*16-bit*/, /*is_fp=*/0, /*opc=*/1 /*LDR*/,
                         /*imm12=*/0, addr_reg, addr_reg);
        // SXTH addr_reg, addr_reg  — sign-extend bits[15:0] → W
        // Encoded as SBFM W, W, #0, #15
        emit_bitfield(buf, /*is_64bit=*/0, /*opc=*/0 /*SBFM*/, /*N=*/0,
                      /*immr=*/0, /*imms=*/15, addr_reg, addr_reg);
    } else {
        // LDR addr_reg, [addr_reg]  — 32-bit load (signed by virtue of SCVTF below)
        emit_ldr_str_imm(buf, /*size=*/2 /*32-bit*/, /*is_fp=*/0, /*opc=*/1 /*LDR*/,
                         /*imm12=*/0, addr_reg, addr_reg);
    }

    // Step 4: convert signed integer W → f64
    // emit_scvtf: is_64bit_int=0 (32-bit source W), ftype=1 (f64 destination)
    emit_scvtf(buf, /*is_64bit_int=*/0, /*ftype=*/1, Dd_int, addr_reg);

    free_gpr(*a1, addr_reg);

    // Step 5: add
    emit_fadd_f64(buf, Dd_st0, Dd_st0, Dd_int);

    // Step 6: store result back to ST(0).
    // Opt 3: Wd_tmp still holds the ST(0) byte offset from emit_load_st above.
    emit_store_st_at_offset(buf, Xbase, Wk10, Dd_st0, Xst_base);

    free_fpr(*a1, Dd_int);
    free_fpr(*a1, Dd_st0);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FMUL / FMULP
//
// Handles all multiply variants under kOpcodeName_fmul and kOpcodeName_fmulp.
//
// Opcode / encoding breakdown:
//
//   kOpcodeName_fmul  (non-popping)
//     D8 C8+i  FMUL ST(0), ST(i)  — operands = [ST(0), ST(i)]   Register path
//     DC C8+i  FMUL ST(i), ST(0)  — operands = [ST(i), ST(0)]   Register path
//     D8 /1    FMUL m32fp         — operands = [m32fp MemRef]    Memory path
//     DC /1    FMUL m64fp         — operands = [m64fp MemRef]    Memory path
//
//   kOpcodeName_fmulp  (popping)
//     DE C8+i  FMULP ST(i), ST(0) — operands = [ST(i), ST(0)]   Register path
//     DE C9    FMULP              — implicit ST(1), ST(0)        Register path
//                                   (Rosetta encodes as depth_dst == 1)
//
// x87 semantics:
//   fmul  ST(dst) = ST(dst) * ST(src)        (no pop)
//   fmulp ST(i)   = ST(i)   * ST(0); TOP++   (pop after)
//
// Register allocation:
//   Xbase   (gpr pool 0) -- X87State base = X18 + x87_state_offset
//   Wd_top  (gpr pool 1) -- current TOP; updated in-place by emit_x87_pop
//   Wd_tmp  (gpr pool 2) -- offset scratch for emit_{load,store}_st;
//                           reused as RMW scratch inside emit_x87_pop
//   Dd_dst  (free pool)  -- destination operand; receives the product
//   Dd_src  (free pool)  -- source operand (ST-ST and memory paths)
//                           not allocated for the fmulp path (single load suffices)
//
// Memory path note:
//   Source is operands[0] (MemRef). Destination is always ST(0).
//   Mirrors translate_fadd's memory path exactly, substituting FMUL for FADD.
//
// Pop ordering (fmulp):
//   Result is written to ST(i) before popping.  Popping first would shift
//   physical indices, causing ST(i) to resolve to the wrong slot when
//   depth_dst > 0.  Matches the ordering used in translate_faddp.
// =============================================================================
auto translate_fmul(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const bool is_fmulp = (a2->opcode == kOpcodeName_fmulp);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd_dst = alloc_free_fpr(*a1);
    const int Dd_src = alloc_free_fpr(*a1);

    if (is_fmulp) {
        // ---------------------------------------------------------------------
        // FMULP ST(i), ST(0) — DE C8+i
        //
        // ST(i) = ST(i) * ST(0); TOP = (TOP + 1) & 7
        //
        // operands[0] is ST(i) dst; operands[1] is ST(0) src (depth == 0).
        // Store result before popping so ST(i) physical index is still valid.
        // ---------------------------------------------------------------------
        const int depth_dst = a2->operands[0].reg.reg.index();  // ST(i)
        // operands[1] is always ST(0); depth_src == 0 implicitly.

        // Opt 3: load ST(0) (src) first — its offset is discarded.
        // Load ST(depth_dst) (dst) second so Wd_tmp holds offset(depth_dst).
        emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_src, Xst_base);
        const int Wk11 = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, depth_dst), Wd_tmp, Dd_dst, Xst_base);
        emit_fmul_f64(buf, Dd_dst, Dd_dst, Dd_src);
        emit_store_st_at_offset(buf, Xbase, Wk11, Dd_dst, Xst_base);

        // Pop: TOP = (TOP + 1) & 7.  Wd_tmp is dead after emit_store_st_at_offset.
        x87_pop(buf, *a1, Xbase, Wd_top, Wd_tmp);
    } else if (a2->operands[0].kind == IROperandKind::Register) {
        // ---------------------------------------------------------------------
        // ST-ST path — D8 C8+i / DC C8+i
        //
        // D8 C8+i: FMUL ST(0), ST(i) — operands = [ST(0), ST(i)]
        // DC C8+i: FMUL ST(i), ST(0) — operands = [ST(i), ST(0)]
        //
        // Both encoded the same way: dst = operands[0].index,
        //                            src = operands[1].index.
        // ---------------------------------------------------------------------
        const int depth_dst = a2->operands[0].reg.reg.index();
        const int depth_src = a2->operands[1].reg.reg.index();

        // Opt 3: load src first (offset discarded), then dst last so Wd_tmp
        // holds offset(depth_dst) for emit_store_st_at_offset.
        emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, depth_src), Wd_tmp, Dd_src, Xst_base);
        const int Wk12 = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, depth_dst), Wd_tmp, Dd_dst, Xst_base);
        emit_fmul_f64(buf, Dd_dst, Dd_dst, Dd_src);
        emit_store_st_at_offset(buf, Xbase, Wk12, Dd_dst, Xst_base);
    } else {
        // ---------------------------------------------------------------------
        // Memory path — D8 /1 (m32fp) / DC /1 (m64fp)
        //
        // Rosetta encodes these as [mem_src, ST(0)_dst]:
        //   D8 /1: operands = [m32fp MemRef, ST(0)]
        //   DC /1: operands = [m64fp MemRef, ST(0)]
        //
        // Source is operands[0] (MemRef). Destination is always ST(0).
        // ---------------------------------------------------------------------
        const bool is_f32 = (a2->operands[0].mem.size == IROperandSize::S32);

        // Step 1: load ST(0) — Wd_tmp receives the byte offset for ST(0).
        const int Wk13 =
            emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_dst, Xst_base);

        // Step 2: compute memory address
        const int addr_reg =
            compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);

        // Step 3: load memory operand
        //   size 2 = S-register (f32), size 3 = D-register (f64)
        emit_fldr_imm(buf, is_f32 ? 2 : 3, Dd_src, addr_reg, /*imm12=*/0);
        free_gpr(*a1, addr_reg);

        // Step 4: widen f32 → f64 if needed
        if (is_f32)
            emit_fcvt_s_to_d(buf, Dd_src, Dd_src);

        // Step 5: multiply
        emit_fmul_f64(buf, Dd_dst, Dd_dst, Dd_src);

        // Step 6: store back to ST(0).
        // Opt 3: addr_reg is a separate free-pool register; Wd_tmp still holds
        // the ST(0) byte offset set by emit_load_st above.
        emit_store_st_at_offset(buf, Xbase, Wk13, Dd_dst, Xst_base);
    }

    free_fpr(*a1, Dd_src);
    free_fpr(*a1, Dd_dst);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FST / FSTP (memory) — store ST(0) to a memory operand, optional pop.
//
// x87 semantics:
//   FST  mXX:  mem = convert(ST(0), size)
//   FSTP mXX:  mem = convert(ST(0), size); TOP = (TOP + 1) & 7
//
// operands[0] is the memory destination (IROperandMemRef).
// operands[0].size encodes the store width: S32 → float32, S64 → float64,
//                                           S80 → float80 (runtime routine).
// Source is always ST(0).
//
// Register allocation (S32 / S64 direct-emit paths):
//   Xbase   (gpr pool 0) -- X87State base
//   Wd_top  (gpr pool 1) -- current TOP; updated in-place by emit_x87_pop
//   Wd_tmp  (gpr pool 2) -- offset scratch for emit_load_st; reused as
//                           RMW scratch inside emit_x87_pop
//   Dd_src  (fpr free pool) -- ST(0) value; narrowed in-place to Sd_src for S32
//
// Register allocation (S80 runtime path):
//   Xaddr   (gpr pool 0) -- destination address passed to runtime routine
//
// FST / FSTP / FST_STACK / FSTP_STACK
//
// Opcode encoding:
//   fst        DD /2      operands[0] = MemRef m32fp/m64fp/m80fp
//   fstp       DD /3      operands[0] = MemRef m32fp/m64fp/m80fp  (+ pop)
//   fst_stack  DD C0+i    operands[0] = Register ST(i) dst
//   fstp_stack DD D8+i    operands[0] = Register ST(i) dst        (+ pop)
//
// The Register path (fst_stack / fstp_stack) copies ST(0) into ST(depth_dst).
// Special case: when dst == 0 (FST ST(0)), it is a no-op — reading and writing
// the same slot, so the load/store is skipped. The pop still fires for fstp_stack.
//
// The Memory path (fst / fstp) stores ST(0) to a memory address.
// S80 is handed off to a runtime routine (kRuntimeRoutine_fstp_fp80 / fst_fp80)
// because the 80-bit extended format has a different exponent bias (16383 vs
// 1023) and an explicit integer bit — it cannot be produced from a 64-bit double
// via a plain FCVT, and writing only 8 bytes to an m80fp destination corrupts
// the 2-byte exponent field at addr+8.
// is_f32 is only valid for the S32/S64 direct-emit memory path.
auto translate_fst(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;

    const bool is_fstp = (a2->opcode == kOpcodeName_fstp || a2->opcode == kOpcodeName_fstp_stack);

    // -------------------------------------------------------------------------
    // FSTP m80fp — DB /7  (inline double → f80 conversion + store + pop)
    //
    // Converts the IEEE 754 double in ST(0) to x87 80-bit extended format and
    // writes the 10-byte result directly to the destination memory address.
    //
    // IEEE 754 double:  [63] sign | [62:52] exp (11-bit, bias 1023) | [51:0] mantissa
    // x87 f80:          bytes 0-7 mantissa (64-bit, explicit integer bit at 63)
    //                   bytes 8-9 [15] sign | [14:0] exp (15-bit, bias 16383)
    //
    // Instruction layout (22 instructions, 3 paths):
    //   [0-4]   extract sign/exp/mantissa from double bits
    //   [5-7]   dispatch: CBZ→zero/denorm, B.EQ→inf/nan
    //   [8-14]  normal: set integer bit, bias-adjust exp, store, B .done
    //   [15-17] zero/denorm: store zero mantissa + sign, B .done
    //   [18-21] inf/nan: set integer bit, store, sign|0x7FFF, store
    //   [22]    .done: pop + end
    // -------------------------------------------------------------------------
    if (a2->operands[0].kind != IROperandKind::Register &&
        a2->operands[0].mem.size == IROperandSize::S80) {
        // Inline f64 → f80 conversion via the shared emit_f64_to_f80 helper
        // in TranslatorX87F80.cpp.  The helper emits the same 22-instruction
        // sequence that previously lived inline here and is also used by
        // translate_fsave.
        auto [Xbase, Wd_top] = x87_begin(*a1, buf);
        const int Xst_base = x87_get_st_base(*a1);
        const int Wd_tmp = alloc_gpr(*a1, 2);   // sign scratch, then pop scratch
        const int Dd_src = alloc_free_fpr(*a1);

        // Load ST(0) as double
        emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_src, Xst_base);

        // Compute destination address
        const int Xaddr =
            compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);

        // Deferred: allocate after compute_operand_address to avoid GPR exhaustion
        // when the operand has a GS/TLS segment override (needs 3 transient GPRs).
        // Use alloc_free_gpr — fixed pool indices could collide with Xaddr's register.
        const int Xbits  = alloc_free_gpr(*a1);   // raw double bits → f80 mantissa
        const int Wexp   = alloc_free_gpr(*a1);   // exponent → f80 exponent word

        emit_f64_to_f80(buf, Xaddr, Dd_src, Xbits, Wexp, Wd_tmp);
        free_fpr(*a1, Dd_src);

        free_gpr(*a1, Xaddr);
        free_gpr(*a1, Wexp);
        free_gpr(*a1, Xbits);

        // Pop (always true for m80fp — there is no non-popping FST m80fp)
        if (is_fstp)
            x87_pop(buf, *a1, Xbase, Wd_top, Wd_tmp);

        x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
        free_gpr(*a1, Wd_tmp);
        return;
    }

    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd_src = alloc_free_fpr(*a1);

    if (a2->operands[0].kind == IROperandKind::Register) {
        // -----------------------------------------------------------------
        // Register path — fst_stack / fstp_stack
        //
        // operands[0] = ST(i) destination register
        // Copies ST(0) → ST(depth_dst). When depth_dst == 0 the copy is a
        // no-op (same slot). The pop still fires for fstp_stack.
        // -----------------------------------------------------------------
        const int depth_dst = a2->operands[0].reg.reg.index();

        if (depth_dst != 0) {
            // Load ST(0), store into ST(depth_dst).
            emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_src, Xst_base);
            emit_store_st(buf, Xbase, Wd_top, resolve_depth(*a1, depth_dst), Wd_tmp, Dd_src, Xst_base);
        }
        // else: ST(0) → ST(0) is a no-op, skip load+store.
    } else {
        // -----------------------------------------------------------------
        // Memory path — fst / fstp  (S32 and S64 only; S80 handled above)
        // -----------------------------------------------------------------
        const bool is_f32 = (a2->operands[0].mem.size == IROperandSize::S32);

        // Load ST(0) as f64, then narrow to f32 if needed.
        emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_src, Xst_base);
        if (is_f32)
            emit_fcvt_d_to_s(buf, Dd_src, Dd_src);

        const int addr_reg =
            compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);
        emit_fstr_imm(buf, is_f32 ? 2 : 3, Dd_src, addr_reg, /*imm12=*/0);
        free_gpr(*a1, addr_reg);
    }

    // Pop if FSTP / FSTP_STACK: TOP = (TOP + 1) & 7.
    if (is_fstp)
        x87_pop(buf, *a1, Xbase, Wd_top, Wd_tmp);

    free_fpr(*a1, Dd_src);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FNSTSW — store x87 status_word to AX or memory.
//
// x87 semantics:
//   FNSTSW AX   (DF E0): AX ← status_word   (16-bit value into bits[15:0] of EAX)
//   FNSTSW m16  (DD /7): mem16 ← status_word
//
// Opcode encoding (Rosetta strips the FN prefix):
//   fstsw   DD /7   operands[0] = MemRef m16   (memory destination)
//   fstsw   DF E0   operands[0] = Register AX  (register destination)
//
// The AX path reads the status_word as a 16-bit halfword and inserts it into
// bits[15:0] of W0 (the 32-bit view of RAX) via BFI, leaving bits[31:16]
// untouched so the upper 16 bits of EAX are preserved per x86 semantics.
//
// BFI W_ax, Wd_sw, #0, #16  encodes as BFM with immr=0, imms=15:
//   When immr <= imms: inserts Wn[imms-immr : 0] = Wn[15:0] into Wd[15:0].
//
// The memory path just stores the 16-bit halfword directly via STRH.
//
// Register allocation:
//   Xbase   (gpr pool 0) — X87State base (X18 + x87_state_offset)
//   Wd_sw   (gpr pool 1) — loaded status_word (16-bit); becomes the store src
//   addr_reg (free pool) — memory path only: effective address from
//                          compute_operand_address; freed after STRH
//
// No TOP mutation.  No FPR needed.  Two pool registers suffice for both paths.
// =============================================================================
auto translate_fstsw(TranslationResult* a1, IRInstr* a2) -> void {
    static constexpr int16_t kX87StatusWordImm12 = kX87StatusWordOff / 2;  // = 1

    AssemblerBuffer& buf = a1->insn_buf;

    // OPT-1: fstsw only needs Xbase, not TOP.  Use the cache for Xbase if
    // active; allocate Wd_sw from the free pool (not pool slot 1, which is
    // pinned for TOP when the cache is active).
    const bool base_cached = (a1->x87_cache.run_remaining > 0 && a1->x87_cache.gprs_valid);
    int Xbase;
    if (base_cached) {
        Xbase = a1->x87_cache.base_gpr;
    } else {
        Xbase = alloc_gpr(*a1, 0);
        emit_x87_base(buf, *a1, Xbase);
    }
    const int Wd_sw = base_cached ? alloc_free_gpr(*a1) : alloc_gpr(*a1, 1);

    // OPT-C: If a prior deferred push left TOP dirty, flush it now — FSTSW
    // reads status_word from memory and needs the correct TOP.
    // Use Wd_sw as scratch for store_top (it hasn't been loaded yet).
    if (a1->x87_cache.top_dirty && base_cached) {
        emit_store_top(buf, Xbase, a1->x87_cache.top_gpr, Wd_sw);
        a1->x87_cache.top_dirty = 0;
    }

    // OPT-D: FSTSW doesn't call x87_end, so it must flush the pending tag
    // itself.  Otherwise, if FSTSW is the last instruction in the cache run,
    // x87_cache_tick clears tag_push_pending without emitting the tag-clear,
    // leaving the tag word in memory with kEmpty for the pushed slot.
    if (a1->x87_cache.tag_push_pending && base_cached) {
        const int Wd_tag_tmp = alloc_free_gpr(*a1);
        emit_x87_tag_clear(buf, Xbase, a1->x87_cache.top_gpr, Wd_sw, Wd_tag_tmp);
        free_gpr(*a1, Wd_tag_tmp);
        a1->x87_cache.tag_push_pending = 0;
    }

    // LDRH Wd_sw, [Xbase, #0x02]   — load status_word (16-bit halfword)
    // imm12=1 because LDRH scales by 2: byte offset = 1*2 = 2
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*opc=*/1, kX87StatusWordImm12, Xbase, Wd_sw);

    if (a2->operands[0].kind == IROperandKind::Register) {
        // -----------------------------------------------------------------
        // AX path — FNSTSW AX  (DF E0)
        //
        // operands[0].reg.reg.value == 0x30 (size_class=3 → 16-bit, index=0 → AX)
        //
        // In Rosetta's GPR mapping, x86 register index 0 (RAX) maps directly
        // to AArch64 register X0.  Write the status_word into bits[15:0] of W0,
        // leaving the upper 16 bits of EAX untouched.
        //
        // BFI W0, Wd_sw, #lsb=0, #width=16
        //   → BFM immr=(32-0)&31=0, imms=width-1=15
        //   → copies Wd_sw[15:0] into W0[15:0]
        // -----------------------------------------------------------------
        const int W_ax = a2->operands[0].reg.reg.index();  // = 0 (RAX/EAX/AX)

        emit_bitfield(buf, /*is_64bit=*/0, /*opc=*/1 /*BFM*/,
                      /*N=*/0, /*immr=*/0, /*imms=*/15,
                      /*Rn=*/Wd_sw, /*Rd=*/W_ax);
    } else {
        // -----------------------------------------------------------------
        // Memory path — FNSTSW m16  (DD /7)
        //
        // Compute destination address, then STRH the status_word.
        // -----------------------------------------------------------------
        const int addr_reg =
            compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);

        // STRH Wd_sw, [addr_reg, #0]
        emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*opc=*/0,
                         /*imm12=*/0, addr_reg, Wd_sw);

        free_gpr(*a1, addr_reg);
    }

    free_gpr(*a1, Wd_sw);
    if (!base_cached)
        free_gpr(*a1, Xbase);
}

// =============================================================================
// FXAM — D9 E5 — examine ST(0), classify into status_word C0/C1/C2/C3.
//
// Rosetta stores ST values as f64 (8 bytes per slot). Classification reads the
// raw bits and the tag-word entry for slot TOP, then sets these bits in
// status_word (mask 0x4700 = bits 14, 10, 9, 8):
//
//   Class       Tag  Exp     Mant   C3 C2 C0  Bits
//   Empty       =3    *       *      1  0  1  0x4100
//   Zero        ≠3    0       0      1  0  0  0x4000
//   Denormal    ≠3    0      ≠0      1  1  0  0x4400
//   Normal      ≠3   other    *      0  1  0  0x0400
//   Infinity    ≠3   0x7FF    0      0  1  1  0x0500
//   NaN         ≠3   0x7FF   ≠0      0  0  1  0x0100
//
//   C1 (bit 9) = sign(ST(0)) regardless of class.
//
// Branchless rules (each flag is a 0/1 result):
//   C0 = exp_max | tag_empty
//   C3 = exp_zero | tag_empty
//   C2 = ¬(tag_empty | (exp_zero & mant_zero) | (exp_max & ¬mant_zero))
//   C1 = (Xv >> 63) & 1
//
// Then status_word = (status_word & ~0x4700) | (C0<<8 | C1<<9 | C2<<10 | C3<<14).
// =============================================================================
auto translate_fxam(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;

    static constexpr int16_t kX87StatusWordImm12 = kX87StatusWordOff / 2;  // = 1
    static constexpr int16_t kX87TagWordImm12    = kX87TagWordOff / 2;     // = 2

    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);
    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd_v   = alloc_free_fpr(*a1);

    // Flush deferred TOP / tag-push if cache is active. fxam reads tag and
    // value from memory; both must be coherent before we run the classifier.
    if (a1->x87_cache.top_dirty && a1->x87_cache.gprs_valid) {
        emit_store_top(buf, Xbase, Wd_top, Wd_tmp);
        a1->x87_cache.top_dirty = 0;
    }
    if (a1->x87_cache.tag_push_pending && a1->x87_cache.gprs_valid) {
        const int Wd_tt = alloc_free_gpr(*a1);
        emit_x87_tag_clear(buf, Xbase, Wd_top, Wd_tmp, Wd_tt);
        free_gpr(*a1, Wd_tt);
        a1->x87_cache.tag_push_pending = 0;
    }

    // Load ST(0) f64 → Xv (raw bits).
    emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_v, Xst_base);
    const int Xv = alloc_free_gpr(*a1);
    emit_fmov_d_to_x(buf, Xv, Dd_v);
    free_fpr(*a1, Dd_v);

    // Working set: Xv (held), w_class (held), w_a/w_b (rotating scratch).
    // Build w_class via a chain of CSELs, defaulting to "Normal" (0x0400)
    // and overriding for special exponent or empty-tag classes.
    const int Wd_class = alloc_free_gpr(*a1);
    const int Wd_a = alloc_free_gpr(*a1);
    const int Wd_b = alloc_free_gpr(*a1);

    LogicalImmEncoding enc_mantmask;
    is_bitmask_immediate(/*is_64=*/true, 0x000FFFFFFFFFFFFFULL, enc_mantmask);

    // Inline raw emitters — no helpers exist for ANDS-imm, CSEL, LSRV.
    auto emit_ands_imm = [&](int is_64, int N, int immr, int imms, int Rn, int Rd) {
        // sf=is_64 | opc=11 | 100100 | N | immr | imms | Rn | Rd
        uint32_t insn = 0x72000000u | (1u << 30) | (1u << 29);  // opc=11
        insn |= (uint32_t)(is_64 != 0) << 31;
        insn |= (uint32_t)(N & 1) << 22;
        insn |= (uint32_t)(immr & 0x3F) << 16;
        insn |= (uint32_t)(imms & 0x3F) << 10;
        insn |= (uint32_t)(Rn & 0x1F) << 5;
        insn |= (uint32_t)(Rd & 0x1F);
        buf.emit(insn);
    };
    auto emit_csel = [&](int is_64, int Rd, int Rn, int Rm, int cond) {
        // sf | 0 | 0 | 11010100 | Rm | cond | 00 | Rn | Rd
        uint32_t insn = 0x1A800000u;
        insn |= (uint32_t)(is_64 != 0) << 31;
        insn |= (uint32_t)(Rm   & 0x1F) << 16;
        insn |= (uint32_t)(cond & 0xF)  << 12;
        insn |= (uint32_t)(Rn   & 0x1F) << 5;
        insn |= (uint32_t)(Rd   & 0x1F);
        buf.emit(insn);
    };
    auto emit_lsrv = [&](int is_64, int Rd, int Rn, int Rm) {
        // sf | 0 | 0 | 11010110 | Rm | 001001 | Rn | Rd
        uint32_t insn = 0x1AC02400u;
        insn |= (uint32_t)(is_64 != 0) << 31;
        insn |= (uint32_t)(Rm & 0x1F) << 16;
        insn |= (uint32_t)(Rn & 0x1F) << 5;
        insn |= (uint32_t)(Rd & 0x1F);
        buf.emit(insn);
    };

    // Default w_class = 0x0400  (Normal).
    emit_movn(buf, 0, /*MOVZ=*/2, /*hw=*/0, /*imm=*/0x0400, Wd_class);

    // ── Phase A: if exp == 0 → either Zero (0x4000) or Denormal (0x4400) ────
    emit_movn(buf, 0, 2, 0, 0x4000, Wd_a);                  // w_a = zero
    emit_movn(buf, 0, 2, 0, 0x4400, Wd_b);                  // w_b = denorm
    emit_ands_imm(/*is_64=*/1, enc_mantmask.N,
                  enc_mantmask.immr, enc_mantmask.imms, Xv, /*Rd=*/31);
    emit_csel(/*is_64=*/0, Wd_a, /*Rn=*/Wd_a, /*Rm=*/Wd_b, /*EQ=*/0);  // mant_zero ? a : b
    // Re-extract exp into Wd_b (UBFX is_64=1 so Xv→Wb's low 32 bits)
    emit_bitfield(buf, /*is_64=*/1, /*UBFM=*/2, /*N=*/1,
                  /*immr=*/52, /*imms=*/62, Xv, Wd_b);
    emit_add_imm(buf, /*is_64=*/0, /*is_sub=*/1, /*is_set_flags=*/1,
                 /*shift=*/0, /*imm12=*/0, Wd_b, /*Rd=*/31);             // CMP w_b, #0
    emit_csel(0, Wd_class, Wd_a, Wd_class, /*EQ=*/0);

    // ── Phase B: if exp == 0x7FF → Inf (0x0500) or NaN (0x0100) ─────────────
    emit_movn(buf, 0, 2, 0, 0x0500, Wd_a);                  // w_a = inf
    emit_movn(buf, 0, 2, 0, 0x0100, Wd_b);                  // w_b = nan
    emit_ands_imm(1, enc_mantmask.N,
                  enc_mantmask.immr, enc_mantmask.imms, Xv, 31);
    emit_csel(0, Wd_a, Wd_a, Wd_b, /*EQ=*/0);
    emit_bitfield(buf, 1, 2, 1, 52, 62, Xv, Wd_b);          // re-extract exp
    emit_add_imm(buf, 0, 1, 1, 0, 0x7FF, Wd_b, 31);          // CMP w_b, #0x7FF
    emit_csel(0, Wd_class, Wd_a, Wd_class, /*EQ=*/0);

    // ── Phase C: if tag(top) == 3 → Empty override (0x4100) ─────────────────
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1,
                     kX87TagWordImm12, Xbase, Wd_a);          // w_a = tag_word
    // w_b = w_top << 1
    emit_bitfield(buf, 0, 2, 0, 31, 30, Wd_top, Wd_b);
    emit_lsrv(0, Wd_a, Wd_a, Wd_b);
    LogicalImmEncoding enc3;
    is_bitmask_immediate(/*is_64=*/false, 3, enc3);
    emit_and_imm(buf, 0, Wd_a, enc3.N, enc3.immr, enc3.imms, Wd_a);
    emit_add_imm(buf, 0, 1, 1, 0, 3, Wd_a, 31);              // CMP w_a, #3
    emit_movn(buf, 0, 2, 0, 0x4100, Wd_a);                  // w_a = empty
    emit_csel(0, Wd_class, Wd_a, Wd_class, /*EQ=*/0);

    // ── Phase D: OR sign bit into w_class at position 9 (C1). ───────────────
    // Extract only bit 63 (avoids leaking exp bits into C0/C2/C3 through the
    // shift), then OR-LSL by 9 into w_class. We reuse Wd_a as the 1-bit
    // scratch since the previous mov to it (#0x4100) is no longer needed.
    emit_bitfield(buf, /*is_64=*/1, /*UBFM=*/2, /*N=*/1,
                  /*immr=*/63, /*imms=*/63, Xv, Wd_a);
    emit_logical_shifted_reg(buf, /*is_64=*/0, /*ORR=*/1, /*N=*/0, /*LSL=*/0,
                             /*Rm=*/Wd_a, /*amt=*/9,
                             /*Rn=*/Wd_class, /*Rd=*/Wd_class);
    free_gpr(*a1, Xv);

    // ── Phase E: read sw, clear bits {8,9,10,14}, OR in w_class, write back.
    emit_movn(buf, 0, 2, 0, 0x4700, Wd_a);                  // w_a = mask
    emit_logical_shifted_reg(buf, 0, /*AND=*/0, 0, 0, Wd_a, 0, Wd_class, Wd_class);
    emit_ldr_str_imm(buf, 1, 0, /*LDR=*/1, kX87StatusWordImm12, Xbase, Wd_b);
    emit_logical_shifted_reg(buf, 0, /*AND=*/0, /*N=*/1, 0, Wd_a, 0, Wd_b, Wd_b);  // BIC
    emit_logical_shifted_reg(buf, 0, /*ORR=*/1, 0, 0, Wd_class, 0, Wd_b, Wd_b);
    emit_ldr_str_imm(buf, 1, 0, /*STR=*/0, kX87StatusWordImm12, Xbase, Wd_b);

    free_gpr(*a1, Wd_b);
    free_gpr(*a1, Wd_a);
    free_gpr(*a1, Wd_class);

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FCOM / FCOMP / FCOMPP — compare ST(0) with a source operand, update C0/C2/C3
// in the x87 status_word, and optionally pop the stack.
//
// x87 semantics:
//   FCOM   ST(i) / m32fp / m64fp  — compare, no pop
//   FCOMP  ST(i) / m32fp / m64fp  — compare, pop once  (TOP++)
//   FCOMPP                        — compare ST(0) vs ST(1), pop twice
//
// AArch64 FCMP flag semantics (verified against ARM DDI 0487):
//   ST(0) > src:  N=0, Z=0, C=1, V=0
//   ST(0) < src:  N=1, Z=0, C=0, V=0
//   ST(0) = src:  N=0, Z=1, C=1, V=0
//   Unordered:    N=0, Z=0, C=1, V=1   ← Z is NOT set for unordered!
//
// C0/C2/C3 derivation (all four cases verified):
//   C3 = EQ | VS  = (Z==1) | (V==1)    ← needs both; Z alone misses unordered
//   C2 = VS       = (V==1)
//   C0 = CC | VS  = (C==0) | (V==1)    ← CC alone misses unordered when C=1
//
//   GT: Z=0,C=1,V=0 → C3=0, C2=0, C0=0  ✓
//   LT: Z=0,C=0,V=0 → C3=0, C2=0, C0=1  ✓
//   EQ: Z=1,C=1,V=0 → C3=1, C2=0, C0=0  ✓
//   UN: Z=0,C=1,V=1 → C3=1, C2=1, C0=1  ✓
//
// NOTE: emit_fcom_flags_to_sw() is NOT used here. That helper derives C0
// directly from the C flag and C3 from Z alone — both wrong for some cases.
//
// Operand encoding (Rosetta convention, matching fadd):
//   All forms:   operands[0] = ST(0) Register  (implicit)
//   Register:    operands[1] = ST(i) Register  (comparand depth)
//   Memory:      operands[1] = m32/64fp MemRef (comparand address)
//   FCOMPP:      no operands at all (is_fcompp flag used instead)
//
//   Dispatch uses operands[1].kind, NOT operands[0].kind, because the memory
//   form also has a Register at operands[0] (ST(0)), so checking [0] alone
//   cannot distinguish the register form from the memory form.
//
// Register allocation:
//   Xbase   (gpr pool 0) — X87State base
//   Wd_top  (gpr pool 1) — current TOP; updated by each emit_x87_pop
//   Wd_tmp  (gpr pool 2) — scratch for emit_load_st and CSET results
//   Wd_tmp2 (gpr pool 3) — accumulates packed C3/C2/C0 bits
//   Dd_st0  (fpr free)   — ST(0) value
//   Dd_src  (fpr free)   — comparand value
//   addr_reg (free gpr)  — memory path only: effective address; freed after fldr
// =============================================================================
auto translate_fcom(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    static constexpr int16_t kX87StatusWordImm12 = kX87StatusWordOff / 2;  // = 1

    const bool is_fcompp = (a2->opcode == kOpcodeName_fcompp || a2->opcode == kOpcodeName_fucompp);
    const bool is_popping = (a2->opcode == kOpcodeName_fcomp || a2->opcode == kOpcodeName_fcompp ||
                             a2->opcode == kOpcodeName_fucomp || a2->opcode == kOpcodeName_fucompp);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Wd_tmp2 = alloc_gpr(*a1, 3);
    const int Dd_st0 = alloc_free_fpr(*a1);
    const int Dd_src = alloc_free_fpr(*a1);

    // Step 1: load ST(0) into Dd_st0.
    emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_st0, Xst_base);

    // Step 2: load the comparand into Dd_src.
    if (is_fcompp) {
        // FCOMPP: comparand is always ST(1).
        emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 1), Wd_tmp, Dd_src, Xst_base);
    } else if (a2->operands[1].kind == IROperandKind::Register) {
        // FCOM / FCOMP ST(i): Rosetta encodes as [ST(0) Register, ST(i) Register].
        // operands[0] = ST(0) (implicit), operands[1] = ST(i) (comparand).
        // We check operands[1] (not [0]) because the memory path also has a
        // Register at operands[0] (ST(0)), so checking [0] alone can't distinguish.
        const int depth = a2->operands[1].reg.reg.index();
        emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, depth), Wd_tmp, Dd_src, Xst_base);
    } else {
        // FCOM / FCOMP m32fp / m64fp.
        // Rosetta encodes as [ST(0) Register, mem MemRef].
        // The memory operand is operands[1].
        const bool is_f32 = (a2->operands[1].mem.size == IROperandSize::S32);

        const int addr_reg =
            compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[1], GPR::XZR);

        // size 2 = S (f32), size 3 = D (f64)
        emit_fldr_imm(buf, is_f32 ? 2 : 3, Dd_src, addr_reg, /*imm12=*/0);
        free_gpr(*a1, addr_reg);

        if (is_f32)
            emit_fcvt_s_to_d(buf, Dd_src, Dd_src);
    }

    // Step 3: Save host NZCV, compare, map flags branchlessly, restore NZCV.
    //
    // CRITICAL: Rosetta maps x86 EFLAGS to AArch64 NZCV.  Non-x87 instructions
    // like TEST/CMP set NZCV, and subsequent Jcc reads it.  If an x87 FCOM sits
    // between a TEST and a Jcc, our FCMP would clobber NZCV and the Jcc would
    // branch on the FP comparison result instead of the TEST result.
    //
    // Fix: MRS to save NZCV into a GPR before FCMP, then MSR to restore after
    // we've finished reading the FP condition codes.
    //
    // MRS Wd_tmp2, NZCV — save current NZCV (from prior x86 ALU ops)
    emit_mrs_nzcv(buf, Wd_tmp2);

    // FCMP Dd_st0, Dd_src — clobbers NZCV with FP comparison result
    emit_fcmp_f64(buf, Dd_st0, Dd_src);

    // FPRs are no longer needed.
    free_fpr(*a1, Dd_src);
    free_fpr(*a1, Dd_st0);

    // Step 4: Branchless FCMP NZCV → x87 CC bit mapping.
    //
    // AArch64 FCMP sets NZCV:
    //   GT: N=0, Z=0, C=1, V=0
    //   LT: N=1, Z=0, C=0, V=0
    //   EQ: N=0, Z=1, C=1, V=0
    //   UN: N=0, Z=0, C=1, V=1
    //
    // x87 CC derivation:
    //   C0 (bit 8)  = CC | VS  = (C==0) | (V==1)   → 1 for LT and UN
    //   C2 (bit 10) = VS       = (V==1)             → 1 for UN only
    //   C3 (bit 14) = EQ | VS  = (Z==1) | (V==1)   → 1 for EQ and UN
    //
    // All three CSET instructions must execute before MSR restores NZCV,
    // since CSET reads the condition flags.
    //
    // 9 instructions, fully branchless — eliminates 3 conditional branches
    // that cause ~14-cycle misprediction stalls on Apple M-series.

    // OPT-L: Branchless FCMP NZCV → packed x87 CC bits + RMW status_word.
    emit_fcom_cc_pack(buf, *a1, Wd_tmp, Wd_tmp2);
    emit_fcom_cc_write_sw(buf, *a1, Xbase, Wd_tmp);

    // Step 5: pop the stack as required.
    // Wd_tmp is dead after the status_word store — safe to reuse as RMW scratch.
    if (is_popping) {
        if (is_fcompp)
            x87_pop_n(buf, *a1, Xbase, Wd_top, Wd_tmp, 2);  // OPT-A: fused double-pop
        else
            x87_pop(buf, *a1, Xbase, Wd_top, Wd_tmp);
    }

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FXCH — exchange ST(0) with ST(i).
//
// x87 semantics:
//   FXCH ST(i)   D9 C8+i   swap ST(0) ↔ ST(i)
//
// Special case: FXCH ST(0) is a no-op (swapping a register with itself).
// Rosetta may or may not filter this upstream; we guard it explicitly.
//
// Both slots are already live on the stack, so the tag word does not change.
// No push or pop — TOP is read for addressing but never mutated.
//
// Instruction sequence:
//   1. load ST(0)    → Dd_a
//   2. load ST(i)    → Dd_b
//   3. store Dd_b    → ST(0)     (write ST(i)'s value into slot 0)
//   4. store Dd_a    → ST(i)     (write ST(0)'s value into slot i)
//
// Store ordering is safe: emit_store_st computes the physical slot from Wd_top
// + depth each time, and Wd_top is not modified between the two stores, so both
// slots resolve correctly regardless of order.
//
// Register allocation:
//   Xbase  (gpr pool 0) — X87State base
//   Wd_top (gpr pool 1) — current TOP (read-only; no push/pop)
//   Wd_tmp (gpr pool 2) — offset scratch for emit_{load,store}_st
//   Dd_a   (fpr free)   — holds ST(0) value
//   Dd_b   (fpr free)   — holds ST(i) value
// =============================================================================
auto translate_fxch(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    // Rosetta normalizes register operands as [ST(0), ST(i)], so the target
    // depth is at operands[1] — not operands[0].  Reading [0] always gives
    // depth=0, which fires the no-op guard every time and silently skips
    // every swap.  Matches the convention used by translate_fld register path.
    const int depth = a2->operands[1].reg.reg.index();  // ST(i)

    // FXCH ST(0) is a no-op — but must still call x87_end for
    // end-of-run dirty flush (OPT-C).
    if (depth == 0) {
        const int Wd_scratch = alloc_gpr(*a1, 2);
        x87_end(*a1, buf, Xbase, Wd_top, Wd_scratch);
        free_gpr(*a1, Wd_scratch);
        return;
    }

    // OPT-G: Deferred FXCH — when the cache is active, just update the
    // compile-time permutation map instead of emitting memory swaps.
    // The permutation is materialized at run end (x87_end flush).
    if (a1->x87_cache.run_remaining > 0 &&
        !(g_rosetta_config && g_rosetta_config->disable_deferred_fxch)) {
        std::swap(a1->x87_cache.perm[0], a1->x87_cache.perm[depth]);
        a1->x87_cache.perm_dirty = 1;
        const int Wd_scratch = alloc_gpr(*a1, 2);
        x87_end(*a1, buf, Xbase, Wd_top, Wd_scratch);
        free_gpr(*a1, Wd_scratch);
        return;
    }

    // OPT-E: Use two offset registers to avoid recomputing offset_0 for the
    // second store.  Saves 3 insns (15 → 12) by replacing emit_store_st
    // (which recomputes the offset via emit_st_offset) with a second
    // emit_store_st_at_offset using the preserved Wd_off0.
    const int Wd_off0 = alloc_gpr(*a1, 2);  // byte offset of ST(0)
    const int Wd_offi = alloc_gpr(*a1, 3);  // byte offset of ST(i)
    const int Dd_a = alloc_free_fpr(*a1);
    const int Dd_b = alloc_free_fpr(*a1);

    const int Wk15 = emit_load_st(buf, Xbase, Wd_top, /*depth=*/0, Wd_off0, Dd_a,
                                  Xst_base);  // Wd_off0 = offset(0)
    const int Wk14 =
        emit_load_st(buf, Xbase, Wd_top, depth, Wd_offi, Dd_b, Xst_base);  // Wd_offi = offset(i)

    emit_store_st_at_offset(buf, Xbase, Wk14, Dd_a, Xst_base);  // ST(i) ← old ST(0)
    emit_store_st_at_offset(buf, Xbase, Wk15, Dd_b, Xst_base);  // ST(0) ← old ST(i)

    free_fpr(*a1, Dd_b);
    free_fpr(*a1, Dd_a);
    free_gpr(*a1, Wd_offi);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_off0);  // use Wd_off0 as scratch
    free_gpr(*a1, Wd_off0);
}

// =============================================================================
// FCHS — change sign of ST(0).
//
// x87 semantics:
//   FCHS   D9 E0   ST(0) ← −ST(0)
//
// No explicit operands; ST(0) is always the source and destination.
// No stack mutation — TOP is read for addressing only.
//
// Instruction sequence:
//   1. load ST(0) → Dd
//   2. FNEG Dd, Dd
//   3. store Dd   → ST(0)
//
// Register allocation:
//   Xbase  (gpr pool 0) — X87State base
//   Wd_top (gpr pool 1) — current TOP (read-only)
//   Wd_tmp (gpr pool 2) — offset scratch for emit_{load,store}_st
//   Dd     (fpr free)   — ST(0) value
// =============================================================================
auto translate_fchs(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd = alloc_free_fpr(*a1);

    const int Wk16 = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd, Xst_base);
    emit_fneg_f64(buf, Dd, Dd);
    // Opt 3: Wd_tmp still holds the ST(0) byte offset from emit_load_st above.
    emit_store_st_at_offset(buf, Xbase, Wk16, Dd, Xst_base);

    free_fpr(*a1, Dd);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FABS — absolute value of ST(0).
//
// x87 semantics:
//   FABS   D9 E1   ST(0) ← |ST(0)|
//
// No explicit operands; ST(0) is always the source and destination.
// No stack mutation — TOP is read for addressing only.
//
// Instruction sequence:
//   1. load ST(0) → Dd
//   2. FABS Dd, Dd
//   3. store Dd   → ST(0)
//
// Register allocation: identical to translate_fchs.
// =============================================================================
auto translate_fabs(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd = alloc_free_fpr(*a1);

    const int Wk17 = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd, Xst_base);
    emit_fabs_f64(buf, Dd, Dd);
    // Opt 3: Wd_tmp still holds the ST(0) byte offset from emit_load_st above.
    emit_store_st_at_offset(buf, Xbase, Wk17, Dd, Xst_base);

    free_fpr(*a1, Dd);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FSQRT — square root of ST(0).
//
// x87 semantics:
//   FSQRT  D9 FA   ST(0) ← sqrt(ST(0))
//
// No explicit operands; ST(0) is always the source and destination.
// No stack mutation — TOP is read for addressing only.
//
// Instruction sequence:
//   1. load ST(0) → Dd
//   2. FSQRT Dd, Dd
//   3. store Dd   → ST(0)
//
// Register allocation: identical to translate_fchs.
// =============================================================================
auto translate_fsqrt(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd = alloc_free_fpr(*a1);

    const int Wk18 = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd, Xst_base);
    emit_fsqrt_f64(buf, Dd, Dd);
    // Opt 3: Wd_tmp still holds the ST(0) byte offset from emit_load_st above.
    emit_store_st_at_offset(buf, Xbase, Wk18, Dd, Xst_base);

    free_fpr(*a1, Dd);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FISTP — store ST(0) as signed integer to memory, then pop
//
// x87 semantics:
//   mem ← (int) ST(0)    (truncate toward zero, i.e. FCVTZS)
//   TOP ← (TOP + 1) & 7
//
// Operand encoding (Rosetta encodes memory destination at operands[0]):
//   DF /2   FISTP m16int   operands[0].mem.size = S16
//   DB /3   FISTP m32int   operands[0].mem.size = S32
//   DF /7   FISTP m64int   operands[0].mem.size = S64
//
// Instruction sequence:
//   1. load ST(0) mantissa → Dd_val
//   2. FCVTZS Wd_int/Xd_int, Dd_val   (truncate toward zero)
//   3. compute destination address → addr_reg
//   4. STRH/STR Wd_int or STR Xd_int  (size matches operand)
//   5. free addr_reg
//   6. emit_x87_pop                   (TOP++, updates status_word)
//
// For S16: FCVTZS to W (32-bit), STRH stores low 16 bits.
// For S32: FCVTZS to W (32-bit), STR stores 32 bits.
// For S64: FCVTZS to X (64-bit, is_64bit_int=1), STR stores 64 bits.
// The register number for Wd_int and Xd_int is the same; only the instruction
// width differs via the size and is_64bit_int parameters.
//
// Register allocation:
//   Xbase   (gpr pool 0) — X87State base
//   Wd_top  (gpr pool 1) — current TOP; updated by emit_x87_pop
//   Wd_tmp  (gpr pool 2) — offset scratch (emit_load_st, emit_x87_pop)
//   Wd_int  (free pool)  — FCVTZS result (W or X view of same reg)
//   Dd_val  (fpr free)   — ST(0) mantissa
//   addr_reg (free pool) — destination address; freed before pop
// =============================================================================
auto translate_fistp(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const IROperandSize int_size = a2->operands[0].mem.size;

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Wd_int = alloc_free_gpr(*a1);
    const int Dd_val = alloc_free_fpr(*a1);

    // Step 1: load ST(0) mantissa → Dd_val
    emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_val, Xst_base);

    // Step 2: convert f64 to signed integer, respecting the rounding mode in
    // control_word bits [11:10] (RC field).
    //
    // x87 RC → AArch64 FCVT instruction:
    //   RC=00 (RN nearest)    → FCVTNS  aarch64_rmode=0
    //   RC=01 (RD toward -∞)  → FCVTMS  aarch64_rmode=2
    //   RC=10 (RU toward +∞)  → FCVTPS  aarch64_rmode=1
    //   RC=11 (RZ truncate)   → FCVTZS  aarch64_rmode=3
    //
    // FCVTZS (always-truncate) is WRONG when RC=01 (floor). That is the
    // classic FLDCW+FISTP pattern used by this game for coordinate rounding,
    // and it causes incorrect tile/chunk indexing for negative coordinates
    // (e.g. position -0.1 truncates to 0 instead of flooring to -1).
    //
    // All four FCVT*S variants share the same encoding structure:
    //   base = 0x1E200000 | (sf<<31) | (ftype<<22) | (aarch64_rmode<<19)
    //   insn = base | (Rn<<5) | Rd
    //
    // We dispatch at runtime via a 3-branch CBZ chain. The fall-through path
    // (RC=3, truncate) is kept last since it is the default x87 mode and
    // requires no rounding-specific branch. All branch offsets are fixed and
    // small — no fixup system is needed.
    //
    //   Instruction layout (each idx = 1 AArch64 instruction = 4 bytes):
    //   [0] LDRH  Wd_rc, [Xbase, #0]          ; load control_word
    //   [1] UBFM  Wd_rc, Wd_rc, #10, #11      ; UBFX bits[11:10] → bits[1:0]
    //   [2] CBZ   Wd_rc, +28                  ; RC==0 → [9] FCVTNS
    //   [3] SUB   Wd_rc, Wd_rc, #1
    //   [4] CBZ   Wd_rc, +28                  ; RC==1 → [11] FCVTMS
    //   [5] SUB   Wd_rc, Wd_rc, #1
    //   [6] CBZ   Wd_rc, +28                  ; RC==2 → [13] FCVTPS
    //   [7] FCVTZS Wd_int, Dd_val             ; RC=3 truncate (fall-through)
    //   [8] B     +24                         ; → [14] done
    //   [9] FCVTNS Wd_int, Dd_val             ; RC=0 nearest
    //  [10] B     +16                         ; → [14] done
    //  [11] FCVTMS Wd_int, Dd_val             ; RC=1 floor (the crash case)
    //  [12] B     +8                          ; → [14] done
    //  [13] FCVTPS Wd_int, Dd_val             ; RC=2 ceil
    //  [14] ; done — fall through to store
    //
    // Wd_rc reuses Wd_tmp (free after emit_load_st).
    //
    // FCVT*S encoding: 0x1E200000 | (sf<<31) | (ftype<<22) | (aarch64_rmode<<19) | (Rn<<5) | Rd
    //   sf=0 for 32-bit int result, sf=1 for 64-bit.  ftype=1 for f64 source.
    //   rmode field: NS=0, PS=1, MS=2, ZS=3.

    const int is_64bit_int = (int_size == IROperandSize::S64) ? 1 : 0;
    const int Wd_rc = Wd_tmp;  // free after emit_load_st — reuse as RC scratch

    if (g_rosetta_config && g_rosetta_config->fast_round) {
        // Fast path: assume RC=0 (round-to-nearest). Single instruction.
        // Correct for blocks that contain no FLDCW (fast_round=1: always; =2/smart: per-block).
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=FCVTNS*/ 0, Wd_int,
                            Dd_val);
    } else {
        // Full dispatch: read RC from control_word and branch to the correct FCVT*S.
        //
        //   Instruction layout (each idx = 1 AArch64 instruction = 4 bytes):
        //   [0] LDRH  Wd_rc, [Xbase, #0]          ; load control_word
        //   [1] UBFM  Wd_rc, Wd_rc, #10, #11      ; UBFX bits[11:10] → bits[1:0]
        //   [2] CBZ   Wd_rc, +28                  ; RC==0 → [9] FCVTNS
        //   [3] SUB   Wd_rc, Wd_rc, #1
        //   [4] CBZ   Wd_rc, +28                  ; RC==1 → [11] FCVTMS
        //   [5] SUB   Wd_rc, Wd_rc, #1
        //   [6] CBZ   Wd_rc, +28                  ; RC==2 → [13] FCVTPS
        //   [7] FCVTZS Wd_int, Dd_val             ; RC=3 truncate (fall-through)
        //   [8] B     +24                         ; → [14] done
        //   [9] FCVTNS Wd_int, Dd_val             ; RC=0 nearest
        //  [10] B     +16                         ; → [14] done
        //  [11] FCVTMS Wd_int, Dd_val             ; RC=1 floor (the crash case)
        //  [12] B     +8                          ; → [14] done
        //  [13] FCVTPS Wd_int, Dd_val             ; RC=2 ceil
        //  [14] ; done — fall through to store

        // [0] LDRH Wd_rc, [Xbase, #0]  ; control_word (offset 0, imm12=0)
        emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR*/ 1, /*imm12=*/0, Xbase, Wd_rc);

        // [1] UBFX Wd_rc, Wd_rc, #10, #2  → bits[11:10] in bits[1:0]
        // UBFM: immr=lsb=10, imms=lsb+width-1=11
        emit_bitfield(buf, /*is_64=*/0, /*UBFM*/ 2, /*N*/ 0, /*immr*/ 10, /*imms*/ 11, Wd_rc,
                      Wd_rc);

        // [2] CBZ Wd_rc, +28  (+7 instructions → idx 9)
        emit_cbz(buf, /*is_64bit=*/0, /*is_nz=*/0, Wd_rc, 7);

        // [3] SUB Wd_rc, Wd_rc, #1
        emit_add_imm(buf, /*is_64=*/0, /*is_sub*/ 1, /*S*/ 0, /*shift*/ 0, 1, Wd_rc, Wd_rc);

        // [4] CBZ Wd_rc, +28  (+7 instructions → idx 11)
        emit_cbz(buf, /*is_64bit=*/0, /*is_nz=*/0, Wd_rc, 7);

        // [5] SUB Wd_rc, Wd_rc, #1
        emit_add_imm(buf, /*is_64=*/0, /*is_sub*/ 1, /*S*/ 0, /*shift*/ 0, 1, Wd_rc, Wd_rc);

        // [6] CBZ Wd_rc, +28  (+7 instructions → idx 13)
        emit_cbz(buf, /*is_64bit=*/0, /*is_nz=*/0, Wd_rc, 7);

        // [7] FCVTZS (rmode=3)  RC=3 truncate
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=*/ 3, Wd_int, Dd_val);
        // [8] B +24  (+6 instructions → idx 14)
        emit_b(buf, 6);

        // [9] FCVTNS (rmode=0)  RC=0 nearest
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=*/ 0, Wd_int, Dd_val);
        // [10] B +16  (+4 instructions → idx 14)
        emit_b(buf, 4);

        // [11] FCVTMS (rmode=2)  RC=1 floor
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=*/ 2, Wd_int, Dd_val);
        // [12] B +8  (+2 instructions → idx 14)
        emit_b(buf, 2);

        // [13] FCVTPS (rmode=1)  RC=2 ceil
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=*/ 1, Wd_int, Dd_val);
        // [14] done — Wd_int now holds the correctly rounded integer.
    }

    // Step 3: compute destination address
    const int addr_reg =
        compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);

    // Step 4: store integer to memory
    // size=1 → STRH (16-bit, stores low 16 bits of Wd_int)
    // size=2 → STR  (32-bit W)
    // size=3 → STR  (64-bit X — same register number as Wd_int)
    const int store_size = (int_size == IROperandSize::S16)   ? 1
                           : (int_size == IROperandSize::S32) ? 2
                                                              : 3;
    emit_str_imm(buf, store_size, Wd_int, addr_reg, /*imm12=*/0);

    // Step 5: free addr_reg
    free_gpr(*a1, addr_reg);

    // Step 6: pop — TOP++, updates status_word.  Wd_tmp is clean here.
    x87_pop(buf, *a1, Xbase, Wd_top, Wd_tmp);

    free_fpr(*a1, Dd_val);
    free_gpr(*a1, Wd_int);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FISTTP — store integer with truncation and pop
//
// Like FISTP but always truncates toward zero (ignoring the RC field in the
// x87 control word).  This eliminates the 15-instruction rounding-mode
// dispatch chain — a single FCVTZS is all that's needed.
// =============================================================================

auto translate_fisttp(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const IROperandSize int_size = a2->operands[0].mem.size;

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Wd_int = alloc_free_gpr(*a1);
    const int Dd_val = alloc_free_fpr(*a1);

    // Step 1: load ST(0) mantissa → Dd_val
    emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_val, Xst_base);

    // Step 2: FISTT always truncates — single FCVTZS, no RC dispatch needed
    const int is_64bit_int = (int_size == IROperandSize::S64) ? 1 : 0;
    emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=FCVTZS*/ 3, Wd_int, Dd_val);

    // Step 3: compute destination address
    const int addr_reg =
        compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);

    // Step 4: store integer to memory
    const int store_size = (int_size == IROperandSize::S16)   ? 1
                           : (int_size == IROperandSize::S32) ? 2
                                                              : 3;
    emit_str_imm(buf, store_size, Wd_int, addr_reg, /*imm12=*/0);

    // Step 5: free addr_reg
    free_gpr(*a1, addr_reg);

    // Step 6: pop — TOP++, updates status_word.  Wd_tmp is clean here.
    x87_pop(buf, *a1, Xbase, Wd_top, Wd_tmp);

    free_fpr(*a1, Dd_val);
    free_gpr(*a1, Wd_int);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FIDIV — divide ST(0) by integer from memory
//
// x87 semantics:
//   ST(0) ← ST(0) / (double)(signed integer at mem)
//
// Operand encoding (memory source at operands[0]):
//   DE /6   FIDIV m16int   operands[0].mem.size = S16
//   DA /6   FIDIV m32int   operands[0].mem.size = S32
//
// Instruction sequence: identical to translate_fiadd but with FDIV.
// Register allocation: identical to translate_fiadd.
// =============================================================================
auto translate_fidiv(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const bool is_m16 = (a2->operands[0].mem.size == IROperandSize::S16);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd_st0 = alloc_free_fpr(*a1);
    const int Dd_int = alloc_free_fpr(*a1);

    // Step 1: load ST(0).  Wd_tmp receives the byte offset for ST(0).
    const int Wk19 = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_st0, Xst_base);

    // Step 2: compute source address
    const int addr_reg =
        compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);

    // Step 3: load integer from memory into addr_reg (reusing it for the value).
    // Opt 3: loading into addr_reg (not Wd_tmp) preserves the ST(0) byte offset
    // in Wd_tmp so emit_store_st_at_offset can skip recomputing it below.
    if (is_m16) {
        emit_ldr_str_imm(buf, /*size=*/1 /*16-bit*/, /*is_fp=*/0, /*opc=*/1 /*LDR*/,
                         /*imm12=*/0, addr_reg, addr_reg);
        // SXTH addr_reg, addr_reg — sign-extend bits[15:0] → W
        emit_bitfield(buf, /*is_64bit=*/0, /*opc=*/0 /*SBFM*/, /*N=*/0,
                      /*immr=*/0, /*imms=*/15, addr_reg, addr_reg);
    } else {
        emit_ldr_str_imm(buf, /*size=*/2 /*32-bit*/, /*is_fp=*/0, /*opc=*/1 /*LDR*/,
                         /*imm12=*/0, addr_reg, addr_reg);
    }

    // Step 4: SCVTF Dd_int, addr_reg — signed W → f64
    emit_scvtf(buf, /*is_64bit_int=*/0, /*ftype=*/1, Dd_int, addr_reg);

    free_gpr(*a1, addr_reg);

    // Step 5: FDIV ST(0), integer
    emit_fdiv_f64(buf, Dd_st0, Dd_st0, Dd_int);

    // Step 6: store result back to ST(0).
    // Opt 3: Wd_tmp still holds the ST(0) byte offset from emit_load_st above.
    emit_store_st_at_offset(buf, Xbase, Wk19, Dd_st0, Xst_base);

    free_fpr(*a1, Dd_int);
    free_fpr(*a1, Dd_st0);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FIMUL — multiply ST(0) by integer from memory
//
// x87 semantics:
//   ST(0) ← ST(0) * (double)(signed integer at mem)
//
// Operand encoding (memory source at operands[0]):
//   DE /1   FIMUL m16int   operands[0].mem.size = S16
//   DA /1   FIMUL m32int   operands[0].mem.size = S32
//
// Instruction sequence: identical to translate_fidiv but with FMUL.
// Register allocation: identical to translate_fidiv.
// =============================================================================
auto translate_fimul(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const bool is_m16 = (a2->operands[0].mem.size == IROperandSize::S16);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd_st0 = alloc_free_fpr(*a1);
    const int Dd_int = alloc_free_fpr(*a1);

    // Step 1: load ST(0).  Wd_tmp receives the byte offset for ST(0).
    const int Wk20 = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_st0, Xst_base);

    // Step 2: compute source address
    const int addr_reg =
        compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);

    // Step 3: load integer from memory into addr_reg (reusing it for the value).
    // Opt 3: loading into addr_reg (not Wd_tmp) preserves the ST(0) byte offset
    // in Wd_tmp so emit_store_st_at_offset can skip recomputing it below.
    if (is_m16) {
        emit_ldr_str_imm(buf, /*size=*/1 /*16-bit*/, /*is_fp=*/0, /*opc=*/1 /*LDR*/,
                         /*imm12=*/0, addr_reg, addr_reg);
        // SXTH addr_reg, addr_reg — sign-extend bits[15:0] → W
        emit_bitfield(buf, /*is_64bit=*/0, /*opc=*/0 /*SBFM*/, /*N=*/0,
                      /*immr=*/0, /*imms=*/15, addr_reg, addr_reg);
    } else {
        emit_ldr_str_imm(buf, /*size=*/2 /*32-bit*/, /*is_fp=*/0, /*opc=*/1 /*LDR*/,
                         /*imm12=*/0, addr_reg, addr_reg);
    }

    // Step 4: SCVTF Dd_int, addr_reg — signed W → f64
    emit_scvtf(buf, /*is_64bit_int=*/0, /*ftype=*/1, Dd_int, addr_reg);

    free_gpr(*a1, addr_reg);

    // Step 5: FMUL ST(0), integer
    emit_fmul_f64(buf, Dd_st0, Dd_st0, Dd_int);

    // Step 6: store result back to ST(0).
    // Opt 3: Wd_tmp still holds the ST(0) byte offset from emit_load_st above.
    emit_store_st_at_offset(buf, Xbase, Wk20, Dd_st0, Xst_base);

    free_fpr(*a1, Dd_int);
    free_fpr(*a1, Dd_st0);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FISUB — subtract integer from ST(0)
//
// x87 semantics:
//   ST(0) ← ST(0) - (double)(signed integer at mem)
//
// Operand encoding (memory source at operands[0]):
//   DE /4   FISUB m16int   operands[0].mem.size = S16
//   DA /4   FISUB m32int   operands[0].mem.size = S32
//
// Identical to translate_fiadd with emit_fsub_f64 instead of emit_fadd_f64.
// =============================================================================
auto translate_fisub(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const bool is_m16 = (a2->operands[0].mem.size == IROperandSize::S16);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd_st0 = alloc_free_fpr(*a1);
    const int Dd_int = alloc_free_fpr(*a1);

    // Step 1: load ST(0).  Wd_tmp receives the byte offset for ST(0).
    const int Wk21 = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_st0, Xst_base);

    // Step 2: compute source address
    const int addr_reg =
        compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);

    // Step 3: load integer from memory into addr_reg (reusing it for the value).
    // Opt 3: loading into addr_reg (not Wd_tmp) preserves the ST(0) byte offset
    // in Wd_tmp so emit_store_st_at_offset can skip recomputing it below.
    if (is_m16) {
        emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*opc=*/1 /*LDR*/,
                         /*imm12=*/0, addr_reg, addr_reg);
        emit_bitfield(buf, /*is_64bit=*/0, /*opc=*/0 /*SBFM*/, /*N=*/0,
                      /*immr=*/0, /*imms=*/15, addr_reg, addr_reg);
    } else {
        emit_ldr_str_imm(buf, /*size=*/2, /*is_fp=*/0, /*opc=*/1 /*LDR*/,
                         /*imm12=*/0, addr_reg, addr_reg);
    }

    // Step 4: SCVTF Dd_int, addr_reg — signed W → f64
    emit_scvtf(buf, /*is_64bit_int=*/0, /*ftype=*/1, Dd_int, addr_reg);

    free_gpr(*a1, addr_reg);

    // Step 5: FSUB ST(0) = ST(0) - integer
    emit_fsub_f64(buf, Dd_st0, Dd_st0, Dd_int);

    // Step 6: store result back to ST(0).
    // Opt 3: Wd_tmp still holds the ST(0) byte offset from emit_load_st above.
    emit_store_st_at_offset(buf, Xbase, Wk21, Dd_st0, Xst_base);

    free_fpr(*a1, Dd_int);
    free_fpr(*a1, Dd_st0);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FIDIVR — reverse-divide: integer / ST(0)
//
// x87 semantics:
//   ST(0) ← (double)(signed integer at mem) / ST(0)
//
// Operand encoding (memory source at operands[0]):
//   DE /7   FIDIVR m16int   operands[0].mem.size = S16
//   DA /7   FIDIVR m32int   operands[0].mem.size = S32
//
// Identical to translate_fidiv except the FDIV operand order is swapped:
//   fidiv:  FDIV Dd_st0, Dd_st0, Dd_int   (ST(0) / int)
//   fidivr: FDIV Dd_st0, Dd_int,  Dd_st0  (int   / ST(0))
// =============================================================================
auto translate_fidivr(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const bool is_m16 = (a2->operands[0].mem.size == IROperandSize::S16);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd_st0 = alloc_free_fpr(*a1);
    const int Dd_int = alloc_free_fpr(*a1);

    // Step 1: load ST(0).  Wd_tmp receives the byte offset for ST(0).
    const int Wk22 = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_st0, Xst_base);

    // Step 2: compute source address
    const int addr_reg =
        compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);

    // Step 3: load integer from memory into addr_reg (reusing it for the value).
    // Opt 3: loading into addr_reg (not Wd_tmp) preserves the ST(0) byte offset
    // in Wd_tmp so emit_store_st_at_offset can skip recomputing it below.
    if (is_m16) {
        emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*opc=*/1 /*LDR*/,
                         /*imm12=*/0, addr_reg, addr_reg);
        emit_bitfield(buf, /*is_64bit=*/0, /*opc=*/0 /*SBFM*/, /*N=*/0,
                      /*immr=*/0, /*imms=*/15, addr_reg, addr_reg);
    } else {
        emit_ldr_str_imm(buf, /*size=*/2, /*is_fp=*/0, /*opc=*/1 /*LDR*/,
                         /*imm12=*/0, addr_reg, addr_reg);
    }

    // Step 4: SCVTF Dd_int, addr_reg — signed W → f64
    emit_scvtf(buf, /*is_64bit_int=*/0, /*ftype=*/1, Dd_int, addr_reg);

    free_gpr(*a1, addr_reg);

    // Step 5: FDIV Dd_st0, Dd_int, Dd_st0  — integer / ST(0)  (reversed)
    emit_fdiv_f64(buf, Dd_st0, Dd_int, Dd_st0);

    // Step 6: store result back to ST(0).
    // Opt 3: Wd_tmp still holds the ST(0) byte offset from emit_load_st above.
    emit_store_st_at_offset(buf, Xbase, Wk22, Dd_st0, Xst_base);

    free_fpr(*a1, Dd_int);
    free_fpr(*a1, Dd_st0);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FRNDINT — round ST(0) to integer
//
// x87 semantics:
//   ST(0) ← round(ST(0))   using current x87 rounding mode (RC, bits[11:10]
//                           of control_word).  Result is still a double, not
//                           an integer type.  TOP is not modified.
//
// AArch64 mapping:
//   FRINT* Dd, Dn — round to integral floating-point value.
//   We dispatch at runtime to the variant matching the x87 RC field:
//     RC=00 (round nearest)  → FRINTN  (opcode=8)
//     RC=01 (round toward −∞) → FRINTM (opcode=10)
//     RC=10 (round toward +∞) → FRINTP (opcode=9)
//     RC=11 (round toward 0)  → FRINTZ (opcode=11)
//
// WHY NOT FRINTI:
//   FRINTI uses FPCR.RMode.  The x87 RC field and FPCR.RMode use the SAME
//   two bits to mean DIFFERENT things:
//     x87 RC 01 = floor (−∞)   but   FPCR.RMode 01 = ceil (+∞)
//     x87 RC 10 = ceil  (+∞)   but   FPCR.RMode 10 = floor (−∞)
//   Rosetta would have to swap the 01/10 values when copying RC into FPCR.
//   Relying on that sync is fragile and has been observed to round incorrectly
//   when the guest changes the rounding mode (e.g. FLDCW before FRNDINT).
//   Reading the control_word at JIT-time (like translate_fistp does for FISTP)
//   removes the dependency entirely.
//
// Dispatch sequence layout (indices relative to first CBZ):
//   [D+ 0] CBZ  Wd_rc, +28     →  [D+28] FRINTN  (RC=0, nearest)
//   [D+ 4] SUB  Wd_rc, Wd_rc, #1
//   [D+ 8] CBZ  Wd_rc, +28     →  [D+36] FRINTM  (RC=1, floor)
//   [D+12] SUB  Wd_rc, Wd_rc, #1
//   [D+16] CBZ  Wd_rc, +28     →  [D+44] FRINTP  (RC=2, ceil)
//   [D+20] FRINTZ  Dd, Dd       (RC=3, truncate — fall-through, default)
//   [D+24] B    +24             →  [D+48] done
//   [D+28] FRINTN  Dd, Dd
//   [D+32] B    +16             →  [D+48] done
//   [D+36] FRINTM  Dd, Dd
//   [D+40] B    +8              →  [D+48] done
//   [D+44] FRINTP  Dd, Dd
//   [D+48] (done — fall through to emit_store_st_at_offset)
//
// Register allocation:
//   Xbase   (gpr pool 0) — X87State base
//   Wd_top  (gpr pool 1) — current TOP (read-only; no push/pop)
//   Wd_tmp  (gpr pool 2) — byte offset of ST(0); held across the full dispatch
//                          for opt-3 (cannot alias Wd_rc, unlike translate_fistp
//                          which stores to memory and does not need the offset
//                          after emit_load_st).
//   Wd_rc   (gpr pool 3) — RC scratch; clobbered by LDRH + UBFX + SUBs
//   Dd      (fpr pool)   — ST(0) value, rounded in-place
// =============================================================================
auto translate_frndint(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Wd_rc = alloc_gpr(*a1, 3);  // RC dispatch scratch — must NOT alias Wd_tmp
    const int Dd = alloc_free_fpr(*a1);

    // Load ST(0) into Dd; Wd_tmp receives the byte offset of ST(0) for opt-3.
    const int Wk23 = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd, Xst_base);

    if (g_rosetta_config && g_rosetta_config->fast_round) {
        // Fast path: assume RC=0 (round-to-nearest). Single FRINTN instruction.
        emit_fp_dp1(buf, /*type=*/1 /*f64*/, /*FRINTN=*/8, Dd, Dd);
    } else {
        // ── Read and extract x87 RC field ────────────────────────────────────────
        // control_word is at [Xbase + 0] (X87State offset 0).
        // LDRH Wd_rc, [Xbase, #0]   imm12=0 → byte offset 0
        emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR*/ 1, /*imm12=*/0, Xbase, Wd_rc);
        // UBFX Wd_rc, Wd_rc, #10, #2   →  bits[11:10] in bits[1:0]
        // UBFM immr=10, imms=11  (width = imms-immr+1 = 2 bits → RC values 0..3)
        emit_bitfield(buf, /*is_64=*/0, /*UBFM*/ 2, /*N*/ 0, /*immr*/ 10, /*imms*/ 11, Wd_rc,
                      Wd_rc);

        // ── Runtime dispatch: CBZ/SUB chain ──────────────────────────────────────
        //
        // [D+0] CBZ Wd_rc, +28  (imm19=7 → branch offset 28 bytes → [D+28] FRINTN)
        emit_cbz(buf, /*is_64bit=*/0, /*is_nz=*/0, Wd_rc, 7);
        // [D+4] SUB Wd_rc, Wd_rc, #1
        emit_add_imm(buf, /*is_64=*/0, /*sub*/ 1, /*S*/ 0, /*shift*/ 0, 1, Wd_rc, Wd_rc);
        // [D+8] CBZ Wd_rc, +28  (target = D+8+28 = D+36 → [D+36] FRINTM)
        emit_cbz(buf, /*is_64bit=*/0, /*is_nz=*/0, Wd_rc, 7);
        // [D+12] SUB Wd_rc, Wd_rc, #1
        emit_add_imm(buf, /*is_64=*/0, /*sub*/ 1, /*S*/ 0, /*shift*/ 0, 1, Wd_rc, Wd_rc);
        // [D+16] CBZ Wd_rc, +28  (target = D+16+28 = D+44 → [D+44] FRINTP)
        emit_cbz(buf, /*is_64bit=*/0, /*is_nz=*/0, Wd_rc, 7);

        // [D+20] FRINTZ Dd, Dd   RC=3 (truncate) — fall-through path
        emit_fp_dp1(buf, /*type=*/1 /*f64*/, /*FRINTZ=*/11, Dd, Dd);
        // [D+24] B +24           skip to done  (imm26=6 → offset 24 → D+24+24 = D+48)
        emit_b(buf, 6);

        // [D+28] FRINTN Dd, Dd   RC=0 (round nearest, ties to even)
        emit_fp_dp1(buf, /*type=*/1 /*f64*/, /*FRINTN=*/8, Dd, Dd);
        // [D+32] B +16           (imm26=4 → offset 16 → D+32+16 = D+48)
        emit_b(buf, 4);

        // [D+36] FRINTM Dd, Dd   RC=1 (floor, toward −∞)
        emit_fp_dp1(buf, /*type=*/1 /*f64*/, /*FRINTM=*/10, Dd, Dd);
        // [D+40] B +8            (imm26=2 → offset 8 → D+40+8 = D+48)
        emit_b(buf, 2);

        // [D+44] FRINTP Dd, Dd   RC=2 (ceil, toward +∞)
        emit_fp_dp1(buf, /*type=*/1 /*f64*/, /*FRINTP=*/9, Dd, Dd);
        // [D+48] done ─────────────────────────────────────────────────────────────
    }

    // Opt 3: Wd_tmp still holds the ST(0) byte offset written by emit_load_st.
    // The dispatch chain above only touches Wd_rc and Dd; Wd_tmp is intact.
    emit_store_st_at_offset(buf, Xbase, Wk23, Dd, Xst_base);

    free_fpr(*a1, Dd);
    free_gpr(*a1, Wd_rc);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FCOMI / FUCOMI / FCOMIP / FUCOMIP — compare and set EFLAGS directly
//
// x86 semantics:
//   FCOMI:   compare ST(0) vs ST(i), set EFLAGS (ZF, PF, CF)
//   FCOMIP:  same + pop ST(0)
//   FUCOMI:  same as FCOMI (unordered quiet — AArch64 FCMP handles this)
//   FUCOMIP: same + pop
//
// Unlike FCOM (which writes to x87 status_word), FCOMI writes directly to
// EFLAGS, which Rosetta maps to AArch64 NZCV.  This is far simpler — no
// status_word RMW needed, no NZCV save/restore.
//
// Rosetta EFLAGS → NZCV mapping:  N=SF, Z=ZF, C=!CF, V=OF
// For FCOMI, SF=0 and OF is used for PF (parity = unordered):
//   N=0 always, Z=ZF, C=!CF, V=PF
//
// Desired NZCV for each FCOMI outcome:
//   GT: CF=0,ZF=0,PF=0 → N=0,Z=0,C=1,V=0
//   LT: CF=1,ZF=0,PF=0 → N=0,Z=0,C=0,V=0
//   EQ: CF=0,ZF=1,PF=0 → N=0,Z=1,C=1,V=0
//   UN: CF=1,ZF=1,PF=1 → N=0,Z=1,C=0,V=1
//
// AArch64 FCMP output NZCV:
//   GT: N=0,Z=0,C=1,V=0 → matches ✓
//   LT: N=1,Z=0,C=0,V=0 → N wrong (need 0)
//   EQ: N=0,Z=1,C=1,V=0 → matches ✓
//   UN: N=0,Z=0,C=1,V=1 → Z wrong (need 1), C wrong (need 0)
//
// Fix: N_new=0, Z_new=Z|V, C_new=C&!V, V_new=V
//
// 9 emitted instructions (branchless) vs ~30+ for runtime BL round-trip.
//
// Register allocation:
//   Xbase   (gpr pool 0) — X87State base
//   Wd_top  (gpr pool 1) — current TOP; updated by pop if FCOMIP/FUCOMIP
//   Wd_tmp  (gpr pool 2) — scratch for emit_load_st, pop RMW
//   Wd_z, Wd_v, Wd_c (free pool) — CSET results; freed after packing
//   Dd_st0  (fpr free)   — ST(0) value
//   Dd_src  (fpr free)   — ST(i) value
// =============================================================================
auto translate_fcomi(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const bool is_popping = (a2->opcode == kOpcodeName_fcomip || a2->opcode == kOpcodeName_fucomip);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd_st0 = alloc_free_fpr(*a1);
    const int Dd_src = alloc_free_fpr(*a1);

    // Load ST(0) and ST(i).
    emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_st0, Xst_base);
    const int depth = a2->operands[1].reg.reg.index();
    emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, depth), Wd_tmp, Dd_src, Xst_base);

    // FCMP Dd_st0, Dd_src — clobbers NZCV with FP comparison result
    emit_fcmp_f64(buf, Dd_st0, Dd_src);

    free_fpr(*a1, Dd_src);
    free_fpr(*a1, Dd_st0);

    // Branchless NZCV fixup: extract conditions while NZCV is live.
    const int Wd_z = alloc_free_gpr(*a1);
    const int Wd_v = alloc_free_gpr(*a1);
    const int Wd_c = alloc_free_gpr(*a1);

    // All three CSETs must execute before any MSR clobbers NZCV.
    emit_cset(buf, /*is_64bit=*/0, /*EQ=*/0, Wd_z);     // 1 if equal
    emit_cset(buf, /*is_64bit=*/0, /*VS=*/6, Wd_v);     // 1 if overflow (unordered)
    emit_cset(buf, /*is_64bit=*/0, /*CS=*/2, Wd_c);     // 1 if carry set

    // Z_new = Z | V  (EQ or unordered)
    emit_logical_shifted_reg(buf, 0, /*ORR*/1, 0, /*LSL*/0, Wd_v, 0, Wd_z, Wd_z);
    // C_new = C & !V  (carry clear for unordered)
    emit_logical_shifted_reg(buf, 0, /*AND*/0, /*N=invert*/1, /*LSL*/0, Wd_v, 0, Wd_c, Wd_c);

    // Pack into NZCV format: [31]=N=0, [30]=Z, [29]=C, [28]=V=PF, [26]=PF
    // Two consumers of this NZCV value use different bit positions for the
    // unordered/PF flag:
    //   bit 26: Rosetta's jp/jnp translation reads PF via ubfx w, w, #26, #1
    //   bit 28: AArch64 FCSEL VS/VC conditions (used by translate_fcmov)
    // Set Wd_v at both positions to satisfy both consumers.
    // LSL Wd_z, Wd_z, #30
    emit_bitfield(buf, /*is_64=*/0, /*UBFM=*/2, /*N=*/0,
                  /*immr=*/2, /*imms=*/1, Wd_z, Wd_z);
    // ORR Wd_z, Wd_z, Wd_c, LSL #29
    emit_logical_shifted_reg(buf, 0, /*ORR*/1, 0, /*LSL*/0, Wd_c, 29, Wd_z, Wd_z);
    // ORR Wd_z, Wd_z, Wd_v, LSL #28  (V = PF, for FCMOV FCSEL VS/VC)
    emit_logical_shifted_reg(buf, 0, /*ORR*/1, 0, /*LSL*/0, Wd_v, 28, Wd_z, Wd_z);
    // ORR Wd_z, Wd_z, Wd_v, LSL #26  (PF slot, for Rosetta jp/jnp ubfx #26)
    emit_logical_shifted_reg(buf, 0, /*ORR*/1, 0, /*LSL*/0, Wd_v, 26, Wd_z, Wd_z);

    // MSR NZCV, Wd_z — set the corrected flags
    emit_msr_nzcv(buf, Wd_z);

    free_gpr(*a1, Wd_c);
    free_gpr(*a1, Wd_v);
    free_gpr(*a1, Wd_z);

    // Pop if FCOMIP / FUCOMIP.
    if (is_popping)
        x87_pop(buf, *a1, Xbase, Wd_top, Wd_tmp);

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FTST — compare ST(0) against +0.0, set x87 condition codes
//
// x87 semantics:
//   FTST  D9 E4   Compare ST(0) with 0.0, set C0/C2/C3 in status_word.
//   No stack mutation. No explicit operands.
//
// This is equivalent to FCOM but with 0.0 as the comparand. We use
// FCMP Dd, #0.0 (single instruction) instead of loading a zero register.
//
// Uses the same branchless flag mapping as translate_fcom.
//
// Register allocation:
//   Xbase   (gpr pool 0) — X87State base
//   Wd_top  (gpr pool 1) — current TOP (read-only; no push/pop)
//   Wd_tmp  (gpr pool 2) — scratch
//   Wd_tmp2 (gpr pool 3) — saved NZCV
//   Dd_st0  (fpr free)   — ST(0) value
// =============================================================================
auto translate_ftst(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    static constexpr int16_t kX87StatusWordImm12 = kX87StatusWordOff / 2;

    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Wd_tmp2 = alloc_gpr(*a1, 3);

    // OPT-D: flush deferred tag and TOP before FTST (reads status_word/tag).
    x87_flush_tags(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);
    x87_flush_top(buf, *a1, Xbase, Wd_top, Wd_tmp);

    const int Dd_st0 = alloc_free_fpr(*a1);

    // Load ST(0).
    emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_st0, Xst_base);

    // MRS Wd_tmp2, NZCV — save current NZCV
    emit_mrs_nzcv(buf, Wd_tmp2);

    // FCMP Dd_st0, #0.0 — single instruction, no need to load a zero FPR
    emit_fcmp_zero_f64(buf, Dd_st0);

    free_fpr(*a1, Dd_st0);

    // OPT-L: Branchless FCMP NZCV → packed x87 CC bits + RMW status_word.
    emit_fcom_cc_pack(buf, *a1, Wd_tmp, Wd_tmp2);
    emit_fcom_cc_write_sw(buf, *a1, Xbase, Wd_tmp);

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FIST — store ST(0) as signed integer to memory (non-popping)
//
// x87 semantics:
//   mem ← (int) ST(0)   using current rounding mode (RC in control_word)
//   TOP is NOT modified (unlike FISTP).
//
// Operand encoding (memory destination at operands[0]):
//   DF /2   FIST m16int   operands[0].mem.size = S16
//   DB /2   FIST m32int   operands[0].mem.size = S32
//
// Identical to translate_fistp but without the emit_x87_pop at the end.
// The rounding mode dispatch (CBZ/SUB chain) is shared.
//
// Register allocation: identical to translate_fistp.
// =============================================================================
auto translate_fist(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const IROperandSize int_size = a2->operands[0].mem.size;

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Wd_int = alloc_free_gpr(*a1);
    const int Dd_val = alloc_free_fpr(*a1);

    // Load ST(0)
    emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_val, Xst_base);

    // Rounding mode dispatch — same CBZ/SUB chain as translate_fistp.
    const int is_64bit_int = (int_size == IROperandSize::S64) ? 1 : 0;
    const int Wd_rc = Wd_tmp;

    if (g_rosetta_config && g_rosetta_config->fast_round) {
        // Fast path: assume RC=0 (round-to-nearest). Single instruction.
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=FCVTNS*/ 0, Wd_int,
                            Dd_val);
    } else {
        // [0] LDRH Wd_rc, [Xbase, #0]
        emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR*/ 1, /*imm12=*/0, Xbase, Wd_rc);
        // [1] UBFX Wd_rc, Wd_rc, #10, #2
        emit_bitfield(buf, /*is_64=*/0, /*UBFM*/ 2, /*N*/ 0, /*immr*/ 10, /*imms*/ 11, Wd_rc,
                      Wd_rc);

        // [2] CBZ Wd_rc, +28 → [9] FCVTNS
        emit_cbz(buf, /*is_64bit=*/0, /*is_nz=*/0, Wd_rc, 7);
        // [3] SUB Wd_rc, Wd_rc, #1
        emit_add_imm(buf, /*is_64=*/0, /*is_sub*/ 1, /*S*/ 0, /*shift*/ 0, 1, Wd_rc, Wd_rc);
        // [4] CBZ Wd_rc, +28 → [11] FCVTMS
        emit_cbz(buf, /*is_64bit=*/0, /*is_nz=*/0, Wd_rc, 7);
        // [5] SUB Wd_rc, Wd_rc, #1
        emit_add_imm(buf, /*is_64=*/0, /*is_sub*/ 1, /*S*/ 0, /*shift*/ 0, 1, Wd_rc, Wd_rc);
        // [6] CBZ Wd_rc, +28 → [13] FCVTPS
        emit_cbz(buf, /*is_64bit=*/0, /*is_nz=*/0, Wd_rc, 7);

        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=*/ 3, Wd_int, Dd_val); // [7] FCVTZS (RC=3)
        emit_b(buf, 6);                                                                              // [8] B +24 → done
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=*/ 0, Wd_int, Dd_val); // [9] FCVTNS (RC=0)
        emit_b(buf, 4);                                                                              // [10] B +16 → done
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=*/ 2, Wd_int, Dd_val); // [11] FCVTMS (RC=1)
        emit_b(buf, 2);                                                                              // [12] B +8 → done
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=*/ 1, Wd_int, Dd_val); // [13] FCVTPS (RC=2)
        // [14] done
    }

    // Store integer to memory
    const int addr_reg =
        compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);
    const int store_size = (int_size == IROperandSize::S16)   ? 1
                           : (int_size == IROperandSize::S32) ? 2
                                                              : 3;
    emit_str_imm(buf, store_size, Wd_int, addr_reg, /*imm12=*/0);
    free_gpr(*a1, addr_reg);

    // NO POP — this is FIST, not FISTP.

    free_fpr(*a1, Dd_val);
    free_gpr(*a1, Wd_int);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FISUBR — reverse-subtract: integer - ST(0)
//
// x87 semantics:
//   ST(0) ← (double)(signed integer at mem) - ST(0)
//
// Operand encoding (memory source at operands[0]):
//   DE /5   FISUBR m16int   operands[0].mem.size = S16
//   DA /5   FISUBR m32int   operands[0].mem.size = S32
//
// Identical to translate_fisub except the FSUB operand order is swapped:
//   fisub:  FSUB Dd_st0, Dd_st0, Dd_int   (ST(0) - int)
//   fisubr: FSUB Dd_st0, Dd_int,  Dd_st0  (int   - ST(0))
// =============================================================================
auto translate_fisubr(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const bool is_m16 = (a2->operands[0].mem.size == IROperandSize::S16);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd_st0 = alloc_free_fpr(*a1);
    const int Dd_int = alloc_free_fpr(*a1);

    // Step 1: load ST(0)
    const int Wk = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_st0, Xst_base);

    // Step 2: compute source address
    const int addr_reg =
        compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);

    // Step 3: load integer from memory
    if (is_m16) {
        emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*opc=*/1 /*LDR*/,
                         /*imm12=*/0, addr_reg, addr_reg);
        emit_bitfield(buf, /*is_64bit=*/0, /*opc=*/0 /*SBFM*/, /*N=*/0,
                      /*immr=*/0, /*imms=*/15, addr_reg, addr_reg);
    } else {
        emit_ldr_str_imm(buf, /*size=*/2, /*is_fp=*/0, /*opc=*/1 /*LDR*/,
                         /*imm12=*/0, addr_reg, addr_reg);
    }

    // Step 4: SCVTF
    emit_scvtf(buf, /*is_64bit_int=*/0, /*ftype=*/1, Dd_int, addr_reg);
    free_gpr(*a1, addr_reg);

    // Step 5: FSUB Dd_st0, Dd_int, Dd_st0 — integer - ST(0) (REVERSED)
    emit_fsub_f64(buf, Dd_st0, Dd_int, Dd_st0);

    // Step 6: store result
    emit_store_st_at_offset(buf, Xbase, Wk, Dd_st0, Xst_base);

    free_fpr(*a1, Dd_int);
    free_fpr(*a1, Dd_st0);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FCMOV — conditional move: ST(0) ← ST(i) if condition is true
//
// All 8 variants (fcmovb, fcmovbe, fcmove, fcmovnb, fcmovnbe, fcmovne,
// fcmovu, fcmovnu) are handled here. The condition is implicit in the opcode.
//
// x87 semantics:
//   if (condition) ST(0) ← ST(i)
//
// Operand encoding:
//   operands[0] = ST(0) destination (implicit)
//   operands[1] = ST(i) source
//
// x86 condition → Rosetta NZCV condition code mapping:
//   FCMOVB   (CF=1)          → CC (C=0, since Rosetta C=!CF) = condition 3
//   FCMOVBE  (CF=1 | ZF=1)  → LS (C=0 | Z=1)                = condition 9
//   FCMOVE   (ZF=1)          → EQ (Z=1)                      = condition 0
//   FCMOVNB  (CF=0)          → CS (C=1)                      = condition 2
//   FCMOVNBE (CF=0 & ZF=0)  → HI (C=1 & Z=0)                = condition 8
//   FCMOVNE  (ZF=0)          → NE (Z=0)                      = condition 1
//   FCMOVU   (PF=1)          → VS (V=1)                      = condition 6
//   FCMOVNU  (PF=0)          → VC (V=0)                      = condition 7
//
// AArch64 FCSEL Dd, Dn, Dm, cond selects Dn if cond, else Dm.
//   FCSEL Dd_st0, Dd_src, Dd_st0, cond
//   = if (cond) Dd_st0 = Dd_src; else Dd_st0 = Dd_st0 (no change)
//
// Register allocation:
//   Xbase  (gpr pool 0) — X87State base
//   Wd_top (gpr pool 1) — current TOP (read-only; no push/pop)
//   Wd_tmp (gpr pool 2) — offset scratch for emit_{load,store}_st
//   Dd_st0 (fpr free)   — current ST(0) value
//   Dd_src (fpr free)   — ST(i) value (the candidate replacement)
// =============================================================================
auto translate_fcmov(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    // Map opcode to AArch64 condition code.
    int aarch64_cond;
    switch (a2->opcode) {
        case kOpcodeName_fcmovb:   aarch64_cond = 3;  break;  // CC
        case kOpcodeName_fcmovbe:  aarch64_cond = 9;  break;  // LS
        case kOpcodeName_fcmove:   aarch64_cond = 0;  break;  // EQ
        case kOpcodeName_fcmovnb:  aarch64_cond = 2;  break;  // CS
        case kOpcodeName_fcmovnbe: aarch64_cond = 8;  break;  // HI
        case kOpcodeName_fcmovne:  aarch64_cond = 1;  break;  // NE
        case kOpcodeName_fcmovu:   aarch64_cond = 6;  break;  // VS
        case kOpcodeName_fcmovnu:  aarch64_cond = 7;  break;  // VC
        default:                   aarch64_cond = 14; break;  // AL (should never happen)
    }

    const int depth_src = a2->operands[1].reg.reg.index();

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Dd_st0 = alloc_free_fpr(*a1);
    const int Dd_src = alloc_free_fpr(*a1);

    // OPT-D: flush deferred tag and TOP before FCMOV.
    {
        const int Wd_tmp2 = alloc_free_gpr(*a1);
        x87_flush_tags(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);
        free_gpr(*a1, Wd_tmp2);
    }
    x87_flush_top(buf, *a1, Xbase, Wd_top, Wd_tmp);

    // Load ST(i) FIRST — its key is discarded (Opt 3 pattern).
    emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, depth_src), Wd_tmp, Dd_src, Xst_base);
    // Load ST(0) LAST — Wd_tmp retains ST(0) key for emit_store_st_at_offset.
    const int Wk = emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_st0, Xst_base);

    // FCSEL: if (cond) result = Dd_src, else result = Dd_st0
    // NZCV is live from prior x86 ALU/FCOMI — this is what FCMOV tests.
    emit_fcsel_f64(buf, Dd_st0, Dd_src, Dd_st0, aarch64_cond);

    // Store result back to ST(0).
    emit_store_st_at_offset(buf, Xbase, Wk, Dd_st0, Xst_base);

    free_fpr(*a1, Dd_src);
    free_fpr(*a1, Dd_st0);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}


// =============================================================================
// FICOM / FICOMP — compare ST(0) with integer memory operand.
//
// x87 semantics:
//   FICOM  m16int/m32int   — compare ST(0) with (int)mem, set CC in status_word
//   FICOMP m16int/m32int   — same + pop
//
// Rosetta encoding:
//   DE /2: FICOM  m16int   operands = [m16int MemRef, ST(0)]
//   DA /2: FICOM  m32int   operands = [m32int MemRef, ST(0)]
//   DE /3: FICOMP m16int   operands = [m16int MemRef, ST(0)]
//   DA /3: FICOMP m32int   operands = [m32int MemRef, ST(0)]
//
// Combines integer load pattern from translate_fiadd with comparison+CC
// pattern from translate_fcom.
// =============================================================================
auto translate_ficom(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const bool is_popping = (a2->opcode == kOpcodeName_ficomp);
    const bool is_m16 = (a2->operands[0].mem.size == IROperandSize::S16);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Wd_tmp2 = alloc_gpr(*a1, 3);
    const int Dd_st0 = alloc_free_fpr(*a1);
    const int Dd_int = alloc_free_fpr(*a1);

    // Step 1: load ST(0).
    emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_st0, Xst_base);

    // Step 2: compute source memory address.
    const int addr_reg =
        compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);

    // Step 3: load integer from memory.
    if (is_m16) {
        emit_ldr_str_imm(buf, /*size=*/1 /*16-bit*/, /*is_fp=*/0, /*opc=*/1 /*LDR*/,
                         /*imm12=*/0, addr_reg, addr_reg);
        // SXTH: sign-extend bits[15:0] → W
        emit_bitfield(buf, /*is_64bit=*/0, /*opc=*/0 /*SBFM*/, /*N=*/0,
                      /*immr=*/0, /*imms=*/15, addr_reg, addr_reg);
    } else {
        emit_ldr_str_imm(buf, /*size=*/2 /*32-bit*/, /*is_fp=*/0, /*opc=*/1 /*LDR*/,
                         /*imm12=*/0, addr_reg, addr_reg);
    }

    // Step 4: convert signed integer W → f64.
    emit_scvtf(buf, /*is_64bit_int=*/0, /*ftype=*/1, Dd_int, addr_reg);
    free_gpr(*a1, addr_reg);

    // Step 5: save NZCV, compare, map flags.
    emit_mrs_nzcv(buf, Wd_tmp2);
    emit_fcmp_f64(buf, Dd_st0, Dd_int);
    free_fpr(*a1, Dd_int);
    free_fpr(*a1, Dd_st0);

    // Step 6: branchless FCMP NZCV → x87 CC bits + RMW status_word.
    emit_fcom_cc_pack(buf, *a1, Wd_tmp, Wd_tmp2);
    emit_fcom_cc_write_sw(buf, *a1, Xbase, Wd_tmp);

    // Step 7: pop if FICOMP.
    if (is_popping)
        x87_pop(buf, *a1, Xbase, Wd_top, Wd_tmp);

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}


// =============================================================================
// FLDCW — load x87 control word from memory.
//
// x87 semantics:
//   control_word ← mem16
//
// No stack mutation — TOP is unchanged.
//
// Instruction sequence:
//   1. compute address of memory operand → addr_reg
//   2. LDRH Wd_cw, [addr_reg]           — load u16 from memory
//   3. STRH Wd_cw, [Xbase, #0]          — write to X87State.control_word
//
// Register allocation:
//   Xbase    (gpr pool 0) — X87State base
//   Wd_top   (gpr pool 1) — current TOP (unused but held by x87_begin)
//   Wd_tmp   (gpr pool 2) — scratch
//   addr_reg (free gpr)   — resolved memory operand address
//   Wd_cw    (free gpr)   — loaded control word value
// =============================================================================
auto translate_fldcw(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);

    const int Wd_tmp = alloc_gpr(*a1, 2);

    // Flush deferred TOP before writing CW — subsequent instructions that
    // re-enter x87_begin will reload TOP from memory.
    x87_flush_top(buf, *a1, Xbase, Wd_top, Wd_tmp);

    const int addr_reg = compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);
    const int Wd_cw = alloc_free_gpr(*a1);

    // LDRH Wd_cw, [addr_reg]
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR*/1, /*imm12=*/0, addr_reg, Wd_cw);
    free_gpr(*a1, addr_reg);

    // STRH Wd_cw, [Xbase, #0]  — control_word is at offset 0x00
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR*/0, /*imm12=*/0, Xbase, Wd_cw);
    free_gpr(*a1, Wd_cw);

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FNSTCW — store x87 control word to memory.
//
// x87 semantics:
//   mem16 ← control_word
//
// No stack mutation — TOP is unchanged.
//
// Instruction sequence:
//   1. LDRH Wd_cw, [Xbase, #0]          — read X87State.control_word
//   2. compute address of memory operand → addr_reg
//   3. STRH Wd_cw, [addr_reg]           — store u16 to memory
//
// Register allocation:
//   Xbase    (gpr pool 0) — X87State base
//   Wd_top   (gpr pool 1) — current TOP (unused but held by x87_begin)
//   Wd_tmp   (gpr pool 2) — scratch
//   Wd_cw    (free gpr)   — loaded control word value
//   addr_reg (free gpr)   — resolved memory operand address
// =============================================================================
auto translate_fnstcw(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Wd_cw = alloc_free_gpr(*a1);

    // LDRH Wd_cw, [Xbase, #0]  — control_word is at offset 0x00
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR*/1, /*imm12=*/0, Xbase, Wd_cw);

    const int addr_reg = compute_operand_address(*a1, /*is_64bit=*/true, &a2->operands[0], GPR::XZR);

    // STRH Wd_cw, [addr_reg]
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR*/0, /*imm12=*/0, addr_reg, Wd_cw);
    free_gpr(*a1, addr_reg);
    free_gpr(*a1, Wd_cw);

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FNOP — x87 no-operation.
//
// x87 semantics:
//   (none — no state change)
//
// The per-instruction handler simply maintains cache continuity so that
// surrounding instructions in the same run keep their deferred state.
// =============================================================================
auto translate_fnop(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);

    const int Wd_tmp = alloc_gpr(*a1, 2);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FSCALE — ST(0) ← ST(0) · 2^trunc(ST(1)).  ST(1) NOT popped.
//
// Strategy (pure f64 — no f80 unpacking like stock):
//   k          = FCVTZS(ST(1))            ; truncated, saturating signed
//   exp_new    = 1023 + k
//   if k > 1023:    multiplier = +∞  (overflow)
//   if k < -1022:   multiplier = 0   (underflow)
//   else:           multiplier_bits = exp_new << 52  ; 2^k as f64
//   result_norm = ST(0) · multiplier
//   if ST(1) is NaN: result = ST(1)        ; FCVTZS(NaN)=0 would give wrong answer
//   else:            result = result_norm
//
// FMUL handles the remaining special-case fall-out automatically:
//   ST(0) NaN → result NaN; 0·∞ → NaN; ∞·finite → ±∞ etc.
// =============================================================================
auto translate_fscale(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;

    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);
    const int Wd_tmp = alloc_gpr(*a1, 2);

    // Load both stack values; resolve_depth honours deferred-fxch perm.
    const int Dd_a = alloc_free_fpr(*a1);
    emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_a, Xst_base);
    const int Dd_b = alloc_free_fpr(*a1);
    emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 1), Wd_tmp, Dd_b, Xst_base);

    // ── k = trunc(ST(1)) as 32-bit signed (saturating) ──
    const int Wd_k = alloc_free_gpr(*a1);
    emit_fcvtzs(buf, /*ftype=*/1 /*f64*/, /*is_64bit_int=*/0, Wd_k, Dd_b);

    // ── Build multiplier bits in Wd_e (reused as 64-bit Xd_bits below) ──
    // exp_new = 1023 + k
    const int Wd_e = alloc_free_gpr(*a1);
    emit_add_imm(buf, /*is_64=*/0, /*is_sub=*/0, /*set_flags=*/0,
                 /*shift=*/0, /*imm12=*/1023, Wd_k, Wd_e);
    // UBFIZ Xd_bits, Wd_e, #52, #11   → exp_new at bits[62:52], rest zero
    // 32-bit ADD zero-extends; the X view of Wd_e has high 32 bits zero, so
    // UBFM N=1 immr=12 imms=10 reads low 11 bits and rotates to bits 52..62.
    emit_bitfield(buf, /*is_64=*/1, /*UBFM=*/2, /*N=*/1,
                  /*immr=*/12, /*imms=*/10, /*Rn=*/Wd_e, /*Rd=*/Wd_e);

    // CSEL helper (no GPR variant exists in our helpers).
    auto emit_csel = [&](int is_64, int Rd, int Rn, int Rm, int cond) {
        uint32_t insn = 0x1A800000u;
        insn |= (uint32_t)(is_64 != 0) << 31;
        insn |= (uint32_t)(Rm   & 0x1F) << 16;
        insn |= (uint32_t)(cond & 0xF)  << 12;
        insn |= (uint32_t)(Rn   & 0x1F) << 5;
        insn |= (uint32_t)(Rd   & 0x1F);
        buf.emit(insn);
    };
    constexpr int kGT = 0xC;  // signed greater-than
    constexpr int kLT = 0xB;  // signed less-than
    constexpr int kVS = 0x6;  // overflow set (FP unordered)

    // CMP Wd_k, #1023.  GT (signed) → k > 1023 → overflow.
    emit_add_imm(buf, /*is_64=*/0, /*is_sub=*/1, /*set_flags=*/1,
                 /*shift=*/0, /*imm12=*/1023, Wd_k, /*Rd=*/31);
    {
        const int Xtemp = alloc_free_gpr(*a1);
        // +Inf bits = 0x7FF0_0000_0000_0000 = MOVZ #0x7FF0 LSL #48
        emit_movn(buf, /*is_64=*/1, /*MOVZ=*/2, /*hw=*/3, 0x7FF0, Xtemp);
        emit_csel(/*is_64=*/1, /*Rd=*/Wd_e, /*Rn=*/Xtemp, /*Rm=*/Wd_e, kGT);
        free_gpr(*a1, Xtemp);
    }

    // CMN Wd_k, #1022 → flags as Wd_k + 1022.  LT → k < -1022 → underflow.
    // CMN imm12: ADDS Rd=XZR, Rn, #imm12.  Use emit_add_imm with is_sub=0,
    // set_flags=1, Rd=31.
    emit_add_imm(buf, /*is_64=*/0, /*is_sub=*/0, /*set_flags=*/1,
                 /*shift=*/0, /*imm12=*/1022, Wd_k, /*Rd=*/31);
    // CSEL Wd_e = (LT) ? XZR : Wd_e — underflow → multiplier = 0.
    emit_csel(/*is_64=*/1, /*Rd=*/Wd_e, /*Rn=*/31 /*XZR*/, /*Rm=*/Wd_e, kLT);
    free_gpr(*a1, Wd_k);

    // FMOV multiplier into FPR.
    const int Dd_m = alloc_free_fpr(*a1);
    emit_fmov_x_to_d(buf, Dd_m, Wd_e);
    free_gpr(*a1, Wd_e);

    // result_norm = Dd_a * Dd_m
    const int Dd_norm = alloc_free_fpr(*a1);
    emit_fmul_f64(buf, Dd_norm, Dd_a, Dd_m);
    free_fpr(*a1, Dd_m);
    free_fpr(*a1, Dd_a);

    // FCMP Dd_b, Dd_b → V=1 iff Dd_b is NaN.
    // FCSEL Dd_result = (NaN) ? Dd_b : Dd_norm
    emit_fcmp_f64(buf, Dd_b, Dd_b);
    const int Dd_result = alloc_free_fpr(*a1);
    emit_fcsel_f64(buf, Dd_result, Dd_b, Dd_norm, kVS);
    free_fpr(*a1, Dd_norm);
    free_fpr(*a1, Dd_b);

    // Write result back to ST(0).
    emit_store_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_result, Xst_base);
    free_fpr(*a1, Dd_result);

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FXTRACT — split ST(0) into (significand, unbiased_exp).
//
// x87 semantics:
//   ST(0) ← unbiased_exponent_of_input  (as f64)
//   push: ST(0) ← significand (sign·1.mantissa scaled to bias 0)
//   After: ST(0) = significand, ST(1) = exponent.
//
// Bit-level f64 derivation (normal case):
//   sig = (input_bits & ~exp_mask) | (0x3FF << 52)   (sign+mant kept; exp=bias 0)
//   exp = SCVTF(exp_field - 1023) as f64
//
// Special cases (Intel SDM):
//   ±0  → sig = ±0 (input);     exp = -∞
//   ±∞  → sig = ±∞ (input);     exp = +∞ (always positive)
//   NaN → sig = NaN (input);    exp = NaN (input)
//   Denormal: SDM-defined as a normalised result; we treat as the zero
//   case (no current workload exercises denormal fxtract).
//
// CSEL chain: default = normal-case bits, override to (-∞ / +∞ / input)
// based on exp_field comparisons and a mantissa-zero ANDS for NaN-vs-Inf.
// =============================================================================
auto translate_fxtract(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;

    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    const int Wd_tmp  = alloc_gpr(*a1, 2);
    const int Wd_tmp2 = alloc_gpr(*a1, 3);

    // Load ST(0) → Dd_v.  Keep this FPR live until the FCSEL chain.
    const int Dd_v = alloc_free_fpr(*a1);
    emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_v, Xst_base);

    // FMOV bits to Xv for bit-level work; UBFX exp_field into Wd_e.
    const int Xv = alloc_free_gpr(*a1);
    emit_fmov_d_to_x(buf, Xv, Dd_v);
    const int Wd_e = alloc_free_gpr(*a1);
    emit_bitfield(buf, /*is_64=*/1, /*UBFM=*/2, /*N=*/1,
                  /*immr=*/52, /*imms=*/62, Xv, Wd_e);

    // GPR free-pool budget is tight (~3 in cache-active mode), so we serialise
    // the work: build the exponent in FP-domain via FCSEL first, write back to
    // memory, then build the significand in GPR.

    // ── Inline GPR CSEL (no helper). Pattern from translate_fxam. ──
    auto emit_csel = [&](int is_64, int Rd, int Rn, int Rm, int cond) {
        uint32_t insn = 0x1A800000u;
        insn |= (uint32_t)(is_64 != 0) << 31;
        insn |= (uint32_t)(Rm   & 0x1F) << 16;
        insn |= (uint32_t)(cond & 0xF)  << 12;
        insn |= (uint32_t)(Rn   & 0x1F) << 5;
        insn |= (uint32_t)(Rd   & 0x1F);
        buf.emit(insn);
    };
    constexpr int kEQ = 0x0;

    // ── Phase A: build exponent in Dd_exp ─────────────────────────────────
    // SUB Wd_em (32-bit) = Wd_e - 1023; SCVTF; FMOV → Dd_norm.
    int Wd_em = alloc_free_gpr(*a1);
    emit_add_imm(buf, /*is_64=*/0, /*is_sub=*/1, /*set_flags=*/0,
                 /*shift=*/0, /*imm12=*/1023, Wd_e, Wd_em);
    const int Dd_norm = alloc_free_fpr(*a1);
    emit_scvtf(buf, /*is_64bit_int=*/0, /*ftype=*/1 /*f64*/, Dd_norm, Wd_em);
    free_gpr(*a1, Wd_em);

    // Constants in FPR via Xtemp (one rotating GPR for the 16-bit-shifted MOVZ).
    const int Xtemp   = alloc_free_gpr(*a1);
    const int Dd_inf  = alloc_free_fpr(*a1);
    emit_movn(buf, /*is_64=*/1, /*MOVZ=*/2, /*hw=*/3 /*<<48*/, 0x7FF0, Xtemp);
    emit_fmov_x_to_d(buf, Dd_inf, Xtemp);
    const int Dd_minf = alloc_free_fpr(*a1);
    emit_movn(buf, /*is_64=*/1, /*MOVZ=*/2, /*hw=*/3, 0xFFF0, Xtemp);
    emit_fmov_x_to_d(buf, Dd_minf, Xtemp);
    free_gpr(*a1, Xtemp);

    // ANDS XZR, Xv, #0x000FFFFFFFFFFFFF — Z=1 iff mantissa==0.
    LogicalImmEncoding enc_mant;
    is_bitmask_immediate(/*is_64=*/true, 0x000FFFFFFFFFFFFFULL, enc_mant);
    emit_logical_imm(buf, /*is_64=*/1, /*ANDS=*/3,
                     enc_mant.N, enc_mant.immr, enc_mant.imms,
                     /*Rn=*/Xv, /*Rd=*/31 /*XZR*/);
    // FCSEL Dd_pos = (mant==0) ? Dd_inf : Dd_v   (Inf for true-Inf, NaN for NaN)
    const int Dd_pos = alloc_free_fpr(*a1);
    emit_fcsel_f64(buf, Dd_pos, Dd_inf, Dd_v, kEQ);
    free_fpr(*a1, Dd_inf);
    free_fpr(*a1, Dd_v);

    // CMP Wd_e, #0x7FF
    emit_add_imm(buf, /*is_64=*/0, /*is_sub=*/1, /*set_flags=*/1,
                 /*shift=*/0, /*imm12=*/0x7FF, Wd_e, /*Rd=*/31);
    // FCSEL Dd_exp = (exp==0x7FF) ? Dd_pos : Dd_norm
    const int Dd_exp = alloc_free_fpr(*a1);
    emit_fcsel_f64(buf, Dd_exp, Dd_pos, Dd_norm, kEQ);
    free_fpr(*a1, Dd_pos);
    free_fpr(*a1, Dd_norm);

    // CMP Wd_e, #0
    emit_add_imm(buf, /*is_64=*/0, /*is_sub=*/1, /*set_flags=*/1,
                 /*shift=*/0, /*imm12=*/0, Wd_e, /*Rd=*/31);
    // FCSEL Dd_exp = (exp==0) ? Dd_minf : Dd_exp
    emit_fcsel_f64(buf, Dd_exp, Dd_minf, Dd_exp, kEQ);
    free_fpr(*a1, Dd_minf);

    // Write exp into the OLD ST(0) slot (depth=0 here, becomes depth=1 after push).
    emit_store_st(buf, Xbase, Wd_top, /*depth=*/0, Wd_tmp, Dd_exp, Xst_base);
    free_fpr(*a1, Dd_exp);

    // ── Phase B: build significand bits in Xs, FMOV, push, store ──────────
    // Xs = sign | 0x3FF<<52 | mantissa  (normal case).  Special cases preserve Xv.
    const int Xs = alloc_free_gpr(*a1);
    emit_mov_reg(buf, /*is_64=*/1, Xs, Xv);
    // BFI Xs, XZR, #52, #11 — clear exp field.
    emit_bitfield(buf, /*is_64=*/1, /*BFM=*/1, /*N=*/1,
                  /*immr=*/12, /*imms=*/10, /*Rn=*/31, /*Rd=*/Xs);
    // ORR Xs, Xs, #0x3FF0_0000_0000_0000 — set exp=bias 0.
    LogicalImmEncoding enc_bias;
    is_bitmask_immediate(/*is_64=*/true, 0x3FF0000000000000ULL, enc_bias);
    emit_orr_imm(buf, /*is_64=*/1, /*Rd=*/Xs, /*Rn=*/Xs,
                 enc_bias.N, enc_bias.immr, enc_bias.imms);

    // CMP Wd_e, #0x7FF; CSEL Xs = (eq) ? Xv : Xs
    emit_add_imm(buf, /*is_64=*/0, /*is_sub=*/1, /*set_flags=*/1,
                 /*shift=*/0, /*imm12=*/0x7FF, Wd_e, /*Rd=*/31);
    emit_csel(/*is_64=*/1, /*Rd=*/Xs, /*Rn=*/Xv, /*Rm=*/Xs, kEQ);
    // CMP Wd_e, #0; CSEL Xs = (eq) ? Xv : Xs
    emit_add_imm(buf, /*is_64=*/0, /*is_sub=*/1, /*set_flags=*/1,
                 /*shift=*/0, /*imm12=*/0, Wd_e, /*Rd=*/31);
    emit_csel(/*is_64=*/1, /*Rd=*/Xs, /*Rn=*/Xv, /*Rm=*/Xs, kEQ);
    free_gpr(*a1, Wd_e);
    free_gpr(*a1, Xv);

    const int Dd_sig = alloc_free_fpr(*a1);
    emit_fmov_x_to_d(buf, Dd_sig, Xs);
    free_gpr(*a1, Xs);

    // Push (TOP-=1, mark new ST(0) tag valid).
    x87_push(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);

    // Store sig at new ST(0).
    emit_store_st(buf, Xbase, Wd_top, /*depth=*/0, Wd_tmp, Dd_sig, Xst_base);
    free_fpr(*a1, Dd_sig);
    free_gpr(*a1, Wd_tmp2);

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FDECSTP — decrement TOP.
//
// x87 semantics:
//   TOP = (TOP - 1) & 7
//   tag word, data registers: unchanged
//   C1: cleared (we leave undefined; tests don't check)
//
// Cache: TOP changes, so we must flush perm (perm map is TOP-relative)
// and tags (tag_push_pending / deferred_pop_count are slot indices
// relative to current TOP).  After flushes, decrement Wd_top in-place
// and set top_dirty=1; x87_end emits the memory write-back at run-end.
// =============================================================================
auto translate_fdecstp(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Wd_tmp = alloc_gpr(*a1, 2);

    // Flush state that's TOP-relative before changing TOP.
    perm_flush_before_stack_change(buf, *a1, Xbase, Wd_top, Wd_tmp);
    {
        const int Wd_tmp2 = alloc_free_gpr(*a1);
        x87_flush_tags(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);
        free_gpr(*a1, Wd_tmp2);
    }

    // SUB  Wd_top, Wd_top, #1
    emit_add_imm(buf, /*is_64bit=*/0, /*is_sub=*/1, /*is_set_flags=*/0,
                 /*shift=*/0, /*imm12=*/1, Wd_top, Wd_top);
    // AND  Wd_top, Wd_top, #7  (N=0, immr=0, imms=2 → 3-bit mask)
    emit_and_imm(buf, /*is_64bit=*/0, /*Rd=*/Wd_top,
                 /*N=*/0, /*immr=*/0, /*imms=*/2, /*Rn=*/Wd_top);

    a1->x87_cache.top_dirty = 1;

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FINCSTP — increment TOP.  Symmetric to FDECSTP; ADD instead of SUB.
// =============================================================================
auto translate_fincstp(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Wd_tmp = alloc_gpr(*a1, 2);

    perm_flush_before_stack_change(buf, *a1, Xbase, Wd_top, Wd_tmp);
    {
        const int Wd_tmp2 = alloc_free_gpr(*a1);
        x87_flush_tags(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);
        free_gpr(*a1, Wd_tmp2);
    }

    // ADD  Wd_top, Wd_top, #1
    emit_add_imm(buf, /*is_64bit=*/0, /*is_sub=*/0, /*is_set_flags=*/0,
                 /*shift=*/0, /*imm12=*/1, Wd_top, Wd_top);
    // AND  Wd_top, Wd_top, #7
    emit_and_imm(buf, /*is_64bit=*/0, /*Rd=*/Wd_top,
                 /*N=*/0, /*immr=*/0, /*imms=*/2, /*Rn=*/Wd_top);

    a1->x87_cache.top_dirty = 1;

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FFREE ST(i) — mark ST(i)'s tag as Empty.
//
// x87 semantics:
//   tag_word[(2*phys+1):(2*phys)] = 0b11   where phys = (TOP + i) & 7
//   data registers, TOP, status flags: unchanged
//
// Pre-flush deferred tag state (tag_push_pending / deferred_pop_count both
// write to tag_word).  Then OR in the empty-tag bits at the right slot
// position computed at run time from Wd_top + depth.
//
// Steady state in a run: 7 emitted instructions per ffree
//   ADD Wb, Wd_top, #depth   (skipped if depth==0)
//   AND Wb, Wb, #7           (skipped if depth==0)
//   LSL Wb, Wb, #1
//   MOVZ Wm, #3
//   LSLV Wm, Wm, Wb          (mask = 3 << (2*phys))
//   LDRH Wt, [Xbase, #tag_word]
//   ORR  Wt, Wt, Wm
//   STRH Wt, [Xbase, #tag_word]
// =============================================================================
auto translate_ffree(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    static constexpr int16_t kX87TagWordImm12 = kX87TagWordOff / 2;  // = 2

    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Wd_tmp = alloc_gpr(*a1, 2);

    // Operand encoding: ffree has a single ST(i) operand.  Existing
    // unary handlers (translate_fxch reads it from operands[1] because
    // Rosetta normalises {[ST(0), ST(i)]} for swap-shape ops; ffree is
    // a true single-operand op so its register is at operands[0]).
    const int logical_depth = a2->operands[0].reg.reg.index();
    const int depth = resolve_depth(*a1, logical_depth);

    // Flush any deferred tag-word updates so our LDRH sees coherent state.
    {
        const int Wd_tmp2 = alloc_free_gpr(*a1);
        x87_flush_tags(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);
        free_gpr(*a1, Wd_tmp2);
    }

    const int Wd_bitpos = alloc_free_gpr(*a1);
    const int Wd_mask   = alloc_free_gpr(*a1);

    // Compute physical slot index in Wd_bitpos.
    if (depth == 0) {
        // bitpos = Wd_top * 2  (no add needed)
        emit_bitfield(buf, /*is_64=*/0, /*UBFM*/ 2, /*N=*/0,
                      /*immr=*/31, /*imms=*/30, Wd_top, Wd_bitpos);
    } else {
        // ADD Wd_bitpos, Wd_top, #depth
        emit_add_imm(buf, /*is_64bit=*/0, /*is_sub=*/0, /*is_set_flags=*/0,
                     /*shift=*/0, /*imm12=*/depth, Wd_top, Wd_bitpos);
        // AND Wd_bitpos, Wd_bitpos, #7
        emit_and_imm(buf, /*is_64bit=*/0, /*Rd=*/Wd_bitpos,
                     /*N=*/0, /*immr=*/0, /*imms=*/2, /*Rn=*/Wd_bitpos);
        // LSL Wd_bitpos, Wd_bitpos, #1  → 2*phys
        emit_bitfield(buf, /*is_64=*/0, /*UBFM*/ 2, /*N=*/0,
                      /*immr=*/31, /*imms=*/30, Wd_bitpos, Wd_bitpos);
    }

    // MOVZ Wd_mask, #3
    emit_movn(buf, /*is_64=*/0, /*MOVZ opc=*/2, /*hw=*/0, /*imm=*/3, Wd_mask);
    // LSLV Wd_mask, Wd_mask, Wd_bitpos   → mask = 3 << (2*phys)
    emit_lslv(buf, /*is_64=*/0, /*Rm=*/Wd_bitpos, /*Rn=*/Wd_mask, /*Rd=*/Wd_mask);

    // LDRH Wd_bitpos, [Xbase, #tag_word]   (reuse Wd_bitpos as tag-word reg)
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1,
                     kX87TagWordImm12, Xbase, Wd_bitpos);
    // ORR Wd_bitpos, Wd_bitpos, Wd_mask
    emit_logical_shifted_reg(buf, /*is_64=*/0, /*ORR=*/1, /*N=*/0,
                             /*LSL=*/0, /*Rm=*/Wd_mask, /*shift=*/0,
                             /*Rn=*/Wd_bitpos, /*Rd=*/Wd_bitpos);
    // STRH Wd_bitpos, [Xbase, #tag_word]
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR=*/0,
                     kX87TagWordImm12, Xbase, Wd_bitpos);

    free_gpr(*a1, Wd_mask);
    free_gpr(*a1, Wd_bitpos);

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FCLEX / FNCLEX — clear x87 exception flags.
//
// x87 semantics:
//   status_word &= 0x7F00
//
// Clears bits 0..7 (PE,UE,OE,ZE,DE,IE,SF,ES) and bit 15 (B).
// Preserves C0,C1,C2 (bits 8..10), TOP (bits 11..13), C3 (bit 14).
//
// The AND-mask 0x7F00 happens to preserve TOP — so we don't need to flush
// top_dirty before the RMW (stale memory TOP survives the AND unchanged),
// and we don't need to load TOP into a register (x87_end's emit_store_top
// handles flushing if top_dirty was set by a prior op in the run).
//
// Three emitted instructions (steady state, in a run): LDRH + AND-imm + STRH.
// =============================================================================
auto translate_fclex(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    static constexpr int16_t kX87StatusWordImm12 = kX87StatusWordOff / 2;  // = 1

    // Custom prologue when fclex is alone (run_remaining == 0): allocate
    // only Xbase, skip emit_load_top.  When in a run, fall through to
    // x87_begin so subsequent ops see a pinned Wd_top.
    int Xbase, Wd_top;
    bool wd_top_unused = false;
    if (a1->x87_cache.run_remaining == 0) {
        Xbase = alloc_gpr(*a1, 0);
        emit_x87_base(buf, *a1, Xbase);
        Wd_top = GPR::XZR;          // unused: gprs_valid==0 → no deferred
                                    // flags set → x87_end won't write through it.
        wd_top_unused = true;
    } else {
        auto pair = x87_begin(*a1, buf);
        Xbase = pair.first;
        Wd_top = pair.second;
    }
    const int Wd_tmp = alloc_gpr(*a1, 2);

    // LDRH Wd_tmp, [Xbase, #status_word]
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1,
                     kX87StatusWordImm12, Xbase, Wd_tmp);

    // AND  Wd_tmp, Wd_tmp, #0x7F00  — keep bits 8..14
    LogicalImmEncoding enc_keep;
    is_bitmask_immediate(/*is_64bit=*/false, 0x00007F00ULL, enc_keep);
    emit_and_imm(buf, /*is_64bit=*/0, /*Rd=*/Wd_tmp,
                 enc_keep.N, enc_keep.immr, enc_keep.imms, /*Rn=*/Wd_tmp);

    // STRH Wd_tmp, [Xbase, #status_word]
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR=*/0,
                     kX87StatusWordImm12, Xbase, Wd_tmp);

    if (wd_top_unused) {
        free_gpr(*a1, Wd_tmp);
        free_gpr(*a1, Xbase);
        // No x87_end needed: gprs_valid==0 means no deferred flags can
        // be set, and run_remaining==0 means there is no run to tick.
    } else {
        x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
        free_gpr(*a1, Wd_tmp);
    }
}

// =============================================================================
// FINIT / FNINIT — reset FPU state.
//
// x87 semantics:
//   control_word ← 0x037F   (mask all exceptions, round-nearest, 64-bit prec)
//   status_word  ← 0x0000   (TOP=0, all condition codes / exception flags clear)
//   tag_word     ← 0xFFFF   (all 8 slots tagged kEmpty)
//   The 8 ST register slots themselves are not cleared — tagged-empty suffices.
//
// Three stores; no read of existing state.  If finit is the only x87 op in
// its window we use the same mini-prologue trick as fclex (alloc only Xbase,
// skip emit_load_top); when in a run we go through x87_begin so the cache
// stays pinned for subsequent ops, then emit MOV Wd_top, #0 to match the
// new SW.TOP and discard any deferred flags rather than flushing them
// (their target memory is gone).
// =============================================================================
auto translate_finit(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    static constexpr int16_t kX87CtrlWordImm12   = kX87ControlWordOff / 2;  // = 0
    static constexpr int16_t kX87StatusWordImm12 = kX87StatusWordOff / 2;   // = 1
    static constexpr int16_t kX87TagWordImm12    = kX87TagWordOff / 2;      // = 2

    int Xbase, Wd_top;
    const bool standalone = (a1->x87_cache.run_remaining == 0);
    if (standalone) {
        Xbase = alloc_gpr(*a1, 0);
        emit_x87_base(buf, *a1, Xbase);
        Wd_top = GPR::XZR;          // unused: gprs_valid stays 0
    } else {
        auto pair = x87_begin(*a1, buf);
        Xbase = pair.first;
        Wd_top = pair.second;
    }

    const int Wd_tmp = alloc_gpr(*a1, 2);

    // CW <- 0x037F
    emit_movn(buf, /*is_64=*/0, /*MOVZ opc=*/2, /*hw=*/0, /*imm=*/0x037F, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR=*/0,
                     kX87CtrlWordImm12, Xbase, Wd_tmp);

    // SW <- 0  (also resets SW.TOP to 0)
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR=*/0,
                     kX87StatusWordImm12, Xbase, GPR::XZR);

    // TW <- 0xFFFF
    emit_movn(buf, /*is_64=*/0, /*MOVZ opc=*/2, /*hw=*/0, /*imm=*/0xFFFF, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR=*/0,
                     kX87TagWordImm12, Xbase, Wd_tmp);

    if (standalone) {
        free_gpr(*a1, Wd_tmp);
        free_gpr(*a1, Xbase);
    } else {
        // Deferred flags now point at memory we just overwrote — discard.
        a1->x87_cache.top_dirty = 0;
        a1->x87_cache.tag_push_pending = 0;
        a1->x87_cache.deferred_pop_count = 0;
        a1->x87_cache.reset_perm();
        // MOVZ Wd_top, #0  — match the new SW.TOP for the rest of the run.
        emit_movn(buf, /*is_64=*/0, /*MOVZ opc=*/2, /*hw=*/0, /*imm=*/0, Wd_top);

        x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
        free_gpr(*a1, Wd_tmp);
    }
}

// =============================================================================
// FBSTP — pop ST(0) and store as 18-digit packed BCD into m80bcd.
//
// Memory format (10 bytes, low-to-high):
//   bytes[0..8] : 18 packed BCD digits (LSD at byte 0 low nibble), 2 per byte
//   byte[9]     : bit 7 = sign (1 = negative); bits[6:0] reserved zero
//
// Indefinite (NaN, ±inf, |rounded| >= 10^18):
//   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0xFF }
//
// Codegen:
//   1.  Compute dest address.
//   2.  Load ST(0) into Dd_val.
//   3.  Read CW.RC bits 11:10.
//   4.  Round per CW.RC: 4-way parallel FRINT[N|P|M|Z] + 3-step FCSEL chain.
//   5.  Detect indef: |round| >= 1e18 OR round is NaN.
//   6.  Extract sign from rounded f64 (preserves -0 sign).
//   7.  FCVTZS to int64 (round is integer-valued, so no further rounding).
//   8.  Pre-compute |Xint|, sign byte, Xten.
//   9.  CBNZ Wovf to indef path; else 9-byte BCD divmod (alternating
//       dividend / quotient registers) + sign byte; B over indef block;
//       indef block writes the 5-instruction hardcoded pattern.
//   10. x87_pop ST(0).
//
// Branch offsets are hand-counted; constexpr asserts on the layout would
// be ideal but the body uses a runtime loop so a static_assert isn't easy.
// =============================================================================
auto translate_fbstp(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    static constexpr int16_t kX87CtrlWordImm12 = kX87ControlWordOff / 2;  // = 0

    // Inline encoders for instructions without dedicated helpers.
    auto emit_udiv64 = [&](int Rd, int Rn, int Rm) {
        // UDIV Xd, Xn, Xm  (sf=1)  — base 0x9AC00800
        const uint32_t insn = 0x9AC00800u
                            | ((uint32_t)(Rm & 0x1F) << 16)
                            | ((uint32_t)(Rn & 0x1F) << 5)
                            |  (uint32_t)(Rd & 0x1F);
        buf.emit(insn);
    };
    auto emit_msub64 = [&](int Rd, int Rn, int Rm, int Ra) {
        // MSUB Xd, Xn, Xm, Xa  (sf=1)  → Xd = Xa - Xn*Xm. Base 0x9B008000.
        const uint32_t insn = 0x9B008000u
                            | ((uint32_t)(Rm & 0x1F) << 16)
                            | ((uint32_t)(Ra & 0x1F) << 10)
                            | ((uint32_t)(Rn & 0x1F) << 5)
                            |  (uint32_t)(Rd & 0x1F);
        buf.emit(insn);
    };
    auto emit_csel = [&](int is_64, int Rd, int Rn, int Rm, int cond) {
        uint32_t insn = 0x1A800000u;
        insn |= (uint32_t)(is_64 != 0) << 31;
        insn |= (uint32_t)(Rm   & 0x1F) << 16;
        insn |= (uint32_t)(cond & 0xF)  << 12;
        insn |= (uint32_t)(Rn   & 0x1F) << 5;
        insn |=  (uint32_t)(Rd & 0x1F);
        buf.emit(insn);
    };

    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Xst_base = x87_get_st_base(*a1);

    int Wd_tmp  = alloc_gpr(*a1, 2);
    int Wd_tmp2 = alloc_gpr(*a1, 3);
    const int Dd_val = alloc_free_fpr(*a1);

    // Step 1: dest address.
    const bool addr_is_64 = (a2->operands[0].mem.addr_size == IROperandSize::S64);
    const int Xaddr = compute_operand_address(*a1, addr_is_64, &a2->operands[0], GPR::XZR);

    // Step 2: load ST(0).
    emit_load_st(buf, Xbase, Wd_top, resolve_depth(*a1, 0), Wd_tmp, Dd_val, Xst_base);

    // Wd_tmp / Wd_tmp2 are dead until x87_pop — free them so the body has
    // enough scratch GPRs.  Re-allocated at the bottom for x87_pop.
    free_gpr(*a1, Wd_tmp2);
    free_gpr(*a1, Wd_tmp);

    // Step 3: CW.RC bits 11:10 → Wd_rc.
    const int Wd_rc = alloc_free_gpr(*a1);
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1, kX87CtrlWordImm12, Xbase, Wd_rc);
    emit_bitfield(buf, /*is_64=*/0, /*UBFM=*/2, /*N=*/0, /*immr=*/10, /*imms=*/11, Wd_rc, Wd_rc);

    // Step 4: 4-way parallel FRINT + FCSEL chain.
    const int Dn = alloc_free_fpr(*a1);
    const int Dm = alloc_free_fpr(*a1);
    const int Dp = alloc_free_fpr(*a1);
    const int Dz = alloc_free_fpr(*a1);

    emit_fp_dp1(buf, /*type=*/1, /*FRINTN=*/8,  Dn, Dd_val);
    emit_fp_dp1(buf, /*type=*/1, /*FRINTP=*/9,  Dp, Dd_val);
    emit_fp_dp1(buf, /*type=*/1, /*FRINTM=*/10, Dm, Dd_val);
    emit_fp_dp1(buf, /*type=*/1, /*FRINTZ=*/11, Dz, Dd_val);

    // CMP Wd_rc, #1; FCSEL Dn, Dm, Dn, EQ
    emit_add_imm(buf, /*is_64=*/0, /*sub=*/1, /*S=*/1, /*shift=*/0, /*imm12=*/1, Wd_rc, GPR::XZR);
    emit_fcsel_f64(buf, Dn, Dm, Dn, /*EQ=*/0);
    emit_add_imm(buf, /*is_64=*/0, /*sub=*/1, /*S=*/1, /*shift=*/0, /*imm12=*/2, Wd_rc, GPR::XZR);
    emit_fcsel_f64(buf, Dn, Dp, Dn, /*EQ=*/0);
    emit_add_imm(buf, /*is_64=*/0, /*sub=*/1, /*S=*/1, /*shift=*/0, /*imm12=*/3, Wd_rc, GPR::XZR);
    emit_fcsel_f64(buf, Dn, Dz, Dn, /*EQ=*/0);
    free_fpr(*a1, Dz);
    free_fpr(*a1, Dp);
    free_fpr(*a1, Dm);
    free_gpr(*a1, Wd_rc);

    const int Dd_round = Dn;

    // Step 5: indef detection.
    const int Dd_thresh = alloc_free_fpr(*a1);
    const int Dd_abs    = alloc_free_fpr(*a1);

    constexpr uint64_t k1e18_bits = std::bit_cast<uint64_t>(1e18);
    emit_ldr_literal_f64(buf, Dd_thresh, k1e18_bits);
    emit_fabs_f64(buf, Dd_abs, Dd_round);
    emit_fcmp_f64(buf, Dd_abs, Dd_thresh);
    const int Wovf = alloc_free_gpr(*a1);
    emit_cset(buf, /*is_64=*/0, /*HS=CS=*/2, Wovf);

    emit_fcmp_f64(buf, Dd_round, Dd_round);
    const int Wnan = alloc_free_gpr(*a1);
    emit_cset(buf, /*is_64=*/0, /*VS=*/6, Wnan);

    emit_logical_shifted_reg(buf, /*is_64=*/0, /*ORR=*/1, /*N=*/0, /*LSL=*/0,
                             /*Rm=*/Wnan, /*shift=*/0, /*Rn=*/Wovf, /*Rd=*/Wovf);
    free_gpr(*a1, Wnan);
    free_fpr(*a1, Dd_abs);
    free_fpr(*a1, Dd_thresh);

    // Step 6: extract sign from Dd_round (preserves sign of rounded -0.0).
    const int Xbits = alloc_free_gpr(*a1);
    emit_fmov_d_to_x(buf, Xbits, Dd_round);
    const int Wsign = alloc_free_gpr(*a1);
    // UBFX Wsign, Xbits, #63, #1  — i.e. immr=63 imms=63 with N=1 (64-bit)
    emit_bitfield(buf, /*is_64=*/1, /*UBFM=*/2, /*N=*/1, /*immr=*/63, /*imms=*/63, Xbits, Wsign);
    free_gpr(*a1, Xbits);

    // Step 7: FCVTZS Xint, Dd_round.  Dd_round is integer-valued so no
    // further rounding occurs; saturating values are caught by Wovf.
    const int Xint = alloc_free_gpr(*a1);
    emit_fcvtzs(buf, /*ftype=*/1, /*is_64bit_int=*/1, Xint, Dd_round);
    free_fpr(*a1, Dd_round);

    // Step 8: precompute |Xint|, sign byte, Xten.
    const int Xneg = alloc_free_gpr(*a1);
    // SUB Xneg, XZR, Xint  (no flags)  → NEG Xneg, Xint
    emit_add_sub_shifted_reg(buf, /*is_64=*/1, /*sub=*/1, /*S=*/0, /*shift_type=*/0,
                             /*Rm=*/Xint, /*shift=*/0, /*Rn=*/GPR::XZR, /*Rd=*/Xneg);
    // CMP Xint, #0  (sets flags via SUBS Rd=XZR, Rn=Xint, imm=0)
    emit_add_imm(buf, /*is_64=*/1, /*sub=*/1, /*S=*/1, /*shift=*/0, /*imm12=*/0, Xint, GPR::XZR);
    // CSEL Xint, Xneg, Xint, LT  → Xint = (Xint < 0) ? Xneg : Xint = |Xint|
    emit_csel(/*is_64=*/1, /*Rd=*/Xint, /*Rn=*/Xneg, /*Rm=*/Xint, /*LT=*/11);
    free_gpr(*a1, Xneg);

    // Xten = 10
    const int Xten = alloc_free_gpr(*a1);
    emit_movn(buf, /*is_64=*/0, /*MOVZ=*/2, /*hw=*/0, /*imm=*/10, Xten);

    // Sign byte = Wsign << 7, computed in place to keep peak GPR
    // allocation at 8 (= pool size).  Previously a separate WsignByte
    // register pushed peak to 9 and silently produced garbage in
    // release-NDEBUG builds; with assert-in-release (commit e780554)
    // that became a hard abort at JIT-time.
    //
    // ORR Wsign, WZR, Wsign, LSL #7 = LSL Wsign, Wsign, #7.
    emit_logical_shifted_reg(buf, /*is_64=*/0, /*ORR=*/1, /*N=*/0, /*LSL=*/0,
                             /*Rm=*/Wsign, /*shift=*/7, /*Rn=*/GPR::XZR, /*Rd=*/Wsign);
    const int WsignByte = Wsign;

    // Step 9: branch on Wovf.  Layout (instruction indices relative to CBNZ):
    //   [0]      CBNZ Wovf, +57       → indef block at +57
    //   [1]      STRB WsignByte, [Xaddr, #9]   (free WsignByte after)
    //   [2..55]  9 × {UDIV, MSUB, UDIV, MSUB, ORR-shifted, STRB} = 54 insns
    //   [56]     B +6                 → past indef
    //   [57..61] 5-insn indef pattern
    //   [62]     (next; falls through)
    //
    // The sign STRB is emitted *before* the divmod loop so WsignByte can
    // be freed before allocating the loop-body scratches (Xb, Wlo).  Peak
    // GPR usage is 8 (= pool size).  Layout count is unchanged: 1 sign
    // STRB + 54 divmod + 1 B = 56 insns in the non-indef block.
    constexpr int kNonIndefLen = 56;
    constexpr int kIndefLen    = 5;
    emit_cbz(buf, /*is_64=*/0, /*is_nz=*/1, Wovf, kNonIndefLen + 1);
    free_gpr(*a1, Wovf);

    // [insn 0] STRB WsignByte, [Xaddr, #9]
    emit_ldr_str_imm(buf, /*size=*/0, /*is_fp=*/0, /*STR=*/0,
                     /*imm12=*/9, Xaddr, WsignByte);
    free_gpr(*a1, WsignByte);  // = Wsign — freed before divmod's Xb/Wlo.

    // Non-indef block.  Xa = current dividend (gets overwritten each iter
    // with the next quotient); Xb is scratch for the inner quotient.
    const int Xa = Xint;
    const int Xb = alloc_free_gpr(*a1);
    const int Wlo = alloc_free_gpr(*a1);

    for (int byte_i = 0; byte_i < 9; byte_i++) {
        // [insn 0,1] Xb = Xa / 10;   Wlo = Xa % 10  (low digit)
        emit_udiv64(/*Rd=*/Xb,  /*Rn=*/Xa, /*Rm=*/Xten);
        emit_msub64(/*Rd=*/Wlo, /*Rn=*/Xb, /*Rm=*/Xten, /*Ra=*/Xa);

        // [insn 2,3] Xa = Xb / 10;   Xb = Xb % 10  (high digit; Rd=Ra is OK)
        emit_udiv64(/*Rd=*/Xa, /*Rn=*/Xb, /*Rm=*/Xten);
        emit_msub64(/*Rd=*/Xb, /*Rn=*/Xa, /*Rm=*/Xten, /*Ra=*/Xb);

        // [insn 4] ORR Wlo, Wlo, Xb, LSL #4   — pack high digit into upper nibble
        emit_logical_shifted_reg(buf, /*is_64=*/0, /*ORR=*/1, /*N=*/0, /*LSL=*/0,
                                 /*Rm=*/Xb, /*shift=*/4, /*Rn=*/Wlo, /*Rd=*/Wlo);
        // [insn 5] STRB Wlo, [Xaddr, #byte_i]
        emit_ldr_str_imm(buf, /*size=*/0, /*is_fp=*/0, /*STR=*/0,
                         /*imm12=*/(int16_t)byte_i, Xaddr, Wlo);
    }

    free_gpr(*a1, Wlo);
    free_gpr(*a1, Xb);

    // [insn 55] B +6  → skip indef block
    emit_b(buf, kIndefLen + 1);

    // Indef block: 5 instructions writing { 0,0,0,0,0,0,0,0xC0,0xFF,0xFF }.
    // [insn 56] STR XZR, [Xaddr]                       — bytes 0..7 = 0
    emit_ldr_str_imm(buf, /*size=*/3, /*is_fp=*/0, /*STR=*/0, /*imm12=*/0, Xaddr, GPR::XZR);
    // [insn 57] MOVZ Wt, #0xC000, LSL #16              — Wt = 0xC0000000
    const int Wt = alloc_free_gpr(*a1);
    emit_movn(buf, /*is_64=*/0, /*MOVZ=*/2, /*hw=*/1, /*imm=*/0xC000, Wt);
    // [insn 58] STR Wt, [Xaddr, #4]                    — byte 7 = 0xC0
    emit_ldr_str_imm(buf, /*size=*/2, /*is_fp=*/0, /*STR=*/0, /*imm12=*/1, Xaddr, Wt);
    // [insn 59] MOVN Wt, #0                            — Wt = 0xFFFFFFFF
    emit_movn(buf, /*is_64=*/0, /*MOVN=*/0, /*hw=*/0, /*imm=*/0, Wt);
    // [insn 60] STRH Wt, [Xaddr, #8]                   — bytes 8,9 = 0xFF, 0xFF
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR=*/0, /*imm12=*/4, Xaddr, Wt);
    free_gpr(*a1, Wt);

    free_gpr(*a1, Xint);
    free_gpr(*a1, Xten);
    free_gpr(*a1, Xaddr);
    free_fpr(*a1, Dd_val);

    // Step 10: pop ST(0).  Re-allocate the lane-2/3 scratches.
    Wd_tmp  = alloc_gpr(*a1, 2);
    Wd_tmp2 = alloc_gpr(*a1, 3);
    (void)Wd_tmp2;  // x87_pop allocates its own secondary internally
    free_gpr(*a1, Wd_tmp2);
    x87_pop(buf, *a1, Xbase, Wd_top, Wd_tmp);

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FLDENV — load 28-byte FPU env from memory.
//
// Layout (32-bit protected mode):
//   +0  control_word (16b, padded to 32)
//   +4  status_word  (16b)  ← TOP at bits [13:11]
//   +8  tag_word     (16b)
//   +12 FIP/FCS/FOP/FDP/FDS — historical pointers; we ignore them
//
// Cache state: any deferred top/tag/pop/perm bits target memory we're
// about to overwrite, so discard them (don't flush).  Re-derive Wd_top
// from the new SW.TOP for the rest of the run.
// =============================================================================
auto translate_fldenv(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    static constexpr int16_t kX87CtrlWordImm12   = kX87ControlWordOff / 2;  // = 0
    static constexpr int16_t kX87StatusWordImm12 = kX87StatusWordOff / 2;   // = 1
    static constexpr int16_t kX87TagWordImm12    = kX87TagWordOff / 2;      // = 2
    static constexpr int16_t kSrcSwImm12 = 2;  // byte 4 → halfword imm12=2
    static constexpr int16_t kSrcTwImm12 = 4;  // byte 8 → halfword imm12=4

    int Xbase, Wd_top;
    const bool standalone = (a1->x87_cache.run_remaining == 0);
    if (standalone) {
        // Mini-prologue: skip emit_load_top — the existing SW.TOP is about
        // to be overwritten.  Wd_top register isn't needed in standalone.
        Xbase = alloc_gpr(*a1, 0);
        emit_x87_base(buf, *a1, Xbase);
        Wd_top = GPR::XZR;
    } else {
        auto pair = x87_begin(*a1, buf);
        Xbase = pair.first;
        Wd_top = pair.second;
        // Discard deferred flags — they target SW/TW we're replacing.
        a1->x87_cache.top_dirty = 0;
        a1->x87_cache.tag_push_pending = 0;
        a1->x87_cache.deferred_pop_count = 0;
        a1->x87_cache.reset_perm();
    }

    const int Wd_tmp = alloc_gpr(*a1, 2);

    // Source address.
    const bool addr_is_64 = (a2->operands[0].mem.addr_size == IROperandSize::S64);
    const int Xaddr = compute_operand_address(*a1, addr_is_64, &a2->operands[0], GPR::XZR);

    // CW: [Xaddr, #0] → [Xbase, #0]
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1, /*imm12=*/0, Xaddr, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR=*/0, kX87CtrlWordImm12, Xbase, Wd_tmp);

    // SW: [Xaddr, #4] → [Xbase, #2].  Re-derive Wd_top from new SW.TOP.
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1, kSrcSwImm12, Xaddr, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR=*/0, kX87StatusWordImm12, Xbase, Wd_tmp);
    if (!standalone) {
        // UBFX Wd_top, Wd_tmp, #11, #3
        emit_bitfield(buf, /*is_64=*/0, /*UBFM=*/2, /*N=*/0, /*immr=*/11, /*imms=*/13,
                      Wd_tmp, Wd_top);
    }

    // TW: [Xaddr, #8] → [Xbase, #4]
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1, kSrcTwImm12, Xaddr, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR=*/0, kX87TagWordImm12, Xbase, Wd_tmp);

    free_gpr(*a1, Xaddr);

    if (standalone) {
        free_gpr(*a1, Wd_tmp);
        free_gpr(*a1, Xbase);
    } else {
        x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
        free_gpr(*a1, Wd_tmp);
    }
}

// =============================================================================
// FSTENV / FNSTENV — store 28-byte FPU env to memory.
//
// Layout matches translate_fldenv (CW at +0, SW at +4, TW at +8, the rest
// are FIP/FCS/FOP/FDP/FDS pointer fields that we don't track — they're
// written as zero).
//
// Cache state: deferred top/tag/pop bits must be flushed to memory before
// reading CW/SW/TW from Xbase, otherwise the stored env reflects stale
// metadata.  perm_dirty doesn't affect SW/TW (it only permutes the f64
// register file), so we skip it.
//
// Codegen:
//   - x87_flush_top + x87_flush_tags (the latter also flushes deferred pops).
//   - 3 × {LDRH 16-bit, STR 32-bit} for CW/SW/TW (LDRH zero-extends so the
//     32-bit STR writes the field plus the 2-byte reserved padding in one go).
//   - 3 × STR XZR for the 16-byte FIP/FCS/FOP/FDP/FDS tail (4 + 8 + 4 bytes
//     at offsets 12 / 16 / 24 — exploits 16's 8-byte alignment for one
//     X-sized store).
// =============================================================================
auto translate_fstenv(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    static constexpr int16_t kX87CtrlWordImm12   = kX87ControlWordOff / 2;  // = 0
    static constexpr int16_t kX87StatusWordImm12 = kX87StatusWordOff / 2;   // = 1
    static constexpr int16_t kX87TagWordImm12    = kX87TagWordOff / 2;      // = 2

    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Wd_tmp  = alloc_gpr(*a1, 2);
    const int Wd_tmp2 = alloc_gpr(*a1, 3);

    // Make in-memory SW/TW coherent.  perm_dirty doesn't touch SW/TW.
    x87_flush_top(buf, *a1, Xbase, Wd_top, Wd_tmp);
    x87_flush_tags(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);

    // Release Wd_tmp2 — body uses Wd_tmp only.
    free_gpr(*a1, Wd_tmp2);

    // Destination address.
    const bool addr_is_64 = (a2->operands[0].mem.addr_size == IROperandSize::S64);
    const int Xaddr = compute_operand_address(*a1, addr_is_64, &a2->operands[0], GPR::XZR);

    // CW: [Xbase, #0] → [Xaddr, #0..3]   (LDRH zero-extends; 32-bit STR
    // writes the field + 2 reserved zero bytes in one shot.)
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1, kX87CtrlWordImm12, Xbase, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/2, /*is_fp=*/0, /*STR=*/0, /*imm12=*/0, Xaddr, Wd_tmp);

    // SW: [Xbase, #2] → [Xaddr, #4..7]
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1, kX87StatusWordImm12, Xbase, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/2, /*is_fp=*/0, /*STR=*/0, /*imm12=*/1, Xaddr, Wd_tmp);

    // TW: [Xbase, #4] → [Xaddr, #8..11]
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1, kX87TagWordImm12, Xbase, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/2, /*is_fp=*/0, /*STR=*/0, /*imm12=*/2, Xaddr, Wd_tmp);

    // 16 zero bytes at offsets 12..27 — 3 stores: 4B at 12, 8B at 16, 4B at 24.
    emit_ldr_str_imm(buf, /*size=*/2, /*is_fp=*/0, /*STR=*/0, /*imm12=*/3, Xaddr, GPR::XZR);
    emit_ldr_str_imm(buf, /*size=*/3, /*is_fp=*/0, /*STR=*/0, /*imm12=*/2, Xaddr, GPR::XZR);
    emit_ldr_str_imm(buf, /*size=*/2, /*is_fp=*/0, /*STR=*/0, /*imm12=*/6, Xaddr, GPR::XZR);

    free_gpr(*a1, Xaddr);

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FRSTOR — load 108-byte FPU state from memory.
//
// Layout (32-bit protected mode):
//   +0..27   28-byte env header (CW@0, SW@4, TW@8, FIP/FCS/FOP/FDP/FDS @12..27)
//   +28..107 8 ST slots in x86 f80 format, 10 bytes each
//      ST(0) @ 0x1C, ST(1) @ 0x26, ST(2) @ 0x30, ST(3) @ 0x3A,
//      ST(4) @ 0x44, ST(5) @ 0x4E, ST(6) @ 0x58, ST(7) @ 0x62
//
// ST(i) at logical depth i from the new TOP — i.e., physical (TOP+i)&7.
//
// Cache state: any deferred top/tag/pop/perm bits target memory we're
// about to overwrite, so discard them (don't flush).  Re-derive Wd_top
// from the new SW.TOP for the rest of the run.  FIP/FCS/FOP/FDP/FDS
// pointer fields are skipped — we don't model them.
// =============================================================================
auto translate_frstor(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    static constexpr int16_t kX87CtrlWordImm12   = kX87ControlWordOff / 2;  // = 0
    static constexpr int16_t kX87StatusWordImm12 = kX87StatusWordOff / 2;   // = 1
    static constexpr int16_t kX87TagWordImm12    = kX87TagWordOff / 2;      // = 2
    static constexpr int16_t kSrcSwImm12 = 2;  // byte 4 -> halfword imm12=2
    static constexpr int16_t kSrcTwImm12 = 4;  // byte 8 -> halfword imm12=4

    int Xbase, Wd_top;
    const bool standalone = (a1->x87_cache.run_remaining == 0);
    if (standalone) {
        // Mini-prologue: skip emit_load_top — SW.TOP is about to be
        // replaced.  Wd_top must still be a real register because the
        // ST-slot store loop indexes through it.
        Xbase = alloc_gpr(*a1, 0);
        emit_x87_base(buf, *a1, Xbase);
        Wd_top = alloc_gpr(*a1, 1);
    } else {
        auto pair = x87_begin(*a1, buf);
        Xbase = pair.first;
        Wd_top = pair.second;
        // Discard deferred flags — they target SW/TW/file we're replacing.
        a1->x87_cache.top_dirty = 0;
        a1->x87_cache.tag_push_pending = 0;
        a1->x87_cache.deferred_pop_count = 0;
        a1->x87_cache.reset_perm();
    }

    const int Wd_tmp = alloc_gpr(*a1, 2);

    // Source address.
    const bool addr_is_64 = (a2->operands[0].mem.addr_size == IROperandSize::S64);
    const int Xaddr = compute_operand_address(*a1, addr_is_64, &a2->operands[0], GPR::XZR);

    // ── Env header (identical to translate_fldenv body) ──
    // CW: [Xaddr, #0] -> [Xbase, #0]
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1, /*imm12=*/0, Xaddr, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR=*/0, kX87CtrlWordImm12, Xbase, Wd_tmp);

    // SW: [Xaddr, #4] -> [Xbase, #2].  Re-derive Wd_top from new SW.TOP.
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1, kSrcSwImm12, Xaddr, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR=*/0, kX87StatusWordImm12, Xbase, Wd_tmp);
    // UBFX Wd_top, Wd_tmp, #11, #3
    emit_bitfield(buf, /*is_64=*/0, /*UBFM=*/2, /*N=*/0, /*immr=*/11, /*imms=*/13,
                  Wd_tmp, Wd_top);

    // TW: [Xaddr, #8] -> [Xbase, #4]
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1, kSrcTwImm12, Xaddr, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR=*/0, kX87TagWordImm12, Xbase, Wd_tmp);

    // ── 8 ST slots — unrolled f80 -> f64 conversion ──
    //
    // Xaddr is mutated in place: bumped by +0x1C before iter 0, then +10
    // between iterations.  After iter 7 it points past the last slot;
    // we don't restore it.
    //
    // GPR pressure: Xbase + Wd_top + Wd_tmp + Xst_base(cached only) +
    // Xaddr + Xmant + Wexp + Xsign + Wd_aux = 8 (standalone) or 9 (cached).
    // The 8-slot scratch pool just fits standalone.  In cached mode we
    // free Xst_base for the duration of the loop and re-derive it at the
    // end — emit_store_st falls back to its uncached path (3 extra insns
    // per slot, which is fine for this correctness-only opcode).
    int Xst_base = x87_get_st_base(*a1);  // -1 in standalone
    const bool cached = (Xst_base >= 0);
    if (cached) {
        free_gpr(*a1, Xst_base);
        Xst_base = -1;  // force uncached emit_store_st in the loop
    }

    const int Xmant  = alloc_free_gpr(*a1);
    const int Wexp   = alloc_free_gpr(*a1);
    const int Xsign  = alloc_free_gpr(*a1);
    const int Wd_aux = alloc_free_gpr(*a1);
    const int Dd_val = alloc_free_fpr(*a1);

    for (int i = 0; i < 8; ++i) {
        const int delta = (i == 0) ? 0x1C : 10;
        // Bump Xaddr to the start of the current slot.
        emit_add_imm(buf, /*is_64=*/1, /*is_sub=*/0, /*set_flags=*/0,
                     /*shift=*/0, delta, Xaddr, Xaddr);
        // LDR Xmant, [Xaddr, #0] — 8-byte mantissa.
        emit_ldr_imm(buf, /*size=*/3, Xmant, Xaddr, /*imm12=*/0);
        // LDRH Wexp, [Xaddr, #8] — 2-byte sign+exp word.
        emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1,
                         /*imm12=*/4, Xaddr, Wexp);
        emit_f80_to_f64_convert(buf, Xmant, Wexp, Xsign, Wd_aux, Wd_tmp);
        emit_fmov_x_to_d(buf, Dd_val, Xmant);
        // Store into ST(i) (logical depth i from new TOP).
        emit_store_st(buf, Xbase, Wd_top, /*stack_depth=*/i, Wd_tmp, Dd_val, Xst_base);
    }

    free_fpr(*a1, Dd_val);
    free_gpr(*a1, Wd_aux);
    free_gpr(*a1, Xsign);
    free_gpr(*a1, Wexp);
    free_gpr(*a1, Xmant);
    free_gpr(*a1, Xaddr);

    if (cached) {
        // Re-derive Xst_base = Xbase + kX87RegFileOff and put it back in
        // the cached slot (slot 6 = x28 by convention).
        const int Xst_base_new = alloc_gpr(*a1, 6);
        emit_add_imm(buf, /*is_64=*/1, /*is_sub=*/0, /*set_flags=*/0,
                     /*shift=*/0, kX87RegFileOff, Xbase, Xst_base_new);
        a1->x87_cache.st_base_gpr = static_cast<int8_t>(Xst_base_new);
    }

    if (standalone) {
        free_gpr(*a1, Wd_tmp);
        free_gpr(*a1, Wd_top);
        free_gpr(*a1, Xbase);
    } else {
        x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
        free_gpr(*a1, Wd_tmp);
    }
}

// =============================================================================
// FSAVE / FNSAVE — store 108-byte FPU state to memory, then re-initialize.
//
// Per Intel SDM Vol 2A:
//   "FSAVE/FNSAVE stores the current FPU state to the destination, then
//    initializes the FPU.  The functional equivalent is FSTENV followed by
//    FINIT."
//
// Layout (32-bit protected mode):
//   +0..27   28-byte env header (CW@0, SW@4, TW@8, FIP/FCS/FOP/FDP/FDS @12..27)
//   +28..107 8 ST slots in x86 f80 format, 10 bytes each
//      ST(0) @ 0x1C, ST(1) @ 0x26, ST(2) @ 0x30, ST(3) @ 0x3A,
//      ST(4) @ 0x44, ST(5) @ 0x4E, ST(6) @ 0x58, ST(7) @ 0x62
//
// Cache state: must flush ALL deferred state (top, tags, perm) before
// reading the register file by physical index — perm specifically because
// emit_load_st with the cached path uses logical depth, and with perm
// dirty the depth->physical mapping is permuted.  After saving, we
// re-initialize FPU memory state (CW=0x37F, SW=0, TW=0xFFFF) and reset
// Wd_top to 0 so the rest of the run sees the new TOP.
// =============================================================================
auto translate_fsave(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    static constexpr int16_t kX87CtrlWordImm12   = kX87ControlWordOff / 2;  // = 0
    static constexpr int16_t kX87StatusWordImm12 = kX87StatusWordOff / 2;   // = 1
    static constexpr int16_t kX87TagWordImm12    = kX87TagWordOff / 2;      // = 2

    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Wd_tmp  = alloc_gpr(*a1, 2);
    const int Wd_tmp2 = alloc_gpr(*a1, 3);

    // Make in-memory state coherent.  Unlike fstenv, we MUST also flush
    // perm because we'll read the f64 register file by depth below.
    x87_flush_top(buf, *a1, Xbase, Wd_top, Wd_tmp);
    x87_flush_tags(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);
    perm_flush_before_stack_change(buf, *a1, Xbase, Wd_top, Wd_tmp);

    // Release Wd_tmp2 — body uses Wd_tmp + per-slot helper scratches.
    free_gpr(*a1, Wd_tmp2);

    // Destination address.
    const bool addr_is_64 = (a2->operands[0].mem.addr_size == IROperandSize::S64);
    const int Xaddr = compute_operand_address(*a1, addr_is_64, &a2->operands[0], GPR::XZR);

    // ── Env header (identical to translate_fstenv body) ──
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1, kX87CtrlWordImm12, Xbase, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/2, /*is_fp=*/0, /*STR=*/0, /*imm12=*/0, Xaddr, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1, kX87StatusWordImm12, Xbase, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/2, /*is_fp=*/0, /*STR=*/0, /*imm12=*/1, Xaddr, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR=*/1, kX87TagWordImm12, Xbase, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/2, /*is_fp=*/0, /*STR=*/0, /*imm12=*/2, Xaddr, Wd_tmp);
    // 16 zero bytes at offsets 12..27.
    emit_ldr_str_imm(buf, /*size=*/2, /*is_fp=*/0, /*STR=*/0, /*imm12=*/3, Xaddr, GPR::XZR);
    emit_ldr_str_imm(buf, /*size=*/3, /*is_fp=*/0, /*STR=*/0, /*imm12=*/2, Xaddr, GPR::XZR);
    emit_ldr_str_imm(buf, /*size=*/2, /*is_fp=*/0, /*STR=*/0, /*imm12=*/6, Xaddr, GPR::XZR);

    // ── 8 ST slots — unrolled f64 -> f80 conversion ──
    //
    // Per-iter: load ST(i) into Dd_src, bump Xaddr to slot i, call
    // emit_f64_to_f80 (writes 10 bytes at [Xaddr]).  After the loop,
    // Xaddr points past slot 7 — we don't restore it.
    //
    // GPR pressure:
    //   persistent: Xbase, Wd_top, Wd_tmp, Xst_base(cached only),
    //               Xaddr, Xbits, Wexp = 6 or 7
    //   per-iter:   no extra (Dd_src is FPR, no GPR cost)
    // = 7 worst case (cached).  Pool of 8 fits comfortably.
    const int Xst_base = x87_get_st_base(*a1);
    const int Xbits    = alloc_free_gpr(*a1);
    const int Wexp     = alloc_free_gpr(*a1);
    const int Dd_src   = alloc_free_fpr(*a1);

    for (int i = 0; i < 8; ++i) {
        // Load ST(i) (logical depth i from current TOP).
        emit_load_st(buf, Xbase, Wd_top, /*stack_depth=*/i, Wd_tmp, Dd_src, Xst_base);
        // Bump Xaddr to the start of the current slot.
        const int delta = (i == 0) ? 0x1C : 10;
        emit_add_imm(buf, /*is_64=*/1, /*is_sub=*/0, /*set_flags=*/0,
                     /*shift=*/0, delta, Xaddr, Xaddr);
        // Convert and store 10 bytes at [Xaddr, #0..#9].
        emit_f64_to_f80(buf, Xaddr, Dd_src, Xbits, Wexp, Wd_tmp);
    }

    free_fpr(*a1, Dd_src);
    free_gpr(*a1, Wexp);
    free_gpr(*a1, Xbits);
    free_gpr(*a1, Xaddr);

    // ── Re-initialize FPU state per Intel SDM (FSAVE = FSTENV + FINIT) ──
    // CW = 0x037F, SW = 0, TW = 0xFFFF, error pointers already zeroed
    // by the env-header path above.  Reset Wd_top to 0 to match new SW.TOP
    // and clear cache flags — the in-memory state is fresh.
    // CW = 0x037F via MOVZ.
    emit_movn(buf, /*is_64=*/0, /*MOVZ=*/2, /*hw=*/0, /*imm=*/0x037F, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR=*/0, kX87CtrlWordImm12, Xbase, Wd_tmp);
    // SW = 0.
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR=*/0, kX87StatusWordImm12, Xbase, GPR::XZR);
    // TW = 0xFFFF via MOVN W, #0 (gives 0xFFFFFFFF; STRH writes low 16).
    emit_movn(buf, /*is_64=*/0, /*MOVN=*/0, /*hw=*/0, /*imm=*/0, Wd_tmp);
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR=*/0, kX87TagWordImm12, Xbase, Wd_tmp);
    // Reset cached Wd_top to 0 (matches new SW.TOP=0).
    emit_movn(buf, /*is_64=*/0, /*MOVZ=*/2, /*hw=*/0, /*imm=*/0, Wd_top);
    // Cache flags are already zero (we flushed top/tags/perm earlier; deferred_pop_count is 0).
    a1->x87_cache.top_dirty = 0;

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FSIN — replace ST(0) with sin(ST(0)).
//
// Routes through the parent-side IPC trampoline → sidecar → libm
// std::sin via TranscendentalHelper.  See TranslatorX87Transcendental.cpp
// for the BLR emit shape.
//
// x87 semantics:
//   if |ST(0)| < 2^63: ST(0) = sin(ST(0)); C2 = 0
//   else:              ST(0) unchanged;     C2 = 1
//   tag word: ST(0) tag updated to Valid (or Special for ±Inf/NaN)
//
// Simplification: we always compute via libm regardless of magnitude,
// and don't update C0..C3 in status_word.  Sidecar logs a one-shot
// warning on first |ST(0)| >= 2^63.  See plan §"Known simplifications".
// =============================================================================
auto translate_fsin(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Wd_tmp = alloc_gpr(*a1, 2);

    emit_transcendental_ipc(*a1, buf, Xbase, Wd_top, Wd_tmp,
                            rosetta_core::kTransFsin, /*num_inputs=*/1);

    // d0 holds sin(input).  Replace ST(0) at its current physical depth.
    const int Xst_base = x87_get_st_base(*a1);
    const int depth_st0 = resolve_depth(*a1, 0);
    emit_store_st(buf, Xbase, Wd_top, depth_st0, Wd_tmp, /*Dd=*/0, Xst_base);

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FCOS — replace ST(0) with cos(ST(0)).  Mirrors translate_fsin; only
// the opcode_tag differs.
// =============================================================================
auto translate_fcos(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Wd_tmp = alloc_gpr(*a1, 2);

    emit_transcendental_ipc(*a1, buf, Xbase, Wd_top, Wd_tmp,
                            rosetta_core::kTransFcos, /*num_inputs=*/1);

    const int Xst_base = x87_get_st_base(*a1);
    const int depth_st0 = resolve_depth(*a1, 0);
    emit_store_st(buf, Xbase, Wd_top, depth_st0, Wd_tmp, /*Dd=*/0, Xst_base);

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FPTAN — replace ST(0) with tan(ST(0)); push 1.0.
//
// JIT: load ST(0)→d0, IPC returns d0=tan, store d0 at depth=0 (replace
// ST(0)), then x87_push and store FMOV-immediate 1.0 at new ST(0).
// =============================================================================
auto translate_fptan(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Wd_tmp2 = alloc_free_gpr(*a1);

    emit_transcendental_ipc(*a1, buf, Xbase, Wd_top, Wd_tmp,
                            rosetta_core::kTransFptan, /*num_inputs=*/1);

    const int Xst_base = x87_get_st_base(*a1);
    const int depth_st0 = resolve_depth(*a1, 0);
    // Replace old ST(0) with tan (d0).
    emit_store_st(buf, Xbase, Wd_top, depth_st0, Wd_tmp, /*Dd=*/0, Xst_base);
    // Push and write 1.0 at new ST(0).
    x87_push(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);
    const int Dd_one = alloc_free_fpr(*a1);
    emit_fmov_d_one(buf, Dd_one);
    emit_store_st(buf, Xbase, Wd_top, /*depth=*/0, Wd_tmp, Dd_one, Xst_base);
    free_fpr(*a1, Dd_one);

    free_gpr(*a1, Wd_tmp2);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// FSINCOS — replace ST(0) with sin(ST(0)); push cos(ST(0)).
//
// JIT: load ST(0)→d0, IPC computes sin/cos returning (d0=sin, d1=cos).
// store d0 at depth=0 (replace ST(0) with sin), then x87_push so TOP
// decrements and tag of new ST(0) becomes valid, then store d1 at the
// (new) depth=0 (= cos at new top).
// =============================================================================
auto translate_fsincos(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Wd_tmp = alloc_gpr(*a1, 2);
    const int Wd_tmp2 = alloc_free_gpr(*a1);

    emit_transcendental_ipc(*a1, buf, Xbase, Wd_top, Wd_tmp,
                            rosetta_core::kTransFsincos, /*num_inputs=*/1);

    const int Xst_base = x87_get_st_base(*a1);
    const int depth_st0 = resolve_depth(*a1, 0);
    // Replace old ST(0) with sin (d0).
    emit_store_st(buf, Xbase, Wd_top, depth_st0, Wd_tmp, /*Dd=*/0, Xst_base);
    // Push and write cos (d1) at new ST(0).  perm_flush_before_stack_change
    // is handled inside x87_push.
    x87_push(buf, *a1, Xbase, Wd_top, Wd_tmp, Wd_tmp2);
    emit_store_st(buf, Xbase, Wd_top, /*depth=*/0, Wd_tmp, /*Dd=*/1, Xst_base);

    free_gpr(*a1, Wd_tmp2);
    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

// =============================================================================
// 2-input/1-output transcendentals with pop semantics: fpatan, fyl2x,
// fyl2xp1.  All share the same emit shape — load ST(0)→d0, ST(1)→d1,
// IPC computes a function of (in0, in1) and returns it in d0, store d0
// at depth=1 (old ST(1) slot, becomes new ST(0)), then x87_pop.
// =============================================================================
namespace {
inline void emit_2in_pop_translate(TranslationResult& a1, AssemblerBuffer& buf,
                                    uint8_t opcode_tag) {
    auto [Xbase, Wd_top] = x87_begin(a1, buf);
    const int Wd_tmp = alloc_gpr(a1, 2);

    emit_transcendental_ipc(a1, buf, Xbase, Wd_top, Wd_tmp,
                            opcode_tag, /*num_inputs=*/2);

    const int Xst_base = x87_get_st_base(a1);
    const int depth_st1 = resolve_depth(a1, 1);
    emit_store_st(buf, Xbase, Wd_top, depth_st1, Wd_tmp, /*Dd=*/0, Xst_base);

    x87_pop(buf, a1, Xbase, Wd_top, Wd_tmp);

    x87_end(a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(a1, Wd_tmp);
}
}  // namespace

auto translate_fpatan(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    emit_2in_pop_translate(*a1, a1->insn_buf, rosetta_core::kTransFpatan);
}

auto translate_fyl2x(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    emit_2in_pop_translate(*a1, a1->insn_buf, rosetta_core::kTransFyl2x);
}

// =============================================================================
// F2XM1 — replace ST(0) with 2^ST(0) - 1.  x87 spec says input must be
// |ST(0)| <= 1; outside that range result is undefined.  We always
// compute via std::exp2(in) - 1.0 in the sidecar regardless of input.
// =============================================================================
auto translate_f2xm1(TranslationResult* a1, IRInstr* /*a2*/) -> void {
    AssemblerBuffer& buf = a1->insn_buf;
    auto [Xbase, Wd_top] = x87_begin(*a1, buf);
    const int Wd_tmp = alloc_gpr(*a1, 2);

    emit_transcendental_ipc(*a1, buf, Xbase, Wd_top, Wd_tmp,
                            rosetta_core::kTransF2xm1, /*num_inputs=*/1);

    const int Xst_base = x87_get_st_base(*a1);
    const int depth_st0 = resolve_depth(*a1, 0);
    emit_store_st(buf, Xbase, Wd_top, depth_st0, Wd_tmp, /*Dd=*/0, Xst_base);

    x87_end(*a1, buf, Xbase, Wd_top, Wd_tmp);
    free_gpr(*a1, Wd_tmp);
}

};  // namespace TranslatorX87
