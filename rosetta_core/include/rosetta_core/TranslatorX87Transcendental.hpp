#pragma once

#include <cstdint>

#include "rosetta_core/AssemblerBuffer.h"
#include "rosetta_core/TranslationResult.h"

// JIT-emit primitive shared by all 10 x87 transcendental opcodes
// (f2xm1, fsin, fcos, fsincos, fptan, fpatan, fyl2x, fyl2xp1, fprem,
// fprem1).  Loads ST(0) (and ST(1) if num_inputs==2) into d0/d1, sets
// x0 = opcode_tag, then BLRs into the parent-side trampoline installed
// at loader time.  On return d0 holds result1; d1 holds result2 (only
// meaningful for num_outputs==2 / fsincos).
//
// Caller is responsible for the per-op writeback semantics (replace
// ST(0), push, pop, etc.) — see translate_fsin / translate_fpatan /
// translate_fsincos in TranslatorX87.cpp for the patterns.
//
// Pre/post conditions:
//   - Caller must have done x87_begin already and own (Xbase, Wd_top, Wd_tmp).
//   - On entry, no caller-saved FPRs (d0..d7, d16..d31) are live across
//     the call.  d0/d1 are clobbered (they're the input/output regs).
//     d24..d31 (FPR scratch pool) are caller-saved by AAPCS64; we don't
//     hold any in this code path so the BLR is safe.
//   - x16 is clobbered (used as branch target).  Cache state held in
//     callee-saved GPRs (x22..x29) survives.
namespace TranslatorX87 {

void emit_transcendental_ipc(TranslationResult& a1, AssemblerBuffer& buf,
                              int Xbase, int Wd_top, int Wd_tmp,
                              uint8_t opcode_tag, int num_inputs);

// Inline ARM64 polynomial approximations of sin(x) / cos(x), replacing
// the IPC roundtrips for fsin / fcos.  Sources:
// ARM-software/optimized-routines' math/aarch64/advsimd/{sin,cos}.c
// scalarised.  ULP <= 3.3 in [-pi/2,pi/2], 2.73 ULP in [-2^23, 2^23].
// For |x| >= 2^23 the 3-step Cody-Waite reduction degrades and the
// result is junk — accepted (no fallback).
//
// Both share three helpers internally (emit_trig_range_reduce,
// emit_sin_poly_estrin, emit_apply_qn_sign) — see the .cpp.
//
// Pre/post conditions match emit_transcendental_ipc: caller has done
// x87_begin and owns (Xbase, Wd_top, Wd_tmp).  Result lands in d0.
void emit_inline_fsin(TranslationResult& a1, AssemblerBuffer& buf,
                      int Xbase, int Wd_top, int Wd_tmp);

void emit_inline_fcos(TranslationResult& a1, AssemblerBuffer& buf,
                      int Xbase, int Wd_top, int Wd_tmp);

// fsincos: replace ST(0) with sin(ST(0)); push cos(ST(0)) onto the
// stack.  Result convention matches the prior IPC path — sin in d0,
// cos in d1.  Both pipelines share the loaded Dx and the Xconst
// pointer; range reductions and sign flips are distinct.
void emit_inline_fsincos(TranslationResult& a1, AssemblerBuffer& buf,
                          int Xbase, int Wd_top, int Wd_tmp);

// f2xm1: replace ST(0) with 2^ST(0) - 1.  Spec input |x| <= 1; the
// inline impl follows optimized-routines' AdvSIMD exp2m1 (table-based,
// 128-entry exp_table + 88-entry small-x scalem1 fixup table).  Result
// in d0.
void emit_inline_f2xm1(TranslationResult& a1, AssemblerBuffer& buf,
                       int Xbase, int Wd_top, int Wd_tmp);

// fyl2x: replace ST(1) with ST(1) * log2(ST(0)); pop.  Inline log2
// follows optimized-routines' AdvSIMD inline_log2 (128-entry
// {invc, log2c} table; ~2 ULP).  Caller (emit_2in_pop_translate)
// handles the writeback at depth 1 and the pop.  Result in d0.
void emit_inline_fyl2x(TranslationResult& a1, AssemblerBuffer& buf,
                        int Xbase, int Wd_top, int Wd_tmp);

// fyl2xp1: replace ST(1) with ST(1) * log2(ST(0) + 1); pop.  Reuses
// the same inline log2 helper as fyl2x — for x87's spec input range
// (-1+ε ≤ ST(0) ≤ 1-√2/2) the simple add-then-log2 form is accurate
// enough; no log1p-style cancellation handling.  Result in d0.
void emit_inline_fyl2xp1(TranslationResult& a1, AssemblerBuffer& buf,
                          int Xbase, int Wd_top, int Wd_tmp);

// fpatan: replace ST(1) with atan2(ST(1), ST(0)); pop.  Order-19
// polynomial port of optimized-routines' AdvSIMD atan2.  Result in d0.
void emit_inline_fpatan(TranslationResult& a1, AssemblerBuffer& buf,
                         int Xbase, int Wd_top, int Wd_tmp);

}  // namespace TranslatorX87
