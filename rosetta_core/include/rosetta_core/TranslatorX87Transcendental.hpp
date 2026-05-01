#pragma once

#include <cstdint>

#include "rosetta_core/AssemblerBuffer.h"
#include "rosetta_core/TranslationResult.h"

// JIT-emit primitives for the 10 x87 transcendental opcodes (f2xm1,
// fsin, fcos, fsincos, fptan, fpatan, fyl2x, fyl2xp1, fprem, fprem1).
// All inline ARM64 — no IPC.  Each function leaves its result in d0
// (and d1 for fsincos's second output).  Caller is responsible for
// the per-op writeback semantics (replace ST(0), push, pop, etc.) —
// see translate_<op> in TranslatorX87.cpp.
//
// Pre/post conditions:
//   - Caller must have done x87_begin and own (Xbase, Wd_top, Wd_tmp).
//   - Polynomial constants and lookup tables live in the trailing pad
//     at the address registered via set_transcendental_constants_addr;
//     each emit materialises that address into a scratch GPR.
namespace TranslatorX87 {

// Inline ARM64 polynomial approximations of sin(x) / cos(x).  Source:
// ARM-software/optimized-routines' math/aarch64/advsimd/{sin,cos}.c
// scalarised.  ULP <= 3.3 in [-pi/2,pi/2], 2.73 ULP in [-2^23, 2^23].
// For |x| >= 2^23 the 3-step Cody-Waite reduction degrades and the
// result is junk — accepted (no fallback).
//
// Both share three helpers internally (emit_trig_range_reduce,
// emit_sin_poly_estrin, emit_apply_qn_sign) — see the .cpp.
void emit_inline_fsin(TranslationResult& a1, AssemblerBuffer& buf,
                      int Xbase, int Wd_top, int Wd_tmp);

void emit_inline_fcos(TranslationResult& a1, AssemblerBuffer& buf,
                      int Xbase, int Wd_top, int Wd_tmp);

// fsincos: replace ST(0) with sin(ST(0)); push cos(ST(0)) onto the
// stack.  Convention: sin in d0, cos in d1.  Both pipelines share the
// loaded Dx and the Xconst pointer; range reductions and sign flips
// are distinct.
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

// fptan: replace ST(0) with tan(ST(0)) (caller pushes 1.0 afterwards).
// Order-7 Estrin in r² + double-angle recombination.  Result in d0.
void emit_inline_fptan(TranslationResult& a1, AssemblerBuffer& buf,
                        int Xbase, int Wd_top, int Wd_tmp);

// fprem: ST(0) := fmod(ST(0), ST(1)).  Single-shot via FDIV + FRINTZ
// + FMSUB (matches the simplification the prior IPC sidecar used —
// stock x87 is iterative).  Result in d0.
void emit_inline_fprem(TranslationResult& a1, AssemblerBuffer& buf,
                        int Xbase, int Wd_top, int Wd_tmp);

// fprem1: ST(0) := IEEE-remainder(ST(0), ST(1)).  Single-shot via
// FDIV + FRINTN + FMSUB.  Same simplification as fprem; FRINTN uses
// ties-to-even rounding to match std::remainder.  Result in d0.
void emit_inline_fprem1(TranslationResult& a1, AssemblerBuffer& buf,
                         int Xbase, int Wd_top, int Wd_tmp);

}  // namespace TranslatorX87
