#pragma once

#include <cstdint>
#include <utility>

#include "rosetta_core/AssemblerBuffer.h"
#include "rosetta_core/TranslationResult.h"

// JIT-emit primitives for the 10 x87 transcendental opcodes (f2xm1,
// fsin, fcos, fsincos, fptan, fpatan, fyl2x, fyl2xp1, fprem, fprem1).
// All inline ARM64 — no IPC.  Each function returns the scratch-pool
// FPR(s) holding its result; the caller stores them back per the op's
// writeback semantics (replace ST(0), push, pop, etc. — see
// translate_<op> in TranslatorX87.cpp) and then frees them.
//
// REGISTER SAFETY: emitted code runs inline inside a stock Rosetta
// block, where v0–v15 hold guest XMM state and v16–v23 can hold stock's
// cached scalars.  Only scratch-pool FPRs (free_fpr_mask) may be
// written.  These helpers used to compute in d0/d1/d2, which corrupted
// live guest xmm registers (e.g. a C `acc += sin(x)` loop accumulator).
//
// Pre/post conditions:
//   - Caller must have done x87_begin and own (Xbase, Wd_top, Wd_tmp).
//   - Polynomial constants and lookup tables live in the trailing pad
//     at the address registered via set_transcendental_constants_addr;
//     each emit materialises that address into a scratch GPR.
namespace TranslatorX87 {

// Mode discriminator for the shared sin/cos polynomial body — selects the
// range-reduction offset (sin: n = round(x/π); cos: n = round(x/π + 0.5)).
enum class TrigReduceMode : std::uint8_t { Sin, Cos };

// FPR-level core of the sin/cos polynomial.  Dx must be a scratch-pool
// FPR; the body takes ownership (frees it after its last read, before
// the polynomial's peak-pressure window) and returns a freshly-owned
// pool FPR holding sin(Dx) / cos(Dx), which the caller must free.
// Caller keeps ownership of Xconst.  This function neither loads from
// nor stores to the x87 stack.  Clearing C0..C3 is the caller's
// responsibility (the wrappers below do it).
int emit_inline_trig_body(TranslationResult& a1, AssemblerBuffer& buf, int Dx, int Xconst,
                          TrigReduceMode mode);

// FPR-level core of fpatan: returns a freshly-owned pool FPR holding
// atan2(Dy_in, Dx_in).  Both inputs must be scratch-pool FPRs; the core
// takes ownership (each is freed after its last read, before the
// polynomial's peak-pressure window).  Caller keeps ownership of Xconst
// and must free the returned FPR.  Does NOT clear C0..C3 (fpatan's spec
// leaves them undefined per `feedback_x87_cc_bits_must_be_cleared.md`).
int emit_inline_fpatan_core(TranslationResult& a1, AssemblerBuffer& buf, int Dy_in, int Dx_in,
                            int Xconst);

// FPR-level core of fptan: returns a freshly-owned pool FPR holding
// tan(Dx_in).  Dx_in must be a scratch-pool FPR; the core takes
// ownership (frees it after its last read, before the peak-pressure
// window).  Caller keeps ownership of Xconst and must free the returned
// FPR.  Does NOT clear C0..C3 (caller must, since fptan's spec defines
// them).
int emit_inline_fptan_core(TranslationResult& a1, AssemblerBuffer& buf, int Dx_in, int Xconst);

// FPR-level core of fyl2x: returns a freshly-owned pool FPR holding
// Dy_in * log2(Dx_in).  Both inputs must be scratch-pool FPRs; the core
// takes ownership (Dx_in doubles as the log2 output slot and becomes the
// result reg; Dy_in stays live until the final fmul — peak is 8 pool
// FPRs, exactly the pool size).  Caller keeps ownership of Xconst and
// must free the returned FPR.  Does NOT clear C0..C3 (fyl2x's spec
// leaves them undefined).
int emit_inline_fyl2x_core(TranslationResult& a1, AssemblerBuffer& buf, int Dy_in, int Dx_in,
                           int Xconst);

// Clear C0/C1/C2/C3 (status_word bits 8/9/10/14).  See
// `feedback_x87_cc_bits_must_be_cleared.md`: ops whose spec defines these
// (fsin/fcos/fsincos/fptan/fprem/fprem1) MUST emit this after their body
// or wine's argument-reduction loops can spin forever (the WoW world-load
// freeze was exactly this).
void emit_clear_x87_cc_bits(TranslationResult& a1, AssemblerBuffer& buf, int Xbase);

// Inline ARM64 polynomial approximations of sin(x) / cos(x).  Source:
// ARM-software/optimized-routines' math/aarch64/advsimd/{sin,cos}.c
// scalarised.  ULP <= 3.3 in [-pi/2,pi/2], 2.73 ULP in [-2^23, 2^23].
// For |x| >= 2^23 the 3-step Cody-Waite reduction degrades and the
// result is junk — accepted (no fallback).
//
// Both share three helpers internally (emit_trig_range_reduce,
// emit_sin_poly_estrin, emit_apply_qn_sign) — see the .cpp.  Return the
// pool FPR holding the result (caller stores + frees).
int emit_inline_fsin(TranslationResult& a1, AssemblerBuffer& buf, int Xbase, int Wd_top,
                     int Wd_tmp);

int emit_inline_fcos(TranslationResult& a1, AssemblerBuffer& buf, int Xbase, int Wd_top,
                     int Wd_tmp);

// fsincos: replace ST(0) with sin(ST(0)); push cos(ST(0)) onto the
// stack.  Returns {sin_fpr, cos_fpr} — both pool-owned by the caller.
// Both pipelines share the Xconst pointer; the loaded input is copied
// once since each trig body consumes its input.
auto emit_inline_fsincos(TranslationResult& a1, AssemblerBuffer& buf, int Xbase, int Wd_top,
                         int Wd_tmp) -> std::pair<int, int>;

// f2xm1: replace ST(0) with 2^ST(0) - 1.  Spec input |x| <= 1; the
// inline impl follows optimized-routines' AdvSIMD exp2m1 (table-based,
// 128-entry exp_table + 88-entry small-x scalem1 fixup table).  Returns
// the pool FPR holding the result (caller stores + frees).
int emit_inline_f2xm1(TranslationResult& a1, AssemblerBuffer& buf, int Xbase, int Wd_top,
                      int Wd_tmp);

// fyl2x: replace ST(1) with ST(1) * log2(ST(0)); pop.  Inline log2
// follows optimized-routines' AdvSIMD inline_log2 (128-entry
// {invc, log2c} table; ~2 ULP).  Caller (translate_fyl2x) handles the
// writeback at depth 1 and the pop.  Returns the pool FPR holding the
// result (caller stores + frees).
int emit_inline_fyl2x(TranslationResult& a1, AssemblerBuffer& buf, int Xbase, int Wd_top,
                      int Wd_tmp);

// fyl2xp1: replace ST(1) with ST(1) * log2(ST(0) + 1); pop.  Reuses
// the same inline log2 helper as fyl2x — for x87's spec input range
// (-1+ε ≤ ST(0) ≤ 1-√2/2) the simple add-then-log2 form is accurate
// enough; no log1p-style cancellation handling.  Returns the pool FPR
// holding the result (caller stores + frees).
int emit_inline_fyl2xp1(TranslationResult& a1, AssemblerBuffer& buf, int Xbase, int Wd_top,
                        int Wd_tmp);

// fpatan: replace ST(1) with atan2(ST(1), ST(0)); pop.  Order-19
// polynomial port of optimized-routines' AdvSIMD atan2.  Returns the
// pool FPR holding the result (caller stores + frees).
int emit_inline_fpatan(TranslationResult& a1, AssemblerBuffer& buf, int Xbase, int Wd_top,
                       int Wd_tmp);

// fptan: replace ST(0) with tan(ST(0)) (caller pushes 1.0 afterwards).
// Order-7 Estrin in r² + double-angle recombination.  Returns the pool
// FPR holding the result (caller stores + frees).
int emit_inline_fptan(TranslationResult& a1, AssemblerBuffer& buf, int Xbase, int Wd_top,
                      int Wd_tmp);

// fprem: ST(0) := fmod(ST(0), ST(1)).  Single-shot via FDIV + FRINTZ
// + FMSUB (matches the simplification the prior IPC sidecar used —
// stock x87 is iterative).  Returns the pool FPR holding the result
// (caller stores + frees).
int emit_inline_fprem(TranslationResult& a1, AssemblerBuffer& buf, int Xbase, int Wd_top,
                      int Wd_tmp);

// fprem1: ST(0) := IEEE-remainder(ST(0), ST(1)).  Single-shot via
// FDIV + FRINTN + FMSUB.  Same simplification as fprem; FRINTN uses
// ties-to-even rounding to match std::remainder.  Returns the pool FPR
// holding the result (caller stores + frees).
int emit_inline_fprem1(TranslationResult& a1, AssemblerBuffer& buf, int Xbase, int Wd_top,
                       int Wd_tmp);

}  // namespace TranslatorX87
