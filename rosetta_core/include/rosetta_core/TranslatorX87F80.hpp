#pragma once

struct AssemblerBuffer;
struct TranslationResult;

namespace TranslatorX87 {

// -----------------------------------------------------------------------------
// f80 (x87 80-bit extended) <-> f64 (IEEE 754 double) conversion emitters.
//
// Scratch is caller-provided. Branches are forward and PC-relative, so the
// helpers can be repeated in unrolled loops and stepped by signal recovery.
// Values are narrowed to binary64 with round-to-nearest-even, preserving
// binary64 subnormals when converting back to the native extended format.
// -----------------------------------------------------------------------------

// Convert pre-loaded f80 fields to IEEE 754 double raw bits in Xmant_inout.
// Caller does FMOV Dd, Xmant_inout afterwards.
//
// Pre-conditions (caller emits the two loads):
//   - LDR Xmant_inout, [Xaddr_slot, #0]   ; 8-byte mantissa
//   - LDRH Wexp, [Xaddr_slot, #8]         ; 2-byte sign+exp word
// Then the caller is free to release Xaddr_slot and alloc Xsign / Wd_aux
// before invoking this helper.  This sequencing matches the original
// inline path in translate_fld m80fp and keeps peak alloc_free GPR count
// under the 8-slot scratch pool.
//
// All scratch is caller-allocated; NZCV is clobbered.
//
// Wd_aux plays two roles sequentially: first it captures the rounding-carry
// flag, then (after the carry is consumed) it holds the 0x7FFF/0x7FF
// constants for the inf/nan exponent override.  Caller passes one register
// for both roles since their lifetimes don't overlap.
void emit_f80_to_f64_convert(AssemblerBuffer& buf, int Xmant_inout, int Wexp, int Xsign, int Wd_aux,
                             int Wd_tmp);

// Writes 10 bytes of x87 f80 at [Xaddr_slot, #0..#9] from the IEEE 754 double
// in Dd_src. All scratch is caller-allocated; NZCV is clobbered.
void emit_f64_to_f80(AssemblerBuffer& buf, int Xaddr_slot, int Dd_src, int Xbits, int Wexp,
                     int Wd_tmp);

// Bracket a complete translate_insn reply. Entering converts native packed
// f80 to private compact f64; leaving converts it back. Preserves NZCV and
// allocator masks. Must run outside an active x87 register-cache run.
void emit_native_state_boundary(TranslationResult& tr, bool entering);

}  // namespace TranslatorX87
