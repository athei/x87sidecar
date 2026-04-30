#pragma once

class AssemblerBuffer;

namespace TranslatorX87 {

// -----------------------------------------------------------------------------
// f80 (x87 80-bit extended) <-> f64 (IEEE 754 double) conversion emitters.
//
// These pure-emit helpers contain the bit-math previously inlined in
// translate_fld (m80fp branch) and translate_fst/fstp (m80fp branch). They
// allocate nothing — all scratch is caller-provided and the registers may
// be reused across multiple invocations in one block (no labels, branches
// are PC-relative). This makes them safe to replicate in the unrolled
// 8-slot loops in fsave/frstor.
//
// Lossy paths (inherited from the original inline code, documented for
// transparency):
//   - emit_f64_to_f80 flushes f64 denormals to ±0 in f80 form.
//   - emit_f80_to_f64 flushes f80 zero/denormal to ±0 (no f64 denormal
//     synthesis).
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
// Branchless (CSEL chain).  All scratch is caller-allocated.
//
// Wd_aux plays two roles sequentially: first it captures the rounding-carry
// flag, then (after the carry is consumed) it holds the 0x7FFF/0x7FF
// constants for the inf/nan exponent override.  Caller passes one register
// for both roles since their lifetimes don't overlap.
void emit_f80_to_f64_convert(AssemblerBuffer& buf,
                             int Xmant_inout,
                             int Wexp,
                             int Xsign,
                             int Wd_aux,
                             int Wd_tmp);

// Writes 10 bytes of x87 f80 at [Xaddr_slot, #0..#9] from the IEEE 754 double
// in Dd_src.  Uses 3 forward branches (CBZ + B.EQ + 2x B) with PC-relative
// offsets — replicate-safe.  All scratch is caller-allocated.
void emit_f64_to_f80(AssemblerBuffer& buf,
                     int Xaddr_slot,
                     int Dd_src,
                     int Xbits,
                     int Wexp,
                     int Wd_tmp);

};  // namespace TranslatorX87
