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

// Reads 10 bytes of x87 f80 at [Xaddr_slot, #0..#9] and converts to IEEE 754
// double raw bits in Xmant_out.  Caller does FMOV Dd, Xmant_out afterwards.
//
// Branchless (CSEL chain).  All scratch is caller-allocated and must be
// distinct from each other and from Xaddr_slot.  The helper does not free
// or reallocate any register; on exit Xmant_out holds the f64 raw bits and
// the other scratch GPRs are clobbered.
void emit_f80_to_f64(AssemblerBuffer& buf,
                     int Xaddr_slot,
                     int Xmant_out,
                     int Wexp,
                     int Xsign,
                     int Wcarry,
                     int Wexp_max,
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
