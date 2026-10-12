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

// The same conversions without touching NZCV, in four GPRs.  Control flow is
// CBZ/CBNZ/TBZ/TBNZ and B, all forward.
//
// emit_f80_bits_to_f64: Xm holds the 64-bit significand and Wx the sign and
// exponent halfword (bits 31:16 zero, as LDRH leaves them).  Xm receives the
// binary64 bits emit_f80_to_f64_convert would produce.  Wx, Xt1 and Xt2 are
// clobbered.
void emit_f80_bits_to_f64(AssemblerBuffer& buf, int Xm, int Wx, int Xt1, int Xt2);

// emit_f64_bits_to_f80: writes the f80 for the binary64 bits in Xb to
// [Xaddr, #0..#9], the bytes emit_f64_to_f80 writes.  Xb, We and Wt are
// clobbered; Xaddr is not.
void emit_f64_bits_to_f80(AssemblerBuffer& buf, int Xaddr, int Xb, int We, int Wt);

// Copies the ten bytes at [Xsrc, #src_off] to [Xdst, #dst_off] with two
// overlapping 8-byte LDUR/STUR pairs (bytes 0-7 and 2-9).  Both loads come
// before the stores.  Offsets must keep src_off + 2 and dst_off + 2 within
// the signed 9-bit range.  Xt1 and Xt2 are clobbered.
void emit_copy_f80(AssemblerBuffer& buf, int Xdst, int dst_off, int Xsrc, int src_off, int Xt1,
                   int Xt2);

// Bracket a complete translate_insn reply. Entering converts native packed
// f80 to private compact f64; leaving converts it back. Preserves NZCV and
// allocator masks. Must run outside an active x87 register-cache run.
void emit_native_state_boundary(TranslationResult& tr, bool entering);

// The boundary for X87_NATIVE_SLOTS: converts each valid slot in place,
// keeping the binary64 value in the slot's first 8 bytes inside the reply.
// Preserves NZCV and allocator masks.
void emit_native_slots_boundary(TranslationResult& tr, bool entering);

// Where Rosetta keeps the guest's MXCSR.FTZ/DAZ on this machine, which is
// what X87_IGNORE_FTZ_DAZ clears and restores.  Chosen the way the runtime
// chooses it, from the commpage CPU capabilities.
enum class FlushControl {
    kFpcr,   // FEAT_AFP: FPCR.FZ (FTZ) and FPCR.FIZ (DAZ)
    kAfpcr,  // no FEAT_AFP: the Apple register S3_6_C15_C2_5, bit 1 FTZ, bit 0 DAZ
    kNone,   // neither capability; the knob emits nothing
};
FlushControl flush_control();
const char* flush_control_name(FlushControl fc);

// -----------------------------------------------------------------------------
// Native transport (X87Cache::is_native_transport_run).  These work on
// Rosetta's packed f80 slots directly and leave NZCV untouched: control flow
// uses CBZ/CBNZ/TBZ/TBNZ only, every branch goes forward, and constants are
// built in GPRs.
// -----------------------------------------------------------------------------

// Xaddr = address of physical slot Wphys (0..7, upper bits zero), the
// 10-byte f80 at Xbase + 6 + 10 * phys.
void emit_native_slot_addr(AssemblerBuffer& buf, int Xaddr, int Xbase, int Wphys);

// Writes the int64 in Xint to the f80 slot at Xslot, exactly.  Xint, Xs and
// Xe are clobbered.
void emit_i64_to_f80_slot(AssemblerBuffer& buf, int Xslot, int Xint, int Xs, int Xe);

// How emit_f80_slot_to_int rounds a value that is not an integer.
enum class NativeIntRound {
    kControlWord,  // RC from the control word at [Xbase]
    kNearest,      // round to nearest even (X87_FAST_ROUND)
    kChop,         // toward zero (fisttp)
};

// Converts the f80 slot at Xslot to an integer of width `bits` (16, 32 or
// 64) in Xint, as fist/fistp/fisttp store it: rounded per `round`, and the
// width's integer indefinite (INT_MIN) for NaN, infinity, an unnormal, or a
// rounded value out of range.  Xslot, Xm, Wx and We are clobbered; Xbase
// is read only for kControlWord.
void emit_f80_slot_to_int(AssemblerBuffer& buf, int Xbase, int Xslot, int Xint, int Xm, int Wx,
                          int We, NativeIntRound round, int bits);

}  // namespace TranslatorX87
