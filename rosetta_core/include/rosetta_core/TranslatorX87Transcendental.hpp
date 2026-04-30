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

}  // namespace TranslatorX87
