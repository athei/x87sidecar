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
    // libRosettaRuntime trailing pad.  4-mov chain (MOVZ + 3×MOVK)
    // covers the full 64-bit range; on macOS userland the top 16 bits
    // are zero so the final MOVK is usually a no-op-ish movk #0,
    // but we emit it unconditionally for safety / uniform code size.
    const uint64_t addr = rosetta_core::get_transcendental_helper_addr();
    assert(addr != 0 && "transcendental helper not installed; loader should have set address");
    emit_movn(buf, /*is_64=*/1, /*opc=MOVZ*/2, /*hw=*/0,
              static_cast<uint16_t>(addr & 0xFFFF), /*Rd=*/16);
    emit_movn(buf, /*is_64=*/1, /*opc=MOVK*/3, /*hw=*/1,
              static_cast<uint16_t>((addr >> 16) & 0xFFFF), /*Rd=*/16);
    emit_movn(buf, /*is_64=*/1, /*opc=MOVK*/3, /*hw=*/2,
              static_cast<uint16_t>((addr >> 32) & 0xFFFF), /*Rd=*/16);
    emit_movn(buf, /*is_64=*/1, /*opc=MOVK*/3, /*hw=*/3,
              static_cast<uint16_t>((addr >> 48) & 0xFFFF), /*Rd=*/16);

    // BLR x16 — caller-saved regs (x0..x18, x30, d0..d7, d16..d31)
    // are clobbered.  Cache GPRs (Xbase=x22, Wd_top=x23, Wd_tmp=x24,
    // Xst_base=x28 if cached) are callee-saved by AAPCS64 and survive.
    buf.emit(encode_blr(/*Rn=*/16));

    // After return: d0 = out0 (first result), d1 = out1 (only
    // meaningful for fsincos).  Caller pulls them via emit_store_st(0)
    // and/or x87_push as appropriate for its op.
}

}  // namespace TranslatorX87
