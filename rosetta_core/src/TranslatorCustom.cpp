#include "rosetta_core/TranslatorCustom.h"

#include "rosetta_core/AssemblerHelpers.hpp"
#include "rosetta_core/IRInstr.h"
#include "rosetta_core/IROperand.h"
#include "rosetta_core/Register.h"
#include "rosetta_core/TranslationResult.h"
#include "rosetta_core/TranslatorHelpers.hpp"

namespace TranslatorCustom {

// =============================================================================
// ARPL r/m16, r16 — Adjust Requested Privilege Level (32-/16-bit legacy only).
//
// Opcode 0x63 is MOVSXD in 64-bit mode, so Rosetta has no ARPL encoding at all.
// The decode hook borrows ADD r/m32,r32's decode (byte-identical ModRM/SIB/disp
// shape, so same operands and same length) and relabels it kOpcodeName_arpl, so
// operand order here matches ADD's:
//   operands[0] = dest (r/m),  operands[1] = src register.
//
// ARPL touches only the low 2 bits (the RPL field) of each 16-bit operand, plus
// ZF:
//   if (dst.RPL < src.RPL) { dst.RPL = src.RPL; ZF = 1; } else { ZF = 0; }
// Every other flag is unaffected, which is why guest NZCV is read-modify-written
// to touch only Z rather than being recomputed. The x86 GPR index maps 1:1 onto
// the AArch64 register (RAX→X0, …), the same assumption translate_fstsw's
// in-place AX write makes.
//
// Seen in the wild in downloaded, obfuscated code: `63 D0` (ARPL AX, DX).
// =============================================================================
auto translate_arpl(TranslationResult* a1, IRInstr* a2) -> void {
    AssemblerBuffer& buf = a1->insn_buf;

    IROperand& dst_op = a2->operands[0];
    const int W_src = a2->operands[1].reg.reg.index();  // src reg (ModRM.reg)

    const int Wsave = alloc_free_gpr(*a1);  // saved guest NZCV
    const int Wa = alloc_free_gpr(*a1);     // dst.RPL
    const int Wb = alloc_free_gpr(*a1);     // src.RPL
    const int Wc = alloc_free_gpr(*a1);     // 1 iff dst.RPL < src.RPL (the new ZF)

    // Wb = src.RPL = W_src[1:0]   (UBFX Wb, W_src, #0, #2)
    emit_bitfield(buf, /*is_64=*/0, /*UBFM=*/2, /*N=*/0, /*immr=*/0, /*imms=*/1, W_src, Wb);

    // The register form is the only encoding seen in practice (ARPL AX, DX =
    // 63 D0); the memory form loads/stores the 16-bit selector around the same
    // core. `dst` names the GPR holding the current selector value.
    const bool dst_is_reg = (dst_op.kind == IROperandKind::Register);
    int addr_reg = GPR::XZR;
    int dst = dst_is_reg ? dst_op.reg.reg.index() : -1;
    if (!dst_is_reg) {
        addr_reg = compute_operand_address(*a1, /*is_64bit=*/true, &dst_op, GPR::XZR);
        dst = alloc_free_gpr(*a1);
        // LDRH dst, [addr_reg]
        emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*opc=*/1, /*imm12=*/0, addr_reg, dst);
    }

    // Save guest flags, isolate dst.RPL, compare the two RPLs.
    emit_mrs_nzcv(buf, Wsave);
    emit_bitfield(buf, 0, /*UBFM=*/2, 0, /*immr=*/0, /*imms=*/1, dst, Wa);   // Wa = dst.RPL
    emit_subs_reg(buf, /*is_64=*/0, /*Rn=*/Wa, /*Rm=*/Wb, /*Rd=*/GPR::XZR);  // cmp
    emit_cset(buf, 0, /*CC/LO=*/3, Wc);  // Wc = (dst.RPL < src.RPL) ? 1 : 0

    // Write ZF only: Wsave[30] = Wc, then restore (N/C/V preserved).
    emit_bitfield(buf, 0, /*BFM=*/1, 0, /*immr=*/2, /*imms=*/0, Wc, Wsave);  // BFI Wsave,Wc,#30,#1
    emit_msr_nzcv(buf, Wsave);

    // When adjusting (Wc == 1), raise dst.RPL to src.RPL; skip otherwise.
    //   CBZ Wc, +2  (skip the single BFI)
    //   BFI dst, Wb, #0, #2
    emit_cbz(buf, /*is_64=*/0, /*is_nz=*/0, Wc, /*imm19=*/2);
    emit_bitfield(buf, 0, /*BFM=*/1, 0, /*immr=*/0, /*imms=*/1, Wb, dst);

    if (!dst_is_reg) {
        // STRH dst, [addr_reg]
        emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*opc=*/0, /*imm12=*/0, addr_reg, dst);
        free_gpr(*a1, dst);
        free_gpr(*a1, addr_reg);
    }

    free_gpr(*a1, Wc);
    free_gpr(*a1, Wb);
    free_gpr(*a1, Wa);
    free_gpr(*a1, Wsave);
}

};  // namespace TranslatorCustom
