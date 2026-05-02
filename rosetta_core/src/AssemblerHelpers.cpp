#include "rosetta_core/AssemblerHelpers.hpp"

#include <cassert>
#include <cstdint>

#include "rosetta_core/AssemblerBuffer.h"
#include "rosetta_core/IROperand.h"
#include "rosetta_core/ProfileRuntime.h"
#include "rosetta_core/Register.h"
#include "rosetta_core/TranslationResult.h"
#include "rosetta_core/TranslatorHelpers.hpp"

// AArch64 logical immediates encode a value as a replicated bitmask:
//   - Pick an element size S ∈ {2,4,8,16,32,64} bits
//   - Within each element, place a contiguous run of 1s (len bits), right-rotated by R
//   - Replicate that element to fill the register width
//
// The encoding is (N, immr, imms):
//   N    : 1 if element size is 64, else 0
//   immr : rotation amount R (right-rotate, mod element_size)
//   imms : encodes element_size and run_length as ~(element_size - 1) | (run_length - 1)
//          i.e. the top bits identify the element size, low bits the run length
//
// Returns false if value cannot be represented as a logical immediate.
// Reimplemented from the original Rosetta binary at 0x1f18
// Faithfully matches the disassembly logic.
bool is_bitmask_immediate(bool is_64bit, uint64_t value, LogicalImmEncoding& out) {
    unsigned int element_size;

    if (is_64bit) {
        // value + 1 < 2  means value == 0 or value == 0xFFFFFFFFFFFFFFFF
        if (value + 1 < 2) {
            return false;
        }
        element_size = 64;
    } else {
        // 32-bit: truncate, then same check
        if (static_cast<uint32_t>(value + 1) < 2) {
            return false;
        }
        value = static_cast<uint32_t>(value);
        element_size = 32;
    }

    // Step 1: find smallest repeating element size (halve while pattern repeats)
    do {
        unsigned int half = element_size >> 1;
        element_size &= ~1U;  // clear low bit (no-op for powers of 2 >= 4)

        uint64_t mask = ~(0xFFFFFFFFFFFFFFFFULL << half);
        if (((value >> half) ^ value) & mask) {
            // Upper and lower halves differ — this is our element size
            break;
        }
        element_size = half;
    } while (element_size > 2);

    // Step 2: isolate one element.
    // Original code: `0xFF..FFULL >> (-(char)element_size)`, which relied on
    // mod-64 of a negative shift count.  That's UB and LLVM optimised it
    // into a fault when this function got inlined into a separate-function
    // helper (see `feedback_is_bitmask_immediate_ub.md`).  The well-defined
    // form: `>> (64 - element_size)`.  element_size ∈ {2,4,8,16,32,64} so
    // (64 - element_size) ∈ {62,60,56,48,32,0} — always a valid shift.
    uint64_t element_mask = 0xFFFFFFFFFFFFFFFFULL >> (64 - element_size);
    uint64_t element = element_mask & value;

    uint8_t rotation;
    uint8_t run_len;

    if (element != 0) {
        // Check if element is a contiguous run of 1s:
        //   (filled + 1) & filled == 0   where filled = (element - 1) | element
        uint64_t filled = (element - 1) | element;
        if (((filled + 1) & filled) == 0) {
            // Yes, contiguous 1s — count trailing zeros for rotation, then run length
            rotation = static_cast<uint8_t>(__builtin_ctzll(element));  // RBIT+CLZ
            run_len =
                static_cast<uint8_t>(__builtin_ctzll(~(element >> rotation)));  // length of 1-run
            goto encode;
        }
    }

    // Not a contiguous run of 1s — try the complement (run of 0s)
    {
        uint64_t zeros = element_mask & ~value;
        if (zeros == 0) {
            return false;
        }

        // Check if zeros is a contiguous run of 1s
        uint64_t filled_z = (zeros - 1) | zeros;
        if (((filled_z + 1) & filled_z) != 0) {
            return false;
        }

        // zeros is contiguous — decode rotation and run length
        // Uses CLZ (not CTZ) — different from the 1s path
        int lz = __builtin_clzll(zeros);                          // leading zeros
        rotation = static_cast<uint8_t>(64 - lz);                 // position of highest set bit + 1
        int tz = __builtin_clzll(__builtin_bitreverse64(zeros));  // trailing zeros via RBIT+CLZ
        run_len = static_cast<uint8_t>(lz + tz - (64 - element_size));
    }

encode:
    // Assertion: rotation must fit within element
    assert(element_size >= rotation && "i should not exceed the element size");

    // Step 3: encode (N, immr, imms)
    auto imms_raw = static_cast<uint8_t>((run_len - 1) | static_cast<uint8_t>(0xFE * element_size));
    out.N = (imms_raw & 0x40) == 0;
    out.immr = static_cast<uint8_t>((element_size - rotation) & (element_size - 1));
    out.imms = imms_raw & 0x3F;
    return true;
}

// ---------------------------------------------------------------------------
// 1a — GPR Data Processing
// ---------------------------------------------------------------------------

auto emit_add_imm(AssemblerBuffer& buf, int is_64bit, int is_sub, int is_set_flags, int shift,
                  int64_t imm12, int64_t Rn, int Rd) -> void {
    // ADD/SUB (immediate): sf | op | S | 10001 | shift | imm12 | Rn | Rd
    // [31]=sf [30]=op(sub) [29]=S(setflags) [28:24]=10001 [23:22]=shift
    // [21:10]=imm12 [9:5]=Rn [4:0]=Rd
    uint32_t insn = 0x11000000;
    insn |= static_cast<uint32_t>(is_64bit != 0) << 31;
    insn |= static_cast<uint32_t>(is_sub != 0) << 30;
    insn |= static_cast<uint32_t>(is_set_flags != 0) << 29;
    insn |= static_cast<uint32_t>(shift & 0x3) << 22;
    insn |= static_cast<uint32_t>(imm12 & 0xFFF) << 10;
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Rd & 0x1F);
    buf.emit(insn);
}

// auto emit_and_imm(AssemblerBuffer& buf, int is_64bit, int Rd, int N,
//                   int64_t immr, int64_t imms, int Rn) -> void {
//     // AND (immediate): sf | 00 | 100100 | N | immr | imms | Rn | Rd
//     // [31]=sf [30:29]=00 [28:23]=100100 [22]=N [21:16]=immr [15:10]=imms
//     // [9:5]=Rn [4:0]=Rd
//     uint32_t insn = 0x12000000;
//     insn |= (uint32_t)(is_64bit != 0) << 31;
//     insn |= (uint32_t)(N & 0x1)       << 22;
//     insn |= (uint32_t)(immr & 0x3F)   << 16;
//     insn |= (uint32_t)(imms & 0x3F)   << 10;
//     insn |= (uint32_t)(Rn & 0x1F)     << 5;
//     insn |= (uint32_t)(Rd & 0x1F);
//     buf.emit(insn);
// }

auto emit_bitfield(AssemblerBuffer& buf, int is_64bit, int opc, int N, int8_t immr, int8_t imms,
                   int Rn, int Rd) -> void {
    // BFM/UBFM/SBFM: sf | opc | 100110 | N | immr | imms | Rn | Rd
    // [31]=sf [30:29]=opc [28:23]=100110 [22]=N [21:16]=immr [15:10]=imms
    // [9:5]=Rn [4:0]=Rd
    uint32_t insn = 0x13000000;
    insn |= static_cast<uint32_t>(is_64bit != 0) << 31;
    insn |= static_cast<uint32_t>(opc & 0x3) << 29;
    insn |= static_cast<uint32_t>(N & 0x1) << 22;
    insn |= static_cast<uint32_t>(immr & 0x3F) << 16;
    insn |= static_cast<uint32_t>(imms & 0x3F) << 10;
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Rd & 0x1F);
    buf.emit(insn);
}

auto emit_movn(AssemblerBuffer& buf, int is_64bit, int opc, int hw, uint16_t imm16, int Rd)
    -> void {
    // MOVN/MOVZ/MOVK: sf | opc | 100101 | hw | imm16 | Rd
    // [31]=sf [30:29]=opc [28:23]=100101 [22:21]=hw [20:5]=imm16 [4:0]=Rd
    uint32_t insn = 0x12800000;
    insn |= static_cast<uint32_t>(is_64bit != 0) << 31;
    insn |= static_cast<uint32_t>(opc & 0x3) << 29;
    insn |= static_cast<uint32_t>(hw & 0x3) << 21;
    insn |= static_cast<uint32_t>(imm16) << 5;
    insn |= static_cast<uint32_t>(Rd & 0x1F);
    buf.emit(insn);
}

auto emit_movz_movk_abs64(AssemblerBuffer& buf, int Rd, uint64_t addr) -> void {
    // MOVZ Rd, #lo16; MOVK Rd, #mid16, lsl 16; MOVK Rd, #hi16, lsl 32;
    // MOVK Rd, #top16, lsl 48 — 4 instructions, materialises the full
    // 64-bit absolute address.  All four are emitted unconditionally so
    // the code size is uniform regardless of address (the top MOVK is
    // a no-op-ish movk #0 on macOS userland, but emitting it keeps every
    // call site the same number of bytes).
    emit_movn(buf, /*is_64bit=*/1, /*opc=MOVZ*/ 2, /*hw=*/0, static_cast<uint16_t>(addr & 0xFFFF),
              Rd);
    emit_movn(buf, /*is_64bit=*/1, /*opc=MOVK*/ 3, /*hw=*/1,
              static_cast<uint16_t>((addr >> 16) & 0xFFFF), Rd);
    emit_movn(buf, /*is_64bit=*/1, /*opc=MOVK*/ 3, /*hw=*/2,
              static_cast<uint16_t>((addr >> 32) & 0xFFFF), Rd);
    emit_movn(buf, /*is_64bit=*/1, /*opc=MOVK*/ 3, /*hw=*/3,
              static_cast<uint16_t>((addr >> 48) & 0xFFFF), Rd);
}

auto emit_ldaddal_x(AssemblerBuffer& buf, int Xs, int Xt, int Xn) -> void {
    // 64-bit LDADDAL encoding:
    //   size=11 op=111000 A=1 R=1 1 Rs:5 0 000 00 Rn:5 Rt:5
    //   = 0xF8E00000 | (Xs<<16) | (Xn<<5) | Xt
    uint32_t insn = 0xF8E00000U;
    insn |= static_cast<uint32_t>(Xs & 0x1F) << 16;
    insn |= static_cast<uint32_t>(Xn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Xt & 0x1F);
    buf.emit(insn);
}

auto emit_block_counter_bump(TranslationResult& tr, uint32_t bid) -> void {
    // 6 ARM64 instructions = 24 bytes, fired once per block entry.
    //   MOVZ/MOVK *4   Xtmp = counter_addr + bid*8
    //   MOVZ Xv, #1
    //   LDADDAL Xv, XZR, [Xtmp]
    AssemblerBuffer& buf = tr.insn_buf;
    const int Xtmp = alloc_free_gpr(tr);
    const int Xv = alloc_free_gpr(tr);
    const uint64_t addr = profile::counter_array_addr() + (static_cast<uint64_t>(bid) * 8U);
    emit_movz_movk_abs64(buf, Xtmp, addr);
    emit_movn(buf, /*is_64bit=*/1, /*opc=MOVZ*/ 2, /*hw=*/0, /*imm16=*/1, Xv);
    emit_ldaddal_x(buf, /*Xs=*/Xv, /*Xt=*/GPR::XZR, /*Xn=*/Xtmp);
    free_gpr(tr, Xv);
    free_gpr(tr, Xtmp);
}

auto emit_mov_reg(AssemblerBuffer& buf, int is_64bit, int Rd, int Rn) -> void {
    // SP case: MOV Xd, SP  →  ADD Xd, SP, #0
    if (Rd == GPR::SP || Rn == GPR::SP) {
        emit_add_imm(buf, is_64bit, 0, 0, 0, 0, Rn, Rd);
    } else {
        // ORR Xd, XZR, Xn  (opc=1, n=0, shift=0, shift_amount=0, Rn=XZR=0x1F)
        emit_logical_shifted_reg(buf, is_64bit, 1, 0, 0, Rn, 0, GPR::XZR, Rd);
    }
}

auto emit_lslv(AssemblerBuffer& buf, int is_64bit, int Rm, int Rn, int Rd) -> void {
    // LSLV: sf | 0 | 0 | 11010110 | Rm | 001000 | Rn | Rd
    uint32_t insn = 0x1AC02000;
    insn |= static_cast<uint32_t>(is_64bit != 0) << 31;
    insn |= static_cast<uint32_t>(Rm & 0x1F) << 16;
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Rd & 0x1F);
    buf.emit(insn);
}

auto emit_subs_reg(AssemblerBuffer& buf, int is_64bit, int Rn, int Rm, int Rd) -> void {
    emit_add_sub_shifted_reg(buf, is_64bit, /*is_sub=*/1, /*is_set_flags=*/1,
                             /*shift_type=*/0, Rm, /*shift_amount=*/0, Rn, Rd);
}

auto emit_add_sub_shifted_reg(AssemblerBuffer& buf, int is_64bit, int is_sub, int is_set_flags,
                              int shift_type, int Rm, int8_t shift_amount, int Rn, int Rd) -> void {
    // sf | op | S | 01011 | shift | 0 | Rm | imm6 | Rn | Rd
    // [31]=sf [30]=op [29]=S [28:24]=01011 [23:22]=shift [21]=0
    // [20:16]=Rm [15:10]=imm6 [9:5]=Rn [4:0]=Rd
    uint32_t insn = 0x0B000000;
    insn |= static_cast<uint32_t>(is_64bit != 0) << 31;
    insn |= static_cast<uint32_t>(is_sub != 0) << 30;
    insn |= static_cast<uint32_t>(is_set_flags != 0) << 29;
    insn |= static_cast<uint32_t>(shift_type & 0x3) << 22;
    insn |= static_cast<uint32_t>(Rm & 0x1F) << 16;
    insn |= static_cast<uint32_t>(shift_amount & 0x3F) << 10;
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Rd & 0x1F);
    buf.emit(insn);
}

auto emit_logical_shifted_reg(AssemblerBuffer& buf, int is_64bit, int opc, int n, int shift_type,
                              int Rm, int8_t shift_amount, int Rn, int Rd) -> void {
    // sf | opc | 01010 | shift | N | Rm | imm6 | Rn | Rd
    // [31]=sf [30:29]=opc [28:24]=01010 [23:22]=shift [21]=N
    // [20:16]=Rm [15:10]=imm6 [9:5]=Rn [4:0]=Rd
    uint32_t insn = 0x0A000000;
    insn |= static_cast<uint32_t>(is_64bit != 0) << 31;
    insn |= static_cast<uint32_t>(opc & 0x3) << 29;
    insn |= static_cast<uint32_t>(shift_type & 0x3) << 22;
    insn |= static_cast<uint32_t>(n & 0x1) << 21;
    insn |= static_cast<uint32_t>(Rm & 0x1F) << 16;
    insn |= static_cast<uint32_t>(shift_amount & 0x3F) << 10;
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Rd & 0x1F);
    buf.emit(insn);
}

// ---------------------------------------------------------------------------
// 1b — Load / Store (GPR + unified)
// ---------------------------------------------------------------------------

auto emit_ldr_str_imm(AssemblerBuffer& buf, int size, int is_fp, int opc, int16_t imm12, int Rn,
                      int Rt) -> void {
    // size | 1 | 1 | is_fp | 0 | opc' | imm12 | Rn | Rt
    // [31:30]=size [29:27]=1 1 1 (V=is_fp at [26]) [25:24]=00
    // [23:22]=opc [21:10]=imm12 [9:5]=Rn [4:0]=Rt
    int effective_opc = (size == 4) ? (opc | 2) : opc;  // 128-bit always sets opc[1]
    uint32_t insn = 0x39000000;
    insn |= static_cast<uint32_t>(size & 0x3) << 30;
    insn |= static_cast<uint32_t>(is_fp & 0x1) << 26;
    insn |= static_cast<uint32_t>(effective_opc & 0x3) << 22;
    insn |= static_cast<uint32_t>(static_cast<uint16_t>(imm12) & 0xFFF) << 10;
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Rt & 0x1F);
    buf.emit(insn);
}

auto emit_ldr_str_reg(AssemblerBuffer& buf, int size, int is_fp, int opc, int Rm, int shift, int Rn,
                      int Rt) -> void {
    // size | 111 | V | 00 | opc | 1 | Rm | option | S | 10 | Rn | Rt
    // [31:30]=size [29:27]=111 [26]=V [25:24]=00 [23:22]=opc [21]=1
    // [20:16]=Rm [15:13]=option(LSL=011) [12]=S(shift) [11:10]=10
    // [9:5]=Rn [4:0]=Rt
    int effective_opc = (size == 4) ? (opc | 2) : opc;
    uint32_t insn = 0x38206800;
    insn |= static_cast<uint32_t>(size & 0x3) << 30;
    insn |= static_cast<uint32_t>(is_fp & 0x1) << 26;
    insn |= static_cast<uint32_t>(effective_opc & 0x3) << 22;
    insn |= static_cast<uint32_t>(Rm & 0x1F) << 16;
    insn |= static_cast<uint32_t>(shift != 0 ? 1 : 0) << 12;
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Rt & 0x1F);
    buf.emit(insn);
}

auto emit_ldr_str_imm_ext(AssemblerBuffer& buf, int data_size, int write_back, int extend_mode,
                          int16_t offset, int Rn, int Rt) -> void {
    // size | 111 | V=0 | 00 | opc | 0 | imm9 | wb | 0 | Rn | Rt
    int effective_opc = (data_size == 4) ? (extend_mode | 2) : extend_mode;
    uint32_t insn = 0x38000000;
    insn |= static_cast<uint32_t>(data_size & 0x3) << 30;
    insn |= static_cast<uint32_t>(effective_opc & 0x3) << 22;
    insn |= static_cast<uint32_t>(offset & 0x1FF) << 12;
    insn |= static_cast<uint32_t>(write_back & 0x1) << 11;  // 1=pre, 0=post at [11:10]=01/11
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Rt & 0x1F);
    buf.emit(insn);
}

auto emit_ldr_imm(AssemblerBuffer& buf, int size, int Rt, int Rn, int16_t imm12) -> void {
    emit_ldr_str_imm(buf, size, /*is_fp=*/0, /*opc=*/1, imm12, Rn, Rt);
}

auto emit_str_imm(AssemblerBuffer& buf, int size, int Rt, int Rn, int16_t imm12) -> void {
    emit_ldr_str_imm(buf, size, /*is_fp=*/0, /*opc=*/0, imm12, Rn, Rt);
}

// ---------------------------------------------------------------------------
// 1c — FPR load/store convenience wrappers
// ---------------------------------------------------------------------------

auto emit_fldr_imm(AssemblerBuffer& buf, int size, int Dt, int Rn, int16_t imm12) -> void {
    emit_ldr_str_imm(buf, size, /*is_fp=*/1, /*opc=*/1, imm12, Rn, Dt);
}

auto emit_fstr_imm(AssemblerBuffer& buf, int size, int Dt, int Rn, int16_t imm12) -> void {
    emit_ldr_str_imm(buf, size, /*is_fp=*/1, /*opc=*/0, imm12, Rn, Dt);
}

auto emit_fldr_reg(AssemblerBuffer& buf, int size, int Dt, int Rn, int Rm, int shift) -> void {
    emit_ldr_str_reg(buf, size, /*is_fp=*/1, /*opc=*/1, Rm, shift, Rn, Dt);
}

auto emit_fstr_reg(AssemblerBuffer& buf, int size, int Dt, int Rn, int Rm, int shift) -> void {
    emit_ldr_str_reg(buf, size, /*is_fp=*/1, /*opc=*/0, Rm, shift, Rn, Dt);
}

// ---------------------------------------------------------------------------
// 1d — FP Arithmetic
// ---------------------------------------------------------------------------

auto emit_fp_dp2(AssemblerBuffer& buf, int type, int opcode, int Rd, int Rn, int Rm) -> void {
    // Scalar FP data-proc 2-source:
    // [31:23]=00011110 | type [22:21] | 1 | Rm [20:16] | opcode [15:12] | 10 | Rn [9:5] | Rd [4:0]
    // Fixed bits [31:24]=0x1E, [21]=1, [11:10]=10
    // type: 00=f32 01=f64
    // opcode[3:0]: 0000=FMUL 0001=FDIV 0010=FADD 0011=FSUB
    uint32_t insn = 0x1E200800;
    insn |= static_cast<uint32_t>(type & 0x3) << 22;
    insn |= static_cast<uint32_t>(Rm & 0x1F) << 16;
    insn |= static_cast<uint32_t>(opcode & 0xF) << 12;
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Rd & 0x1F);
    buf.emit(insn);
}

auto emit_fadd_f64(AssemblerBuffer& buf, int Dd, int Dn, int Dm) -> void {
    emit_fp_dp2(buf, /*type=*/1, /*opcode=*/2 /*FADD*/, Dd, Dn, Dm);
}
auto emit_fsub_f64(AssemblerBuffer& buf, int Dd, int Dn, int Dm) -> void {
    emit_fp_dp2(buf, /*type=*/1, /*opcode=*/3 /*FSUB*/, Dd, Dn, Dm);
}
auto emit_fmul_f64(AssemblerBuffer& buf, int Dd, int Dn, int Dm) -> void {
    emit_fp_dp2(buf, /*type=*/1, /*opcode=*/0 /*FMUL*/, Dd, Dn, Dm);
}
auto emit_fdiv_f64(AssemblerBuffer& buf, int Dd, int Dn, int Dm) -> void {
    emit_fp_dp2(buf, /*type=*/1, /*opcode=*/1 /*FDIV*/, Dd, Dn, Dm);
}

auto emit_fp_dp3(AssemblerBuffer& buf, int type, int o1, int o0, int Rd, int Rn, int Rm, int Ra)
    -> void {
    // Scalar FP data-proc 3-source:
    // [31]=M=0 [30]=0 [29]=S=0 [28:24]=11111 [23:22]=type [21]=o1
    // [20:16]=Rm [15]=o0 [14:10]=Ra [9:5]=Rn [4:0]=Rd
    uint32_t insn = 0x1F000000;
    insn |= static_cast<uint32_t>(type & 0x3) << 22;
    insn |= static_cast<uint32_t>(o1 & 0x1) << 21;
    insn |= static_cast<uint32_t>(Rm & 0x1F) << 16;
    insn |= static_cast<uint32_t>(o0 & 0x1) << 15;
    insn |= static_cast<uint32_t>(Ra & 0x1F) << 10;
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Rd & 0x1F);
    buf.emit(insn);
}

auto emit_fmadd_f64(AssemblerBuffer& buf, int Dd, int Dn, int Dm, int Da) -> void {
    emit_fp_dp3(buf, /*type=*/1, /*o1=*/0, /*o0=*/0, Dd, Dn, Dm, Da);
}
auto emit_fmsub_f64(AssemblerBuffer& buf, int Dd, int Dn, int Dm, int Da) -> void {
    emit_fp_dp3(buf, /*type=*/1, /*o1=*/0, /*o0=*/1, Dd, Dn, Dm, Da);
}
auto emit_fnmsub_f64(AssemblerBuffer& buf, int Dd, int Dn, int Dm, int Da) -> void {
    emit_fp_dp3(buf, /*type=*/1, /*o1=*/1, /*o0=*/1, Dd, Dn, Dm, Da);
}

auto emit_fp_dp1(AssemblerBuffer& buf, int type, int opcode, int Rd, int Rn) -> void {
    // Scalar FP data-proc 1-source:
    // [31:24]=0x1E [23:22]=type [21]=1 [20:15]=opcode [14:10]=10000 [9:5]=Rn [4:0]=Rd
    // opcode[5:0]: 000000=FMOV 000001=FABS 000010=FNEG 000011=FSQRT
    //              000100=FCVT(other type — use emit_fcvt instead)
    uint32_t insn = 0x1E204000;
    insn |= static_cast<uint32_t>(type & 0x3) << 22;
    insn |= static_cast<uint32_t>(opcode & 0x3F) << 15;
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Rd & 0x1F);
    buf.emit(insn);
}

auto emit_fmov_f64(AssemblerBuffer& buf, int Dd, int Dn) -> void {
    emit_fp_dp1(buf, /*type=*/1, /*opcode=*/0 /*FMOV*/, Dd, Dn);
}
auto emit_fabs_f64(AssemblerBuffer& buf, int Dd, int Dn) -> void {
    emit_fp_dp1(buf, /*type=*/1, /*opcode=*/1 /*FABS*/, Dd, Dn);
}
auto emit_fneg_f64(AssemblerBuffer& buf, int Dd, int Dn) -> void {
    emit_fp_dp1(buf, /*type=*/1, /*opcode=*/2 /*FNEG*/, Dd, Dn);
}
auto emit_fsqrt_f64(AssemblerBuffer& buf, int Dd, int Dn) -> void {
    emit_fp_dp1(buf, /*type=*/1, /*opcode=*/3 /*FSQRT*/, Dd, Dn);
}
auto emit_frinta_f64(AssemblerBuffer& buf, int Dd, int Dn) -> void {
    // FRINTA: round to integral, ties-away-from-zero.  DP1 opcode 0b001100.
    emit_fp_dp1(buf, /*type=*/1, /*opcode=*/0xC /*FRINTA*/, Dd, Dn);
}
auto emit_frintz_f64(AssemblerBuffer& buf, int Dd, int Dn) -> void {
    // FRINTZ: round to integral, toward zero (truncate).  DP1 opcode 0b001011.
    emit_fp_dp1(buf, /*type=*/1, /*opcode=*/0xB /*FRINTZ*/, Dd, Dn);
}
auto emit_frintn_f64(AssemblerBuffer& buf, int Dd, int Dn) -> void {
    // FRINTN: round to integral, ties-to-even (IEEE default).  DP1 opcode 0b001000.
    emit_fp_dp1(buf, /*type=*/1, /*opcode=*/0x8 /*FRINTN*/, Dd, Dn);
}

auto emit_fp_cmp(AssemblerBuffer& buf, int type, int Rn, int Rm) -> void {
    // FCMP: [31:24]=0x1E [23:22]=type [21]=1 [20:16]=Rm [15:14]=00
    // [13:10]=1000 [9:5]=Rn [4:3]=00 [2:0]=000 (compare with register, no trap)
    uint32_t insn = 0x1E202000;
    insn |= static_cast<uint32_t>(type & 0x3) << 22;
    insn |= static_cast<uint32_t>(Rm & 0x1F) << 16;
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    buf.emit(insn);
}

auto emit_fcmp_f64(AssemblerBuffer& buf, int Dn, int Dm) -> void {
    emit_fp_cmp(buf, /*type=*/1, Dn, Dm);
}

// =============================================================================
// CSET Wd, cond — conditional set (1 if cond, else 0)
//
// Encodes as CSINC Rd, XZR, XZR, invert(cond).
// CSINC encoding: sf | 0 | 0 | 11010100 | Rm | cond | 0 | 1 | Rn | Rd
//   sf = is_64bit
//   Rm = XZR (31), Rn = XZR (31)
//   cond = inverted condition (flip bit 0)
// =============================================================================
auto emit_cset(AssemblerBuffer& buf, int is_64bit, int cond, int Rd) -> void {
    const uint32_t inv_cond = cond ^ 1;
    uint32_t insn = 0x1A9F07E0U;  // CSINC Rd, XZR, XZR, AL (base with XZR in Rm and Rn)
    insn |= static_cast<uint32_t>(is_64bit & 1) << 31;
    insn |= (inv_cond & 0xFU) << 12;
    insn |= static_cast<uint32_t>(Rd & 0x1F);
    buf.emit(insn);
}

// =============================================================================
// FCSEL Dd, Dn, Dm, cond — conditional FP select (f64)
//
// Encoding: M=0, S=0, 11110, type=01(f64), 1, Rm, cond, 11, Rn, Rd
// Base: 0x1E600C00
// =============================================================================
auto emit_fcsel_f64(AssemblerBuffer& buf, int Dd, int Dn, int Dm, int cond) -> void {
    uint32_t insn = 0x1E600C00U;
    insn |= static_cast<uint32_t>(Dm & 0x1F) << 16;
    insn |= static_cast<uint32_t>(cond & 0xF) << 12;
    insn |= static_cast<uint32_t>(Dn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Dd & 0x1F);
    buf.emit(insn);
}

// =============================================================================
// FCMP Dn, #0.0 — compare FP register against zero
//
// Encoding: same as FCMP but bit[3]=1, Rm field=00000
// 0x1E602008 | (Rn << 5)  for f64
// =============================================================================
auto emit_fcmp_zero_f64(AssemblerBuffer& buf, int Dn) -> void {
    uint32_t insn = 0x1E602008U;
    insn |= static_cast<uint32_t>(Dn & 0x1F) << 5;
    buf.emit(insn);
}

auto emit_fcvt(AssemblerBuffer& buf, int dst_type, int src_type, int Rd, int Rn) -> void {
    // FCVT: same encoding as emit_fp_dp1 but opcode encodes destination type
    // [31:24]=0x1E [23:22]=src_type [21]=1 [20:15]=0001|dst_type<<2 [14:10]=10000
    // dst_type in bits [4:3] of opcode field: 00=f32 01=f64
    // opcode = 0b000100 | (dst_type << 2) ... actually:
    // opcode[5:0] for FCVT to double: 000101 (0x05), to single: 000100 (0x04)
    int opcode = 4 | (dst_type & 0x3);  // 0x04=→f32, 0x05=→f64
    emit_fp_dp1(buf, src_type, opcode, Rd, Rn);
}

auto emit_fcvt_s_to_d(AssemblerBuffer& buf, int Dd, int Sn) -> void {
    emit_fcvt(buf, /*dst_type=*/1, /*src_type=*/0, Dd, Sn);
}
auto emit_fcvt_d_to_s(AssemblerBuffer& buf, int Sd, int Dn) -> void {
    emit_fcvt(buf, /*dst_type=*/0, /*src_type=*/1, Sd, Dn);
}

auto emit_fmov_gpr_fpr(AssemblerBuffer& buf, int dir, int gpr, int fpr) -> void {
    // FMOV (GPR/FPR) scalar 64-bit:
    // GPR→FPR (Dd=Xn): 0x9E670000 | (Rn<<5) | Rd
    // FPR→GPR (Xd=Dn): 0x9E660000 | (Rn<<5) | Rd
    uint32_t insn;
    if (dir == 0) {
        // Xn → Dd:  type=01, rmode=00, opcode=111  [moves int to float]
        insn = 0x9E670000;
        insn |= static_cast<uint32_t>(gpr & 0x1F) << 5;
        insn |= static_cast<uint32_t>(fpr & 0x1F);
    } else {
        // Dn → Xd:  type=01, rmode=00, opcode=110  [moves float to int]
        insn = 0x9E660000;
        insn |= static_cast<uint32_t>(fpr & 0x1F) << 5;
        insn |= static_cast<uint32_t>(gpr & 0x1F);
    }
    buf.emit(insn);
}

auto emit_fmov_x_to_d(AssemblerBuffer& buf, int Dd, int Xn) -> void {
    emit_fmov_gpr_fpr(buf, /*dir=*/0, Xn, Dd);
}
auto emit_fmov_d_to_x(AssemblerBuffer& buf, int Xd, int Dn) -> void {
    emit_fmov_gpr_fpr(buf, /*dir=*/1, Xd, Dn);
}

// ---------------------------------------------------------------------------
// OPT-5: FP register zero/one without GPR intermediaries
// ---------------------------------------------------------------------------

auto emit_movi_d_zero(AssemblerBuffer& buf, int Dd) -> void {
    // MOVI Dd, #0x0000000000000000
    // Advanced SIMD modified-immediate, 64-bit scalar variant.
    // Encoding: 0 01 0111100000 abc=000 cmode=1110 01 defgh=00000 Rd
    //         = 0x2F00E400 | Rd
    // Zeroes the entire 128-bit V register (bits[127:64] cleared implicitly).
    // No GPR dependency — can issue independently from the ADD Xbase in the
    // same cycle on Apple M-series (4-wide dispatch).
    buf.emit(0x2F00E400U | static_cast<uint32_t>(Dd & 0x1F));
}

auto emit_fmov_d_one(AssemblerBuffer& buf, int Dd) -> void {
    // FMOV Dd, #1.0
    // FP scalar immediate, double-precision.
    // Encoding: 0 00 11110 01 1 imm8 100 00000 Rd
    //   imm8 for +1.0: sign=0 exp=0b111 frac=0b0000 → 0b01110000 = 0x70
    //   base = 0x1E601000
    //   full = 0x1E601000 | (0x70 << 13) | Rd = 0x1E6E1000 | Rd
    // Single instruction, no GPR needed — replaces MOVZ+FMOV (2 insns +
    // cross-domain latency).
    buf.emit(0x1E6E1000U | static_cast<uint32_t>(Dd & 0x1F));
}

// ---------------------------------------------------------------------------
// NEON broadcast helpers — used by X87IRLower's StoreF32-run coalescer to
// collapse N consecutive scalar f32 stores into one DUP + (STR Q | STP S, S)
// groups.  See AssemblerHelpers.hpp section 1e.
// ---------------------------------------------------------------------------

auto emit_dup_v4s_from_s(AssemblerBuffer& buf, int Vd, int Vn) -> void {
    // DUP Vd.4S, Vn.S[0] — broadcast the low 32-bit lane of Vn into all four
    // lanes of Vd (Q-form, 128-bit).
    //
    // Advanced SIMD copy, "DUP (element)" variant:
    //   bit[31] 0
    //   bit[30] Q=1            // 4S form
    //   bit[29] op=0
    //   bits[28:21]=01110000
    //   bits[20:16] imm5=00100  // size=S (bits[19:18]=01) + index=0 (bits[20]=0,
    //                           //   bit[17:16]=lane index lower bits = 0)
    //   bits[15:11]=00000
    //   bit[10] 1
    //   bits[9:5]=Rn
    //   bits[4:0]=Rd
    //
    // Concatenated: 0_1_0_01110000_00100_00000_1_Rn_Rd
    //             = 0x4E040400 | (Rn<<5) | Rd
    uint32_t insn = 0x4E040400U;
    insn |= static_cast<uint32_t>(Vn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Vd & 0x1F);
    buf.emit(insn);
}

auto emit_str_q_imm(AssemblerBuffer& buf, int Qt, int Rn, int16_t imm12) -> void {
    // STR Qt, [Rn, #imm] — store 128-bit Q register, unsigned-immediate form.
    //
    // Load/store register (unsigned immediate), SIMD&FP variant:
    //   bits[31:30] size=00      // for Q-form, opc=10 selects 128-bit
    //   bits[29:27]=111
    //   bit[26]    V=1
    //   bits[25:24]=01
    //   bits[23:22] opc=10       // store, Q-form
    //   bits[21:10] imm12        // byte offset / 16
    //   bits[9:5]   Rn
    //   bits[4:0]   Rt
    //
    // Concatenated: 00_111_1_01_10_imm12_Rn_Rt
    //             = 0x3D800000 | (imm12<<10) | (Rn<<5) | Qt
    //
    // imm12 must fit in 12 unsigned bits; byte offset = imm12 * 16.
    uint32_t insn = 0x3D800000U;
    insn |= (static_cast<uint32_t>(imm12) & 0xFFFU) << 10;
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Qt & 0x1F);
    buf.emit(insn);
}

auto emit_stp_s_imm(AssemblerBuffer& buf, int St1, int St2, int Rn, int16_t simm7) -> void {
    // STP St1, St2, [Rn, #simm] — pair-store two 32-bit S registers, signed
    // offset addressing mode.
    //
    // Load/store pair (signed offset), SIMD&FP variant:
    //   bits[31:30] opc=00       // 32-bit S form
    //   bits[29:27]=101
    //   bit[26]    V=1
    //   bits[25:23]=010          // signed offset
    //   bit[22]    L=0           // store
    //   bits[21:15] imm7         // byte offset / 4 (signed, 7-bit)
    //   bits[14:10] Rt2
    //   bits[9:5]   Rn
    //   bits[4:0]   Rt
    //
    // Concatenated: 00_101_1_010_0_imm7_Rt2_Rn_Rt
    //             = 0x2D000000 | ((imm7&0x7F)<<15) | (Rt2<<10) | (Rn<<5) | Rt
    //
    // simm7 must be in [-64, +63] (encoded), giving byte offsets [-256, +252]
    // step 4.
    uint32_t insn = 0x2D000000U;
    insn |= (static_cast<uint32_t>(simm7) & 0x7FU) << 15;
    insn |= static_cast<uint32_t>(St2 & 0x1F) << 10;
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(St1 & 0x1F);
    buf.emit(insn);
}

auto emit_ldr_literal_f64(AssemblerBuffer& buf, int Dd, uint64_t constant) -> void {
    // OPT-H: LDR Dd, [PC, #8] — load from 2 instructions ahead (the .quad)
    // Encoding (LDR literal, SIMD&FP):
    //   opc=01 (64-bit) | 011 | V=1 | 00 | imm19=2 | Rt
    //   0b01_011_1_00_0000000000000000010_00000 = 0x5C000040
    buf.emit(0x5C000040U | static_cast<uint32_t>(Dd & 0x1F));

    // B #3 — skip over the 8 bytes of constant data, land on next real insn
    emit_b(buf, 3);

    // .quad constant — 8 bytes of raw data in 2 instruction slots
    buf.emit(static_cast<uint32_t>(constant & 0xFFFFFFFFU));
    buf.emit(static_cast<uint32_t>(constant >> 32));
}

auto emit_scvtf(AssemblerBuffer& buf, int is_64bit_int, int ftype, int Rd, int Rn) -> void {
    // SCVTF (scalar, integer): converts GPR to FP
    // [31]=sf [30:24]=0011110 [23:22]=ftype [21]=1 [20:19]=00 [18:16]=010
    // [15:10]=000000 [9:5]=Rn [4:0]=Rd
    uint32_t insn = 0x1E220000;
    insn |= static_cast<uint32_t>(is_64bit_int != 0) << 31;
    insn |= static_cast<uint32_t>(ftype & 0x3) << 22;
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Rd & 0x1F);
    buf.emit(insn);
}

auto emit_scvtf_x_to_d(AssemblerBuffer& buf, int Dd, int Xn) -> void {
    emit_scvtf(buf, /*is_64bit_int=*/1, /*ftype=*/1, Dd, Xn);
}

auto emit_fcvtzs(AssemblerBuffer& buf, int ftype, int is_64bit_int, int Rd, int Rn) -> void {
    // FCVTZS (scalar, integer): FP to signed GPR, truncate toward zero
    // [31]=sf [30:24]=0011110 [23:22]=ftype [21]=1 [20:19]=11 [18:16]=000
    // [15:10]=000000 [9:5]=Rn [4:0]=Rd
    uint32_t insn = 0x1E380000;
    insn |= static_cast<uint32_t>(is_64bit_int != 0) << 31;
    insn |= static_cast<uint32_t>(ftype & 0x3) << 22;
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Rd & 0x1F);
    buf.emit(insn);
}

// ---------------------------------------------------------------------------
// 1e — System / Flags
// ---------------------------------------------------------------------------

auto emit_mrs_nzcv(AssemblerBuffer& buf, int Xd) -> void {
    // MRS Xd, NZCV
    // NZCV sysreg: op0=3, op1=3, CRn=4, CRm=2, op2=0 → encoding 0xD53B4200 | Rt
    // Bit layout: 1101 0101 0011 1011 0100 0010 0000 | Rt
    buf.emit(0xD53B4200U | static_cast<uint32_t>(Xd & 0x1F));
}

auto emit_msr_nzcv(AssemblerBuffer& buf, int Xd) -> void {
    // MSR NZCV, Xd
    // Same sysreg as MRS but with L=0 (write): 0xD51B4200 | Rt
    buf.emit(0xD51B4200U | static_cast<uint32_t>(Xd & 0x1F));
}

auto emit_cbz(AssemblerBuffer& buf, int is_64bit, int is_nz, int Rt, int imm19) -> void {
    // CBZ/CBNZ Rt, #imm19
    // Encoding: sf | 011010 | b5=0 | op | imm19 | Rt
    // 32-bit: 0 011 0100 / 0 011 0101   (0x34 / 0x35)
    // 64-bit: 1 011 0100 / 1 011 0101   (0xB4 / 0xB5)
    uint32_t insn = 0x34000000U;
    insn |= static_cast<uint32_t>(is_64bit != 0) << 31;
    insn |= static_cast<uint32_t>(is_nz != 0) << 24;
    insn |= (static_cast<uint32_t>(imm19) & 0x7FFFFU) << 5;
    insn |= static_cast<uint32_t>(Rt & 0x1F);
    buf.emit(insn);
}

auto emit_b(AssemblerBuffer& buf, int imm26) -> void {
    // B #imm26
    // Encoding: 0 00101 | imm26
    buf.emit(0x14000000U | (static_cast<uint32_t>(imm26) & 0x3FFFFFFU));
}

auto emit_b_cond(AssemblerBuffer& buf, int cond, int imm19) -> void {
    // B.cond #imm19
    // Encoding: 0101 0100 | imm19 | 0 | cond[3:0]
    uint32_t insn = 0x54000000U;
    insn |= (static_cast<uint32_t>(imm19) & 0x7FFFFU) << 5;
    insn |= static_cast<uint32_t>(cond & 0xF);
    buf.emit(insn);
}

auto emit_fmov_f64_reg(AssemblerBuffer& buf, int Dd, int Dn) -> void {
    // FMOV Dd, Dn — double-precision FPR-to-FPR copy
    // Encoding: 0x1E604000 | (Rn<<5) | Rd
    buf.emit(0x1E604000U | (static_cast<uint32_t>(Dn & 0x1F) << 5) |
             static_cast<uint32_t>(Dd & 0x1F));
}

auto emit_fcvt_fp_to_int(AssemblerBuffer& buf, int sf, int ftype, int rmode, int Rd, int Rn)
    -> void {
    // FCVT{N,P,M,Z}S — FP to signed integer with explicit rounding mode
    // Encoding: sf | 0 0 11110 | ftype | 1 | rmode | 000 | Rn | Rd
    uint32_t insn = 0x1E200000U;
    insn |= static_cast<uint32_t>(sf != 0) << 31;
    insn |= static_cast<uint32_t>(ftype & 0x3) << 22;
    insn |= 1U << 21;  // fixed
    insn |= static_cast<uint32_t>(rmode & 0x3) << 19;
    // opcode bits [18:16] = 000 (FCVT*S signed)
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Rd & 0x1F);
    buf.emit(insn);
}

auto emit_add_reg(AssemblerBuffer& buf, int is_64bit, int dst, int lhs, int rhs) -> void {
    if (dst == GPR::SP || lhs == GPR::SP) {
        // Use extended-reg form (UXTX/UXTW)
        int ext = is_64bit ? 3 : 2;
        // emit_add_sub_ext_reg_inner(buf, is_sub=0, set_flags=0, is_64bit, rhs, ext, lhs, dst&0x1F)
        uint32_t v = 0x0B200000;
        v |= static_cast<uint32_t>(is_64bit != 0) << 29;  // sf in bit29 of ext-reg form
        // Full encoding: sf|0|0|01011|00|1|Rm|option|imm3|Rn|Rd
        // option=UXTX(011 for 64-bit) or UXTW(010 for 32-bit), imm3=0
        v = 0x0B200000;
        v |= static_cast<uint32_t>(is_64bit != 0) << 31;
        v |= static_cast<uint32_t>(rhs & 0x1F) << 16;
        v |= static_cast<uint32_t>(ext & 0x7) << 13;
        v |= static_cast<uint32_t>(lhs & 0x1F) << 5;
        v |= static_cast<uint32_t>(dst & 0x1F);
        buf.emit(v);
    } else {
        emit_add_sub_shifted_reg(buf, is_64bit, /*is_sub=*/0, /*is_set_flags=*/0,
                                 /*shift_type=*/0, rhs, /*shift_amount=*/0, lhs, dst);
    }
}

auto emit_logical_imm(AssemblerBuffer& buf, int is_64bit, int opc, int N, int8_t immr, int8_t imms,
                      int Rn, int Rd) -> void {
    assert(Rn != GPR::SP && "emit_logical_imm: SP used in unexpected context");
    assert(Rd != GPR::SP && "emit_logical_imm: SP used in unexpected context");

    uint32_t insn = 0x12000000;
    insn |= static_cast<uint32_t>(is_64bit != 0) << 31;
    insn |= static_cast<uint32_t>(opc & 0x3) << 29;
    insn |= static_cast<uint32_t>(N != 0) << 22;
    insn |= static_cast<uint32_t>(immr & 0x3F) << 16;
    insn |= static_cast<uint32_t>(imms & 0x3F) << 10;
    insn |= static_cast<uint32_t>(Rn & 0x1F) << 5;
    insn |= static_cast<uint32_t>(Rd & 0x1F);
    buf.emit(insn);
}

// ---------------------------------------------------------------------------
// emit_and_imm — @ 0xae0
//
// AND (immediate). Asserts that N==0 when operating in 32-bit mode, since
// the N bit must be 0 for all 32-bit logical immediates per the ARM spec.
//
// Arg order confirmed from disasm: (buf, is_64bit, Rd, N, immr, imms, Rn)
// Call at 0xb08 remaps to emit_logical_imm as: opc=0, N, immr, imms, Rn, Rd
// ---------------------------------------------------------------------------
auto emit_and_imm(AssemblerBuffer& buf, int is_64bit, int Rd, int N, int64_t immr, int64_t imms,
                  int Rn) -> void {
    assert((is_64bit == 1 || N == 0) &&
           "emit_and_imm: data_size == DataSize::S64 || N == 0 — invalid value of N");
    emit_logical_imm(buf, is_64bit, /*opc=*/0, N, static_cast<int8_t>(immr),
                     static_cast<int8_t>(imms), Rn, Rd);
}

// ---------------------------------------------------------------------------
// emit_orr_imm — @ 0x197c
//
// ORR (immediate). Same N==0 constraint for 32-bit.
//
// Arg order from binary: (buf, is_64bit, Rd, Rn, N, immr, imms)
// Delegates to emit_logical_imm with opc=1.
// ---------------------------------------------------------------------------
auto emit_orr_imm(AssemblerBuffer& buf, int is_64bit, int Rd, int Rn, int N, int64_t immr,
                  int64_t imms) -> void {
    assert((is_64bit == 1 || N == 0) &&
           "emit_orr_imm: data_size == DataSize::S64 || N == 0 — invalid value of N");
    emit_logical_imm(buf, is_64bit, /*opc=*/1, N, static_cast<int8_t>(immr),
                     static_cast<int8_t>(imms), Rn, Rd);
}

auto emit_adr(AssemblerBuffer& buf, int is_adrp, int Rd, uint32_t imm) -> void {
    assert(Rd != GPR::XZR && "emit_adr: XZR used in unexpected context");

    // ADR:  0x10000000  (bit28 clear)
    // ADRP: 0x90000000  (bit31 and bit28 set)
    uint32_t insn = is_adrp ? 0x90000000 : 0x10000000;

    // [4:0]   = Rd
    // [30:29] = imm[1:0]   (low 2 bits of the offset)
    // [23:5]  = imm[20:2]  (high 19 bits of the offset, shifted right by 2)
    insn |= static_cast<uint32_t>(Rd & 0x1F);
    insn |= (imm & 0x3) << 29;
    insn |= (imm >> 2 & 0x7FFFF) << 5;

    buf.emit(insn);
}

auto emit_ldrs(AssemblerBuffer& insn_buf, int is_64bit, unsigned int size, int dst_gpr,
               int addr_gpr) -> void {
    // data_size: the width of the destination register interpretation.
    //   is_64bit=true  → S32 (LDRSW sign-extends into 64-bit X register)
    //   is_64bit=false → S16 (LDRSH sign-extends into 32-bit W register)
    const unsigned int data_size = (is_64bit == 1) ? static_cast<unsigned int>(IROperandSize::S32)
                                                   : static_cast<unsigned int>(IROperandSize::S16);

    assert(data_size >= size &&
           "emit_ldrs: unsupported LDRS size — data_size must be >= source size");
    assert(dst_gpr != GPR::SP && "emit_ldrs: SP used in unexpected context");

    // opc field for emit_ldr_str_imm:
    //   is_64bit=true  → opc = S32 (2) — encodes LDRSW
    //   is_64bit=false → opc = S64 (3) — encodes LDRSH/LDRSB into W reg
    const int opc = (is_64bit == 1) ? static_cast<int>(IROperandSize::S32)   // 2
                                    : static_cast<int>(IROperandSize::S64);  // 3

    emit_ldr_str_imm(insn_buf, static_cast<int>(size),
                     /*is_fp=*/0, opc,
                     /*imm12=*/0, addr_gpr, dst_gpr);
}