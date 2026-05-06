#include "stub_asm.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "rosetta_core/IRInstr.h"
#include "rosetta_core/Opcode.h"
#include "rosetta_core/OpcodeCompatibility.h"

namespace stub_asm {
namespace {

// ──── arm64 instruction encoders ──────────────────────────────────────────────
// Reference: ARM Architecture Reference Manual, A-profile, A64 ISA.
// All instructions are 32-bit, little-endian on Apple platforms.

// MOVZ Xd, #imm16, lsl #(hw*16)
//   sf=1, opc=10, 100101, hw, imm16, Rd
//   Pattern: 1_10_100101_hw_imm16_Rd
constexpr uint32_t movz(uint32_t rd, uint16_t imm, uint32_t lsl_shift) {
    uint32_t hw = lsl_shift / 16;  // 0, 1, 2, 3
    return 0xD2800000U | (hw << 21) | (static_cast<uint32_t>(imm) << 5) | (rd & 0x1F);
}

// MOVK Xd, #imm16, lsl #(hw*16)
//   sf=1, opc=11, 100101, hw, imm16, Rd
constexpr uint32_t movk(uint32_t rd, uint16_t imm, uint32_t lsl_shift) {
    uint32_t hw = lsl_shift / 16;
    return 0xF2800000U | (hw << 21) | (static_cast<uint32_t>(imm) << 5) | (rd & 0x1F);
}

// MOVN Xd, #imm16, lsl #(hw*16)   (move-with-NOT — used for negative immediates)
//   sf=1, opc=00, 100101, hw, imm16, Rd
constexpr uint32_t movn(uint32_t rd, uint16_t imm, uint32_t lsl_shift) {
    uint32_t hw = lsl_shift / 16;
    return 0x92800000U | (hw << 21) | (static_cast<uint32_t>(imm) << 5) | (rd & 0x1F);
}

// BR Xn  (unconditional branch via register)
//   1101_0110_0001_1111_0000_00_Rn_00000
constexpr uint32_t br(uint32_t rn) {
    return 0xD61F0000U | ((rn & 0x1F) << 5);
}

// SVC #imm16  (supervisor call — used with imm16=0x80 for syscalls on Darwin)
//   1101_0100_000_imm16_00001
constexpr uint32_t svc(uint16_t imm16) {
    return 0xD4000001U | (static_cast<uint32_t>(imm16) << 5);
}

// STP Xt1, Xt2, [Xn|SP, #imm7]!  (pre-index, 64-bit pair store)
//   When imm is signed 7-bit scaled by 8.  Encoding for pre-indexed:
//   10_101_0_011_0_imm7_Rt2_Rn_Rt1
constexpr uint32_t stp_preindex(uint32_t rt1, uint32_t rt2, uint32_t rn, int32_t imm) {
    int32_t scaled = imm / 8;
    uint32_t imm7 = static_cast<uint32_t>(scaled) & 0x7F;
    return 0xA9800000U | (imm7 << 15) | ((rt2 & 0x1F) << 10) | ((rn & 0x1F) << 5) | (rt1 & 0x1F);
}

// STP Xt1, Xt2, [Xn|SP, #imm]  (signed-offset, 64-bit pair store)
//   10_101_0_010_0_imm7_Rt2_Rn_Rt1
constexpr uint32_t stp_offset(uint32_t rt1, uint32_t rt2, uint32_t rn, int32_t imm) {
    int32_t scaled = imm / 8;
    uint32_t imm7 = static_cast<uint32_t>(scaled) & 0x7F;
    return 0xA9000000U | (imm7 << 15) | ((rt2 & 0x1F) << 10) | ((rn & 0x1F) << 5) | (rt1 & 0x1F);
}

// LDP Xt1, Xt2, [Xn|SP, #imm]  (signed-offset, 64-bit pair load)
//   10_101_0_010_1_imm7_Rt2_Rn_Rt1
constexpr uint32_t ldp_offset(uint32_t rt1, uint32_t rt2, uint32_t rn, int32_t imm) {
    int32_t scaled = imm / 8;
    uint32_t imm7 = static_cast<uint32_t>(scaled) & 0x7F;
    return 0xA9400000U | (imm7 << 15) | ((rt2 & 0x1F) << 10) | ((rn & 0x1F) << 5) | (rt1 & 0x1F);
}

// LDP Xt1, Xt2, [Xn|SP], #imm  (post-index, 64-bit pair load)
//   10_101_0_001_1_imm7_Rt2_Rn_Rt1
constexpr uint32_t ldp_postindex(uint32_t rt1, uint32_t rt2, uint32_t rn, int32_t imm) {
    int32_t scaled = imm / 8;
    uint32_t imm7 = static_cast<uint32_t>(scaled) & 0x7F;
    return 0xA8C00000U | (imm7 << 15) | ((rt2 & 0x1F) << 10) | ((rn & 0x1F) << 5) | (rt1 & 0x1F);
}

// STR Wt, [Xn|SP, #imm]  (32-bit store, unsigned offset)
//   10_111_0_01_00_imm12_Rn_Rt
constexpr uint32_t str_w_offset(uint32_t wt, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = (imm / 4) & 0xFFF;
    return 0xB9000000U | (imm12 << 10) | ((rn & 0x1F) << 5) | (wt & 0x1F);
}

// STR Xt, [Xn|SP, #imm]  (64-bit store, unsigned offset, scaled by 8)
//   11_111_0_01_00_imm12_Rn_Rt
constexpr uint32_t str_x_offset(uint32_t xt, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = (imm / 8) & 0xFFF;
    return 0xF9000000U | (imm12 << 10) | ((rn & 0x1F) << 5) | (xt & 0x1F);
}

// CBZ Xt, +imm19*4   (branch if zero — 64-bit). imm19 is signed PC-relative
//   in 4-byte units.
constexpr uint32_t cbz(uint32_t rt, int32_t imm19_words) {
    uint32_t imm19 = static_cast<uint32_t>(imm19_words) & 0x7FFFF;
    return 0xB4000000U | (imm19 << 5) | (rt & 0x1F);
}

// CBZ Wt, +imm19*4   (32-bit variant)
constexpr uint32_t cbz_w(uint32_t rt, int32_t imm19_words) {
    uint32_t imm19 = static_cast<uint32_t>(imm19_words) & 0x7FFFF;
    return 0x34000000U | (imm19 << 5) | (rt & 0x1F);
}

// LDR Xt, [Xn|SP, #imm]  (64-bit load, unsigned offset, scaled by 8)
//   11_111_0_01_01_imm12_Rn_Rt
constexpr uint32_t ldr_x_offset(uint32_t xt, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = (imm / 8) & 0xFFF;
    return 0xF9400000U | (imm12 << 10) | ((rn & 0x1F) << 5) | (xt & 0x1F);
}

// LDR Wt, [Xn|SP, #imm]  (32-bit load, unsigned offset, scaled by 4)
//   10_111_0_01_01_imm12_Rn_Rt
constexpr uint32_t ldr_w_offset(uint32_t wt, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = (imm / 4) & 0xFFF;
    return 0xB9400000U | (imm12 << 10) | ((rn & 0x1F) << 5) | (wt & 0x1F);
}

// BRK #imm16
constexpr uint32_t brk_imm(uint16_t imm) {
    return 0xD4200000U | (static_cast<uint32_t>(imm) << 5);
}

// ADD Xd, Xn, #imm12  (signed-offset, no shift, 64-bit add immediate)
//   sf=1, op=0, S=0, 10001, sh=0, imm12, Rn, Rd
constexpr uint32_t add_imm(uint32_t rd, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = imm & 0xFFF;
    return 0x91000000U | (imm12 << 10) | ((rn & 0x1F) << 5) | (rd & 0x1F);
}

// SUB Wd, Wn, #imm12  (32-bit subtract immediate, no flags)
//   sf=0, op=1, S=0, 10001, sh=0, imm12, Rn, Rd
constexpr uint32_t sub_imm_w(uint32_t rd, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = imm & 0xFFF;
    return 0x51000000U | (imm12 << 10) | ((rn & 0x1F) << 5) | (rd & 0x1F);
}

// CMP Wn, #imm12   (= SUBS WZR, Wn, #imm12 — 32-bit compare immediate)
//   sf=0, op=1, S=1, 10001, sh=0, imm12, Rn, 11111
constexpr uint32_t cmp_imm_w(uint32_t rn, uint32_t imm) {
    uint32_t imm12 = imm & 0xFFF;
    return 0x7100001FU | (imm12 << 10) | ((rn & 0x1F) << 5);
}

// CMP Wn, Wm   (= SUBS WZR, Wn, Wm — 32-bit register compare).
//   sf=0, op=1, S=1, 01011, shift=00, 0, Rm, imm6=0, Rn, Rd=11111
constexpr uint32_t cmp_reg_w(uint32_t rn, uint32_t rm) {
    return 0x6B00001FU | ((rm & 0x1F) << 16) | ((rn & 0x1F) << 5);
}

// SUBS Wd, Wn, #imm12  (32-bit subtract immediate, sets NZCV).  Same shape
// as cmp_imm_w but writes Rd instead of WZR.
constexpr uint32_t subs_imm_w(uint32_t rd, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = imm & 0xFFF;
    return 0x71000000U | (imm12 << 10) | ((rn & 0x1F) << 5) | (rd & 0x1F);
}

// ADD Wd, Wn, #imm12  (32-bit add immediate, no flags).
constexpr uint32_t add_imm_w(uint32_t rd, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = imm & 0xFFF;
    return 0x11000000U | (imm12 << 10) | ((rn & 0x1F) << 5) | (rd & 0x1F);
}

// SUB Xd, Xn, #imm12  (64-bit subtract immediate, no flags).
constexpr uint32_t sub_imm_x(uint32_t rd, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = imm & 0xFFF;
    return 0xD1000000U | (imm12 << 10) | ((rn & 0x1F) << 5) | (rd & 0x1F);
}

// LSR Xd, Xn, #shift   (= UBFM Xd, Xn, #shift, #63).  Logical shift right
// by an immediate amount, 64-bit register.
//   sf=1, opc=10, 100110, N=1, immr=shift, imms=63, Rn, Rd
constexpr uint32_t lsr_imm_x(uint32_t rd, uint32_t rn, uint32_t shift) {
    return 0xD340FC00U | ((shift & 0x3F) << 16) | ((rn & 0x1F) << 5) | (rd & 0x1F);
}

// LSL Xd, Xn, #shift   (= UBFM Xd, Xn, #(-shift mod 64), #(63-shift)).
// Logical shift left by an immediate amount, 64-bit register.  Used by the
// mach_msg2 IPC body to place msgh_id into the high half of x4
// (mach_msg2_trap packs (voucher_port) | (msgh_id << 32)).
//   sf=1, opc=10, 100110, N=1, immr=(-shift)%64, imms=63-shift, Rn, Rd
constexpr uint32_t lsl_imm_x(uint32_t rd, uint32_t rn, uint32_t shift) {
    uint32_t immr = (64U - shift) & 0x3FU;
    uint32_t imms = (63U - shift) & 0x3FU;
    return 0xD3400000U | (immr << 16) | (imms << 10) | ((rn & 0x1F) << 5) | (rd & 0x1F);
}

// AND Wd, Wn, #0xF   — 32-bit AND with bitmask immediate hardcoded for #0xF
// (4 LSBs).  Bitmask-imm encoding (N=0, immr=0, imms=0b000011) per the
// `feedback_is_bitmask_immediate_ub.md` rule about hardcoding known
// constants instead of computing from is_bitmask_immediate (which has
// signed-shift UB when inlined into separate-function helpers).
constexpr uint32_t and_w_imm_0xf(uint32_t rd, uint32_t rn) {
    return 0x12000C00U | ((rn & 0x1F) << 5) | (rd & 0x1F);
}

// STRB Wt, [Xn|SP, #imm12]   (8-bit store, unsigned offset, no scaling)
//   00_111_0_01_00_imm12_Rn_Rt
constexpr uint32_t strb_w_offset(uint32_t wt, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = imm & 0xFFF;
    return 0x39000000U | (imm12 << 10) | ((rn & 0x1F) << 5) | (wt & 0x1F);
}

// MOV Xd, Xn   (alias for ORR Xd, XZR, Xn).  Note: cannot encode SP as Rd
// or Rm — use add_imm(rd, SP, 0) for that.
constexpr uint32_t mov_reg_x(uint32_t rd, uint32_t rm) {
    return 0xAA0003E0U | ((rm & 0x1F) << 16) | (rd & 0x1F);
}

// CBNZ Wt, +imm19*4   (32-bit branch-if-non-zero).
constexpr uint32_t cbnz_w(uint32_t rt, int32_t imm19_words) {
    uint32_t imm19 = static_cast<uint32_t>(imm19_words) & 0x7FFFF;
    return 0x35000000U | (imm19 << 5) | (rt & 0x1F);
}

// B.cond +imm19*4   (signed PC-relative conditional branch, 4-byte units).
//   0101_0100_imm19_0_cond
constexpr uint32_t b_cond(uint32_t cond, int32_t imm19_words) {
    uint32_t imm19 = static_cast<uint32_t>(imm19_words) & 0x7FFFF;
    return 0x54000000U | (imm19 << 5) | (cond & 0xF);
}
constexpr uint32_t COND_EQ = 0x0;  // equal
constexpr uint32_t COND_NE = 0x1;  // not equal
constexpr uint32_t COND_LS = 0x9;  // unsigned ≤
constexpr uint32_t COND_LT = 0xB;  // signed <
constexpr uint32_t COND_LE = 0xD;  // signed ≤

// B +imm26*4   (unconditional PC-relative branch, 26-bit signed scaled by 4
// — ±128 MB range).  Used by the IPC body's interrupt-retry blocks to jump
// back to the kr-dispatch checkpoint.
//   0001_0100_imm26
constexpr uint32_t b_uncond(int32_t imm26_words) {
    uint32_t imm26 = static_cast<uint32_t>(imm26_words) & 0x03FFFFFF;
    return 0x14000000U | imm26;
}

// LDRH Wt, [Xn|SP, #imm]   (16-bit load-zero-extend, unsigned offset, scaled by 2)
//   01_111_0_01_01_imm12_Rn_Rt
constexpr uint32_t ldrh_w_offset(uint32_t wt, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = (imm / 2) & 0xFFF;
    return 0x79400000U | (imm12 << 10) | ((rn & 0x1F) << 5) | (wt & 0x1F);
}

// STR Dt, [Xn|SP, #imm]  (64-bit FP/SIMD store, unsigned offset, scaled by 8)
//   1111 1101 00 imm12 Rn Rt   — size=11, V=1, opc=00
constexpr uint32_t str_d_offset(uint32_t dt, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = (imm / 8) & 0xFFF;
    return 0xFD000000U | (imm12 << 10) | ((rn & 0x1F) << 5) | (dt & 0x1F);
}

// LDR Dt, [Xn|SP, #imm]  (64-bit FP/SIMD load, unsigned offset, scaled by 8)
//   1111 1101 01 imm12 Rn Rt
constexpr uint32_t ldr_d_offset(uint32_t dt, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = (imm / 8) & 0xFFF;
    return 0xFD400000U | (imm12 << 10) | ((rn & 0x1F) << 5) | (dt & 0x1F);
}

// RET (= BR x30)
constexpr uint32_t RET_INSN = 0xD65F03C0U;

// MADD Xd, Xn, Xm, Xa     (Xd = Xn * Xm + Xa, 64-bit)
//   sf=1, op54=00, 11011, op31=000, Rm, o0=0, Ra, Rn, Rd
constexpr uint32_t madd(uint32_t rd, uint32_t rn, uint32_t rm, uint32_t ra) {
    return 0x9B000000U | ((rm & 0x1F) << 16) | ((ra & 0x1F) << 10) | ((rn & 0x1F) << 5) |
           (rd & 0x1F);
}

// 32-bit register encoder helpers — only differ from 64-bit by sf=0 in
// some contexts.  We don't need them yet (mach_msg uses 64-bit GPRs).

constexpr uint32_t SP = 31;
constexpr uint32_t LR = 30;

// ──── helpers ────────────────────────────────────────────────────────────────

void emit(std::vector<uint8_t>& out, uint32_t insn) {
    out.push_back(static_cast<uint8_t>(insn));
    out.push_back(static_cast<uint8_t>(insn >> 8));
    out.push_back(static_cast<uint8_t>(insn >> 16));
    out.push_back(static_cast<uint8_t>(insn >> 24));
}

// movz/movk/movk/movk to load a 64-bit address into Xd (4 instructions).
void emit_load_imm64(std::vector<uint8_t>& out, uint32_t rd, uint64_t imm) {
    emit(out, movz(rd, static_cast<uint16_t>(imm & 0xFFFF), 0));
    emit(out, movk(rd, static_cast<uint16_t>((imm >> 16) & 0xFFFF), 16));
    emit(out, movk(rd, static_cast<uint16_t>((imm >> 32) & 0xFFFF), 32));
    emit(out, movk(rd, static_cast<uint16_t>((imm >> 48) & 0xFFFF), 48));
}

// Overwrite a 4-byte instruction at byte offset `off` in an already-emitted
// blob.  Used to back-patch forward branches and abs-jump targets we
// couldn't compute when first emitting them.
void patch_word(std::vector<uint8_t>& v, size_t off, uint32_t insn) {
    v[off + 0] = static_cast<uint8_t>(insn);
    v[off + 1] = static_cast<uint8_t>(insn >> 8);
    v[off + 2] = static_cast<uint8_t>(insn >> 16);
    v[off + 3] = static_cast<uint8_t>(insn >> 24);
}

// Patch the 3 movz/movk words of an emit_abs_jump_3movs sequence at byte
// offset `off` so the abs-jump targets `target`.  The 4th word (br x16) is
// independent of the target and does not need patching.
void patch_abs_jump_3movs(std::vector<uint8_t>& v, size_t off, uint64_t target) {
    patch_word(v, off + 0, movz(16, static_cast<uint16_t>(target & 0xFFFF), 0));
    patch_word(v, off + 4, movk(16, static_cast<uint16_t>((target >> 16) & 0xFFFF), 16));
    patch_word(v, off + 8, movk(16, static_cast<uint16_t>((target >> 32) & 0xFFFF), 32));
}

// Append a placeholder abs-jump (3 movz/movk + br) whose target will be
// patched in later via patch_abs_jump_3movs.  Returns the byte offset of
// the first movz, suitable for handing back to the patcher.
size_t emit_abs_jump_placeholder(std::vector<uint8_t>& out) {
    size_t pos = out.size();
    emit(out, movz(16, 0, 0));
    emit(out, movk(16, 0, 16));
    emit(out, movk(16, 0, 32));
    emit(out, br(16));
    return pos;
}

// Build a 16-byte abs-jump to `target` via x16 (3 mov + 1 br = 16 bytes).
// The high 16 bits are assumed zero, which is true for kernel-managed
// userland virtual addresses on macOS (top byte zeroed; canonical sub-256TB).
void emit_abs_jump_3movs(std::vector<uint8_t>& out, uint64_t target) {
    emit(out, movz(16, static_cast<uint16_t>(target & 0xFFFF), 0));
    emit(out, movk(16, static_cast<uint16_t>((target >> 16) & 0xFFFF), 16));
    emit(out, movk(16, static_cast<uint16_t>((target >> 32) & 0xFFFF), 32));
    emit(out, br(16));
}

// ──── Abort routine ─────────────────────────────────────────────────────────
//
// Called from the IPC body's checkpoint thunks when mach_msg returns a
// non-zero kr, the reply's msgh_size is too small, or msgh_id doesn't match
// what we sent (Step 1b).  The routine writes a single fixed-format line
// to stderr then exits — never returns.
//
// Wine prints this line into its launcher log so we can tell, after a freeze
// repro, exactly which condition tripped and what the failing value was.
//
// Caller convention (set up by the IPC thunks):
//   x0 = diagnostic value (kr / msgh_size / actual msgh_id) — printed as 8 hex
//   x1 = error kind       (0 = bad-kr, 1 = bad-size, 2 = id-mismatch)
//   x2 = insn_idx         (caller's translate_insn arg, also printed)
//
// The output is exactly 59 bytes:
//   "rosettax87 stub abort kind=K val=0x........ idx=0x........\n"
// with K = '0' + kind, and the two `........` slots overwritten in place
// by 8 hex chars each.
//
// We can't write into the routine's own RX-mapped trailing pad — Apple
// makes the whole region executable + read-only after install (see
// `feedback_trailing_pad_page_protection.md`).  So we copy the 64-byte
// template onto the stack first via ldp/stp pairs, patch in place, then
// write(2) from the stack.
//
// Don't use BRK to crash: libRosettaRuntime's __TEXT eats EXC_BREAKPOINT
// before SIGTRAP is delivered (`feedback_brk_vs_svc_in_rosetta_text.md`).
// Use sys_write (#4) + sys_exit (#1) syscalls.
constexpr size_t kAbortRoutineCodeSize = 48UL * 4;  // 48 instructions, 192 B
constexpr size_t kAbortMsgSize = 64;                // template padded to 64 B
constexpr size_t kAbortMsgWriteLen = 59;            // bytes actually written
constexpr size_t kAbortMsgKindOff = 27;
constexpr size_t kAbortMsgValHexOff = 35;
constexpr size_t kAbortMsgIdxHexOff = 50;

void emit_abort_hex_loop(std::vector<uint8_t>& out, uint32_t valReg, uint32_t bufStartOff) {
    // Convert the 8 low nibbles of `valReg` to ASCII hex chars at
    // [sp, #bufStartOff..bufStartOff+8].  Bottom-up: end pointer in x6,
    // counter in w7, working copy in x4 (caller is responsible for spilling
    // valReg first if it overlaps a scratch we'll clobber).
    emit(out, mov_reg_x(4, valReg));             // x4 = valReg
    emit(out, add_imm(6, SP, bufStartOff + 8));  // x6 = sp + (off+8)
    emit(out, movz(7, 8, 0));                    // w7 = 8

    const size_t loop_start = out.size();
    emit(out, sub_imm_x(6, 6, 1));         // x6 -= 1
    emit(out, and_w_imm_0xf(8, 4));        // w8 = w4 & 0xF
    emit(out, add_imm_w(9, 8, '0'));       // w9 = w8 + 0x30
    emit(out, cmp_imm_w(8, 9));            // cmp w8, #9
    emit(out, b_cond(COND_LE, 2));         // b.le +2 → strb
    emit(out, add_imm_w(9, 8, 'a' - 10));  // w9 = w8 + 0x57
    emit(out, strb_w_offset(9, 6, 0));     // strb w9, [x6]
    emit(out, lsr_imm_x(4, 4, 4));         // x4 >>= 4
    emit(out, subs_imm_w(7, 7, 1));        // w7 -= 1, set flags
    const size_t loop_back_pc = out.size();
    const auto back_words = -static_cast<int32_t>((loop_back_pc - loop_start) / 4);
    emit(out, b_cond(COND_NE, back_words));  // b.ne loop_start
}

void emit_abort_routine(std::vector<uint8_t>& out, uint64_t templateAddr) {
    const size_t start = out.size();

    // sub sp, sp, #64 — reserve a writable msg buffer
    emit(out, sub_imm_x(SP, SP, 64));

    // x4 = templateAddr (3 movz/movk; assumes top 16 bits zero, same as
    // emit_abs_jump_3movs)
    emit(out, movz(4, static_cast<uint16_t>(templateAddr & 0xFFFF), 0));
    emit(out, movk(4, static_cast<uint16_t>((templateAddr >> 16) & 0xFFFF), 16));
    emit(out, movk(4, static_cast<uint16_t>((templateAddr >> 32) & 0xFFFF), 32));

    // Copy 64 bytes from RX template (x4) to stack buffer (sp) via 4×ldp/stp.
    emit(out, ldp_offset(5, 6, 4, 0));
    emit(out, stp_offset(5, 6, SP, 0));
    emit(out, ldp_offset(5, 6, 4, 16));
    emit(out, stp_offset(5, 6, SP, 16));
    emit(out, ldp_offset(5, 6, 4, 32));
    emit(out, stp_offset(5, 6, SP, 32));
    emit(out, ldp_offset(5, 6, 4, 48));
    emit(out, stp_offset(5, 6, SP, 48));

    // Patch kind digit at [sp, #kAbortMsgKindOff] from x1.
    emit(out, add_imm_w(1, 1, '0'));
    emit(out, strb_w_offset(1, SP, kAbortMsgKindOff));

    // Hex-render val (x0) into [sp, #kAbortMsgValHexOff..+8].
    emit_abort_hex_loop(out, /*valReg=*/0, kAbortMsgValHexOff);
    // Hex-render idx (x2) into [sp, #kAbortMsgIdxHexOff..+8].
    emit_abort_hex_loop(out, /*valReg=*/2, kAbortMsgIdxHexOff);

    // write(1, sp, kAbortMsgWriteLen) — fd 1 (stdout) rather than fd 2
    // (stderr): stock translate_insn ends up routing parent stderr through
    // an internal channel we don't see from the launcher, so abort messages
    // emitted to fd 2 vanish.  fd 1 lands on the launcher's stdout.
    emit(out, movz(0, 1, 0));
    emit(out, add_imm(1, SP, 0));
    emit(out, movz(2, kAbortMsgWriteLen, 0));
    emit(out, movz(16, 4, 0));
    emit(out, svc(0x80));

    // exit(137)
    emit(out, movz(0, 137, 0));
    emit(out, movz(16, 1, 0));
    emit(out, svc(0x80));

    if (out.size() - start != kAbortRoutineCodeSize) {
        // Caller will see oversized handler blob and abort install.  We
        // can't propagate a richer error from here without complicating
        // the API; sentinel-corrupt the blob so the caller's size check
        // catches it.
        out.clear();
    }
}

void append_abort_msg_template(std::vector<uint8_t>& out) {
    static constexpr char kMsg[] = "rosettax87 stub abort kind=. val=0x........ idx=0x........\n";
    static_assert(sizeof(kMsg) - 1 == kAbortMsgWriteLen,
                  "abort message template length must match write() size");
    const size_t start = out.size();
    out.insert(out.end(), kMsg, kMsg + kAbortMsgWriteLen);
    // Pad to kAbortMsgSize so the ldp copy reads aligned 16-B chunks without
    // overrunning the trailing pad.  These bytes are not written by the
    // abort routine (write len = 59, not 64).
    while (out.size() - start < kAbortMsgSize) {
        out.push_back('\n');
    }
}

// ──── Stack frame & message-buffer layout for both IPC paths ─────────────
// FRAME_SIZE = 192 bytes:
//   sp[  0..  8] : x0   (caller's translate_insn arg0 = TR*)
//   sp[  8.. 16] : x1   (block*)
//   sp[ 16.. 24] : x2   (instr_array*)
//   sp[ 24.. 32] : x3   (num_instrs)
//   sp[ 32.. 40] : x4   (insn_idx)
//   sp[ 40.. 48] : x5   (caller scratch)
//   sp[ 48.. 56] : x16  (caller scratch / kept for sym with stp)
//   sp[ 56.. 64] : LR   (return address)
//   sp[ 64..128] : 64-byte mach_msg buffer.  Send: header (24 B at +64..
//                  +88) + 5×8 args body (40 B at +88..+128).  RCV is
//                  capped at 64 B (kernel never writes past sp+128).
//                  Reply layout: header (24 B) + result (8 B at +88) +
//                  some_flag (8 B at +96).
//   sp[128..132] : expected msgh_id stash (Step 1b transaction-id check).
//   sp[132..192] : currently unused.
//
// After the trap returns:
//   x0 = KERN_RETURN.  Reply at sp+64.. is validated then dispatched on
//   some_flag (sp+96): 0 = None (fall through to STASH), 1 = Some
//   (return result at sp+88 in x0).
constexpr int kFrameSize = 192;
constexpr uint32_t kMsgBits = 0x13U | (0x15U << 8);  // = 0x1513
constexpr uint32_t kMsgSendSize = 24 + 40;           // header + 5×8 args
constexpr uint32_t kMsgRcvSize = 64;                 // reply cap
constexpr uint32_t kMsgIdSentinel = 0x10000000;
constexpr uint32_t kReplySizeMin = 24 + 8 + 8;  // 40 — see sidecar.cpp ReplyMsg

// ──── Patch positions returned by every IPC-body emitter ─────────────────
// Only abs-jump placeholders that depend on addresses outside the IPC body
// (stash, abort_routine).  Conditional-branch patches are entirely internal
// to the postlude — handled there directly.
struct IpcPatches {
    size_t none_jump;        // NONE path → STASH
    size_t kr_thunk_jump;    // kr abort thunk → abort_routine
    size_t size_thunk_jump;  // size abort thunk → abort_routine
    size_t id_thunk_jump;    // id abort thunk → abort_routine
};

// Common prologue used by both IPC bodies: stash caller-saved x0..x5,
// x16, and LR into the bottom 64 B of the new frame.
void emit_ipc_prologue(std::vector<uint8_t>& ipc) {
    emit(ipc, stp_preindex(0, 1, SP, -kFrameSize));  // stp x0, x1, [sp, #-192]!
    emit(ipc, stp_offset(2, 3, SP, 16));             // stp x2, x3, [sp, #16]
    emit(ipc, stp_offset(4, 5, SP, 32));             // stp x4, x5, [sp, #32]
    emit(ipc, stp_offset(16, LR, SP, 48));           // stp x16, lr, [sp, #48]
}

// Stash the per-call msgh_id at sp+128 for the post-svc cross-talk check.
// msgh_id = (kMsgIdSentinel byte 0x10) << 24 | (SP >> 20) & 0x00FFFFFF
// — top byte distinguishes our requests from any other Mach traffic on
// the reply port; low 24 bits derived from SP give a thread-distinguishing
// tag (different parent threads have stacks at different VAs).  Returns
// the scratch register that holds msgh_id (always x9) so callers can
// reuse it for further packing without recomputing.
void emit_build_msgh_id(std::vector<uint8_t>& ipc) {
    emit(ipc, add_imm(9, SP, 0));    // x9 = sp
    emit(ipc, lsr_imm_x(9, 9, 20));  // x9 = sp >> 20
    emit(ipc, movk(9, static_cast<uint16_t>(kMsgIdSentinel >> 16), 16));
    emit(ipc, str_w_offset(9, SP, 128));  // stash expected msgh_id
}

// Store the five caller-arg registers (x0..x4) into the message body at
// sp+88..sp+128.  Order matches the sidecar's TranslateRequest layout.
void emit_body_stores(std::vector<uint8_t>& ipc) {
    emit(ipc, str_x_offset(0, SP, 88));   // body[+0]  = TR*
    emit(ipc, str_x_offset(1, SP, 96));   // body[+8]  = block*
    emit(ipc, str_x_offset(2, SP, 104));  // body[+16] = instr_array*
    emit(ipc, str_x_offset(3, SP, 112));  // body[+24] = num_instrs
    emit(ipc, str_x_offset(4, SP, 120));  // body[+32] = insn_idx
}

// Emit the post-trap sequence shared by both IPC bodies: three reply
// validation checkpoints, the SOME-path return, the NONE-path abs-jump
// placeholder, and the three abort thunks.  Returns the patch positions
// for build() to fill in once stashAddr / abortRoutineAddr are known.
//
// Pre-conditions (set by the trap-specific code preceding this):
//   x0 = kr (kern_return_t — zero-extended into x0 by mach_msg{,2}_trap)
// Reply lives at sp+64..sp+128.  msgh_size at sp+68, msgh_id at sp+84,
// result at sp+88, some_flag at sp+96.  Expected msgh_id stashed at
// sp+128 by emit_build_msgh_id() pre-svc.
//
// Every non-success path ends in a loud abort + write to fd 2 — we do
// NOT silently fall through on a bad reply: doing so previously masked
// the multi-threaded reply-port cross-talk that froze WoW on world-load.
IpcPatches emit_ipc_postlude(std::vector<uint8_t>& ipc) {
    IpcPatches p{};

    // ── Reply validation (Step 1a) ──────────────────────────────────────
    //
    // Three checkpoints, in order:
    //   1. kr != KERN_SUCCESS         (mach_msg failed)
    //   2. msgh_size < kReplySizeMin  (reply too small for Some/None)
    //   3. msgh_id  != expected       (cross-talk — Step 1b transaction id)

    // Checkpoint 1: kr != 0 → abort kind=0.  Uses cbnz_w (32-bit) since
    // kern_return_t is 32-bit and the trap zero-extends into x0.
    const size_t krCbnzPos = ipc.size();
    emit(ipc, cbnz_w(0, 0));  // patched after thunks emitted

    // Checkpoint 2: msgh_size < kReplySizeMin (40) → abort kind=1.
    emit(ipc, ldr_w_offset(10, SP, 68));  // w10 = msgh_size
    emit(ipc, cmp_imm_w(10, kReplySizeMin));
    const size_t sizeBltPos = ipc.size();
    emit(ipc, b_cond(COND_LT, 0));  // patched after thunks emitted

    // Checkpoint 3: msgh_id != expected → abort kind=2 (cross-talk).
    emit(ipc, ldr_w_offset(11, SP, 84));   // w11 = reply msgh_id
    emit(ipc, ldr_w_offset(12, SP, 128));  // w12 = expected (stashed pre-svc)
    emit(ipc, cmp_reg_w(11, 12));
    const size_t idBnePos = ipc.size();
    emit(ipc, b_cond(COND_NE, 0));  // patched after thunks emitted

    // Dispatch on some_flag at sp+96.  0 = None (fall through to STASH),
    // 1 = Some.  CBZ skips 7 SOME-path instructions to land on NONE.
    emit(ipc, ldr_w_offset(9, SP, 96));
    emit(ipc, cbz_w(9, 8));

    // ── SOME PATH ───────────────────────────────────────────────────────
    emit(ipc, ldr_x_offset(0, SP, 88));      // x0 = result
    emit(ipc, ldr_x_offset(1, SP, 8));       // restore x1
    emit(ipc, ldp_offset(2, 3, SP, 16));     // restore x2, x3
    emit(ipc, ldp_offset(4, 5, SP, 32));     // restore x4, x5
    emit(ipc, ldp_offset(16, LR, SP, 48));   // restore x16, lr
    emit(ipc, add_imm(SP, SP, kFrameSize));  // sp += FRAME_SIZE
    emit(ipc, RET_INSN);                     // ret

    // ── NONE PATH ───────────────────────────────────────────────────────
    // Restore caller regs and abs-jump to STASH.  x0 comes from sp+0 —
    // the SOME path read from sp+88 (reply body), but for NONE we hand
    // stock the caller's untouched args.
    emit(ipc, ldr_x_offset(0, SP, 0));
    emit(ipc, ldr_x_offset(1, SP, 8));
    emit(ipc, ldp_offset(2, 3, SP, 16));
    emit(ipc, ldp_offset(4, 5, SP, 32));
    emit(ipc, ldp_offset(16, LR, SP, 48));
    emit(ipc, add_imm(SP, SP, kFrameSize));
    p.none_jump = emit_abs_jump_placeholder(ipc);

    // ── Abort thunks ────────────────────────────────────────────────────
    // Each runs only when its checkpoint above branched here.  Sets up
    // x0 = diagnostic value, x1 = kind, x2 = insn_idx (loaded from sp+32
    // where the prologue stashed caller's x4), then abs-jumps to
    // abort_routine.  Abs-jump targets patched by the caller.

    // Thunk 0: bad-kr.  x0 already holds kr; kind = 0.
    const size_t krThunkPos = ipc.size();
    emit(ipc, movz(1, 0, 0));
    emit(ipc, ldr_x_offset(2, SP, 32));
    p.kr_thunk_jump = emit_abs_jump_placeholder(ipc);

    // Thunk 1: bad-size.  msgh_size sits in w10; kind = 1.
    const size_t sizeThunkPos = ipc.size();
    emit(ipc, mov_reg_x(0, 10));
    emit(ipc, movz(1, 1, 0));
    emit(ipc, ldr_x_offset(2, SP, 32));
    p.size_thunk_jump = emit_abs_jump_placeholder(ipc);

    // Thunk 2: id-mismatch.  Actual msgh_id is in w11; kind = 2.
    const size_t idThunkPos = ipc.size();
    emit(ipc, mov_reg_x(0, 11));
    emit(ipc, movz(1, 2, 0));
    emit(ipc, ldr_x_offset(2, SP, 32));
    p.id_thunk_jump = emit_abs_jump_placeholder(ipc);

    // Patch the three checkpoint forward branches to land on their
    // thunks now that we know each thunk's offset.
    patch_word(ipc, krCbnzPos, cbnz_w(0, static_cast<int32_t>((krThunkPos - krCbnzPos) / 4)));
    patch_word(ipc, sizeBltPos,
               b_cond(COND_LT, static_cast<int32_t>((sizeThunkPos - sizeBltPos) / 4)));
    patch_word(ipc, idBnePos, b_cond(COND_NE, static_cast<int32_t>((idThunkPos - idBnePos) / 4)));

    return p;
}

// mach_msg2_trap argument packing + svc.  Used both for the initial
// SEND+RCV+MQ_CALL and for the interrupt-retry paths below.
//
// Argument packing is `(low32_field) | (high32_field << 32)` per
// mach_msg2_trap — verified by disassembling _mach_msg_overwrite in
// /usr/lib/system/libsystem_kernel.dylib (macOS 26).  Public SDK headers
// don't expose the trap or MACH64_* constants, so the values are baked
// here.
//
//   x0  = data pointer            (sp + 64)
//   x1  = options64               (caller-supplied)
//   x2  = (msgh_bits)             | (send_size      << 32)
//   x3  = (sidecarReqName)        | (parentReplyName<< 32)
//   x4  = (voucher = 0)           | (msgh_id        << 32)
//   x5  = (desc_count = 0)        | (rcv_name = parentReplyName << 32)
//   x6  = (rcv_size = 64)         | (priority = 0   << 32)
//   x7  = timeout = 0
//   x16 = -47  (mach_msg2_trap)
//
// The xnu MACH64_* bit assignments (from osfmk/mach/message.h):
//   MACH64_SEND_USER_CALL    = 0x0000'0002'0000'0000   (bit 33)
//   MACH64_SEND_MQ_CALL      = 0x0000'0004'0000'0000   (bit 34)
//   MACH64_SEND_KOBJECT_CALL = 0x0000'0008'0000'0000   (bit 35)
//   MACH64_SEND_DK_CALL      = 0x0000'0010'0000'0000   (bit 36)
// These aren't exposed in the public SDK headers; verified by
// breakpointing libsystem_kernel's `mach_msg2_internal` and reading the
// x1 register on a working SEND|RCV call.
//
// `send_args`: when false (RCV-only retry), x2/x3/x4 are zeroed — the
// kernel ignores them when MACH_SEND_MSG isn't set.  When true and
// `msgh_id_in_x9` is true, x9 is assumed to still hold msgh_id from a
// preceding emit_build_msgh_id; otherwise we reload from the sp+128
// stash (x9 is clobbered across the trap, so retries must reload).
constexpr uint64_t kMach64SendMqCall = 0x0000'0004'0000'0000ULL;
constexpr uint64_t kSendRcvOpt = 0x0000'0000'0000'0003ULL;
constexpr uint64_t kRcvOnlyOpt = 0x0000'0000'0000'0002ULL;

void emit_mach_msg2_call(std::vector<uint8_t>& ipc, uint32_t sidecarReqName,
                         uint32_t parentReplyName, uint64_t options64, bool send_args,
                         bool msgh_id_in_x9) {
    emit(ipc, add_imm(0, SP, 64));  // x0 = sp + 64
    emit_load_imm64(ipc, 1, options64);

    if (send_args) {
        emit_load_imm64(
            ipc, 2, static_cast<uint64_t>(kMsgBits) | (static_cast<uint64_t>(kMsgSendSize) << 32));
        emit_load_imm64(
            ipc, 3,
            static_cast<uint64_t>(sidecarReqName) | (static_cast<uint64_t>(parentReplyName) << 32));
        if (!msgh_id_in_x9) {
            emit(ipc, ldr_w_offset(9, SP, 128));  // reload msgh_id from stash
        }
        emit(ipc, lsl_imm_x(4, 9, 32));  // x4 = msgh_id << 32 (voucher = 0)
    } else {
        // RCV-only: send-side args ignored by the kernel; zero for hygiene.
        emit(ipc, movz(2, 0, 0));
        emit(ipc, movz(3, 0, 0));
        emit(ipc, movz(4, 0, 0));
    }

    // x5 = parentReplyName << 32 (rcv_name in high half, desc_count = 0 in low).
    emit_load_imm64(ipc, 10, static_cast<uint64_t>(parentReplyName) << 32);
    emit(ipc, mov_reg_x(5, 10));

    emit(ipc, movz(6, kMsgRcvSize, 0));  // x6 = rcv_size (priority = 0)
    emit(ipc, movz(7, 0, 0));            // x7 = timeout = 0
    emit(ipc, movn(16, 46, 0));          // x16 = ~46 = -47 (mach_msg2_trap)
    emit(ipc, svc(0x80));
}

// Build the IPC body around `mach_msg2_trap` (svc -47) with
// `MACH64_SEND_MQ_CALL` set in the high half of options.  This is the
// XPC-style "MIG call" fast path: the kernel knows we'll immediately RCV
// on `parentReplyName` after our SEND lands at the sidecar, and can do
// a direct hand-off to the sidecar thread (already blocked in
// mach_msg(MACH_RCV_MSG)) without going through the scheduler.
//
// libsystem's `mach_msg()` / `mach_msg2()` wrappers contain a retry loop
// on MACH_SEND_INTERRUPTED / MACH_RCV_INTERRUPTED — a trap interrupted
// by a signal (timer, BSD signal, exception port).  Our direct svc -47
// has no such loop, so without the dispatch block below an interrupted
// receive bubbled up to a kind=0 abort and killed the wine'd process.
// TurtleWoW launches were occasionally hitting this with `kr=0x10004005`
// (MACH_RCV_INTERRUPTED).  We mirror the libsystem behavior here:
//
//   kr == 0                    → fall through to validation.
//   kr == MACH_RCV_INTERRUPTED → SEND completed, RCV interrupted; reply
//                                is queued, drain it with an RCV-only
//                                trap.  A blind full-retry would
//                                duplicate the request and leave a stale
//                                reply on the port that would trip the
//                                kind=2 cross-talk check on the next call.
//   kr == MACH_SEND_INTERRUPTED → SEND didn't land; full re-trap is
//                                idempotent.  Rare with MQ_CALL but cheap.
//   anything else              → fall through; postlude's cbnz_w(0)
//                                catches it and routes to kind=0 abort.
//
// We don't pre-populate the header in the on-stack buffer — the kernel
// synthesizes it from the register-packed args.  Body bytes still live
// at sp+88..128 (kernel reads from `data + 24`).  An interrupted RCV
// leaves the receive buffer untouched, so the request body and the
// msgh_id stash at sp+128 are still valid for retry.
constexpr uint32_t kMachRcvInterrupted = 0x10004005U;
constexpr uint32_t kMachSendInterrupted = 0x10000007U;

IpcPatches emit_mach_msg2_ipc(std::vector<uint8_t>& ipc, uint32_t sidecarReqName,
                              uint32_t parentReplyName) {
    emit_ipc_prologue(ipc);

    // Body: five translate_insn args (still in x0..x4).
    emit_body_stores(ipc);

    // Build msgh_id in x9 and stash at sp+128 for the post-svc check.
    // We use x9 directly for the initial trap's x4-pack below; retries
    // reload from sp+128 because x9 doesn't survive the trap.
    emit_build_msgh_id(ipc);

    // Initial call: full SEND+RCV with MIG fast-path hand-off.
    emit_mach_msg2_call(ipc, sidecarReqName, parentReplyName, kSendRcvOpt | kMach64SendMqCall,
                        /*send_args=*/true,
                        /*msgh_id_in_x9=*/true);

    // ── Retry-on-interrupt dispatch ─────────────────────────────────────
    // Both b.eq forward branches and the cbz_w skip-to-validate are
    // patched once we know the postlude/retry-block offsets.
    const size_t dispatchPos = ipc.size();

    const size_t cbzZeroPos = ipc.size();
    emit(ipc, cbz_w(0, 0));  // patched to validatePos

    emit(ipc, movz(13, static_cast<uint16_t>(kMachRcvInterrupted & 0xFFFF), 0));
    emit(ipc, movk(13, static_cast<uint16_t>(kMachRcvInterrupted >> 16), 16));
    emit(ipc, cmp_reg_w(0, 13));
    const size_t bRetryRcvPos = ipc.size();
    emit(ipc, b_cond(COND_EQ, 0));  // patched to retryRcvPos

    emit(ipc, movz(13, static_cast<uint16_t>(kMachSendInterrupted & 0xFFFF), 0));
    emit(ipc, movk(13, static_cast<uint16_t>(kMachSendInterrupted >> 16), 16));
    emit(ipc, cmp_reg_w(0, 13));
    const size_t bRetryFullPos = ipc.size();
    emit(ipc, b_cond(COND_EQ, 0));  // patched to retryFullPos

    // Anything else falls through to postlude; its first instruction
    // (cbnz_w(0)) catches non-zero kr and routes to the kind=0 abort.
    const size_t validatePos = ipc.size();
    patch_word(ipc, cbzZeroPos, cbz_w(0, static_cast<int32_t>((validatePos - cbzZeroPos) / 4)));

    IpcPatches p = emit_ipc_postlude(ipc);

    // ── Retry blocks ────────────────────────────────────────────────────
    // Past the postlude, reachable only via the patched b.eq forwards
    // above.  Each re-traps and branches back to dispatchPos to re-check
    // the new kr (in case the retry was itself interrupted).
    const size_t retryRcvPos = ipc.size();
    emit_mach_msg2_call(ipc, sidecarReqName, parentReplyName, kRcvOnlyOpt,
                        /*send_args=*/false, /*msgh_id_in_x9=*/false);
    {
        const auto back = static_cast<int32_t>((dispatchPos - ipc.size()) / 4);
        emit(ipc, b_uncond(back));
    }

    const size_t retryFullPos = ipc.size();
    emit_mach_msg2_call(ipc, sidecarReqName, parentReplyName, kSendRcvOpt | kMach64SendMqCall,
                        /*send_args=*/true,
                        /*msgh_id_in_x9=*/false);
    {
        const auto back = static_cast<int32_t>((dispatchPos - ipc.size()) / 4);
        emit(ipc, b_uncond(back));
    }

    patch_word(ipc, bRetryRcvPos,
               b_cond(COND_EQ, static_cast<int32_t>((retryRcvPos - bRetryRcvPos) / 4)));
    patch_word(ipc, bRetryFullPos,
               b_cond(COND_EQ, static_cast<int32_t>((retryFullPos - bRetryFullPos) / 4)));

    return p;
}

}  // namespace

// ──── public ────────────────────────────────────────────────────────────────

StubBlobs build(uint64_t handlerAddr, uint64_t translateInsnAddr, const uint8_t origPrologue16[16],
                uint32_t sidecarReqName, uint32_t parentReplyName) {
    StubBlobs blobs;

    // ENTRY: 16-byte abs-jump to handlerAddr, written into translate_insn[0..16].
    blobs.entry.reserve(16);
    emit_abs_jump_3movs(blobs.entry, handlerAddr);

    // Sanity: the 3-mov+br encoding only works if the target's top 16 bits
    // are zero.  macOS userland addresses are always sub-256TB, so this
    // holds.  Caller sees entry shorter than 16 → failure signal.
    if (handlerAddr & 0xFFFF000000000000ULL) {
        blobs.entry.clear();
        return blobs;
    }

    // We build in three stages so the filter prologue can absolute-jump
    // straight to STASH for non-x87 opcodes (skipping the IPC entirely):
    //   1. Build the IPC body — its size determines STASH's offset.
    //   2. Build the filter prologue (fixed size).
    //   3. Concatenate filter + ipc + STASH + STASH_JUMP + abort_routine + msg.

    constexpr size_t kFilterInstrs = 13;
    constexpr size_t kFilterBytes = kFilterInstrs * 4;

    std::vector<uint8_t> ipc;
    const IpcPatches patches = emit_mach_msg2_ipc(ipc, sidecarReqName, parentReplyName);

    // ── Resolve final addresses and patch placeholders ──────────────────────
    // STASH starts at handlerAddr + kFilterBytes + ipc.size_final;
    // abort_routine starts at STASH_JUMP_END = STASH + 32.
    const uint64_t stashAddrFinal = handlerAddr + kFilterBytes + ipc.size();
    const uint64_t abortRoutineAddr = stashAddrFinal + 32;
    if ((stashAddrFinal | abortRoutineAddr) & 0xFFFF000000000000ULL) {
        blobs.entry.clear();
        return blobs;
    }
    patch_abs_jump_3movs(ipc, patches.none_jump, stashAddrFinal);
    patch_abs_jump_3movs(ipc, patches.kr_thunk_jump, abortRoutineAddr);
    patch_abs_jump_3movs(ipc, patches.size_thunk_jump, abortRoutineAddr);
    patch_abs_jump_3movs(ipc, patches.id_thunk_jump, abortRoutineAddr);

    // ──── FILTER prologue ───────────────────────────────────────────────────
    // Bypass the IPC entirely for non-x87 opcodes — translate_insn fires for
    // EVERY x86 instruction Rosetta translates (~85k/test) but only ~14% of
    // them are x87.  Without this filter every non-x87 call paid an IPC
    // round-trip just to be told "fall through to stock".
    //
    // The filter dereferences the IRInstr at &instr_array[insn_idx] (parent
    // memory; we run inside parent's address space) and reads its 16-bit
    // opcode field.  If the opcode is outside both x87 ranges
    //   low:  kOpcodeName_fcmovb .. kOpcodeName_fucomip (FCMOVcc + FCOMI variants)
    //   high: kOpcodeName_f2xm1  .. kOpcodeName_fyl2xp1 (most x87 ops)
    // it abs-jumps straight to STASH so the original translate_insn prologue
    // runs and stock handles the instruction itself — same outcome as an IPC
    // reply of None, but without the round-trip.
    //
    // Uses x9, x10, x11 — caller-saved scratch in AAPCS, so we don't need to
    // preserve them across the filter.
    static_assert(sizeof(IRInstr) == 0x50, "stub filter assumes IRInstr stride 0x50");
    static_assert(offsetof(IRInstr, opcode_) == 0x4, "stub filter assumes IRInstr.opcode_ at +4");
    static_assert(kOpcodeName_fucomip - kOpcodeName_fcmovb == 11,
                  "stub filter assumes fcmovb..fucomip is 12 contiguous opcodes");
    static_assert(kOpcodeName_fyl2xp1 - kOpcodeName_f2xm1 == 79,
                  "stub filter assumes f2xm1..fyl2xp1 is 80 contiguous opcodes");
    static_assert(
        kOpcodeName_fxsave >= kOpcodeName_f2xm1 && kOpcodeName_fxsave <= kOpcodeName_fyl2xp1,
        "stub filter assumes fxsave sits inside the high x87 range");
    static_assert(
        kOpcodeName_fxrstor >= kOpcodeName_f2xm1 && kOpcodeName_fxrstor <= kOpcodeName_fyl2xp1,
        "stub filter assumes fxrstor sits inside the high x87 range");

    // The IRInstr::opcode_ field carries the runtime's *host* opcode IDs, not
    // our canonical (kOpcodeName_*) IDs.  Both 26.4 and 26.5 keep the two x87
    // ranges contiguous and same-sized, so the cmp_imm_w (range size) stays
    // canonical, but the sub_imm_w (range start) must be host-translated via
    // the compat layer.  rosetta_core_init must have run before stub injection.
    const auto host_fcmovb = opcode_internal_to_host(kOpcodeName_fcmovb);
    const auto host_f2xm1 = opcode_internal_to_host(kOpcodeName_f2xm1);

    std::vector<uint8_t> filter;
    filter.reserve(kFilterBytes);
    //  0: movz w9, #0x50              ; sizeof(IRInstr) — fits in 16 bits
    emit(filter, movz(9, 0x50, 0));
    //  1: madd x10, x4, x9, x2        ; x10 = &instr_array[insn_idx]
    emit(filter, madd(10, 4, 9, 2));
    //  2: ldrh w9, [x10, #4]          ; w9 = uint16_t opcode
    emit(filter, ldrh_w_offset(9, 10, 4));
    //  3: sub w11, w9, #host_fcmovb
    emit(filter, sub_imm_w(11, 9, host_fcmovb));
    //  4: cmp w11, #(fucomip-fcmovb)  ; in-low-range if w11 ≤ this unsigned
    emit(filter, cmp_imm_w(11, kOpcodeName_fucomip - kOpcodeName_fcmovb));
    //  5: b.ls do_ipc                 ; +8 instr → land at do_ipc (instr 13)
    emit(filter, b_cond(COND_LS, 8));
    //  6: sub w11, w9, #host_f2xm1
    emit(filter, sub_imm_w(11, 9, host_f2xm1));
    //  7: cmp w11, #(fyl2xp1-f2xm1)   ; in-high-range if w11 ≤ this unsigned
    emit(filter, cmp_imm_w(11, kOpcodeName_fyl2xp1 - kOpcodeName_f2xm1));
    //  8: b.ls do_ipc                 ; +5 instr → land at do_ipc (instr 13)
    emit(filter, b_cond(COND_LS, 5));
    //  9..12: abs-jump to STASH (movz/movk/movk x16, $stashAddr; br x16)
    emit_abs_jump_3movs(filter, stashAddrFinal);
    // 13: do_ipc — IPC body starts here when filter falls through.

    if (filter.size() != kFilterBytes) {
        blobs.entry.clear();
        return blobs;
    }

    // ──── Concatenate: filter + IPC + STASH + STASH_JUMP + abort + msg ─────
    auto& h = blobs.handler;
    h.reserve(filter.size() + ipc.size() + 16 + 16 + kAbortRoutineCodeSize + kAbortMsgSize);
    h.insert(h.end(), filter.begin(), filter.end());
    h.insert(h.end(), ipc.begin(), ipc.end());
    // STASH (4 instructions = original 16 bytes of translate_insn).
    h.insert(h.end(), origPrologue16, origPrologue16 + 16);
    // STASH_JUMP (abs-jump to translate_insn + 16).
    emit_abs_jump_3movs(h, translateInsnAddr + 16);

    // Abort routine + message template.  Address must equal abortRoutineAddr
    // (= stashAddrFinal + 32) computed above; if emit_abort_routine ever
    // grows beyond kAbortRoutineCodeSize, the IPC's pre-patched abs-jumps
    // would land on the wrong instruction, so it self-checks and clears
    // its output on size mismatch.  Abort install on that.
    const size_t abortStart = h.size();
    if (handlerAddr + abortStart != abortRoutineAddr) {
        blobs.entry.clear();
        return blobs;
    }
    const uint64_t templateAddr = abortRoutineAddr + kAbortRoutineCodeSize;
    emit_abort_routine(h, templateAddr);
    if (h.size() != abortStart + kAbortRoutineCodeSize) {
        blobs.entry.clear();
        return blobs;
    }
    append_abort_msg_template(h);

    return blobs;
}

}  // namespace stub_asm
