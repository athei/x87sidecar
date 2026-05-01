#include "stub_asm.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "rosetta_core/IRInstr.h"

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
constexpr uint32_t COND_NE = 0x1;  // not equal
constexpr uint32_t COND_LS = 0x9;  // unsigned ≤
constexpr uint32_t COND_LT = 0xB;  // signed <
constexpr uint32_t COND_LE = 0xD;  // signed ≤

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

    // write(2, sp, kAbortMsgWriteLen)
    emit(out, movz(0, 2, 0));                  // x0 = 2 (stderr)
    emit(out, add_imm(1, SP, 0));              // x1 = sp (mov xd, sp)
    emit(out, movz(2, kAbortMsgWriteLen, 0));  // x2 = 59
    emit(out, movz(16, 4, 0));                 // x16 = 4 (sys_write)
    emit(out, svc(0x80));

    // exit(137)
    emit(out, movz(0, 137, 0));  // x0 = 137
    emit(out, movz(16, 1, 0));   // x16 = 1 (sys_exit)
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

}  // namespace

// ──── public ────────────────────────────────────────────────────────────────

StubBlobs build(uint64_t handlerAddr, uint64_t translateInsnAddr, const uint8_t origPrologue16[16],
                uint32_t sidecarReqName, uint32_t parentReplyName) {
    StubBlobs blobs;

    // ENTRY: 16-byte abs-jump to handlerAddr, written into translate_insn[0..16].
    blobs.entry.reserve(16);
    emit_abs_jump_3movs(blobs.entry, handlerAddr);

    // Sanity: the 3-mov+br encoding only works if the target's top 16 bits
    // are zero. macOS userland addresses are always sub-256TB, so this
    // holds.  Assert at runtime if the assumption breaks.
    if (handlerAddr & 0xFFFF000000000000ULL) {
        // Caller will see entry shorter than 16 — use as failure signal.
        blobs.entry.clear();
        return blobs;
    }

    // We build in three stages so the filter prologue can absolute-jump
    // straight to STASH for non-x87 opcodes (skipping the IPC entirely).
    //   1. Build the IPC body (`ipc`) — its size determines STASH's offset.
    //   2. Build the filter prologue (`filter`) — fixed size, computes
    //      &instr_array[insn_idx], loads opcode, and abs-jumps to STASH if
    //      the opcode is outside both x87 ranges.
    //   3. Concatenate filter + ipc + STASH + STASH_JUMP into the handler.

    // Filter size is fixed; declared up here so the IPC body's NONE
    // path can compute the same stashAddr the filter prologue uses
    // below at line ~471.
    constexpr size_t kFilterInstrs = 13;
    constexpr size_t kFilterBytes = kFilterInstrs * 4;

    std::vector<uint8_t> ipc;

    // ──── OUR_HANDLER (RPC version: send + receive) ─────────────────────────
    // Stack frame layout (FRAME_SIZE = 192 bytes):
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
    //                  capped at 64 B too (RCV_SIZE), so the kernel never
    //                  writes past sp+128 — leaving sp+128.. clear.
    //                  Reply lands at sp+64..sp+(64+RCV_SIZE).
    //                  Reply layout: header (24 B) + result (8 B at +88) +
    //                  some_flag (8 B at +96).
    //   sp[128..132] : expected msgh_id stash (Step 1b transaction-id check).
    //   sp[132..192] : currently unused.
    //
    // After mach_msg(SEND|RCV) returns:
    //   x0 = KERN_RETURN.
    //   Reply buffer at sp+64..  Validation order (Step 1a/1b):
    //     1. kr (x0) == KERN_SUCCESS — abort kind=0 if not
    //     2. msgh_size (sp+68) >= 40 — abort kind=1 if not
    //     3. msgh_id (sp+84) == expected (sp+128) — abort kind=2 if not
    //   Then dispatch on some_flag (sp+96): 0 = None (fall through to STASH),
    //   1 = Some, return result at sp+88 in x0.

    constexpr int FRAME_SIZE = 192;

    // stp x0, x1, [sp, #-128]!
    emit(ipc, stp_preindex(0, 1, SP, -FRAME_SIZE));
    // stp x2, x3, [sp, #16]
    emit(ipc, stp_offset(2, 3, SP, 16));
    // stp x4, x5, [sp, #32]
    emit(ipc, stp_offset(4, 5, SP, 32));
    // stp x16, lr, [sp, #48]
    emit(ipc, stp_offset(16, LR, SP, 48));

    // ── Build mach_msg_header_t at sp+64 ────────────────────────────────────
    // mach_msg_header_t layout (24 bytes):
    //   uint32_t  msgh_bits           [+ 0]
    //   uint32_t  msgh_size           [+ 4]
    //   uint32_t  msgh_remote_port    [+ 8]
    //   uint32_t  msgh_local_port     [+12]
    //   uint32_t  msgh_voucher_port   [+16]
    //   uint32_t  msgh_id             [+20]
    //
    // We send a "tickle" message with no body. msgh_bits selects the
    // disposition for the remote port. We use COPY_SEND so the right at
    // parent's name (planted via mach_port_insert_right) is preserved
    // across the many calls translate_insn fires.
    //
    // MACH_MSGH_BITS_REMOTE(MACH_MSG_TYPE_COPY_SEND) = 19 = 0x13.
    // (NOT 0x11 — that's MOVE_SEND which consumes the right after one use.)
    // After the 24-byte header we lay down a 40-byte body holding the five
    // translate_insn arguments in their natural register order:
    //   body[+ 0..+ 8] : x0  (TranslationResult* — parent-side pointer)
    //   body[+ 8..+16] : x1  (IRBlock*          — parent-side pointer)
    //   body[+16..+24] : x2  (IRInstr*          — parent-side pointer to array)
    //   body[+24..+32] : x3  (num_instrs)
    //   body[+32..+40] : x4  (insn_idx)
    //
    // MACH_MSGH_BITS encoding: remote in low byte, local in next byte.
    //   remote = MACH_MSG_TYPE_COPY_SEND      = 19 = 0x13  (preserves the
    //            send right at parent's namespaced sidecarReqName)
    //   local  = MACH_MSG_TYPE_MAKE_SEND_ONCE = 21 = 0x15  (kernel hands
    //            the sidecar a fresh send-once for the reply)
    constexpr uint32_t MSG_BITS = 0x13U | (0x15U << 8);  // = 0x1513
    constexpr uint32_t MSG_SIZE = 24 + 40;               // header + 5×8 args
    constexpr uint32_t RCV_SIZE = 64;                    // reply cap (header
                                                         // 24 + result 8 +
                                                         // some_flag 8 +
                                                         // trailer slack);
                                                         // capped low so
                                                         // the kernel never
                                                         // writes past
                                                         // sp+128 — keeps
                                                         // sp+128.. free
                                                         // for the stashed
                                                         // expected msgh_id.
    constexpr uint32_t REPLY_SIZE_MIN = 24 + 8 + 8;      // 40 — see sidecar.cpp ReplyMsg

    // Per-call transaction id sentinel byte.  Sidecar dispatches on
    // (msgh_id & 0xFF000000) == kMsgIdSentinel.  Bottom 24 bits carry a
    // thread-distinguishing tag derived from SP — different parent threads
    // have stacks at different VAs, so SP>>20 (megabyte-aligned) gives an
    // identifier that's stable per-thread within a single call.
    constexpr uint32_t MSG_ID_SENTINEL = 0x10000000;

    // Use x9 as scratch for header field values.

    // msgh_bits — 0x1513 fits in 16 bits (single movz)
    emit(ipc, movz(9, MSG_BITS, 0));
    emit(ipc, str_w_offset(9, SP, 64));  // [sp+64]

    // msgh_size
    emit(ipc, movz(9, MSG_SIZE, 0));
    emit(ipc, str_w_offset(9, SP, 68));  // [sp+68]

    // msgh_remote_port = sidecarReqName (32-bit)
    emit_load_imm64(ipc, 9, sidecarReqName);  // 4 instructions
    emit(ipc, str_w_offset(9, SP, 72));       // [sp+72]

    // msgh_local_port = parentReplyName (32-bit) — kernel auto-derives
    // a SEND_ONCE right via MAKE_SEND_ONCE in MSG_BITS.local.
    emit_load_imm64(ipc, 9, parentReplyName);  // 4 instructions
    emit(ipc, str_w_offset(9, SP, 76));        // [sp+76]

    // msgh_voucher_port = 0
    emit(ipc, movz(9, 0, 0));
    emit(ipc, str_w_offset(9, SP, 80));  // [sp+80]

    // msgh_id — per-call transaction id (Step 1b).  Built as
    //   (sentinel byte 0x10) << 24  |  (SP >> 20) & 0x00FFFFFF
    // and stashed at sp+128 for post-svc match check.  We replicate the
    // sentinel byte into bits 16..27 via movk's lsl-16 chunk, so the top
    // half of msgh_id is always 0x1000_xxxx (where xxxx = SP[35:20]).
    emit(ipc, add_imm(9, SP, 0));                                    // x9 = sp
    emit(ipc, lsr_imm_x(9, 9, 20));                                  // x9 = sp >> 20
    emit(ipc, movk(9, static_cast<uint16_t>(MSG_ID_SENTINEL >> 16),  // top 16 = 0x1000
                   16));
    emit(ipc, str_w_offset(9, SP, 84));   // request msgh_id
    emit(ipc, str_w_offset(9, SP, 128));  // stash expected

    // ── Body: five translate_insn args (still in x0..x4 at this point) ──────
    // Header lives at sp+64 .. sp+88. Body lives at sp+88 .. sp+128.
    emit(ipc, str_x_offset(0, SP, 88));   // body[+0]  = x0 = TR*
    emit(ipc, str_x_offset(1, SP, 96));   // body[+8]  = x1 = block*
    emit(ipc, str_x_offset(2, SP, 104));  // body[+16] = x2 = instr_array*
    emit(ipc, str_x_offset(3, SP, 112));  // body[+24] = x3 = num_instrs
    emit(ipc, str_x_offset(4, SP, 120));  // body[+32] = x4 = insn_idx

    // ── mach_msg_trap arguments ──────────────────────────────────────────────
    //   x0  = msg pointer (sp + 64)
    //   x1  = options = MACH_SEND_MSG | MACH_RCV_MSG = 0x3
    //   x2  = send_size = 64
    //   x3  = rcv_size  = 128
    //   x4  = rcv_name  = parentReplyName  (low 32 bits)
    //   x5  = timeout   = 0  (block forever)
    //   x6  = notify    = 0
    //   x16 = -31  (mach_msg_trap)
    emit(ipc, add_imm(0, SP, 64));             // x0 = sp + 64
    emit(ipc, movz(1, 0x0003, 0));             // x1 = SEND | RCV
    emit(ipc, movz(2, MSG_SIZE, 0));           // x2 = send_size
    emit(ipc, movz(3, RCV_SIZE, 0));           // x3 = rcv_size
    emit_load_imm64(ipc, 4, parentReplyName);  // x4 = rcv_name
    emit(ipc, movz(5, 0, 0));                  // x5 = timeout
    emit(ipc, movz(6, 0, 0));                  // x6 = notify
    emit(ipc, movn(16, 30, 0));                // x16 = -31 (mach_msg_trap)
    emit(ipc, svc(0x80));

    // ── Reply validation (Step 1a) ──────────────────────────────────────────
    // Every non-success path from here ends in a loud abort + write to
    // stderr.  We do NOT silently fall through on a bad reply: doing so
    // hides regressions and previously masked the multi-threaded
    // reply-port cross-talk that froze WoW on world-load.
    //
    // Three checkpoints, in order:
    //   1. kr != KERN_SUCCESS         (mach_msg failed)
    //   2. msgh_size < 32             (reply too small to be a well-formed Some/None)
    //   3. msgh_id  != expected       (cross-talk — Step 1b adds per-call ids)
    // Each fails into an in-IPC thunk that sets up
    //   x0 = diagnostic value, x1 = kind, x2 = insn_idx
    // and abs-jumps to abort_routine (appended to the handler blob after
    // STASH_JUMP).  abort_routine writes a fixed 59-byte line to fd 2 and
    // exit(137)s.
    //
    // The conditional branches below are forward-only and use placeholder
    // imm19 = 0; we patch them after the thunks are emitted (we don't yet
    // know how far ahead the thunks land).  Same trick for the thunks'
    // abs-jumps to abort_routine — patched after we know its address.

    // Checkpoint 1: kr != 0 → abort kind=0.  Uses cbnz_w (32-bit) since
    // kern_return_t is 32-bit and mach_msg_trap zero-extends into x0.
    const size_t krCbnzPos = ipc.size();
    emit(ipc, cbnz_w(0, 0));  // patched below

    // Checkpoint 2: msgh_size < REPLY_SIZE_MIN (40) → abort kind=1.
    // ReplyMsg = mach_msg_header_t (24) + uint64_t result (8) +
    //            uint64_t some_flag (8).
    emit(ipc, ldr_w_offset(10, SP, 68));  // w10 = msgh_size
    emit(ipc, cmp_imm_w(10, REPLY_SIZE_MIN));
    const size_t sizeBltPos = ipc.size();
    emit(ipc, b_cond(COND_LT, 0));  // patched below

    // Checkpoint 3: msgh_id != expected → abort kind=2 (cross-talk).
    // Sidecar echoes the request msgh_id we sent into the reply; if a
    // different reply got delivered to our parentReplyName receive port
    // first (multi-threaded parent), the values differ.
    emit(ipc, ldr_w_offset(11, SP, 84));   // w11 = reply msgh_id
    emit(ipc, ldr_w_offset(12, SP, 128));  // w12 = expected (stashed pre-svc)
    emit(ipc, cmp_reg_w(11, 12));
    const size_t idBnePos = ipc.size();
    emit(ipc, b_cond(COND_NE, 0));  // patched below

    // Read some_flag into w9 — Step 1b moves the Some/None signal out of
    // msgh_id (which is now a transaction tag) into a dedicated body word
    // at sp+96.  0 = None (fall through to STASH), 1 = Some.
    emit(ipc, ldr_w_offset(9, SP, 96));  // w9 = some_flag
    // CBZ branch is `target = CBZ_addr + imm19*4`. We want to land at the
    // FIRST instruction of NONE path, which is one past the SOME path's
    // last (7th) instruction, i.e., 8 instructions ahead of CBZ.
    emit(ipc, cbz_w(9, 8));  // skip 7 some-path → none_label

    // ── SOME PATH ───────────────────────────────────────────────────────────
    emit(ipc, ldr_x_offset(0, SP, 88));      // x0 = result (replaces saved x0)
    emit(ipc, ldr_x_offset(1, SP, 8));       // restore x1
    emit(ipc, ldp_offset(2, 3, SP, 16));     // restore x2, x3
    emit(ipc, ldp_offset(4, 5, SP, 32));     // restore x4, x5
    emit(ipc, ldp_offset(16, LR, SP, 48));   // restore x16, lr
    emit(ipc, add_imm(SP, SP, FRAME_SIZE));  // sp += FRAME_SIZE
    emit(ipc, 0xD65F03C0U);                  // ret  (= BR x30)

    // ── NONE PATH ───────────────────────────────────────────────────────────
    // Sidecar declined the opcode. Restore caller registers and abs-jump
    // to STASH (stock's original 16-byte translate_insn prologue, which
    // tail-jumps to translate_insn+16). x0 comes from sp+0 — the SOME
    // path read from sp+88 (reply body), but for NONE we hand stock the
    // caller's untouched arguments.
    emit(ipc, ldr_x_offset(0, SP, 0));       // x0 = caller's TR*
    emit(ipc, ldr_x_offset(1, SP, 8));       // restore x1
    emit(ipc, ldp_offset(2, 3, SP, 16));     // restore x2, x3
    emit(ipc, ldp_offset(4, 5, SP, 32));     // restore x4, x5
    emit(ipc, ldp_offset(16, LR, SP, 48));   // restore x16, lr
    emit(ipc, add_imm(SP, SP, FRAME_SIZE));  // sp += FRAME_SIZE
    // Abs-jump to STASH (patched once we know the final ipc.size, after
    // the abort thunks below have been emitted).  STASH lives at the end
    // of the concatenated handler blob: filter | ipc | STASH | STASH_JUMP
    // | abort_routine | template.  Its address is
    //   handlerAddr + kFilterBytes + ipc.size_final.
    const size_t noneJumpPos = emit_abs_jump_placeholder(ipc);

    // ── Abort thunks ────────────────────────────────────────────────────────
    // Each thunk runs only when its checkpoint above branched here.  Sets
    // up x0 = diagnostic value, x1 = kind, x2 = insn_idx (loaded from
    // sp+32, where the prologue stashed caller's x4), then abs-jumps to
    // abort_routine.  Targets are patched after we know its address.
    //
    // Thunk 0: bad-kr.  x0 already holds kr (the value that triggered
    // cbnz_w); kind = 0.
    const size_t krThunkPos = ipc.size();
    emit(ipc, movz(1, 0, 0));            // x1 = 0 (kind)
    emit(ipc, ldr_x_offset(2, SP, 32));  // x2 = insn_idx
    const size_t krJumpPos = emit_abs_jump_placeholder(ipc);

    // Thunk 1: bad-size.  msgh_size sits in w10 (zero-extended into x10
    // by the ldr_w above); kind = 1.
    const size_t sizeThunkPos = ipc.size();
    emit(ipc, mov_reg_x(0, 10));         // x0 = x10 (diag = msgh_size)
    emit(ipc, movz(1, 1, 0));            // x1 = 1 (kind)
    emit(ipc, ldr_x_offset(2, SP, 32));  // x2 = insn_idx
    const size_t sizeJumpPos = emit_abs_jump_placeholder(ipc);

    // Thunk 2: id-mismatch.  Actual reply msgh_id is in w11; kind = 2.
    const size_t idThunkPos = ipc.size();
    emit(ipc, mov_reg_x(0, 11));         // x0 = x11 (diag = actual id)
    emit(ipc, movz(1, 2, 0));            // x1 = 2 (kind)
    emit(ipc, ldr_x_offset(2, SP, 32));  // x2 = insn_idx
    const size_t idJumpPos = emit_abs_jump_placeholder(ipc);

    // ── Resolve final addresses and patch placeholders ──────────────────────
    // ipc.size() is now final.  STASH starts at handlerAddr + kFilterBytes +
    // ipc.size_final; abort_routine starts at STASH_JUMP_END = STASH + 32.
    const uint64_t stashAddrFinal = handlerAddr + kFilterBytes + ipc.size();
    const uint64_t abortRoutineAddr = stashAddrFinal + 32;
    if ((stashAddrFinal | abortRoutineAddr) & 0xFFFF000000000000ULL) {
        blobs.entry.clear();
        return blobs;
    }

    // Patch NONE's abs-jump → STASH.
    patch_abs_jump_3movs(ipc, noneJumpPos, stashAddrFinal);

    // Patch checkpoint forward branches → their thunks.
    {
        const auto imm = static_cast<int32_t>((krThunkPos - krCbnzPos) / 4);
        patch_word(ipc, krCbnzPos, cbnz_w(0, imm));
    }
    {
        const auto imm = static_cast<int32_t>((sizeThunkPos - sizeBltPos) / 4);
        patch_word(ipc, sizeBltPos, b_cond(COND_LT, imm));
    }
    {
        const auto imm = static_cast<int32_t>((idThunkPos - idBnePos) / 4);
        patch_word(ipc, idBnePos, b_cond(COND_NE, imm));
    }

    // Patch each thunk's abs-jump → abort_routine.
    patch_abs_jump_3movs(ipc, krJumpPos, abortRoutineAddr);
    patch_abs_jump_3movs(ipc, sizeJumpPos, abortRoutineAddr);
    patch_abs_jump_3movs(ipc, idJumpPos, abortRoutineAddr);

    // ──── FILTER prologue ───────────────────────────────────────────────────
    // Bypass the IPC entirely for non-x87 opcodes — translate_insn fires for
    // EVERY x86 instruction Rosetta translates (~85k/test) but only ~14% of
    // them are x87. Without this filter every non-x87 call paid a Mach
    // round-trip just to be told "fall through to stock".
    //
    // The filter dereferences the IRInstr at &instr_array[insn_idx] (parent
    // memory; we run inside parent's address space) and reads its 16-bit
    // opcode field. If the opcode is outside both x87 ranges
    //   low:  0x25..0x30 = FCMOVcc + FCOMI variants
    //   high: 0xBD..0x10C = most x87 ops
    // it abs-jumps straight to STASH so the original translate_insn prologue
    // runs and stock handles the instruction itself — same outcome as an IPC
    // reply of None, but without the ~25 µs round-trip.
    //
    // Uses x9, x10, x11 — caller-saved scratch in AAPCS, so we don't need to
    // preserve them across the filter.
    // (kFilterInstrs / kFilterBytes are declared at the top of build() so
    // the IPC body's NONE path can compute the same stashAddr.)
    constexpr uint32_t kX87LoLo = 0x25;
    constexpr uint32_t kX87LoHi = 0x30;
    constexpr uint32_t kX87HiLo = 0xBD;
    constexpr uint32_t kX87HiHi = 0x10C;

    static_assert(sizeof(IRInstr) == 0x50, "stub filter assumes IRInstr stride 0x50");
    static_assert(offsetof(IRInstr, opcode) == 0x4, "stub filter assumes IRInstr.opcode at +4");

    // STASH lives right after the IPC body in the final blob — same address
    // we just computed above for the NONE path.
    const uint64_t stashAddr = stashAddrFinal;

    std::vector<uint8_t> filter;
    filter.reserve(kFilterBytes);
    //  0: movz w9, #0x50              ; sizeof(IRInstr) — fits in 16 bits
    emit(filter, movz(9, 0x50, 0));
    //  1: madd x10, x4, x9, x2        ; x10 = &instr_array[insn_idx]
    emit(filter, madd(10, 4, 9, 2));
    //  2: ldrh w9, [x10, #4]          ; w9 = uint16_t opcode
    emit(filter, ldrh_w_offset(9, 10, 4));
    //  3: sub w11, w9, #0x25
    emit(filter, sub_imm_w(11, 9, kX87LoLo));
    //  4: cmp w11, #0xB               ; in-low-range if w11 ≤ 0xB unsigned
    emit(filter, cmp_imm_w(11, kX87LoHi - kX87LoLo));
    //  5: b.ls do_ipc                 ; +8 instr → land at do_ipc (instr 13)
    emit(filter, b_cond(COND_LS, 8));
    //  6: sub w11, w9, #0xBD
    emit(filter, sub_imm_w(11, 9, kX87HiLo));
    //  7: cmp w11, #0x4F              ; in-high-range if w11 ≤ 0x4F unsigned
    emit(filter, cmp_imm_w(11, kX87HiHi - kX87HiLo));
    //  8: b.ls do_ipc                 ; +5 instr → land at do_ipc (instr 13)
    emit(filter, b_cond(COND_LS, 5));
    //  9..12: abs-jump to STASH (movz/movk/movk x16, $stashAddr; br x16)
    emit_abs_jump_3movs(filter, stashAddr);
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
