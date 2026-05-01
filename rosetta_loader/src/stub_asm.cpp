#include "stub_asm.hpp"

#include <cstddef>
#include <cstring>

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
    return 0xA9800000U | (imm7 << 15) | ((rt2 & 0x1F) << 10) | ((rn & 0x1F) << 5) |
           (rt1 & 0x1F);
}

// STP Xt1, Xt2, [Xn|SP, #imm]  (signed-offset, 64-bit pair store)
//   10_101_0_010_0_imm7_Rt2_Rn_Rt1
constexpr uint32_t stp_offset(uint32_t rt1, uint32_t rt2, uint32_t rn, int32_t imm) {
    int32_t scaled = imm / 8;
    uint32_t imm7 = static_cast<uint32_t>(scaled) & 0x7F;
    return 0xA9000000U | (imm7 << 15) | ((rt2 & 0x1F) << 10) | ((rn & 0x1F) << 5) |
           (rt1 & 0x1F);
}

// LDP Xt1, Xt2, [Xn|SP, #imm]  (signed-offset, 64-bit pair load)
//   10_101_0_010_1_imm7_Rt2_Rn_Rt1
constexpr uint32_t ldp_offset(uint32_t rt1, uint32_t rt2, uint32_t rn, int32_t imm) {
    int32_t scaled = imm / 8;
    uint32_t imm7 = static_cast<uint32_t>(scaled) & 0x7F;
    return 0xA9400000U | (imm7 << 15) | ((rt2 & 0x1F) << 10) | ((rn & 0x1F) << 5) |
           (rt1 & 0x1F);
}

// LDP Xt1, Xt2, [Xn|SP], #imm  (post-index, 64-bit pair load)
//   10_101_0_001_1_imm7_Rt2_Rn_Rt1
constexpr uint32_t ldp_postindex(uint32_t rt1, uint32_t rt2, uint32_t rn, int32_t imm) {
    int32_t scaled = imm / 8;
    uint32_t imm7 = static_cast<uint32_t>(scaled) & 0x7F;
    return 0xA8C00000U | (imm7 << 15) | ((rt2 & 0x1F) << 10) | ((rn & 0x1F) << 5) |
           (rt1 & 0x1F);
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

// B.cond +imm19*4   (signed PC-relative conditional branch, 4-byte units).
//   0101_0100_imm19_0_cond
constexpr uint32_t b_cond(uint32_t cond, int32_t imm19_words) {
    uint32_t imm19 = static_cast<uint32_t>(imm19_words) & 0x7FFFF;
    return 0x54000000U | (imm19 << 5) | (cond & 0xF);
}
constexpr uint32_t COND_LS = 0x9;  // unsigned ≤

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
    return 0x9B000000U | ((rm & 0x1F) << 16) | ((ra & 0x1F) << 10) |
           ((rn & 0x1F) << 5) | (rd & 0x1F);
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

// Build a 16-byte abs-jump to `target` via x16 (3 mov + 1 br = 16 bytes).
// The high 16 bits are assumed zero, which is true for kernel-managed
// userland virtual addresses on macOS (top byte zeroed; canonical sub-256TB).
void emit_abs_jump_3movs(std::vector<uint8_t>& out, uint64_t target) {
    emit(out, movz(16, static_cast<uint16_t>(target & 0xFFFF), 0));
    emit(out, movk(16, static_cast<uint16_t>((target >> 16) & 0xFFFF), 16));
    emit(out, movk(16, static_cast<uint16_t>((target >> 32) & 0xFFFF), 32));
    emit(out, br(16));
}

}  // namespace

// ──── public ────────────────────────────────────────────────────────────────

StubBlobs build(uint64_t handlerAddr, uint64_t translateInsnAddr,
                const uint8_t origPrologue16[16], uint32_t sidecarReqName,
                uint32_t parentReplyName) {
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
    constexpr size_t kFilterBytes  = kFilterInstrs * 4;

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
    //   sp[ 64..192] : 128-byte buffer used for both send (msg ~= 64 B) and
    //                  receive (kernel writes reply here on RCV).
    //                  Header (24 B) at sp+64..sp+88.
    //                  Send body (40 B) at sp+88..sp+128.
    //                  Reply body lands at sp+88..sp+(64+rcv_size).
    //
    // After mach_msg(SEND|RCV) returns:
    //   x0 = KERN_RETURN. Reply is in the buffer.
    //   We read msgh_id (sp+84): 0 = None (fall through), 1 = Some(N).
    //   For Some, body[0] (sp+88) is the int64_t value to return.

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
    constexpr uint32_t MSG_SIZE = 24 + 40;                // header + 5×8 args
    constexpr uint32_t RCV_SIZE = 128;                    // reply cap
    constexpr uint32_t MSG_ID   = 0x10000001;             // sidecar dispatches on it

    // Use x9 as scratch for header field values.

    // msgh_bits — 0x1513 fits in 16 bits (single movz)
    emit(ipc, movz(9, MSG_BITS, 0));
    emit(ipc, str_w_offset(9, SP, 64));         // [sp+64]

    // msgh_size
    emit(ipc, movz(9, MSG_SIZE, 0));
    emit(ipc, str_w_offset(9, SP, 68));         // [sp+68]

    // msgh_remote_port = sidecarReqName (32-bit)
    emit_load_imm64(ipc, 9, sidecarReqName);    // 4 instructions
    emit(ipc, str_w_offset(9, SP, 72));         // [sp+72]

    // msgh_local_port = parentReplyName (32-bit) — kernel auto-derives
    // a SEND_ONCE right via MAKE_SEND_ONCE in MSG_BITS.local.
    emit_load_imm64(ipc, 9, parentReplyName);   // 4 instructions
    emit(ipc, str_w_offset(9, SP, 76));         // [sp+76]

    // msgh_voucher_port = 0
    emit(ipc, movz(9, 0, 0));
    emit(ipc, str_w_offset(9, SP, 80));         // [sp+80]

    // msgh_id
    emit(ipc, movz(9, static_cast<uint16_t>(MSG_ID & 0xFFFF), 0));
    emit(ipc, movk(9, static_cast<uint16_t>((MSG_ID >> 16) & 0xFFFF), 16));
    emit(ipc, str_w_offset(9, SP, 84));         // [sp+84]

    // ── Body: five translate_insn args (still in x0..x4 at this point) ──────
    // Header lives at sp+64 .. sp+88. Body lives at sp+88 .. sp+128.
    emit(ipc, str_x_offset(0, SP, 88));         // body[+0]  = x0 = TR*
    emit(ipc, str_x_offset(1, SP, 96));         // body[+8]  = x1 = block*
    emit(ipc, str_x_offset(2, SP, 104));        // body[+16] = x2 = instr_array*
    emit(ipc, str_x_offset(3, SP, 112));        // body[+24] = x3 = num_instrs
    emit(ipc, str_x_offset(4, SP, 120));        // body[+32] = x4 = insn_idx

    // ── mach_msg_trap arguments ──────────────────────────────────────────────
    //   x0  = msg pointer (sp + 64)
    //   x1  = options = MACH_SEND_MSG | MACH_RCV_MSG = 0x3
    //   x2  = send_size = 64
    //   x3  = rcv_size  = 128
    //   x4  = rcv_name  = parentReplyName  (low 32 bits)
    //   x5  = timeout   = 0  (block forever)
    //   x6  = notify    = 0
    //   x16 = -31  (mach_msg_trap)
    emit(ipc, add_imm(0, SP, 64));              // x0 = sp + 64
    emit(ipc, movz(1, 0x0003, 0));              // x1 = SEND | RCV
    emit(ipc, movz(2, MSG_SIZE, 0));            // x2 = send_size
    emit(ipc, movz(3, RCV_SIZE, 0));            // x3 = rcv_size
    emit_load_imm64(ipc, 4, parentReplyName);   // x4 = rcv_name
    emit(ipc, movz(5, 0, 0));                   // x5 = timeout
    emit(ipc, movz(6, 0, 0));                   // x6 = notify
    emit(ipc, movn(16, 30, 0));                 // x16 = -31 (mach_msg_trap)
    emit(ipc, svc(0x80));

    // ── Reply parsing ───────────────────────────────────────────────────────
    // After mach_msg returns:
    //   x0 holds the kern_return_t (we ignore non-zero — Wine will hang
    //        otherwise but for now treat it as fall-through implicitly via
    //        the message buffer's leftover msgh_id which is whatever).
    //   The buffer at sp+64..sp+192 holds the reply: header at +64..+88,
    //                                                body at +88..end.
    //   msgh_id (sp+84) tells us what to do: 0 = None (fall through to
    //                                            stock's translate_insn),
    //                                        1 = Some, body[0] = result.
    // The NONE path was briefly an _exit(133) trap while helper-using
    // x87 opcodes (transcendentals) lacked inline handlers — composition
    // of our emit with stock's helper-call emit broke on the
    // {x22, w23} f80 ABI mismatch. With every helper-using opcode now
    // inlined, the only path that ever returns nullopt is the deliberate
    // memory-block / NOP-class fall-through set:
    //   fxsave, fxrstor, fnop, fdisi, feni, fclex, finit, fldenv, fstenv.
    // All share the same property: stock's emit is pure block memory
    // I/O via x22 = X87State* (or zero instructions for the NOP family),
    // no helper-call ABI to clash with our cache.  is_handled_x87 stops
    // the run before each one, so x87_end has already flushed deferred
    // state to memory by the time stock takes over.  NONE restores
    // caller registers and abs-jumps to STASH, the same fall-through
    // path the FILTER prologue uses for non-x87 opcodes.
    //
    // Read msgh_id into w9.
    emit(ipc, ldr_w_offset(9, SP, 84));         // w9 = msgh_id
    // CBZ branch is `target = CBZ_addr + imm19*4`. We want to land at the
    // FIRST instruction of NONE path, which is one past the SOME path's
    // last (7th) instruction, i.e., 8 instructions ahead of CBZ.
    emit(ipc, cbz_w(9, 8));                     // skip 7 some-path → none_label

    // ── SOME PATH ───────────────────────────────────────────────────────────
    emit(ipc, ldr_x_offset(0, SP, 88));         // x0 = result (replaces saved x0)
    emit(ipc, ldr_x_offset(1, SP, 8));          // restore x1
    emit(ipc, ldp_offset(2, 3, SP, 16));        // restore x2, x3
    emit(ipc, ldp_offset(4, 5, SP, 32));        // restore x4, x5
    emit(ipc, ldp_offset(16, LR, SP, 48));      // restore x16, lr
    emit(ipc, add_imm(SP, SP, FRAME_SIZE));     // sp += FRAME_SIZE
    emit(ipc, 0xD65F03C0U);                     // ret  (= BR x30)

    // ── NONE PATH ───────────────────────────────────────────────────────────
    // Sidecar declined the opcode. Restore caller registers and abs-jump
    // to STASH (stock's original 16-byte translate_insn prologue, which
    // tail-jumps to translate_insn+16). x0 comes from sp+0 — the SOME
    // path read from sp+88 (reply body), but for NONE we hand stock the
    // caller's untouched arguments.
    emit(ipc, ldr_x_offset(0, SP, 0));          // x0 = caller's TR*
    emit(ipc, ldr_x_offset(1, SP, 8));          // restore x1
    emit(ipc, ldp_offset(2, 3, SP, 16));        // restore x2, x3
    emit(ipc, ldp_offset(4, 5, SP, 32));        // restore x4, x5
    emit(ipc, ldp_offset(16, LR, SP, 48));      // restore x16, lr
    emit(ipc, add_imm(SP, SP, FRAME_SIZE));     // sp += FRAME_SIZE
    // Abs-jump to STASH. STASH lives at the end of the concatenated
    // handler blob (filter + ipc + STASH + STASH_JUMP), so its address
    // is handlerAddr + kFilterBytes + ipc.size() once `ipc` is final.
    // The 4-instruction abs-jump we're about to emit is the IPC body's
    // final tail, so at this point in the build, the FINAL ipc.size()
    // will be (current size) + 16. The same value is recomputed at
    // line ~453 below for the filter's bypass jump — they target the
    // same address.
    const uint64_t noneStashAddr =
        handlerAddr + kFilterBytes + ipc.size() + 16;
    emit_abs_jump_3movs(ipc, noneStashAddr);

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
    constexpr uint32_t kX87LoLo    = 0x25;
    constexpr uint32_t kX87LoHi    = 0x30;
    constexpr uint32_t kX87HiLo    = 0xBD;
    constexpr uint32_t kX87HiHi    = 0x10C;

    static_assert(sizeof(IRInstr) == 0x50,
                  "stub filter assumes IRInstr stride 0x50");
    static_assert(offsetof(IRInstr, opcode) == 0x4,
                  "stub filter assumes IRInstr.opcode at +4");

    // STASH lives right after the IPC body in the final blob.
    const uint64_t stashAddr = handlerAddr + kFilterBytes + ipc.size();
    if (stashAddr & 0xFFFF000000000000ULL) {
        blobs.entry.clear();
        return blobs;
    }

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

    // ──── Concatenate: filter + IPC + STASH + STASH_JUMP ───────────────────
    auto& h = blobs.handler;
    h.reserve(filter.size() + ipc.size() + 16 + 16);
    h.insert(h.end(), filter.begin(), filter.end());
    h.insert(h.end(), ipc.begin(),    ipc.end());
    // STASH (4 instructions = original 16 bytes of translate_insn).
    h.insert(h.end(), origPrologue16, origPrologue16 + 16);
    // STASH_JUMP (abs-jump to translate_insn + 16).
    emit_abs_jump_3movs(h, translateInsnAddr + 16);

    return blobs;
}


}  // namespace stub_asm
