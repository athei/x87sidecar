#include "stub_asm.hpp"

#include <cstring>

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
    return 0xD2800000u | (hw << 21) | (uint32_t(imm) << 5) | (rd & 0x1F);
}

// MOVK Xd, #imm16, lsl #(hw*16)
//   sf=1, opc=11, 100101, hw, imm16, Rd
constexpr uint32_t movk(uint32_t rd, uint16_t imm, uint32_t lsl_shift) {
    uint32_t hw = lsl_shift / 16;
    return 0xF2800000u | (hw << 21) | (uint32_t(imm) << 5) | (rd & 0x1F);
}

// MOVN Xd, #imm16, lsl #(hw*16)   (move-with-NOT — used for negative immediates)
//   sf=1, opc=00, 100101, hw, imm16, Rd
constexpr uint32_t movn(uint32_t rd, uint16_t imm, uint32_t lsl_shift) {
    uint32_t hw = lsl_shift / 16;
    return 0x92800000u | (hw << 21) | (uint32_t(imm) << 5) | (rd & 0x1F);
}

// BR Xn  (unconditional branch via register)
//   1101_0110_0001_1111_0000_00_Rn_00000
constexpr uint32_t br(uint32_t rn) {
    return 0xD61F0000u | ((rn & 0x1F) << 5);
}

// SVC #imm16  (supervisor call — used with imm16=0x80 for syscalls on Darwin)
//   1101_0100_000_imm16_00001
constexpr uint32_t svc(uint16_t imm16) {
    return 0xD4000001u | (uint32_t(imm16) << 5);
}

// STP Xt1, Xt2, [Xn|SP, #imm7]!  (pre-index, 64-bit pair store)
//   When imm is signed 7-bit scaled by 8.  Encoding for pre-indexed:
//   10_101_0_011_0_imm7_Rt2_Rn_Rt1
constexpr uint32_t stp_preindex(uint32_t rt1, uint32_t rt2, uint32_t rn, int32_t imm) {
    int32_t scaled = imm / 8;
    uint32_t imm7 = uint32_t(scaled) & 0x7F;
    return 0xA9800000u | (imm7 << 15) | ((rt2 & 0x1F) << 10) | ((rn & 0x1F) << 5) |
           (rt1 & 0x1F);
}

// STP Xt1, Xt2, [Xn|SP, #imm]  (signed-offset, 64-bit pair store)
//   10_101_0_010_0_imm7_Rt2_Rn_Rt1
constexpr uint32_t stp_offset(uint32_t rt1, uint32_t rt2, uint32_t rn, int32_t imm) {
    int32_t scaled = imm / 8;
    uint32_t imm7 = uint32_t(scaled) & 0x7F;
    return 0xA9000000u | (imm7 << 15) | ((rt2 & 0x1F) << 10) | ((rn & 0x1F) << 5) |
           (rt1 & 0x1F);
}

// LDP Xt1, Xt2, [Xn|SP, #imm]  (signed-offset, 64-bit pair load)
//   10_101_0_010_1_imm7_Rt2_Rn_Rt1
constexpr uint32_t ldp_offset(uint32_t rt1, uint32_t rt2, uint32_t rn, int32_t imm) {
    int32_t scaled = imm / 8;
    uint32_t imm7 = uint32_t(scaled) & 0x7F;
    return 0xA9400000u | (imm7 << 15) | ((rt2 & 0x1F) << 10) | ((rn & 0x1F) << 5) |
           (rt1 & 0x1F);
}

// LDP Xt1, Xt2, [Xn|SP], #imm  (post-index, 64-bit pair load)
//   10_101_0_001_1_imm7_Rt2_Rn_Rt1
constexpr uint32_t ldp_postindex(uint32_t rt1, uint32_t rt2, uint32_t rn, int32_t imm) {
    int32_t scaled = imm / 8;
    uint32_t imm7 = uint32_t(scaled) & 0x7F;
    return 0xA8C00000u | (imm7 << 15) | ((rt2 & 0x1F) << 10) | ((rn & 0x1F) << 5) |
           (rt1 & 0x1F);
}

// STR Wt, [Xn|SP, #imm]  (32-bit store, unsigned offset)
//   10_111_0_01_00_imm12_Rn_Rt
constexpr uint32_t str_w_offset(uint32_t wt, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = (imm / 4) & 0xFFF;
    return 0xB9000000u | (imm12 << 10) | ((rn & 0x1F) << 5) | (wt & 0x1F);
}

// STR Xt, [Xn|SP, #imm]  (64-bit store, unsigned offset, scaled by 8)
//   11_111_0_01_00_imm12_Rn_Rt
constexpr uint32_t str_x_offset(uint32_t xt, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = (imm / 8) & 0xFFF;
    return 0xF9000000u | (imm12 << 10) | ((rn & 0x1F) << 5) | (xt & 0x1F);
}

// CBZ Xt, +imm19*4   (branch if zero — 64-bit). imm19 is signed PC-relative
//   in 4-byte units.
constexpr uint32_t cbz(uint32_t rt, int32_t imm19_words) {
    uint32_t imm19 = uint32_t(imm19_words) & 0x7FFFF;
    return 0xB4000000u | (imm19 << 5) | (rt & 0x1F);
}

// CBZ Wt, +imm19*4   (32-bit variant)
constexpr uint32_t cbz_w(uint32_t rt, int32_t imm19_words) {
    uint32_t imm19 = uint32_t(imm19_words) & 0x7FFFF;
    return 0x34000000u | (imm19 << 5) | (rt & 0x1F);
}

// LDR Xt, [Xn|SP, #imm]  (64-bit load, unsigned offset, scaled by 8)
//   11_111_0_01_01_imm12_Rn_Rt
constexpr uint32_t ldr_x_offset(uint32_t xt, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = (imm / 8) & 0xFFF;
    return 0xF9400000u | (imm12 << 10) | ((rn & 0x1F) << 5) | (xt & 0x1F);
}

// LDR Wt, [Xn|SP, #imm]  (32-bit load, unsigned offset, scaled by 4)
//   10_111_0_01_01_imm12_Rn_Rt
constexpr uint32_t ldr_w_offset(uint32_t wt, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = (imm / 4) & 0xFFF;
    return 0xB9400000u | (imm12 << 10) | ((rn & 0x1F) << 5) | (wt & 0x1F);
}

// BRK #imm16
constexpr uint32_t brk_imm(uint16_t imm) {
    return 0xD4200000u | (uint32_t(imm) << 5);
}

// ADD Xd, Xn, #imm12  (signed-offset, no shift, 64-bit add immediate)
//   sf=1, op=0, S=0, 10001, sh=0, imm12, Rn, Rd
constexpr uint32_t add_imm(uint32_t rd, uint32_t rn, uint32_t imm) {
    uint32_t imm12 = imm & 0xFFF;
    return 0x91000000u | (imm12 << 10) | ((rn & 0x1F) << 5) | (rd & 0x1F);
}

// 32-bit register encoder helpers — only differ from 64-bit by sf=0 in
// some contexts.  We don't need them yet (mach_msg uses 64-bit GPRs).

constexpr uint32_t SP = 31;
constexpr uint32_t LR = 30;

// ──── helpers ────────────────────────────────────────────────────────────────

void emit(std::vector<uint8_t>& out, uint32_t insn) {
    out.push_back(uint8_t(insn));
    out.push_back(uint8_t(insn >> 8));
    out.push_back(uint8_t(insn >> 16));
    out.push_back(uint8_t(insn >> 24));
}

// movz/movk/movk/movk to load a 64-bit address into Xd (4 instructions).
void emit_load_imm64(std::vector<uint8_t>& out, uint32_t rd, uint64_t imm) {
    emit(out, movz(rd, uint16_t(imm & 0xFFFF), 0));
    emit(out, movk(rd, uint16_t((imm >> 16) & 0xFFFF), 16));
    emit(out, movk(rd, uint16_t((imm >> 32) & 0xFFFF), 32));
    emit(out, movk(rd, uint16_t((imm >> 48) & 0xFFFF), 48));
}

// Build a 16-byte abs-jump to `target` via x16 (3 mov + 1 br = 16 bytes).
// The high 16 bits are assumed zero, which is true for kernel-managed
// userland virtual addresses on macOS (top byte zeroed; canonical sub-256TB).
void emit_abs_jump_3movs(std::vector<uint8_t>& out, uint64_t target) {
    emit(out, movz(16, uint16_t(target & 0xFFFF), 0));
    emit(out, movk(16, uint16_t((target >> 16) & 0xFFFF), 16));
    emit(out, movk(16, uint16_t((target >> 32) & 0xFFFF), 32));
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

    auto& h = blobs.handler;

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
    emit(h, stp_preindex(0, 1, SP, -FRAME_SIZE));
    // stp x2, x3, [sp, #16]
    emit(h, stp_offset(2, 3, SP, 16));
    // stp x4, x5, [sp, #32]
    emit(h, stp_offset(4, 5, SP, 32));
    // stp x16, lr, [sp, #48]
    emit(h, stp_offset(16, LR, SP, 48));

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
    constexpr uint32_t MSG_BITS = 0x13u | (0x15u << 8);  // = 0x1513
    constexpr uint32_t MSG_SIZE = 24 + 40;                // header + 5×8 args
    constexpr uint32_t RCV_SIZE = 128;                    // reply cap
    constexpr uint32_t MSG_ID   = 0x10000001;             // sidecar dispatches on it

    // Use x9 as scratch for header field values.

    // msgh_bits — 0x1513 fits in 16 bits (single movz)
    emit(h, movz(9, MSG_BITS, 0));
    emit(h, str_w_offset(9, SP, 64));         // [sp+64]

    // msgh_size
    emit(h, movz(9, MSG_SIZE, 0));
    emit(h, str_w_offset(9, SP, 68));         // [sp+68]

    // msgh_remote_port = sidecarReqName (32-bit)
    emit_load_imm64(h, 9, sidecarReqName);    // 4 instructions
    emit(h, str_w_offset(9, SP, 72));         // [sp+72]

    // msgh_local_port = parentReplyName (32-bit) — kernel auto-derives
    // a SEND_ONCE right via MAKE_SEND_ONCE in MSG_BITS.local.
    emit_load_imm64(h, 9, parentReplyName);   // 4 instructions
    emit(h, str_w_offset(9, SP, 76));         // [sp+76]

    // msgh_voucher_port = 0
    emit(h, movz(9, 0, 0));
    emit(h, str_w_offset(9, SP, 80));         // [sp+80]

    // msgh_id
    emit(h, movz(9, uint16_t(MSG_ID & 0xFFFF), 0));
    emit(h, movk(9, uint16_t((MSG_ID >> 16) & 0xFFFF), 16));
    emit(h, str_w_offset(9, SP, 84));         // [sp+84]

    // ── Body: five translate_insn args (still in x0..x4 at this point) ──────
    // Header lives at sp+64 .. sp+88. Body lives at sp+88 .. sp+128.
    emit(h, str_x_offset(0, SP, 88));         // body[+0]  = x0 = TR*
    emit(h, str_x_offset(1, SP, 96));         // body[+8]  = x1 = block*
    emit(h, str_x_offset(2, SP, 104));        // body[+16] = x2 = instr_array*
    emit(h, str_x_offset(3, SP, 112));        // body[+24] = x3 = num_instrs
    emit(h, str_x_offset(4, SP, 120));        // body[+32] = x4 = insn_idx

    // ── mach_msg_trap arguments ──────────────────────────────────────────────
    //   x0  = msg pointer (sp + 64)
    //   x1  = options = MACH_SEND_MSG | MACH_RCV_MSG = 0x3
    //   x2  = send_size = 64
    //   x3  = rcv_size  = 128
    //   x4  = rcv_name  = parentReplyName  (low 32 bits)
    //   x5  = timeout   = 0  (block forever)
    //   x6  = notify    = 0
    //   x16 = -31  (mach_msg_trap)
    emit(h, add_imm(0, SP, 64));              // x0 = sp + 64
    emit(h, movz(1, 0x0003, 0));              // x1 = SEND | RCV
    emit(h, movz(2, MSG_SIZE, 0));            // x2 = send_size
    emit(h, movz(3, RCV_SIZE, 0));            // x3 = rcv_size
    emit_load_imm64(h, 4, parentReplyName);   // x4 = rcv_name
    emit(h, movz(5, 0, 0));                   // x5 = timeout
    emit(h, movz(6, 0, 0));                   // x6 = notify
    emit(h, movn(16, 30, 0));                 // x16 = -31 (mach_msg_trap)
    emit(h, svc(0x80));

    // ── Reply parsing ───────────────────────────────────────────────────────
    // After mach_msg returns:
    //   x0 holds the kern_return_t (we ignore non-zero — Wine will hang
    //        otherwise but for now treat it as fall-through implicitly via
    //        the message buffer's leftover msgh_id which is whatever).
    //   The buffer at sp+64..sp+192 holds the reply: header at +64..+88,
    //                                                body at +88..end.
    //   msgh_id (sp+84) tells us what to do: 0 = None (fall through),
    //                                        1 = Some, body[0] = result.
    //
    // Read msgh_id into w9.
    emit(h, ldr_w_offset(9, SP, 84));         // w9 = msgh_id
    // CBZ branch is `target = CBZ_addr + imm19*4`. We want to land at the
    // FIRST instruction of NONE path, which is one past the SOME path's
    // last (7th) instruction, i.e., 8 instructions ahead of CBZ.
    emit(h, cbz_w(9, 8));                     // skip 7 some-path → none_label

    // ── SOME PATH ───────────────────────────────────────────────────────────
    emit(h, ldr_x_offset(0, SP, 88));         // x0 = result (replaces saved x0)
    emit(h, ldr_x_offset(1, SP, 8));          // restore x1
    emit(h, ldp_offset(2, 3, SP, 16));        // restore x2, x3
    emit(h, ldp_offset(4, 5, SP, 32));        // restore x4, x5
    emit(h, ldp_offset(16, LR, SP, 48));      // restore x16, lr
    emit(h, add_imm(SP, SP, FRAME_SIZE));     // sp += FRAME_SIZE
    emit(h, 0xD65F03C0u);                     // ret  (= BR x30)

    // ── NONE PATH ───────────────────────────────────────────────────────────
    // Restore caller regs then fall through to STASH below.
    emit(h, ldp_offset(16, LR, SP, 48));
    emit(h, ldp_offset(4, 5, SP, 32));
    emit(h, ldp_offset(2, 3, SP, 16));
    emit(h, ldp_postindex(0, 1, SP, FRAME_SIZE));

    // ── STASH (4 instructions = original 16 bytes of translate_insn) ───────
    h.insert(h.end(), origPrologue16, origPrologue16 + 16);

    // ── STASH_JUMP (16 bytes abs-jump to translate_insn + 16) ──────────────
    emit_abs_jump_3movs(h, translateInsnAddr + 16);

    return blobs;
}

}  // namespace stub_asm
