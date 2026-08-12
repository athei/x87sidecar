#pragma once

#include <cstdint>
#include <vector>

// Builds the inline IPC stub blobs that get COW-written into stock
// libRosettaRuntime's __TEXT.
//
// Layout summary (see plan/architecture):
//   stock translate_insn[0..16]:
//       16-byte abs-jump to OUR_HANDLER  (movz/movk/movk x16 + br x16)
//   trailing padding region of __TEXT segment:
//       OUR_HANDLER  (~60 instr): saves caller regs, packs the 5×8 body
//                                 args onto the stack, then issues
//                                 mach_msg2_trap (svc -47) with
//                                 MACH64_SEND_MQ_CALL set in options64
//                                 for SEND|RCV.  After reply, branches
//                                 on some_flag:
//                                   1 (Some) → load body[0] into x0,
//                                              restore, ret.
//                                   0 (None) → restore, fall through
//                                              to STASH below.
//       STASH        (4 instr):   copy of translate_insn[0..16] original bytes.
//       STASH_JUMP   (4 instr):   abs-jump to translate_insn+16.
//
// Total bytes needed in trailing padding = sizeof(handler) + 16 + 16.
// All produced bytes are arm64 instructions encoded little-endian.
namespace stub_asm {

struct StubBlobs {
    // 16 bytes; written to translate_insn[0..16].
    std::vector<uint8_t> entry;

    // OUR_HANDLER + STASH + STASH_JUMP, contiguous; written to the
    // trailing-padding location in libRosettaRuntime's __TEXT.
    std::vector<uint8_t> handler;
};

// Build the blob bytes.
//   handlerAddr        : absolute address of where `handler` will be written
//                         (i.e., where OUR_HANDLER will live in the parent)
//   translateInsnAddr  : absolute address of stock translate_insn entry
//   origPrologue16     : the 16 bytes currently at translate_insn[0..16],
//                         to be preserved verbatim into STASH
//   sidecarReqName     : Mach SEND-right name, in the parent's namespace,
//                         pointing at the loader-process service port.
//                         Stub uses this as msgh_remote_port (COPY_SEND).
//   parentReplyName    : Mach RECEIVE-right name, in the parent's namespace.
//                         Stub uses it as msgh_local_port with
//                         MAKE_SEND_ONCE; sidecar replies on the resulting
//                         send-once and the reply lands here.
StubBlobs build(uint64_t handlerAddr, uint64_t translateInsnAddr, const uint8_t origPrologue16[16],
                uint32_t sidecarReqName, uint32_t parentReplyName);

// ──── decode_opcode hook ─────────────────────────────────────────────────────
//
// A second, much smaller stub on the stage before translate_insn.  Rosetta's
// decoder rejects two encodings that real hardware runs, so neither ever
// becomes an IRInstr and neither can reach the translate_insn hook — they raise
// illegal-instruction traps instead.  Both are what winerosetta.dll was fixing
// up from inside the guest, and both occur in WoW 1.12:
//
//   DC D8   the undocumented alias of `fcomp st(0)`, at .text:006FA876 in
//           luaH_set's "table index is NaN" check.  D8 D8 is the same
//           instruction with the same length, so a pure substitution suffices
//           and stock translates the result correctly.
//   63 /r   ARPL r/m16, r16, legacy-mode only (0x63 is MOVSXD in 64-bit mode,
//           which is why Rosetta's tables have no ARPL).  Seen as `63 D0` in
//           downloaded, obfuscated code.  Nothing else means the same thing, so
//           this borrows 0x01 (ADD r/m32, r32), whose ModRM/SIB/disp encoding
//           is byte-identical, and then forces DecodedInsn::mnemonic to our
//           synthetic ARPL id so it reaches TranslatorCustom::translate_arpl
//           rather than being emitted as a real ADD.
//
// Neither needs IPC: the stub decides in the tracee and never talks to us.
//
//   handler:  call the original through STASH.  If it did not return INVALID,
//             return that verbatim.  Otherwise build the substitute, point
//             ctx->code_base / code_end at it, call the original again at
//             offset 0, put back code_base / code_end / insn_start / cursor,
//             apply any mnemonic fixup, and return the substitute's result.
//             Anything else INVALID is handed back untouched.
//   STASH:      copy of decode_opcode[0..16].
//   STASH_JUMP: abs-jump to decode_opcode+16.
//
// Calling the original through STASH rather than through the patched entry is
// what keeps the second call from re-entering the hook.
//
// The substitute is built on the handler's OWN STACK FRAME, which is per-thread
// and always mapped.  That is deliberate and was learned the hard way: an
// earlier version put it, plus a pair of diagnostic counters, on a page the
// sidecar allocated and mach_vm_remap'd into the tracee with VM_INHERIT_NONE.
// It passed every test binary and wedged wine, because wine forks and a forked
// child inherited the patched runtime (an image mapping, so it is inherited)
// but not that page, faulting once per decode.  Anything the handler touches
// has to be reachable in every process that inherits the patch.

// Byte offsets of the two DecoderCtx fields the handler swaps.  Verified
// against the local runtime from decode_opcode's own prologue, which does
//   str xzr, [x24, #0x28]!   ; x24 = ctx + 0x28, zeroing ctx->fault
//   ldp x11, x8, [x24, #-0x20]   ; code_base, code_end
constexpr uint32_t kDecoderCtxCodeBase = 0x08;
constexpr uint32_t kDecoderCtxCodeEnd = 0x10;
// The decoder also seeds these two from code_base + offset on entry:
//   add x9, x11, w1, uxtw ; stp x9, x9, [x24, #-0x10]
// so after a substitute decode they point into the scratch buffer. They must be
// put back to what an in-place decode would have left, or anything deriving the
// instruction's guest address from them gets a garbage pc.
constexpr uint32_t kDecoderCtxInsnStart = 0x18;
constexpr uint32_t kDecoderCtxCursor = 0x20;

//   handlerAddr       : absolute address the returned `handler` blob will live at
//   decodeOpcodeAddr  : absolute address of stock decode_opcode
//   origPrologue16    : the 16 bytes currently at decode_opcode[0..16]
//   hostArplOpcode    : the synthetic ARPL id in the runtime's host numbering,
//                        forced into DecodedInsn::mnemonic after a borrowed ADD
StubBlobs buildDecodeHook(uint64_t handlerAddr, uint64_t decodeOpcodeAddr,
                          const uint8_t origPrologue16[16], uint16_t hostArplOpcode);

}  // namespace stub_asm
