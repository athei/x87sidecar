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

}  // namespace stub_asm
