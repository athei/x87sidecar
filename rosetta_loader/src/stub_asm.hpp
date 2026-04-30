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
//       OUR_HANDLER  (~60 instr): saves caller regs, builds mach_msg header
//                                 + 5×8 args body on stack, svc into
//                                 mach_msg_trap with SEND|RCV. After reply,
//                                 branches on msgh_id:
//                                   id == 1 (Some) → load body[0] into x0,
//                                                    restore, ret.
//                                   id == 0 (None) → restore, fall through
//                                                    to STASH below.
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
StubBlobs build(uint64_t handlerAddr, uint64_t translateInsnAddr,
                const uint8_t origPrologue16[16], uint32_t sidecarReqName,
                uint32_t parentReplyName);

// Build the runtime transcendental-IPC trampoline bytes.
//
// Called from JIT-emitted code via `BLR x16` after loading the absolute
// address of this trampoline into x16.  Performs a Mach IPC roundtrip to
// the sidecar with msgh_id=0x10000002 carrying (opcode_tag, ST(0), ST(1));
// receives back two doubles (out0, out1).
//
// Calling convention:
//   On entry  : x0=opcode_tag, d0=ST(0), d1=ST(1) (or undefined for 1-input ops)
//   On return : d0=result1, d1=result2 (only meaningful for fsincos)
//   Preserves : x19..x28, d8..d15, x29, x30, sp (AAPCS64 callee-saved)
//   Clobbers  : x0..x18, d0..d7 (except as outputs), d16..d31
//
// Lives in libRosettaRuntime's __TEXT trailing pad so the parent's PC
// stays inside Rosetta-registered pages — same panic-class fix as the
// translate-time stub.
//
// Returns the raw instruction bytes; caller installs them at a chosen
// absolute address in the parent and exposes that address to the sidecar
// (see rosetta_core::set_transcendental_helper_addr).
std::vector<uint8_t> buildTranscendentalHelper(uint32_t sidecarReqName,
                                                uint32_t parentReplyName);

}  // namespace stub_asm
