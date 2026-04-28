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
//       OUR_HANDLER  (~64 instr): builds mach_msg header on stack, sends it,
//                                 falls through to STASH unconditionally for M2.
//       STASH        (4 instr):   copy of translate_insn[0..16] original bytes.
//       STASH_JUMP   (4 instr):   abs-jump to translate_insn+16.
//
// Sizes:
//   16-byte stub at translate_insn[0..16].
//   Total bytes needed in trailing padding = sizeof(handler) + 16 + 16.
//
// All the produced bytes are arm64 instructions encoded little-endian.
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
//   sidecarPortName    : Mach send-right name, in the parent's namespace,
//                         pointing at the loader-process service port
StubBlobs build(uint64_t handlerAddr, uint64_t translateInsnAddr,
                const uint8_t origPrologue16[16], uint32_t sidecarPortName);

}  // namespace stub_asm
