#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// Everything the loader patches or reads inside Rosetta is located here, from
// the two on-disk images, before anything is launched:
//
//   /usr/libexec/rosetta/runtime                     (determineOffsets)
//   /Library/Apple/usr/libexec/oah/libRosettaRuntime (determineRuntimeOffsets)
//
// Both images are position independent with __TEXT at vmaddr 0, so a file
// offset is also the offset from the live image base.
//
// Functions are found by the anchors that survive a rebuild: the export table
// names `translator_translate`, every function names itself in the string it
// hands to Rosetta's assert helper, and the opcode mnemonic table is laid out
// in enum order. Byte-pattern signatures of prologues remain as the fallback
// and as a cross-check. Whatever is found is then validated structurally: the
// hook on decode_opcode, for instance, is only installed if the function still
// keeps the DecoderCtx fields where the hook writes them.
struct OffsetFinder {
    auto determineOffsets() -> bool;
    auto determineRuntimeOffsets() -> bool;

    // /usr/libexec/rosetta/runtime
    std::uint64_t offsetExportsFetch_ = 0;
    std::uint64_t offsetSvcCallEntry_ = 0;
    std::uint64_t offsetSvcCallRet_ = 0;
    std::uint64_t offsetDisableAot_ = 0;
    // Root of the ARM-keyed code-fragment tree the sampler walks, from the
    // lookup that panics with "cannot locate code fragment for arm address".
    // 0 when not located (the sampler keeps its built-in value).
    std::uint64_t armTreeRootOffset_ = 0;

    // libRosettaRuntime
    std::uint64_t offsetTransactionResultSize_ = 0;
    std::uint64_t offsetTranslateInsn_ = 0;
    std::uint64_t offsetInitLibrary_ = 0;
    std::uint64_t offsetTranslatorTranslate_ = 0;  // exported by name
    std::uint64_t offsetDecodeOpcode_ = 0;         // 0 when not located or not hookable
    // Size of the TranslationResult stock allocates, read from the immediate
    // translator_translate hands its allocator. 0 when not found.
    std::uint32_t translationResultSize_ = 0;
    // The first 16 bytes of each hooked function as they are on disk; the live
    // bytes must match before a hook is written over them.
    std::array<std::uint8_t, 16> translateInsnPrologue_{};
    std::array<std::uint8_t, 16> decodeOpcodePrologue_{};
    // Host opcode id -> mnemonic, from the runtime's own table. Empty when the
    // table was not found.
    std::vector<std::string> opcodeNames_;

    // Exports.version, read from the on-disk runtime by determineRuntimeOffsets.
    // Seeds the OpcodeCompatibility layer (26.4<->26.5) without needing the live
    // Exports struct (X19), so both attach modes get it the same way.
    std::uint64_t runtimeVersion_ = 0;

    // translate_insn's 36-byte prologue signature (stp/sub-sp/mov prologue).
    // Fallback locator, and what identifies the image in live memory.
    static constexpr std::array<std::uint8_t, 36> kTranslateInsnPattern = {
        0xFF, 0x43, 0x03, 0xD1, 0xFC, 0x6F, 0x07, 0xA9, 0xfa, 0x67, 0x08, 0xa9,
        0xF8, 0x5F, 0x09, 0xA9, 0xF6, 0x57, 0x0A, 0xA9, 0xF4, 0x4F, 0x0B, 0xA9,
        0xFD, 0x7B, 0x0C, 0xA9, 0xFD, 0x03, 0x03, 0x91, 0xF3, 0x03, 0x00, 0xAA};

    // decode_opcode's prologue signatures, the fallback locator.  Rosetta's
    // single-instruction x86 decoder:
    //   int decode_opcode(DecoderCtx*, unsigned offset, DecodedInsn* out,
    //                     uint8_t* out_len)   -> 0=ok, 1=invalid, 2=fault
    //
    // The first 32 bytes are a generic 0x70-frame prologue that matches some
    // seventy places in the runtime, so a signature has to run through what
    // follows.  Two layouts exist, both starting with the same 32 bytes and
    // both ending in `str xzr, [x24, #0x28]!` (zeroing ctx->fault, which is
    // also what pins DecoderCtx's layout):
    //
    //   V1 (runtime version 0x16f0140000000): `str x3, [sp]` (stashing
    //      out_len), `mov x24, x0`, `str xzr, [x24, #0x28]!`.
    //   V2 (runtime version 0x16f0200000000): `mov x24, x0`,
    //      `str xzr, [x24, #0x28]!`; the out_len stash comes later.
    static constexpr std::array<std::uint8_t, 44> kDecodeOpcodePattern = {
        0xFF, 0xC3, 0x01, 0xD1, 0xFC, 0x6F, 0x01, 0xA9, 0xFA, 0x67, 0x02, 0xA9, 0xF8, 0x5F, 0x03,
        0xA9, 0xF6, 0x57, 0x04, 0xA9, 0xF4, 0x4F, 0x05, 0xA9, 0xFD, 0x7B, 0x06, 0xA9, 0xFD, 0x83,
        0x01, 0x91, 0xE3, 0x03, 0x00, 0xF9, 0xF8, 0x03, 0x00, 0xAA, 0x1F, 0x8F, 0x02, 0xF8};
    static constexpr std::array<std::uint8_t, 40> kDecodeOpcodePatternV2 = {
        0xFF, 0xC3, 0x01, 0xD1, 0xFC, 0x6F, 0x01, 0xA9, 0xFA, 0x67, 0x02, 0xA9, 0xF8, 0x5F,
        0x03, 0xA9, 0xF6, 0x57, 0x04, 0xA9, 0xF4, 0x4F, 0x05, 0xA9, 0xFD, 0x7B, 0x06, 0xA9,
        0xFD, 0x83, 0x01, 0x91, 0xF8, 0x03, 0x00, 0xAA, 0x1F, 0x8F, 0x02, 0xF8};
};

// True when none of the AArch64 instructions in `bytes` (a multiple of 4)
// is pc-relative, so the sequence runs unchanged from another address. Both
// hooks displace the first 16 bytes of their target into a stash and execute
// them there.
auto prologueIsRelocatable(const std::uint8_t* bytes, std::size_t len) -> bool;

// True when decode_opcode at `off` in `image` still has the shape the hook
// relies on (see offset_finder.cpp for the exact checks).
auto decodeOpcodeShapeOk(const std::vector<std::uint8_t>& image, std::uint64_t off) -> bool;
