#pragma once

#include <array>
#include <cstdint>
#include <iostream>

struct OffsetFinder {
    auto setDefaultOffsets() -> void;
    auto determineOffsets() -> bool;
    auto determineRuntimeOffsets() -> bool;

    std::uint64_t offsetExportsFetch_;
    std::uint64_t offsetSvcCallEntry_;
    std::uint64_t offsetSvcCallRet_;
    std::uint64_t offsetDisableAot_;

    std::uint64_t offsetTransactionResultSize_;
    std::uint64_t offsetTranslateInsn_;
    std::uint64_t offsetInitLibrary_;

    // Exports.version, read from the on-disk runtime by determineRuntimeOffsets.
    // Seeds the OpcodeCompatibility layer (26.4↔26.5) without needing the live
    // Exports struct (X19) — so both attach modes get it the same way.
    std::uint64_t runtimeVersion_ = 0;

    // translate_insn's 36-byte prologue signature (stp/sub-sp/mov prologue).
    // Unique enough to locate translate_insn in live memory via a content scan,
    // so we never need the Exports struct to derive its address.
    static constexpr std::array<std::uint8_t, 36> kTranslateInsnPattern = {
        0xFF, 0x43, 0x03, 0xD1, 0xFC, 0x6F, 0x07, 0xA9, 0xfa, 0x67, 0x08, 0xa9,
        0xF8, 0x5F, 0x09, 0xA9, 0xF6, 0x57, 0x0A, 0xA9, 0xF4, 0x4F, 0x0B, 0xA9,
        0xFD, 0x7B, 0x0C, 0xA9, 0xFD, 0x03, 0x03, 0x91, 0xF3, 0x03, 0x00, 0xAA};

    // decode_opcode's 44-byte prologue signature.  Rosetta's single-instruction
    // x86 decoder:
    //   int decode_opcode(DecoderCtx*, unsigned offset, DecodedInsn* out,
    //                     uint8_t* out_len)   -> 0=ok, 1=invalid, 2=fault
    //
    // The first 32 bytes are a generic 0x70-frame prologue that matches 72
    // places in the runtime, so the signature has to run through the three
    // instructions that follow: `str x3, [sp]` (stashing out_len), `mov x24,
    // x0`, and `str xzr, [x24, #0x28]!` (zeroing ctx->fault, which is also
    // what pins DecoderCtx's layout).  With those it is unique.
    static constexpr std::array<std::uint8_t, 44> kDecodeOpcodePattern = {
        0xFF, 0xC3, 0x01, 0xD1, 0xFC, 0x6F, 0x01, 0xA9, 0xFA, 0x67, 0x02,
        0xA9, 0xF8, 0x5F, 0x03, 0xA9, 0xF6, 0x57, 0x04, 0xA9, 0xF4, 0x4F,
        0x05, 0xA9, 0xFD, 0x7B, 0x06, 0xA9, 0xFD, 0x83, 0x01, 0x91, 0xE3,
        0x03, 0x00, 0xF9, 0xF8, 0x03, 0x00, 0xAA, 0x1F, 0x8F, 0x02, 0xF8};
};
