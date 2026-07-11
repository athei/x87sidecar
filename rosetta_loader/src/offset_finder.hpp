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
};
