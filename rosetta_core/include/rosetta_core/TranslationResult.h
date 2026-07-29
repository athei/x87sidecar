#pragma once

#include <cstdint>

#include "rosetta_core/AssemblerBuffer.h"
#include "rosetta_core/Fixup.h"
#include "rosetta_core/IRModuleData.h"
#include "rosetta_core/RemoteVector.h"
#include "rosetta_core/ThreadContextOffsets.h"
#include "rosetta_core/TransactionalList.h"
#include "rosetta_core/X87Cache.h"

struct IRBlock;

/// One entry of a segment's ARM/x86 instruction map.
///
/// Offsets are relative to the owning segment's `x86_begin` / `arm_begin`.
/// Entries are sorted, strictly increasing in both offsets, and the last one is
/// a sentinel holding the segment's end offsets rather than a real instruction.
///
/// The map is not per-instruction: an entry marks a point where the ARM output
/// resynchronises with an x86 instruction boundary, so a fused run of x86
/// instructions produces a single entry covering the whole run.
struct InstructionOffset {
    uint32_t x86_offset;
    uint32_t arm_offset;  ///< bytes, not instructions
    uint32_t flags;       ///< 0x002 and 0x200 observed; rest of the field unused so far
};

static_assert(sizeof(InstructionOffset) == 12, "InstructionOffset size mismatch");

/// One output segment of a translation.
///
/// JIT translations always have exactly one; only AOT translations are ever
/// segmented. Every observed segment starts at zero, so the range pairs below
/// are indistinguishable from {0, size}.
struct TranslationSegment {
    uint32_t x86_begin;
    uint32_t x86_end;
    uint32_t arm_begin;
    uint32_t arm_end;
    RemoteVector instruction_offsets;  ///< InstructionOffset
};

static_assert(sizeof(TranslationSegment) == 0x28, "TranslationSegment size mismatch");
static_assert(offsetof(TranslationSegment, instruction_offsets) == 0x10,
              "TranslationSegment::instruction_offsets offset mismatch");

struct TranslationResult {
    IRModuleData* ir_module_data;
    char _mode;
    char field_9;
    char field_A;
    char field_B;
    char field_C;
    char field_D;
    char field_E;
    char field_F;
    AssemblerBuffer insn_buf;
    uint32_t text_base_align_offset;
    uint32_t field_34;
    // Inherited names, never verified against stock and never read. The
    // ARM/x86 instruction map is `segments[0].instruction_offsets`, below.
    uint64_t arm_to_x86_map_begin;
    uint64_t arm_to_x86_map_end;
    uint64_t field_48;
    TransactionalList<Fixup> external_fixups;  ///< ADR+ADD for x86 abs addr reloc
    TransactionalList<Fixup> internal_fixups;  ///< Internal branch patch-ups
    TransactionalList<Fixup> _fixups;          ///< BL into runtime helper stubs
    TransactionalList<Fixup> field_B0;
    TransactionalList<Fixup> dyld_stub_fixups;  ///< dyld stub GOT-style relocations
    uint64_t field_F0;
    uint64_t field_F8;
    uint64_t field_100;
    uint64_t field_108;
    uint64_t field_110;
    uint64_t field_118;
    uint64_t field_120;
    uint64_t field_128;
    uint64_t field_130;
    uint64_t field_138;
    uint64_t field_140;
    uint64_t field_148;
    uint64_t field_150;
    uint64_t field_158;
    uint64_t field_160;
    uint64_t field_168;
    uint64_t field_170;
    uint64_t field_178;
    uint64_t field_180;
    uint64_t field_188;
    uint32_t field_190;
    uint32_t field_194;
    uint32_t field_198;
    uint32_t branch_slots_offset;
    uint32_t branch_slots_count;
    uint32_t max_translated_x86_pc;
    TransactionalList<Fixup> field_1A8;  ///< Cross-block/sequential fixups
    RemoteVector segments;               ///< TranslationSegment; exactly 1 for JIT
    uint64_t field_1E0;
    uint64_t field_1E8;
    uint64_t field_1F0;
    uint64_t field_1F8;
    uint64_t field_200;
    uint64_t field_208;
    uint32_t free_gpr_mask;
    uint32_t free_fpr_mask;
    uint32_t _unoccupied_temporary_fprs_for_xmm_scalars;
    uint32_t _pinned_temporary_scalars;
    uint64_t field_220;
    uint64_t field_228;
    ThreadContextOffsets* thread_context_offsets;
    char translator_variant;
    char field_239;
    uint16_t ymm_upper_half_alloc_mask;
    uint8_t ymm_upper_half_fpr[16];
    RemoteVector branch_entries;  ///< 12-byte entries; layout not determined
    uint32_t dword268;
    uint32_t field_26C;
    // Past stock's real allocation size (0x268) — stock never writes here.
    uint64_t field_270;
    uint64_t field_278;
    uint64_t field_280;

    // ── OPT-1: Per-instance x87 base/TOP register cache ────────────────────────
    X87Cache x87_cache;
};

// sizeof changed from 0x288 due to OPT-1 cache fields appended at end.
static_assert(offsetof(TranslationResult, ir_module_data) == 0x00,
              "TranslationResult::ir_module_data offset mismatch");
static_assert(offsetof(TranslationResult, insn_buf) == 0x10,
              "TranslationResult::insn_buf offset mismatch");
static_assert(offsetof(TranslationResult, external_fixups) == 0x50,
              "TranslationResult::external_fixups offset mismatch");
static_assert(offsetof(TranslationResult, internal_fixups) == 0x70,
              "TranslationResult::internal_fixups offset mismatch");
static_assert(offsetof(TranslationResult, _fixups) == 0x90,
              "TranslationResult::_fixups offset mismatch");
static_assert(offsetof(TranslationResult, dyld_stub_fixups) == 0xD0,
              "TranslationResult::dyld_stub_fixups offset mismatch");
static_assert(offsetof(TranslationResult, text_base_align_offset) == 0x30,
              "TranslationResult::text_base_align_offset offset mismatch");
static_assert(offsetof(TranslationResult, branch_slots_offset) == 0x19C,
              "TranslationResult::branch_slots_offset offset mismatch");
static_assert(offsetof(TranslationResult, branch_slots_count) == 0x1A0,
              "TranslationResult::branch_slots_count offset mismatch");
static_assert(offsetof(TranslationResult, max_translated_x86_pc) == 0x1A4,
              "TranslationResult::max_translated_x86_pc offset mismatch");
static_assert(offsetof(TranslationResult, segments) == 0x1C8,
              "TranslationResult::segments offset mismatch");
static_assert(offsetof(TranslationResult, branch_entries) == 0x250,
              "TranslationResult::branch_entries offset mismatch");
static_assert(offsetof(TranslationResult, free_gpr_mask) == 0x210,
              "TranslationResult::free_gpr_mask offset mismatch");
static_assert(offsetof(TranslationResult, free_fpr_mask) == 0x214,
              "TranslationResult::free_fpr_mask offset mismatch");
static_assert(offsetof(TranslationResult, thread_context_offsets) == 0x230,
              "TranslationResult::thread_context_offsets offset mismatch");