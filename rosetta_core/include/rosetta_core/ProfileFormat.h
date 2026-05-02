#pragma once

#include <cstdint>

// Binary record format for X87_PROFILE dumps written by the sidecar
// (rosetta_loader/src/sidecar.cpp) and read by the analyzer
// (tools/profile_analyze).  Append-only; one block_record per first-seen
// IRBlock pointer in the sidecar's process lifetime.
//
// All fields are little-endian (Apple Silicon native).
//
// File layout:
//   block_record { magic, block_id, num_instrs, start_pc,
//                  instr_record[num_instrs] }
//   block_record ...
//
// Resync: every block_record begins with kProfileBlockMagic so a partial
// write (kill -9 mid-flush) leaves a recoverable file.

namespace profile {

constexpr uint32_t kProfileBlockMagic = 0x30584C42U;  // 'BLK0'

struct BlockHeader {
    uint32_t magic;       // = kProfileBlockMagic
    uint32_t block_id;    // monotonic, sidecar-assigned
    uint32_t num_instrs;  // count of InstrRecord that follow
    uint32_t start_pc;    // localIR[0].pc
};
static_assert(sizeof(BlockHeader) == 16);

// Operand summary: one byte per dimension we care about for pattern keys.
//
//   kind   = IROperandKind cast to u8 (Register=0, MemRef=1, AbsMem=2,
//                                     Immediate=3, BranchOffset=4,
//                                     ConditionCode=5, SegmentRegister=6)
//   size   = IROperandSize cast to u8 (S8=0..S256=5, S80=0xFF)
//   reg    = base/index/sole register byte; 0 if not applicable
//   flags  = mem.mem_flags for MemRef (bit0=has_base, bit1=has_index);
//            imm.mem_flags for Immediate; 0 otherwise
struct OperandSummary {
    uint8_t kind;
    uint8_t size;
    uint8_t reg;
    uint8_t flags;
};
static_assert(sizeof(OperandSummary) == 4);

struct InstrRecord {
    uint16_t opcode;             // IRInstr::opcode (kOpcodeName_* enum)
    uint8_t num_operands;        // IRInstr::num_operands
    uint8_t ir_kind;             // IRInstr::ir_kind
    OperandSummary operands[4];  // padded with zeroed entries past num_operands
};
static_assert(sizeof(InstrRecord) == 20);

}  // namespace profile
