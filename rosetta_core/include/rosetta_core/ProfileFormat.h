#pragma once

#include <cstdint>

// Binary file format for X87_PROFILE dumps.
//
// Writers (rosetta_loader/src/sidecar.cpp) stream one block_record per
// first-seen IRBlock during the run, then append a single counter_section
// at parent exit (`sidecar::dumpCountersIfEnabled`, called from the
// kqueue NOTE_EXIT path in main.cpp).  Reader (tools/profile_analyze)
// consumes block_records sequentially until the first 4 bytes match the
// counter_section magic, then parses the counter array.
//
// A profile lacking the counter_section is incomplete (the parent process
// died via SIGKILL or similar before the loader could read counters).
// The analyzer rejects such files — runtime exec_count is the whole point
// of the dataset; ranking by compile-occurrence alone is intentionally
// not supported.
//
// All fields are little-endian (Apple Silicon native).

namespace profile {

// counter_section magic ('CNT0').  Block records carry no per-record
// magic; this 4-byte sentinel marks the transition from streamed
// block_records to the counter array.  No InstrRecord starts with these
// bytes because opcode is uint16 (top of range = 0x29B) and ir_kind is
// uint8 — the leading 4 bytes of any InstrRecord encode (opcode_lo,
// opcode_hi, num_operands, ir_kind).  'C'=0x43, 'N'=0x4E in the first
// two bytes would imply opcode 0x4E43 = 20035, well beyond our 0x29B
// max, so collision is structurally impossible.
constexpr uint32_t kCounterSectionMagic = 0x30544E43U;  // 'CNT0'

struct BlockHeader {
    uint32_t block_id;    // monotonic, from profile::register_block
    uint32_t num_instrs;  // count of InstrRecord that follow
    uint32_t start_pc;    // localIR[0].pc
    uint32_t _reserved;   // pad to 16 B; must be zero on write
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

// Counter section (single instance, end of file).  count == the value
// returned by profile::block_count() at exit time.  counts[i] is the
// uint64_t execution count for block_id i.
struct CounterSectionHeader {
    uint32_t magic;  // = kCounterSectionMagic
    uint32_t count;  // number of u64 entries that follow
};
static_assert(sizeof(CounterSectionHeader) == 8);

}  // namespace profile
