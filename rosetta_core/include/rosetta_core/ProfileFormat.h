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
// Block records store full IRInstr values (sizeof(IRInstr) == 64).  The
// analyzer feeds these straight to Translator::translate_instruction to
// measure real ARM emit per pattern; storing a lossy summary would force
// the analyzer to either stub or re-derive operand fields, both of which
// distort the count.  Writers MUST zero per-instruction `pc` so identical
// patterns hash identically across captures — PC varies per run via PIC
// and the analyzer never reads it.
//
// All fields are little-endian (Apple Silicon native).

namespace profile {

// counter_section magic ('CNT0').  Block records carry no per-record
// magic; this 4-byte sentinel marks the transition from streamed
// block_records to the counter array.  Each block_record's first 4 bytes
// are an IRInstr's leading `pc` field, which the writer always zeroes —
// so the all-zero leading word can't collide with any printable-ASCII
// magic ('CNT0', 'TLY1', 'BFO0').
constexpr uint32_t kCounterSectionMagic = 0x30544E43U;  // 'CNT0'

struct BlockHeader {
    uint32_t block_id;    // monotonic, from profile::register_block
    uint32_t num_instrs;  // count of IRInstr that follow
    uint32_t start_pc;    // localIR[0].pc
    uint32_t _reserved;   // pad to 16 B; must be zero on write
};
static_assert(sizeof(BlockHeader) == 16);

// Counter section (single instance, end of file).  count == the value
// returned by profile::block_count() at exit time.  counts[i] is the
// uint64_t execution count for block_id i.
struct CounterSectionHeader {
    uint32_t magic;  // = kCounterSectionMagic
    uint32_t count;  // number of u64 entries that follow
};
static_assert(sizeof(CounterSectionHeader) == 8);

// Tally section magic ('TLY1', bumped from 'TLY0' when the entry grew to
// include IR-failure-reason classifiers in 2026-05-03).  Optional section
// written immediately after the counter section at exit time.  Per
// block_id (0..count-1), records which translation-dispatch path consumed
// each x87 op AND which gate refused on IR failures.  Dump happens at
// exit because path is only known after all of the block's
// translate_instruction calls have completed; dumpBlockIfNew is called
// *before* translate_instruction (see sidecar.cpp:~370), so inline per-
// block tally would race the bumps.  Older .prof files (no tally section
// or 'TLY0' magic with the smaller 8-byte entry) parse only the counter
// section in current builds — the failure-reason columns are absent.
constexpr uint32_t kTallySectionMagic = 0x31594C54U;  // 'TLY1'

struct TallySectionHeader {
    uint32_t magic;  // = kTallySectionMagic
    uint32_t count;  // number of BlockTallyEntry rows that follow
};
static_assert(sizeof(TallySectionHeader) == 8);

struct BlockTallyEntry {
    uint16_t ir_ops;             // X87IR::compile_run consumed N ops
    uint16_t peephole_ops;       // try_peephole consumed N ops
    uint16_t single_ops;         // single-op translate_*
    uint16_t fallthrough_ops;    // returned nullopt → forwarded to stock
    uint16_t ir_build_fail_ops;  // compile_run failed at build (kMaxNodes / unhandled op)
    uint16_t ir_fpr_fail_ops;    // peak_live_fprs > available
    uint16_t ir_gpr_fail_ops;    // peak_live_gprs > available
    uint16_t max_gpr_peak;       // max peak_live_gprs(ctx) observed (diagnostic)
};
static_assert(sizeof(BlockTallyEntry) == 16);

// Build-bail-opcode side-table magic ('BFO0').  Optional section written
// after the tally section at exit time.  Per block_id (0..count-1) records
// the kOpcodeName_* value at which X87IR::build()'s default arm bailed —
// the most-recent observation, latest-write-wins.  Sentinel 0xFFFF means
// the block never tripped an unsupported-opcode bail.  Use this section
// alongside the counter section to compute exec-weighted attribution of
// which x87 opcodes are blocking IR coverage in the workload.  Older
// .prof files lacking this section still parse; analyzer treats absence as
// "all-sentinel".
constexpr uint32_t kBuildFailOpSectionMagic = 0x304F4642U;  // 'BFO0'

struct BuildFailOpSectionHeader {
    uint32_t magic;  // = kBuildFailOpSectionMagic
    uint32_t count;  // number of u16 entries that follow
};
static_assert(sizeof(BuildFailOpSectionHeader) == 8);

constexpr uint16_t kNoBuildFailOpcode = 0xFFFFU;

// IR-gate per-reason refusal counter side-table magic ('IRG1', bumped from
// 'IRG0' when the section grew from a single per-block sentinel to a
// 5-counter array).  Optional section written after the build-bail section
// at exit time.  Per block_id (0..count-1), records how many times each
// of the 5 IR-eligibility gate conditions refused entry to compile_run.
// Per-reason counts avoid the trailing-tail problem of a single sentinel:
// when IR fails on a longer run, the dispatcher peels one op via
// peep+single, then re-enters the gate at decreasing run_remaining,
// eventually firing short_run on the tail.  Counters capture all
// refusals; the analyzer then attributes them per pattern by exec-weighted
// fraction (same shape as ir_build_fail_ops / ir_fpr_fail_ops in TLY1).
// Distinct from the ir_*_fail counters in BlockTallyEntry, which only
// count failures *after* compile_run was called.  Older .prof files
// lacking this section (or carrying an old IRG0 magic) parse with the
// rest of the file intact; analyzer treats absence as "no refusals
// recorded".
constexpr uint32_t kIRGateRefuseSectionMagic = 0x31475249U;  // 'IRG1'

constexpr uint16_t kIRGateReasonShortRun = 0;     // run_remaining < 3
constexpr uint16_t kIRGateReasonTopDirty = 1;     // top_dirty != 0
constexpr uint16_t kIRGateReasonTagPush = 2;      // tag_push_pending != 0
constexpr uint16_t kIRGateReasonDeferredPop = 3;  // deferred_pop_count != 0
constexpr uint16_t kIRGateReasonPermDirty = 4;    // perm_dirty
constexpr uint16_t kIRGateReasonCount = 5;

struct IRGateRefuseSectionHeader {
    uint32_t magic;  // = kIRGateRefuseSectionMagic
    uint32_t count;  // number of BlockIRGateCounters rows that follow
};
static_assert(sizeof(IRGateRefuseSectionHeader) == 8);

struct BlockIRGateCounters {
    uint16_t counts[kIRGateReasonCount];  // indexed by kIRGateReason*
};
static_assert(sizeof(BlockIRGateCounters) == 10);

constexpr const char* kIRGateReasonNames[kIRGateReasonCount] = {
    "short_run", "top_dirty", "tag_push_pending", "deferred_pop", "perm_dirty",
};

}  // namespace profile
