#pragma once

#include <cstddef>
#include <cstdint>

#include "rosetta_core/ProfileFormat.h"

struct IRBlock;

// Cross-component runtime state for X87_PROFILE.
//
// Both the sidecar (Stage A IR-stream dump) and the Translator (Stage B
// JIT-emitted block-entry counter bump) need a stable mapping from
// IRBlock* (parent VA) to a uint32_t block_id, plus a single source of
// truth for the parent-side counter array address.  These are the two
// integration points; profile::register_block is the only allocator of
// new ids and the only writer to the next_id counter, guaranteeing that
// dump records and counter slots cannot disagree.
//
// All functions are thread-safe (mutex-guarded internally); no caller-
// side locking required.

namespace profile {

constexpr uint32_t kMaxBlocks = 1U << 20;                             // 1 M ids
constexpr uint32_t kOverflowId = 0xFFFFFFFFU;                         // returned past kMaxBlocks
constexpr std::size_t kCounterBytes = sizeof(uint64_t) * kMaxBlocks;  // 8 MiB

// Register the counter array.  The same backing pages are mapped into
// BOTH processes via mach_vm_remap(copy=FALSE):
//  - parent_addr is what JIT-emitted code materializes via MOVZ/MOVK
//    (parent process VAs).
//  - local_addr is what the sidecar reads directly at exit time
//    (sidecar process VAs).
// Both 0 means profiling is disabled / allocation failed.
void set_counter_array(uint64_t parent_addr, uint64_t local_addr);
uint64_t counter_array_addr();        // parent VA, used by JIT emit
uint64_t counter_array_local_addr();  // sidecar VA, used by exit-readback

// First-see assigns a fresh id; subsequent calls for the same block
// pointer return the same id (idempotent).  Returns kOverflowId once
// the registry has handed out kMaxBlocks ids — caller must skip the
// counter bump for that block.
uint32_t register_block(const IRBlock* block);

// Number of registered blocks (== next id that would be assigned).
// Used by the exit-readback path to size the counter mach_vm_read.
uint32_t block_count();

// Per-block translation-path tally.  Set during translate_instruction (one
// idempotent atomic store per partial accumulation; the value at dump time
// is the final count of x87 ops consumed by each path).  Read once by the
// sidecar at dumpBlockIfNew time and emitted into the BlockHeader.
struct BlockTally {
    uint16_t ir_ops;
    uint16_t peephole_ops;
    uint16_t single_ops;
    uint16_t fallthrough_ops;
    // IR-failure classifiers (disjoint from above): when the IR gate
    // attempted compile_run but it returned 0, which gate refused?
    uint16_t ir_build_fail_ops;
    uint16_t ir_fpr_fail_ops;
    uint16_t ir_gpr_fail_ops;
    // Max peak_live_gprs observed for this block across all compile_run
    // attempts.  Diagnostic only — not a bumped op counter, so it doesn't
    // contribute to the per-pattern path-mix percentages.
    uint16_t max_gpr_peak;
};
static_assert(sizeof(BlockTally) == 16);

void set_block_tally(uint32_t bid, BlockTally tally);
BlockTally get_block_tally(uint32_t bid);

// Per-block build-bail opcode (kOpcodeName_*).  Latest-write-wins; sentinel
// 0xFFFF means no bail observed.  Lazy-allocated on first set call (2 B/slot
// × kMaxBlocks = 2 MiB), so profile-disabled runs never pay the cost.
void set_block_build_fail_op(uint32_t bid, uint16_t opcode);
uint16_t get_block_build_fail_op(uint32_t bid);

// Per-block IR-gate refusal counters (5 × uint16, indexed by kIRGateReason*).
// Lazy-allocated on first set call (10 B/slot × kMaxBlocks = 10 MiB total
// across the 5 atomic arrays), so profile-disabled runs never pay the cost.
// Records which pre-build dispatcher gate condition refused entry to
// compile_run AND how many times.  Idempotent set (latest-write-wins per
// counter); the translator accumulates non-atomically in X87Cache and
// mirrors the running totals here.
void set_block_ir_gate_counters(uint32_t bid, BlockIRGateCounters counters);
BlockIRGateCounters get_block_ir_gate_counters(uint32_t bid);

// Per-block predecessor-of-top_dirty (kOpcodeName_*).  Latest-write-wins;
// sentinel 0xFFFF means no top_dirty refusal observed.  Lazy-allocated.
// Records the last x87 opcode translated in the block before the
// most-recent top_dirty gate refusal — pinpoints which op left top_dirty=1.
void set_block_top_dirty_predecessor(uint32_t bid, uint16_t opcode);
uint16_t get_block_top_dirty_predecessor(uint32_t bid);

}  // namespace profile
