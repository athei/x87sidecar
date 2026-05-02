#pragma once

#include <cstddef>
#include <cstdint>

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

}  // namespace profile
