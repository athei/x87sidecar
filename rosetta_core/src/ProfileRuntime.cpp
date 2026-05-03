#include "rosetta_core/ProfileRuntime.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace profile {

namespace {

std::mutex g_mu;
std::unordered_map<const IRBlock*, uint32_t> g_block_ids;
uint32_t g_next_id = 0;
uint64_t g_counter_parent_addr = 0;
uint64_t g_counter_local_addr = 0;

// Per-block tally storage: 16 B per slot × kMaxBlocks = 16 MiB.  Lazy-
// allocated on first set_block_tally call; profile-disabled runs never pay
// the cost.  Stored as two parallel atomic<uint64> arrays (low + high half
// of the 16-B BlockTally) so concurrent translation can store torn-free
// per-half.  A torn read at dump time is harmless because the translator
// writes idempotent partial sums — by exit, both halves have settled.
std::unique_ptr<std::atomic<uint64_t>[]> g_block_tally_lo;
std::unique_ptr<std::atomic<uint64_t>[]> g_block_tally_hi;

// Per-block build-bail opcode side-table.  2 B per slot × kMaxBlocks = 2 MiB
// when allocated.  Lazy-allocated on first set_block_build_fail_op call.
std::unique_ptr<std::atomic<uint16_t>[]> g_block_build_fail_op;

void pack_tally(BlockTally t, uint64_t& lo, uint64_t& hi) {
    static_assert(sizeof(BlockTally) == 16);
    std::memcpy(&lo, reinterpret_cast<const std::byte*>(&t) + 0, 8);
    std::memcpy(&hi, reinterpret_cast<const std::byte*>(&t) + 8, 8);
}

BlockTally unpack_tally(uint64_t lo, uint64_t hi) {
    BlockTally t{};
    std::memcpy(reinterpret_cast<std::byte*>(&t) + 0, &lo, 8);
    std::memcpy(reinterpret_cast<std::byte*>(&t) + 8, &hi, 8);
    return t;
}

}  // namespace

void set_counter_array(uint64_t parent_addr, uint64_t local_addr) {
    std::scoped_lock lock(g_mu);
    g_counter_parent_addr = parent_addr;
    g_counter_local_addr = local_addr;
}

uint64_t counter_array_addr() {
    std::scoped_lock lock(g_mu);
    return g_counter_parent_addr;
}

uint64_t counter_array_local_addr() {
    std::scoped_lock lock(g_mu);
    return g_counter_local_addr;
}

uint32_t register_block(const IRBlock* block) {
    std::scoped_lock lock(g_mu);
    auto [it, inserted] = g_block_ids.try_emplace(block, g_next_id);
    if (!inserted) {
        return it->second;
    }
    if (g_next_id >= kMaxBlocks) {
        // Roll back the speculative emplace — we don't want to occupy a
        // map slot for an id we can't honour.
        g_block_ids.erase(it);
        return kOverflowId;
    }
    ++g_next_id;
    return it->second;
}

uint32_t block_count() {
    std::scoped_lock lock(g_mu);
    return g_next_id;
}

void set_block_tally(uint32_t bid, BlockTally tally) {
    if (bid >= kMaxBlocks) {
        return;
    }
    {
        std::scoped_lock lock(g_mu);
        if (!g_block_tally_lo) {
            g_block_tally_lo = std::make_unique<std::atomic<uint64_t>[]>(kMaxBlocks);
            g_block_tally_hi = std::make_unique<std::atomic<uint64_t>[]>(kMaxBlocks);
        }
    }
    uint64_t lo = 0;
    uint64_t hi = 0;
    pack_tally(tally, lo, hi);
    g_block_tally_lo[bid].store(lo, std::memory_order_relaxed);
    g_block_tally_hi[bid].store(hi, std::memory_order_relaxed);
}

BlockTally get_block_tally(uint32_t bid) {
    if (bid >= kMaxBlocks) {
        return {};
    }
    std::atomic<uint64_t>* lo_arr;
    std::atomic<uint64_t>* hi_arr;
    {
        std::scoped_lock lock(g_mu);
        if (!g_block_tally_lo) {
            return {};
        }
        lo_arr = g_block_tally_lo.get();
        hi_arr = g_block_tally_hi.get();
    }
    const uint64_t lo = lo_arr[bid].load(std::memory_order_relaxed);
    const uint64_t hi = hi_arr[bid].load(std::memory_order_relaxed);
    return unpack_tally(lo, hi);
}

void set_block_build_fail_op(uint32_t bid, uint16_t opcode) {
    if (bid >= kMaxBlocks) {
        return;
    }
    {
        std::scoped_lock lock(g_mu);
        if (!g_block_build_fail_op) {
            g_block_build_fail_op = std::make_unique<std::atomic<uint16_t>[]>(kMaxBlocks);
            // Default-constructed atomic<uint16_t> holds 0 (i.e. kOpcodeName_aaa,
            // which is never an x87 opcode and never reaches build()'s default
            // arm).  We initialize explicitly to the 0xFFFF sentinel so dump-
            // time readback is unambiguous.
            for (uint32_t i = 0; i < kMaxBlocks; ++i) {
                g_block_build_fail_op[i].store(0xFFFFU, std::memory_order_relaxed);
            }
        }
    }
    g_block_build_fail_op[bid].store(opcode, std::memory_order_relaxed);
}

uint16_t get_block_build_fail_op(uint32_t bid) {
    if (bid >= kMaxBlocks) {
        return 0xFFFFU;
    }
    std::atomic<uint16_t>* arr;
    {
        std::scoped_lock lock(g_mu);
        if (!g_block_build_fail_op) {
            return 0xFFFFU;
        }
        arr = g_block_build_fail_op.get();
    }
    return arr[bid].load(std::memory_order_relaxed);
}

}  // namespace profile
