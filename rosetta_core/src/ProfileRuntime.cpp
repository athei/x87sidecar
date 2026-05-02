#include "rosetta_core/ProfileRuntime.h"

#include <atomic>
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

// Per-block tally storage: 8 B per slot × kMaxBlocks = 8 MiB.  Lazy-allocated
// on first set_block_tally call; profile-disabled runs never pay the cost.
// Each slot is an atomic<uint64_t> so the four u16 packed fields are written
// torn-free under concurrent translation.  The translator writes the same
// value repeatedly during a block's translation (idempotent partial sums);
// final state at dump time is the correct full tally.
std::unique_ptr<std::atomic<uint64_t>[]> g_block_tally;

uint64_t pack_tally(BlockTally t) {
    uint64_t v = 0;
    std::memcpy(&v, &t, sizeof(t));
    return v;
}

BlockTally unpack_tally(uint64_t v) {
    BlockTally t{};
    std::memcpy(&t, &v, sizeof(t));
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
        if (!g_block_tally) {
            g_block_tally = std::make_unique<std::atomic<uint64_t>[]>(kMaxBlocks);
        }
    }
    g_block_tally[bid].store(pack_tally(tally), std::memory_order_relaxed);
}

BlockTally get_block_tally(uint32_t bid) {
    if (bid >= kMaxBlocks) {
        return {};
    }
    std::atomic<uint64_t>* arr;
    {
        std::scoped_lock lock(g_mu);
        if (!g_block_tally) {
            return {};
        }
        arr = g_block_tally.get();
    }
    return unpack_tally(arr[bid].load(std::memory_order_relaxed));
}

}  // namespace profile
