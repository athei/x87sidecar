#include "rosetta_core/ProfileRuntime.h"

#include <cstdint>
#include <mutex>
#include <unordered_map>

namespace profile {

namespace {

std::mutex g_mu;
std::unordered_map<const IRBlock*, uint32_t> g_block_ids;
uint32_t g_next_id = 0;
uint64_t g_counter_parent_addr = 0;
uint64_t g_counter_local_addr = 0;

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

}  // namespace profile
