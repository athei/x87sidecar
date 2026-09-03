#include "rosetta_core/ProfileRuntime.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "rosetta_core/IRInstr.h"
#include "rosetta_core/OpcodeCompatibility.h"
#include "rosetta_core/ProfileFormat.h"

namespace profile {

namespace {

std::mutex g_mu;
struct BlockKey {
    const IRBlock* ptr;
    uint64_t hash;
    bool operator==(const BlockKey& o) const noexcept { return ptr == o.ptr && hash == o.hash; }
};
struct BlockKeyHash {
    size_t operator()(const BlockKey& k) const noexcept {
        const auto p = std::hash<const void*>{}(k.ptr);
        // 64-bit boost-style hash_combine.
        return p ^ (k.hash + 0x9E3779B97F4A7C15ULL + (p << 6) + (p >> 2));
    }
};
std::unordered_map<BlockKey, uint32_t, BlockKeyHash> g_block_ids;
uint32_t g_next_id = 0;
uint64_t g_counter_parent_addr = 0;
uint64_t g_counter_local_addr = 0;

// Per-block tally storage: 24 B per slot × kMaxBlocks = 24 MiB.  Lazy-
// allocated on first set_block_tally call; profile-disabled runs never pay
// the cost.  Stored as three parallel atomic<uint64> arrays (thirds of the
// 24-B BlockTally) so concurrent translation can store torn-free per-word.
// A torn read at dump time is harmless because the translator writes
// idempotent partial sums — by exit, all words have settled.
std::unique_ptr<std::atomic<uint64_t>[]> g_block_tally_lo;
std::unique_ptr<std::atomic<uint64_t>[]> g_block_tally_hi;
std::unique_ptr<std::atomic<uint64_t>[]> g_block_tally_x3;

// Per-block build-bail opcode side-table.  2 B per slot × kMaxBlocks = 2 MiB
// when allocated.  Lazy-allocated on first set_block_build_fail_op call.
std::unique_ptr<std::atomic<uint16_t>[]> g_block_build_fail_op;

// Per-block IR-gate refusal counters: 5 parallel atomic<u16>[] arrays
// indexed by kIRGateReason*.  Lazy-allocated together on first
// set_block_ir_gate_counters call; absent when profiling is disabled.
std::unique_ptr<std::atomic<uint16_t>[]> g_block_ir_gate_counts[kIRGateReasonCount];

// Per-block predecessor-of-top_dirty side-table.  Same shape as the
// build-bail side-table; lazy-allocated on first
// set_block_top_dirty_predecessor call.
std::unique_ptr<std::atomic<uint16_t>[]> g_block_top_dirty_pred;

// Per-block max-run-at-refuse side-table: 5 parallel atomic<u16>[] arrays
// indexed by kIRGateReason*.  Lazy-allocated on first
// set_block_max_run_at_refuse call.
std::unique_ptr<std::atomic<uint16_t>[]> g_block_max_run_at_refuse[kIRGateReasonCount];

void pack_tally(BlockTally t, uint64_t& lo, uint64_t& hi, uint64_t& x3) {
    static_assert(sizeof(BlockTally) == 24);
    std::memcpy(&lo, reinterpret_cast<const std::byte*>(&t) + 0, 8);
    std::memcpy(&hi, reinterpret_cast<const std::byte*>(&t) + 8, 8);
    std::memcpy(&x3, reinterpret_cast<const std::byte*>(&t) + 16, 8);
}

BlockTally unpack_tally(uint64_t lo, uint64_t hi, uint64_t x3) {
    BlockTally t{};
    std::memcpy(reinterpret_cast<std::byte*>(&t) + 0, &lo, 8);
    std::memcpy(reinterpret_cast<std::byte*>(&t) + 8, &hi, 8);
    std::memcpy(reinterpret_cast<std::byte*>(&t) + 16, &x3, 8);
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

uint32_t register_block(const IRBlock* block, uint64_t ir_hash) {
    std::scoped_lock lock(g_mu);
    const BlockKey key{.ptr = block, .hash = ir_hash};
    auto [it, inserted] = g_block_ids.try_emplace(key, g_next_id);
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

uint64_t hash_ir_stream(const IRInstr* instrs, size_t num_instrs) {
    // 64-bit FNV-1a over a CANONICALIZED copy of each IRInstr, so the same
    // logical IR hashes identically across launches, re-decodes AND host
    // Rosetta versions:
    //   - `opcode_` mapped to the internal id (opcode_host_to_internal):
    //     26.4 hosts and 26.5+ hosts encode the same instruction with
    //     different raw ids; hashing the internal id keeps X87_*_HASH_LIST
    //     values portable across macOS versions.  On modern hosts the map
    //     is identity, so existing modern-host hashes are unchanged.
    //   - `pc` zeroed: codegen may place the block at a different PC each run.
    //   - `flag_liveness` zeroed: recomputed from the block's successors on
    //     every decode; changes with no code change.
    //   - `_pad08` zeroed.
    //   - operand slots >= num_operands zeroed entirely: the decoder leaves
    //     stack garbage (per-run pointer values) in them.
    //   - used operands rebuilt field-by-field per kind: the union's dead
    //     value fields (IROperandRegister::_unused et al.) and pad bytes
    //     carry the same per-run garbage.
    //   - absolute addresses (AbsMem, which is what rip-relative lea/mov
    //     decode to, and fixup-carrying Immediates) keep only their page
    //     offset: the image slides by whole pages between launches, so the
    //     low 12 bits are the part of the address that survives ASLR.
    // Zeroing only `pc` (the old behaviour) left that garbage in the hash
    // input, so the hash was NOT stable across launches — which silently
    // broke every cross-run use of X87_*_HASH_LIST and profile_analyze
    // --dump-block-by-hash.  Verified empirically on the same binary run
    // twice: every block hashed differently until dead bytes were masked.
    constexpr uint64_t kFnvOffset = 14695981039346656037ULL;
    constexpr uint64_t kFnvPrime = 1099511628211ULL;
    uint64_t h = kFnvOffset;
    for (size_t i = 0; i < num_instrs; ++i) {
        IRInstr tmp = instrs[i];
        tmp.opcode_ = opcode_host_to_internal(tmp.opcode_);
        tmp.pc = 0;
        tmp._pad08 = 0;
        tmp.flag_liveness = 0;
        const int nops = tmp.num_operands > 4 ? 4 : tmp.num_operands;
        for (int o = 0; o < 4; ++o) {
            const IROperand src = tmp.operands[o];
            IROperand canon{};
            std::memset(&canon, 0, sizeof(canon));
            if (o < nops) {
                switch (src.kind) {
                    case IROperandKind::Register:
                        canon.reg.kind = src.reg.kind;
                        canon.reg.size = src.reg.size;
                        canon.reg.reg = src.reg.reg;
                        canon.reg.seg_override = src.reg.seg_override;
                        break;
                    case IROperandKind::MemRef:
                        canon.mem = src.mem;  // every byte is semantic
                        break;
                    case IROperandKind::AbsMem:
                        canon.abs_mem.kind = src.abs_mem.kind;
                        canon.abs_mem.size = src.abs_mem.size;
                        canon.abs_mem.addr_size = src.abs_mem.addr_size;
                        canon.abs_mem.value = src.abs_mem.value & 0xfff;
                        break;
                    case IROperandKind::Immediate:
                        canon.imm.kind = src.imm.kind;
                        canon.imm.size = src.imm.size;
                        canon.imm.addr_size = src.imm.addr_size;
                        canon.imm.mem_flags = src.imm.mem_flags;
                        canon.imm.value =
                            src.imm.mem_flags != 0 ? (src.imm.value & 0xfff) : src.imm.value;
                        break;
                    case IROperandKind::BranchOffset:
                        canon.branch.kind = src.branch.kind;
                        canon.branch.value = src.branch.value;
                        break;
                    case IROperandKind::ConditionCode:
                        canon.cc.kind = src.cc.kind;
                        canon.cc.cc = src.cc.cc;
                        break;
                    case IROperandKind::SegmentRegister:
                        canon.seg.kind = src.seg.kind;
                        canon.seg.seg_idx = src.seg.seg_idx;
                        break;
                    default:
                        canon = src;  // unknown kind: keep raw, don't guess
                        break;
                }
            }
            tmp.operands[o] = canon;
        }
        const auto* bytes = reinterpret_cast<const uint8_t*>(&tmp);
        for (size_t b = 0; b < sizeof(IRInstr); ++b) {
            h ^= bytes[b];
            h *= kFnvPrime;
        }
    }
    return h;
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
            g_block_tally_x3 = std::make_unique<std::atomic<uint64_t>[]>(kMaxBlocks);
        }
    }
    uint64_t lo = 0;
    uint64_t hi = 0;
    uint64_t x3 = 0;
    pack_tally(tally, lo, hi, x3);
    g_block_tally_lo[bid].store(lo, std::memory_order_relaxed);
    g_block_tally_hi[bid].store(hi, std::memory_order_relaxed);
    g_block_tally_x3[bid].store(x3, std::memory_order_relaxed);
}

BlockTally get_block_tally(uint32_t bid) {
    if (bid >= kMaxBlocks) {
        return {};
    }
    std::atomic<uint64_t>* lo_arr;
    std::atomic<uint64_t>* hi_arr;
    std::atomic<uint64_t>* x3_arr;
    {
        std::scoped_lock lock(g_mu);
        if (!g_block_tally_lo) {
            return {};
        }
        lo_arr = g_block_tally_lo.get();
        hi_arr = g_block_tally_hi.get();
        x3_arr = g_block_tally_x3.get();
    }
    const uint64_t lo = lo_arr[bid].load(std::memory_order_relaxed);
    const uint64_t hi = hi_arr[bid].load(std::memory_order_relaxed);
    const uint64_t x3 = x3_arr[bid].load(std::memory_order_relaxed);
    return unpack_tally(lo, hi, x3);
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

void set_block_ir_gate_counters(uint32_t bid, BlockIRGateCounters counters) {
    if (bid >= kMaxBlocks) {
        return;
    }
    {
        std::scoped_lock lock(g_mu);
        if (!g_block_ir_gate_counts[0]) {
            for (auto& slot : g_block_ir_gate_counts) {
                slot = std::make_unique<std::atomic<uint16_t>[]>(kMaxBlocks);
            }
        }
    }
    for (uint32_t r = 0; r < kIRGateReasonCount; ++r) {
        g_block_ir_gate_counts[r][bid].store(counters.counts[r], std::memory_order_relaxed);
    }
}

void set_block_top_dirty_predecessor(uint32_t bid, uint16_t opcode) {
    if (bid >= kMaxBlocks) {
        return;
    }
    {
        std::scoped_lock lock(g_mu);
        if (!g_block_top_dirty_pred) {
            g_block_top_dirty_pred = std::make_unique<std::atomic<uint16_t>[]>(kMaxBlocks);
            for (uint32_t i = 0; i < kMaxBlocks; ++i) {
                g_block_top_dirty_pred[i].store(0xFFFFU, std::memory_order_relaxed);
            }
        }
    }
    g_block_top_dirty_pred[bid].store(opcode, std::memory_order_relaxed);
}

uint16_t get_block_top_dirty_predecessor(uint32_t bid) {
    if (bid >= kMaxBlocks) {
        return 0xFFFFU;
    }
    std::atomic<uint16_t>* arr;
    {
        std::scoped_lock lock(g_mu);
        if (!g_block_top_dirty_pred) {
            return 0xFFFFU;
        }
        arr = g_block_top_dirty_pred.get();
    }
    return arr[bid].load(std::memory_order_relaxed);
}

void set_block_max_run_at_refuse(uint32_t bid, BlockMaxRunAtRefuse counters) {
    if (bid >= kMaxBlocks) {
        return;
    }
    {
        std::scoped_lock lock(g_mu);
        if (!g_block_max_run_at_refuse[0]) {
            for (auto& slot : g_block_max_run_at_refuse) {
                slot = std::make_unique<std::atomic<uint16_t>[]>(kMaxBlocks);
            }
        }
    }
    for (uint32_t r = 0; r < kIRGateReasonCount; ++r) {
        g_block_max_run_at_refuse[r][bid].store(counters.max_run[r], std::memory_order_relaxed);
    }
}

BlockMaxRunAtRefuse get_block_max_run_at_refuse(uint32_t bid) {
    BlockMaxRunAtRefuse out{};
    if (bid >= kMaxBlocks) {
        return out;
    }
    std::atomic<uint16_t>* arrs[kIRGateReasonCount];
    {
        std::scoped_lock lock(g_mu);
        if (!g_block_max_run_at_refuse[0]) {
            return out;
        }
        for (uint32_t r = 0; r < kIRGateReasonCount; ++r) {
            arrs[r] = g_block_max_run_at_refuse[r].get();
        }
    }
    for (uint32_t r = 0; r < kIRGateReasonCount; ++r) {
        out.max_run[r] = arrs[r][bid].load(std::memory_order_relaxed);
    }
    return out;
}

BlockIRGateCounters get_block_ir_gate_counters(uint32_t bid) {
    BlockIRGateCounters out{};
    if (bid >= kMaxBlocks) {
        return out;
    }
    std::atomic<uint16_t>* arrs[kIRGateReasonCount];
    {
        std::scoped_lock lock(g_mu);
        if (!g_block_ir_gate_counts[0]) {
            return out;
        }
        for (uint32_t r = 0; r < kIRGateReasonCount; ++r) {
            arrs[r] = g_block_ir_gate_counts[r].get();
        }
    }
    for (uint32_t r = 0; r < kIRGateReasonCount; ++r) {
        out.counts[r] = arrs[r][bid].load(std::memory_order_relaxed);
    }
    return out;
}

}  // namespace profile
