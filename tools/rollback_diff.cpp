// rollback_diff -- replay a captured x87 IR block through Translator twice
// (rollback OFF vs ON for top_dirty + deferred_pop) and hex-diff the emitted
// ARM.  Investigates Bug 2: the IR-gate speculative-flush rollback corrupts
// WoW geometry when re-enabled for the top_dirty / deferred_pop branches.
//
// Usage:
//   rollback_diff <ir_blob>
//
// <ir_blob> is a raw IRInstr[] file, no header.  Produced by
//   profile_analyze --dump-ir-binary 0xH out.ir <profile.prof>
//
// The current geom-trigger blob ships under tests/data/geom_block_874c40.ir
// (block hash 0x874c409431b77f5e, 169 instrs).  Its 29 FprPressure rollback
// firings during translation are the smallest reproducer in the wild for
// the corruption.
//
// Diff verdict:
//   - If insn_buf differs between runs, the divergence pin-points the
//     post-rollback emit decision that breaks WoW.  The byte offset is
//     reported alongside the IR insn_idx where the divergence appeared.
//   - If insn_buf is byte-identical, the bug is *not* in the emitted code
//     for this block — it lives in cross-block runtime state (cache
//     re-init, X87State memory truth, store-forwarding).  The investigation
//     pivots to lldb-on-WoW.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <ios>
#include <iterator>
#include <vector>

#include "rosetta_core/Config.h"
#include "rosetta_core/CoreConfig.h"
#include "rosetta_core/IRBlock.h"
#include "rosetta_core/IRInstr.h"
#include "rosetta_core/ProfileRuntime.h"
#include "rosetta_core/Register.h"
#include "rosetta_core/ThreadContextOffsets.h"
#include "rosetta_core/TranscendentalHelper.h"
#include "rosetta_core/TranslationResult.h"
#include "rosetta_core/Translator.h"
#include "rosetta_core/X87Cache.h"

namespace {

struct CacheSnap {
    int8_t top_dirty;
    int8_t tag_push_pending;
    int8_t deferred_pop_count;
    int8_t perm_dirty;
    int8_t perm[8];
    int16_t run_remaining;
    int8_t cur_top;  // x87_cache.top_gpr's logical TOP value tracked separately
    bool operator==(const CacheSnap&) const = default;
};

struct RunResult {
    std::vector<uint32_t> arm;             // emitted ARM instructions (one per uint32_t)
    std::vector<int64_t> idx_per_word;     // IR insn_idx that produced each ARM word
    std::vector<CacheSnap> snap_after_idx; // post-translate_instruction cache state per idx
};

// One translation pass.  cfg is installed via rosetta_set_config; the
// transcendental address + counter array are static singletons set by main.
// idx_per_word records, for each emitted ARM word, the IR position the
// translator was on when that word was written — lets us map a byte
// divergence back to "which IR op produced this delta".
RunResult runOnce(const std::vector<IRInstr>& instrs, const RosettaConfig& cfg) {
    static ThreadContextOffsets stub_tco{};

    TranslationResult result{};
    constexpr size_t kBufSize = 256UL * 1024;
    result.insn_buf.data = static_cast<uint32_t*>(std::malloc(kBufSize));
    result.insn_buf.end = 0;
    result.insn_buf.end_cap = kBufSize;
    result.insn_buf.use_heap = 1;
    result.free_gpr_mask = kGprScratchMask;
    result.free_fpr_mask = kFprScratchMask;
    result._unoccupied_temporary_fprs_for_xmm_scalars = kFprScratchMask;
    result._pinned_temporary_scalars = 0;
    result.thread_context_offsets = &stub_tco;
    result.translator_variant = 0;

    IRBlock dummy{};
    dummy.num_instrs = static_cast<uint32_t>(instrs.size());

    rosetta_set_config(&cfg);

    std::vector<IRInstr> mut(instrs.begin(), instrs.end());
    std::vector<int64_t> idx_per_word;
    std::vector<CacheSnap> snaps(mut.size(), CacheSnap{});
    int64_t idx = 0;
    const auto n = static_cast<int64_t>(mut.size());
    while (idx < n) {
        const uint64_t before = result.insn_buf.end;
        const auto next = Translator::translate_instruction(&result, &dummy, mut.data(), n, idx);
        const uint64_t after = result.insn_buf.end;
        const auto words_emitted = static_cast<size_t>((after - before) / sizeof(uint32_t));
        for (size_t w = 0; w < words_emitted; ++w) {
            idx_per_word.push_back(idx);
        }
        // Snapshot cache state after the call.  consumed = next - idx; record at
        // the LAST consumed idx so a peephole that ate 2-3 ops sees the snapshot
        // attached to its terminating idx.  Earlier consumed-but-skipped idx
        // get a sentinel (run_remaining=-1) so the comparator can ignore them.
        const int64_t advance = next.value_or(idx + 1) - idx;
        const int64_t snap_idx = idx + advance - 1;
        if (snap_idx >= 0 && snap_idx < n) {
            CacheSnap s{};
            s.top_dirty = result.x87_cache.top_dirty;
            s.tag_push_pending = result.x87_cache.tag_push_pending;
            s.deferred_pop_count = result.x87_cache.deferred_pop_count;
            s.perm_dirty = result.x87_cache.perm_dirty;
            for (int i = 0; i < 8; ++i) {
                s.perm[i] = result.x87_cache.perm[i];
            }
            s.run_remaining = result.x87_cache.run_remaining;
            s.cur_top = -1;  // not directly tracked in cache (top_gpr is a register #)
            snaps[static_cast<size_t>(snap_idx)] = s;
        }
        idx = next.value_or(idx + 1);
    }

    rosetta_set_config(nullptr);

    RunResult out;
    const auto words = static_cast<size_t>(result.insn_buf.end / sizeof(uint32_t));
    out.arm.assign(result.insn_buf.data, result.insn_buf.data + words);
    out.idx_per_word = std::move(idx_per_word);
    out.snap_after_idx = std::move(snaps);
    std::free(result.insn_buf.data);
    return out;
}

void printContext(const char* label, const std::vector<uint32_t>& arm, size_t at_word,
                  size_t window) {
    const size_t lo = at_word > window ? at_word - window : 0;
    const size_t hi = std::min(arm.size(), at_word + window + 1);
    std::printf("  %s [words %zu..%zu of %zu]\n", label, lo, hi - 1, arm.size());
    for (size_t i = lo; i < hi; ++i) {
        const char* mark = (i == at_word) ? " <-- first divergence" : "";
        std::printf("    [%5zu] 0x%08x%s\n", i, arm[i], mark);
    }
}

}  // namespace

int main(int argc, char** argv) try {
    if (argc < 2 || argc > 3) {
        std::fprintf(stderr, "usage: %s <ir_blob> [--determinism]\n", argv[0]);
        std::fprintf(stderr,
                     "  default mode: rollback-OFF vs rollback-ON for top_dirty + deferred_pop\n"
                     "                (the bug branches).\n"
                     "  --determinism: run rollback-OFF twice; should produce zero divergence\n"
                     "                 (sanity-checks harness setup/teardown).\n");
        return 2;
    }
    const char* path = argv[1];
    const bool determinism = (argc == 3 && std::string(argv[2]) == "--determinism");

    std::ifstream f(path, std::ios::binary);
    if (!f) {
        std::fprintf(stderr, "error: cannot open %s\n", path);
        return 1;
    }
    std::vector<uint8_t> raw((std::istreambuf_iterator<char>(f)),
                             std::istreambuf_iterator<char>());
    if (raw.empty() || raw.size() % sizeof(IRInstr) != 0) {
        std::fprintf(stderr, "error: %s size %zu is not a multiple of sizeof(IRInstr)=%zu\n", path,
                     raw.size(), sizeof(IRInstr));
        return 1;
    }
    std::vector<IRInstr> instrs(raw.size() / sizeof(IRInstr));
    std::memcpy(instrs.data(), raw.data(), raw.size());
    std::printf("loaded %zu IRInstr from %s\n", instrs.size(), path);

    // Inline transcendentals materialise an absolute constants address.
    // Any non-zero value satisfies the assertion; the emit count and bytes
    // are stable regardless of the value.
    rosetta_core::set_transcendental_constants_addr(0x1000);

    // Activate the profile machinery so cache.profile_bid gets a real id
    // (instead of kOverflowId).  The rollback gate's block_id range filter
    // rejects kOverflowId even at default bounds (bid >= max).  Two non-zero
    // placeholders satisfy the addr check; we never read or execute the
    // counter bump that gets emitted into insn_buf.
    static uint64_t counter_stub[profile::kMaxBlocks];
    profile::set_counter_array(reinterpret_cast<uint64_t>(counter_stub),
                               reinterpret_cast<uint64_t>(counter_stub));

    // Two configs: shipping defaults vs same + rollback enabled for both
    // investigation branches.  All other knobs equal.  Empty hash lists
    // keep rollback firing whenever the per-branch enable knob is set.
    const auto base_cfg = []() {
        RosettaConfig c{};
        return c;
    };

    RosettaConfig cfg_off = base_cfg();
    cfg_off.x87_enable_rollback_top_dirty = 0;
    cfg_off.x87_enable_rollback_deferred_pop = 0;

    RosettaConfig cfg_on = base_cfg();
    cfg_on.x87_enable_rollback_top_dirty = 1;
    cfg_on.x87_enable_rollback_deferred_pop = 1;

    if (determinism) {
        std::printf("[determinism mode: running rollback-OFF twice]\n");
    }

    const RunResult off = runOnce(instrs, cfg_off);
    const RunResult on = runOnce(instrs, determinism ? cfg_off : cfg_on);

    std::printf("rollback OFF: %zu ARM words\n", off.arm.size());
    std::printf("rollback ON : %zu ARM words\n", on.arm.size());

    // Group emitted ARM words by source IR idx for both runs.  Per-idx
    // comparison answers the question that bare byte-diff cannot: at each
    // IR op, did OFF and ON emit semantically different ARM, or just a
    // length-differing prefix (the speculative-flush rewind that rollback
    // is meant to undo)?
    //
    // Expected shapes per idx:
    //   OFF == ON in bytes        -> no rollback effect at this idx
    //   OFF longer, OFF[suffix] == ON  -> rollback ELIDED a prefix (the speculative
    //                                     emit_store_top / tag-batch in OFF; benign)
    //   ON longer, OR content differs at same length, OR longer/shorter without
    //   suffix-match -> SMOKING GUN: rollback caused the next emit to take a
    //                   DIFFERENT path, not just removed an emit
    auto group_by_idx = [&instrs](const RunResult& r) {
        std::vector<std::vector<uint32_t>> per(instrs.size());
        for (size_t w = 0; w < r.arm.size(); ++w) {
            const int64_t idx = r.idx_per_word[w];
            if (idx >= 0 && static_cast<size_t>(idx) < per.size()) {
                per[static_cast<size_t>(idx)].push_back(r.arm[w]);
            }
        }
        return per;
    };
    const auto off_per = group_by_idx(off);
    const auto on_per = group_by_idx(on);

    size_t same_count = 0;
    size_t benign_rewind_count = 0;  // OFF longer, ON is suffix of OFF
    size_t suspect_count = 0;
    size_t first_suspect = SIZE_MAX;
    for (size_t i = 0; i < instrs.size(); ++i) {
        const auto& a = off_per[i];
        const auto& b = on_per[i];
        if (a == b) {
            ++same_count;
            continue;
        }
        // Check if b is a suffix of a (rollback rewound a's prefix).
        bool suffix_match = false;
        if (a.size() >= b.size()) {
            const size_t prefix = a.size() - b.size();
            suffix_match = std::equal(a.begin() + static_cast<std::ptrdiff_t>(prefix), a.end(),
                                      b.begin());
        }
        if (suffix_match) {
            ++benign_rewind_count;
            continue;
        }
        ++suspect_count;
        if (first_suspect == SIZE_MAX) {
            first_suspect = i;
        }
    }

    std::printf("\nPer-IR-idx classification (n=%zu):\n", instrs.size());
    std::printf("  identical                 : %zu\n", same_count);
    std::printf("  benign-rewind (ON suffix) : %zu\n", benign_rewind_count);
    std::printf("  SUSPECT divergence        : %zu\n", suspect_count);

    // Cache-state divergence: walk through every IR idx and find where
    // post-translate cache state differs between OFF and ON.  This is the
    // smoking-gun question — at every idx, do the two runs maintain the
    // same deferred-flag and permutation state?  If they diverge AND don't
    // re-converge, downstream ops will emit semantically different memory
    // writes that compound into the WoW corruption.
    auto print_snap = [](const char* label, const CacheSnap& s) {
        std::printf("    %s td=%d tp=%d dpc=%d pd=%d run_rem=%d perm=[%d,%d,%d,%d,%d,%d,%d,%d]\n",
                    label, s.top_dirty, s.tag_push_pending, s.deferred_pop_count, s.perm_dirty,
                    s.run_remaining, s.perm[0], s.perm[1], s.perm[2], s.perm[3], s.perm[4],
                    s.perm[5], s.perm[6], s.perm[7]);
    };
    std::printf("\nCache-state divergences across the block:\n");
    size_t state_diff_count = 0;
    int8_t last_off_td = 0, last_on_td = 0;
    for (size_t i = 0; i < instrs.size(); ++i) {
        const CacheSnap& a = off.snap_after_idx[i];
        const CacheSnap& b = on.snap_after_idx[i];
        if (a.run_remaining < 0 && b.run_remaining < 0) {
            continue;
        }
        if (a == b) {
            continue;
        }
        ++state_diff_count;
        if (state_diff_count <= 20) {
            std::printf("  idx=%3zu  op=0x%04x\n", i,
                        static_cast<unsigned>(instrs[i].opcode));
            print_snap("OFF", a);
            print_snap("ON ", b);
        }
        last_off_td = a.top_dirty;
        last_on_td = b.top_dirty;
    }
    std::printf("Total cache-state divergent idx: %zu\n", state_diff_count);
    if (state_diff_count > 0) {
        std::printf("Last seen top_dirty: OFF=%d ON=%d\n", last_off_td, last_on_td);
    }

    if (suspect_count == 0) {
        std::printf("\nVERDICT: every divergence is a clean rollback rewind.  "
                    "ON is exactly OFF with the speculative-flush prefix elided "
                    "at each rollback site.  Bug is cross-block — not in the "
                    "emitted code for this block alone.\n");
        std::printf("Next step: lldb on running WoW with X87_ROLLBACK_HASH_LIST=0x... "
                    "scoping rollback to the geom block; capture cache state vs "
                    "X87State memory truth at each rollback firing.\n");
        return 0;
    }

    std::printf("\nVERDICT: %zu IR positions emit semantically different ARM "
                "between rollback OFF and ON.  The first such position pins "
                "the divergence.\n",
                suspect_count);

    // Print every suspect position in detail.  For each, show the IR op,
    // the ARM emitted in each run.  In a suspect, ON and OFF emit different
    // ARM at the same source IR op — neither is a clean rewind of the
    // other's prefix.
    size_t printed = 0;
    for (size_t i = first_suspect; i < instrs.size(); ++i) {
        const auto& a = off_per[i];
        const auto& b = on_per[i];
        if (a == b) {
            continue;
        }
        bool suffix_match = false;
        if (a.size() >= b.size()) {
            const size_t prefix = a.size() - b.size();
            suffix_match = std::equal(a.begin() + static_cast<std::ptrdiff_t>(prefix), a.end(),
                                      b.begin());
        }
        if (suffix_match) {
            continue;
        }
        ++printed;
        const auto& ins = instrs[i];
        std::printf("\n--- suspect #%zu  IR idx=%zu  opcode=0x%04x  num_operands=%u\n", printed, i,
                    static_cast<unsigned>(ins.opcode),
                    static_cast<unsigned>(ins.num_operands));
        std::printf("  OFF emitted %zu ARM:", a.size());
        for (uint32_t w : a) {
            std::printf(" 0x%08x", w);
        }
        std::printf("\n  ON  emitted %zu ARM:", b.size());
        for (uint32_t w : b) {
            std::printf(" 0x%08x", w);
        }
        std::printf("\n");
    }
    return 0;
} catch (const std::exception& e) {
    std::fprintf(stderr, "uncaught: %s\n", e.what());
    return 1;
}
