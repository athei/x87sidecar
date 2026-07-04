// ir_pressure_replay -- replay a captured x87 IR block through Translator
// under a clamped register-pressure gate and report how the run fared:
// dispatch-path tallies, pressure refusals, splits, and total ARM emit.
//
// Usage:
//   ir_pressure_replay <ir_blob> [--fpr-pool N] [--gpr-pool N] [--no-split] [--log]
//
// <ir_blob> is a raw IRInstr[] file, no header.  Produced by
//   profile_analyze --dump-ir-binary 0xH out.ir <profile.prof>
//
// The canonical pressure reproducer ships under
// tests/data/geom_block_874c40.ir (169 instrs, WoW geometry; historically
// 29 FprPressure firings at the 8-slot pool).  A/B example:
//
//   ir_pressure_replay tests/data/geom_block_874c40.ir --fpr-pool 8 --runtime-version 0 --no-split
//   ir_pressure_replay tests/data/geom_block_874c40.ir --fpr-pool 8 --runtime-version 0
//
// (--runtime-version 0 because the geom blob is a legacy capture with
// 26.4-host opcodes; modern captures use the default identity regime.)
//
// The first shows the all-or-nothing gate (fpr_fail ops, high ARM); the
// second shows the split loop converting those refusals into shorter IR
// runs.  Exit code 0 always (reporting tool; assertions live in tests).

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <ios>
#include <iterator>
#include <string>
#include <vector>

#include "rosetta_core/Config.h"
#include "rosetta_core/CoreConfig.h"
#include "rosetta_core/IRBlock.h"
#include "rosetta_core/IRInstr.h"
#include "rosetta_core/Opcode_26_4.h"
#include "rosetta_core/ProfileRuntime.h"
#include "rosetta_core/Register.h"
#include "rosetta_core/RosettaCore.h"
#include "rosetta_core/ThreadContextOffsets.h"
#include "rosetta_core/TranscendentalHelper.h"
#include "rosetta_core/TranslationResult.h"
#include "rosetta_core/Translator.h"
#include "rosetta_core/X87Cache.h"

namespace {

struct ReplayStats {
    size_t arm_words = 0;
    uint16_t tally_ir = 0;
    uint16_t tally_peep = 0;
    uint16_t tally_single = 0;
    uint16_t tally_ft = 0;
    uint16_t tally_ir_build_fail = 0;
    uint16_t tally_ir_fpr_fail = 0;
    uint16_t tally_ir_gpr_fail = 0;
    uint16_t tally_ir_split = 0;
    uint16_t tally_ir_remat = 0;
    uint16_t tally_max_gpr_peak = 0;
};

ReplayStats runOnce(const std::vector<IRInstr>& instrs, const RosettaConfig& cfg) {
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
    int64_t idx = 0;
    const auto n = static_cast<int64_t>(mut.size());
    while (idx < n) {
        const auto next = Translator::translate_instruction(&result, &dummy, mut.data(), n, idx);
        idx = next.value_or(idx + 1);
    }

    rosetta_set_config(nullptr);

    ReplayStats s;
    s.arm_words = static_cast<size_t>(result.insn_buf.end / sizeof(uint32_t));
    s.tally_ir = result.x87_cache.tally_ir;
    s.tally_peep = result.x87_cache.tally_peep;
    s.tally_single = result.x87_cache.tally_single;
    s.tally_ft = result.x87_cache.tally_ft;
    s.tally_ir_build_fail = result.x87_cache.tally_ir_build_fail;
    s.tally_ir_fpr_fail = result.x87_cache.tally_ir_fpr_fail;
    s.tally_ir_gpr_fail = result.x87_cache.tally_ir_gpr_fail;
    s.tally_ir_split = result.x87_cache.tally_ir_split;
    s.tally_ir_remat = result.x87_cache.tally_ir_remat;
    s.tally_max_gpr_peak = result.x87_cache.tally_max_gpr_peak;
    std::free(result.insn_buf.data);
    return s;
}

}  // namespace

int main(int argc, char** argv) try {
    const char* path = nullptr;
    int fpr_pool = 0;
    int gpr_pool = 0;
    bool no_split = false;
    bool no_remat = false;
    bool log_split = false;
    uint64_t runtime_version_arg = 0;
    bool runtime_version_arg_set = false;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--fpr-pool" && i + 1 < argc) {
            fpr_pool = static_cast<int>(std::strtol(argv[++i], nullptr, 10));
        } else if (a == "--gpr-pool" && i + 1 < argc) {
            gpr_pool = static_cast<int>(std::strtol(argv[++i], nullptr, 10));
        } else if (a == "--no-split") {
            no_split = true;
        } else if (a == "--no-remat") {
            no_remat = true;
        } else if (a == "--runtime-version" && i + 1 < argc) {
            runtime_version_arg = std::strtoull(argv[++i], nullptr, 0);
            runtime_version_arg_set = true;
        } else if (a == "--log") {
            log_split = true;
        } else if (path == nullptr) {
            path = argv[i];
        } else {
            std::fprintf(stderr,
                         "usage: %s <ir_blob> [--fpr-pool N] [--gpr-pool N] [--no-split] [--log]\n",
                         argv[0]);
            return 2;
        }
    }
    if (path == nullptr) {
        std::fprintf(stderr,
                     "usage: %s <ir_blob> [--fpr-pool N] [--gpr-pool N] [--no-split] [--log]\n",
                     argv[0]);
        return 2;
    }

    std::ifstream f(path, std::ios::binary);
    if (!f) {
        std::fprintf(stderr, "error: cannot open %s\n", path);
        return 1;
    }
    std::vector<uint8_t> raw((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (raw.empty() || raw.size() % sizeof(IRInstr) != 0) {
        std::fprintf(stderr, "error: %s size %zu is not a multiple of sizeof(IRInstr)=%zu\n", path,
                     raw.size(), sizeof(IRInstr));
        return 1;
    }
    std::vector<IRInstr> instrs(raw.size() / sizeof(IRInstr));
    std::memcpy(instrs.data(), raw.data(), raw.size());

    // Seed the runtime opcode regime BEFORE any IRInstr::opcode() call.
    // The unseeded default (0) selects the ≤26.4 compat table, which
    // mis-decodes blobs captured under modern Rosetta (e.g. internal fld
    // 0xDE reads as 26.4-host fisubr) — same gotcha profile_analyze fixed.
    // Blobs come from profile_analyze --dump-ir-binary of modern captures,
    // so default to the identity regime; pass --runtime-version 0 for a
    // legacy ≤26.4 blob.
    uint64_t runtime_version = kVersion_26_4 + 1;
    // (parsed above if --runtime-version was given)
    rosetta_core_set_runtime_version(runtime_version_arg_set ? runtime_version_arg
                                                             : runtime_version);

    // Inline transcendentals materialise an absolute constants address; any
    // non-zero value keeps emit counts stable.
    rosetta_core::set_transcendental_constants_addr(0x1000);

    // Activate the profile machinery so cache.profile_bid is a real id and
    // the tallies mirror; we never execute the emitted counter bump.
    static uint64_t counter_stub[profile::kMaxBlocks];
    profile::set_counter_array(reinterpret_cast<uint64_t>(counter_stub),
                               reinterpret_cast<uint64_t>(counter_stub));

    RosettaConfig cfg{};
    cfg.enable_fma_reduce = 1;  // production defaults
    cfg.x87_enable_rollback_top_dirty = 1;
    cfg.x87_enable_rollback_deferred_pop = 1;
    cfg.enable_bridge = 1;
    cfg.bridge_max_gap = 2;
    cfg.bridge_max_total = 8;
    cfg.enable_ir_split = no_split ? 0 : 1;
    cfg.enable_ir_remat = (no_split || no_remat) ? 0 : 1;
    cfg.log_ir_split = log_split ? 1 : 0;
    cfg.fpr_pool_limit = static_cast<uint8_t>(fpr_pool);
    cfg.gpr_pool_limit = static_cast<uint8_t>(gpr_pool);

    const ReplayStats s = runOnce(instrs, cfg);

    std::printf("instrs=%zu fpr_pool=%s gpr_pool=%s split=%s\n", instrs.size(),
                fpr_pool > 0 ? std::to_string(fpr_pool).c_str() : "full",
                gpr_pool > 0 ? std::to_string(gpr_pool).c_str() : "full",
                no_split ? "off" : "on");
    std::printf("arm_words,%zu\n", s.arm_words);
    std::printf("ir,%u\n", s.tally_ir);
    std::printf("peep,%u\n", s.tally_peep);
    std::printf("single,%u\n", s.tally_single);
    std::printf("ft,%u\n", s.tally_ft);
    std::printf("ir_build_fail,%u\n", s.tally_ir_build_fail);
    std::printf("ir_fpr_fail,%u\n", s.tally_ir_fpr_fail);
    std::printf("ir_gpr_fail,%u\n", s.tally_ir_gpr_fail);
    std::printf("ir_split,%u\n", s.tally_ir_split);
    std::printf("ir_remat,%u\n", s.tally_ir_remat);
    std::printf("max_gpr_peak,%u\n", s.tally_max_gpr_peak);
    return 0;
} catch (const std::exception& e) {
    std::fprintf(stderr, "fatal: %s\n", e.what());
    return 1;
}
