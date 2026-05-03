// profile_analyze — offline pattern miner + JIT emit-quality scorer for
// X87_PROFILE dumps.
//
// Reads one or more binary .prof files produced by the sidecar's
// dumpBlockIfNew() (rosetta_loader/src/sidecar.cpp) and emits a CSV of
// adjacent-instruction patterns ranked by their exec-weighted ARM emit
// cost.  Each unique pattern is run through the real translator twice
// (production / IR-disabled) and the emitted ARM64 instruction count is
// reported alongside the workload's runtime path mix from TLY1.
//
// Usage:
//   profile_analyze [--min-exec N] [--max-rows N] [--max-arm-per-x87 R]
//                   [--rank-by exec|emit] file1.prof [file2.prof ...]

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <ios>
#include <iterator>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rosetta_core/Config.h"
#include "rosetta_core/CoreConfig.h"
#include "rosetta_core/IRBlock.h"
#include "rosetta_core/IRInstr.h"
#include "rosetta_core/IROperand.h"
#include "rosetta_core/Opcode.h"
#include "rosetta_core/ProfileFormat.h"
#include "rosetta_core/Register.h"
#include "rosetta_core/ThreadContextOffsets.h"
#include "rosetta_core/TranscendentalHelper.h"
#include "rosetta_core/TranslationResult.h"
#include "rosetta_core/Translator.h"
#include "rosetta_core/X87Cache.h"

namespace {

struct Block {
    uint32_t id;
    uint32_t start_pc;
    std::vector<IRInstr> instrs;
};

struct Tally {
    uint16_t ir = 0;
    uint16_t peep = 0;
    uint16_t single = 0;
    uint16_t ft = 0;
    uint16_t ir_build_fail = 0;
    uint16_t ir_fpr_fail = 0;
    uint16_t ir_gpr_fail = 0;
    uint16_t max_gpr_peak = 0;
    [[nodiscard]] uint64_t total() const {
        return static_cast<uint64_t>(ir) + peep + single + ft + ir_build_fail + ir_fpr_fail +
               ir_gpr_fail;
    }
};

bool readFile(const std::string& path, std::vector<Block>& out, std::vector<uint64_t>& counters,
              std::vector<Tally>& tallies, std::vector<uint16_t>& build_fail_ops,
              std::vector<profile::BlockIRGateCounters>& ir_gate_counters,
              std::vector<uint16_t>& top_dirty_preds,
              std::vector<profile::BlockMaxRunAtRefuse>& max_run_at_refuse) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        std::fprintf(stderr, "error: cannot open %s\n", path.c_str());
        return false;
    }
    std::vector<uint8_t> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    size_t off = 0;
    while (off + sizeof(uint32_t) <= buf.size()) {
        uint32_t lead = 0;
        std::memcpy(&lead, buf.data() + off, sizeof(lead));
        if (lead == profile::kCounterSectionMagic) {
            break;  // start of counter section
        }
        if (off + sizeof(profile::BlockHeader) > buf.size()) {
            break;
        }
        profile::BlockHeader hdr;
        std::memcpy(&hdr, buf.data() + off, sizeof(hdr));
        off += sizeof(hdr);
        const size_t need = static_cast<size_t>(hdr.num_instrs) * sizeof(IRInstr);
        if (off + need > buf.size()) {
            std::fprintf(stderr, "error: truncated IRInstr array in %s at offset %zu\n",
                         path.c_str(), off);
            return false;
        }
        Block b{.id = hdr.block_id, .start_pc = hdr.start_pc, .instrs = {}};
        b.instrs.resize(hdr.num_instrs);
        std::memcpy(b.instrs.data(), buf.data() + off, need);
        off += need;
        out.push_back(std::move(b));
    }

    if (off + sizeof(profile::CounterSectionHeader) > buf.size()) {
        std::fprintf(stderr,
                     "error: %s is incomplete — no counter section found.  The rosettax87 "
                     "(sidecar) process must finish writing the file at parent-exit time; if "
                     "rosettax87 itself was killed (e.g. SIGKILL'd via Activity Monitor) the "
                     "dump never happened.  Parent crashes / SIGKILL are fine — kqueue "
                     "NOTE_EXIT still fires.  Rerun.\n",
                     path.c_str());
        return false;
    }
    profile::CounterSectionHeader chdr;
    std::memcpy(&chdr, buf.data() + off, sizeof(chdr));
    if (chdr.magic != profile::kCounterSectionMagic) {
        std::fprintf(stderr,
                     "error: %s has stray bytes between block records and counter "
                     "section (offset %zu)\n",
                     path.c_str(), off);
        return false;
    }
    off += sizeof(chdr);
    const size_t cbytes = static_cast<size_t>(chdr.count) * sizeof(uint64_t);
    if (off + cbytes > buf.size()) {
        std::fprintf(stderr, "error: %s counter section truncated (declared %u entries)\n",
                     path.c_str(), chdr.count);
        return false;
    }
    counters.resize(chdr.count);
    std::memcpy(counters.data(), buf.data() + off, cbytes);
    off += cbytes;

    // Optional tally section.  Older .prof files (before path-tally
    // instrumentation landed) have nothing past the counter array; treat
    // that as "all-zero tallies" so the analyzer still works.
    if (off + sizeof(profile::TallySectionHeader) <= buf.size()) {
        profile::TallySectionHeader thdr;
        std::memcpy(&thdr, buf.data() + off, sizeof(thdr));
        if (thdr.magic == profile::kTallySectionMagic) {
            off += sizeof(thdr);
            const size_t tbytes =
                static_cast<size_t>(thdr.count) * sizeof(profile::BlockTallyEntry);
            if (off + tbytes > buf.size()) {
                std::fprintf(stderr, "error: %s tally section truncated (declared %u entries)\n",
                             path.c_str(), thdr.count);
                return false;
            }
            tallies.resize(thdr.count);
            for (uint32_t i = 0; i < thdr.count; ++i) {
                profile::BlockTallyEntry e;
                std::memcpy(&e, buf.data() + off + (i * sizeof(e)), sizeof(e));
                tallies[i] = Tally{
                    .ir = e.ir_ops,
                    .peep = e.peephole_ops,
                    .single = e.single_ops,
                    .ft = e.fallthrough_ops,
                    .ir_build_fail = e.ir_build_fail_ops,
                    .ir_fpr_fail = e.ir_fpr_fail_ops,
                    .ir_gpr_fail = e.ir_gpr_fail_ops,
                    .max_gpr_peak = e.max_gpr_peak,
                };
            }
            off += tbytes;
        }
    }

    // Optional build-bail-opcode side-table.  Older .prof files (before the
    // histogram diagnostic landed) lack it; treat absence as "all-sentinel"
    // so the histogram report just prints "no bails recorded".
    if (off + sizeof(profile::BuildFailOpSectionHeader) <= buf.size()) {
        profile::BuildFailOpSectionHeader bhdr;
        std::memcpy(&bhdr, buf.data() + off, sizeof(bhdr));
        if (bhdr.magic == profile::kBuildFailOpSectionMagic) {
            off += sizeof(bhdr);
            const size_t bbytes = static_cast<size_t>(bhdr.count) * sizeof(uint16_t);
            if (off + bbytes > buf.size()) {
                std::fprintf(stderr,
                             "error: %s build-fail-op section truncated (declared %u entries)\n",
                             path.c_str(), bhdr.count);
                return false;
            }
            build_fail_ops.resize(bhdr.count);
            std::memcpy(build_fail_ops.data(), buf.data() + off, bbytes);
            off += bbytes;
        }
    }

    // Optional IR-gate per-reason counter side-table (IRG1).  Older .prof
    // files (before the per-reason instrumentation landed, or with the
    // earlier IRG0 sentinel format) lack it; treat absence as "no refusals
    // recorded" so the histogram + per-pattern columns gracefully degrade.
    if (off + sizeof(profile::IRGateRefuseSectionHeader) <= buf.size()) {
        profile::IRGateRefuseSectionHeader ihdr;
        std::memcpy(&ihdr, buf.data() + off, sizeof(ihdr));
        if (ihdr.magic == profile::kIRGateRefuseSectionMagic) {
            off += sizeof(ihdr);
            const size_t ibytes =
                static_cast<size_t>(ihdr.count) * sizeof(profile::BlockIRGateCounters);
            if (off + ibytes > buf.size()) {
                std::fprintf(stderr,
                             "error: %s ir-gate-counter section truncated (declared %u entries)\n",
                             path.c_str(), ihdr.count);
                return false;
            }
            ir_gate_counters.resize(ihdr.count);
            std::memcpy(ir_gate_counters.data(), buf.data() + off, ibytes);
            off += ibytes;
        }
    }

    // Optional top-dirty predecessor side-table (TDP0).  Older .prof files
    // lack it; treat absence as "all-sentinel" so the histogram report
    // just prints "no predecessors recorded".
    if (off + sizeof(profile::TopDirtyPredSectionHeader) <= buf.size()) {
        profile::TopDirtyPredSectionHeader thdr;
        std::memcpy(&thdr, buf.data() + off, sizeof(thdr));
        if (thdr.magic == profile::kTopDirtyPredSectionMagic) {
            off += sizeof(thdr);
            const size_t tbytes = static_cast<size_t>(thdr.count) * sizeof(uint16_t);
            if (off + tbytes > buf.size()) {
                std::fprintf(stderr,
                             "error: %s top-dirty-pred section truncated (declared %u entries)\n",
                             path.c_str(), thdr.count);
                return false;
            }
            top_dirty_preds.resize(thdr.count);
            std::memcpy(top_dirty_preds.data(), buf.data() + off, tbytes);
            off += tbytes;
        }
    }

    // Optional max-run-at-refusal side-table (RRR0).  Older .prof files
    // lack it; treat absence as "no data".
    if (off + sizeof(profile::MaxRunAtRefuseSectionHeader) <= buf.size()) {
        profile::MaxRunAtRefuseSectionHeader mhdr;
        std::memcpy(&mhdr, buf.data() + off, sizeof(mhdr));
        if (mhdr.magic == profile::kMaxRunAtRefuseSectionMagic) {
            off += sizeof(mhdr);
            const size_t mbytes =
                static_cast<size_t>(mhdr.count) * sizeof(profile::BlockMaxRunAtRefuse);
            if (off + mbytes > buf.size()) {
                std::fprintf(
                    stderr, "error: %s max-run-at-refuse section truncated (declared %u entries)\n",
                    path.c_str(), mhdr.count);
                return false;
            }
            max_run_at_refuse.resize(mhdr.count);
            std::memcpy(max_run_at_refuse.data(), buf.data() + off, mbytes);
            off += mbytes;
        }
    }
    return true;
}

// Convert one operand to a short suffix: m32/m64/m80, abs, i8/i16/i32/i64,
// reg, st(i), cc, seg, br.  Display-only formatting; no JIT counterpart.
const char* sizeStr(IROperandSize s) {
    switch (s) {
        case IROperandSize::S8:
            return "8";
        case IROperandSize::S16:
            return "16";
        case IROperandSize::S32:
            return "32";
        case IROperandSize::S64:
            return "64";
        case IROperandSize::S128:
            return "128";
        case IROperandSize::S256:
            return "256";
        case IROperandSize::S80:
            return "80";
    }
    return "?";
}

void appendOpToken(std::string& s, const IROperand& op) {
    switch (op.kind) {
        case IROperandKind::Register:
            // STi (stack regs are encoded 0x70..0x77) vs gpr.
            if ((op.reg.reg.value & 0xF0) == 0x70) {
                s += "st";
                s += static_cast<char>('0' + (op.reg.reg.value & 0x07));
            } else {
                s += "r";
                s += sizeStr(op.reg.size);
            }
            break;
        case IROperandKind::MemRef:
            s += "m";
            s += sizeStr(op.mem.size);
            break;
        case IROperandKind::AbsMem:
            s += "abs";
            s += sizeStr(op.abs_mem.size);
            break;
        case IROperandKind::Immediate:
            s += "i";
            s += sizeStr(op.imm.size);
            break;
        case IROperandKind::BranchOffset:
            s += "br";
            break;
        case IROperandKind::ConditionCode:
            s += "cc";
            break;
        case IROperandKind::SegmentRegister:
            s += "seg";
            break;
    }
}

std::string opSuffix(const IRInstr& ins) {
    std::string s;
    const uint8_t n = std::min<uint8_t>(ins.num_operands, 4);
    for (uint8_t k = 0; k < n; ++k) {
        s.push_back(k == 0 ? '_' : ',');
        appendOpToken(s, ins.operands[k]);
    }
    return s;
}

std::string mnemonic(uint16_t opcode) {
    if (opcode < kOpcodeNames.size() && kOpcodeNames[opcode] != nullptr) {
        return kOpcodeNames[opcode];
    }
    char buf[16];
    std::snprintf(buf, sizeof(buf), "op%x", opcode);
    return buf;
}

// Pattern key: pipe-separated tokens like "fld_m32|fmul_m32|fstp_m32".
std::string patternKey(const std::vector<IRInstr>& instrs, size_t start, size_t len) {
    std::string s;
    for (size_t i = 0; i < len; ++i) {
        if (i > 0) {
            s.push_back('|');
        }
        const IRInstr& ins = instrs[start + i];
        s += mnemonic(ins.opcode);
        s += opSuffix(ins);
    }
    return s;
}

// ── JIT emit-quality measurement ──────────────────────────────────────
// For each unique pattern, run the translator twice (production / IR
// disabled) and report ARM64-insn emit count.  See the plan for the
// rationale: production is IR-first, --disable-x87-ir is the meaningful
// "without IR" config — three columns would have collapsed into two.
struct EmitMeasurement {
    uint32_t arm_production = 0;
    uint32_t arm_no_ir = 0;
    uint32_t arm_ir_forced = 0;  // gate bypassed via RosettaConfig::force_x87_ir_gate
};

uint32_t runOneMode(const IRInstr* instrs, size_t len, const RosettaConfig* cfg) {
    // emit_x87_base reads thread_context_offsets->x87_state_offset.  We
    // never execute the emitted code, so any small value picks the
    // single-ADD branch and keeps emit count stable.  Static so all
    // translations see the same address/offset (avoids cache-base resets).
    static ThreadContextOffsets stub_tco{};

    TranslationResult result{};
    constexpr size_t kBufSize = 64UL * 1024;
    result.insn_buf.data = static_cast<uint32_t*>(std::malloc(kBufSize));
    result.insn_buf.end = 0;
    result.insn_buf.end_cap = kBufSize;
    result.insn_buf.use_heap = 1;
    result.free_gpr_mask = kGprScratchMask;
    result.free_fpr_mask = kFprScratchMask;
    // The IR-success path restores free_fpr_mask from this field after
    // each whole-run lower (Translator.cpp:156).  In production, stock
    // sets it before invoking our hook; offline we have to seed it
    // ourselves so subsequent translate calls still have FPR scratch.
    result._unoccupied_temporary_fprs_for_xmm_scalars = kFprScratchMask;
    result._pinned_temporary_scalars = 0;
    result.thread_context_offsets = &stub_tco;
    result.translator_variant = 0;

    IRBlock dummy{};  // pointer identity only; cache.invalidate() runs on first call

    rosetta_set_config(cfg);
    // Translator::translate_instruction takes IRInstr* (non-const) but
    // doesn't mutate the array.  Copy to a local mutable buffer.
    std::vector<IRInstr> mut(instrs, instrs + len);
    int64_t idx = 0;
    const auto n = static_cast<int64_t>(len);
    while (idx < n) {
        auto next = Translator::translate_instruction(&result, &dummy, mut.data(), n, idx);
        idx = next.value_or(idx + 1);
    }
    rosetta_set_config(nullptr);

    const auto arm_count = static_cast<uint32_t>(result.insn_buf.end / 4);
    std::free(result.insn_buf.data);
    return arm_count;
}

EmitMeasurement measurePattern(const IRInstr* instrs, size_t len) {
    RosettaConfig no_ir_cfg{};
    no_ir_cfg.disable_x87_ir = 1;
    RosettaConfig force_gate_cfg{};
    force_gate_cfg.force_x87_ir_gate = 1;
    return EmitMeasurement{
        .arm_production = runOneMode(instrs, len, nullptr),
        .arm_no_ir = runOneMode(instrs, len, &no_ir_cfg),
        .arm_ir_forced = runOneMode(instrs, len, &force_gate_cfg),
    };
}

}  // namespace

int main(int argc, char** argv) try {
    uint64_t min_exec = 1000;
    size_t max_rows = 200;
    double max_arm_per_x87 = 0.0;  // 0 = no filter
    enum class RankBy : std::uint8_t { Emit, Exec } rank_by = RankBy::Emit;
    std::vector<std::string> files;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--min-exec" && i + 1 < argc) {
            min_exec = std::strtoull(argv[++i], nullptr, 10);
        } else if (a == "--max-rows" && i + 1 < argc) {
            max_rows = std::strtoull(argv[++i], nullptr, 10);
        } else if (a == "--max-arm-per-x87" && i + 1 < argc) {
            max_arm_per_x87 = std::strtod(argv[++i], nullptr);
        } else if (a == "--rank-by" && i + 1 < argc) {
            const std::string v = argv[++i];
            if (v == "exec") {
                rank_by = RankBy::Exec;
            } else if (v == "emit") {
                rank_by = RankBy::Emit;
            } else {
                std::fprintf(stderr, "unknown --rank-by value: %s (expected exec|emit)\n",
                             v.c_str());
                return 2;
            }
        } else if (a == "--help" || a == "-h") {
            std::printf(
                "Usage: profile_analyze [--min-exec N] [--max-rows N]\n"
                "                       [--max-arm-per-x87 R] [--rank-by exec|emit]\n"
                "                       file1.prof [file2.prof ...]\n"
                "\n"
                "Each unique x87 pattern is run through Translator::translate_instruction\n"
                "three times: once with no Config installed (production: IR-first dispatcher),\n"
                "once with disable_x87_ir=1 (peephole + single-op only), and once with\n"
                "force_x87_ir_gate=1 (gate bypassed — IR called regardless of conditions).\n"
                "Counts are reported as arm_production / arm_no_ir / arm_ir_forced.  The\n"
                "third column answers 'what would IR emit for this pattern if we lifted the\n"
                "gate' — particularly relevant for length-2 patterns and chains where the\n"
                "gate refuses on top_dirty etc.  arm_per_x87 = arm_production / pattern_length.\n"
                "\n"
                "When the input contains a tally section (TLY1), workload-mix columns are\n"
                "printed: ir%% (share of pattern's executions whose containing block had\n"
                "its x87 ops dispatched via the IR pipeline), ft%% (forwarded to stock),\n"
                "and the IR-failure-reason classifiers build_fail%% / fpr_fail%% /\n"
                "gpr_fail%% with max_gpr_peak diagnostic.  peep%% and single%% are now\n"
                "implicit (the residual non-IR non-FT share); the with-IR vs without-IR\n"
                "emit comparison answers 'did peep help' more directly than the path mix.\n"
                "\n"
                "Patterns are sliced within natural x87 runs via X87Cache::lookahead — the\n"
                "JIT's own run-detection function — so every measured pattern is contiguous\n"
                "x87 by construction.\n"
                "\n"
                "Default sort: exec_count * arm_production desc (highest exec-weighted\n"
                "ARM emit first).  --rank-by exec restores raw exec_count ordering.\n"
                "--max-arm-per-x87 R hides rows whose ratio is at or below R (already\n"
                "well-optimized).\n"
                "\n"
                "Caveat: the pattern is measured in isolation.  Block context (cache dirty\n"
                "state, surrounding ops) that determines whether IR's gate refuses in\n"
                "production is not modeled — the TLY1 columns cover that side.  Read both:\n"
                "arm_production/arm_no_ir says 'what's the best each configuration can do\n"
                "for this pattern'; ir%% says 'how often we actually achieve the with-IR\n"
                "result in this workload'.\n");
            return 0;
        } else if (!a.empty() && a[0] == '-') {
            std::fprintf(stderr, "unknown flag: %s\n", a.c_str());
            return 2;
        } else {
            files.push_back(std::move(a));
        }
    }

    if (files.empty()) {
        std::fprintf(stderr, "error: no input files (use --help)\n");
        return 2;
    }

    // Inline transcendentals (fsin/fcos/fpatan/...) materialise an absolute
    // address constant via emit_movz_movk_abs64.  The translator asserts the
    // address has been installed; emit count doesn't depend on the value, so
    // any non-zero placeholder satisfies the gate.
    rosetta_core::set_transcendental_constants_addr(0x1000);

    std::vector<Block> blocks;
    std::vector<uint64_t> counters;
    std::vector<Tally> tallies;
    std::vector<uint16_t> build_fail_ops;
    std::vector<profile::BlockIRGateCounters> ir_gate_counters;
    std::vector<uint16_t> top_dirty_preds;
    std::vector<profile::BlockMaxRunAtRefuse> max_run_at_refuse;
    for (const auto& f : files) {
        std::vector<uint64_t> file_counters;
        std::vector<Tally> file_tallies;
        std::vector<uint16_t> file_bfo;
        std::vector<profile::BlockIRGateCounters> file_irg;
        std::vector<uint16_t> file_tdp;
        std::vector<profile::BlockMaxRunAtRefuse> file_mrr;
        if (!readFile(f, blocks, file_counters, file_tallies, file_bfo, file_irg, file_tdp,
                      file_mrr)) {
            return 1;
        }
        if (counters.empty()) {
            counters = std::move(file_counters);
            tallies = std::move(file_tallies);
            build_fail_ops = std::move(file_bfo);
            ir_gate_counters = std::move(file_irg);
            top_dirty_preds = std::move(file_tdp);
            max_run_at_refuse = std::move(file_mrr);
        } else {
            counters.insert(counters.end(), file_counters.begin(), file_counters.end());
            tallies.insert(tallies.end(), file_tallies.begin(), file_tallies.end());
            build_fail_ops.insert(build_fail_ops.end(), file_bfo.begin(), file_bfo.end());
            ir_gate_counters.insert(ir_gate_counters.end(), file_irg.begin(), file_irg.end());
            top_dirty_preds.insert(top_dirty_preds.end(), file_tdp.begin(), file_tdp.end());
            max_run_at_refuse.insert(max_run_at_refuse.end(), file_mrr.begin(), file_mrr.end());
        }
    }
    const bool have_tallies = !tallies.empty();
    const bool have_bfo = !build_fail_ops.empty();
    const bool have_irg = !ir_gate_counters.empty();
    const bool have_tdp = !top_dirty_preds.empty();
    const bool have_mrr = !max_run_at_refuse.empty();
    std::fprintf(stderr,
                 "loaded %zu blocks, %zu counter entries, %zu tally entries, %zu bfo entries, "
                 "%zu irg entries, %zu tdp entries, %zu mrr entries from %zu file(s)\n",
                 blocks.size(), counters.size(), tallies.size(), build_fail_ops.size(),
                 ir_gate_counters.size(), top_dirty_preds.size(), max_run_at_refuse.size(),
                 files.size());

    // Slice patterns within natural x87 runs.  X87Cache::lookahead is the
    // JIT's own run-detection function (Translator.cpp:74 calls the same).
    // For each block: walk idx; for each x87 run, generate every window
    // length 2..run_len starting at the run's first idx, and accumulate
    // exec-weighted contributions per pattern key.
    struct Stats {
        uint64_t exec_count = 0;
        size_t window_size = 0;
        uint32_t arm_production = 0;
        uint32_t arm_no_ir = 0;
        uint32_t arm_ir_forced = 0;
        // Path-weighted exec contributions; ratios at print time.
        long double ir_w = 0.0L;
        long double ft_w = 0.0L;
        long double build_fail_w = 0.0L;
        long double fpr_fail_w = 0.0L;
        long double gpr_fail_w = 0.0L;
        uint16_t max_gpr_peak = 0;
        // Per-pattern exec-weighted IRG reason attribution.  Each containing
        // block contributes its IRG sentinel × pattern_exec to one bucket.
        // At print time the dominant bucket gives the pattern's gate column.
        long double gate_w[profile::kIRGateReasonCount] = {};
        // Pointer to one occurrence's first IRInstr; the analyzer picks the
        // first sighting and keeps the pointer stable until measurement time.
        const IRInstr* sample_instrs = nullptr;
    };
    std::unordered_map<std::string, Stats> patterns;
    patterns.reserve(blocks.size() * 4);

    for (const auto& b : blocks) {
        const uint64_t exec = (b.id < counters.size()) ? counters[b.id] : 0;
        if (exec == 0) {
            continue;  // dead-code block contributes nothing
        }
        long double ir_frac = 0.0L;
        long double ft_frac = 0.0L;
        long double build_fail_frac = 0.0L;
        long double fpr_fail_frac = 0.0L;
        long double gpr_fail_frac = 0.0L;
        if (b.id < tallies.size()) {
            const auto& t = tallies[b.id];
            const uint64_t tot = t.total();
            if (tot > 0) {
                ir_frac = static_cast<long double>(t.ir) / tot;
                ft_frac = static_cast<long double>(t.ft) / tot;
                build_fail_frac = static_cast<long double>(t.ir_build_fail) / tot;
                fpr_fail_frac = static_cast<long double>(t.ir_fpr_fail) / tot;
                gpr_fail_frac = static_cast<long double>(t.ir_gpr_fail) / tot;
            }
        }
        const auto L = static_cast<int64_t>(b.instrs.size());
        // X87Cache::lookahead expects non-const IRInstr*.  It doesn't
        // mutate; this is just an API quirk.
        auto* mut_instrs = const_cast<IRInstr*>(b.instrs.data());
        int64_t idx = 0;
        while (idx < L) {
            const int run = X87Cache::lookahead(mut_instrs, L, idx);
            if (run < 2) {
                idx += (run == 0) ? 1 : run;
                continue;
            }
            for (int len = 2; len <= run; ++len) {
                std::string key =
                    patternKey(b.instrs, static_cast<size_t>(idx), static_cast<size_t>(len));
                auto& s = patterns[key];
                if (s.exec_count == 0) {
                    s.window_size = static_cast<size_t>(len);
                    s.sample_instrs = &b.instrs[static_cast<size_t>(idx)];
                }
                s.exec_count += exec;
                const auto execL = static_cast<long double>(exec);
                s.ir_w += execL * ir_frac;
                s.ft_w += execL * ft_frac;
                s.build_fail_w += execL * build_fail_frac;
                s.fpr_fail_w += execL * fpr_fail_frac;
                s.gpr_fail_w += execL * gpr_fail_frac;
                if (b.id < tallies.size() && tallies[b.id].max_gpr_peak > s.max_gpr_peak) {
                    s.max_gpr_peak = tallies[b.id].max_gpr_peak;
                }
                if (b.id < ir_gate_counters.size()) {
                    // Per-reason exec-weighted accumulation.  Each pattern
                    // inherits ALL of its containing block's gate counts
                    // weighted by block exec — so a pattern that appears
                    // only in blocks where top_dirty fired sees gate_w
                    // concentrated on top_dirty.
                    const auto& gc = ir_gate_counters[b.id];
                    for (uint16_t r = 0; r < profile::kIRGateReasonCount; ++r) {
                        s.gate_w[r] += execL * static_cast<long double>(gc.counts[r]);
                    }
                }
            }
            idx += run;
        }
    }

    // Filter by min_exec, then run the JIT once per surviving unique
    // pattern.  Measurement is the expensive step (TranslationResult
    // alloc + 64 KB malloc + two translator runs); cap by --max-rows
    // before measuring to bound cost.
    std::vector<std::pair<std::string, Stats>> rows;
    rows.reserve(patterns.size());
    for (auto& kv : patterns) {
        if (kv.second.exec_count < min_exec) {
            continue;
        }
        rows.emplace_back(kv.first, kv.second);
    }

    // Pre-rank by exec_count desc for the bound-by---max-rows pre-measure.
    // After measurement we re-rank by the user's --rank-by choice.
    std::ranges::sort(rows, [](const auto& a, const auto& b) {
        return a.second.exec_count > b.second.exec_count;
    });

    // Cap rows BEFORE measuring so we don't JIT 50k unique patterns.
    // --max-rows N keeps the top N by exec_count for measurement.  After
    // re-ranking, the displayed top-N is the same set ordered by emit
    // leverage if --rank-by emit (default) or by exec if --rank-by exec.
    const size_t to_measure = std::min(max_rows * 4, rows.size());
    rows.resize(to_measure);

    for (auto& [_, s] : rows) {
        if (s.sample_instrs == nullptr || s.window_size == 0) {
            continue;
        }
        const auto m = measurePattern(s.sample_instrs, s.window_size);
        s.arm_production = m.arm_production;
        s.arm_no_ir = m.arm_no_ir;
        s.arm_ir_forced = m.arm_ir_forced;
    }

    // Apply the optional emit-ratio filter post-measurement.
    if (max_arm_per_x87 > 0.0) {
        std::erase_if(rows, [max_arm_per_x87](const auto& kv) {
            const auto& s = kv.second;
            if (s.window_size == 0) {
                return false;
            }
            const double ratio =
                static_cast<double>(s.arm_production) / static_cast<double>(s.window_size);
            return ratio <= max_arm_per_x87;
        });
    }

    // Final sort by --rank-by.
    if (rank_by == RankBy::Emit) {
        std::ranges::sort(rows, [](const auto& a, const auto& b) {
            const auto la = static_cast<long double>(a.second.exec_count) *
                            static_cast<long double>(a.second.arm_production);
            const auto lb = static_cast<long double>(b.second.exec_count) *
                            static_cast<long double>(b.second.arm_production);
            return la > lb;
        });
    } else {
        std::ranges::sort(rows, [](const auto& a, const auto& b) {
            return a.second.exec_count > b.second.exec_count;
        });
    }

    if (have_tallies) {
        std::printf(
            "exec_count,arm_production,arm_no_ir,arm_ir_forced,arm_per_x87,"
            "ir%%,ft%%,build_fail%%,fpr_fail%%,gpr_fail%%,max_gpr_peak,"
            "gate_reason,sequence\n");
    } else {
        std::printf("exec_count,arm_production,arm_no_ir,arm_ir_forced,arm_per_x87,sequence\n");
    }
    const size_t n = std::min(max_rows, rows.size());
    for (size_t i = 0; i < n; ++i) {
        const auto& [key, s] = rows[i];
        const double arm_per_x87 = s.window_size > 0 ? static_cast<double>(s.arm_production) /
                                                           static_cast<double>(s.window_size)
                                                     : 0.0;
        if (have_tallies) {
            const auto exec = static_cast<long double>(s.exec_count);
            const auto pct = [&](long double w) -> double {
                return exec > 0 ? static_cast<double>(100.0L * w / exec) : 0.0;
            };
            // Dominant IRG reason for this pattern (or "-" if none).
            const char* gate_reason = "-";
            long double best = 0.0L;
            for (uint16_t r = 0; r < profile::kIRGateReasonCount; ++r) {
                if (s.gate_w[r] > best) {
                    best = s.gate_w[r];
                    gate_reason = profile::kIRGateReasonNames[r];
                }
            }
            std::printf("%llu,%u,%u,%u,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%s,%s\n",
                        static_cast<unsigned long long>(s.exec_count), s.arm_production,
                        s.arm_no_ir, s.arm_ir_forced, arm_per_x87, pct(s.ir_w), pct(s.ft_w),
                        pct(s.build_fail_w), pct(s.fpr_fail_w), pct(s.gpr_fail_w),
                        static_cast<unsigned>(s.max_gpr_peak), gate_reason, key.c_str());
        } else {
            std::printf("%llu,%u,%u,%u,%.2f,%s\n", static_cast<unsigned long long>(s.exec_count),
                        s.arm_production, s.arm_no_ir, s.arm_ir_forced, arm_per_x87, key.c_str());
        }
    }
    std::fprintf(stderr, "%zu measured patterns >= min_exec=%llu (showing %zu, ranked by %s%s)\n",
                 rows.size(), static_cast<unsigned long long>(min_exec), n,
                 rank_by == RankBy::Emit ? "exec_count*arm_production" : "exec_count",
                 have_tallies
                     ? "; columns ir%/ft%/build_fail%/fpr_fail%/gpr_fail% give workload mix"
                     : "; no tally section in input — workload-mix columns omitted");

    // ── Build-bail-opcode histogram ─────────────────────────────────────────
    // For each block whose translation observed an unsupported x87 opcode in
    // build()'s default arm, attribute counter[bid] to that opcode.  Top-N
    // entries are the highest-leverage opcodes to add to X87IRBuild.cpp's
    // build switch.  Only printed when the .prof contains a BFO0 section.
    if (have_bfo) {
        struct Bucket {
            uint64_t exec_w = 0;
            uint32_t blocks = 0;
        };
        std::unordered_map<uint16_t, Bucket> hist;
        uint64_t total_exec_w = 0;
        for (const auto& b : blocks) {
            if (b.id >= build_fail_ops.size()) {
                continue;
            }
            const uint16_t op = build_fail_ops[b.id];
            if (op == profile::kNoBuildFailOpcode) {
                continue;
            }
            const uint64_t exec = (b.id < counters.size()) ? counters[b.id] : 0;
            if (exec == 0) {
                continue;
            }
            auto& bk = hist[op];
            bk.exec_w += exec;
            bk.blocks += 1;
            total_exec_w += exec;
        }
        std::vector<std::pair<uint16_t, Bucket>> hrows(hist.begin(), hist.end());
        std::ranges::sort(
            hrows, [](const auto& a, const auto& b) { return a.second.exec_w > b.second.exec_w; });
        std::printf("\n");
        std::printf("# Top opcodes blocking X87IR::build() (exec-weighted)\n");
        std::printf("rank,opcode,exec_count,share%%,blocks\n");
        const size_t hn = std::min<size_t>(hrows.size(), 20);
        for (size_t i = 0; i < hn; ++i) {
            const auto& [op, bk] = hrows[i];
            const char* name =
                (op < kOpcodeNames.size() && kOpcodeNames[op] != nullptr) ? kOpcodeNames[op] : "?";
            const double share = total_exec_w > 0 ? 100.0 * static_cast<double>(bk.exec_w) /
                                                        static_cast<double>(total_exec_w)
                                                  : 0.0;
            std::printf("%zu,%s,%llu,%.1f,%u\n", i + 1, name,
                        static_cast<unsigned long long>(bk.exec_w), share, bk.blocks);
        }
        std::fprintf(stderr,
                     "build-bail histogram: %zu distinct opcodes across %llu exec-weighted bails "
                     "(showing top %zu)\n",
                     hrows.size(), static_cast<unsigned long long>(total_exec_w), hn);
    } else {
        std::fprintf(stderr,
                     "build-bail histogram: skipped (no BFO0 section in input — older profile)\n");
    }

    // ── IR-gate per-reason refusal histogram ───────────────────────────────
    // Per-reason exec-weighted bumps from BlockIRGateCounters (IRG1).  Each
    // gate refusal increments a counter; the analyzer multiplies by the
    // block's exec_count to compute exec-weighted contribution.  Per-reason
    // counts (vs a single sentinel) avoid trailing-tail short_run records
    // masking longer-run refusals — every refusal is recorded in its own
    // bucket and contributes to the histogram independently.
    if (have_irg) {
        struct Bucket {
            uint64_t exec_w = 0;   // exec_count × per-block refusal count
            uint32_t blocks = 0;   // distinct blocks recording this reason
            uint16_t max_run = 0;  // max cache.run_remaining at refusal
        };
        Bucket per_reason[profile::kIRGateReasonCount]{};
        uint64_t total_exec_w = 0;
        for (const auto& b : blocks) {
            if (b.id >= ir_gate_counters.size()) {
                continue;
            }
            const uint64_t exec = (b.id < counters.size()) ? counters[b.id] : 0;
            if (exec == 0) {
                continue;
            }
            const auto& gc = ir_gate_counters[b.id];
            for (uint16_t r = 0; r < profile::kIRGateReasonCount; ++r) {
                if (gc.counts[r] == 0) {
                    continue;
                }
                per_reason[r].exec_w += exec * static_cast<uint64_t>(gc.counts[r]);
                per_reason[r].blocks += 1;
                total_exec_w += exec * static_cast<uint64_t>(gc.counts[r]);
                if (have_mrr && b.id < max_run_at_refuse.size()) {
                    const uint16_t mr = max_run_at_refuse[b.id].max_run[r];
                    if (mr > per_reason[r].max_run) {
                        per_reason[r].max_run = mr;
                    }
                }
            }
        }
        struct Row {
            uint16_t reason;
            uint64_t exec_w;
            uint32_t blocks;
            uint16_t max_run;
        };
        std::vector<Row> rows_h;
        for (uint16_t r = 0; r < profile::kIRGateReasonCount; ++r) {
            if (per_reason[r].blocks > 0) {
                rows_h.push_back({.reason = r,
                                  .exec_w = per_reason[r].exec_w,
                                  .blocks = per_reason[r].blocks,
                                  .max_run = per_reason[r].max_run});
            }
        }
        std::ranges::sort(rows_h, [](const Row& a, const Row& b) { return a.exec_w > b.exec_w; });
        std::printf("\n");
        std::printf("# Top reasons IR-gate refused (exec-weighted)\n");
        std::printf("rank,reason,exec_count,share%%,blocks,max_run\n");
        for (size_t i = 0; i < rows_h.size(); ++i) {
            const auto& row = rows_h[i];
            const double share = total_exec_w > 0 ? 100.0 * static_cast<double>(row.exec_w) /
                                                        static_cast<double>(total_exec_w)
                                                  : 0.0;
            std::printf("%zu,%s,%llu,%.1f,%u,%u\n", i + 1, profile::kIRGateReasonNames[row.reason],
                        static_cast<unsigned long long>(row.exec_w), share, row.blocks,
                        static_cast<unsigned>(row.max_run));
        }
        std::fprintf(
            stderr,
            "ir-gate-refusal histogram: %zu distinct reasons across %llu exec-weighted refusals\n",
            rows_h.size(), static_cast<unsigned long long>(total_exec_w));
    } else {
        std::fprintf(
            stderr,
            "ir-gate-refusal histogram: skipped (no IRG0 section in input — older profile)\n");
    }

    // ── Predecessor-of-top_dirty histogram ────────────────────────────────
    // For each block whose top_dirty refusal was observed, attribute the
    // block's exec_count × top_dirty_count to the kOpcodeName_* of the last
    // x87 op translated before that refusal.  Top entries are the highest-
    // leverage opcodes to add an early-flush handler for.
    if (have_tdp) {
        struct Bucket {
            uint64_t exec_w = 0;
            uint32_t blocks = 0;
        };
        std::unordered_map<uint16_t, Bucket> hist;
        uint64_t total_exec_w = 0;
        for (const auto& b : blocks) {
            if (b.id >= top_dirty_preds.size()) {
                continue;
            }
            const uint16_t op = top_dirty_preds[b.id];
            if (op == profile::kNoTopDirtyPredecessor) {
                continue;
            }
            const uint64_t exec = (b.id < counters.size()) ? counters[b.id] : 0;
            if (exec == 0) {
                continue;
            }
            // Weight by per-block top_dirty refusal count when IRG1 is
            // present, otherwise by exec alone.
            uint64_t weight = exec;
            if (have_irg && b.id < ir_gate_counters.size()) {
                const uint16_t td = ir_gate_counters[b.id].counts[profile::kIRGateReasonTopDirty];
                if (td > 0) {
                    weight = exec * static_cast<uint64_t>(td);
                }
            }
            auto& bk = hist[op];
            bk.exec_w += weight;
            bk.blocks += 1;
            total_exec_w += weight;
        }
        std::vector<std::pair<uint16_t, Bucket>> hrows(hist.begin(), hist.end());
        std::ranges::sort(
            hrows, [](const auto& a, const auto& b) { return a.second.exec_w > b.second.exec_w; });
        std::printf("\n");
        std::printf("# Top opcodes preceding top_dirty refusal (exec-weighted)\n");
        std::printf("rank,opcode,exec_count,share%%,blocks\n");
        const size_t hn = std::min<size_t>(hrows.size(), 20);
        for (size_t i = 0; i < hn; ++i) {
            const auto& [op, bk] = hrows[i];
            const char* name =
                (op < kOpcodeNames.size() && kOpcodeNames[op] != nullptr) ? kOpcodeNames[op] : "?";
            const double share = total_exec_w > 0 ? 100.0 * static_cast<double>(bk.exec_w) /
                                                        static_cast<double>(total_exec_w)
                                                  : 0.0;
            std::printf("%zu,%s,%llu,%.1f,%u\n", i + 1, name,
                        static_cast<unsigned long long>(bk.exec_w), share, bk.blocks);
        }
        std::fprintf(stderr,
                     "top-dirty-pred histogram: %zu distinct opcodes across %llu exec-weighted "
                     "refusals (showing top %zu)\n",
                     hrows.size(), static_cast<unsigned long long>(total_exec_w), hn);
    } else {
        std::fprintf(
            stderr,
            "top-dirty-pred histogram: skipped (no TDP0 section in input — older profile)\n");
    }

    // ── Top-N blocks by max_run-at-top_dirty refusal ──────────────────────
    // Picks the blocks where the longest contiguous x87 run was refused
    // for top_dirty.  Useful for sanity-checking that "yes, this is a real
    // workload sequence" before shipping a flush-and-proceed fix.
    if (have_mrr && have_irg) {
        struct R {
            uint32_t bid;
            uint32_t start_pc;
            uint64_t exec;
            uint16_t max_run;
            uint16_t td_count;
            uint32_t block_len;
        };
        std::vector<R> rs;
        rs.reserve(blocks.size());
        for (const auto& b : blocks) {
            if (b.id >= max_run_at_refuse.size() || b.id >= ir_gate_counters.size()) {
                continue;
            }
            const uint16_t mr = max_run_at_refuse[b.id].max_run[profile::kIRGateReasonTopDirty];
            if (mr == 0) {
                continue;
            }
            const uint64_t exec = (b.id < counters.size()) ? counters[b.id] : 0;
            if (exec == 0) {
                continue;
            }
            rs.push_back({.bid = b.id,
                          .start_pc = b.start_pc,
                          .exec = exec,
                          .max_run = mr,
                          .td_count = ir_gate_counters[b.id].counts[profile::kIRGateReasonTopDirty],
                          .block_len = static_cast<uint32_t>(b.instrs.size())});
        }
        // Sort by max_run desc, tie-break by exec desc.
        std::ranges::sort(rs, [](const R& a, const R& b) {
            if (a.max_run != b.max_run) {
                return a.max_run > b.max_run;
            }
            return a.exec > b.exec;
        });
        std::printf("\n");
        std::printf("# Top blocks by max_run at top_dirty refusal\n");
        std::printf("rank,bid,start_pc,max_run,td_refusals,exec_count,block_len\n");
        const size_t n = std::min<size_t>(rs.size(), 10);
        for (size_t i = 0; i < n; ++i) {
            const auto& r = rs[i];
            std::printf("%zu,%u,0x%08x,%u,%u,%llu,%u\n", i + 1, r.bid, r.start_pc, r.max_run,
                        r.td_count, static_cast<unsigned long long>(r.exec), r.block_len);
        }
    }
    return 0;
} catch (const std::exception& e) {
    std::fprintf(stderr, "profile_analyze: %s\n", e.what());
    return 1;
} catch (...) {
    std::fprintf(stderr, "profile_analyze: unknown exception\n");
    return 1;
}
