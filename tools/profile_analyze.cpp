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
#include "rosetta_core/Opcode_26_4.h"
#include "rosetta_core/ProfileFormat.h"
#include "rosetta_core/ProfileRuntime.h"
#include "rosetta_core/Register.h"
#include "rosetta_core/RosettaCore.h"
#include "rosetta_core/ThreadContextOffsets.h"
#include "rosetta_core/TranscendentalHelper.h"
#include "rosetta_core/TranslationResult.h"
#include "rosetta_core/Translator.h"
#include "rosetta_core/X87Bridge.h"
#include "rosetta_core/X87Cache.h"
#include "rosetta_core/X87IR.h"

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
    uint16_t ir_split = 0;     // runs rescued by pressure splitting (TLY2+)
    uint16_t ir_remat = 0;     // runs relieved by remat/sink (TLY2+)
    uint16_t bridge = 0;       // bridge instrs consumed in IR runs (TLY3+)
    uint16_t bridge_fail = 0;  // bridged attempts that fell back (TLY3+)
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
            // Current 'TLY3' format (24-byte entries with pressure-relief
            // and run-bridging attribution).
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
                    .ir_split = e.ir_split_runs,
                    .ir_remat = e.ir_remat_runs,
                    .bridge = e.bridge_ops,
                    .bridge_fail = e.bridge_fail_runs,
                };
            }
            off += tbytes;
        } else if (thdr.magic == profile::kTallySectionMagicV2) {
            // Legacy 'TLY2' capture: 20-byte entries; bridging columns
            // zero-filled.
            off += sizeof(thdr);
            const size_t tbytes =
                static_cast<size_t>(thdr.count) * sizeof(profile::BlockTallyEntryV2);
            if (off + tbytes > buf.size()) {
                std::fprintf(stderr, "error: %s tally section truncated (declared %u entries)\n",
                             path.c_str(), thdr.count);
                return false;
            }
            tallies.resize(thdr.count);
            for (uint32_t i = 0; i < thdr.count; ++i) {
                profile::BlockTallyEntryV2 e;
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
                    .ir_split = e.ir_split_runs,
                    .ir_remat = e.ir_remat_runs,
                };
            }
            off += tbytes;
        } else if (thdr.magic == profile::kTallySectionMagicV1) {
            // Legacy 'TLY1' capture: 16-byte entries; pressure-relief
            // columns zero-filled.
            off += sizeof(thdr);
            const size_t tbytes =
                static_cast<size_t>(thdr.count) * sizeof(profile::BlockTallyEntryV1);
            if (off + tbytes > buf.size()) {
                std::fprintf(stderr, "error: %s tally section truncated (declared %u entries)\n",
                             path.c_str(), thdr.count);
                return false;
            }
            tallies.resize(thdr.count);
            for (uint32_t i = 0; i < thdr.count; ++i) {
                profile::BlockTallyEntryV1 e;
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
                    .ir_split = 0,
                    .ir_remat = 0,
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
        s += mnemonic(ins.opcode());
        s += opSuffix(ins);
    }
    return s;
}

// ── JIT emit-quality measurement ──────────────────────────────────────
// For each unique pattern, run the translator twice (production / IR
// disabled) and report ARM64-insn emit count.  See the plan for the
// rationale: production is IR-first, X87_DISABLE_X87_IR=1 is the meaningful
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

// Production-equivalent config: matches what `load_config_from_env()`
// produces when the user launches with no env overrides.  Knobs that
// default ON (env_default_on) get 1; knobs that default OFF (env_truthy)
// stay 0.  Use this for the "production" measurement so its emit count
// reflects what the JIT actually produces in default operation, rather
// than the all-zero-cfg legacy baseline.
RosettaConfig make_production_cfg() {
    RosettaConfig cfg{};
    cfg.enable_fma_reduce = 1;
    cfg.enable_ir_split = 1;
    cfg.enable_ir_remat = 1;
    cfg.enable_bridge = 1;
    cfg.enable_bridge_v2 = 1;
    cfg.bridge_max_gap = 2;
    cfg.bridge_max_total = 8;
    // Rollback knobs default ON too but only matter to Translator's gate
    // cascade, which measurePattern doesn't exercise (it builds a fresh
    // x87 cache per call); leave them 0 here.
    return cfg;
}

EmitMeasurement measurePattern(const IRInstr* instrs, size_t len) {
    RosettaConfig prod_cfg = make_production_cfg();
    RosettaConfig no_ir_cfg{};
    no_ir_cfg.disable_x87_ir = 1;
    RosettaConfig force_gate_cfg{};
    force_gate_cfg.force_x87_ir_gate = 1;
    return EmitMeasurement{
        .arm_production = runOneMode(instrs, len, &prod_cfg),
        .arm_no_ir = runOneMode(instrs, len, &no_ir_cfg),
        .arm_ir_forced = runOneMode(instrs, len, &force_gate_cfg),
    };
}

}  // namespace

int main(int argc, char** argv) try {
    uint64_t min_exec = 1000;
    size_t max_rows = 200;
    size_t hot_addrs = 100;  // rows in the hot-address section; 0 = suppress
    size_t frag_rows = 30;   // rows in the FP-run-fragmentation section; 0 = suppress
    double max_arm_per_x87 = 0.0;  // 0 = no filter
    enum class RankBy : std::uint8_t { Emit, Exec } rank_by = RankBy::Emit;
    std::vector<std::string> files;
    std::vector<uint32_t> dump_block_ids;
    std::vector<uint64_t> dump_block_hashes;
    uint64_t dump_ir_binary_hash = 0;
    bool dump_ir_binary_set = false;
    std::string dump_ir_binary_path;
    // Rosetta runtime version the profile was captured under.  The sidecar's
    // loader seeds rosetta_core with the live Rosetta version (main.cpp), which
    // drives opcode_host_to_internal(): for versions > 26.4 the host opcode IS
    // already the internal kOpcodeName_* value (identity), while <= 26.4 host
    // opcodes are remapped through the 26.4 compat table.  profile_analyze runs
    // in its own process, so it must seed the same version — otherwise the
    // identity-encoded opcodes in a modern (.prof captured under > 26.4) file
    // get wrongly remapped (e.g. internal fld 0xDE is read as 26.4-host
    // fisubr 0xDE), mis-dispatching and corrupting the whole analysis.  The
    // .prof carries no version field, so default to the current regime
    // (> 26.4, identity).  Pass --runtime-version 0 (or any value <= 26.4) to
    // analyze a legacy profile captured under Rosetta <= 26.4.
    uint64_t runtime_version = kVersion_26_4 + 1;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--min-exec" && i + 1 < argc) {
            min_exec = std::strtoull(argv[++i], nullptr, 10);
        } else if (a == "--max-rows" && i + 1 < argc) {
            max_rows = std::strtoull(argv[++i], nullptr, 10);
        } else if (a == "--hot-addrs" && i + 1 < argc) {
            hot_addrs = static_cast<size_t>(std::strtoull(argv[++i], nullptr, 10));
        } else if (a == "--frag-rows" && i + 1 < argc) {
            frag_rows = static_cast<size_t>(std::strtoull(argv[++i], nullptr, 10));
        } else if (a == "--max-arm-per-x87" && i + 1 < argc) {
            max_arm_per_x87 = std::strtod(argv[++i], nullptr);
        } else if (a == "--dump-block" && i + 1 < argc) {
            // Comma-separated list of block_ids to dump verbatim.  Skips
            // pattern analysis and exits after dumping.
            std::string list = argv[++i];
            size_t start = 0;
            while (start <= list.size()) {
                const size_t comma = list.find(',', start);
                const std::string tok = list.substr(
                    start, comma == std::string::npos ? std::string::npos : comma - start);
                if (!tok.empty()) {
                    dump_block_ids.push_back(
                        static_cast<uint32_t>(std::strtoul(tok.c_str(), nullptr, 10)));
                }
                if (comma == std::string::npos) {
                    break;
                }
                start = comma + 1;
            }
        } else if (a == "--dump-block-by-hash" && i + 1 < argc) {
            // Comma-separated 64-bit hex IR-content hashes (with or without
            // "0x" prefix).  Hash matches profile::hash_ir_stream, so the
            // value is stable across WoW launches — the right key for
            // bisect-target lookup.  Skips pattern analysis and exits.
            std::string list = argv[++i];
            size_t start = 0;
            while (start <= list.size()) {
                const size_t comma = list.find(',', start);
                std::string tok = list.substr(
                    start, comma == std::string::npos ? std::string::npos : comma - start);
                if (!tok.empty()) {
                    const char* p = tok.c_str();
                    if (tok.size() > 2 && p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
                        p += 2;
                    }
                    dump_block_hashes.push_back(std::strtoull(p, nullptr, 16));
                }
                if (comma == std::string::npos) {
                    break;
                }
                start = comma + 1;
            }
        } else if (a == "--dump-ir-binary" && i + 2 < argc) {
            // --dump-ir-binary 0xH out_path: write the matching block's
            // raw IRInstr[] to out_path (no header).  Used by
            // tests/test_rollback_geom_diff to re-feed a captured IR
            // stream into the in-process Translator.
            const char* p = argv[++i];
            if (std::strlen(p) > 2 && p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
                p += 2;
            }
            dump_ir_binary_hash = std::strtoull(p, nullptr, 16);
            dump_ir_binary_path = argv[++i];
            dump_ir_binary_set = true;
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
        } else if (a == "--runtime-version" && i + 1 < argc) {
            // Accepts decimal or 0x-prefixed hex (strtoull base 0).
            runtime_version = std::strtoull(argv[++i], nullptr, 0);
        } else if (a == "--help" || a == "-h") {
            std::printf(
                "Usage: profile_analyze [--min-exec N] [--max-rows N] [--hot-addrs N]\n"
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
                "Hot-address section (when a TLY1 section is present):\n"
                "  --hot-addrs N           Print the top N hottest guest x86 block-entry\n"
                "                          addresses (start_pc), collapsed by address and\n"
                "                          ranked by exec-weighted ARM emit (cost).  Each\n"
                "                          row carries cost, share%%, exec, x87_ops, the\n"
                "                          number of blocks sharing the address, and the\n"
                "                          constituent block_ids (for --dump-block).  These\n"
                "                          are basic-block entries, not functions; map each\n"
                "                          address to its containing function in a Ghidra\n"
                "                          disassembly and sum cost per function.  Default\n"
                "                          100; pass 0 to suppress the section.\n"
                "\n"
                "FP-run fragmentation section (when a TLY1 section is present):\n"
                "  --frag-rows N           Print the top N blocks whose x87 runs are split\n"
                "                          by short gaps of bridgeable integer instructions\n"
                "                          (mov/lea = v1; +add/sub/inc/dec/and/or/xor with\n"
                "                          the flag_liveness==0 dead-flags proof = v2),\n"
                "                          ranked by the exec-weighted ARM that joining the\n"
                "                          v1 gaps would save (measured by re-translating\n"
                "                          the block with eligible gaps spliced out, minus\n"
                "                          2 ARM per bridged instruction for our own emit).\n"
                "                          bridge_v2_savings_pct sizes v1+proven-v2 the\n"
                "                          same way.  Feeds go/no-go on run bridging.\n"
                "                          Default 30; pass 0 to suppress the section.\n"
                "\n"
                "Caveat: the pattern is measured in isolation.  Block context (cache dirty\n"
                "state, surrounding ops) that determines whether IR's gate refuses in\n"
                "production is not modeled — the TLY1 columns cover that side.  Read both:\n"
                "arm_production/arm_no_ir says 'what's the best each configuration can do\n"
                "for this pattern'; ir%% says 'how often we actually achieve the with-IR\n"
                "result in this workload'.\n"
                "\n"
                "Block dump mode:\n"
                "  --dump-block N1,N2,N3   Print the IR stream for the listed block_ids\n"
                "                          (verbose form: insn_idx, opcode_name, operand\n"
                "                          summary, original x86 PC).  Useful with the\n"
                "                          X87_LOG_ROLLBACK / X87_LOG_OPS lines to lift\n"
                "                          the surrounding instructions of a hot block.\n"
                "                          Skips pattern analysis and exits after dump.\n"
                "  --dump-block-by-hash 0xH1,0xH2  Same, but lookup is by IR-content hash\n"
                "                          (FNV-1a, PC zeroed) — stable across WoW launches\n"
                "                          while block_id is registration-order-dependent.\n"
                "                          Hash is shown in the header of every dumped\n"
                "                          block, and printed in the [rollback] log line.\n"
                "  --dump-ir-binary 0xH P  Write the matching block's raw IRInstr[] (no\n"
                "                          header) to path P.  Used by tests that re-feed\n"
                "                          a captured IR stream into the Translator.\n"
                "\n"
                "  --runtime-version V     Rosetta version the profile was captured under,\n"
                "                          driving opcode_host_to_internal().  Default is\n"
                "                          the current regime (> 26.4: host opcodes are\n"
                "                          already internal).  Pass 0 (or any value <= 26.4)\n"
                "                          for a legacy profile captured under Rosetta\n"
                "                          <= 26.4.  Decimal or 0x-hex.\n");
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

    // Seed the Rosetta runtime version before any IRInstr::opcode() call, so
    // opcode_host_to_internal() interprets the dumped opcodes the same way the
    // sidecar did when it captured them (see runtime_version comment above).
    rosetta_core_set_runtime_version(runtime_version);
    std::fprintf(stderr, "runtime version: 0x%llx (%s)\n",
                 static_cast<unsigned long long>(runtime_version),
                 runtime_version > kVersion_26_4 ? "> 26.4: opcodes identity-mapped"
                                                 : "<= 26.4: opcodes remapped via compat table");

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
    // --dump-block / --dump-block-by-hash: print requested blocks' IR
    // streams and exit.  The header shows both block_id AND ir_hash so a
    // user can take an id from one capture and feed the hash back to a
    // future bisect run, or vice versa.
    const auto print_block = [&](const Block& b) {
        const uint64_t exec = static_cast<size_t>(b.id) < counters.size() ? counters[b.id] : 0ULL;
        const uint64_t ir_hash = profile::hash_ir_stream(b.instrs.data(), b.instrs.size());
        std::printf(
            "# block_id=%u hash=0x%016llx start_pc=0x%08x num_instrs=%zu "
            "exec_count=%llu\n",
            b.id, static_cast<unsigned long long>(ir_hash), b.start_pc, b.instrs.size(),
            static_cast<unsigned long long>(exec));
        for (size_t i = 0; i < b.instrs.size(); ++i) {
            const IRInstr& ins = b.instrs[i];
            const char* name =
                (ins.opcode() < kOpcodeNames.size() && kOpcodeNames[ins.opcode()] != nullptr)
                    ? kOpcodeNames[ins.opcode()]
                    : "?";
            std::printf("  [%3zu] pc=0x%08x op=0x%04x %-12s fl=0x%02x nops=%u", i,
                        static_cast<unsigned>(ins.pc), static_cast<unsigned>(ins.opcode()), name,
                        static_cast<unsigned>(ins.flag_liveness),
                        static_cast<unsigned>(ins.num_operands));
            const int nops = std::min<int>(ins.num_operands, 4);
            for (int op_idx = 0; op_idx < nops; ++op_idx) {
                const IROperand& o = ins.operands[op_idx];
                const char* kn = "?";
                switch (o.kind) {
                    case IROperandKind::Register:
                        kn = "Reg";
                        break;
                    case IROperandKind::MemRef:
                        kn = "Mem";
                        break;
                    case IROperandKind::AbsMem:
                        kn = "AbsMem";
                        break;
                    case IROperandKind::Immediate:
                        kn = "Imm";
                        break;
                    case IROperandKind::BranchOffset:
                        kn = "Brn";
                        break;
                    case IROperandKind::ConditionCode:
                        kn = "CC";
                        break;
                    case IROperandKind::SegmentRegister:
                        kn = "Seg";
                        break;
                }
                std::printf(" [%d:%s", op_idx, kn);
                if (o.kind == IROperandKind::Register) {
                    std::printf(" s=0x%02x reg=%d", static_cast<uint8_t>(o.reg.size),
                                o.reg.reg.index());
                } else if (o.kind == IROperandKind::MemRef) {
                    std::printf(" s=0x%02x base=%d idx=%d disp=%lld",
                                static_cast<uint8_t>(o.mem.size), static_cast<int>(o.mem.base_reg),
                                static_cast<int>(o.mem.index_reg),
                                static_cast<long long>(o.mem.disp));
                } else if (o.kind == IROperandKind::Immediate) {
                    std::printf(" s=0x%02x v=0x%llx", static_cast<uint8_t>(o.imm.size),
                                static_cast<unsigned long long>(o.imm.value));
                } else if (o.kind == IROperandKind::AbsMem) {
                    std::printf(" s=0x%02x v=0x%llx", static_cast<uint8_t>(o.abs_mem.size),
                                static_cast<unsigned long long>(o.abs_mem.value));
                } else if (o.kind == IROperandKind::BranchOffset) {
                    std::printf(" v=0x%llx", static_cast<unsigned long long>(o.branch.value));
                } else if (o.kind == IROperandKind::ConditionCode) {
                    std::printf(" cc=%u", static_cast<unsigned>(o.cc.cc));
                } else if (o.kind == IROperandKind::SegmentRegister) {
                    std::printf(" seg=%u", static_cast<unsigned>(o.seg.seg_idx));
                }
                std::printf("]");
            }
            std::printf("\n");
        }
    };
    if (dump_ir_binary_set) {
        const Block* match = nullptr;
        for (const Block& b : blocks) {
            if (profile::hash_ir_stream(b.instrs.data(), b.instrs.size()) == dump_ir_binary_hash) {
                match = &b;
                break;
            }
        }
        if (match == nullptr) {
            std::fprintf(stderr, "error: hash 0x%016llx not found in profile\n",
                         static_cast<unsigned long long>(dump_ir_binary_hash));
            return 1;
        }
        std::ofstream out(dump_ir_binary_path, std::ios::binary | std::ios::trunc);
        if (!out) {
            std::fprintf(stderr, "error: cannot open %s for write\n", dump_ir_binary_path.c_str());
            return 1;
        }
        const size_t bytes = match->instrs.size() * sizeof(IRInstr);
        out.write(reinterpret_cast<const char*>(match->instrs.data()),
                  static_cast<std::streamsize>(bytes));
        if (!out) {
            std::fprintf(stderr, "error: write failed for %s\n", dump_ir_binary_path.c_str());
            return 1;
        }
        std::fprintf(stderr, "wrote %zu IRInstr (%zu bytes) for hash 0x%016llx to %s\n",
                     match->instrs.size(), bytes,
                     static_cast<unsigned long long>(dump_ir_binary_hash),
                     dump_ir_binary_path.c_str());
        return 0;
    }
    if (!dump_block_ids.empty() || !dump_block_hashes.empty()) {
        for (const uint32_t want : dump_block_ids) {
            const auto it =
                std::ranges::find_if(blocks, [want](const Block& b) { return b.id == want; });
            if (it == blocks.end()) {
                std::printf("# block_id=%u: NOT FOUND in profile\n", want);
                continue;
            }
            print_block(*it);
        }
        for (const uint64_t want : dump_block_hashes) {
            bool found = false;
            for (const Block& b : blocks) {
                const uint64_t h = profile::hash_ir_stream(b.instrs.data(), b.instrs.size());
                if (h == want) {
                    print_block(b);
                    found = true;
                }
            }
            if (!found) {
                std::printf("# hash=0x%016llx: NOT FOUND in profile\n",
                            static_cast<unsigned long long>(want));
            }
        }
        return 0;
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

    // Workload-wide x87 op composition.  Computed at the BLOCK level (not
    // per-pattern), so no double-counting from window slicing.  Each x87
    // op in a block is counted once via the path that translated it
    // (ir/peep/single/ft).  Refusals are call-counts (not ops), so they
    // are normalized per 1000 ops to be comparable.  Plus: total ARM
    // emit / total x87 op = global emit-per-op ratio, the actual perf
    // metric (lower = better).  This is the single most useful number
    // for tracking the impact of changes across captures of different
    // durations / different play sessions.
    if (have_tallies) {
        // Per-block ARM emit measurement, in production mode (matches JIT
        // env defaults).  Each block's full IR stream is fed to translate_
        // instruction (same machinery as the per-pattern measurement);
        // non-x87 ops return nullopt and skip without emit, so the ARM
        // count reflects only x87-attributable emit.
        //
        // The companion `total_arm_no_fma_reduce` measurement reuses the
        // same loop with a config that disables the FMA-reduce pass, so
        // we can quantify how much that one pass contributes to the
        // workload.  Pattern is general — extend with another column for
        // any future pass we want to measure delta on.
        const RosettaConfig prod_cfg = make_production_cfg();
        RosettaConfig no_fma_cfg = prod_cfg;
        no_fma_cfg.enable_fma_reduce = 0;
        RosettaConfig no_relief_cfg = prod_cfg;
        no_relief_cfg.enable_ir_split = 0;
        no_relief_cfg.enable_ir_remat = 0;
        // Run bridging defaults ON; the companion measures what turning it
        // OFF would cost (mirrors the FMA-reduce/pressure-relief pattern).
        RosettaConfig bridge_cfg = prod_cfg;
        bridge_cfg.enable_bridge = 0;
        long double total_arm = 0;
        long double total_arm_no_fma_reduce = 0;
        long double total_arm_no_relief = 0;
        long double total_arm_bridged = 0;
        for (const auto& b : blocks) {
            const uint64_t exec = (b.id < counters.size()) ? counters[b.id] : 0;
            if (exec == 0 || b.instrs.empty()) {
                continue;
            }
            const auto arm = runOneMode(b.instrs.data(), b.instrs.size(), &prod_cfg);
            total_arm += static_cast<long double>(exec) * arm;
            const auto arm_no_fma = runOneMode(b.instrs.data(), b.instrs.size(), &no_fma_cfg);
            total_arm_no_fma_reduce += static_cast<long double>(exec) * arm_no_fma;
            const auto arm_no_relief = runOneMode(b.instrs.data(), b.instrs.size(), &no_relief_cfg);
            total_arm_no_relief += static_cast<long double>(exec) * arm_no_relief;
            const auto arm_bridged = runOneMode(b.instrs.data(), b.instrs.size(), &bridge_cfg);
            total_arm_bridged += static_cast<long double>(exec) * arm_bridged;
        }
        long double total_ir = 0;
        long double total_peep = 0;
        long double total_single = 0;
        long double total_ft = 0;
        long double total_build_fail = 0;
        long double total_fpr_fail = 0;
        long double total_gpr_fail = 0;
        long double total_ir_split = 0;
        long double total_ir_remat = 0;
        long double total_bridge = 0;
        long double total_bridge_fail = 0;
        long double total_irg[profile::kIRGateReasonCount] = {};
        for (const auto& b : blocks) {
            const uint64_t exec = (b.id < counters.size()) ? counters[b.id] : 0;
            if (exec == 0 || b.id >= tallies.size()) {
                continue;
            }
            const auto& t = tallies[b.id];
            const auto e = static_cast<long double>(exec);
            total_ir += e * t.ir;
            total_peep += e * t.peep;
            total_single += e * t.single;
            total_ft += e * t.ft;
            total_build_fail += e * t.ir_build_fail;
            total_fpr_fail += e * t.ir_fpr_fail;
            total_gpr_fail += e * t.ir_gpr_fail;
            total_ir_split += e * t.ir_split;
            total_ir_remat += e * t.ir_remat;
            total_bridge += e * t.bridge;
            total_bridge_fail += e * t.bridge_fail;
            if (have_irg && b.id < ir_gate_counters.size()) {
                for (uint16_t r = 0; r < profile::kIRGateReasonCount; ++r) {
                    total_irg[r] += e * ir_gate_counters[b.id].counts[r];
                }
            }
        }
        const long double total_ops = total_ir + total_peep + total_single + total_ft;
        const auto pct = [&](long double v) -> double {
            return total_ops > 0 ? static_cast<double>(100.0L * v / total_ops) : 0.0;
        };
        const auto per_kop = [&](long double v) -> double {
            return total_ops > 0 ? static_cast<double>(1000.0L * v / total_ops) : 0.0;
        };
        const double arm_per_x87 = total_ops > 0 ? static_cast<double>(total_arm / total_ops) : 0.0;
        std::printf(
            "# Workload-wide x87 op composition (block-level, %llu exec-weighted ops, "
            "%llu exec-weighted ARM)\n",
            static_cast<unsigned long long>(total_ops), static_cast<unsigned long long>(total_arm));
        std::printf("path,share%%\n");
        std::printf("ir,%.2f\n", pct(total_ir));
        std::printf("peep,%.2f\n", pct(total_peep));
        std::printf("single,%.2f\n", pct(total_single));
        std::printf("ft,%.2f\n", pct(total_ft));
        std::printf("global_arm_per_x87,%.2f\n", arm_per_x87);
        const double arm_per_x87_no_fma_reduce =
            total_ops > 0 ? static_cast<double>(total_arm_no_fma_reduce / total_ops) : 0.0;
        const double fma_contribution_pct =
            arm_per_x87_no_fma_reduce > 0
                ? 100.0 * (arm_per_x87_no_fma_reduce - arm_per_x87) / arm_per_x87_no_fma_reduce
                : 0.0;
        std::printf("global_arm_per_x87_without_fma_reduce,%.2f  (FMA-reduce pass saves %.2f%%)\n",
                    arm_per_x87_no_fma_reduce, fma_contribution_pct);
        const double arm_per_x87_no_relief =
            total_ops > 0 ? static_cast<double>(total_arm_no_relief / total_ops) : 0.0;
        const double relief_contribution_pct =
            arm_per_x87_no_relief > 0
                ? 100.0 * (arm_per_x87_no_relief - arm_per_x87) / arm_per_x87_no_relief
                : 0.0;
        std::printf(
            "global_arm_per_x87_without_pressure_relief,%.2f  (split+remat saves %.2f%%)\n",
            arm_per_x87_no_relief, relief_contribution_pct);
        const double arm_per_x87_no_bridge =
            total_ops > 0 ? static_cast<double>(total_arm_bridged / total_ops) : 0.0;
        const double bridge_contribution_pct =
            arm_per_x87_no_bridge > 0
                ? 100.0 * (arm_per_x87_no_bridge - arm_per_x87) / arm_per_x87_no_bridge
                : 0.0;
        std::printf(
            "global_arm_per_x87_without_bridging,%.2f  (bridging saves %.2f%% — conservative: "
            "our emission of the bridge instrs is counted, stock's saved emission is not)\n",
            arm_per_x87_no_bridge, bridge_contribution_pct);
        if (std::getenv("X87_LOG_FMA_REDUCE") != nullptr) {
            X87IR::fma_reduce_print_stats();
        }
        std::printf("\n");
        std::printf("# Workload-wide refusals per 1000 x87 ops (call-counts, not ops)\n");
        std::printf("event,per_kop\n");
        std::printf("ir_build_fail,%.2f\n", per_kop(total_build_fail));
        std::printf("ir_fpr_fail,%.2f\n", per_kop(total_fpr_fail));
        std::printf("ir_gpr_fail,%.2f\n", per_kop(total_gpr_fail));
        // Pressure-relief attribution (run-counts, not ops; TLY2 captures).
        std::printf("ir_split_runs,%.2f\n", per_kop(total_ir_split));
        std::printf("ir_remat_runs,%.2f\n", per_kop(total_ir_remat));
        // Run-bridging attribution (TLY3 captures; bridge_ops counts bridge
        // INSTRUCTIONS absorbed into IR runs).
        std::printf("bridge_ops,%.2f\n", per_kop(total_bridge));
        std::printf("bridge_fail_runs,%.2f\n", per_kop(total_bridge_fail));
        if (have_irg) {
            for (uint16_t r = 0; r < profile::kIRGateReasonCount; ++r) {
                std::printf("gate_%s,%.2f\n", profile::kIRGateReasonNames[r],
                            per_kop(total_irg[r]));
            }
        }
        std::printf("\n");

        // Per-block ranking by exec-weighted ARM emit.  The per-pattern
        // table below slices every prefix length 2..run_len of each run
        // and shares one exec_count across all the slices, which makes it
        // easy to over-count savings if you sum (prod-ir_forced)*exec
        // across rows of the same block.  This table reports each block
        // exactly once with the actual production ARM emit, so cross-block
        // ranking by total cost is unambiguous.  arm_no_ir / arm_ir_forced
        // come from the same three configs as measurePattern: production,
        // disable_x87_ir=1, force_x87_ir_gate=1.
        struct BlockEmitRow {
            uint32_t start_pc;  // guest x86 VA of the block's first instruction
            uint32_t block_id;  // for --dump-block drill-in
            uint64_t exec;
            uint16_t x87_ops;
            // Longest single consecutive x87 run inside the block.  When this
            // is < 2 the block can never benefit from peep+IR fusion (the
            // cache only activates at run length >= 2; isolated x87 ops
            // always go through single-op emit), regardless of how many x87
            // ops the block has in total.
            uint16_t max_run;
            uint32_t arm_prod;
            uint32_t arm_no_ir;
            uint32_t arm_ir_forced;
            std::string prefix;
            const Tally* tally;
        };
        std::vector<BlockEmitRow> block_rows;
        block_rows.reserve(blocks.size());
        // prod_cfg already declared above for the workload-wide loop.
        RosettaConfig no_ir_cfg{};
        no_ir_cfg.disable_x87_ir = 1;
        RosettaConfig force_gate_cfg{};
        force_gate_cfg.force_x87_ir_gate = 1;
        for (const auto& b : blocks) {
            const uint64_t exec = (b.id < counters.size()) ? counters[b.id] : 0;
            if (exec == 0 || b.instrs.empty()) {
                continue;
            }
            BlockEmitRow row{};
            row.start_pc = b.start_pc;
            row.block_id = b.id;
            row.exec = exec;
            auto* mut_instrs = const_cast<IRInstr*>(b.instrs.data());
            const auto L = static_cast<int64_t>(b.instrs.size());
            int64_t idx = 0;
            uint32_t x87_count = 0;
            uint32_t max_run = 0;
            while (idx < L) {
                const int run = X87Cache::lookahead(mut_instrs, L, idx);
                if (run >= 1) {
                    x87_count += static_cast<uint32_t>(run);
                    max_run = std::max(max_run, static_cast<uint32_t>(run));
                    idx += run;
                } else {
                    idx += 1;
                }
            }
            row.x87_ops = static_cast<uint16_t>(std::min<uint32_t>(x87_count, 0xFFFFU));
            row.max_run = static_cast<uint16_t>(std::min<uint32_t>(max_run, 0xFFFFU));
            row.arm_prod = runOneMode(b.instrs.data(), b.instrs.size(), &prod_cfg);
            row.arm_no_ir = runOneMode(b.instrs.data(), b.instrs.size(), &no_ir_cfg);
            row.arm_ir_forced = runOneMode(b.instrs.data(), b.instrs.size(), &force_gate_cfg);
            int picked = 0;
            for (size_t i = 0; i < b.instrs.size() && picked < 5; ++i) {
                if (X87Cache::lookahead(mut_instrs, L, static_cast<int64_t>(i)) == 0) {
                    continue;
                }
                if (picked > 0) {
                    row.prefix.push_back('|');
                }
                row.prefix += mnemonic(b.instrs[i].opcode());
                row.prefix += opSuffix(b.instrs[i]);
                ++picked;
            }
            row.tally = (b.id < tallies.size()) ? &tallies[b.id] : nullptr;
            block_rows.push_back(std::move(row));
        }
        std::ranges::sort(block_rows, [](const BlockEmitRow& a, const BlockEmitRow& b) {
            return static_cast<long double>(a.exec) * a.arm_prod >
                   static_cast<long double>(b.exec) * b.arm_prod;
        });

        constexpr size_t kTopBlockN = 20;
        const size_t top_n = std::min(kTopBlockN, block_rows.size());
        std::printf("# Top %zu blocks by exec-weighted ARM emit (no prefix double-count)\n", top_n);
        std::printf(
            "rank,exec,x87_ops,max_run,arm_prod,arm_no_ir,arm_ir_forced,share%%,"
            "ir%%,peep%%,single%%,t_ir,t_peep,t_single,t_ft,prefix\n");
        for (size_t i = 0; i < top_n; ++i) {
            const auto& r = block_rows[i];
            const auto block_arm = static_cast<long double>(r.exec) * r.arm_prod;
            const double share =
                total_arm > 0 ? static_cast<double>(100.0L * block_arm / total_arm) : 0.0;
            double ir_pct = 0.0;
            double peep_pct = 0.0;
            double single_pct = 0.0;
            if (r.tally != nullptr) {
                const uint64_t tot = r.tally->total();
                if (tot > 0) {
                    ir_pct = 100.0 * static_cast<double>(r.tally->ir) / static_cast<double>(tot);
                    peep_pct =
                        100.0 * static_cast<double>(r.tally->peep) / static_cast<double>(tot);
                    single_pct =
                        100.0 * static_cast<double>(r.tally->single) / static_cast<double>(tot);
                }
            }
            const unsigned t_ir = r.tally != nullptr ? r.tally->ir : 0U;
            const unsigned t_peep = r.tally != nullptr ? r.tally->peep : 0U;
            const unsigned t_single = r.tally != nullptr ? r.tally->single : 0U;
            const unsigned t_ft = r.tally != nullptr ? r.tally->ft : 0U;
            std::printf("%zu,%llu,%u,%u,%u,%u,%u,%.2f,%.1f,%.1f,%.1f,%u,%u,%u,%u,%s\n", i + 1,
                        static_cast<unsigned long long>(r.exec), static_cast<unsigned>(r.x87_ops),
                        static_cast<unsigned>(r.max_run), r.arm_prod, r.arm_no_ir, r.arm_ir_forced,
                        share, ir_pct, peep_pct, single_pct, t_ir, t_peep, t_single, t_ft,
                        r.prefix.c_str());
        }
        std::printf("\n");

        // ── Hot guest-x86 addresses (collapsed by start_pc) ──────────────────
        // Collapse block_rows by start_pc into one row per guest address.  The
        // list feeds a Ghidra disassembly: map each start_pc to its containing
        // function (getFunctionContaining) and sum cost per function to rank
        // functions for wholesale vectorized replacement.  start_pc is a basic-
        // block entry, not a function; the same address can carry several
        // block_ids (different IR length/content from the same entry, or code-
        // cache churn), so costs are summed per address.  Cost is the additive
        // exec-weighted ARM emit (Σ exec*arm_prod) — comparable to the per-block
        // table above and the workload-wide total_arm used for share%.
        if (hot_addrs > 0 && !block_rows.empty()) {
            struct AddrAgg {
                long double cost = 0.0L;       // Σ exec * arm_prod over collapsed blocks
                long double top_cost = 0.0L;   // dominant block's own cost
                uint64_t exec = 0;             // Σ exec
                uint32_t x87_ops = 0;          // from the dominant (highest-cost) block
                uint32_t max_run = 0;          // from the dominant block
                uint32_t blocks = 0;           // count of block records at this addr
                std::string prefix;            // dominant block's mnemonic preview
                std::vector<uint32_t> ids;     // constituent block_ids (cost-desc order)
            };
            std::unordered_map<uint32_t, AddrAgg> by_addr;
            by_addr.reserve(block_rows.size());
            // block_rows is already sorted by cost desc, so the first block seen
            // for an address is its dominant one and ids accumulate cost-desc.
            for (const auto& r : block_rows) {
                const long double c = static_cast<long double>(r.exec) * r.arm_prod;
                auto& a = by_addr[r.start_pc];
                a.cost += c;
                a.exec += r.exec;
                a.blocks += 1;
                a.ids.push_back(r.block_id);
                if (c >= a.top_cost) {
                    a.top_cost = c;
                    a.x87_ops = r.x87_ops;
                    a.max_run = r.max_run;
                    a.prefix = r.prefix;
                }
            }
            std::vector<std::pair<uint32_t, AddrAgg>> addr_rows(by_addr.begin(), by_addr.end());
            std::ranges::sort(addr_rows, [](const auto& a, const auto& b) {
                if (a.second.cost != b.second.cost) {
                    return a.second.cost > b.second.cost;
                }
                if (a.second.exec != b.second.exec) {
                    return a.second.exec > b.second.exec;
                }
                return a.first < b.first;  // tie-break by address for stable output
            });
            const size_t addr_n = std::min(hot_addrs, addr_rows.size());
            std::printf("# Top %zu hot x86 addresses by exec-weighted ARM emit "
                        "(collapsed by start_pc) - map to functions in Ghidra\n",
                        addr_n);
            std::printf("rank,start_pc,cost,share%%,exec,x87_ops,max_run,blocks,block_ids,prefix\n");
            for (size_t i = 0; i < addr_n; ++i) {
                const auto& [pc, a] = addr_rows[i];
                const double share =
                    total_arm > 0 ? static_cast<double>(100.0L * a.cost / total_arm) : 0.0;
                // Cap the printed id list to keep rows narrow; blocks count stays exact.
                std::string ids;
                constexpr size_t kMaxIds = 8;
                const size_t shown = std::min(kMaxIds, a.ids.size());
                for (size_t k = 0; k < shown; ++k) {
                    if (k > 0) {
                        ids.push_back(';');
                    }
                    ids += std::to_string(a.ids[k]);
                }
                if (a.ids.size() > shown) {
                    ids += "+";
                    ids += std::to_string(a.ids.size() - shown);
                }
                std::printf("%zu,0x%08x,%llu,%.2f,%llu,%u,%u,%u,%s,%s\n", i + 1, pc,
                            static_cast<unsigned long long>(a.cost), share,
                            static_cast<unsigned long long>(a.exec),
                            static_cast<unsigned>(a.x87_ops), static_cast<unsigned>(a.max_run),
                            a.blocks, ids.c_str(), a.prefix.c_str());
            }
            std::printf("\n");
        }

        // ── FP-run fragmentation (run-bridging opportunity sizing) ───────────
        // For each block, find the x87 runs and the non-x87 gaps that JOIN two
        // runs (trailing/leading gaps are ignored — bridging them buys
        // nothing).  Classify each joining gap against the shared predicates
        // in X87Bridge.h, then estimate what v1 bridging would save by
        // re-translating the block with every v1-eligible gap (len <=
        // kMaxGapV1) spliced out: the translator then sees one contiguous run
        // where production sees two, which is exactly the run-length and
        // spill/reload effect bridging would have.  Since a real bridge would
        // also emit the gap instructions ourselves (~1-2 ARM each, replacing
        // stock's own emit for them), subtract a conservative 2 ARM per
        // spliced instruction from the delta.
        if (frag_rows > 0 && !blocks.empty()) {
            struct FragRow {
                uint32_t block_id;
                uint32_t start_pc;
                uint64_t exec;
                uint16_t segments;
                uint16_t joins_v1;
                uint16_t joins_v2;
                uint16_t joins_v2p;  // v2 joins with the flag-dead proof (subset of joins_v2)
                uint16_t joins_inelig;
                uint16_t spliced;     // instrs removed by the v1 splice
                uint16_t spliced_v2;  // instrs removed by the v2-proven splice (v1 ∪ proven v2)
                uint32_t arm_orig;
                uint32_t arm_bridged;
                uint32_t arm_bridged_v2;
                uint16_t bridge_live;  // TLY3: bridge ops per pass that actually ran
                long double saved_w;
                long double saved_v2_w;
                std::string preview;
            };
            std::vector<FragRow> frows;
            long double sum_saved_w = 0.0L;
            long double sum_saved_v2_w = 0.0L;
            uint64_t gaps_v1_w = 0;        // exec-weighted v1 joining-gap instrs
            uint64_t gaps_v2only_w = 0;    // exec-weighted v2-only joining-gap instrs
            uint64_t gaps_v2proven_w = 0;  // v2-only with the flag-dead proof
            uint64_t gaps_inelig_w = 0;    // exec-weighted ineligible joining-gap instrs
            uint64_t joins_short_side = 0;  // v1 joins whose merged x87 length < 3
            uint64_t joins_total = 0;
            std::unordered_map<uint8_t, uint64_t> flag_hist;  // v2-only gap instrs

            for (const auto& b : blocks) {
                const uint64_t exec = (b.id < counters.size()) ? counters[b.id] : 0;
                if (exec == 0 || b.instrs.empty()) {
                    continue;
                }
                auto* mut_instrs = const_cast<IRInstr*>(b.instrs.data());
                const auto L = static_cast<int64_t>(b.instrs.size());

                // Segment walk: [start,len] of each x87 run (run >= 1).
                struct Seg {
                    int64_t start;
                    int64_t len;
                };
                std::vector<Seg> segs;
                int64_t idx = 0;
                while (idx < L) {
                    const int run = X87Cache::lookahead(mut_instrs, L, idx);
                    if (run >= 1) {
                        segs.push_back(Seg{.start = idx, .len = run});
                        idx += run;
                    } else {
                        idx += 1;
                    }
                }
                if (segs.size() < 2) {
                    continue;  // nothing to join
                }

                FragRow row{};
                row.block_id = b.id;
                row.start_pc = b.start_pc;
                row.exec = exec;
                row.segments = static_cast<uint16_t>(std::min<size_t>(segs.size(), 0xFFFF));
                if (b.id < tallies.size()) {
                    row.bridge_live = tallies[b.id].bridge;
                }

                // Joining gaps + splice plans (v1, and v2-proven = v1 ∪ v2
                // gaps whose every flag writer carries the flag_liveness==0
                // proof — the exact condition the v2 runtime bridges on,
                // including the field-populated block gate).
                const bool has_fl = x87bridge::block_has_flag_liveness(mut_instrs, L);
                std::vector<bool> drop(b.instrs.size(), false);
                std::vector<bool> drop_v2(b.instrs.size(), false);
                for (size_t s = 0; s + 1 < segs.size(); ++s) {
                    const int64_t gstart = segs[s].start + segs[s].len;
                    const int64_t gend = segs[s + 1].start;  // exclusive
                    const auto glen = static_cast<int>(gend - gstart);
                    joins_total++;
                    bool all_v1 = true;
                    bool all_v2 = true;
                    bool all_proven = true;
                    for (int64_t g = gstart; g < gend; ++g) {
                        const IRInstr& ins = b.instrs[static_cast<size_t>(g)];
                        const bool v1 = x87bridge::is_bridge_v1(ins);
                        const bool v2 = x87bridge::is_bridge_v2(ins);
                        const bool proven = has_fl && x87bridge::is_bridge_v2_proven(ins);
                        all_v1 = all_v1 && v1;
                        all_v2 = all_v2 && v2;
                        all_proven = all_proven && proven;
                        if (!v1 && v2) {
                            gaps_v2only_w += exec;
                            flag_hist[ins.flag_liveness] += exec;
                            if (proven) {
                                gaps_v2proven_w += exec;
                            }
                        } else if (v1) {
                            gaps_v1_w += exec;
                        } else {
                            gaps_inelig_w += exec;
                        }
                    }
                    if (all_v1 && glen <= x87bridge::kMaxGapV1) {
                        row.joins_v1++;
                        if (segs[s].len + segs[s + 1].len < 3) {
                            joins_short_side++;
                        }
                        for (int64_t g = gstart; g < gend; ++g) {
                            drop[static_cast<size_t>(g)] = true;
                            row.spliced++;
                        }
                    } else if (all_v2) {
                        row.joins_v2++;
                    } else {
                        row.joins_inelig++;
                    }
                    if (all_proven && glen <= x87bridge::kMaxGapV1) {
                        if (!(all_v1 && glen <= x87bridge::kMaxGapV1)) {
                            row.joins_v2p++;
                        }
                        for (int64_t g = gstart; g < gend; ++g) {
                            drop_v2[static_cast<size_t>(g)] = true;
                            row.spliced_v2++;
                        }
                    }
                }
                if (row.joins_v1 == 0 && row.joins_v2 == 0) {
                    continue;  // no bridgeable joins; skip measurement and row
                }

                row.arm_orig = runOneMode(b.instrs.data(), b.instrs.size(), &prod_cfg);
                const auto measure_splice = [&](const std::vector<bool>& dropped,
                                                uint16_t n_spliced, uint32_t& arm_out,
                                                long double& saved_out) {
                    if (n_spliced == 0) {
                        arm_out = row.arm_orig;
                        return;
                    }
                    std::vector<IRInstr> spliced;
                    spliced.reserve(b.instrs.size());
                    for (size_t k = 0; k < b.instrs.size(); ++k) {
                        if (!dropped[k]) {
                            spliced.push_back(b.instrs[k]);
                        }
                    }
                    arm_out = runOneMode(spliced.data(), spliced.size(), &prod_cfg);
                    const long double delta = static_cast<long double>(row.arm_orig) -
                                              static_cast<long double>(arm_out) -
                                              2.0L * n_spliced;
                    if (delta > 0) {
                        saved_out = static_cast<long double>(exec) * delta;
                    }
                };
                measure_splice(drop, row.spliced, row.arm_bridged, row.saved_w);
                sum_saved_w += row.saved_w;
                if (row.spliced_v2 > row.spliced) {
                    measure_splice(drop_v2, row.spliced_v2, row.arm_bridged_v2, row.saved_v2_w);
                } else {
                    // No additional proven-v2 joins: the v2 splice IS the v1
                    // splice.
                    row.arm_bridged_v2 = row.arm_bridged;
                    row.saved_v2_w = row.saved_w;
                }
                sum_saved_v2_w += row.saved_v2_w;

                int picked = 0;
                for (size_t k = 0; k < b.instrs.size() && picked < 6; ++k) {
                    if (picked > 0) {
                        row.preview.push_back('|');
                    }
                    row.preview += mnemonic(b.instrs[k].opcode());
                    ++picked;
                }
                frows.push_back(std::move(row));
            }

            std::ranges::sort(frows, [](const FragRow& a, const FragRow& b) {
                if (a.saved_w != b.saved_w) {
                    return a.saved_w > b.saved_w;
                }
                return static_cast<long double>(a.exec) * a.arm_orig >
                       static_cast<long double>(b.exec) * b.arm_orig;
            });

            const double v1_savings_pct =
                total_arm > 0 ? static_cast<double>(100.0L * sum_saved_w / total_arm) : 0.0;
            const double v2_savings_pct =
                total_arm > 0 ? static_cast<double>(100.0L * sum_saved_v2_w / total_arm) : 0.0;
            std::printf("# FP-run fragmentation (run-bridging opportunity, v1 = mov/lea "
                        "gaps <= %d; v2 adds flag-dead ALU with the flag_liveness==0 proof)\n",
                        x87bridge::kMaxGapV1);
            std::printf("bridge_v1_saved_arm_w,%llu\n",
                        static_cast<unsigned long long>(sum_saved_w));
            std::printf("bridge_v1_savings_pct,%.3f\n", v1_savings_pct);
            std::printf("bridge_v2_saved_arm_w,%llu\n",
                        static_cast<unsigned long long>(sum_saved_v2_w));
            std::printf("bridge_v2_savings_pct,%.3f\n", v2_savings_pct);
            std::printf("gap_instrs_w_v1,%llu\n", static_cast<unsigned long long>(gaps_v1_w));
            std::printf("gap_instrs_w_v2only,%llu\n",
                        static_cast<unsigned long long>(gaps_v2only_w));
            std::printf("gap_instrs_w_v2proven,%llu\n",
                        static_cast<unsigned long long>(gaps_v2proven_w));
            std::printf("gap_instrs_w_ineligible,%llu\n",
                        static_cast<unsigned long long>(gaps_inelig_w));
            std::printf("joins_total,%llu\n", static_cast<unsigned long long>(joins_total));
            std::printf("joins_v1_merged_below_ir_gate,%llu\n",
                        static_cast<unsigned long long>(joins_short_side));
            if (!flag_hist.empty()) {
                std::vector<std::pair<uint8_t, uint64_t>> fh(flag_hist.begin(), flag_hist.end());
                std::ranges::sort(fh, [](const auto& a, const auto& b) {
                    return a.second > b.second;
                });
                std::printf("# flag_liveness histogram of v2-only gap instrs "
                            "(byte=count_w, top 8)\n");
                const size_t fn = std::min<size_t>(8, fh.size());
                for (size_t k = 0; k < fn; ++k) {
                    std::printf("flag_liveness_0x%02x,%llu\n", fh[k].first,
                                static_cast<unsigned long long>(fh[k].second));
                }
            }
            const size_t fr_n = std::min(frag_rows, frows.size());
            std::printf("rank,block_id,start_pc,exec,segments,joins_v1,joins_v2,joins_v2p,"
                        "joins_inelig,spliced,spliced_v2,arm_orig,arm_bridged,arm_bridged_v2,"
                        "bridge_live,saved_w,saved_v2_w,preview\n");
            for (size_t k = 0; k < fr_n; ++k) {
                const auto& r = frows[k];
                std::printf("%zu,%u,0x%08x,%llu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%llu,%llu,%s\n",
                            k + 1, r.block_id, r.start_pc,
                            static_cast<unsigned long long>(r.exec),
                            static_cast<unsigned>(r.segments), static_cast<unsigned>(r.joins_v1),
                            static_cast<unsigned>(r.joins_v2), static_cast<unsigned>(r.joins_v2p),
                            static_cast<unsigned>(r.joins_inelig),
                            static_cast<unsigned>(r.spliced), static_cast<unsigned>(r.spliced_v2),
                            r.arm_orig, r.arm_bridged, r.arm_bridged_v2,
                            static_cast<unsigned>(r.bridge_live),
                            static_cast<unsigned long long>(r.saved_w),
                            static_cast<unsigned long long>(r.saved_v2_w), r.preview.c_str());
            }
            std::printf("\n");
        }
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
                    per_reason[r].max_run = std::max(mr, per_reason[r].max_run);
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
            // After flush-and-proceed, the IRG1 top_dirty counter is ~0
            // (only bumps on the rare defensive refusal), so exec_count is
            // the right primary weight.  If a refusal IS recorded, scale
            // by it — that's the sigal we still care about (defensive
            // path firing == invariant violation worth investigating).
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
        std::printf("# Top opcodes preceding top_dirty event (flush or refusal, exec-weighted)\n");
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

    // ── Top-N blocks by max_run at top_dirty event ────────────────────────
    // Picks the blocks where the longest contiguous x87 run hit top_dirty
    // at the IR gate — now flushed-and-proceeded instead of refused, but
    // max_run still tracks the longest run length seen.  Useful for
    // identifying which blocks have the chains the flush-and-proceed fix
    // unblocked.  td_refusals is the actual-refusal count (defensive
    // path; ~0 in normal operation).
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
        std::printf("# Top blocks by max_run at top_dirty event\n");
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
