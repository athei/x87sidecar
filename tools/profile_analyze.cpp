// profile_analyze — offline pattern miner for X87_PROFILE dumps.
//
// Reads one or more binary .prof files produced by the sidecar's
// dumpBlockIfNew() (rosetta_loader/src/sidecar.cpp) and emits a CSV of
// adjacent-instruction patterns ranked by occurrence count across all
// blocks.  Window length is unbounded — every N from 2 to block_length
// is considered, so longer fusions (5+, 6+) surface naturally if they
// repeat often enough.  Min-support filter drops the long tail of
// unique-to-one-block sequences.
//
// Usage:
//   profile_analyze [--min-count N] [--max-rows N] file1.prof [file2.prof ...]

#include <algorithm>
#include <array>
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

#include "rosetta_core/Opcode.h"
#include "rosetta_core/ProfileFormat.h"

namespace {

struct Instr {
    uint16_t opcode;
    uint8_t num_operands;
    uint8_t ir_kind;
    profile::OperandSummary operands[4];
};

struct Block {
    uint32_t id;
    uint32_t start_pc;
    std::vector<Instr> instrs;
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
              std::vector<Tally>& tallies) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        std::fprintf(stderr, "error: cannot open %s\n", path.c_str());
        return false;
    }
    std::vector<uint8_t> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    size_t off = 0;
    while (off + sizeof(uint32_t) <= buf.size()) {
        uint32_t lead;
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
        const size_t need = static_cast<size_t>(hdr.num_instrs) * sizeof(profile::InstrRecord);
        if (off + need > buf.size()) {
            std::fprintf(stderr, "error: truncated InstrRecord array in %s at offset %zu\n",
                         path.c_str(), off);
            return false;
        }
        Block b{.id = hdr.block_id, .start_pc = hdr.start_pc, .instrs = {}};
        b.instrs.reserve(hdr.num_instrs);
        for (uint32_t i = 0; i < hdr.num_instrs; ++i) {
            profile::InstrRecord rec;
            std::memcpy(&rec, buf.data() + off, sizeof(rec));
            off += sizeof(rec);
            Instr ins{
                .opcode = rec.opcode,
                .num_operands = rec.num_operands,
                .ir_kind = rec.ir_kind,
                .operands = {rec.operands[0], rec.operands[1], rec.operands[2], rec.operands[3]},
            };
            b.instrs.push_back(ins);
        }
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
        }
    }
    return true;
}

// Convert operand summary to a short suffix: m32/m64/m80, abs, i8/i16/i32/i64,
// reg, st(i), cc, seg, br.  Empty if num_operands == 0.
std::string opSuffix(const Instr& ins) {
    auto sizeStr = [](uint8_t s) -> const char* {
        switch (s) {
            case 0:
                return "8";
            case 1:
                return "16";
            case 2:
                return "32";
            case 3:
                return "64";
            case 4:
                return "128";
            case 5:
                return "256";
            case 0xFF:
                return "80";
            default:
                return "?";
        }
    };
    std::string s;
    const uint8_t n = std::min<uint8_t>(ins.num_operands, 4);
    for (uint8_t k = 0; k < n; ++k) {
        const auto& op = ins.operands[k];
        s.push_back(k == 0 ? '_' : ',');
        switch (op.kind) {
            case 0:  // Register
                // STi vs gpr: stack regs have value (0x70..0x77).
                if ((op.reg & 0xF0) == 0x70) {
                    s += "st";
                    s += static_cast<char>('0' + (op.reg & 0x07));
                } else {
                    s += "r";
                    s += sizeStr(op.size);
                }
                break;
            case 1:  // MemRef
                s += "m";
                s += sizeStr(op.size);
                break;
            case 2:  // AbsMem
                s += "abs";
                s += sizeStr(op.size);
                break;
            case 3:  // Immediate
                s += "i";
                s += sizeStr(op.size);
                break;
            case 4:  // BranchOffset
                s += "br";
                break;
            case 5:  // ConditionCode
                s += "cc";
                break;
            case 6:  // SegmentRegister
                s += "seg";
                break;
            default:
                s += "?";
                break;
        }
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
std::string patternKey(const std::vector<Instr>& instrs, size_t start, size_t len) {
    std::string s;
    for (size_t i = 0; i < len; ++i) {
        if (i > 0) {
            s.push_back('|');
        }
        const Instr& ins = instrs[start + i];
        s += mnemonic(ins.opcode);
        s += opSuffix(ins);
    }
    return s;
}

bool isX87(uint16_t op) {
    return (op >= 0x25 && op <= 0x30) || (op >= 0xBD && op <= 0x10C);
}

// ── Fusion-coverage classifier ────────────────────────────────────────
// Faithfully replicates each try_fuse_X predicate in
// rosetta_core/src/TranslatorX87Fusion.cpp so we can tell whether a
// pattern's prefix is ALREADY fused (no opportunity) versus actually
// uncovered (real fusion candidate).  The shallow heuristic this
// replaces ("first opcode appears in the dispatcher") was misleading —
// e.g. fld_m32|fstp_m32 gets prefix_in_fusion_family=1 but is in fact
// already handled by try_fuse_fld_fstp.

bool is_fld_class(uint16_t op) {
    switch (op) {
        case kOpcodeName_fld:
        case kOpcodeName_fild:
        case kOpcodeName_fldz:
        case kOpcodeName_fld1:
        case kOpcodeName_fldpi:
        case kOpcodeName_fldl2e:
        case kOpcodeName_fldl2t:
        case kOpcodeName_fldlg2:
        case kOpcodeName_fldln2:
            return true;
        default:
            return false;
    }
}

// classify_fld_source rejects m80 FLDs.
bool fld_source_valid(const Instr& ins) {
    if (!is_fld_class(ins.opcode)) {
        return false;
    }
    if (ins.opcode == kOpcodeName_fld || ins.opcode == kOpcodeName_fild) {
        // fld/fild from a memory operand — m80 is rejected by the
        // fusions, register-form FLD ST(i) is accepted.
        if (ins.num_operands >= 1 && ins.operands[0].kind == 1 /*MemRef*/ &&
            ins.operands[0].size == 0xFF /*S80*/) {
            return false;
        }
    }
    return true;
}

bool is_fcomp_reg_or_mem_class(uint16_t op) {
    return op == kOpcodeName_fcomp || op == kOpcodeName_fucomp || op == kOpcodeName_fcom ||
           op == kOpcodeName_fucom;
}

bool is_fcompp_class(uint16_t op) {
    return op == kOpcodeName_fcompp || op == kOpcodeName_fucompp;
}

bool is_arith_nonpopping(uint16_t op) {
    return op == kOpcodeName_fadd || op == kOpcodeName_fsub || op == kOpcodeName_fsubr ||
           op == kOpcodeName_fmul || op == kOpcodeName_fdiv || op == kOpcodeName_fdivr;
}

bool is_arithp(uint16_t op) {
    return op == kOpcodeName_faddp || op == kOpcodeName_fsubp || op == kOpcodeName_fsubrp ||
           op == kOpcodeName_fmulp || op == kOpcodeName_fdivp || op == kOpcodeName_fdivrp;
}

bool is_fstp(uint16_t op) {
    return op == kOpcodeName_fstp || op == kOpcodeName_fstp_stack;
}

bool first_op_is_st1(const Instr& ins) {
    return ins.num_operands >= 1 && ins.operands[0].kind == 0 /*Register*/ &&
           ins.operands[0].reg == 0x71;  // ST(1)
}

bool first_op_is_st0(const Instr& ins) {
    return ins.num_operands >= 1 && ins.operands[0].kind == 0 /*Register*/ &&
           ins.operands[0].reg == 0x70;  // ST(0)
}

bool first_op_mem_not_m80(const Instr& ins) {
    return ins.num_operands >= 1 && ins.operands[0].kind == 1 /*MemRef*/ &&
           ins.operands[0].size != 0xFF;
}

// FSTP destination predicate matching try_fuse_fld_fstp / try_fuse_fld_arith_fstp:
// reg form ST(1) OR mem form non-m80.
bool fstp_dest_valid(const Instr& ins) {
    if (!is_fstp(ins.opcode)) {
        return false;
    }
    if (ins.num_operands < 1) {
        return false;
    }
    if (ins.operands[0].kind == 0 /*Register*/) {
        return ins.operands[0].reg == 0x71;  // ST(1) only
    }
    return ins.operands[0].size != 0xFF;  // non-m80 mem
}

// ARITH operand-kind predicate for fld_arith_fstp / arith_fstp / fld_arith_arithp:
// either register-register with ST(0)/ST(1) OR mem source.
bool arith_operands_for_fld_arith_fstp(const Instr& ins) {
    if (ins.num_operands < 1) {
        return false;
    }
    if (ins.operands[0].kind != 0 /*Register*/) {
        return true;  // memory source — accepted
    }
    // reg-reg: dst must be ST(0), src must be ST(1)
    if (ins.num_operands < 2) {
        return false;
    }
    return ins.operands[0].reg == 0x70 && ins.operands[1].reg == 0x71;
}

// Returns the name of the fusion that would fire on the prefix of
// instrs[start..end), or nullptr if none.  When non-null, *consumed_out
// is set to how many instructions the fusion absorbs (2..4).  Match
// order mirrors try_fuse_*_group + try_peephole exactly.
const char* classifyFusion(const std::vector<Instr>& w, size_t start, size_t end,
                           int* consumed_out) {
    const auto have = [&](size_t k) { return start + k <= end; };

    if (!have(2)) {
        return nullptr;
    }
    const Instr& a = w[start];
    const Instr& b = w[start + 1];

    // ── FLD-class first opcode: try_fuse_fld_group ────────────────────
    if (is_fld_class(a.opcode) && fld_source_valid(a)) {
        // 4-op: fld_fld_fucompp [+ fstsw]
        if (have(4)) {
            const Instr& c = w[start + 2];
            const Instr& d = w[start + 3];
            if (is_fld_class(b.opcode) && fld_source_valid(b) && is_fcompp_class(c.opcode) &&
                d.opcode == kOpcodeName_fstsw) {
                *consumed_out = 4;
                return "fld_fld_fucompp_fstsw";
            }
        }
        // 3-op fusions
        if (have(3)) {
            const Instr& c = w[start + 2];
            // fld_arith_fstp: FLD + arith(reg/mem) + FSTP
            if (is_arith_nonpopping(b.opcode) && arith_operands_for_fld_arith_fstp(b) &&
                fstp_dest_valid(c)) {
                *consumed_out = 3;
                return "fld_arith_fstp";
            }
            // fld_arith_arithp: FLD + arith + arithp ST(1)
            if (is_arith_nonpopping(b.opcode) && arith_operands_for_fld_arith_fstp(b) &&
                is_arithp(c.opcode) && first_op_is_st1(c)) {
                *consumed_out = 3;
                return "fld_arith_arithp";
            }
            // fld_fcomp_fstsw
            if (is_fcomp_reg_or_mem_class(b.opcode) && c.opcode == kOpcodeName_fstsw) {
                *consumed_out = 3;
                return "fld_fcomp_fstsw";
            }
            // fld_fcompp_fstsw
            if (is_fcompp_class(b.opcode) && c.opcode == kOpcodeName_fstsw) {
                *consumed_out = 3;
                return "fld_fcompp_fstsw";
            }
            // fld_fld_fucompp without trailing fstsw
            if (is_fld_class(b.opcode) && fld_source_valid(b) && is_fcompp_class(c.opcode)) {
                *consumed_out = 3;
                return "fld_fld_fucompp";
            }
        }
        // 2-op fusions
        if (is_arithp(b.opcode) && first_op_is_st1(b)) {
            *consumed_out = 2;
            return "fld_arithp";
        }
        if (fstp_dest_valid(b)) {
            *consumed_out = 2;
            return "fld_fstp";
        }
        if (is_fcomp_reg_or_mem_class(b.opcode)) {
            *consumed_out = 2;
            return "fld_fcomp";
        }
    }

    // ── FXCH-group ────────────────────────────────────────────────────
    if (a.opcode == kOpcodeName_fxch) {
        // fxch_arithp: fxch ST(1), arithp ST(1) — operand index check.
        if (a.num_operands >= 2 && a.operands[1].kind == 0 && a.operands[1].reg == 0x71) {
            if (is_arithp(b.opcode) && first_op_is_st1(b)) {
                *consumed_out = 2;
                return "fxch_arithp";
            }
            if (is_fstp(b.opcode) && first_op_is_st1(b)) {
                *consumed_out = 2;
                return "fxch_fstp";
            }
        }
    }

    // ── FCOM-group ────────────────────────────────────────────────────
    if ((is_fcomp_reg_or_mem_class(a.opcode) || is_fcompp_class(a.opcode)) &&
        b.opcode == kOpcodeName_fstsw) {
        *consumed_out = 2;
        return "fcom_fstsw";
    }

    // ── ARITHp-group ──────────────────────────────────────────────────
    if (is_arithp(a.opcode) && first_op_is_st1(a) && is_fstp(b.opcode) && first_op_mem_not_m80(b)) {
        *consumed_out = 2;
        return "arithp_fstp";
    }

    // ── ARITH-group ───────────────────────────────────────────────────
    if (is_arith_nonpopping(a.opcode)) {
        // arith_faddp: FMUL + FADDP/FSUBP/FSUBRP (FMA fusion)
        if (a.opcode == kOpcodeName_fmul &&
            (b.opcode == kOpcodeName_faddp || b.opcode == kOpcodeName_fsubp ||
             b.opcode == kOpcodeName_fsubrp) &&
            first_op_is_st1(b)) {
            *consumed_out = 2;
            return "arith_faddp";
        }
        // arith_fstp: arith mem + fstp mem
        if (a.num_operands >= 1 && a.operands[0].kind == 1 /*MemRef*/ && is_fstp(b.opcode) &&
            first_op_mem_not_m80(b)) {
            *consumed_out = 2;
            return "arith_fstp";
        }
    }

    // ── FSTP-group ────────────────────────────────────────────────────
    if (is_fstp(a.opcode) && is_fld_class(b.opcode) && fld_source_valid(b)) {
        *consumed_out = 2;
        return "fstp_fld";
    }

    return nullptr;
}

}  // namespace

int main(int argc, char** argv) try {
    uint64_t min_exec = 1000;
    size_t max_rows = 200;
    bool show_covered = false;
    std::vector<std::string> files;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--min-exec" && i + 1 < argc) {
            min_exec = std::strtoull(argv[++i], nullptr, 10);
        } else if (a == "--max-rows" && i + 1 < argc) {
            max_rows = std::strtoull(argv[++i], nullptr, 10);
        } else if (a == "--show-covered") {
            show_covered = true;
        } else if (a == "--help" || a == "-h") {
            std::printf(
                "Usage: profile_analyze [--min-exec N] [--max-rows N] [--show-covered] "
                "file1.prof [file2.prof ...]\n"
                "\n"
                "Each window is classified by classifyFusion() — a faithful re-encoding of\n"
                "every try_fuse_X predicate in TranslatorX87Fusion.cpp.  The 'fusion' column\n"
                "is the name of the fusion that fires on the window's prefix (or '-' if no\n"
                "existing fusion matches).  When fusion!='-' AND consumed >= window_size, the\n"
                "ENTIRE pattern is already fused in production — those rows are hidden by\n"
                "default to keep the output focused on uncovered targets.  --show-covered\n"
                "includes them.\n"
                "\n"
                "Patterns are ranked by exec_count = sum over blocks of (counter[block_id] *\n"
                "occurrences of the pattern within that block).  --min-exec drops patterns\n"
                "that never accumulate that many executions.\n"
                "\n"
                "When the input contains a tally section (TLY0), four extra columns are\n"
                "printed: ir%%/peep%%/single%%/ft%% — the exec-weighted share of the\n"
                "pattern's executions whose containing block had its x87 ops dispatched via\n"
                "the X87IR pipeline, the peephole fusion family, the single-op fallthrough,\n"
                "or the stock-forward fallthrough.  Per-block path mix is propagated into\n"
                "per-pattern shares as the average over contributing blocks.  Older .prof\n"
                "files without TLY0 just omit the four columns.\n");
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

    std::vector<Block> blocks;
    std::vector<uint64_t> counters;
    std::vector<Tally> tallies;
    for (const auto& f : files) {
        std::vector<uint64_t> file_counters;
        std::vector<Tally> file_tallies;
        if (!readFile(f, blocks, file_counters, file_tallies)) {
            return 1;
        }
        // Multi-file aggregation: union counters by block_id (each file's
        // block_ids are independent, so we offset on append). For now the
        // common case is one file per run; the simple path is fine.
        if (counters.empty()) {
            counters = std::move(file_counters);
            tallies = std::move(file_tallies);
        } else {
            counters.insert(counters.end(), file_counters.begin(), file_counters.end());
            tallies.insert(tallies.end(), file_tallies.begin(), file_tallies.end());
        }
    }
    const bool have_tallies = !tallies.empty();
    std::fprintf(stderr,
                 "loaded %zu blocks, %zu counter entries, %zu tally entries from %zu file(s)\n",
                 blocks.size(), counters.size(), tallies.size(), files.size());

    // Slide all window lengths over each block.  Aggregate by pattern key,
    // weighting each occurrence by counter[block_id].  Per-block path mix
    // (ir_ops/peephole_ops/single_ops/fallthrough_ops over all the block's
    // x87 ops) is propagated into a per-pattern share by accumulating
    // exec * (path_ops / total_x87_ops) for each path.
    struct Stats {
        uint64_t exec_count = 0;
        size_t window_size = 0;
        const char* fusion = nullptr;  // existing fusion firing on prefix, or null
        int fusion_consumed = 0;       // how many leading instrs the fusion absorbs
        // Path-weighted exec contributions; ratios at print time.
        long double ir_w = 0.0L;
        long double peep_w = 0.0L;
        long double single_w = 0.0L;
        long double ft_w = 0.0L;
        long double build_fail_w = 0.0L;
        long double fpr_fail_w = 0.0L;
        long double gpr_fail_w = 0.0L;
        uint16_t max_gpr_peak = 0;  // max across contributing blocks
    };
    std::unordered_map<std::string, Stats> patterns;
    patterns.reserve(blocks.size() * 4);

    for (const auto& b : blocks) {
        const uint64_t exec = (b.id < counters.size()) ? counters[b.id] : 0;
        if (exec == 0) {
            continue;  // dead-code block contributes nothing — skip work
        }
        // Per-block path fractions; default to all-zero when no tally
        // section was present in the input.
        long double ir_frac = 0.0L;
        long double peep_frac = 0.0L;
        long double single_frac = 0.0L;
        long double ft_frac = 0.0L;
        long double build_fail_frac = 0.0L;
        long double fpr_fail_frac = 0.0L;
        long double gpr_fail_frac = 0.0L;
        if (b.id < tallies.size()) {
            const auto& t = tallies[b.id];
            const uint64_t tot = t.total();
            if (tot > 0) {
                ir_frac = static_cast<long double>(t.ir) / tot;
                peep_frac = static_cast<long double>(t.peep) / tot;
                single_frac = static_cast<long double>(t.single) / tot;
                ft_frac = static_cast<long double>(t.ft) / tot;
                build_fail_frac = static_cast<long double>(t.ir_build_fail) / tot;
                fpr_fail_frac = static_cast<long double>(t.ir_fpr_fail) / tot;
                gpr_fail_frac = static_cast<long double>(t.ir_gpr_fail) / tot;
            }
        }
        const size_t L = b.instrs.size();
        for (size_t start = 0; start < L; ++start) {
            const size_t maxLen = L - start;
            for (size_t len = 2; len <= maxLen; ++len) {
                bool hasX87 = false;
                for (size_t k = 0; k < len; ++k) {
                    if (isX87(b.instrs[start + k].opcode)) {
                        hasX87 = true;
                        break;
                    }
                }
                if (!hasX87) {
                    continue;
                }
                std::string key = patternKey(b.instrs, start, len);
                auto& s = patterns[key];
                if (s.exec_count == 0) {
                    s.window_size = len;
                    int consumed = 0;
                    s.fusion = classifyFusion(b.instrs, start, start + len, &consumed);
                    s.fusion_consumed = consumed;
                }
                s.exec_count += exec;
                const auto execL = static_cast<long double>(exec);
                s.ir_w += execL * ir_frac;
                s.peep_w += execL * peep_frac;
                s.single_w += execL * single_frac;
                s.ft_w += execL * ft_frac;
                s.build_fail_w += execL * build_fail_frac;
                s.fpr_fail_w += execL * fpr_fail_frac;
                s.gpr_fail_w += execL * gpr_fail_frac;
                if (b.id < tallies.size() && tallies[b.id].max_gpr_peak > s.max_gpr_peak) {
                    s.max_gpr_peak = tallies[b.id].max_gpr_peak;
                }
            }
        }
    }

    std::vector<std::pair<std::string, Stats>> rows;
    rows.reserve(patterns.size());
    for (auto& kv : patterns) {
        if (kv.second.exec_count < min_exec) {
            continue;
        }
        // A pattern is "fully covered" iff a fusion fires on its prefix
        // AND consumes at least the entire window.  These are not new
        // fusion targets — hide unless --show-covered.
        const bool fully_covered =
            kv.second.fusion != nullptr && kv.second.fusion_consumed > 0 &&
            std::cmp_greater_equal(kv.second.fusion_consumed, kv.second.window_size);
        if (!show_covered && fully_covered) {
            continue;
        }
        rows.emplace_back(kv.first, kv.second);
    }
    std::ranges::sort(rows, [](const auto& a, const auto& b) {
        return a.second.exec_count > b.second.exec_count;
    });

    if (have_tallies) {
        std::printf(
            "exec_count,window_size,fusion,consumed,"
            "ir%%,peep%%,single%%,ft%%,build_fail%%,fpr_fail%%,gpr_fail%%,max_gpr_peak,"
            "sequence\n");
    } else {
        std::printf("exec_count,window_size,fusion,consumed,sequence\n");
    }
    const size_t n = std::min(max_rows, rows.size());
    for (size_t i = 0; i < n; ++i) {
        const auto& [key, s] = rows[i];
        if (have_tallies) {
            const auto exec = static_cast<long double>(s.exec_count);
            const auto pct = [&](long double w) -> double {
                return exec > 0 ? static_cast<double>(100.0L * w / exec) : 0.0;
            };
            std::printf("%llu,%zu,%s,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%s\n",
                        static_cast<unsigned long long>(s.exec_count), s.window_size,
                        s.fusion ? s.fusion : "-", s.fusion_consumed, pct(s.ir_w), pct(s.peep_w),
                        pct(s.single_w), pct(s.ft_w), pct(s.build_fail_w), pct(s.fpr_fail_w),
                        pct(s.gpr_fail_w), static_cast<unsigned>(s.max_gpr_peak), key.c_str());
        } else {
            std::printf("%llu,%zu,%s,%d,%s\n", static_cast<unsigned long long>(s.exec_count),
                        s.window_size, s.fusion ? s.fusion : "-", s.fusion_consumed, key.c_str());
        }
    }
    std::fprintf(
        stderr, "%zu unique patterns >= min_exec=%llu (showing %zu; %s fully-covered patterns%s)\n",
        rows.size(), static_cast<unsigned long long>(min_exec), n,
        show_covered ? "including" : "hiding",
        have_tallies ? "; columns ir%/peep%/single%/ft% give exec-weighted path mix"
                     : "; no tally section in input — path columns omitted");
    return 0;
} catch (const std::exception& e) {
    std::fprintf(stderr, "profile_analyze: %s\n", e.what());
    return 1;
} catch (...) {
    std::fprintf(stderr, "profile_analyze: unknown exception\n");
    return 1;
}
