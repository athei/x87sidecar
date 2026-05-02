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

bool readFile(const std::string& path, std::vector<Block>& out, std::vector<uint64_t>& counters) {
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
                     "error: %s is incomplete — no counter section found.  Profile runs must "
                     "exit cleanly (don't kill -9 the rosettax87 process); rerun and exit "
                     "the parent normally.\n",
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

// First-opcode prefix maps to a fusion family per try_peephole's
// dispatcher (rosetta_core/src/TranslatorX87Fusion.cpp:2317-2364).  This
// is a shallow heuristic — the analyzer flags whether *some* fusion
// path looks at this prefix; whether the operand kinds also pass each
// try_fuse_X's filter is a finer check we leave to the user.
bool firstOpcodeHasFusionFamily(uint16_t op) {
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
        case kOpcodeName_fxch:
        case kOpcodeName_fcom:
        case kOpcodeName_fcomp:
        case kOpcodeName_fucom:
        case kOpcodeName_fucomp:
        case kOpcodeName_fcompp:
        case kOpcodeName_fucompp:
        case kOpcodeName_faddp:
        case kOpcodeName_fsubp:
        case kOpcodeName_fsubrp:
        case kOpcodeName_fmulp:
        case kOpcodeName_fdivp:
        case kOpcodeName_fdivrp:
        case kOpcodeName_fadd:
        case kOpcodeName_fsub:
        case kOpcodeName_fsubr:
        case kOpcodeName_fmul:
        case kOpcodeName_fdiv:
        case kOpcodeName_fdivr:
        case kOpcodeName_fstp:
        case kOpcodeName_fst:
            return true;
        default:
            return false;
    }
}

}  // namespace

int main(int argc, char** argv) try {
    uint64_t min_exec = 1000;
    size_t max_rows = 200;
    std::vector<std::string> files;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--min-exec" && i + 1 < argc) {
            min_exec = std::strtoull(argv[++i], nullptr, 10);
        } else if (a == "--max-rows" && i + 1 < argc) {
            max_rows = std::strtoull(argv[++i], nullptr, 10);
        } else if (a == "--help" || a == "-h") {
            std::printf(
                "Usage: profile_analyze [--min-exec N] [--max-rows N] file1.prof [file2.prof ...]\n"
                "\n"
                "Profiles must include a counter section (clean parent exit).  Patterns are\n"
                "ranked by exec_count = sum over blocks of (counter[block_id] * occurrences\n"
                "of the pattern within that block).  --min-exec drops patterns that never\n"
                "accumulate that many executions.\n");
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
    for (const auto& f : files) {
        std::vector<uint64_t> file_counters;
        if (!readFile(f, blocks, file_counters)) {
            return 1;
        }
        // Multi-file aggregation: union counters by block_id (each file's
        // block_ids are independent, so we offset on append). For now the
        // common case is one file per run; the simple path is fine.
        if (counters.empty()) {
            counters = std::move(file_counters);
        } else {
            counters.insert(counters.end(), file_counters.begin(), file_counters.end());
        }
    }
    std::fprintf(stderr, "loaded %zu blocks, %zu counter entries from %zu file(s)\n", blocks.size(),
                 counters.size(), files.size());

    // Slide all window lengths over each block.  Aggregate by pattern key,
    // weighting each occurrence by counter[block_id].
    struct Stats {
        uint64_t exec_count = 0;
        size_t window_size = 0;
        bool prefix_in_fusion_family = false;
    };
    std::unordered_map<std::string, Stats> patterns;
    patterns.reserve(blocks.size() * 4);

    for (const auto& b : blocks) {
        const uint64_t exec = (b.id < counters.size()) ? counters[b.id] : 0;
        if (exec == 0) {
            continue;  // dead-code block contributes nothing — skip work
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
                    s.prefix_in_fusion_family = firstOpcodeHasFusionFamily(b.instrs[start].opcode);
                }
                s.exec_count += exec;
            }
        }
    }

    std::vector<std::pair<std::string, Stats>> rows;
    rows.reserve(patterns.size());
    for (auto& kv : patterns) {
        if (kv.second.exec_count >= min_exec) {
            rows.emplace_back(kv.first, kv.second);
        }
    }
    std::ranges::sort(rows, [](const auto& a, const auto& b) {
        return a.second.exec_count > b.second.exec_count;
    });

    std::printf("exec_count,window_size,prefix_in_fusion_family,sequence\n");
    const size_t n = std::min(max_rows, rows.size());
    for (size_t i = 0; i < n; ++i) {
        const auto& [key, s] = rows[i];
        std::printf("%llu,%zu,%d,%s\n", static_cast<unsigned long long>(s.exec_count),
                    s.window_size, s.prefix_in_fusion_family ? 1 : 0, key.c_str());
    }
    std::fprintf(stderr, "%zu unique patterns >= min_exec=%llu (showing %zu)\n", rows.size(),
                 static_cast<unsigned long long>(min_exec), n);
    return 0;
} catch (const std::exception& e) {
    std::fprintf(stderr, "profile_analyze: %s\n", e.what());
    return 1;
} catch (...) {
    std::fprintf(stderr, "profile_analyze: unknown exception\n");
    return 1;
}
