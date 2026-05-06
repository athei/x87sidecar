#include "rosetta_core/ConfigEnv.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "rosetta_core/Config.h"

namespace {

struct FusionEntry {
    const char* name;
    FusionId id;
};

constexpr FusionEntry kFusionTable[] = {
    {.name = "fld_arithp", .id = FusionId::fld_arithp},
    {.name = "fld_fstp", .id = FusionId::fld_fstp},
    {.name = "fld_arith_fstp", .id = FusionId::fld_arith_fstp},
    {.name = "fld_fcomp_fstsw", .id = FusionId::fld_fcomp_fstsw},
    {.name = "fxch_arithp", .id = FusionId::fxch_arithp},
    {.name = "fxch_fstp", .id = FusionId::fxch_fstp},
    {.name = "fxch_fcom_fstsw", .id = FusionId::fxch_fcom_fstsw},
    {.name = "fxch_fcom", .id = FusionId::fxch_fcom},
    {.name = "fcom_fstsw", .id = FusionId::fcom_fstsw},
    {.name = "fld_fcompp_fstsw", .id = FusionId::fld_fcompp_fstsw},
    {.name = "fld_fld_fucompp", .id = FusionId::fld_fld_fucompp},
    {.name = "fld_fcomp", .id = FusionId::fld_fcomp},
    {.name = "fld_arith_arithp", .id = FusionId::fld_arith_arithp},
    {.name = "fld_arith", .id = FusionId::fld_arith},
    {.name = "arithp_fstp", .id = FusionId::arithp_fstp},
    {.name = "fstp_fld", .id = FusionId::fstp_fld},
    {.name = "arith_fstp", .id = FusionId::arith_fstp},
    {.name = "arith_faddp", .id = FusionId::arith_faddp},
    {.name = "fstp_arith_fstp", .id = FusionId::fstp_arith_fstp},
};

// Treat any non-null env value other than "" / "0" as truthy.  Matches
// the pre-refactor convention so existing X87_FOO=1 invocations keep
// working unchanged.
bool env_truthy(const char* name) {
    const char* v = std::getenv(name);
    if (v == nullptr || v[0] == '\0') {
        return false;
    }
    return std::strcmp(v, "0") != 0;
}

// Inverse of env_truthy for knobs that default ON: returns 1 unless
// the env var is explicitly set to "0".  Unset / empty / any other
// value reads as on.
uint8_t env_default_on(const char* name) {
    const char* v = std::getenv(name);
    if (v == nullptr || v[0] == '\0') {
        return 1;
    }
    return std::strcmp(v, "0") == 0 ? 0 : 1;
}

void apply_fusion_list(const char* csv, uint64_t& mask) {
    char buf[512];
    std::strncpy(buf, csv, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char* save = nullptr;
    for (char* tok = strtok_r(buf, ",", &save); tok != nullptr;
         tok = strtok_r(nullptr, ",", &save)) {
        bool matched = false;
        for (const auto& e : kFusionTable) {
            if (std::strcmp(tok, e.name) == 0) {
                mask |= 1ULL << static_cast<int>(e.id);
                matched = true;
                break;
            }
        }
        if (!matched) {
            std::fprintf(stderr, "X87_DISABLE_FUSIONS: unknown fusion name '%s' (ignored)\n", tok);
        }
    }
}

// Comma-separated 64-bit hex hashes (with or without "0x" prefix).  Sized
// for ~1k unique hashes worst-case (each "0xHHHHHHHHHHHHHHHH," = 19 bytes →
// ~19 KB envelope), so the local buffer just bounds the input length.
void parse_hash_list(const char* csv, std::vector<uint64_t>& out) {
    out.clear();
    if (csv == nullptr || csv[0] == '\0') {
        return;
    }
    char buf[32 * 1024];
    std::strncpy(buf, csv, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char* save = nullptr;
    for (char* tok = strtok_r(buf, ",", &save); tok != nullptr;
         tok = strtok_r(nullptr, ",", &save)) {
        if (tok[0] == '0' && (tok[1] == 'x' || tok[1] == 'X')) {
            tok += 2;
        }
        char* end = nullptr;
        const uint64_t h = std::strtoull(tok, &end, 16);
        if (end != tok && *end == '\0') {
            out.push_back(h);
        }
    }
    std::ranges::sort(out);
    const auto dup = std::ranges::unique(out);
    out.erase(dup.begin(), dup.end());
}

}  // namespace

RosettaConfig load_config_from_env() {
    RosettaConfig cfg{};

    // Translator knobs.
    cfg.disable_x87_cache = env_truthy("X87_DISABLE_CACHE") ? 1 : 0;
    cfg.fast_round = env_truthy("X87_FAST_ROUND") ? 1 : 0;
    cfg.disable_deferred_fxch = env_truthy("X87_DISABLE_DEFERRED_FXCH") ? 1 : 0;
    cfg.disable_x87_ir = env_truthy("X87_DISABLE_X87_IR") ? 1 : 0;
    cfg.enable_fma_reduce = env_default_on("X87_ENABLE_FMA_REDUCE");

    if (env_truthy("X87_DISABLE_ALL_FUSIONS")) {
        cfg.disabled_fusions_mask = ~0ULL;
    }
    if (const char* csv = std::getenv("X87_DISABLE_FUSIONS"); csv != nullptr && csv[0] != '\0') {
        apply_fusion_list(csv, cfg.disabled_fusions_mask);
    }

    // X87_GATE_FLUSH_THRESHOLD[_DEFERRED_POP|_PERM_DIRTY]:
    // per-branch override of the IR-gate flush-and-proceed minimum
    // run length.  Clamp to [3, 16]; outside that range fall back to
    // default (0 = compile-time default of 3 for every branch).  The
    // tag_push branch always refuses; no threshold knob exists.  See
    // Config.h.
    auto parse_gate_threshold = [](const char* env_name, const char* label, uint8_t& target) {
        const char* t = std::getenv(env_name);
        if (t == nullptr || t[0] == '\0') {
            return;
        }
        char* end = nullptr;
        const long v = std::strtol(t, &end, 10);
        if (end != t && *end == '\0' && v >= 3 && v <= 16) {
            target = static_cast<uint8_t>(v);
            std::printf("[rosettax87] %s=%ld (%s IR-gate)\n", env_name, v, label);
        } else {
            std::printf("[rosettax87] %s: '%s' out of range [3,16] or not an integer (ignored)\n",
                        env_name, t);
        }
    };
    parse_gate_threshold("X87_GATE_FLUSH_THRESHOLD", "top_dirty",
                         cfg.x87_ir_gate_flush_threshold_top_dirty);
    parse_gate_threshold("X87_GATE_FLUSH_THRESHOLD_DEFERRED_POP", "deferred_pop",
                         cfg.x87_ir_gate_flush_threshold_deferred_pop);
    parse_gate_threshold("X87_GATE_FLUSH_THRESHOLD_PERM_DIRTY", "perm_dirty",
                         cfg.x87_ir_gate_flush_threshold_perm_dirty);

    // Speculative-flush rollback machinery (Translator.cpp).
    // perm_dirty rollback is unconditional.  top_dirty and deferred_pop
    // default ON since 2026-05-06: the lower() prologue flush at
    // X87IRLower.cpp:343-350 (commit 855a424) closed the cascade hole
    // that previously corrupted WoW geom + weapon when these branches
    // rolled back.  Set =0 to disable an individual branch (bisect /
    // diagnostic).  X87_LOG_ROLLBACK stays default-off.
    cfg.x87_log_rollback = env_truthy("X87_LOG_ROLLBACK") ? 1 : 0;
    cfg.x87_enable_rollback_top_dirty = env_default_on("X87_ENABLE_ROLLBACK_TOP_DIRTY");
    cfg.x87_enable_rollback_deferred_pop = env_default_on("X87_ENABLE_ROLLBACK_DEFERRED_POP");
    if (const char* v = std::getenv("X87_ROLLBACK_HASH_LIST"); v != nullptr && v[0] != '\0') {
        parse_hash_list(v, cfg.x87_rollback_hash_list);
        std::printf("[rosettax87] X87_ROLLBACK_HASH_LIST: %zu unique hashes\n",
                    cfg.x87_rollback_hash_list.size());
    }
    if (const char* v = std::getenv("X87_NO_ROLLBACK_HASH_LIST"); v != nullptr && v[0] != '\0') {
        parse_hash_list(v, cfg.x87_no_rollback_hash_list);
        std::printf("[rosettax87] X87_NO_ROLLBACK_HASH_LIST: %zu unique hashes\n",
                    cfg.x87_no_rollback_hash_list.size());
    }

    // Loader-only knobs (aotinvoke leaves them at 0; harmless because it
    // ignores the loader_* fields anyway).
    cfg.loader_logs = env_truthy("X87_LOGS") ? 1 : 0;
    cfg.loader_force_attach = env_truthy("X87_FORCE_ATTACH") ? 1 : 0;
    cfg.loader_disable_hook = env_truthy("X87_DISABLE_HOOK") ? 1 : 0;
    cfg.loader_always_none = env_truthy("X87_ALWAYS_NONE") ? 1 : 0;
    cfg.loader_log_ops = env_truthy("X87_LOG_OPS") ? 1 : 0;
    cfg.loader_log_throughput = env_truthy("X87_LOG_THROUGHPUT") ? 1 : 0;

    if (const char* p = std::getenv("X87_PROFILE"); p != nullptr && p[0] != '\0') {
        cfg.profile_path = p;
    }

    return cfg;
}

void print_env_help(std::FILE* out) {
    std::fprintf(out,
                 "Environment variables (read once at startup; no later getenv):\n"
                 "  X87_LOGS=1                    verbose loader logging to stdout\n"
                 "                                (rosettax87 only)\n"
                 "  X87_FORCE_ATTACH=1            attach even for x64 PE binaries\n"
                 "                                (rosettax87 only)\n"
                 "  X87_DISABLE_HOOK=1            passthrough mode for benchmark baselines\n"
                 "                                (rosettax87 only): still attaches and writes\n"
                 "                                g_disable_aot=1, but skips the translate_insn\n"
                 "                                entry patch.  Apple's runtime then translates\n"
                 "                                with stock JIT codegen, providing an\n"
                 "                                apples-to-apples baseline against the\n"
                 "                                optimised path (both have AOT cache +\n"
                 "                                interpreter disabled).\n"
                 "  X87_ALWAYS_NONE=1             diagnostic: sidecar always replies None,\n"
                 "                                so the stub falls through to stock for every\n"
                 "                                request.  Use to A/B whether a freeze is in\n"
                 "                                our JIT output or the IPC marshalling itself.\n"
                 "  X87_LOG_OPS=1                 diagnostic: sidecar prints one line per\n"
                 "                                handled op with mnemonic + insn_idx.  With a\n"
                 "                                deterministic freeze repro, the last few\n"
                 "                                lines name the suspect.  HIGH-VOLUME — only\n"
                 "                                enable when bisecting.\n"
                 "  X87_LOG_THROUGHPUT=1          diagnostic: sidecar reporter thread prints\n"
                 "                                req/s every 2 s + an idle-transition line.\n"
                 "                                Off by default; enable when telling 'stuck'\n"
                 "                                apart from 'just slow' on long workloads.\n"
                 "  X87_DISABLE_CACHE=1           drop the cross-instruction GPR cache\n"
                 "  X87_FAST_ROUND=1              skip RC dispatch; always emit FCVTNS/FRINTN\n"
                 "                                (round-to-nearest only — UNSAFE for code that\n"
                 "                                 uses FLDCW to change rounding mode, e.g. Lua)\n"
                 "  X87_DISABLE_DEFERRED_FXCH=1   disable OPT-G (deferred FXCH permutation)\n"
                 "  X87_DISABLE_X87_IR=1          disable the IR optimisation pipeline\n"
                 "  X87_ENABLE_FMA_REDUCE=0       disable NEON FMA-reduction lowering for serial\n"
                 "                                FMADD chains.  Default ON.  Pays off only on\n"
                 "                                workloads with +4-contiguous data/weight\n"
                 "                                streams (audio FIR/IIR, software vertex\n"
                 "                                pipelines).  TurtleWoW's matrix-vector idiom\n"
                 "                                is stride-16, so the pass detects no chains\n"
                 "                                there but is correctness-clean and ships ON\n"
                 "                                so it stays exercised.\n"
                 "  X87_DISABLE_ALL_FUSIONS=1     disable every peephole fusion\n"
                 "  X87_GATE_FLUSH_THRESHOLD=N             override the IR-gate flush-and-\n"
                 "  X87_GATE_FLUSH_THRESHOLD_DEFERRED_POP=N proceed minimum run length per\n"
                 "  X87_GATE_FLUSH_THRESHOLD_PERM_DIRTY=N   branch.  Defaults: top_dirty=3,\n"
                 "                                deferred_pop=3, perm_dirty=3.  Clamp to\n"
                 "                                [3,16].  Useful for dialing back a specific\n"
                 "                                branch if a workload regresses (bumps the\n"
                 "                                threshold so the branch flushes only on\n"
                 "                                longer runs).  The tag_push branch always\n"
                 "                                refuses; no threshold knob.\n"
                 "  X87_LOG_ROLLBACK=1            DIAGNOSTIC: print one stdout line per IR-gate\n"
                 "                                speculative-flush rollback firing.  Format:\n"
                 "                                [rollback] branch=<name> ir_fail=<reason>\n"
                 "                                buf_end_delta=<bytes> td <pre>-><post> tp\n"
                 "                                <pre>-><post> dpc <pre>-><post> pd <pre>-\n"
                 "                                ><post> opcode=<x87 op> pc=0x<rip>\n"
                 "                                block_id=<u32> hash=0x<u64> insn_idx=<n>\n"
                 "                                run_remaining=<n>\n"
                 "  X87_ENABLE_ROLLBACK_TOP_DIRTY=0     DIAGNOSTIC: disable rollback for the\n"
                 "  X87_ENABLE_ROLLBACK_DEFERRED_POP=0  top_dirty / deferred_pop gate branches.\n"
                 "                                Default ON since 2026-05-06 (the lower()\n"
                 "                                prologue flush at X87IRLower.cpp:343-350\n"
                 "                                closed the cascade hole).  perm_dirty rolls\n"
                 "                                back unconditionally and has no knob.\n"
                 "  X87_ROLLBACK_HASH_LIST=0xH,…  DIAGNOSTIC: bisect rollback by IR-content\n"
                 "  X87_NO_ROLLBACK_HASH_LIST=…   hash (FNV-1a, PC zeroed); stable across\n"
                 "                                runs.  Comma-separated 64-bit hex values.\n"
                 "                                Include list non-empty → rollback only for\n"
                 "                                those hashes.  Exclude list non-empty →\n"
                 "                                rollback never for those hashes (exclude\n"
                 "                                wins over include).  Hashes are printed in\n"
                 "                                the [rollback] log line.\n"
                 "  X87_DISABLE_FUSIONS=name1,…   disable specific fusions; names:\n");
    for (const auto& e : kFusionTable) {
        std::fprintf(out, "                                  %s\n", e.name);
    }
}
