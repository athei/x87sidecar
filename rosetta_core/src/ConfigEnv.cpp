#include "rosetta_core/ConfigEnv.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

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

}  // namespace

RosettaConfig load_config_from_env() {
    RosettaConfig cfg{};

    // Translator knobs.
    cfg.disable_x87_cache = env_truthy("X87_DISABLE_CACHE") ? 1 : 0;
    cfg.fast_round = env_truthy("X87_FAST_ROUND") ? 1 : 0;
    cfg.disable_deferred_fxch = env_truthy("X87_DISABLE_DEFERRED_FXCH") ? 1 : 0;
    cfg.disable_x87_ir = env_truthy("X87_DISABLE_X87_IR") ? 1 : 0;

    if (env_truthy("X87_DISABLE_ALL_FUSIONS")) {
        cfg.disabled_fusions_mask = ~0ULL;
    }
    if (const char* csv = std::getenv("X87_DISABLE_FUSIONS"); csv != nullptr && csv[0] != '\0') {
        apply_fusion_list(csv, cfg.disabled_fusions_mask);
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
                 "  X87_EXTENDED_FPR_SCRATCH=1    expand FPR scratch pool from 8 (V24-V31)\n"
                 "                                to 16 (V16-V31)\n"
                 "  X87_DISABLE_ALL_FUSIONS=1     disable every peephole fusion\n"
                 "  X87_DISABLE_FUSIONS=name1,…   disable specific fusions; names:\n");
    for (const auto& e : kFusionTable) {
        std::fprintf(out, "                                  %s\n", e.name);
    }
}
