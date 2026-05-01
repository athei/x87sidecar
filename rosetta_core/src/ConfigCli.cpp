#include "rosetta_core/ConfigCli.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

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
    {.name = "fcom_fstsw", .id = FusionId::fcom_fstsw},
    {.name = "fld_fcompp_fstsw", .id = FusionId::fld_fcompp_fstsw},
    {.name = "fld_fld_fucompp", .id = FusionId::fld_fld_fucompp},
    {.name = "fld_fcomp", .id = FusionId::fld_fcomp},
    {.name = "fld_arith_arithp", .id = FusionId::fld_arith_arithp},
    {.name = "arithp_fstp", .id = FusionId::arithp_fstp},
    {.name = "fstp_fld", .id = FusionId::fstp_fld},
    {.name = "arith_fstp", .id = FusionId::arith_fstp},
    {.name = "arith_faddp", .id = FusionId::arith_faddp},
};

bool apply_fusion_list(const char* csv, uint64_t& mask, const char*& bad_name) {
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
            bad_name = tok;
            return false;
        }
    }
    return true;
}

}  // namespace

bool apply_translator_flag(std::string_view flag, RosettaConfig& cfg, const char*& bad_value) {
    bad_value = nullptr;
    if (flag == "--disable-cache") {
        cfg.disable_x87_cache = 1;
        return true;
    }
    if (flag == "--fast-round") {
        cfg.fast_round = 1;
        return true;
    }
    if (flag == "--disable-deferred-fxch") {
        cfg.disable_deferred_fxch = 1;
        return true;
    }
    if (flag == "--disable-x87-ir") {
        cfg.disable_x87_ir = 1;
        return true;
    }
    if (flag == "--extended-fpr-scratch") {
        cfg.extended_fpr_scratch = 1;
        return true;
    }
    if (flag == "--disable-all-fusions") {
        cfg.disabled_fusions_mask = ~0ULL;
        return true;
    }
    if (flag.starts_with("--disable-fusions=")) {
        const auto csv = flag.substr(std::strlen("--disable-fusions="));
        // On false return, apply_fusion_list populates bad_value for the
        // caller's diagnostic.
        return apply_fusion_list(std::string(csv).c_str(), cfg.disabled_fusions_mask, bad_value);
    }
    return false;
}

void print_translator_flag_help(std::FILE* out) {
    std::fprintf(out,
                 "Translator knobs (read by rosetta_core):\n"
                 "  --disable-cache              drop the cross-instruction GPR cache\n"
                 "  --fast-round                 skip RC dispatch; always emit FCVTNS/FRINTN\n"
                 "                               (round-to-nearest only — UNSAFE for code that\n"
                 "                                uses FLDCW to change rounding mode, e.g. Lua)\n"
                 "  --disable-deferred-fxch      disable OPT-G (deferred FXCH permutation)\n"
                 "  --disable-x87-ir             disable the IR optimisation pipeline\n"
                 "  --extended-fpr-scratch       expand FPR scratch pool from 8 (V24-V31)\n"
                 "                               to 16 (V16-V31)\n"
                 "  --disable-all-fusions        disable every peephole fusion\n"
                 "  --disable-fusions=name1,...  disable specific fusions; names:\n");
    for (const auto& e : kFusionTable) {
        std::fprintf(out, "                                 %s\n", e.name);
    }
}
