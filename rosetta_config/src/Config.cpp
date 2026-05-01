#include "rosetta_config/Config.h"

#include <cstdlib>
#include <cstring>

struct NameBit {
    const char* name;
    int bit;
};

static const NameBit kFusionBits[] = {
    {.name="fld_arithp",      .bit=static_cast<int>(FusionId::fld_arithp)},
    {.name="fld_fstp",        .bit=static_cast<int>(FusionId::fld_fstp)},
    {.name="fld_arith_fstp",  .bit=static_cast<int>(FusionId::fld_arith_fstp)},
    {.name="fld_fcomp_fstsw", .bit=static_cast<int>(FusionId::fld_fcomp_fstsw)},
    {.name="fxch_arithp",     .bit=static_cast<int>(FusionId::fxch_arithp)},
    {.name="fxch_fstp",       .bit=static_cast<int>(FusionId::fxch_fstp)},
    {.name="fcom_fstsw",      .bit=static_cast<int>(FusionId::fcom_fstsw)},
    {.name="fld_fcompp_fstsw",.bit=static_cast<int>(FusionId::fld_fcompp_fstsw)},
    {.name="fld_fld_fucompp", .bit=static_cast<int>(FusionId::fld_fld_fucompp)},
    {.name="fld_fcomp",       .bit=static_cast<int>(FusionId::fld_fcomp)},
    {.name="fld_arith_arithp",.bit=static_cast<int>(FusionId::fld_arith_arithp)},
    {.name="arithp_fstp",     .bit=static_cast<int>(FusionId::arithp_fstp)},
    {.name="fstp_fld",        .bit=static_cast<int>(FusionId::fstp_fld)},
    {.name="arith_fstp",      .bit=static_cast<int>(FusionId::arith_fstp)},
    {.name="arith_faddp",     .bit=static_cast<int>(FusionId::arith_faddp)},
};

static void apply_mask_from_env(const char* env_var, uint64_t& mask,
                                const NameBit* table, int table_len) {
    const char* v = std::getenv(env_var);
    if (!v || !*v) {
        return;
}

    char buf[512];
    std::strncpy(buf, v, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char* save = nullptr;
    char* tok = strtok_r(buf, ",", &save);
    while (tok) {
        for (int i = 0; i < table_len; i++) {
            if (std::strcmp(tok, table[i].name) == 0) {
                mask |= 1ULL << table[i].bit;
                break;
            }
        }
        tok = strtok_r(nullptr, ",", &save);
    }
}

RosettaConfig parse_config_from_env() {
    RosettaConfig cfg = {};

    if (const char* v = std::getenv("ROSETTA_X87_DISABLE_CACHE")) {
        cfg.disable_x87_cache = (*v == '1') ? 1 : 0;
}

    if (const char* v = std::getenv("ROSETTA_X87_FAST_ROUND")) {
        cfg.fast_round = (*v == '1') ? 1 : 0;
}

    if (const char* v = std::getenv("ROSETTA_X87_DISABLE_DEFERRED_FXCH")) {
        cfg.disable_deferred_fxch = (*v == '1') ? 1 : 0;
}

    if (const char* v = std::getenv("ROSETTA_X87_DISABLE_IR")) {
        cfg.disable_x87_ir = (*v == '1') ? 1 : 0;
}

    if (const char* v = std::getenv("ROSETTA_X87_EXTENDED_FPR_SCRATCH")) {
        cfg.extended_fpr_scratch = (*v == '1') ? 1 : 0;
}

    if (const char* v = std::getenv("ROSETTA_X87_DISABLE_ALL_FUSIONS")) {
        if (*v == '1') {
            cfg.disabled_fusions_mask = ~0ULL;
}
}

    constexpr int kFusionCount = sizeof(kFusionBits) / sizeof(kFusionBits[0]);

    apply_mask_from_env("ROSETTA_X87_DISABLE_FUSIONS", cfg.disabled_fusions_mask,
                        kFusionBits, kFusionCount);

    return cfg;
}
