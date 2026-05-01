#pragma once

#include <cstdint>

// ── x87 peephole-fusion identifiers ─────────────────────────────────────────
// Bit positions in `RosettaConfig::disabled_fusions_mask`.  See
// `TranslatorX87Fusion.cpp` for the actual fusion implementations.
enum class FusionId : int {
    fld_arithp = 0,    // FLD + FADDP / FSUBP / FDIVP / FMULP
    fld_fstp,          // FLD + FSTP
    fld_arith_fstp,    // FLD + ARITH + FSTP  (3-instruction)
    fld_fcomp_fstsw,   // FLD + FCOMP + FSTSW (3-instruction)
    fxch_arithp,       // FXCH + FADDP / FSUBP / etc.
    fxch_fstp,         // FXCH + FSTP
    fcom_fstsw,        // FCOM/FCOMP/FUCOM/FUCOMP/FCOMPP/FUCOMPP + FSTSW
    fld_fcompp_fstsw,  // FLD + FCOMPP/FUCOMPP + FSTSW (3-instruction, net pop)
    fld_fld_fucompp,   // FLD + FLD + FCOMPP/FUCOMPP [+ FSTSW] (3- or 4-instruction)
    fld_fcomp,         // FLD + FCOMP/FUCOMP (2-instruction, no FSTSW)
    fld_arith_arithp,  // FLD + ARITH + ARITHp (3-instruction, push+pop cancel)
    arithp_fstp,       // ARITHp ST(1) + FSTP mem (2-instruction, skip stack writeback)
    fstp_fld,          // FSTP + FLD/FILD/FLDZ/FLD1/FLDconst (2-instruction, pop+push cancel)
    arith_fstp,   // non-popping ARITH + FSTP mem (2-instruction, skip intermediate stack store)
    arith_faddp,  // FMUL + FADDP/FSUBP/FSUBRP → FMADD/FMSUB/FNMSUB (FMA fusion)
    kCount
};

// ── Runtime configuration ───────────────────────────────────────────────────
// Populated by the executable's command-line parser (rosettax87 / aotinvoke)
// and registered globally via `rosetta_set_config(&cfg)` so the Translator
// pipeline can read it via `g_rosetta_config`.  See `CoreConfig.h`.
//
// Most fields are translator knobs read by `rosetta_core`.  The two
// `loader_*` fields are loader-only — `aotinvoke` ignores them.
struct RosettaConfig {
    // Translator knobs (read by rosetta_core's Translator / X87IRLower / etc.)
    uint8_t disable_x87_cache;      // --disable-cache         drop the cross-instr GPR cache
    uint8_t fast_round;             // --fast-round            skip RC dispatch (round-to-nearest)
    uint8_t disable_deferred_fxch;  // --disable-deferred-fxch disable OPT-G
    uint8_t disable_x87_ir;         // --disable-x87-ir        disable IR optimisation pipeline
    uint8_t extended_fpr_scratch;   // --extended-fpr-scratch  V16-V31 scratch pool (16, not 8)
    uint8_t _pad_a[3];
    uint64_t disabled_fusions_mask;  // --disable-fusions=fld_arithp,...

    // Loader-only knobs (read by rosettax87 main; aotinvoke leaves them 0)
    uint8_t loader_logs;          // --logs           verbose loader logging
    uint8_t loader_force_attach;  // --force-attach   attach even for x64 PE binaries
    uint8_t loader_disable_hook;  // --disable-hook   passthrough mode for benchmark
                                  //                  baselines.  Still attaches and writes
                                  //                  g_disable_aot=1, but skips the
                                  //                  translate_insn entry patch.  Apple's
                                  //                  runtime then JIT-translates everything
                                  //                  with stock codegen, providing an
                                  //                  apples-to-apples comparison against
                                  //                  the optimised path (both have AOT
                                  //                  cache + interpreter disabled).
    uint8_t _pad_b[5];
};
static_assert(sizeof(RosettaConfig) == 0x18);

inline bool fusion_is_disabled(const RosettaConfig& cfg, FusionId id) {
    return (cfg.disabled_fusions_mask >> static_cast<int>(id)) & 1U;
}
