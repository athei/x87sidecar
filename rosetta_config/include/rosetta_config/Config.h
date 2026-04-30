#pragma once

#include <cstdint>

// Bit positions for peephole fusion patterns in TranslatorX87Fusion.cpp.
enum class FusionId : int {
    fld_arithp = 0,     // FLD + FADDP / FSUBP / FDIVP / FMULP
    fld_fstp,           // FLD + FSTP
    fld_arith_fstp,     // FLD + ARITH + FSTP  (3-instruction)
    fld_fcomp_fstsw,    // FLD + FCOMP + FSTSW (3-instruction)
    fxch_arithp,        // FXCH + FADDP / FSUBP / etc.
    fxch_fstp,          // FXCH + FSTP
    fcom_fstsw,         // FCOM/FCOMP/FUCOM/FUCOMP/FCOMPP/FUCOMPP + FSTSW
    fld_fcompp_fstsw,   // FLD + FCOMPP/FUCOMPP + FSTSW (3-instruction, net pop)
    fld_fld_fucompp,    // FLD + FLD + FCOMPP/FUCOMPP [+ FSTSW] (3- or 4-instruction)
    fld_fcomp,          // FLD + FCOMP/FUCOMP (2-instruction, no FSTSW)
    fld_arith_arithp,   // FLD + ARITH + ARITHp (3-instruction, push+pop cancel)
    arithp_fstp,        // ARITHp ST(1) + FSTP mem (2-instruction, skip stack writeback)
    fstp_fld,           // FSTP + FLD/FILD/FLDZ/FLD1/FLDconst (2-instruction, pop+push cancel)
    arith_fstp,         // non-popping ARITH + FSTP mem (2-instruction, skip intermediate stack store)
    arith_faddp,        // FMUL + FADDP/FSUBP/FSUBRP → FMADD/FMSUB/FNMSUB (2-instruction, FMA fusion)
    kCount
};

struct RosettaConfig {
    uint8_t  disable_x87_cache;      // ROSETTA_X87_DISABLE_CACHE=1
    uint8_t  fast_round;             // ROSETTA_X87_FAST_ROUND=1 — skip RC dispatch, always round-to-nearest
    uint8_t  disable_deferred_fxch;  // ROSETTA_X87_DISABLE_DEFERRED_FXCH=1 — disable OPT-G
    uint8_t  disable_x87_ir;         // ROSETTA_X87_DISABLE_IR=1 — disable IR optimization pipeline
    uint8_t  extended_fpr_scratch;   // ROSETTA_X87_EXTENDED_FPR_SCRATCH=1 — expand FPR scratch pool from 8 (V24–V31) to 16 (V16–V31)
    uint8_t  _pad[3];
    uint64_t disabled_fusions_mask;  // ROSETTA_X87_DISABLE_FUSIONS=fld_arithp,...
};
static_assert(sizeof(RosettaConfig) == 0x10);

inline bool fusion_is_disabled(const RosettaConfig& cfg, FusionId id) {
    return (cfg.disabled_fusions_mask >> static_cast<int>(id)) & 1u;
}

// Parse configuration from environment variables.
// Only call from normal executables (aotinvoke, runtime_loader).
// Environment variables:
//   ROSETTA_X87_DISABLE_CACHE=1          disable X87Cache
//   ROSETTA_X87_DISABLE_FUSIONS=fld_arithp,fcom_fstsw  disable specific fusions
//   ROSETTA_X87_DISABLE_ALL_FUSIONS=1    disable all fusion patterns
//   ROSETTA_X87_FAST_ROUND=1             skip RC dispatch; always emit FCVTNS/FRINTN (nearest only)
//   ROSETTA_X87_DISABLE_IR=1             disable IR-based optimization pipeline
//   ROSETTA_X87_EXTENDED_FPR_SCRATCH=1   expand FPR scratch pool from 8 (V24–V31) to 16 (V16–V31)
RosettaConfig parse_config_from_env();
