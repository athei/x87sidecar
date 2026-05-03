#pragma once

#include <cstdint>
#include <string>

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
    fxch_fcom_fstsw,   // FXCH ST(i) + FCOM/FCOMP* + FSTSW (3-instruction)
    fxch_fcom,         // FXCH ST(i) + FCOM/FCOMP* (2-instruction, no FSTSW)
    fcom_fstsw,        // FCOM/FCOMP/FUCOM/FUCOMP/FCOMPP/FUCOMPP + FSTSW
    fld_fcompp_fstsw,  // FLD + FCOMPP/FUCOMPP + FSTSW (3-instruction, net pop)
    fld_fld_fucompp,   // FLD + FLD + FCOMPP/FUCOMPP [+ FSTSW] (3- or 4-instruction)
    fld_fcomp,         // FLD + FCOMP/FUCOMP (2-instruction, no FSTSW)
    fld_arith_arithp,  // FLD + ARITH + ARITHp (3-instruction, push+pop cancel)
    arithp_fstp,       // ARITHp ST(1) + FSTP mem (2-instruction, skip stack writeback)
    fstp_fld,          // FSTP + FLD/FILD/FLDZ/FLD1/FLDconst (2-instruction, pop+push cancel)
    arith_fstp,       // non-popping ARITH + FSTP mem (2-instruction, skip intermediate stack store)
    arith_faddp,      // FMUL + FADDP/FSUBP/FSUBRP → FMADD/FMSUB/FNMSUB (FMA fusion)
    fstp_arith_fstp,  // FSTP mem + non-popping ARITH mem + FSTP mem (3-instruction, batched 2-pop)
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
    uint8_t disable_x87_cache;       // --disable-cache         drop the cross-instr GPR cache
    uint8_t fast_round;              // --fast-round            skip RC dispatch (round-to-nearest)
    uint8_t disable_deferred_fxch;   // --disable-deferred-fxch disable OPT-G
    uint8_t disable_x87_ir;          // --disable-x87-ir        disable IR optimisation pipeline
    uint8_t force_x87_ir_gate;       // measurement-only flag for tools/profile_analyze: bypass
                                     // the IR-eligibility gate's pre-build refusal conditions
                                     // (run_remaining<3, top_dirty, tag_push_pending,
                                     // deferred_pop_count, perm_dirty) so compile_run is
                                     // called regardless.  The emitted code is *not*
                                     // semantically correct when the dirty conditions hold —
                                     // do NOT set in production.  Used to answer "what would
                                     // IR emit for this pattern if the gate were lifted?".
    uint64_t disabled_fusions_mask;  // --disable-fusions=fld_arithp,...

    // Loader-only knobs (read by rosettax87 main; aotinvoke leaves them 0)
    uint8_t loader_logs;            // --logs           verbose loader logging
    uint8_t loader_force_attach;    // --force-attach   attach even for x64 PE binaries
    uint8_t loader_disable_hook;    // --disable-hook   passthrough mode for benchmark
                                    //                  baselines.  Still attaches and writes
                                    //                  g_disable_aot=1, but skips the
                                    //                  translate_insn entry patch.  Apple's
                                    //                  runtime then JIT-translates everything
                                    //                  with stock codegen, providing an
                                    //                  apples-to-apples comparison against
                                    //                  the optimised path (both have AOT
                                    //                  cache + interpreter disabled).
    uint8_t loader_always_none;     // X87_ALWAYS_NONE  diagnostic: sidecar replies None for
                                    //                  every translate_insn request, so the
                                    //                  stub falls through to stock for all
                                    //                  ops.  Hook + IPC mechanics still run;
                                    //                  only our JIT output is bypassed.
                                    //                  Used to A/B "is the freeze in our
                                    //                  emitted code or in the marshalling?"
    uint8_t loader_log_ops;         // X87_LOG_OPS      diagnostic: sidecar prints one line
                                    //                  per processTranslateRequest call with
                                    //                  the opcode name and insn_idx.  Use
                                    //                  with a deterministic freeze repro to
                                    //                  see which op is the last one before
                                    //                  the hang.
    uint8_t loader_log_throughput;  // X87_LOG_THROUGHPUT  diagnostic: sidecar starts a
                                    //                     reporter thread that prints requests/sec
                                    //                     every 2 s and an idle-transition line.
                                    //                     Useful to tell "stuck" from "just slow"
                                    //                     during long workloads (WoW world-load).

    // X87_PROFILE=<path>  When non-empty, sidecar appends a binary
    // record per first-seen IRBlock to this file (full IR stream).
    // Drives offline fusion-candidate analysis via tools/profile_analyze.
    std::string profile_path;
};

inline bool fusion_is_disabled(const RosettaConfig& cfg, FusionId id) {
    return (cfg.disabled_fusions_mask >> static_cast<int>(id)) & 1U;
}
