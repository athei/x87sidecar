#pragma once

#include <cstdint>
#include <string>
#include <vector>

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
    fld_arith,         // FLD + non-popping ARITH mem (2-instruction, single net push)
    arithp_fstp,       // ARITHp ST(1) + FSTP mem (2-instruction, skip stack writeback)
    fstp_fld,          // FSTP + FLD/FILD/FLDZ/FLD1/FLDconst (2-instruction, pop+push cancel)
    arith_fstp,       // non-popping ARITH + FSTP mem (2-instruction, skip intermediate stack store)
    arith_faddp,      // FMUL + FADDP/FSUBP/FSUBRP → FMADD/FMSUB/FNMSUB (FMA fusion)
    fstp_arith_fstp,  // FSTP mem + non-popping ARITH mem + FSTP mem (3-instruction, batched 2-pop)
    kCount
};

// ── Runtime configuration ───────────────────────────────────────────────────
// Populated from the `X87_*` environment block via `load_config_from_env()`
// (see `ConfigEnv.h`) and registered globally via `rosetta_set_config(&cfg)`
// so the Translator pipeline can read it via `g_rosetta_config`.
//
// Most fields are translator knobs read by `rosetta_core`.  The `loader_*`
// fields are loader-only — `aotinvoke` ignores them.
struct RosettaConfig {
    // Translator knobs (read by rosetta_core's Translator / X87IRLower / etc.)
    uint8_t disable_x87_cache;      // X87_DISABLE_CACHE         drop the cross-instr GPR cache
    uint8_t fast_round;             // X87_FAST_ROUND            skip RC dispatch (round-to-nearest)
    uint8_t disable_deferred_fxch;  // X87_DISABLE_DEFERRED_FXCH disable OPT-G
    uint8_t disable_x87_ir;         // X87_DISABLE_X87_IR        disable IR optimisation pipeline
    uint8_t disable_x87_single_fast;  // X87_DISABLE_SINGLE_FAST fall back to the generic
                                      // per-op emitters for isolated (run==1) fld/fst/fstp
    uint8_t enable_fma_reduce;      // X87_ENABLE_FMA_REDUCE     NEON FMA-reduction pass
                                    //                           (default ON).  Pays off on
                                    //                           +4-contiguous dot products
                                    //                           (audio DSP, sw vertex pipelines).
                                    //                           Set =0 to disable.  Diagnostic
                                    //                           counters via fma_reduce_stats()
                                    //                           and X87_LOG_FMA_REDUCE=1.
    uint8_t enable_ir_split;        // X87_ENABLE_IR_SPLIT       (default ON) when compile_run's
                                    //                           FPR/GPR pressure gate refuses a
                                    //                           run, retry with the prefix ending
                                    //                           just before the overflow point
                                    //                           instead of refusing outright.
                                    //                           The suffix re-enters the gate on
                                    //                           the next dispatch.  Set =0 to
                                    //                           restore all-or-nothing gating.
    uint8_t log_ir_split;           // X87_LOG_IR_SPLIT          one stderr line per split retry
                                    //                           and per rescued run.
    uint8_t fpr_pool_limit;         // X87_FPR_POOL_LIMIT        test-only [1,16]: clamp the FPR
                                    //                           count the pressure gate believes
                                    //                           is available (0 = off).  Makes
                                    //                           split behavior deterministic in
                                    //                           tests; allocation is unaffected.
    uint8_t gpr_pool_limit;         // X87_GPR_POOL_LIMIT        test-only GPR-side equivalent.
    uint8_t force_x87_ir_gate;      // measurement-only flag for tools/profile_analyze: bypass
                                    // the IR-eligibility gate's pre-build refusal conditions
                                    // (run_remaining<3, top_dirty, deferred_pop_count,
                                    // perm_dirty) so compile_run is called regardless.  The
                                    // emitted code is *not* semantically correct when the
                                    // dirty conditions hold — do NOT set in production.  Used
                                    // to answer "what would IR emit for this pattern if the
                                    // gate were lifted?".
    uint64_t disabled_fusions_mask;  // X87_DISABLE_FUSIONS=fld_arithp,...

    // X87_GATE_FLUSH_THRESHOLD[_DEFERRED_POP|_PERM_DIRTY]
    // Per-branch override of the IR-gate flush-and-proceed minimum
    // run length in `Translator.cpp`'s gate cascade.  0 (default)
    // keeps the compile-time default for that branch.  Valid override
    // range is [3, 16] (clamped at parse time).
    //
    // Compile-time defaults: top_dirty=3, deferred_pop=3, perm_dirty=3.
    // The cascade has no tag_push_pending arm — incoming tag state is
    // handled by lower()'s prologue (X87IRLower.cpp:343-350), and
    // cache.tag_push_pending is preserved through compile_run bails by
    // the pre-lower FPR/GPR pressure check.  See
    // feedback_ir_gate_top_dirty_threshold.md for history.
    uint8_t x87_ir_gate_flush_threshold_top_dirty;
    uint8_t x87_ir_gate_flush_threshold_deferred_pop;
    uint8_t x87_ir_gate_flush_threshold_perm_dirty;

    // IR-gate speculative-flush rollback machinery (Translator.cpp).
    // When the gate's top_dirty / deferred_pop / perm_dirty branch
    // emits a flush and falls through to compile_run, and compile_run
    // subsequently bails on FprPressure/GprPressure, the flush ARM is
    // wasted — the next dispatch path (peephole / single-op) handles
    // the deferred state itself.  The rollback rewinds the buffer +
    // restores the cleared cache field so the wasted ARM is dropped.
    //
    // perm_dirty rollback is unconditional (no knob).  top_dirty and
    // deferred_pop default ON since 2026-05-06: the X87IRLower::lower()
    // prologue flush (`855a424`) closed the cascade hole that had
    // corrupted WoW geom + weapon when these branches rolled back.
    // The knobs remain as bisect / diagnostic kill-switches — set =0
    // to disable an individual branch.
    //
    // X87_LOG_ROLLBACK=1   (default off)
    //   Print one stdout line per rollback firing: branch, ir_fail
    //   reason, buf_end delta, restored cache fields (pre->post), and
    //   the cur_instr opcode/pc.  For offline correlation with
    //   X87_PROFILE captures via tools/rollback_diff.
    //
    // X87_ENABLE_ROLLBACK_TOP_DIRTY=0       (default on)
    //   Disable rollback for the top_dirty gate branch.
    //
    // X87_ENABLE_ROLLBACK_DEFERRED_POP=0    (default on)
    //   Disable rollback for the deferred_pop gate branch.
    uint8_t x87_log_rollback;
    uint8_t x87_enable_rollback_top_dirty;
    uint8_t x87_enable_rollback_deferred_pop;

    // X87_ROLLBACK_HASH_LIST / X87_NO_ROLLBACK_HASH_LIST — comma-separated
    // 64-bit hex hashes (with or without "0x" prefix).  Hash is
    // profile::hash_ir_stream over the block's IR (PC zeroed), stable
    // across runs.  Populated unconditionally on block transition into
    // cache.profile_hash so these gates work even when X87_PROFILE is off.
    //   - Include list non-empty → rollback only when current hash is in it.
    //   - Exclude list non-empty → rollback never when current hash is in it.
    //     Exclude takes precedence over include.
    std::vector<uint64_t> x87_rollback_hash_list;     // sorted, binary-searched
    std::vector<uint64_t> x87_no_rollback_hash_list;  // sorted, binary-searched

    // Loader-only knobs (read by rosettax87 main; aotinvoke leaves them 0)
    uint8_t loader_logs;            // X87_LOGS          verbose loader logging
    uint8_t loader_disable_hook;    // X87_DISABLE_HOOK  passthrough mode for benchmark
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
