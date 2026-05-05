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

    // X87_GATE_FLUSH_THRESHOLD[_DEFERRED_POP|_PERM_DIRTY]
    // Per-branch override of the IR-gate flush-and-proceed minimum
    // run length in `Translator.cpp`'s gate cascade.  0 (default)
    // keeps the compile-time default for that branch.  Valid override
    // range is [3, 16] (clamped at parse time).
    //
    // Compile-time defaults: top_dirty=3, deferred_pop=3, perm_dirty=3.
    // The tag_push_pending branch has no threshold knob: it always
    // refuses (never flushes).  The previous flush-and-proceed design
    // was wasted ARM in 100% of WoW firings (compile_run bailed every
    // time on GPR pressure from the FCmp-heavy follow-on shape) and
    // was the suspected culprit for character-rotation matrix
    // corruption when its threshold was lowered to 3.  See
    // feedback_ir_gate_top_dirty_threshold.md.
    //
    // Originally all four had higher defaults (5/8/10/8) under the
    // suspicion that lowering top_dirty caused WoW vertex-transform
    // corruption.  That attribution was wrong: the actual culprit was
    // the speculative-flush rollback in the (since-reverted) commit
    // 923ad2e, not top_dirty's threshold.  Once 923ad2e was reverted,
    // top_dirty=3 / deferred_pop=3 / perm_dirty=3 are all clean.
    uint8_t x87_ir_gate_flush_threshold_top_dirty;
    uint8_t x87_ir_gate_flush_threshold_deferred_pop;
    uint8_t x87_ir_gate_flush_threshold_perm_dirty;

    // Investigation knobs for the open Bug 2 (speculative-flush rollback
    // corruption).  These are NOT for production — they exist on the
    // inv/rollback-hyp3 branch only.  When all three are unset, gate +
    // rollback behavior is bit-exact identical to current ship.  See
    // ~/.claude/plans/recall-memory-lets-attack-functional-naur.md.
    //
    // X87_LOG_ROLLBACK=1
    //   Print one stdout line per IR-gate speculative-flush rollback
    //   firing: which branch, ir_fail reason, buf_end delta, the cache
    //   fields restored (pre->post), and the cur_instr opcode/pc.
    //
    // X87_ENABLE_ROLLBACK_TOP_DIRTY=1
    //   Re-enables rollback for the top_dirty branch.  Saves
    //   cache.top_dirty before the flush and restores it on bail.  Sets
    //   the `flushed` flag so the rollback block fires.  Default off
    //   because this corrupts WoW weapon vertex transforms via an
    //   unidentified mechanism (testing Hypothesis 3).
    //
    // X87_ENABLE_ROLLBACK_DEFERRED_POP=1
    //   Symmetric for the deferred_pop branch.  Saves
    //   cache.deferred_pop_count and restores it on bail.  Default off
    //   for the same reason.
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
