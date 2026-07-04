#include "rosetta_core/Translator.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <optional>

#include "rosetta_core/AssemblerHelpers.hpp"
#include "rosetta_core/Config.h"
#include "rosetta_core/CoreConfig.h"
#include "rosetta_core/IRInstr.h"
#include "rosetta_core/Opcode.h"
#include "rosetta_core/ProfileFormat.h"
#include "rosetta_core/ProfileRuntime.h"
#include "rosetta_core/Register.h"
#include "rosetta_core/TranslationResult.h"
#include "rosetta_core/TranslatorHelpers.hpp"
#include "rosetta_core/TranslatorX87.h"
#include "rosetta_core/TranslatorX87Fusion.h"
#include "rosetta_core/TranslatorX87Helpers.hpp"
#include "rosetta_core/X87Cache.h"
#include "rosetta_core/X87IR.h"

// True for opcode IDs in the two x87 ranges that Rosetta assigns to x87
// instructions.  Mirrors the JIT stub's FILTER prologue range check at
// rosetta_loader/src/stub_asm.cpp:464-467.  Used by the dispatch default
// case so we only complain about *x87* ops we forgot to handle — the AOT
// path (CustomTranslationHook) sees every non-x87 op too and would
// otherwise spam stdout.
static bool is_x87_opcode(uint16_t op) {
    return (op >= kOpcodeName_fcmovb && op <= kOpcodeName_fucomip) ||
           (op >= kOpcodeName_f2xm1 && op <= kOpcodeName_fyl2xp1);
}

auto Translator::translate_instruction(TranslationResult* translation_result, IRBlock* block,
                                       IRInstr* instr_array, int64_t num_instrs, int64_t insn_idx)
    -> std::optional<int64_t> {
    auto* const cur_instr = &instr_array[insn_idx];
    const auto opcode = cur_instr->opcode();
    auto& cache = translation_result->x87_cache;

    // ── OPT-1: x87 cross-instruction cache management ───────────────────────
    // Invalidate the cache if we've moved to a different block (between blocks,
    // branches may have executed non-x87 code that clobbers scratch GPRs).
    // Then, if no cache is active, scan ahead to find the length of the current
    // consecutive x87 run.  x87_cache_set_run only activates if run >= 2.
    {
        if (block != cache.prev_block) {
            cache.invalidate(translation_result->free_gpr_mask, kGprScratchMask);
            cache.prev_block = block;
            cache.tally_ir = 0;
            cache.tally_peep = 0;
            cache.tally_single = 0;
            cache.tally_ft = 0;
            cache.tally_ir_build_fail = 0;
            cache.tally_ir_fpr_fail = 0;
            cache.tally_ir_gpr_fail = 0;
            cache.tally_ir_split = 0;
            cache.tally_ir_remat = 0;
            cache.tally_bridge = 0;
            cache.tally_bridge_fail = 0;
            cache.tally_max_gpr_peak = 0;
            cache.tally_ir_gate_short_run = 0;
            cache.tally_ir_gate_top_dirty = 0;
            cache.tally_ir_gate_tag_push = 0;
            cache.tally_ir_gate_deferred_pop = 0;
            cache.tally_ir_gate_perm_dirty = 0;
            for (auto& v : cache.max_run_at_gate) {
                v = 0;
            }
            cache.prev_x87_opcode = 0xFFFFU;
            // X87_FAST_ROUND=2: one scan per block for control-word writers;
            // x87_fast_round_active keeps the full RC dispatch in blocks
            // that contain one.
            cache.block_has_cw_write = 0;
            if (g_rosetta_config && g_rosetta_config->fast_round == 2) {
                for (int64_t i = 0; i < num_instrs; i++) {
                    switch (instr_array[i].opcode()) {
                        case kOpcodeName_fldcw:
                        case kOpcodeName_fldenv:
                        case kOpcodeName_frstor:
                        case kOpcodeName_fxrstor:
                        case kOpcodeName_finit:
                        case kOpcodeName_fsave:
                            cache.block_has_cw_write = 1;
                            break;
                        default:
                            break;
                    }
                    if (cache.block_has_cw_write) {
                        break;
                    }
                }
            }
            cache.profile_bid = profile::kOverflowId;
            // Compute the IR hash unconditionally so the X87_*_HASH_LIST
            // rollback gate works without X87_PROFILE.  The hash is
            // FNV-1a over the IR stream with each instr's `pc` zeroed,
            // so it identifies the block by content and is stable across
            // launches (unlike profile_bid, which is registration order).
            cache.profile_hash =
                profile::hash_ir_stream(instr_array, static_cast<size_t>(num_instrs));
            if (profile::counter_array_addr() != 0) {
                const uint32_t bid = profile::register_block(block, cache.profile_hash);
                if (bid != profile::kOverflowId) {
                    emit_block_counter_bump(*translation_result, bid);
                    cache.profile_bid = bid;
                }
            }
        }
        if (!cache.active()) {
            const bool cache_disabled = g_rosetta_config && g_rosetta_config->disable_x87_cache;
            if (!cache_disabled) {
                const int run = X87Cache::lookahead(instr_array, num_instrs, insn_idx);
                cache.set_run(run);
                // Run bridging: at a fresh run start, check whether short v1
                // gaps join this run to following x87 segments.  Stash the
                // descriptor; the IR dispatch below fires the all-or-nothing
                // attempt when it reaches this index with no deferred state.
                cache.bridge_pending_total = 0;
                cache.bridge_pending_idx = -1;
                // Per-block bisect: include list restricts bridging to the
                // listed IR-content hashes; exclude list vetoes them
                // (exclude wins).  cache.profile_hash is populated on block
                // transition, before any run start in the block.
                bool bridge_allowed = g_rosetta_config != nullptr &&
                                      g_rosetta_config->enable_bridge != 0;
                if (bridge_allowed) {
                    const auto& cfg = *g_rosetta_config;
                    if (!cfg.x87_no_bridge_hash_list.empty() &&
                        std::ranges::binary_search(cfg.x87_no_bridge_hash_list,
                                                   cache.profile_hash)) {
                        bridge_allowed = false;
                    } else if (!cfg.x87_bridge_hash_list.empty() &&
                               !std::ranges::binary_search(cfg.x87_bridge_hash_list,
                                                           cache.profile_hash)) {
                        bridge_allowed = false;
                    }
                }
                if (bridge_allowed && run >= 1) {
                    const auto br = X87Cache::lookahead_bridged(
                        instr_array, num_instrs, insn_idx, g_rosetta_config->bridge_max_gap,
                        g_rosetta_config->bridge_max_total);
                    if (br.total > run) {
                        cache.bridge_pending_total = br.total;
                        cache.bridge_pending_x87 = br.x87_count;
                        cache.bridge_pending_plain = static_cast<int16_t>(run);
                        cache.bridge_pending_idx = static_cast<int16_t>(insn_idx);
                    }
                }
            }
        }
    }

    // X87_PROFILE — mirror the in-cache tally to ProfileRuntime so the sidecar
    // dump (kqueue NOTE_EXIT) reads the final count.  No-op when profiling is
    // disabled or this block hit kMaxBlocks.
    const auto mirror_tally = [&] {
        if (cache.profile_bid != profile::kOverflowId) {
            profile::set_block_tally(cache.profile_bid,
                                     profile::BlockTally{
                                         .ir_ops = cache.tally_ir,
                                         .peephole_ops = cache.tally_peep,
                                         .single_ops = cache.tally_single,
                                         .fallthrough_ops = cache.tally_ft,
                                         .ir_build_fail_ops = cache.tally_ir_build_fail,
                                         .ir_fpr_fail_ops = cache.tally_ir_fpr_fail,
                                         .ir_gpr_fail_ops = cache.tally_ir_gpr_fail,
                                         .max_gpr_peak = cache.tally_max_gpr_peak,
                                         .ir_split_runs = cache.tally_ir_split,
                                         .ir_remat_runs = cache.tally_ir_remat,
                                         .bridge_ops = cache.tally_bridge,
                                         .bridge_fail_runs = cache.tally_bridge_fail,
                                     });
        }
    };

    // Mirror the per-reason gate-refusal counters (separate side-table from
    // BlockTally; see ProfileFormat.h IRG1 section).  Called whenever any of
    // the 5 gate counters increments.  Also mirrors the max-run-at-refusal
    // side-table (RRR0) — same trigger.
    const auto mirror_gate_counters = [&] {
        if (cache.profile_bid != profile::kOverflowId) {
            profile::BlockIRGateCounters c{};
            c.counts[profile::kIRGateReasonShortRun] = cache.tally_ir_gate_short_run;
            c.counts[profile::kIRGateReasonTopDirty] = cache.tally_ir_gate_top_dirty;
            c.counts[profile::kIRGateReasonTagPush] = cache.tally_ir_gate_tag_push;
            c.counts[profile::kIRGateReasonDeferredPop] = cache.tally_ir_gate_deferred_pop;
            c.counts[profile::kIRGateReasonPermDirty] = cache.tally_ir_gate_perm_dirty;
            profile::set_block_ir_gate_counters(cache.profile_bid, c);

            profile::BlockMaxRunAtRefuse mr{};
            for (uint32_t r = 0; r < profile::kIRGateReasonCount; ++r) {
                mr.max_run[r] = cache.max_run_at_gate[r];
            }
            profile::set_block_max_run_at_refuse(cache.profile_bid, mr);
        }
    };

    // Update max_run_at_gate[reason] = max(prev, current run_remaining).
    // Called inline at each gate refusal branch — captures the run length
    // at the moment of refusal (before any tick() decrement).
    const auto bump_max_run = [&](uint16_t reason) {
        const auto run = static_cast<uint16_t>(cache.run_remaining);
        cache.max_run_at_gate[reason] = std::max(run, cache.max_run_at_gate[reason]);
    };

    // Consolidate the rollback-eligibility check used by the top_dirty /
    // deferred_pop gates.  Returns true when the per-branch enable knob is
    // set AND the hash include/exclude lists allow this block.  Hash exclude
    // wins over hash include.  Empty hash lists are no-ops.
    const auto rollback_allowed = [&](uint8_t branch_enable) -> bool {
        if (g_rosetta_config == nullptr || branch_enable == 0) {
            return false;
        }
        const auto& cfg = *g_rosetta_config;
        if (!cfg.x87_no_rollback_hash_list.empty() &&
            std::ranges::binary_search(cfg.x87_no_rollback_hash_list, cache.profile_hash)) {
            return false;
        }
        if (!cfg.x87_rollback_hash_list.empty() &&
            !std::ranges::binary_search(cfg.x87_rollback_hash_list, cache.profile_hash)) {
            return false;
        }
        return true;
    };

    // ── IR pipeline: try whole-run optimization for runs of 3+ ─────────────
    // The IR fires once at the start of a fresh run (no deferred cache state).
    // It builds an SSA-like IR, runs optimization passes (DSE, FMA, FCOM+FSTSW
    // fusion), and lowers directly to AArch64.
    //
    // X87_PROFILE diagnostic: when the cache is active (i.e. an x87 run is
    // in flight) but one of the eligibility conditions below refuses entry
    // to compile_run, record which condition fired into the IRG0 side-table.
    // Latest-write-wins per block — analyzer aggregates exec-weighted across
    // blocks to surface the dominant refusal cause.  Was previously a single
    // combined &&-chain whose failures were invisible to the profiler.
    // ── Run bridging: all-or-nothing bridged IR attempt ─────────────────────
    // Fires exactly once per stashed descriptor, at the region's start
    // instruction, and only with no deferred cache state (a fresh run start
    // has none; the guard is belt-and-braces).  On success the whole region
    // — x87 segments plus the mov/lea bridges joining them — lowers as ONE
    // IR run, so the FP stack never round-trips through memory at the gaps.
    // On any failure nothing was emitted (build/gate failures precede
    // lower()); restore the plain run and fall through to the normal paths.
    {
        const bool ir_disabled = g_rosetta_config && g_rosetta_config->disable_x87_ir;
        if (!ir_disabled && cache.bridge_pending_total > 0 &&
            cache.bridge_pending_idx == static_cast<int16_t>(insn_idx) && cache.top_dirty == 0 &&
            cache.tag_push_pending == 0 && cache.deferred_pop_count == 0 &&
            cache.perm_dirty == 0) {
            const int total = cache.bridge_pending_total;
            const int x87_count = cache.bridge_pending_x87;
            const int plain_run = cache.bridge_pending_plain;
            cache.bridge_pending_total = 0;
            cache.bridge_pending_idx = -1;

            cache.set_run(total);  // pin GPRs across the whole region
            X87IR::IRFailReason bridge_reason = X87IR::IRFailReason::kNone;
            const int consumed =
                X87IR::compile_run(translation_result, instr_array, num_instrs, insn_idx, total,
                                   &bridge_reason, nullptr, nullptr, /*bridged=*/true);
            if (consumed == total) {
                if (cache.tally_bridge != 0xFFFFU) {
                    cache.tally_bridge =
                        static_cast<uint16_t>(cache.tally_bridge + (total - x87_count));
                }
                cache.tally_ir = static_cast<uint16_t>(cache.tally_ir + x87_count);
                mirror_tally();
                if (g_rosetta_config->log_bridge) {
                    std::fprintf(stderr,
                                 "[x87-bridge] joined region: total=%d x87=%d bridges=%d "
                                 "hash=0x%016llx idx=%lld\n",
                                 total, x87_count, total - x87_count,
                                 static_cast<unsigned long long>(cache.profile_hash),
                                 static_cast<long long>(insn_idx));
                }
                for (int i = 0; i < total; i++) {
                    cache.tick();
                }
                if (cache.active()) {
                    translation_result->free_gpr_mask = kGprScratchMask & ~cache.pinned_mask();
                } else {
                    translation_result->free_gpr_mask = kGprScratchMask;
                }
                translation_result->free_fpr_mask =
                    translation_result->_unoccupied_temporary_fprs_for_xmm_scalars;
                translation_result->_pinned_temporary_scalars = 0;
                return insn_idx + total;
            }
            // Fallback: nothing emitted; restore the plain x87 run.  Direct
            // field write — set_run() ignores lengths < 2, which would leave
            // run_remaining at the bridged total and keep the cache active
            // across the gap's stock-translated instructions.
            cache.run_remaining = static_cast<int16_t>(plain_run >= 2 ? plain_run : 0);
            if (cache.tally_bridge_fail != 0xFFFFU) {
                cache.tally_bridge_fail++;
            }
            if (g_rosetta_config->log_bridge) {
                std::fprintf(stderr,
                             "[x87-bridge] fallback: total=%d reason=%d hash=0x%016llx idx=%lld\n",
                             total, static_cast<int>(bridge_reason),
                             static_cast<unsigned long long>(cache.profile_hash),
                             static_cast<long long>(insn_idx));
            }
        }
    }

    {
        const bool ir_disabled = g_rosetta_config && g_rosetta_config->disable_x87_ir;
        if (!ir_disabled && cache.active()) {
            // Measurement-only override: profile_analyze can ask for compile_run
            // to fire regardless of the gate so we can score "what would IR emit
            // for this pattern if the gate were lifted?".  The emitted code is
            // not necessarily correct under the dirty conditions; the analyzer
            // discards it and only reads insn_buf.end / 4.
            const bool force_gate =
                g_rosetta_config != nullptr && g_rosetta_config->force_x87_ir_gate != 0;
            bool gate_refused = false;
            // Speculative-flush rollback state.  The gate-refusal cascade
            // may emit a small flush (3-14 ARM) and clear a deferred cache
            // field, then fall through to compile_run.  If compile_run
            // subsequently bails on kFprPressure / kGprPressure, the
            // flush ARM is wasted — the next dispatch path (peephole /
            // single-op) handles the deferred state itself.  Rollback
            // rewinds insn_buf.end and restores the cleared cache field.
            //
            // perm_dirty participates unconditionally.  top_dirty and
            // deferred_pop default ON since 2026-05-06: the
            // X87IRLower::lower() prologue flush (`855a424`) closed the
            // cascade hole that previously corrupted WoW geom + weapon
            // when these branches rolled back.  X87_ENABLE_ROLLBACK_*=0
            // remains as a per-branch kill-switch for bisect / future
            // diagnostic work.  X87_LOG_ROLLBACK=1 prints one stdout
            // line per firing.
            //
            // The standalone tag_push_pending refusal arm was removed
            // 2026-05-06: lower()'s prologue at X87IRLower.cpp:343-350
            // emits the tag clear when it runs, and compile_run's
            // FPR/GPR pressure check (X87IRLower.cpp:1536-1564) bails
            // before lower(), so cache.tag_push_pending is preserved on
            // bail.  The cascade absorbs the common (top_dirty,
            // tag_push) co-set case at the top_dirty arm; the rare
            // tag_push-only case falls through with no gate-side flush.
            const uint64_t saved_buf_end = translation_result->insn_buf.end;
            const int8_t saved_perm_dirty = cache.perm_dirty;
            int8_t saved_perm[8];
            for (int i = 0; i < 8; i++) {
                saved_perm[i] = cache.perm[i];
            }
            // Captured unconditionally (cheap); restored only when the
            // matching X87_ENABLE_ROLLBACK_* knob is set.
            const int8_t saved_top_dirty = cache.top_dirty;
            const int8_t saved_tag_push_pending = cache.tag_push_pending;
            const int8_t saved_deferred_pop_count = cache.deferred_pop_count;
            bool flushed = false;
            // Track which branch set `flushed` so the X87_LOG_ROLLBACK
            // diagnostic can name it.  kNone when no branch fired.
            enum RollbackBranch : uint8_t {
                kRollbackNone = 0,
                kRollbackTopDirty,
                kRollbackDeferredPop,
                kRollbackPermDirty,
            } flushed_branch = kRollbackNone;
            if (!force_gate) {
                if (cache.run_remaining < 3) {
                    cache.tally_ir_gate_short_run =
                        static_cast<uint16_t>(cache.tally_ir_gate_short_run + 1);
                    bump_max_run(profile::kIRGateReasonShortRun);
                    gate_refused = true;
                } else if (cache.top_dirty != 0) {
                    // FLUSH top_dirty only (3 ARM).  tag_push_pending is
                    // commonly set alongside (x87_push set both); we leave
                    // it untouched here.  If compile_run consumes the run
                    // and lower() runs, lower's prologue
                    // (X87IRLower.cpp:343-350) emits the tag clear.  If
                    // compile_run bails before lower (FPR / GPR pressure),
                    // cache.tag_push_pending is still set; the next call
                    // hits this branch again or the deferred_pop /
                    // perm_dirty arms.  The standalone tag_push refusal arm
                    // was removed 2026-05-06: lower()'s prologue makes the
                    // fall-through correct, and the cascade absorbs the
                    // common (top_dirty, tag_push) co-set case here.
                    // Override via X87_GATE_FLUSH_THRESHOLD; 0 = default 3.
                    const int kMinRunForFlush =
                        (g_rosetta_config != nullptr &&
                         g_rosetta_config->x87_ir_gate_flush_threshold_top_dirty != 0)
                            ? g_rosetta_config->x87_ir_gate_flush_threshold_top_dirty
                            : 3;
                    if (cache.gprs_valid && cache.run_remaining >= kMinRunForFlush) {
                        AssemblerBuffer& buf = translation_result->insn_buf;
                        const int Wd_tmp = alloc_free_gpr(*translation_result);
                        emit_store_top(buf, cache.base_gpr, cache.top_gpr, Wd_tmp);
                        free_gpr(*translation_result, Wd_tmp);
                        cache.top_dirty = 0;
                        bump_max_run(profile::kIRGateReasonTopDirty);
                        if (cache.profile_bid != profile::kOverflowId &&
                            cache.prev_x87_opcode != 0xFFFFU) {
                            profile::set_block_top_dirty_predecessor(cache.profile_bid,
                                                                     cache.prev_x87_opcode);
                        }
                        mirror_gate_counters();
                        // Per-branch kill-switch (default on).  Composes
                        // hash-list bounds via rollback_allowed(); with
                        // empty hash lists this is equivalent to
                        // "branch enable knob set?".
                        if (rollback_allowed(g_rosetta_config != nullptr
                                                 ? g_rosetta_config->x87_enable_rollback_top_dirty
                                                 : 0)) {
                            flushed = true;
                            flushed_branch = kRollbackTopDirty;
                        }
                        // gate_refused stays false — fall through to compile_run.
                    } else {
                        cache.tally_ir_gate_top_dirty =
                            static_cast<uint16_t>(cache.tally_ir_gate_top_dirty + 1);
                        bump_max_run(profile::kIRGateReasonTopDirty);
                        gate_refused = true;
                    }
                } else if (cache.deferred_pop_count != 0) {
                    // FLUSH deferred_pop_count alone (2 + 6*count ARM).  Reaches
                    // here when top_dirty=0 (the top_dirty branch fired earlier
                    // in the run + IR partial-consumed leaving deferred_pop_count
                    // behind).  Cap on count<=2 keeps the flush cost under
                    // ~14 ARM; count>=3 is rare.
                    // Override via X87_GATE_FLUSH_THRESHOLD_DEFERRED_POP; 0 = default 3.
                    const int kMinRunForFlush =
                        (g_rosetta_config != nullptr &&
                         g_rosetta_config->x87_ir_gate_flush_threshold_deferred_pop != 0)
                            ? g_rosetta_config->x87_ir_gate_flush_threshold_deferred_pop
                            : 3;
                    if (cache.gprs_valid && cache.run_remaining >= kMinRunForFlush &&
                        cache.deferred_pop_count <= 2) {
                        AssemblerBuffer& buf = translation_result->insn_buf;
                        const int Wd_tmp = alloc_free_gpr(*translation_result);
                        const int Wd_tmp2 = alloc_free_gpr(*translation_result);
                        const int Wd_tagw = alloc_free_gpr(*translation_result);
                        emit_x87_tag_set_empty_batch(buf, cache.base_gpr, cache.top_gpr, Wd_tmp,
                                                     Wd_tmp2, Wd_tagw, cache.deferred_pop_count);
                        free_gpr(*translation_result, Wd_tagw);
                        free_gpr(*translation_result, Wd_tmp2);
                        free_gpr(*translation_result, Wd_tmp);
                        cache.deferred_pop_count = 0;
                        bump_max_run(profile::kIRGateReasonDeferredPop);
                        mirror_gate_counters();
                        // Per-branch kill-switch (default on).  Composes
                        // hash-list bounds via rollback_allowed().
                        if (rollback_allowed(
                                g_rosetta_config != nullptr
                                    ? g_rosetta_config->x87_enable_rollback_deferred_pop
                                    : 0)) {
                            flushed = true;
                            flushed_branch = kRollbackDeferredPop;
                        }
                    } else {
                        cache.tally_ir_gate_deferred_pop =
                            static_cast<uint16_t>(cache.tally_ir_gate_deferred_pop + 1);
                        bump_max_run(profile::kIRGateReasonDeferredPop);
                        gate_refused = true;
                    }
                } else if (cache.perm_dirty) {
                    // FLUSH perm_dirty alone (~6 ARM for the typical 2-cycle
                    // FXCH-ST(1) case).  By design invariant (every push/pop
                    // path calls perm_flush_before_stack_change first),
                    // perm_dirty=1 implies all other deferred flags=0, so this
                    // branch always fires alone in the cascade.
                    // Override via X87_GATE_FLUSH_THRESHOLD_PERM_DIRTY; 0 = default 3.
                    const int kMinRunForFlush =
                        (g_rosetta_config != nullptr &&
                         g_rosetta_config->x87_ir_gate_flush_threshold_perm_dirty != 0)
                            ? g_rosetta_config->x87_ir_gate_flush_threshold_perm_dirty
                            : 3;
                    if (cache.gprs_valid && cache.run_remaining >= kMinRunForFlush) {
                        AssemblerBuffer& buf = translation_result->insn_buf;
                        const int Wd_tmp = alloc_free_gpr(*translation_result);
                        const int Dd_save = alloc_free_fpr(*translation_result);
                        const int Dd_chain = alloc_free_fpr(*translation_result);
                        const int Xst_base =
                            (cache.gprs_valid && cache.st_base_valid) ? cache.st_base_gpr : -1;
                        emit_x87_perm_flush(buf, cache.base_gpr, cache.top_gpr, Wd_tmp, cache.perm,
                                            Xst_base, Dd_save, Dd_chain);
                        free_fpr(*translation_result, Dd_chain);
                        free_fpr(*translation_result, Dd_save);
                        free_gpr(*translation_result, Wd_tmp);
                        cache.reset_perm();
                        flushed = true;
                        flushed_branch = kRollbackPermDirty;
                        bump_max_run(profile::kIRGateReasonPermDirty);
                        mirror_gate_counters();
                    } else {
                        cache.tally_ir_gate_perm_dirty =
                            static_cast<uint16_t>(cache.tally_ir_gate_perm_dirty + 1);
                        bump_max_run(profile::kIRGateReasonPermDirty);
                        gate_refused = true;
                    }
                }
            }
            if (gate_refused) {
                mirror_gate_counters();
            } else {
                X87IR::IRFailReason ir_reason = X87IR::IRFailReason::kNone;
                int ir_peak_gprs = 0;
                // out_fail_opcode is only useful when we'll record it into the
                // profile side-table.  Pass nullptr in non-profile builds so
                // compile_run skips the conditional store entirely.
                const bool profiling_block = cache.profile_bid != profile::kOverflowId;
                uint16_t ir_fail_opcode = profile::kNoBuildFailOpcode;
                const int ir_consumed = X87IR::compile_run(
                    translation_result, instr_array, num_instrs, insn_idx, cache.run_remaining,
                    &ir_reason, &ir_peak_gprs, profiling_block ? &ir_fail_opcode : nullptr);
                if (profiling_block && ir_fail_opcode != profile::kNoBuildFailOpcode) {
                    profile::set_block_build_fail_op(cache.profile_bid, ir_fail_opcode);
                }
                if (ir_peak_gprs > 0) {
                    const auto peak_unsigned = static_cast<uint32_t>(ir_peak_gprs);
                    if (peak_unsigned > cache.tally_max_gpr_peak) {
                        cache.tally_max_gpr_peak = static_cast<uint16_t>(peak_unsigned);
                    }
                }
                if (ir_consumed == 0) {
                    switch (ir_reason) {
                        case X87IR::IRFailReason::kBuildFail:
                            cache.tally_ir_build_fail =
                                static_cast<uint16_t>(cache.tally_ir_build_fail + 1);
                            break;
                        case X87IR::IRFailReason::kFprPressure:
                            cache.tally_ir_fpr_fail =
                                static_cast<uint16_t>(cache.tally_ir_fpr_fail + 1);
                            break;
                        case X87IR::IRFailReason::kGprPressure:
                            cache.tally_ir_gpr_fail =
                                static_cast<uint16_t>(cache.tally_ir_gpr_fail + 1);
                            break;
                        case X87IR::IRFailReason::kNone:
                        case X87IR::IRFailReason::kBridgePartial:
                            // kBridgePartial never reaches this switch (the
                            // bridged attempt handles its own fallback), but
                            // keep -Wswitch exhaustive.
                            break;
                    }
                    mirror_tally();
                    // Speculative-flush rollback: if the gate-cascade flush
                    // emitted ARM and compile_run bailed, the flush ARM is
                    // wasted — peephole / single-op handles the deferred
                    // state itself.  Rewind insn_buf.end and restore the
                    // cleared cache field(s) to pre-flush state.  The
                    // perm_dirty branch always restores; top_dirty /
                    // deferred_pop only when flushed_branch indicates
                    // the matching opt-in enable knob fired.  Diagnostic
                    // counters (bump_max_run, mirror_gate_counters) stay
                    // — they record that an attempt was made.
                    if (flushed) {
                        const uint64_t pre_rewind_end = translation_result->insn_buf.end;
                        const int8_t pre_top_dirty = cache.top_dirty;
                        const int8_t pre_tag_push_pending = cache.tag_push_pending;
                        const int8_t pre_deferred_pop_count = cache.deferred_pop_count;
                        const int8_t pre_perm_dirty = cache.perm_dirty;
                        translation_result->insn_buf.end = saved_buf_end;
                        // perm_dirty rollback is unconditional (not gated by
                        // an opt-in enable); restores cache.perm[] + perm_dirty.
                        if (flushed_branch == kRollbackPermDirty) {
                            cache.perm_dirty = saved_perm_dirty;
                            for (int i = 0; i < 8; i++) {
                                cache.perm[i] = saved_perm[i];
                            }
                        }
                        // Branch-specific cache restores for the opt-in
                        // top_dirty / deferred_pop rollback paths.
                        if (flushed_branch == kRollbackTopDirty) {
                            cache.top_dirty = saved_top_dirty;
                        }
                        if (flushed_branch == kRollbackDeferredPop) {
                            cache.deferred_pop_count = saved_deferred_pop_count;
                        }
                        if (g_rosetta_config != nullptr &&
                            g_rosetta_config->x87_log_rollback != 0) {
                            const char* branch_name = "none";
                            switch (flushed_branch) {
                                case kRollbackTopDirty:
                                    branch_name = "top_dirty";
                                    break;
                                case kRollbackDeferredPop:
                                    branch_name = "deferred_pop";
                                    break;
                                case kRollbackPermDirty:
                                    branch_name = "perm_dirty";
                                    break;
                                case kRollbackNone:
                                    break;
                            }
                            const char* fail_name = "None";
                            switch (ir_reason) {
                                case X87IR::IRFailReason::kBuildFail:
                                    fail_name = "BuildFail";
                                    break;
                                case X87IR::IRFailReason::kFprPressure:
                                    fail_name = "FprPressure";
                                    break;
                                case X87IR::IRFailReason::kGprPressure:
                                    fail_name = "GprPressure";
                                    break;
                                case X87IR::IRFailReason::kBridgePartial:
                                    fail_name = "BridgePartial";
                                    break;
                                case X87IR::IRFailReason::kNone:
                                    break;
                            }
                            // block_id = cache.profile_bid when X87_PROFILE is on
                            // (kOverflowId otherwise).  hash is FNV-1a IR-content
                            // hash, populated unconditionally — stable across
                            // launches and the right key for cross-run bisect.
                            // insn_idx + run_remaining pin the rollback to a
                            // position inside the IR stream so an offline tool
                            // can lift the surrounding instructions from the
                            // .prof file.
                            std::printf(
                                "[rollback] branch=%s ir_fail=%s buf_end_delta=%lld "
                                "td %d->%d tp %d->%d dpc %d->%d pd %d->%d "
                                "opcode=0x%04x pc=0x%08x "
                                "block_id=%u hash=0x%016llx insn_idx=%lld "
                                "run_remaining=%d\n",
                                branch_name, fail_name,
                                static_cast<long long>(pre_rewind_end -
                                                       translation_result->insn_buf.end),
                                pre_top_dirty, cache.top_dirty, pre_tag_push_pending,
                                cache.tag_push_pending, pre_deferred_pop_count,
                                cache.deferred_pop_count, pre_perm_dirty, cache.perm_dirty,
                                cur_instr->opcode(), cur_instr->pc,
                                static_cast<unsigned>(cache.profile_bid),
                                static_cast<unsigned long long>(cache.profile_hash),
                                static_cast<long long>(insn_idx),
                                static_cast<int>(cache.run_remaining));
                        }
                    }
                }
                if (ir_consumed > 0) {
                    cache.tally_ir = static_cast<uint16_t>(cache.tally_ir + ir_consumed);
                    mirror_tally();
                    for (int i = 0; i < ir_consumed; i++) {
                        cache.tick();
                    }
                    if (cache.active()) {
                        translation_result->free_gpr_mask = kGprScratchMask & ~cache.pinned_mask();
                    } else {
                        translation_result->free_gpr_mask = kGprScratchMask;
                    }
                    translation_result->free_fpr_mask =
                        translation_result->_unoccupied_temporary_fprs_for_xmm_scalars;
                    translation_result->_pinned_temporary_scalars = 0;
                    return insn_idx + ir_consumed;
                }
            }
        }
    }

    // ── Peephole: try 2-instruction fusion patterns ─────────────────────────
    const uint64_t fusions_mask = g_rosetta_config ? g_rosetta_config->disabled_fusions_mask : 0;
    const auto fused = TranslatorX87::try_peephole(translation_result, instr_array, num_instrs,
                                                   insn_idx, fusions_mask);

    // ── Single-op fast path: fused emitters for isolated fld/fst/fstp ──────
    // Only when the cache is inactive (run==1, or cache disabled): no pinned
    // GPRs and no deferred TOP/tag/perm state can exist then — x87_end flushes
    // everything at run end.  Counts into tally_single so profile_analyze
    // stays comparable across configs.
    bool single_fast = false;
    if (!fused && !cache.active() &&
        !(g_rosetta_config && g_rosetta_config->disable_x87_single_fast)) {
        single_fast = TranslatorX87::try_translate_single_fast(translation_result, cur_instr);
    }

    if (!fused && !single_fast) {
        switch (opcode) {
            case Opcode::kOpcodeName_fldz:
                TranslatorX87::translate_fldz(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fld1:
                TranslatorX87::translate_fld1(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fldl2e:
                TranslatorX87::translate_fldl2e(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fldl2t:
                TranslatorX87::translate_fldl2t(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fldlg2:
                TranslatorX87::translate_fldlg2(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fldln2:
                TranslatorX87::translate_fldln2(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fldpi:
                TranslatorX87::translate_fldpi(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fld:
                TranslatorX87::translate_fld(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fild:
                TranslatorX87::translate_fild(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fbld:
                TranslatorX87::translate_fbld(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fadd:
                TranslatorX87::translate_fadd(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_faddp:
                TranslatorX87::translate_faddp(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fiadd:
                TranslatorX87::translate_fiadd(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fsub:
            case Opcode::kOpcodeName_fsubr:
                TranslatorX87::translate_fsub(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fsubp:
            case Opcode::kOpcodeName_fsubrp:
                TranslatorX87::translate_fsubp(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fdiv:
            case Opcode::kOpcodeName_fdivr:
                TranslatorX87::translate_fdiv(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fdivp:
            case Opcode::kOpcodeName_fdivrp:
                TranslatorX87::translate_fdivp(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fmul:
            case Opcode::kOpcodeName_fmulp:
                TranslatorX87::translate_fmul(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fst:
            case Opcode::kOpcodeName_fst_stack:
            case Opcode::kOpcodeName_fstp:
            case Opcode::kOpcodeName_fstp_stack:
                TranslatorX87::translate_fst(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fstsw:
                TranslatorX87::translate_fstsw(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fxam:
                TranslatorX87::translate_fxam(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fdecstp:
                TranslatorX87::translate_fdecstp(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fincstp:
                TranslatorX87::translate_fincstp(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_ffree:
                TranslatorX87::translate_ffree(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fxtract:
                TranslatorX87::translate_fxtract(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fscale:
                TranslatorX87::translate_fscale(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fbstp:
                TranslatorX87::translate_fbstp(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_frstor:
                TranslatorX87::translate_frstor(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fsave:
                TranslatorX87::translate_fsave(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fcom:
            case Opcode::kOpcodeName_fcomp:
            case Opcode::kOpcodeName_fcompp:
            case Opcode::kOpcodeName_fucom:
            case Opcode::kOpcodeName_fucomp:
            case Opcode::kOpcodeName_fucompp:
                TranslatorX87::translate_fcom(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fxch:
                TranslatorX87::translate_fxch(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fchs:
                TranslatorX87::translate_fchs(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fabs:
                TranslatorX87::translate_fabs(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fsqrt:
                TranslatorX87::translate_fsqrt(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fistp:
                TranslatorX87::translate_fistp(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fisttp:
                TranslatorX87::translate_fisttp(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fidiv:
                TranslatorX87::translate_fidiv(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fimul:
                TranslatorX87::translate_fimul(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fisub:
                TranslatorX87::translate_fisub(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fidivr:
                TranslatorX87::translate_fidivr(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_frndint:
                TranslatorX87::translate_frndint(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fcomi:
            case Opcode::kOpcodeName_fcomip:
            case Opcode::kOpcodeName_fucomi:
            case Opcode::kOpcodeName_fucomip:
                TranslatorX87::translate_fcomi(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_ftst:
                TranslatorX87::translate_ftst(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fist:
                TranslatorX87::translate_fist(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fisubr:
                TranslatorX87::translate_fisubr(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_ficom:
            case Opcode::kOpcodeName_ficomp:
                TranslatorX87::translate_ficom(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fcmovb:
            case Opcode::kOpcodeName_fcmovbe:
            case Opcode::kOpcodeName_fcmove:
            case Opcode::kOpcodeName_fcmovnb:
            case Opcode::kOpcodeName_fcmovnbe:
            case Opcode::kOpcodeName_fcmovne:
            case Opcode::kOpcodeName_fcmovu:
            case Opcode::kOpcodeName_fcmovnu:
                TranslatorX87::translate_fcmov(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fldcw:
                TranslatorX87::translate_fldcw(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fnstcw:
                TranslatorX87::translate_fnstcw(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fnop:
            case Opcode::kOpcodeName_fdisi:  // 8087 FPU-int disable; NOP on 80287+
            case Opcode::kOpcodeName_feni:   // 8087 FPU-int enable; NOP on 80287+
                TranslatorX87::translate_fnop(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fsin:
                TranslatorX87::translate_fsin(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fcos:
                TranslatorX87::translate_fcos(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_f2xm1:
                TranslatorX87::translate_f2xm1(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fpatan:
                TranslatorX87::translate_fpatan(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fsincos:
                TranslatorX87::translate_fsincos(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fptan:
                TranslatorX87::translate_fptan(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fyl2x:
                TranslatorX87::translate_fyl2x(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fyl2xp1:
                TranslatorX87::translate_fyl2xp1(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fprem:
                TranslatorX87::translate_fprem(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fprem1:
                TranslatorX87::translate_fprem1(translation_result, cur_instr);
                break;

            // Identical body to default: is intentional — the explicit
            // case documents the "deliberate fall-through" set and pairs
            // with the sidecar's kKnownFallThrough allowlist that
            // suppresses the UNHANDLED warning for these opcodes only.
            // Don't merge with default:.
            //
            // Deliberate fall-through to stock:
            //   • fclex/finit/fldenv/fstenv: metadata-only ops with no
            //     stack-aware path; inlining buys nothing measurable
            //     (parity at best, fstenv was a 0.66× regression).
            //     Test coverage in test_*_compose.c confirms no
            //     m108-style internal-offset bug — see
            //     project_native_rosetta_lazy_f80.md.
            //   • fxsave/fxrstor: SSE-era extended save/restore
            //     (512 B incl. 8 ST f80 slots + 16 XMM/YMM + MXCSR).
            //     Inlining would inherit frstor's 0.38× eager-f80
            //     regression; we don't translate SSE.
            //
            // For all six: is_handled_x87 returns false, so the run
            // terminates before this point and the preceding handled
            // op's x87_end has already flushed deferred state to
            // memory — stock reads coherent X87State via x22.
            case Opcode::kOpcodeName_fclex:
            case Opcode::kOpcodeName_finit:
            case Opcode::kOpcodeName_fldenv:
            case Opcode::kOpcodeName_fstenv:
            case Opcode::kOpcodeName_fxsave:
            case Opcode::kOpcodeName_fxrstor:
                cache.tally_ft = static_cast<uint16_t>(cache.tally_ft + 1);
                mirror_tally();
                return std::nullopt;

            default:
                // Returning nullopt means "I have no handler for this
                // opcode."  In the runtime/sidecar flow, the stub's FILTER
                // prologue routes only x87 opcodes to us, so reaching here
                // with an x87 opcode means an op we *forgot* to handle —
                // log a loud line so we notice.  Add a translate_* and
                // dispatch case, or list it as deliberate fall-through
                // above (and extend kKnownFallThrough on the sidecar side
                // for a new stock-only op like fxsave/fxrstor).
                // In the offline AOT flow (CustomTranslationHook), every
                // non-x87 op also reaches here; that path falls back to
                // stock translate_insn unchanged.  is_x87_opcode gates the
                // log so the AOT path isn't spammed for non-x87 ops.
                //
                // Critical: do NOT mutate free_gpr_mask / free_fpr_mask
                // before returning nullopt.  Stock's translate_insn (run
                // by the stub's STASH abs-jump in JIT mode, or by
                // original_translate_insn in AOT mode) carries its own
                // register-allocation state across instructions in a
                // basic block.  Overwriting those masks with our scratch
                // values forces stock to redo allocation as if every
                // instruction starts a fresh block, costing several extra
                // ARM instructions per instruction.  Pass-through clean.
                if (is_x87_opcode(opcode)) {
                    const char* name = (opcode < kOpcodeNames.size()) ? kOpcodeNames[opcode] : "?";
                    fprintf(stdout,
                            "[rosettax87] unexpected x87 opcode %s (0x%x) reached "
                            "translator default — add a translate_* and dispatch "
                            "case, or list it as deliberate fall-through above\n",
                            name, static_cast<unsigned>(opcode));
                    fflush(stdout);
                }
                cache.tally_ft = static_cast<uint16_t>(cache.tally_ft + 1);
                mirror_tally();
                return std::nullopt;
        }
    }

    // OPT-1: Tick the cache (decrements run counter; releases on expiry).
    // Then reset the mask, excluding any GPRs still pinned by the cache.
    // Fused pairs consumed 2 instructions — tick twice.
    const int consumed = fused.value_or(1);
    if (fused.has_value()) {
        cache.tally_peep = static_cast<uint16_t>(cache.tally_peep + consumed);
    } else {
        cache.tally_single = static_cast<uint16_t>(cache.tally_single + 1);
    }
    mirror_tally();
    for (int i = 0; i < consumed; i++) {
        cache.tick();
    }

    if (cache.active()) {
        translation_result->free_gpr_mask = kGprScratchMask & ~cache.pinned_mask();
    } else {
        translation_result->free_gpr_mask = kGprScratchMask;
    }
    translation_result->free_fpr_mask =
        translation_result->_unoccupied_temporary_fprs_for_xmm_scalars;
    translation_result->_pinned_temporary_scalars = 0;

    // Track the most-recent x87 op translated in this block via the
    // peep+single fallback — the IR-gate top_dirty diagnostic reads this
    // to attribute which preceding op left top_dirty=1.  IR success path
    // doesn't update this (it returns earlier at line 225); IR-consumed
    // ops can't be the immediate predecessor of a NEXT-call top_dirty
    // refusal because IR resets all dirty state on success.
    cache.prev_x87_opcode = opcode;

    return insn_idx + consumed;
}
