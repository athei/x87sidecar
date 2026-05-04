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
    const auto opcode = cur_instr->opcode;
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
            cache.profile_bid = profile::kOverflowId;
            if (profile::counter_array_addr() != 0) {
                const uint64_t ir_hash =
                    profile::hash_ir_stream(instr_array, static_cast<size_t>(num_instrs));
                const uint32_t bid = profile::register_block(block, ir_hash);
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
            if (!force_gate) {
                if (cache.run_remaining < 3) {
                    cache.tally_ir_gate_short_run =
                        static_cast<uint16_t>(cache.tally_ir_gate_short_run + 1);
                    bump_max_run(profile::kIRGateReasonShortRun);
                    gate_refused = true;
                } else if (cache.top_dirty != 0) {
                    // FLUSH top_dirty only (3 ARM).  When tag_push_pending is
                    // also set (common after x87_push), we deliberately leave
                    // it: if IR consumes all ops the tick() chain clears
                    // tag_push as a side effect; on partial-consume (rare) the
                    // tag_push branch below catches it on the next call.
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
                        // gate_refused stays false — fall through to compile_run.
                    } else {
                        cache.tally_ir_gate_top_dirty =
                            static_cast<uint16_t>(cache.tally_ir_gate_top_dirty + 1);
                        bump_max_run(profile::kIRGateReasonTopDirty);
                        gate_refused = true;
                    }
                } else if (cache.tag_push_pending != 0) {
                    // FLUSH tag_push_pending alone (6 ARM).  Reaches here when
                    // top_dirty=0 but tag_push lingers — typically because the
                    // top_dirty branch fired earlier in the run, IR partial-
                    // consumed leaving tag_push set, and the next call now sees
                    // tag_push alone.
                    //
                    // Default stays at 8 (NOT 3 like the other three branches):
                    // empirically, lowering this branch to 3 corrupts WoW
                    // character-rotation matrix transforms while leaving weapon
                    // transforms intact (bisected 2026-05-04 with the per-branch
                    // X87_GATE_FLUSH_THRESHOLD_TAG_PUSH knob).  The other three
                    // gate branches lower to 3 cleanly; tag_push exposes a real
                    // IR-side mishandling on short runs after a flush-and-proceed.
                    // Override via X87_GATE_FLUSH_THRESHOLD_TAG_PUSH; 0 = default 8.
                    const int kMinRunForFlush =
                        (g_rosetta_config != nullptr &&
                         g_rosetta_config->x87_ir_gate_flush_threshold_tag_push != 0)
                            ? g_rosetta_config->x87_ir_gate_flush_threshold_tag_push
                            : 8;
                    if (cache.gprs_valid && cache.run_remaining >= kMinRunForFlush) {
                        AssemblerBuffer& buf = translation_result->insn_buf;
                        const int Wd_tmp = alloc_free_gpr(*translation_result);
                        const int Wd_tmp2 = alloc_free_gpr(*translation_result);
                        emit_x87_tag_clear(buf, cache.base_gpr, cache.top_gpr, Wd_tmp, Wd_tmp2);
                        free_gpr(*translation_result, Wd_tmp2);
                        free_gpr(*translation_result, Wd_tmp);
                        cache.tag_push_pending = 0;
                        bump_max_run(profile::kIRGateReasonTagPush);
                        mirror_gate_counters();
                    } else {
                        cache.tally_ir_gate_tag_push =
                            static_cast<uint16_t>(cache.tally_ir_gate_tag_push + 1);
                        bump_max_run(profile::kIRGateReasonTagPush);
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
                            break;
                    }
                    mirror_tally();
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

    if (!fused) {
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
