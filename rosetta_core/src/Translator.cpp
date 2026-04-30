#include "rosetta_core/Translator.h"

#include <cstdio>

#include "rosetta_core/CoreConfig.h"
#include "rosetta_core/IRInstr.h"
#include "rosetta_core/Opcode.h"
#include "rosetta_core/TranslationResult.h"
#include "rosetta_core/TranslatorX87.h"
#include "rosetta_core/TranslatorX87Fusion.h"
#include "rosetta_core/X87Cache.h"
#include "rosetta_core/X87IR.h"
#include "rosetta_config/Config.h"
#include "TranslatorX87Internal.hpp"


auto Translator::translate_instruction(TranslationResult* translation_result, IRBlock* block,
                                       IRInstr* instr_array, int64_t num_instrs, int64_t insn_idx)
    -> std::optional<int64_t> {
    const auto cur_instr = &instr_array[insn_idx];
    const auto opcode = cur_instr->opcode;
    auto& cache = translation_result->x87_cache;

    // If extended FPR scratch is enabled, upgrade the mask from 8-reg to 16-reg form on first use.
    if (g_rosetta_config && g_rosetta_config->extended_fpr_scratch &&
        translation_result->free_fpr_mask == kFprScratchMask) {
        translation_result->free_fpr_mask = kFprScratchMaskExt;
    }

    // ── OPT-1: x87 cross-instruction cache management ───────────────────────
    // Invalidate the cache if we've moved to a different block (between blocks,
    // branches may have executed non-x87 code that clobbers scratch GPRs).
    // Then, if no cache is active, scan ahead to find the length of the current
    // consecutive x87 run.  x87_cache_set_run only activates if run >= 2.
    {
        if (block != cache.prev_block) {
            cache.invalidate(translation_result->free_gpr_mask, kGprScratchMask);
            cache.prev_block = block;
        }
        if (!cache.active()) {
            const bool cache_disabled = g_rosetta_config && g_rosetta_config->disable_x87_cache;
            if (!cache_disabled) {
                const int run = X87Cache::lookahead(instr_array, num_instrs, insn_idx);
                cache.set_run(run);
            }
        }
    }

    // ── IR pipeline: try whole-run optimization for runs of 3+ ─────────────
    // The IR fires once at the start of a fresh run (no deferred cache state).
    // It builds an SSA-like IR, runs optimization passes (DSE, FMA, FCOM+FSTSW
    // fusion), and lowers directly to AArch64.
    {
        const bool ir_disabled = g_rosetta_config && g_rosetta_config->disable_x87_ir;
        if (!ir_disabled && cache.active() && cache.run_remaining >= 3 &&
            cache.top_dirty == 0 && cache.tag_push_pending == 0 &&
            cache.deferred_pop_count == 0 && !cache.perm_dirty) {
            const int ir_consumed = X87IR::compile_run(
                translation_result, instr_array, num_instrs, insn_idx, cache.run_remaining);
            if (ir_consumed > 0) {
                for (int i = 0; i < ir_consumed; i++)
                    cache.tick();
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

    // ── Peephole: try 2-instruction fusion patterns ─────────────────────────
    const uint64_t fusions_mask = g_rosetta_config ? g_rosetta_config->disabled_fusions_mask : 0;
    const auto fused =
        TranslatorX87::try_peephole(translation_result, instr_array, num_instrs, insn_idx,
                                    fusions_mask);

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

            case Opcode::kOpcodeName_fclex:
                TranslatorX87::translate_fclex(translation_result, cur_instr);
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

            case Opcode::kOpcodeName_finit:
                TranslatorX87::translate_finit(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fbstp:
                TranslatorX87::translate_fbstp(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fldenv:
                TranslatorX87::translate_fldenv(translation_result, cur_instr);
                break;

            case Opcode::kOpcodeName_fstenv:
                TranslatorX87::translate_fstenv(translation_result, cur_instr);
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

            case Opcode::kOpcodeName_fxsave:
            case Opcode::kOpcodeName_fxrstor:
                // fxsave/fxrstor touch x87 state (env header + 8 ST slots
                // overlap fsave's 108-byte layout) plus XMM0..15 + MXCSR.
                // We want to inline them eventually, but XMM↔V mapping and
                // NEON 128 b emit don't yet exist in this codebase. Print
                // a one-line diagnostic so we notice if a workload starts
                // using them, then fall through to stock.
                fprintf(stdout,
                        "[rosettax87] %s not yet handled (opcode 0x%x); "
                        "deferring to stock\n",
                        kOpcodeNames[opcode], static_cast<unsigned>(opcode));
                [[fallthrough]];
            default:
                // Hand translation back to stock for opcodes we don't
                // support — currently just the transcendentals, which are
                // already broken at the our-emit/stock-emit boundary
                // (ABI mismatch on x22:w23, f80↔f64 representation gap).
                // No flush of deferred state is emitted: stock would
                // misread it, and the transcendentals are slated to
                // route through sidecar IPC instead.
                translation_result->free_gpr_mask = kGprScratchMask;
                translation_result->free_fpr_mask =
                    translation_result->_unoccupied_temporary_fprs_for_xmm_scalars;
                return std::nullopt;
        }
    }

    // OPT-1: Tick the cache (decrements run counter; releases on expiry).
    // Then reset the mask, excluding any GPRs still pinned by the cache.
    // Fused pairs consumed 2 instructions — tick twice.
    const int consumed = fused.value_or(1);
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

    return insn_idx + consumed;
}
