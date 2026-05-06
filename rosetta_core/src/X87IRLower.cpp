#include <algorithm>
#include <cstdint>
#include <cstring>
#include <utility>

#include "rosetta_core/AssemblerBuffer.h"
#include "rosetta_core/AssemblerHelpers.hpp"
#include "rosetta_core/Config.h"
#include "rosetta_core/CoreConfig.h"
#include "rosetta_core/IROperand.h"
#include "rosetta_core/Register.h"
#include "rosetta_core/TranscendentalHelper.h"
#include "rosetta_core/TranslationResult.h"
#include "rosetta_core/TranslatorHelpers.hpp"
#include "rosetta_core/TranslatorX87.h"
#include "rosetta_core/TranslatorX87Helpers.hpp"
#include "rosetta_core/TranslatorX87Transcendental.hpp"
#include "rosetta_core/X87Cache.h"
#include "rosetta_core/X87IR.h"

// Internal helpers from TranslatorX87Internal.hpp that we need.
namespace TranslatorX87 {
inline auto x87_begin(TranslationResult& a1, AssemblerBuffer& buf) -> std::pair<int, int> {
    if (a1.x87_cache.run_remaining > 0 && a1.x87_cache.gprs_valid) {
        return {a1.x87_cache.base_gpr, a1.x87_cache.top_gpr};
    }
    const int Xbase = alloc_gpr(a1, 0);
    const int Wd_top = alloc_gpr(a1, 1);
    emit_x87_base(buf, a1, Xbase);
    emit_load_top(buf, a1, Xbase, Wd_top);
    if (a1.x87_cache.run_remaining > 0) {
        a1.x87_cache.base_gpr = static_cast<int8_t>(Xbase);
        a1.x87_cache.top_gpr = static_cast<int8_t>(Wd_top);
        const int Xst_base = alloc_gpr(a1, 6);
        emit_add_imm(buf, 1, 0, 0, 0, kX87RegFileOff, Xbase, Xst_base);
        a1.x87_cache.st_base_gpr = static_cast<int8_t>(Xst_base);
        a1.x87_cache.gprs_valid = 1;
        a1.x87_cache.st_base_valid = 1;
    }
    return {Xbase, Wd_top};
}
inline int x87_get_st_base(TranslationResult& a1) {
    return (a1.x87_cache.gprs_valid && a1.x87_cache.st_base_valid) ? a1.x87_cache.st_base_gpr : -1;
}
inline void x87_flush_deferred_pops(AssemblerBuffer& buf, TranslationResult& a1, int Xbase,
                                    int Wd_top, int Wd_tmp) {
    if (a1.x87_cache.deferred_pop_count > 0) {
        const int Wd_tmp2 = alloc_free_gpr(a1);
        const int Wd_tagw = alloc_free_gpr(a1);
        emit_x87_tag_set_empty_batch(buf, Xbase, Wd_top, Wd_tmp, Wd_tmp2, Wd_tagw,
                                     a1.x87_cache.deferred_pop_count);
        free_gpr(a1, Wd_tagw);
        free_gpr(a1, Wd_tmp2);
        a1.x87_cache.deferred_pop_count = 0;
    }
}
inline void perm_flush_before_stack_change(AssemblerBuffer& buf, TranslationResult& a1, int Xbase,
                                           int Wd_top, int Wd_tmp) {
    if (a1.x87_cache.perm_dirty) {
        const int Xst_base = x87_get_st_base(a1);
        const int Dd_save = alloc_free_fpr(a1);
        const int Dd_chain = alloc_free_fpr(a1);
        emit_x87_perm_flush(buf, Xbase, Wd_top, Wd_tmp, a1.x87_cache.perm, Xst_base, Dd_save,
                            Dd_chain);
        free_fpr(a1, Dd_chain);
        free_fpr(a1, Dd_save);
        a1.x87_cache.reset_perm();
    }
}
}  // namespace TranslatorX87

namespace X87IR {

static constexpr int16_t kX87TagWordImm12 = kX87TagWordOff / 2;  // = 2

// ── FPR assignment ──────────────────────────────────────────────────────────

struct FPRState {
    int8_t node_fpr[kMaxNodes];   // D register for each node, or -1
    int16_t last_use[kMaxNodes];  // last node index that uses each node, or -1

    void compute_last_uses(const Context& ctx) {
        memset(last_use, -1, sizeof(last_use));
        memset(node_fpr, -1, sizeof(node_fpr));
        for (int i = 0; i < ctx.num_nodes; i++) {
            const auto& n = ctx.nodes[i];
            if (n.flags & kDead) {
                continue;
            }
            for (short input : n.inputs) {
                if (input >= 0) {
                    last_use[input] = static_cast<int16_t>(i);
                }
            }
        }
        // Final stack values are live past all nodes.
        for (short val : ctx.slot_val) {
            if (val >= 0 && val < ctx.num_nodes) {
                last_use[val] = static_cast<int16_t>(ctx.num_nodes);
            }
        }
    }

    // Free FPRs whose last use was node `i`.
    void free_dead_inputs(TranslationResult& result, const Node& n, int i) {
        for (short in : n.inputs) {
            if (in >= 0 && last_use[in] == i && node_fpr[in] >= 0) {
                free_fpr(result, node_fpr[in]);
                node_fpr[in] = -1;
            }
        }
    }

    [[nodiscard]] int get(int16_t node_id) const {
        if (node_id < 0 || node_id >= kMaxNodes) {
            return -1;
        }
        return node_fpr[node_id];
    }

    // Try to reuse an FPR from a dying input of node `i`.
    // Returns the FPR register number, or -1 if no reuse is possible.
    // On success, sets node_fpr[input] = -1 so free_dead_inputs skips it.
    // Caller MUST capture input FPRs via get() BEFORE calling this.
    int try_reuse_input(const Context& ctx, int i) {
        const auto& n = ctx.nodes[i];
        // Prefer inputs[0] (Dn, natural accumulator position)
        for (short in : n.inputs) {
            if (in >= 0 && last_use[in] == i && node_fpr[in] >= 0) {
                int fpr = static_cast<unsigned char>(node_fpr[in]);
                node_fpr[in] = -1;  // claimed — free_dead_inputs will skip
                return fpr;
            }
        }
        return -1;
    }
};

// ── StoreF32-run coalescing helpers ─────────────────────────────────────────
//
// Inspect a Node's mem_operand to decide whether two consecutive StoreF32s
// can be merged into a single STR Q (4 floats) or STP S, S (2 floats).  We
// only merge MemRefs that have a base register but no index register and no
// segment override — anything more complex is left to the scalar fallback
// where compute_operand_address handles the full addressing mode.
//
// See ~/.claude/plans/recall-memory-analyze-the-snappy-scott.md and
// feedback_ir_pipeline_storef32_baseline.md for the motivation.

struct SimpleMemRef {
    bool ok;           // true → operand is a "simple" base+disp MemRef
    uint8_t base_reg;  // x86 base register index
    int64_t disp;      // signed displacement
};

static SimpleMemRef get_simple_mem(const Node& n) {
    if (n.mem_operand == nullptr) {
        return {.ok = false, .base_reg = 0, .disp = 0};
    }
    const IROperand& op = *n.mem_operand;
    if (op.kind != IROperandKind::MemRef) {
        return {.ok = false, .base_reg = 0, .disp = 0};
    }
    if ((op.mem.mem_flags & 1U) == 0) {  // no base register
        return {.ok = false, .base_reg = 0, .disp = 0};
    }
    if ((op.mem.mem_flags & 2U) != 0) {  // index register present
        return {.ok = false, .base_reg = 0, .disp = 0};
    }
    if (op.mem.seg_override != 0) {
        return {.ok = false, .base_reg = 0, .disp = 0};
    }
    return {.ok = true, .base_reg = op.mem.base_reg, .disp = op.mem.disp};
}

// True iff four consecutive StoreF32 nodes target a 16-byte-aligned region of
// 16 contiguous bytes off the same x86 base register — the precondition for
// emitting a single `STR Q` after the broadcast DUP.
static bool can_emit_str_q(const Node& a, const Node& b, const Node& c, const Node& d) {
    auto sa = get_simple_mem(a);
    auto sb = get_simple_mem(b);
    auto sc = get_simple_mem(c);
    auto sd = get_simple_mem(d);
    if (!sa.ok || !sb.ok || !sc.ok || !sd.ok) {
        return false;
    }
    if (sa.base_reg != sb.base_reg || sa.base_reg != sc.base_reg || sa.base_reg != sd.base_reg) {
        return false;
    }
    if (sb.disp != sa.disp + 4 || sc.disp != sa.disp + 8 || sd.disp != sa.disp + 12) {
        return false;
    }
    return (sa.disp & 0xF) == 0;  // 16-byte alignment required by STR Q
}

// True iff two consecutive StoreF32 nodes target adjacent f32 slots off the
// same x86 base register — the precondition for `STP S, S`.
static bool can_emit_stp_s(const Node& a, const Node& b) {
    auto sa = get_simple_mem(a);
    auto sb = get_simple_mem(b);
    if (!sa.ok || !sb.ok) {
        return false;
    }
    if (sa.base_reg != sb.base_reg) {
        return false;
    }
    return sb.disp == sa.disp + 4;
}

// ── RC preamble: load control_word and extract rounding-control bits ────────

static void emit_rc_preamble(AssemblerBuffer& buf, int Xbase, int Wd_out) {
    // LDRH Wd_out, [Xbase, #0]  — control_word is at offset 0x00
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR*/ 1, /*imm12=*/0, Xbase, Wd_out);
    // UBFX Wd_out, Wd_out, #10, #2  — extract RC field (bits 11:10)
    emit_bitfield(buf, /*is_64bit=*/0, /*UBFM*/ 2, /*N*/ 0, /*immr*/ 10, /*imms*/ 11, Wd_out,
                  Wd_out);
}

// ── Cached RC dispatch for FISTP/FIST ──────────────────────────────────────
//
// Uses a pre-extracted RC value in Wd_rc_cached.  Copies to Wd_rc_scratch
// first because the CBZ/SUB chain is destructive.

static void emit_rcmode_dispatch_cached(AssemblerBuffer& buf, int Wd_int, int Dd_val,
                                        int is_64bit_int, int Wd_rc_cached, int Wd_rc_scratch) {
    // [0] MOV Wd_rc_scratch, Wd_rc_cached
    emit_mov_reg(buf, /*is_64bit=*/0, Wd_rc_scratch, Wd_rc_cached);
    // [1] CBZ Wd_rc_scratch, +7 → [8] FCVTNS
    emit_cbz(buf, 0, 0, Wd_rc_scratch, 7);
    // [2] SUB Wd_rc_scratch, Wd_rc_scratch, #1
    emit_add_imm(buf, 0, 1, 0, 0, 1, Wd_rc_scratch, Wd_rc_scratch);
    // [3] CBZ Wd_rc_scratch, +7 → [10] FCVTMS
    emit_cbz(buf, 0, 0, Wd_rc_scratch, 7);
    // [4] SUB
    emit_add_imm(buf, 0, 1, 0, 0, 1, Wd_rc_scratch, Wd_rc_scratch);
    // [5] CBZ Wd_rc_scratch, +7 → [12] FCVTPS
    emit_cbz(buf, 0, 0, Wd_rc_scratch, 7);
    // [6] FCVTZS (rmode=3)  RC=3 truncate
    emit_fcvt_fp_to_int(buf, is_64bit_int, 1, 3, Wd_int, Dd_val);
    // [7] B +6 → done
    emit_b(buf, 6);
    // [8] FCVTNS (rmode=0)  RC=0 nearest
    emit_fcvt_fp_to_int(buf, is_64bit_int, 1, 0, Wd_int, Dd_val);
    // [9] B +4 → done
    emit_b(buf, 4);
    // [10] FCVTMS (rmode=2)  RC=1 floor
    emit_fcvt_fp_to_int(buf, is_64bit_int, 1, 2, Wd_int, Dd_val);
    // [11] B +2 → done
    emit_b(buf, 2);
    // [12] FCVTPS (rmode=1)  RC=2 ceil
    emit_fcvt_fp_to_int(buf, is_64bit_int, 1, 1, Wd_int, Dd_val);
    // [13] done
}

// ── Cached RC dispatch for FRndInt ─────────────────────────────────────────

static void emit_frint_dispatch_cached(AssemblerBuffer& buf, int Dd, int Dn, int Wd_rc_cached,
                                       int Wd_rc_scratch) {
    emit_mov_reg(buf, 0, Wd_rc_scratch, Wd_rc_cached);
    emit_cbz(buf, 0, 0, Wd_rc_scratch, 7);
    emit_add_imm(buf, 0, 1, 0, 0, 1, Wd_rc_scratch, Wd_rc_scratch);
    emit_cbz(buf, 0, 0, Wd_rc_scratch, 7);
    emit_add_imm(buf, 0, 1, 0, 0, 1, Wd_rc_scratch, Wd_rc_scratch);
    emit_cbz(buf, 0, 0, Wd_rc_scratch, 7);
    // RC=3: FRINTZ (truncate) — fall-through
    emit_fp_dp1(buf, 1, /*opcode=*/11 /*FRINTZ*/, Dd, Dn);
    emit_b(buf, 6);
    // RC=0: FRINTN (nearest)
    emit_fp_dp1(buf, 1, /*opcode=*/8 /*FRINTN*/, Dd, Dn);
    emit_b(buf, 4);
    // RC=1: FRINTM (floor)
    emit_fp_dp1(buf, 1, /*opcode=*/10 /*FRINTM*/, Dd, Dn);
    emit_b(buf, 2);
    // RC=2: FRINTP (ceil)
    emit_fp_dp1(buf, 1, /*opcode=*/9 /*FRINTP*/, Dd, Dn);
}

// ── Rounding-mode dispatch for FISTP/FIST ──────────────────────────────────
//
// Emits the same 15-instruction CBZ/SUB chain as translate_fistp (or a single
// FCVTNS under fast_round).  See TranslatorX87.cpp for the detailed layout.

static void emit_rcmode_dispatch(AssemblerBuffer& buf, int Wd_int, int Dd_val, int is_64bit_int,
                                 int Xbase, int Wd_rc) {
    if (g_rosetta_config && g_rosetta_config->fast_round) {
        // Fast path: assume RC=0 (round-to-nearest).
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=FCVTNS*/ 0, Wd_int,
                            Dd_val);
    } else {
        // [0] LDRH Wd_rc, [Xbase, #0]  ; control_word
        emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*opc=*/1 /*LDR*/, /*imm12=*/0, Xbase,
                         Wd_rc);
        // [1] UBFX Wd_rc, Wd_rc, #10, #2
        emit_bitfield(buf, /*is_64bit=*/0, /*opc=*/2 /*UBFM*/, /*N*/ 0, /*immr*/ 10, /*imms*/ 11,
                      Wd_rc, Wd_rc);
        // [2] CBZ Wd_rc, +7 → [9] FCVTNS
        emit_cbz(buf, /*is_64bit=*/0, /*is_nz=*/0, Wd_rc, 7);
        // [3] SUB Wd_rc, Wd_rc, #1
        emit_add_imm(buf, /*is_64bit=*/0, /*is_sub*/ 1, /*S*/ 0, /*shift*/ 0, 1, Wd_rc, Wd_rc);
        // [4] CBZ Wd_rc, +7 → [11] FCVTMS
        emit_cbz(buf, /*is_64bit=*/0, /*is_nz=*/0, Wd_rc, 7);
        // [5] SUB Wd_rc, Wd_rc, #1
        emit_add_imm(buf, /*is_64bit=*/0, /*is_sub*/ 1, /*S*/ 0, /*shift*/ 0, 1, Wd_rc, Wd_rc);
        // [6] CBZ Wd_rc, +7 → [13] FCVTPS
        emit_cbz(buf, /*is_64bit=*/0, /*is_nz=*/0, Wd_rc, 7);
        // [7] FCVTZS (rmode=3)  RC=3 truncate
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=*/3, Wd_int, Dd_val);
        // [8] B +6 → done
        emit_b(buf, 6);
        // [9] FCVTNS (rmode=0)  RC=0 nearest
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=*/0, Wd_int, Dd_val);
        // [10] B +4 → done
        emit_b(buf, 4);
        // [11] FCVTMS (rmode=2)  RC=1 floor
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=*/2, Wd_int, Dd_val);
        // [12] B +2 → done
        emit_b(buf, 2);
        // [13] FCVTPS (rmode=1)  RC=2 ceil
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=*/1, Wd_int, Dd_val);
        // [14] done
    }
}

// ── Main lowering ───────────────────────────────────────────────────────────

void lower(Context& ctx, TranslationResult* result) {
    auto& buf = result->insn_buf;

    // ── Preamble: acquire base, TOP, and st_base ────────────────────────────
    auto [Xbase, Wd_top] = TranslatorX87::x87_begin(*result, buf);
    int Xst_base = TranslatorX87::x87_get_st_base(*result);
    int Wd_tmp = alloc_gpr(*result, 2);

    // Flush incoming deferred state.  The IR-gate's top_dirty branch
    // (Translator.cpp) flushes only top_dirty and falls through with
    // tag_push_pending / deferred_pop_count / perm_dirty potentially still
    // set; the deferred_pop branch can fall through with perm_dirty still
    // set.  The body's tag-update epilogue covers only its own net delta,
    // so without these the entering slot's tag/perm state never reaches
    // memory.  Mirror x87_push's incoming-flag handling
    // (TranslatorX87Internal.hpp:170-178).
    if (result->x87_cache.tag_push_pending) {
        const int Wd_tmp2 = alloc_free_gpr(*result);
        emit_x87_tag_clear(buf, Xbase, Wd_top, Wd_tmp, Wd_tmp2);
        free_gpr(*result, Wd_tmp2);
        result->x87_cache.tag_push_pending = 0;
    }
    TranslatorX87::x87_flush_deferred_pops(buf, *result, Xbase, Wd_top, Wd_tmp);
    TranslatorX87::perm_flush_before_stack_change(buf, *result, Xbase, Wd_top, Wd_tmp);

    // ── FPR assignment ──────────────────────────────────────────────────────
    FPRState fprs;
    fprs.compute_last_uses(ctx);

    // ── RC caching: hoist LDRH+UBFX when ≥2 RC consumers in a segment ────
    int Wd_rc_cached = -1;
    bool rc_cache_valid = false;

    if (!(g_rosetta_config && g_rosetta_config->fast_round)) {
        int rc_count = 0;
        bool has_fcmp = false;
        for (int i = 0; i < ctx.num_nodes; i++) {
            auto& n = ctx.nodes[i];
            if (n.flags & kDead) {
                continue;
            }
            if (n.op == Op::FCmp || n.op == Op::FTst) {
                has_fcmp = true;
                break;
            }
            if (n.op == Op::StoreCW) {
                continue;
            }
            bool is_rc = (n.op == Op::FRndInt) ||
                         ((n.op == Op::StoreI16 || n.op == Op::StoreI32 || n.op == Op::StoreI64) &&
                          !(n.flags & kTruncate));
            if (is_rc) {
                rc_count++;
            }
        }
        const bool use_rc_cache = !has_fcmp && rc_count >= 2;
        if (use_rc_cache) {
            Wd_rc_cached = alloc_gpr(*result, 3);
        }
    }

    // ── Emit each IR node ───────────────────────────────────────────────────
    for (int i = 0; i < ctx.num_nodes; i++) {
        auto& n = ctx.nodes[i];
        if (n.flags & kDead) {
            continue;
        }

        switch (n.op) {
            // ── Value nodes ─────────────────────────────────────────────────
            case Op::ReadSt: {
                int Dd = alloc_free_fpr(*result);
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_load_st(buf, Xbase, Wd_top, n.initial_depth, Wd_tmp, Dd, Xst_base);
                break;
            }
            case Op::LoadF64: {
                int Dd = alloc_free_fpr(*result);
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                int addr = compute_operand_address(*result, true, n.mem_operand, GPR::XZR);
                emit_fldr_imm(buf, 3, Dd, addr, 0);
                free_gpr(*result, addr);
                break;
            }
            case Op::LoadF32: {
                int Dd = alloc_free_fpr(*result);
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                int addr = compute_operand_address(*result, true, n.mem_operand, GPR::XZR);
                emit_fldr_imm(buf, 2, Dd, addr, 0);
                free_gpr(*result, addr);
                emit_fcvt_s_to_d(buf, Dd, Dd);
                break;
            }
            case Op::LoadI16: {
                int Dd = alloc_free_fpr(*result);
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                int addr = compute_operand_address(*result, true, n.mem_operand, GPR::XZR);
                int Wd_val = alloc_free_gpr(*result);
                // LDRSH Wd_val, [addr] — sign-extending load
                emit_ldrs(buf, /*is_64bit=*/0, /*size=S16*/ 1, Wd_val, addr);
                free_gpr(*result, addr);
                // SCVTF Dd, Wd_val
                emit_scvtf(buf, /*is_64bit_int=*/0, /*ftype=f64*/ 1, Dd, Wd_val);
                free_gpr(*result, Wd_val);
                break;
            }
            case Op::LoadI32: {
                int Dd = alloc_free_fpr(*result);
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                int addr = compute_operand_address(*result, true, n.mem_operand, GPR::XZR);
                int Wd_val = alloc_free_gpr(*result);
                emit_ldr_imm(buf, /*size=S32*/ 2, Wd_val, addr, 0);
                free_gpr(*result, addr);
                // SXTW + SCVTF: sign-extend 32→64 then convert
                emit_scvtf(buf, /*is_64bit_int=*/0, /*ftype=f64*/ 1, Dd, Wd_val);
                free_gpr(*result, Wd_val);
                break;
            }
            case Op::LoadI64: {
                int Dd = alloc_free_fpr(*result);
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                int addr = compute_operand_address(*result, true, n.mem_operand, GPR::XZR);
                int Xd_val = alloc_free_gpr(*result);
                emit_ldr_imm(buf, /*size=S64*/ 3, Xd_val, addr, 0);
                free_gpr(*result, addr);
                emit_scvtf_x_to_d(buf, Dd, Xd_val);
                free_gpr(*result, Xd_val);
                break;
            }
            case Op::ConstZero: {
                int Dd = alloc_free_fpr(*result);
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_movi_d_zero(buf, Dd);
                break;
            }
            case Op::ConstOne: {
                int Dd = alloc_free_fpr(*result);
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_fmov_d_one(buf, Dd);
                break;
            }
            case Op::ConstF64: {
                int Dd = alloc_free_fpr(*result);
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_ldr_literal_f64(buf, Dd, n.imm_bits);
                break;
            }

            // ── Binary arithmetic ───────────────────────────────────────────
            case Op::FAdd: {
                int Dn = fprs.get(n.inputs[0]);
                int Dm = fprs.get(n.inputs[1]);
                int Dd = fprs.try_reuse_input(ctx, i);
                if (Dd < 0) {
                    Dd = alloc_free_fpr(*result);
                }
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_fadd_f64(buf, Dd, Dn, Dm);
                break;
            }
            case Op::FSub: {
                int Dn = fprs.get(n.inputs[0]);
                int Dm = fprs.get(n.inputs[1]);
                int Dd = fprs.try_reuse_input(ctx, i);
                if (Dd < 0) {
                    Dd = alloc_free_fpr(*result);
                }
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_fsub_f64(buf, Dd, Dn, Dm);
                break;
            }
            case Op::FMul: {
                int Dn = fprs.get(n.inputs[0]);
                int Dm = fprs.get(n.inputs[1]);
                int Dd = fprs.try_reuse_input(ctx, i);
                if (Dd < 0) {
                    Dd = alloc_free_fpr(*result);
                }
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_fmul_f64(buf, Dd, Dn, Dm);
                break;
            }
            case Op::FDiv: {
                int Dn = fprs.get(n.inputs[0]);
                int Dm = fprs.get(n.inputs[1]);
                int Dd = fprs.try_reuse_input(ctx, i);
                if (Dd < 0) {
                    Dd = alloc_free_fpr(*result);
                }
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_fdiv_f64(buf, Dd, Dn, Dm);
                break;
            }

            // ── FMA ─────────────────────────────────────────────────────────
            case Op::FMAdd: {
                // FMADD Dd, Dn, Dm, Da → Da + Dn * Dm
                // inputs[0] = Dn, inputs[1] = Dm, inputs[2] = Da
                int Dn = fprs.get(n.inputs[0]);
                int Dm = fprs.get(n.inputs[1]);
                int Da = fprs.get(n.inputs[2]);
                int Dd = fprs.try_reuse_input(ctx, i);
                if (Dd < 0) {
                    Dd = alloc_free_fpr(*result);
                }
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_fmadd_f64(buf, Dd, Dn, Dm, Da);
                break;
            }
            case Op::FMSub: {
                // FMSUB Dd, Dn, Dm, Da → Da - Dn * Dm
                int Dn = fprs.get(n.inputs[0]);
                int Dm = fprs.get(n.inputs[1]);
                int Da = fprs.get(n.inputs[2]);
                int Dd = fprs.try_reuse_input(ctx, i);
                if (Dd < 0) {
                    Dd = alloc_free_fpr(*result);
                }
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_fmsub_f64(buf, Dd, Dn, Dm, Da);
                break;
            }
            case Op::FNMSub: {
                // FNMSUB Dd, Dn, Dm, Da → Dn * Dm - Da
                int Dn = fprs.get(n.inputs[0]);
                int Dm = fprs.get(n.inputs[1]);
                int Da = fprs.get(n.inputs[2]);
                int Dd = fprs.try_reuse_input(ctx, i);
                if (Dd < 0) {
                    Dd = alloc_free_fpr(*result);
                }
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_fnmsub_f64(buf, Dd, Dn, Dm, Da);
                break;
            }

            // ── Unary ───────────────────────────────────────────────────────
            case Op::FNeg: {
                int Dn = fprs.get(n.inputs[0]);
                int Dd = fprs.try_reuse_input(ctx, i);
                if (Dd < 0) {
                    Dd = alloc_free_fpr(*result);
                }
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_fneg_f64(buf, Dd, Dn);
                break;
            }
            case Op::FAbs: {
                int Dn = fprs.get(n.inputs[0]);
                int Dd = fprs.try_reuse_input(ctx, i);
                if (Dd < 0) {
                    Dd = alloc_free_fpr(*result);
                }
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_fabs_f64(buf, Dd, Dn);
                break;
            }
            case Op::FSqrt: {
                int Dn = fprs.get(n.inputs[0]);
                int Dd = fprs.try_reuse_input(ctx, i);
                if (Dd < 0) {
                    Dd = alloc_free_fpr(*result);
                }
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_fsqrt_f64(buf, Dd, Dn);
                break;
            }
            case Op::FRndInt: {
                int Dn = fprs.get(n.inputs[0]);
                int Dd = fprs.try_reuse_input(ctx, i);
                if (Dd < 0) {
                    Dd = alloc_free_fpr(*result);
                }
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);

                if (g_rosetta_config && g_rosetta_config->fast_round) {
                    // Fast path: RC=0 → FRINTN
                    emit_fp_dp1(buf, /*type=*/1, /*opcode=*/8 /*FRINTN*/, Dd, Dn);
                } else if (Wd_rc_cached >= 0) {
                    // RC caching: emit preamble on first use, then reuse cached RC.
                    if (!rc_cache_valid) {
                        emit_rc_preamble(buf, Xbase, Wd_rc_cached);
                        rc_cache_valid = true;
                    }
                    emit_frint_dispatch_cached(buf, Dd, Dn, Wd_rc_cached, Wd_tmp);
                } else {
                    int Wd_rc = alloc_gpr(*result, 3);
                    // LDRH Wd_rc, [Xbase, #0]  — read control_word
                    emit_ldr_str_imm(buf, 1, 0, 1, 0, Xbase, Wd_rc);
                    // UBFX Wd_rc, Wd_rc, #10, #2  — extract RC
                    emit_bitfield(buf, 0, 2, 0, 10, 11, Wd_rc, Wd_rc);
                    // CBZ/SUB dispatch chain
                    emit_cbz(buf, 0, 0, Wd_rc, 7);                   // RC==0 → FRINTN
                    emit_add_imm(buf, 0, 1, 0, 0, 1, Wd_rc, Wd_rc);  // SUB 1
                    emit_cbz(buf, 0, 0, Wd_rc, 7);                   // RC==1 → FRINTM
                    emit_add_imm(buf, 0, 1, 0, 0, 1, Wd_rc, Wd_rc);  // SUB 1
                    emit_cbz(buf, 0, 0, Wd_rc, 7);                   // RC==2 → FRINTP
                    // RC=3: FRINTZ (truncate) — fall-through
                    emit_fp_dp1(buf, 1, /*opcode=*/11 /*FRINTZ*/, Dd, Dn);
                    emit_b(buf, 6);
                    // RC=0: FRINTN (nearest)
                    emit_fp_dp1(buf, 1, /*opcode=*/8 /*FRINTN*/, Dd, Dn);
                    emit_b(buf, 4);
                    // RC=1: FRINTM (floor)
                    emit_fp_dp1(buf, 1, /*opcode=*/10 /*FRINTM*/, Dd, Dn);
                    emit_b(buf, 2);
                    // RC=2: FRINTP (ceil)
                    emit_fp_dp1(buf, 1, /*opcode=*/9 /*FRINTP*/, Dd, Dn);
                    free_gpr(*result, Wd_rc);
                }
                break;
            }

            // ── Transcendentals ─────────────────────────────────────────────
            // Inputs are FMOVed into d0 (and d1 for 2-input ops) before the
            // *_core helpers run.  d0/d1 are NOT in kFprScratchPool, so once
            // we release dying inputs the polynomial body has the full
            // 8-slot pool for internal scratch.  Result is FMOVed from d0
            // back into a freshly-allocated scratch FPR.
            //
            // The mid-case free_dead_inputs() lets pool slots that were
            // holding the ReadSt / Load / prior-arith outputs be reclaimed
            // for the core's transient FPR allocations.  See
            // peak_live_fprs() for the matching pressure model.
            case Op::FSin:
            case Op::FCos: {
                int Dn = fprs.get(n.inputs[0]);
                emit_fmov_f64(buf, /*Dd=*/0, Dn);
                fprs.free_dead_inputs(*result, n, i);
                int Xconst = alloc_free_gpr(*result);
                emit_movz_movk_abs64(buf, Xconst,
                                     rosetta_core::get_transcendental_constants_addr());
                TranslatorX87::emit_inline_trig_body(*result, buf, /*Dx=*/0, Xconst, /*Dd_out=*/0,
                                                     n.op == Op::FSin
                                                         ? TranslatorX87::TrigReduceMode::Sin
                                                         : TranslatorX87::TrigReduceMode::Cos);
                free_gpr(*result, Xconst);
                TranslatorX87::emit_clear_x87_cc_bits(*result, buf, Xbase);
                int Dd = alloc_free_fpr(*result);
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_fmov_f64(buf, Dd, /*Dn=*/0);
                break;
            }
            case Op::FPtan: {
                int Dn = fprs.get(n.inputs[0]);
                emit_fmov_f64(buf, /*Dd=*/0, Dn);
                fprs.free_dead_inputs(*result, n, i);
                int Xconst = alloc_free_gpr(*result);
                emit_movz_movk_abs64(buf, Xconst,
                                     rosetta_core::get_transcendental_constants_addr());
                TranslatorX87::emit_inline_fptan_core(*result, buf, /*Dx_in=*/0, /*Dd_out=*/0,
                                                      Xconst);
                free_gpr(*result, Xconst);
                TranslatorX87::emit_clear_x87_cc_bits(*result, buf, Xbase);
                int Dd = alloc_free_fpr(*result);
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_fmov_f64(buf, Dd, /*Dn=*/0);
                break;
            }
            case Op::FPatan: {
                int Dy = fprs.get(n.inputs[0]);
                int Dx = fprs.get(n.inputs[1]);
                emit_fmov_f64(buf, /*Dd=*/0, Dx);
                emit_fmov_f64(buf, /*Dd=*/1, Dy);
                fprs.free_dead_inputs(*result, n, i);
                int Xconst = alloc_free_gpr(*result);
                emit_movz_movk_abs64(buf, Xconst,
                                     rosetta_core::get_transcendental_constants_addr());
                TranslatorX87::emit_inline_fpatan_core(*result, buf, /*Dy_in=*/1, /*Dx_in=*/0,
                                                       /*Dd_out=*/0, Xconst);
                free_gpr(*result, Xconst);
                // fpatan's spec leaves C0..C3 undefined — no clear.
                int Dd = alloc_free_fpr(*result);
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_fmov_f64(buf, Dd, /*Dn=*/0);
                break;
            }
            case Op::FYl2x: {
                int Dy = fprs.get(n.inputs[0]);
                int Dx = fprs.get(n.inputs[1]);
                emit_fmov_f64(buf, /*Dd=*/0, Dx);
                emit_fmov_f64(buf, /*Dd=*/1, Dy);
                fprs.free_dead_inputs(*result, n, i);
                int Xconst = alloc_free_gpr(*result);
                emit_movz_movk_abs64(buf, Xconst,
                                     rosetta_core::get_transcendental_constants_addr());
                TranslatorX87::emit_inline_fyl2x_core(*result, buf, /*Dy_in=*/1, /*Dx_in=*/0,
                                                      /*Dd_out=*/0, Xconst);
                free_gpr(*result, Xconst);
                // fyl2x's spec leaves C0..C3 undefined — no clear.
                int Dd = alloc_free_fpr(*result);
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_fmov_f64(buf, Dd, /*Dn=*/0);
                break;
            }
            case Op::FScale: {
                int Dx = fprs.get(n.inputs[0]);
                int Dy = fprs.get(n.inputs[1]);
                emit_fmov_f64(buf, /*Dd=*/0, Dx);
                emit_fmov_f64(buf, /*Dd=*/1, Dy);
                fprs.free_dead_inputs(*result, n, i);
                TranslatorX87::emit_inline_fscale_core(*result, buf, /*Dx_in=*/0, /*Dy_in=*/1,
                                                       /*Dd_out=*/0);
                // fscale's spec leaves C0..C3 undefined — no clear.
                int Dd = alloc_free_fpr(*result);
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                emit_fmov_f64(buf, Dd, /*Dn=*/0);
                break;
            }

            // ── Conditional select (FCMOV) ────────────────────────────────
            case Op::FCSel: {
                int Dn = fprs.get(n.inputs[0]);  // ST(0) — false arm
                int Dm = fprs.get(n.inputs[1]);  // ST(i) — true arm
                int Dd = fprs.try_reuse_input(ctx, i);
                if (Dd < 0) {
                    Dd = alloc_free_fpr(*result);
                }
                fprs.node_fpr[i] = static_cast<int8_t>(Dd);
                int cond = static_cast<int>(n.imm_bits & 0xF);
                // FCSEL Dd, Dn_true, Dm_false, cond → Dd = cond ? Dn : Dm
                emit_fcsel_f64(buf, Dd, Dm, Dn, cond);
                break;
            }

            // ── Memory stores ───────────────────────────────────────────────
            case Op::StoreF64: {
                int addr = compute_operand_address(*result, true, n.mem_operand, GPR::XZR);
                emit_fstr_imm(buf, 3, fprs.get(n.inputs[0]), addr, 0);
                free_gpr(*result, addr);
                break;
            }
            case Op::StoreF32: {
                // Forward-scan: coalesce consecutive StoreF32 nodes that share
                // the same SSA input into one fcvt + a ladder of STR Q / STP S /
                // STR S groups.  This is the WoW L=12 fst-broadcast hot path —
                // see feedback_ir_pipeline_storef32_baseline.md.
                const int16_t src_node = n.inputs[0];
                int j = i;
                while (j + 1 < ctx.num_nodes) {
                    const auto& nx = ctx.nodes[j + 1];
                    if ((nx.flags & kDead) != 0) {
                        break;
                    }
                    if (nx.op != Op::StoreF32 || nx.inputs[0] != src_node) {
                        break;
                    }
                    ++j;
                }

                // One narrowing for the whole run; reused as the scalar source
                // for STP S / STR S, and as the source lane for the broadcast
                // DUP that feeds STR Q.
                const int Ds_narrow = alloc_free_fpr(*result);
                emit_fcvt_d_to_s(buf, Ds_narrow, fprs.get(src_node));

                // Allocate the broadcast Q register lazily — only if at least
                // one STR Q group fires.  Most benches that take the STP S or
                // scalar STR S path skip the DUP entirely.
                int Vq_broadcast = -1;
                auto ensure_broadcast = [&] {
                    if (Vq_broadcast < 0) {
                        Vq_broadcast = alloc_free_fpr(*result);
                        emit_dup_v4s_from_s(buf, Vq_broadcast, Ds_narrow);
                    }
                };

                int k = i;
                while (k <= j) {
                    // STR Q: 4-float aligned-contiguous group.
                    if (k + 3 <= j && can_emit_str_q(ctx.nodes[k], ctx.nodes[k + 1],
                                                     ctx.nodes[k + 2], ctx.nodes[k + 3])) {
                        ensure_broadcast();
                        const int Xaddr = compute_operand_address(
                            *result, /*is_64bit=*/true, ctx.nodes[k].mem_operand, GPR::XZR);
                        emit_str_q_imm(buf, Vq_broadcast, Xaddr, /*imm12=*/0);
                        free_gpr(*result, Xaddr);
                        k += 4;
                        continue;
                    }
                    // STP S, S: 2-float contiguous group.
                    if (k + 1 <= j && can_emit_stp_s(ctx.nodes[k], ctx.nodes[k + 1])) {
                        const int Xaddr = compute_operand_address(
                            *result, /*is_64bit=*/true, ctx.nodes[k].mem_operand, GPR::XZR);
                        emit_stp_s_imm(buf, Ds_narrow, Ds_narrow, Xaddr, /*simm7=*/0);
                        free_gpr(*result, Xaddr);
                        k += 2;
                        continue;
                    }
                    // STR S: scalar fallback (today's per-store shape).
                    const int Xaddr = compute_operand_address(*result, /*is_64bit=*/true,
                                                              ctx.nodes[k].mem_operand, GPR::XZR);
                    emit_fstr_imm(buf, /*size=*/2, Ds_narrow, Xaddr, /*imm12=*/0);
                    free_gpr(*result, Xaddr);
                    k += 1;
                }

                if (Vq_broadcast >= 0) {
                    free_fpr(*result, Vq_broadcast);
                }
                free_fpr(*result, Ds_narrow);

                // Skip the rest of the run; the outer loop's ++i lands on j+1.
                // free_dead_inputs runs on ctx.nodes[j], whose inputs[0] is
                // src_node — its FPR is freed there if last_use == j.
                i = j;
                break;
            }

            // ── Integer stores (FISTP/FIST/FISTTP) ──────────────────────────
            case Op::StoreI16:
            case Op::StoreI32:
            case Op::StoreI64: {
                int Dd_val = fprs.get(n.inputs[0]);
                int Wd_int = alloc_free_gpr(*result);
                int is_64bit_int = (n.op == Op::StoreI64) ? 1 : 0;
                int store_size = 3;
                if (n.op == Op::StoreI16) {
                    store_size = 1;
                } else if (n.op == Op::StoreI32) {
                    store_size = 2;
                }

                if (n.flags & kTruncate) {
                    // FISTTP: always truncate, single FCVTZS.
                    emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1,
                                        /*rmode=FCVTZS*/ 3, Wd_int, Dd_val);
                } else if (Wd_rc_cached >= 0) {
                    // RC caching: emit preamble on first use, then reuse cached RC.
                    if (!rc_cache_valid) {
                        emit_rc_preamble(buf, Xbase, Wd_rc_cached);
                        rc_cache_valid = true;
                    }
                    emit_rcmode_dispatch_cached(buf, Wd_int, Dd_val, is_64bit_int, Wd_rc_cached,
                                                Wd_tmp);
                } else {
                    // FISTP/FIST: respect rounding mode from control_word.
                    emit_rcmode_dispatch(buf, Wd_int, Dd_val, is_64bit_int, Xbase, Wd_tmp);
                }

                int addr = compute_operand_address(*result, true, n.mem_operand, GPR::XZR);
                emit_str_imm(buf, store_size, Wd_int, addr, /*imm12=*/0);
                free_gpr(*result, addr);
                free_gpr(*result, Wd_int);
                break;
            }

            // ── Compare ─────────────────────────────────────────────────────
            case Op::FCmp: {
                int Wd_save = alloc_free_gpr(*result);
                emit_mrs_nzcv(buf, Wd_save);
                emit_fcmp_f64(buf, fprs.get(n.inputs[0]), fprs.get(n.inputs[1]));

                int Wd_packed = alloc_free_gpr(*result);
                emit_fcom_cc_pack(buf, *result, Wd_packed, Wd_save);
                // emit_fcom_cc_pack restores NZCV and frees Wd_save internally.

                if (n.flags & kFcomFused) {
                    // Fused: keep packed CC alive for FStsw to consume.
                    // Store the GPR number in node_fpr[] (repurposed for GPR tracking).
                    fprs.node_fpr[i] = static_cast<int8_t>(Wd_packed);
                } else {
                    // Non-fused: write CC to status_word now.
                    emit_fcom_cc_write_sw(buf, *result, Xbase, Wd_packed);
                    free_gpr(*result, Wd_packed);
                }
                break;
            }
            case Op::FTst: {
                int Wd_save = alloc_free_gpr(*result);
                emit_mrs_nzcv(buf, Wd_save);
                emit_fcmp_zero_f64(buf, fprs.get(n.inputs[0]));

                int Wd_packed = alloc_free_gpr(*result);
                emit_fcom_cc_pack(buf, *result, Wd_packed, Wd_save);

                if (n.flags & kFcomFused) {
                    fprs.node_fpr[i] = static_cast<int8_t>(Wd_packed);
                } else {
                    emit_fcom_cc_write_sw(buf, *result, Xbase, Wd_packed);
                    free_gpr(*result, Wd_packed);
                }
                break;
            }

            // ── FCOMI / FCOMIP / FUCOMI / FUCOMIP ───────────────────────────
            case Op::FComI: {
                // Direct port of translate_fcomi: FCMP two FPRs, then pack and
                // write x86-compatible flags into NZCV. Does NOT touch status_word.
                int Dd_st0 = fprs.get(n.inputs[0]);
                int Dd_src = fprs.get(n.inputs[1]);

                emit_fcmp_f64(buf, Dd_st0, Dd_src);

                // Extract condition bits before any MSR clobbers NZCV.
                int Wd_z = alloc_free_gpr(*result);
                int Wd_v = alloc_free_gpr(*result);
                int Wd_c = alloc_free_gpr(*result);

                emit_cset(buf, /*is_64bit=*/0, /*cond=*/0 /*EQ*/, Wd_z);  // 1 if equal
                emit_cset(buf, /*is_64bit=*/0, /*cond=*/6 /*VS*/, Wd_v);  // 1 if unordered
                emit_cset(buf, /*is_64bit=*/0, /*cond=*/2 /*CS*/, Wd_c);  // 1 if carry set

                // Z_new = Z | V  (equal or unordered → ZF)
                emit_logical_shifted_reg(buf, 0, /*ORR*/ 1, 0, /*LSL*/ 0, Wd_v, 0, Wd_z, Wd_z);
                // C_new = C & !V  (carry clear for unordered → CF)
                emit_logical_shifted_reg(buf, 0, /*AND*/ 0, /*N=invert rhs*/ 1, /*LSL*/ 0, Wd_v, 0,
                                         Wd_c, Wd_c);

                // Pack NZCV: bit30=ZF, bit29=CF, bit28=V(PF for FCMOV), bit26=PF
                emit_bitfield(buf, /*is_64bit=*/0, /*opc=*/2 /*UBFM*/, /*N=*/0,
                              /*immr=*/2, /*imms=*/1, Wd_z, Wd_z);
                emit_logical_shifted_reg(buf, 0, /*ORR*/ 1, 0, /*LSL*/ 0, Wd_c, 29, Wd_z, Wd_z);
                emit_logical_shifted_reg(buf, 0, /*ORR*/ 1, 0, /*LSL*/ 0, Wd_v, 28, Wd_z, Wd_z);
                emit_logical_shifted_reg(buf, 0, /*ORR*/ 1, 0, /*LSL*/ 0, Wd_v, 26, Wd_z, Wd_z);

                emit_msr_nzcv(buf, Wd_z);

                free_gpr(*result, Wd_c);
                free_gpr(*result, Wd_v);
                free_gpr(*result, Wd_z);
                // Pop (for FCOMIP/FUCOMIP) is handled by the IR epilogue via top_delta.
                break;
            }

            // ── Control word ────────────────────────────────────────────────
            case Op::StoreCW: {
                // FLDCW: load u16 from memory, write to X87State.control_word.
                int addr = compute_operand_address(*result, true, n.mem_operand, GPR::XZR);
                int Wd_cw = alloc_free_gpr(*result);
                emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR*/ 1, /*imm12=*/0, addr, Wd_cw);
                free_gpr(*result, addr);
                // STRH Wd_cw, [Xbase, #0]  — control_word is at offset 0x00 → imm12=0
                emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR*/ 0, /*imm12=*/0, Xbase,
                                 Wd_cw);
                // Re-cache RC from the just-written control word.
                if (Wd_rc_cached >= 0) {
                    // UBFX Wd_rc_cached, Wd_cw, #10, #2
                    emit_bitfield(buf, 0, 2, 0, 10, 11, Wd_cw, Wd_rc_cached);
                    rc_cache_valid = true;
                } else {
                    rc_cache_valid = false;
                }
                free_gpr(*result, Wd_cw);
                break;
            }
            case Op::LoadCW: {
                // FNSTCW: read X87State.control_word, store u16 to memory.
                int Wd_cw = alloc_free_gpr(*result);
                // LDRH Wd_cw, [Xbase, #0]  — control_word is at offset 0x00 → imm12=0
                emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR*/ 1, /*imm12=*/0, Xbase,
                                 Wd_cw);
                int addr = compute_operand_address(*result, true, n.mem_operand, GPR::XZR);
                emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR*/ 0, /*imm12=*/0, addr, Wd_cw);
                free_gpr(*result, addr);
                free_gpr(*result, Wd_cw);
                break;
            }

            // ── FSTSW AX ───────────────────────────────────────────────────
            case Op::FStsw: {
                static constexpr int16_t kSwImm12 = kX87StatusWordOff / 2;  // = 1

                if (n.flags & kFcomFused) {
                    // Fused: retrieve packed CC from the FCmp node.
                    int16_t fcmp_id = n.inputs[0];
                    int Wd_packed = fprs.get(fcmp_id);

                    // RMW status_word with packed CC bits.
                    emit_fcom_cc_write_sw(buf, *result, Xbase, Wd_packed);
                    free_gpr(*result, Wd_packed);
                    fprs.node_fpr[fcmp_id] = -1;
                }
                // Both paths: load status_word (which now has correct CC), BFI into AX.
                {
                    int Wd_sw = alloc_free_gpr(*result);
                    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR*/ 1, kSwImm12, Xbase,
                                     Wd_sw);

                    // Patch TOP if pops occurred before this FSTSW.
                    int16_t td = n.inputs[2];  // top_delta snapshot
                    if (td != 0) {
                        int Wd_adj = alloc_free_gpr(*result);
                        if (td < 0) {
                            emit_add_imm(buf, 0, /*is_sub=*/1, 0, 0, -td, Wd_top, Wd_adj);
                        } else {
                            emit_add_imm(buf, 0, /*is_sub=*/0, 0, 0, td, Wd_top, Wd_adj);
                        }
                        emit_and_imm(buf, 0, Wd_adj, 0, 0, 2, Wd_adj);
                        // BFI Wd_sw, Wd_adj, #11, #3 — patch TOP field
                        emit_bitfield(buf, 0, /*BFM*/ 1, 0, /*immr=*/21, /*imms=*/2, Wd_adj, Wd_sw);
                        free_gpr(*result, Wd_adj);
                    }

                    // BFI W_ax, Wd_sw, #0, #16 — write status_word into x86 AX
                    int W_ax = n.inputs[1];  // destination register index (usually 0 = W0)
                    emit_bitfield(buf, 0, /*BFM*/ 1, 0, /*immr=*/0, /*imms=*/15, Wd_sw, W_ax);
                    free_gpr(*result, Wd_sw);
                }
                break;
            }

        }  // switch

        // Free FPRs whose last use was this node.
        fprs.free_dead_inputs(*result, n, i);
    }

    // ── Epilogue: update x87 state ──────────────────────────────────────────

    // 1. Update TOP register.
    if (ctx.top_delta != 0) {
        if (ctx.top_delta < 0) {
            emit_add_imm(buf, 0, /*is_sub=*/1, 0, 0, -ctx.top_delta, Wd_top, Wd_top);
        } else {
            emit_add_imm(buf, 0, /*is_sub=*/0, 0, 0, ctx.top_delta, Wd_top, Wd_top);
        }
        emit_and_imm(buf, 0, Wd_top, 0, 0, 2, Wd_top);
    }

    // 2. Store modified stack values.
    // At this point, Wd_top holds the FINAL top. slot_val[d] tells us what value
    // should be at logical depth d relative to the final TOP.
    for (int d = 0; d < 8; d++) {
        int16_t val = ctx.slot_val[d];
        if (val < 0) {
            continue;  // initial slot, unchanged (no store needed)
        }
        // Skip redundant write-back: if the value is a ReadSt loaded from the
        // same physical slot it would be stored to, the store is a no-op.
        if (ctx.nodes[val].op == Op::ReadSt && ctx.nodes[val].initial_depth == d + ctx.top_delta) {
            continue;
        }
        int Dd = fprs.get(val);
        if (Dd < 0) {
            continue;  // dead or already freed
        }
        emit_store_st(buf, Xbase, Wd_top, d, Wd_tmp, Dd, Xst_base);
    }

    // 3. Write TOP to status_word (if changed).
    if (ctx.top_delta != 0) {
        emit_store_top(buf, Xbase, Wd_top, Wd_tmp);
    }

    // 4. Update tag word for net pushes/pops.
    if (ctx.top_delta != 0) {
        // Free Xst_base before allocating Wd_tmp2/Wd_tagw — the tag-batch
        // helpers don't use Xst_base, and freeing here drops peak GPR by 1
        // (so the cohort `fld|fmul|...|faddp` that previously hit
        // peak=6 > 5-slot pool fits at peak=5).  If the run is a partial
        // consume, subsequent peep+single ops fall back to the uncached
        // emit_load_st/emit_store_st branch (~+1-3 ARM per ST access).
        // peak_live_gprs mirrors this slim in its epilogue model.
        if (Xst_base >= 0 && result->x87_cache.st_base_valid) {
            free_gpr(*result, Xst_base);
            result->x87_cache.st_base_valid = 0;
            Xst_base = -1;
        }
        int Wd_tmp2 = alloc_free_gpr(*result);
        if (ctx.top_delta > 0) {
            // Net pops: use the batch helper.
            int Wd_tagw = alloc_free_gpr(*result);
            emit_x87_tag_set_empty_batch(buf, Xbase, Wd_top, Wd_tmp, Wd_tmp2, Wd_tagw,
                                         ctx.top_delta);
            free_gpr(*result, Wd_tagw);
        } else {
            // Net pushes: clear tag bits for new slots.
            int abs_delta = -ctx.top_delta;
            int Wd_tagw = alloc_free_gpr(*result);
            emit_x87_tag_set_valid_batch(buf, Xbase, Wd_top, Wd_tmp, Wd_tmp2, Wd_tagw, abs_delta);
            free_gpr(*result, Wd_tagw);
        }
        free_gpr(*result, Wd_tmp2);
    }

    // 5. Free all remaining FPRs held by node values.
    for (int i = 0; i < ctx.num_nodes; i++) {
        if (fprs.node_fpr[i] >= 0) {
            free_fpr(*result, fprs.node_fpr[i]);
            fprs.node_fpr[i] = -1;
        }
    }

    // 6. Clean up cache deferred state (we handled everything inline).
    result->x87_cache.top_dirty = 0;
    result->x87_cache.tag_push_pending = 0;
    result->x87_cache.deferred_pop_count = 0;
    result->x87_cache.reset_perm();

    // 7. Free scratch GPRs.
    if (Wd_rc_cached >= 0) {
        free_gpr(*result, Wd_rc_cached);
    }
    free_gpr(*result, Wd_tmp);

    // 8. If cache is about to expire (run_remaining will hit 0 after ticks),
    //    free the cache GPRs. Otherwise, leave them pinned for the next run.
    if (result->x87_cache.run_remaining <= ctx.consumed) {
        // Cache will be deactivated by tick(). Release GPRs now so they don't
        // stay allocated past the run.
        // (tick() resets gprs_valid=0 but doesn't free the mask bits.)
        // The caller (Translator.cpp) resets free_gpr_mask after ticking.
    }
}

// ── Peak GPR pressure query ──────────────────────────────────────────────────
//
// Returns the maximum number of scratch GPRs simultaneously in use during
// lowering (permanent + transient).  If this exceeds the available pool the
// caller should bail out to the ordinary per-instruction translation path.
//
// Permanent GPRs (held for the entire lower() duration):
//   - Xbase (pool 0), Wd_top (pool 1), Wd_tmp (pool 2), Xst_base (pool 6) = 4
//   - Wd_rc_cached (pool 3) when RC caching is active = +1
//
// Per-node transient GPR demand (mirrors the lowering code exactly):
//   - ReadSt, Const*, FAdd/FSub/FMul/FDiv, FMA*, FNeg/FAbs/FSqrt, FCSel: 0
//   - LoadF64, LoadF32, StoreF64, StoreF32: 1 (addr from compute_operand_address)
//   - LoadI16/I32/I64: 2 (addr + Wd_val)
//   - StoreI16/I32/I64: 2 (Wd_int + addr)
//   - FRndInt without RC cache: 1 (Wd_rc via alloc_gpr(3))
//   - FCmp/FTst: 4 peak inside emit_fcom_cc_pack (Wd_save + Wd_packed + Wd_cc + Wd_vs)
//     If kFcomFused, Wd_packed stays alive until consumed by FStsw.
//   - FComI: 3 (Wd_z + Wd_v + Wd_c)
//   - FStsw (fused): 2 (Wd_packed held from FCmp + Wd_sw_inner inside emit_fcom_cc_write_sw,
//     or Wd_sw + Wd_adj when top_delta != 0)
//   - FStsw (non-fused): 2 (Wd_sw + Wd_adj if top_delta != 0)
//   - StoreCW/LoadCW: 2 (addr + Wd_cw)
//   - Epilogue (top_delta != 0): 2 (Wd_tmp2 + Wd_tagw)
//
// Fused FCmp/FTst holds 1 GPR (Wd_packed) alive across nodes until FStsw.
// We track this as a "held" count that overlaps with per-node transient demand.
// The held delta is applied AFTER each node's peak — Wd_packed is included
// in FCmp's transient=4 during the FCmp's own emit, so incrementing held
// before node_total would double-count it (predicting peak=9 instead of 8
// for the common single-fused-FCmp+FStsw pair, gating off real WoW blocks
// that fit in the 8-slot pool).
int peak_live_gprs(const Context& ctx) {
    // Determine if RC caching will be active (same logic as lower()).
    // RC cache is disabled when the run contains FCmp/FTst: emit_fcom_cc_pack
    // has a structurally-unavoidable 4-wide GPR peak (Wd_save + Wd_packed +
    // Wd_cc + Wd_vs) which combined with rc_cache's pinned Wd_rc_cached
    // pushes total demand to 9 vs. the 8-slot scratch pool.  Letting the
    // FCmp run lower successfully saves far more (long compare-heavy blocks
    // see arm_no_ir 590 → arm_ir_forced 305, ~285 ARM/exec) than rc_cache's
    // per-RC-op micro-saving (~2 ARM × few ops).
    bool rc_cache = false;
    if (!(g_rosetta_config && g_rosetta_config->fast_round)) {
        int rc_count = 0;
        bool has_fcmp = false;
        for (int i = 0; i < ctx.num_nodes; i++) {
            const auto& n = ctx.nodes[i];
            if (n.flags & kDead) {
                continue;
            }
            if (n.op == Op::FCmp || n.op == Op::FTst) {
                has_fcmp = true;
                break;
            }
            if (n.op == Op::StoreCW) {
                continue;
            }
            bool is_rc = (n.op == Op::FRndInt) ||
                         ((n.op == Op::StoreI16 || n.op == Op::StoreI32 || n.op == Op::StoreI64) &&
                          !(n.flags & kTruncate));
            if (is_rc) {
                rc_count++;
            }
        }
        rc_cache = !has_fcmp && rc_count >= 2;
    }

    int pinned = 4;  // Xbase, Wd_top, Wd_tmp, Xst_base
    if (rc_cache) {
        pinned++;  // Wd_rc_cached
    }

    // Simulate GPR pressure across IR nodes.
    // "held" tracks GPRs held alive across node boundaries (fused FCmp→FStsw).
    int held = 0;
    int peak = pinned;  // at minimum, permanent GPRs

    for (int i = 0; i < ctx.num_nodes; i++) {
        const auto& n = ctx.nodes[i];
        if (n.flags & kDead) {
            continue;
        }

        int transient = 0;  // per-node transient GPR demand

        switch (n.op) {
            // No transient GPRs
            case Op::ReadSt:
            case Op::ConstZero:
            case Op::ConstOne:
            case Op::ConstF64:
            case Op::FAdd:
            case Op::FSub:
            case Op::FMul:
            case Op::FDiv:
            case Op::FMAdd:
            case Op::FMSub:
            case Op::FNMSub:
            case Op::FNeg:
            case Op::FAbs:
            case Op::FSqrt:
            case Op::FCSel:
                break;

            // Transcendentals — peak GPR demand inside emit_inline_<op>_core:
            //   sin/cos: Xconst held + ~2 internal in trig_range_reduce/poly = 3
            //   fptan:   Xconst + Xqi held + ~1 brief = 3
            //   fpatan:  Xconst + Xsign_xy held + ~1 brief = 3
            //   fyl2x:   Xconst + ~3 inside log2 (Xu, Xu_off, Xidx peak) = 4
            //   fscale:  Wd_k + Wd_e + brief Xtemp during overflow CSEL = 3
            // Conservative bound across the family.
            case Op::FSin:
            case Op::FCos:
            case Op::FPtan:
            case Op::FPatan:
            case Op::FScale:
                transient = 3;
                break;
            case Op::FYl2x:
                transient = 4;
                break;

            // 1 GPR: addr from compute_operand_address
            case Op::LoadF64:
            case Op::LoadF32:
            case Op::StoreF64:
            case Op::StoreF32:
                transient = 1;
                break;

            // 2 GPRs: Loads need addr + Wd_val; stores need Wd_int + addr.
            case Op::LoadI16:
            case Op::LoadI32:
            case Op::LoadI64:
            case Op::StoreI16:
            case Op::StoreI32:
            case Op::StoreI64:
                transient = 2;
                break;

            // FRndInt: 1 GPR if no RC cache (alloc_gpr(3) for Wd_rc), 0 with cache
            case Op::FRndInt:
                if (!rc_cache) {
                    transient = 1;
                }
                break;

            // FCmp/FTst: 4 peak inside emit_fcom_cc_pack
            // (Wd_save + Wd_packed + Wd_cc + Wd_vs simultaneously live).
            // Wd_packed is one of those 4; if fused, it remains held after
            // the node, so `held` is incremented AFTER node_total below —
            // incrementing here would double-count Wd_packed.
            case Op::FCmp:
            case Op::FTst:
                transient = 4;
                break;

            // FComI: 3 GPRs (Wd_z + Wd_v + Wd_c)
            case Op::FComI:
                transient = 3;
                break;

            // FStsw: fused path holds Wd_packed (counted in held) + up to 2 more
            // Non-fused: up to 2 (Wd_sw + Wd_adj)
            case Op::FStsw:
                // emit_fcom_cc_write_sw adds 1 internally (Wd_sw), then the LDRH
                // block adds Wd_sw + possibly Wd_adj.
                // Fused: Wd_packed(held) + max(Wd_sw_inner=1, Wd_sw+Wd_adj=2) = 2
                // Non-fused: max(Wd_sw+Wd_adj) = 2
                transient = 2;
                if (n.flags & kFcomFused) {
                    // Wd_packed is released during this node
                    held--;
                }
                break;

            // StoreCW/LoadCW: 2 GPRs (addr + Wd_cw)
            case Op::StoreCW:
            case Op::LoadCW:
                transient = 2;
                break;
        }

        int node_total = pinned + held + transient;
        peak = std::max(node_total, peak);

        // FCmp/FTst fused: Wd_packed becomes held AFTER the node — counted
        // here so that subsequent nodes see it in `held` but the FCmp's own
        // node_total above isn't double-counted.  FStsw's held-- happens
        // BEFORE its node_total because Wd_packed is consumed inside FStsw
        // and the transient=2 accounting already includes it.
        if ((n.op == Op::FCmp || n.op == Op::FTst) && (n.flags & kFcomFused)) {
            held++;
        }
    }

    // Epilogue: if top_delta != 0, needs 2 more transient GPRs (Wd_tmp2 + Wd_tagw).
    // lower() releases Xst_base before allocating those two — the tag-batch
    // helpers don't use it — so the actual peak at the epilogue is
    // (pinned - 1) + held + 2.  Mirror that here so the gate predicts peak
    // accurately and runs that previously hit peak=6 fit at peak=5.
    if (ctx.top_delta != 0) {
        int epilogue_total = (pinned - 1) + held + 2;
        peak = std::max(epilogue_total, peak);
    }

    return peak;
}

// ── Peak FPR pressure query ──────────────────────────────────────────────────
//
// Simulates the liveness model used by the lowering pass and returns the
// maximum number of scratch FPRs simultaneously in use at any point.
//
// Rules (mirroring the lowering pass exactly):
//   - Value-producing nodes (ReadSt, Load*, Const*, FAdd, …) hold one FPR from
//     the point they are emitted until their last use is emitted.
//   - free_dead_inputs() fires at the end of each node, so the transient peak
//     *during* a node is: (currently live) + 1 for the new output, before dead
//     inputs are freed.  try_reuse_input() can avoid the +1 by recycling a
//     dying input's FPR, but we conservatively ignore reuse here.
//   - StoreF32 allocates one extra transient FPR (Ds_tmp) that is freed before
//     the node ends.  Model as a +1 spike.
//   - StoreI*, FCmp, FTst, FStsw produce no FPR output and need no extra FPRs.
//   - Dead (kDead) nodes are skipped.
int peak_live_fprs(const Context& ctx) {
    // Step 1: compute last_use[] — same as FPRState::compute_last_uses.
    int16_t last_use[kMaxNodes];
    memset(last_use, -1, sizeof(last_use));
    for (int i = 0; i < ctx.num_nodes; i++) {
        const auto& n = ctx.nodes[i];
        if (n.flags & kDead) {
            continue;
        }
        for (short input : n.inputs) {
            if (input >= 0) {
                last_use[input] = static_cast<int16_t>(i);
            }
        }
    }
    for (short val : ctx.slot_val) {
        if (val >= 0 && val < ctx.num_nodes) {
            last_use[val] = static_cast<int16_t>(ctx.num_nodes);
        }
    }

    // Step 2: simulate FPR allocation order, tracking live count.
    // A node holds an FPR from instruction i (inclusive) to last_use[i]
    // (exclusive — freed at the end of the last-use instruction).
    // live[i] = number of nodes alive at the *start* of instruction i.
    int live = 0;
    int peak = 0;

    // Track which nodes are currently holding an FPR (bit vector over kMaxNodes).
    bool holding[kMaxNodes] = {};

    for (int i = 0; i < ctx.num_nodes; i++) {
        const auto& n = ctx.nodes[i];
        if (n.flags & kDead) {
            continue;
        }

        // Allocate FPR for nodes that produce an FPR-bearing value.
        bool produces_fpr = false;
        switch (n.op) {
            case Op::ReadSt:
            case Op::LoadF64:
            case Op::LoadF32:
            case Op::LoadI16:
            case Op::LoadI32:
            case Op::LoadI64:
            case Op::ConstZero:
            case Op::ConstOne:
            case Op::ConstF64:
            case Op::FAdd:
            case Op::FSub:
            case Op::FMul:
            case Op::FDiv:
            case Op::FMAdd:
            case Op::FMSub:
            case Op::FNMSub:
            case Op::FNeg:
            case Op::FAbs:
            case Op::FSqrt:
            case Op::FRndInt:
            case Op::FCSel:
            case Op::FSin:
            case Op::FCos:
            case Op::FPtan:
            case Op::FPatan:
            case Op::FYl2x:
            case Op::FScale:
                produces_fpr = true;
                break;
            default:
                break;
        }

        if (produces_fpr) {
            live++;
            holding[i] = true;
            peak = std::max(live, peak);
        }

        // StoreF32 allocates a transient Ds_tmp (fcvt d→s narrowing) that is
        // freed before the node finishes.  Model as a +1 spike on top of the
        // current live count.
        if (n.op == Op::StoreF32 && live + 1 > peak) {
            peak = live + 1;
        }

        // Transcendentals: the lowering FMOVs inputs to d0/d1 (not in pool),
        // calls free_dead_inputs() mid-case, then runs the polynomial body
        // which allocates many transient FPRs from the pool.  Model the
        // peak as (live - dying_inputs + transient_spike), where dying
        // inputs have already been released by the time the spike happens.
        // Output is allocated AFTER the core, so it's not part of the spike.
        //
        // Peak transient FPRs inside each *_core (re-verified by inspection,
        // 2026-05-03 — earlier estimates were too low and the gate let real
        // WoW blocks through that overflowed the 8-slot pool):
        //   trig_body (sin/cos): range_reduce holds Dn + Dr + Dtmp = 3,
        //     then frees Dn/Dtmp; sin_poly_estrin then allocs Dr2 + Dr4 +
        //     Dp2 + Dp3 + Dtmp = 5 internal while Dr + Dy_out (held by
        //     trig_body) are still live → peak 7
        //   fptan:   step 6 inner peaks at Dr + Dr2 + Dr4 + Dr8 + Dp_acc +
        //     Dtmp + Dp_b + Dp_c = 8 simultaneously
        //   fpatan:  step 7 peaks at Dz + Dshift + Dz2 + Dz3 + Dpoly +
        //     Dtmp = 6
        //   fyl2x:   log2 step 8 polynomial holds Dr + Dr2 + Dhi + Dy + Dp +
        //     Dtmp + Dlog2c (still live) = 7 simultaneous; the +Dy_in fmul
        //     at end is after log2's scratch is freed
        //   fscale:  Dd_m + Dd_norm = 2
        int trans_spike = 0;
        switch (n.op) {
            case Op::FSin:
            case Op::FCos:
                trans_spike = 7;
                break;
            case Op::FPtan:
                trans_spike = 8;
                break;
            case Op::FPatan:
                trans_spike = 6;
                break;
            case Op::FYl2x:
                trans_spike = 7;
                break;
            case Op::FScale:
                trans_spike = 2;
                break;
            default:
                break;
        }
        if (trans_spike > 0) {
            int dying_inputs = 0;
            for (short in : n.inputs) {
                if (in >= 0 && last_use[in] == i && holding[in]) {
                    dying_inputs++;
                }
            }
            // The output (just allocated above via produces_fpr=true) is in d0
            // during the spike — it's not yet in the scratch pool, so don't
            // count it.  Subtract 1 from `live` to compensate.
            const int spike = (live - 1) - dying_inputs + trans_spike;
            peak = std::max(spike, peak);
        }

        // Free inputs whose last use is this node.
        for (short in : n.inputs) {
            if (in >= 0 && last_use[in] == i && holding[in]) {
                holding[in] = false;
                live--;
            }
        }
    }

    return peak;
}

// ── Entry point ─────────────────────────────────────────────────────────────

int compile_run(TranslationResult* result, IRInstr* instr_array, int64_t num_instrs,
                int64_t start_idx, int run_length, IRFailReason* out_reason, int* out_peak_gprs,
                uint16_t* out_fail_opcode) {
    Context ctx;

    const bool built = build(ctx, instr_array, num_instrs, start_idx, run_length);

    // Surface the bail opcode whenever build observed an unsupported one,
    // including the success-with-early-stop case (consumed >= 2 but the loop
    // halted at position N).
    if (out_fail_opcode && ctx.fail_opcode != 0xFFFFU) {
        *out_fail_opcode = ctx.fail_opcode;
    }

    if (!built) {
        if (out_reason) {
            *out_reason = IRFailReason::kBuildFail;
        }
        if (out_peak_gprs) {
            *out_peak_gprs = 0;
        }
        return 0;
    }

    optimize(ctx);

    // Gate lowering on actual FPR pressure vs. available pool.
    uint32_t fpr_pool = result->free_fpr_mask;
    int available = 0;
    while (fpr_pool) {
        available++;
        fpr_pool &= fpr_pool - 1;
    }
    if (peak_live_fprs(ctx) > available) {
        if (out_reason) {
            *out_reason = IRFailReason::kFprPressure;
        }
        if (out_peak_gprs) {
            *out_peak_gprs = peak_live_gprs(ctx);
        }
        return 0;
    }

    // Gate lowering on GPR pressure vs. available pool.
    const int peak_gprs = peak_live_gprs(ctx);
    if (out_peak_gprs) {
        *out_peak_gprs = peak_gprs;
    }
    {
        uint32_t gpr_pool = result->free_gpr_mask;
        int gpr_available = 0;
        while (gpr_pool) {
            gpr_available++;
            gpr_pool &= gpr_pool - 1;
        }
        if (peak_gprs > gpr_available) {
            if (out_reason) {
                *out_reason = IRFailReason::kGprPressure;
            }
            return 0;
        }
    }

    lower(ctx, result);

    return ctx.consumed;
}

}  // namespace X87IR
