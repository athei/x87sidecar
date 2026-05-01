#include "rosetta_core/X87IR.h"

#include <algorithm>
#include <cstring>
#include <utility>

#include "rosetta_core/AssemblerHelpers.hpp"
#include "rosetta_core/IROperand.h"
#include "rosetta_core/Register.h"
#include "rosetta_core/TranslationResult.h"
#include "rosetta_core/TranslatorHelpers.hpp"
#include "rosetta_core/TranslatorX87Helpers.hpp"
#include "rosetta_config/Config.h"
#include "rosetta_core/CoreConfig.h"
#include "rosetta_core/X87Cache.h"

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
    }
    return {Xbase, Wd_top};
}
inline int x87_get_st_base(TranslationResult& a1) {
    return a1.x87_cache.gprs_valid ? a1.x87_cache.st_base_gpr : -1;
}
}  // namespace TranslatorX87

namespace X87IR {

static constexpr int16_t kX87TagWordImm12 = kX87TagWordOff / 2;  // = 2

// ── FPR assignment ──────────────────────────────────────────────────────────

struct FPRState {
    int8_t node_fpr[kMaxNodes];      // D register for each node, or -1
    int16_t last_use[kMaxNodes];     // last node index that uses each node, or -1

    void compute_last_uses(const Context& ctx) {
        memset(last_use, -1, sizeof(last_use));
        memset(node_fpr, -1, sizeof(node_fpr));
        for (int i = 0; i < ctx.num_nodes; i++) {
            const auto& n = ctx.nodes[i];
            if (n.flags & kDead) { continue;
}
            for (short input : n.inputs) {
                if (input >= 0) { last_use[input] = static_cast<int16_t>(i);
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
        if (node_id < 0 || node_id >= kMaxNodes) { return -1;
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
                int fpr = node_fpr[in];
                node_fpr[in] = -1;  // claimed — free_dead_inputs will skip
                return fpr;
            }
        }
        return -1;
    }
};

// ── RC preamble: load control_word and extract rounding-control bits ────────

static void emit_rc_preamble(AssemblerBuffer& buf, int Xbase, int Wd_out) {
    // LDRH Wd_out, [Xbase, #0]  — control_word is at offset 0x00
    emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR*/1, /*imm12=*/0, Xbase, Wd_out);
    // UBFX Wd_out, Wd_out, #10, #2  — extract RC field (bits 11:10)
    emit_bitfield(buf, /*is_64bit=*/0, /*UBFM*/2, /*N*/0, /*immr*/10, /*imms*/11, Wd_out, Wd_out);
}

// ── Cached RC dispatch for FISTP/FIST ──────────────────────────────────────
//
// Uses a pre-extracted RC value in Wd_rc_cached.  Copies to Wd_rc_scratch
// first because the CBZ/SUB chain is destructive.

static void emit_rcmode_dispatch_cached(AssemblerBuffer& buf, int Wd_int, int Dd_val,
                                         int is_64bit_int, int Wd_rc_cached,
                                         int Wd_rc_scratch) {
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

static void emit_frint_dispatch_cached(AssemblerBuffer& buf, int Dd, int Dn,
                                        int Wd_rc_cached, int Wd_rc_scratch) {
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

static void emit_rcmode_dispatch(AssemblerBuffer& buf, int Wd_int, int Dd_val,
                                  int is_64bit_int, int Xbase, int Wd_rc) {
    if (g_rosetta_config && g_rosetta_config->fast_round) {
        // Fast path: assume RC=0 (round-to-nearest).
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=FCVTNS*/ 0,
                            Wd_int, Dd_val);
    } else {
        // [0] LDRH Wd_rc, [Xbase, #0]  ; control_word
        emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*opc=*/1 /*LDR*/, /*imm12=*/0, Xbase, Wd_rc);
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
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=*/ 3, Wd_int, Dd_val);
        // [8] B +6 → done
        emit_b(buf, 6);
        // [9] FCVTNS (rmode=0)  RC=0 nearest
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=*/ 0, Wd_int, Dd_val);
        // [10] B +4 → done
        emit_b(buf, 4);
        // [11] FCVTMS (rmode=2)  RC=1 floor
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=*/ 2, Wd_int, Dd_val);
        // [12] B +2 → done
        emit_b(buf, 2);
        // [13] FCVTPS (rmode=1)  RC=2 ceil
        emit_fcvt_fp_to_int(buf, is_64bit_int, /*ftype=double*/ 1, /*rmode=*/ 1, Wd_int, Dd_val);
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

    // ── FPR assignment ──────────────────────────────────────────────────────
    FPRState fprs;
    fprs.compute_last_uses(ctx);

    // ── RC caching: hoist LDRH+UBFX when ≥2 RC consumers in a segment ────
    int Wd_rc_cached = -1;
    bool rc_cache_valid = false;

    if (!(g_rosetta_config && g_rosetta_config->fast_round)) {
        int rc_count = 0;
        bool use_rc_cache = false;
        for (int i = 0; i < ctx.num_nodes; i++) {
            auto& n = ctx.nodes[i];
            if (n.flags & kDead) { continue;
}
            if (n.op == Op::StoreCW) { continue; }
            bool is_rc = (n.op == Op::FRndInt) ||
                ((n.op == Op::StoreI16 || n.op == Op::StoreI32 || n.op == Op::StoreI64)
                 && !(n.flags & kTruncate));
            if (is_rc && ++rc_count >= 2) { use_rc_cache = true; break; }
        }
        if (use_rc_cache) {
            Wd_rc_cached = alloc_gpr(*result, 3);
}
    }

    // ── Emit each IR node ───────────────────────────────────────────────────
    for (int i = 0; i < ctx.num_nodes; i++) {
        auto& n = ctx.nodes[i];
        if (n.flags & kDead) { continue;
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
            emit_ldrs(buf, /*is_64bit=*/0, /*size=S16*/1, Wd_val, addr);
            free_gpr(*result, addr);
            // SCVTF Dd, Wd_val
            emit_scvtf(buf, /*is_64bit_int=*/0, /*ftype=f64*/1, Dd, Wd_val);
            free_gpr(*result, Wd_val);
            break;
        }
        case Op::LoadI32: {
            int Dd = alloc_free_fpr(*result);
            fprs.node_fpr[i] = static_cast<int8_t>(Dd);
            int addr = compute_operand_address(*result, true, n.mem_operand, GPR::XZR);
            int Wd_val = alloc_free_gpr(*result);
            emit_ldr_imm(buf, /*size=S32*/2, Wd_val, addr, 0);
            free_gpr(*result, addr);
            // SXTW + SCVTF: sign-extend 32→64 then convert
            emit_scvtf(buf, /*is_64bit_int=*/0, /*ftype=f64*/1, Dd, Wd_val);
            free_gpr(*result, Wd_val);
            break;
        }
        case Op::LoadI64: {
            int Dd = alloc_free_fpr(*result);
            fprs.node_fpr[i] = static_cast<int8_t>(Dd);
            int addr = compute_operand_address(*result, true, n.mem_operand, GPR::XZR);
            int Xd_val = alloc_free_gpr(*result);
            emit_ldr_imm(buf, /*size=S64*/3, Xd_val, addr, 0);
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
            if (Dd < 0) { Dd = alloc_free_fpr(*result);
}
            fprs.node_fpr[i] = static_cast<int8_t>(Dd);
            emit_fadd_f64(buf, Dd, Dn, Dm);
            break;
        }
        case Op::FSub: {
            int Dn = fprs.get(n.inputs[0]);
            int Dm = fprs.get(n.inputs[1]);
            int Dd = fprs.try_reuse_input(ctx, i);
            if (Dd < 0) { Dd = alloc_free_fpr(*result);
}
            fprs.node_fpr[i] = static_cast<int8_t>(Dd);
            emit_fsub_f64(buf, Dd, Dn, Dm);
            break;
        }
        case Op::FMul: {
            int Dn = fprs.get(n.inputs[0]);
            int Dm = fprs.get(n.inputs[1]);
            int Dd = fprs.try_reuse_input(ctx, i);
            if (Dd < 0) { Dd = alloc_free_fpr(*result);
}
            fprs.node_fpr[i] = static_cast<int8_t>(Dd);
            emit_fmul_f64(buf, Dd, Dn, Dm);
            break;
        }
        case Op::FDiv: {
            int Dn = fprs.get(n.inputs[0]);
            int Dm = fprs.get(n.inputs[1]);
            int Dd = fprs.try_reuse_input(ctx, i);
            if (Dd < 0) { Dd = alloc_free_fpr(*result);
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
            if (Dd < 0) { Dd = alloc_free_fpr(*result);
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
            if (Dd < 0) { Dd = alloc_free_fpr(*result);
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
            if (Dd < 0) { Dd = alloc_free_fpr(*result);
}
            fprs.node_fpr[i] = static_cast<int8_t>(Dd);
            emit_fnmsub_f64(buf, Dd, Dn, Dm, Da);
            break;
        }

        // ── Unary ───────────────────────────────────────────────────────
        case Op::FNeg: {
            int Dn = fprs.get(n.inputs[0]);
            int Dd = fprs.try_reuse_input(ctx, i);
            if (Dd < 0) { Dd = alloc_free_fpr(*result);
}
            fprs.node_fpr[i] = static_cast<int8_t>(Dd);
            emit_fneg_f64(buf, Dd, Dn);
            break;
        }
        case Op::FAbs: {
            int Dn = fprs.get(n.inputs[0]);
            int Dd = fprs.try_reuse_input(ctx, i);
            if (Dd < 0) { Dd = alloc_free_fpr(*result);
}
            fprs.node_fpr[i] = static_cast<int8_t>(Dd);
            emit_fabs_f64(buf, Dd, Dn);
            break;
        }
        case Op::FSqrt: {
            int Dn = fprs.get(n.inputs[0]);
            int Dd = fprs.try_reuse_input(ctx, i);
            if (Dd < 0) { Dd = alloc_free_fpr(*result);
}
            fprs.node_fpr[i] = static_cast<int8_t>(Dd);
            emit_fsqrt_f64(buf, Dd, Dn);
            break;
        }
        case Op::FRndInt: {
            int Dn = fprs.get(n.inputs[0]);
            int Dd = fprs.try_reuse_input(ctx, i);
            if (Dd < 0) { Dd = alloc_free_fpr(*result);
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
                emit_cbz(buf, 0, 0, Wd_rc, 7);                         // RC==0 → FRINTN
                emit_add_imm(buf, 0, 1, 0, 0, 1, Wd_rc, Wd_rc);       // SUB 1
                emit_cbz(buf, 0, 0, Wd_rc, 7);                         // RC==1 → FRINTM
                emit_add_imm(buf, 0, 1, 0, 0, 1, Wd_rc, Wd_rc);       // SUB 1
                emit_cbz(buf, 0, 0, Wd_rc, 7);                         // RC==2 → FRINTP
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

        // ── Conditional select (FCMOV) ────────────────────────────────
        case Op::FCSel: {
            int Dn = fprs.get(n.inputs[0]);   // ST(0) — false arm
            int Dm = fprs.get(n.inputs[1]);   // ST(i) — true arm
            int Dd = fprs.try_reuse_input(ctx, i);
            if (Dd < 0) { Dd = alloc_free_fpr(*result);
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
            // Narrow f64 → f32, then store.
            int Ds_tmp = alloc_free_fpr(*result);
            emit_fcvt_d_to_s(buf, Ds_tmp, fprs.get(n.inputs[0]));
            int addr = compute_operand_address(*result, true, n.mem_operand, GPR::XZR);
            emit_fstr_imm(buf, 2, Ds_tmp, addr, 0);
            free_gpr(*result, addr);
            free_fpr(*result, Ds_tmp);
            break;
        }

        // ── Integer stores (FISTP/FIST/FISTTP) ──────────────────────────
        case Op::StoreI16:
        case Op::StoreI32:
        case Op::StoreI64: {
            int Dd_val = fprs.get(n.inputs[0]);
            int Wd_int = alloc_free_gpr(*result);
            int is_64bit_int = (n.op == Op::StoreI64) ? 1 : 0;
            int store_size = (n.op == Op::StoreI16) ? 1
                           : (n.op == Op::StoreI32) ? 2
                                                    : 3;

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
                emit_rcmode_dispatch_cached(buf, Wd_int, Dd_val, is_64bit_int,
                                             Wd_rc_cached, Wd_tmp);
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

            emit_cset(buf, /*is_64bit=*/0, /*cond=*/0 /*EQ*/, Wd_z);   // 1 if equal
            emit_cset(buf, /*is_64bit=*/0, /*cond=*/6 /*VS*/, Wd_v);   // 1 if unordered
            emit_cset(buf, /*is_64bit=*/0, /*cond=*/2 /*CS*/, Wd_c);   // 1 if carry set

            // Z_new = Z | V  (equal or unordered → ZF)
            emit_logical_shifted_reg(buf, 0, /*ORR*/1, 0, /*LSL*/0, Wd_v, 0, Wd_z, Wd_z);
            // C_new = C & !V  (carry clear for unordered → CF)
            emit_logical_shifted_reg(buf, 0, /*AND*/0, /*N=invert rhs*/1, /*LSL*/0, Wd_v, 0, Wd_c, Wd_c);

            // Pack NZCV: bit30=ZF, bit29=CF, bit28=V(PF for FCMOV), bit26=PF
            emit_bitfield(buf, /*is_64bit=*/0, /*opc=*/2 /*UBFM*/, /*N=*/0,
                          /*immr=*/2, /*imms=*/1, Wd_z, Wd_z);
            emit_logical_shifted_reg(buf, 0, /*ORR*/1, 0, /*LSL*/0, Wd_c, 29, Wd_z, Wd_z);
            emit_logical_shifted_reg(buf, 0, /*ORR*/1, 0, /*LSL*/0, Wd_v, 28, Wd_z, Wd_z);
            emit_logical_shifted_reg(buf, 0, /*ORR*/1, 0, /*LSL*/0, Wd_v, 26, Wd_z, Wd_z);

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
            emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR*/1, /*imm12=*/0, addr, Wd_cw);
            free_gpr(*result, addr);
            // STRH Wd_cw, [Xbase, #0]  — control_word is at offset 0x00 → imm12=0
            emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR*/0, /*imm12=*/0, Xbase, Wd_cw);
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
            emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR*/1, /*imm12=*/0, Xbase, Wd_cw);
            int addr = compute_operand_address(*result, true, n.mem_operand, GPR::XZR);
            emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*STR*/0, /*imm12=*/0, addr, Wd_cw);
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
                emit_ldr_str_imm(buf, /*size=*/1, /*is_fp=*/0, /*LDR*/1,
                                 kSwImm12, Xbase, Wd_sw);

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
                    emit_bitfield(buf, 0, /*BFM*/1, 0, /*immr=*/21, /*imms=*/2,
                                  Wd_adj, Wd_sw);
                    free_gpr(*result, Wd_adj);
                }

                // BFI W_ax, Wd_sw, #0, #16 — write status_word into x86 AX
                int W_ax = n.inputs[1];  // destination register index (usually 0 = W0)
                emit_bitfield(buf, 0, /*BFM*/1, 0, /*immr=*/0, /*imms=*/15,
                              Wd_sw, W_ax);
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
        if (val < 0) { continue;  // initial slot, unchanged (no store needed)
}
        // Skip redundant write-back: if the value is a ReadSt loaded from the
        // same physical slot it would be stored to, the store is a no-op.
        if (ctx.nodes[val].op == Op::ReadSt &&
            ctx.nodes[val].initial_depth == d + ctx.top_delta) {
            continue;
}
        int Dd = fprs.get(val);
        if (Dd < 0) { continue;   // dead or already freed
}
        emit_store_st(buf, Xbase, Wd_top, d, Wd_tmp, Dd, Xst_base);
    }

    // 3. Write TOP to status_word (if changed).
    if (ctx.top_delta != 0) {
        emit_store_top(buf, Xbase, Wd_top, Wd_tmp);
    }

    // 4. Update tag word for net pushes/pops.
    if (ctx.top_delta != 0) {
        int Wd_tmp2 = alloc_free_gpr(*result);
        if (ctx.top_delta > 0) {
            // Net pops: use the batch helper.
            int Wd_tagw = alloc_free_gpr(*result);
            emit_x87_tag_set_empty_batch(buf, Xbase, Wd_top, Wd_tmp, Wd_tmp2,
                                          Wd_tagw, ctx.top_delta);
            free_gpr(*result, Wd_tagw);
        } else {
            // Net pushes: clear tag bits for new slots.
            int abs_delta = -ctx.top_delta;
            int Wd_tagw = alloc_free_gpr(*result);
            emit_x87_tag_set_valid_batch(buf, Xbase, Wd_top, Wd_tmp, Wd_tmp2,
                                          Wd_tagw, abs_delta);
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
int peak_live_gprs(const Context& ctx) {
    // Determine if RC caching will be active (same logic as lower()).
    bool rc_cache = false;
    if (!(g_rosetta_config && g_rosetta_config->fast_round)) {
        int rc_count = 0;
        for (int i = 0; i < ctx.num_nodes; i++) {
            const auto& n = ctx.nodes[i];
            if (n.flags & kDead) { continue;
}
            if (n.op == Op::StoreCW) { continue;
}
            bool is_rc = (n.op == Op::FRndInt) ||
                ((n.op == Op::StoreI16 || n.op == Op::StoreI32 || n.op == Op::StoreI64)
                 && !(n.flags & kTruncate));
            if (is_rc && ++rc_count >= 2) { rc_cache = true; break; }
        }
    }

    int pinned = 4;  // Xbase, Wd_top, Wd_tmp, Xst_base
    if (rc_cache) { pinned++;  // Wd_rc_cached
}

    // Simulate GPR pressure across IR nodes.
    // "held" tracks GPRs held alive across node boundaries (fused FCmp→FStsw).
    int held = 0;
    int peak = pinned;  // at minimum, permanent GPRs

    for (int i = 0; i < ctx.num_nodes; i++) {
        const auto& n = ctx.nodes[i];
        if (n.flags & kDead) { continue;
}

        int transient = 0;  // per-node transient GPR demand

        switch (n.op) {
        // No transient GPRs
        case Op::ReadSt:
        case Op::ConstZero: case Op::ConstOne: case Op::ConstF64:
        case Op::FAdd: case Op::FSub: case Op::FMul: case Op::FDiv:
        case Op::FMAdd: case Op::FMSub: case Op::FNMSub:
        case Op::FNeg: case Op::FAbs: case Op::FSqrt:
        case Op::FCSel:
            break;

        // 1 GPR: addr from compute_operand_address
        case Op::LoadF64: case Op::LoadF32:
        case Op::StoreF64: case Op::StoreF32:
            transient = 1;
            break;

        // 2 GPRs: addr + Wd_val
        case Op::LoadI16: case Op::LoadI32: case Op::LoadI64:
            transient = 2;
            break;

        // 2 GPRs: Wd_int + addr
        case Op::StoreI16: case Op::StoreI32: case Op::StoreI64:
            transient = 2;
            break;

        // FRndInt: 1 GPR if no RC cache (alloc_gpr(3) for Wd_rc), 0 with cache
        case Op::FRndInt:
            if (!rc_cache) { transient = 1;
}
            break;

        // FCmp/FTst: 4 peak inside emit_fcom_cc_pack
        // (Wd_save + Wd_packed + Wd_cc + Wd_vs simultaneously live)
        // If fused, Wd_packed stays alive after → held++
        case Op::FCmp: case Op::FTst:
            transient = 4;
            if (n.flags & kFcomFused) {
                // After the node, Wd_packed remains held until FStsw
                held++;
            }
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
        case Op::StoreCW: case Op::LoadCW:
            transient = 2;
            break;
        }

        int node_total = pinned + held + transient;
        peak = std::max(node_total, peak);
    }

    // Epilogue: if top_delta != 0, needs 2 more transient GPRs (Wd_tmp2 + Wd_tagw)
    if (ctx.top_delta != 0) {
        int epilogue_total = pinned + held + 2;
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
        if (n.flags & kDead) { continue;
}
        for (short input : n.inputs) {
            if (input >= 0) { last_use[input] = static_cast<int16_t>(i);
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
        if (n.flags & kDead) { continue;
}

        // Allocate FPR for nodes that produce an FPR-bearing value.
        bool produces_fpr = false;
        switch (n.op) {
        case Op::ReadSt:
        case Op::LoadF64: case Op::LoadF32:
        case Op::LoadI16: case Op::LoadI32: case Op::LoadI64:
        case Op::ConstZero: case Op::ConstOne: case Op::ConstF64:
        case Op::FAdd: case Op::FSub: case Op::FMul: case Op::FDiv:
        case Op::FMAdd: case Op::FMSub: case Op::FNMSub:
        case Op::FNeg: case Op::FAbs: case Op::FSqrt: case Op::FRndInt:
        case Op::FCSel:
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
                int64_t start_idx, int run_length) {
    Context ctx;

    if (!build(ctx, instr_array, num_instrs, start_idx, run_length)) {
        return 0;
}

    optimize(ctx);

    // Gate lowering on actual FPR pressure vs. available pool.
    uint32_t fpr_pool = result->free_fpr_mask;
    int available = 0;
    while (fpr_pool) { available++; fpr_pool &= fpr_pool - 1; }
    if (peak_live_fprs(ctx) > available) {
        return 0;
    }

    // Gate lowering on GPR pressure vs. available pool.
    {
        uint32_t gpr_pool = result->free_gpr_mask;
        int gpr_available = 0;
        while (gpr_pool) { gpr_available++; gpr_pool &= gpr_pool - 1; }
        if (peak_live_gprs(ctx) > gpr_available) {
            return 0;
        }
    }

    lower(ctx, result);

    return ctx.consumed;
}

}  // namespace X87IR
