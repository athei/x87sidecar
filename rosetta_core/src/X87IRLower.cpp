#include <algorithm>
#include <cstdint>
#include <cstdio>
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
        // Re-acquire Xst_base if a prior epilogue dropped it (it is freed
        // before the tag-batch alloc when top_delta != 0).  One ADD here
        // keeps the rest of the run on the cached 2-3 insn ST access path
        // instead of the uncached 5-insn path — this is the common shape
        // after compile_run consumes a run prefix (pressure split or build
        // early-stop) and the suffix re-enters lowering.
        if (!a1.x87_cache.st_base_valid) {
            const int Xst_base = alloc_gpr(a1, 6);
            emit_add_imm(buf, 1, 0, 0, 0, kX87RegFileOff, a1.x87_cache.base_gpr, Xst_base);
            a1.x87_cache.st_base_gpr = static_cast<int8_t>(Xst_base);
            a1.x87_cache.st_base_valid = 1;
        }
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
                                 int Xbase, int Wd_rc, bool fast_round) {
    if (fast_round) {
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

// ── FMA reduction chain lowering ────────────────────────────────────────────
//
// Emits the body for an FMA reduction chain whose head is at `head_idx`.
// Pre-conditions (established by pass_fma_reduce in X87IROptimize.cpp):
//   * Every chain FMAdd has kFmaReduceMember; head additionally kFmaReduceHead
//   * Every absorbed L_i / W_i load has kFmaReduceMember
//   * For each chain member k, inputs[0]/inputs[1] are loads with simple
//     base+disp memrefs, all the same width (LoadF32 or, chain-wide,
//     LoadF64); the L and W streams each advance by their own constant
//     stride from a per-stream base register
//
// Body shape (N = chain length).  Each pair packs two trios into the two
// f64 lanes of V_acc.  How the lane pair is loaded depends on element width
// and stride (derived from the first step, mirroring the pass):
//   FMOV V_acc, A_init          ; lane 0 = A_init, lane 1 = 0 (NEON write
//                                 semantics zero the upper 64 bits)
//   for p in 0..floor(N/2)-1:
//       load V_data lanes ← L_{2p}, L_{2p+1}   (see load_fma_pair_v2d)
//       load V_w    lanes ← W_{2p}, W_{2p+1}
//       FMLA V_acc.2D, V_data.2D, V_w.2D
//   FADDP D_result, V_acc.2D    ; horizontal sum: D_result = lane0 + lane1
//   if N odd: scalar load(+widen) L_{N-1}, W_{N-1}; FMADD into D_result
//
// Lane-load strategy per stream (load_fma_pair_v2d):
//   * f32 contiguous (stride 4): one LDR D loads the adjacent pair, FCVTL .2D
//   * f64 contiguous (stride 8): one LDR Q loads the adjacent pair (already .2D)
//   * strided:  LDR S/D lane 0, then LD1 {V.S/D}[1] lane 1 (FCVTL for f32)
//
// FP rounding note: the vector path accumulates even-indexed and odd-
// indexed products into separate lanes, then does a single FADDP combine
// at the end, plus a leading FMOV that places A_init in lane 0.  This
// re-associates the additions versus the strict left-to-right scalar
// chain (the widened products themselves are unchanged, and each is still
// fused via FMLA).  Bit-exact equality with the scalar path is therefore
// not guaranteed in general; tests pin inputs that are exactly
// representable so the re-association is exact.

// Load the f64 lane-pair (lane 0 ← n_lane0, lane 1 ← n_lane1) of one stream
// into V as a .2D vector, widening from f32 when needed.  `contiguous` means
// n_lane1 sits exactly element_size bytes after n_lane0, so a single
// LDR D (f32) / LDR Q (f64) fetches both lanes; otherwise each lane is loaded
// separately (LDR + LD1-into-lane-1).
static void load_fma_pair_v2d(AssemblerBuffer& buf, TranslationResult* result, Op elem_op,
                              bool contiguous, int V, const Node& n_lane0, const Node& n_lane1) {
    if (contiguous) {
        const int Xa =
            compute_operand_address(*result, /*is_64bit=*/true, n_lane0.mem_operand, GPR::XZR);
        if (elem_op == Op::LoadF64) {
            emit_ldr_q_imm(buf, V, Xa, /*imm12=*/0);  // 2 adjacent f64 → .2D
        } else {
            emit_fldr_imm(buf, /*size=*/3, V, Xa, /*imm12=*/0);  // 2 adjacent f32 → .2S
        }
        free_gpr(*result, Xa);
    } else {
        const int Xa0 =
            compute_operand_address(*result, /*is_64bit=*/true, n_lane0.mem_operand, GPR::XZR);
        emit_fldr_imm(buf, elem_op == Op::LoadF64 ? 3 : 2, V, Xa0, /*imm12=*/0);  // lane 0
        free_gpr(*result, Xa0);
        const int Xa1 =
            compute_operand_address(*result, /*is_64bit=*/true, n_lane1.mem_operand, GPR::XZR);
        if (elem_op == Op::LoadF64) {
            emit_ld1_lane_d(buf, V, Xa1, /*lane=*/1);
        } else {
            emit_ld1_lane_s(buf, V, Xa1, /*lane=*/1);
        }
        free_gpr(*result, Xa1);
    }
    if (elem_op == Op::LoadF32) {
        emit_fcvtl_v2d_from_v2s(buf, V, V);
    }
}

static void lower_fma_reduce(AssemblerBuffer& buf, TranslationResult* result, FPRState& fprs,
                             const Context& ctx, int head_idx) {
    // Re-walk the chain (mirror of pass_fma_reduce's extension loop).  No
    // re-validation needed — the pass already proved the shape.
    int chain[X87IR::kMaxNodes];
    int chain_len = 0;
    chain[chain_len++] = head_idx;
    int tail = head_idx;
    while (chain_len < X87IR::kMaxNodes) {
        int next = -1;
        for (int j = tail + 1; j < ctx.num_nodes; j++) {
            const auto& m = ctx.nodes[j];
            if (m.flags & kDead) {
                continue;
            }
            if (m.op != Op::FMAdd) {
                continue;
            }
            if (!(m.flags & kFmaReduceMember)) {
                continue;
            }
            if (m.inputs[2] != tail) {
                continue;
            }
            next = j;
            break;
        }
        if (next < 0) {
            break;
        }
        chain[chain_len++] = next;
        tail = next;
    }

    const int N = chain_len;
    const int num_pairs = N / 2;
    const bool has_odd = (N & 1) != 0;

    // Element width and per-stream stride (mirrors pass_fma_reduce's
    // validation; the pass guarantees both streams share one load op and each
    // advances by a constant stride, and that N >= 2 so chain[1] exists).
    // stride == element_size is the contiguous fast path; anything else needs
    // per-lane strided loads.
    const auto& l0n = ctx.nodes[ctx.nodes[chain[0]].inputs[0]];
    const auto& w0n = ctx.nodes[ctx.nodes[chain[0]].inputs[1]];
    const auto& l1n = ctx.nodes[ctx.nodes[chain[1]].inputs[0]];
    const auto& w1n = ctx.nodes[ctx.nodes[chain[1]].inputs[1]];
    const Op elem_op = l0n.op;
    const int elem_size = (elem_op == Op::LoadF64) ? 8 : 4;
    const bool contig_l = (l1n.mem_operand->mem.disp - l0n.mem_operand->mem.disp) == elem_size;
    const bool contig_w = (w1n.mem_operand->mem.disp - w0n.mem_operand->mem.disp) == elem_size;

    // Capture A_init's FPR (head's input[2]).  Will be FMOV'd into V_acc
    // lane 0 to seed the accumulator.  Auto-freed by free_dead_inputs at
    // the end of the head's iteration since last_use[A_init] == head_idx.
    const int D_init = fprs.get(ctx.nodes[head_idx].inputs[2]);

    // Allocate the vector accumulator and seed it with A_init in lane 0.
    const int V_acc = alloc_free_fpr(*result);
    emit_fmov_f64(buf, V_acc, D_init);

    // Allocate transient vector regs reused across all pairs.
    const int V_data = alloc_free_fpr(*result);
    const int V_w = alloc_free_fpr(*result);

    for (int p = 0; p < num_pairs; p++) {
        // Each pair processes trios chain[2p] (lane 0) and chain[2p+1] (lane 1).
        const auto& a0 = ctx.nodes[chain[2 * p]];
        const auto& a1 = ctx.nodes[chain[2 * p + 1]];
        load_fma_pair_v2d(buf, result, elem_op, contig_l, V_data, ctx.nodes[a0.inputs[0]],
                          ctx.nodes[a1.inputs[0]]);
        load_fma_pair_v2d(buf, result, elem_op, contig_w, V_w, ctx.nodes[a0.inputs[1]],
                          ctx.nodes[a1.inputs[1]]);
        emit_fmla_v2d(buf, V_acc, V_data, V_w);
    }

    free_fpr(*result, V_data);
    free_fpr(*result, V_w);

    // Horizontal reduction into a fresh scalar D register.
    const int D_result = alloc_free_fpr(*result);
    emit_faddp_d_from_v2d(buf, D_result, V_acc);
    free_fpr(*result, V_acc);

    // Odd-trio scalar tail (only when N is odd).
    if (has_odd) {
        const auto& a_tail = ctx.nodes[chain[N - 1]];
        const auto& l_node = ctx.nodes[a_tail.inputs[0]];
        const auto& w_node = ctx.nodes[a_tail.inputs[1]];
        const int ld_size = (elem_op == Op::LoadF64) ? 3 : 2;

        const int D_l = alloc_free_fpr(*result);
        const int Xaddr_l =
            compute_operand_address(*result, /*is_64bit=*/true, l_node.mem_operand, GPR::XZR);
        emit_fldr_imm(buf, ld_size, D_l, Xaddr_l, /*imm12=*/0);
        free_gpr(*result, Xaddr_l);
        if (elem_op == Op::LoadF32) {
            emit_fcvt_s_to_d(buf, D_l, D_l);
        }

        const int D_w = alloc_free_fpr(*result);
        const int Xaddr_w =
            compute_operand_address(*result, /*is_64bit=*/true, w_node.mem_operand, GPR::XZR);
        emit_fldr_imm(buf, ld_size, D_w, Xaddr_w, /*imm12=*/0);
        free_gpr(*result, Xaddr_w);
        if (elem_op == Op::LoadF32) {
            emit_fcvt_s_to_d(buf, D_w, D_w);
        }

        emit_fmadd_f64(buf, D_result, D_l, D_w, D_result);
        free_fpr(*result, D_l);
        free_fpr(*result, D_w);
    }

    // Assign result to chain tail's FPR slot so downstream consumers see it
    // via fprs.get(A_N).  Intermediate A_2..A_{N-1} keep node_fpr=-1 (they
    // have no externally-visible value).
    fprs.node_fpr[chain[N - 1]] = static_cast<int8_t>(D_result);
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
    const bool fast_round = x87_fast_round_active(*result);

    if (!fast_round) {
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
        // FMA reduction members (LoadF32 / FMAdd in a tagged chain) are
        // absorbed by the head's lower_fma_reduce emission.  The head is
        // dispatched from the FMAdd case below.
        if ((n.flags & kFmaReduceMember) && !(n.flags & kFmaReduceHead)) {
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
                // Vectorised FMA reduction chain head: emit the whole chain
                // body via lower_fma_reduce.  Members of the chain were
                // already filtered out by the kFmaReduceMember early-skip
                // above; only the head reaches here.
                if (n.flags & kFmaReduceHead) {
                    lower_fma_reduce(buf, result, fprs, ctx, i);
                    break;
                }
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

                if (fast_round) {
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
                    emit_rcmode_dispatch(buf, Wd_int, Dd_val, is_64bit_int, Xbase, Wd_tmp,
                                         fast_round);
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
int peak_live_gprs(const Context& ctx, int budget, int* first_over_node, bool fast_round) {
    if (first_over_node) {
        *first_over_node = -1;
    }
    // Determine if RC caching will be active (same logic as lower()).
    // RC cache is disabled when the run contains FCmp/FTst: emit_fcom_cc_pack
    // has a structurally-unavoidable 4-wide GPR peak (Wd_save + Wd_packed +
    // Wd_cc + Wd_vs) which combined with rc_cache's pinned Wd_rc_cached
    // pushes total demand to 9 vs. the 8-slot scratch pool.  Letting the
    // FCmp run lower successfully saves far more (long compare-heavy blocks
    // see arm_no_ir 590 → arm_ir_forced 305, ~285 ARM/exec) than rc_cache's
    // per-RC-op micro-saving (~2 ARM × few ops).
    bool rc_cache = false;
    if (!fast_round) {
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
        if (first_over_node && *first_over_node < 0 && node_total > budget) {
            *first_over_node = i;
        }

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
        // Attribute an epilogue-only overflow to the last node: a shorter
        // prefix may have a different (possibly zero) top_delta, so the
        // split loop still gets a boundary to try.
        if (first_over_node && *first_over_node < 0 && epilogue_total > budget &&
            ctx.num_nodes > 0) {
            *first_over_node = ctx.num_nodes - 1;
        }
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
int peak_live_fprs(const Context& ctx, int budget, int* first_over_node) {
    if (first_over_node) {
        *first_over_node = -1;
    }
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

    // Raise the running peak, recording the first node whose demand
    // exceeds the caller's budget (split-boundary query).
    const auto raise_peak = [&](int candidate, int node_idx) {
        peak = std::max(candidate, peak);
        if (first_over_node && *first_over_node < 0 && candidate > budget) {
            *first_over_node = node_idx;
        }
    };

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

        // FMA-reduction members (non-head FMAdd, absorbed LoadF32) are
        // emitted as part of the chain head's lower_fma_reduce body — no
        // FPR is allocated for them.  Skip the produces_fpr accounting.
        const bool absorbed_member = (n.flags & kFmaReduceMember) && !(n.flags & kFmaReduceHead);
        if (produces_fpr && !absorbed_member) {
            live++;
            holding[i] = true;
            raise_peak(live, i);
        }

        // FMA-reduction head: the whole chain emits as one fused body at the
        // head.  lower_fma_reduce allocates 2 transient vector FPRs (V_data,
        // V_w) on top of the held accumulator V_acc (counted by produces_fpr
        // above); the odd-tail D_l/D_w reuse those freed slots, so one +2
        // spike covers both.
        if (n.flags & kFmaReduceHead) {
            raise_peak(live + 2, i);
            // The chain produces a single result FPR (lower_fma_reduce assigns
            // it to the *tail* member) that stays live until the tail's
            // external consumer.  produces_fpr above attributed that FPR to
            // the head, which the free loop would release at the head's
            // immediate use (chain[1]) — too early, undercounting `live` for
            // every node between chain[1] and the real consumer.  Re-attribute
            // the holding to the tail so it is freed at last_use[tail].
            int tail_node = i;
            for (int j = i + 1; j < ctx.num_nodes; j++) {
                const auto& m = ctx.nodes[j];
                if (m.flags & kDead) {
                    continue;
                }
                if (m.op != Op::FMAdd || !(m.flags & kFmaReduceMember)) {
                    continue;
                }
                if (m.inputs[2] == tail_node) {
                    tail_node = j;
                }
            }
            if (tail_node != i && holding[i]) {
                holding[i] = false;
                holding[tail_node] = true;
            }
        }

        // StoreF32 lowering coalesces consecutive same-input StoreF32 nodes
        // into one fcvt + STR Q / STP S / STR S ladder.  Ds_narrow is always
        // allocated; when at least one 4-wide STR Q group fires, Vq_broadcast
        // is allocated too and both stay live until the group ends — a +2
        // spike, not +1.  Mirror the lowering's group scan and greedy window
        // walk exactly so the model never undercounts (an undercount trips
        // alloc_free_fpr's assert(mask != 0), UB in release).
        if (n.op == Op::StoreF32) {
            int jj = i;
            while (jj + 1 < ctx.num_nodes) {
                const auto& nx = ctx.nodes[jj + 1];
                if ((nx.flags & kDead) != 0) {
                    break;
                }
                if (nx.op != Op::StoreF32 || nx.inputs[0] != n.inputs[0]) {
                    break;
                }
                ++jj;
            }
            int spike = 1;  // Ds_narrow
            int k = i;
            while (k <= jj) {
                if (k + 3 <= jj && can_emit_str_q(ctx.nodes[k], ctx.nodes[k + 1],
                                                  ctx.nodes[k + 2], ctx.nodes[k + 3])) {
                    spike = 2;  // + Vq_broadcast
                    break;
                }
                if (k + 1 <= jj && can_emit_stp_s(ctx.nodes[k], ctx.nodes[k + 1])) {
                    k += 2;
                    continue;
                }
                k += 1;
            }
            raise_peak(live + spike, i);
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
            raise_peak(spike, i);
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

// Pick the run length to retry with after a pressure overflow at
// `first_over_node`: cut just before the instruction that created the
// overflowing node, and always make progress (strictly shorter than
// `attempt_len`).  Returns the new run length (may be < 3 = give up).
static int split_prefix_len(const Context& ctx, int first_over_node, int attempt_len) {
    int len = attempt_len - 1;
    if (first_over_node >= 0 && first_over_node < ctx.num_nodes) {
        const int src = ctx.node_src[first_over_node];
        if (src >= 0 && src < len) {
            len = src;
        }
    }
    return len;
}

// ── Pressure-relief rematerialization / load sinking ────────────────────────
//
// When the FPR gate would refuse, try shortening live ranges before
// resorting to a split: a Const* or (alias-clear) LoadF32/F64 whose only
// remaining use lies beyond the overflow point doesn't need to hold an FPR
// across the gap — clone it immediately before that use and retarget the
// use.  When the late use was the value's ONLY use, the original goes dead
// and the "clone" is just the load sunk to its consumer: zero extra emit.
// Otherwise the clone costs 1-2 ARM, still far below a ~10-ARM split or a
// whole-run refusal.
//
// Lives here (not X87IROptimize.cpp) because candidate legality and payoff
// are joined at the hip with the peak_live_fprs model above.

// Insert a copy of nodes[src] at index `pos` (shifting [pos..) up one) and
// return false if the node budget is exhausted.  All node references —
// inputs, slot_val, initial_read, last_fcmp/last_fcomi, node_src — are
// remapped.  The caller retargets the consumer afterwards.
static bool insert_clone_at(Context& ctx, int src, int pos) {
    if (ctx.num_nodes >= kMaxNodes || src >= pos) {
        return false;
    }
    const int n = ctx.num_nodes;
    std::memmove(&ctx.nodes[pos + 1], &ctx.nodes[pos],
                 static_cast<size_t>(n - pos) * sizeof(Node));
    std::memmove(&ctx.node_src[pos + 1], &ctx.node_src[pos],
                 static_cast<size_t>(n - pos) * sizeof(int16_t));
    ctx.num_nodes = static_cast<int16_t>(n + 1);

    const auto remap = [pos](int16_t& ref) {
        if (ref >= pos) {
            ref = static_cast<int16_t>(ref + 1);
        }
    };
    for (int i = 0; i < ctx.num_nodes; i++) {
        if (i == pos) {
            continue;  // clone slot, written below
        }
        for (auto& in : ctx.nodes[i].inputs) {
            if (in >= 0) {
                remap(in);
            }
        }
    }
    for (auto& v : ctx.slot_val) {
        if (v >= 0) {
            remap(v);
        }
    }
    for (auto& v : ctx.initial_read) {
        if (v >= 0) {
            remap(v);
        }
    }
    if (ctx.last_fcmp >= 0) {
        remap(ctx.last_fcmp);
    }
    if (ctx.last_fcomi >= 0) {
        remap(ctx.last_fcomi);
    }

    ctx.nodes[pos] = ctx.nodes[src];  // src < pos: unshifted
    ctx.nodes[pos].flags = kNone;
    ctx.node_src[pos] = ctx.node_src[pos + 1];  // same instruction as the consumer
    return true;
}

// Returns true if any clone/sink was performed.  Iterates until the FPR
// model fits `budget`, no candidate exists, or the clone cap is reached.
static bool remat_relieve_fpr_pressure(Context& ctx, int budget) {
    constexpr int kMaxClones = 8;
    bool changed = false;

    for (int iter = 0; iter < kMaxClones; iter++) {
        int over = -1;
        if (peak_live_fprs(ctx, budget, &over) <= budget || over < 0) {
            return changed;
        }

        // last_use over non-dead consumers (slot-final values are anchored
        // at num_nodes and excluded from retargeting below).
        int16_t last_use[kMaxNodes];
        std::memset(last_use, -1, sizeof(last_use));
        for (int i = 0; i < ctx.num_nodes; i++) {
            if (ctx.nodes[i].flags & kDead) {
                continue;
            }
            for (const short in : ctx.nodes[i].inputs) {
                if (in >= 0) {
                    last_use[in] = static_cast<int16_t>(i);
                }
            }
        }
        bool slot_live[kMaxNodes] = {};
        for (const short v : ctx.slot_val) {
            if (v >= 0 && v < ctx.num_nodes) {
                slot_live[v] = true;
            }
        }

        // Pick the candidate whose single post-overflow use is farthest out.
        int best = -1;
        int best_use = -1;
        for (int h = 0; h < over; h++) {
            const auto& hn = ctx.nodes[h];
            if (hn.flags != kNone || slot_live[h]) {
                continue;
            }
            const bool is_const = hn.op == Op::ConstZero || hn.op == Op::ConstOne ||
                                  hn.op == Op::ConstF64;
            const bool is_load = hn.op == Op::LoadF32 || hn.op == Op::LoadF64;
            if (!is_const && !is_load) {
                continue;
            }
            if (last_use[h] <= over) {
                continue;  // not live past the overflow point
            }
            // Exactly one use after the overflow point (which is last_use[h]).
            int uses_after = 0;
            for (int j = over + 1; j < ctx.num_nodes; j++) {
                if (ctx.nodes[j].flags & kDead) {
                    continue;
                }
                for (const short in : ctx.nodes[j].inputs) {
                    if (in == h) {
                        uses_after++;
                    }
                }
            }
            if (uses_after != 1) {
                continue;
            }
            const int use = last_use[h];
            // A load may not cross a memory write or FStsw (writes AX or
            // memory) between its original position and the sunk position.
            if (is_load) {
                bool barrier = false;
                for (int j = h + 1; j < use && !barrier; j++) {
                    const auto& m = ctx.nodes[j];
                    if (m.flags & kDead) {
                        continue;
                    }
                    switch (m.op) {
                        case Op::StoreF64:
                        case Op::StoreF32:
                        case Op::StoreI16:
                        case Op::StoreI32:
                        case Op::StoreI64:
                        case Op::StoreCW:
                        case Op::FStsw:
                            barrier = true;
                            break;
                        default:
                            break;
                    }
                }
                if (barrier) {
                    continue;
                }
            }
            if (use > best_use) {
                best = h;
                best_use = use;
            }
        }
        if (best < 0) {
            return changed;
        }

        if (!insert_clone_at(ctx, best, best_use)) {
            return changed;
        }
        // Retarget the consumer (shifted to best_use + 1) to the clone.
        auto& consumer = ctx.nodes[best_use + 1];
        for (auto& in : consumer.inputs) {
            if (in == best) {
                in = static_cast<int16_t>(best_use);
            }
        }
        // If that was the original's only use anywhere, it is now dead —
        // the clone is a pure sink, not a duplicate.
        bool any_use = false;
        for (int j = 0; j < ctx.num_nodes && !any_use; j++) {
            if (j == best || (ctx.nodes[j].flags & kDead)) {
                continue;
            }
            for (const short in : ctx.nodes[j].inputs) {
                if (in == best) {
                    any_use = true;
                }
            }
        }
        if (!any_use && !slot_live[best]) {
            ctx.nodes[best].flags |= kDead;
        }
        changed = true;
    }
    return changed;
}

int compile_run(TranslationResult* result, IRInstr* instr_array, int64_t num_instrs,
                int64_t start_idx, int run_length, IRFailReason* out_reason, int* out_peak_gprs,
                uint16_t* out_fail_opcode) {
    // Pool sizes.  The optional pool-limit clamps (X87_FPR_POOL_LIMIT /
    // X87_GPR_POOL_LIMIT) are test knobs that make pool starvation
    // deterministic — the real pool depends on stock's dynamic
    // _unoccupied_temporary_fprs_for_xmm_scalars seeding.  They narrow the
    // gate only; allocation still draws from the full mask.
    uint32_t fpr_pool = result->free_fpr_mask;
    int available = 0;
    while (fpr_pool) {
        available++;
        fpr_pool &= fpr_pool - 1;
    }
    uint32_t gpr_pool = result->free_gpr_mask;
    int gpr_available = 0;
    while (gpr_pool) {
        gpr_available++;
        gpr_pool &= gpr_pool - 1;
    }
    if (g_rosetta_config) {
        if (g_rosetta_config->fpr_pool_limit > 0 && available > g_rosetta_config->fpr_pool_limit) {
            available = g_rosetta_config->fpr_pool_limit;
        }
        if (g_rosetta_config->gpr_pool_limit > 0 &&
            gpr_available > g_rosetta_config->gpr_pool_limit) {
            gpr_available = g_rosetta_config->gpr_pool_limit;
        }
    }

    // When a run's peak register pressure exceeds the pool, split it: retry
    // with the prefix that ends just before the instruction whose node first
    // overflowed the budget.  Every live value at an instruction boundary is
    // a symbolic-stack value, so the prefix's normal epilogue/ReadSt
    // machinery IS the spill — no new code shapes.  The dispatcher treats
    // the partial consumed count exactly like a build early-stop and
    // re-enters the IR gate at the suffix, which splits recursively if
    // needed.  Bounded retries: each attempt is strictly shorter.
    const bool split_enabled = g_rosetta_config == nullptr || g_rosetta_config->enable_ir_split;
    const bool log_split =
        g_rosetta_config != nullptr && g_rosetta_config->log_ir_split && split_enabled;
    constexpr int kMaxSplitRetries = 4;

    int attempt_len = run_length;
    for (int attempt = 0;; attempt++) {
        Context ctx;

        const bool built = build(ctx, instr_array, num_instrs, start_idx, attempt_len);

        // Surface the bail opcode whenever build observed an unsupported one,
        // including the success-with-early-stop case (consumed >= 2 but the
        // loop halted at position N).  Only the full-length attempt reports:
        // a shrunk prefix stops at run_length, not at an unsupported opcode.
        if (attempt == 0 && out_fail_opcode && ctx.fail_opcode != 0xFFFFU) {
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

        // Gate lowering on actual FPR pressure vs. available pool.  Before
        // splitting, try the cheaper relief: sink/clone long-lived consts
        // and loads past the overflow point (1-2 ARM each vs ~10 per split).
        int fpr_over_node = -1;
        bool remat_changed = false;
        if (peak_live_fprs(ctx, available, &fpr_over_node) > available &&
            (g_rosetta_config == nullptr || g_rosetta_config->enable_ir_remat)) {
            remat_changed = remat_relieve_fpr_pressure(ctx, available);
            if (remat_changed) {
                fpr_over_node = -1;
                if (log_split) {
                    std::fprintf(stderr, "[x87ir-remat] relieved run=%d nodes=%d\n", attempt_len,
                                 static_cast<int>(ctx.num_nodes));
                }
            }
        }
        if (peak_live_fprs(ctx, available, &fpr_over_node) > available) {
            const int next_len = split_prefix_len(ctx, fpr_over_node, attempt_len);
            if (split_enabled && attempt < kMaxSplitRetries && next_len >= 3) {
                if (log_split) {
                    std::fprintf(stderr,
                                 "[x87ir-split] fpr overflow: run=%d -> retry len=%d "
                                 "(over_node=%d avail=%d)\n",
                                 attempt_len, next_len, fpr_over_node, available);
                }
                attempt_len = next_len;
                continue;
            }
            if (out_reason) {
                *out_reason = IRFailReason::kFprPressure;
            }
            if (out_peak_gprs) {
                *out_peak_gprs = peak_live_gprs(ctx, 0x7FFFFFFF, nullptr,
                                                x87_fast_round_active(*result));
            }
            return 0;
        }

        // Gate lowering on GPR pressure vs. available pool.
        int gpr_over_node = -1;
        const int peak_gprs =
            peak_live_gprs(ctx, gpr_available, &gpr_over_node, x87_fast_round_active(*result));
        if (out_peak_gprs) {
            *out_peak_gprs = peak_gprs;
        }
        if (peak_gprs > gpr_available) {
            const int next_len = split_prefix_len(ctx, gpr_over_node, attempt_len);
            if (split_enabled && attempt < kMaxSplitRetries && next_len >= 3) {
                if (log_split) {
                    std::fprintf(stderr,
                                 "[x87ir-split] gpr overflow: run=%d -> retry len=%d "
                                 "(over_node=%d avail=%d)\n",
                                 attempt_len, next_len, gpr_over_node, gpr_available);
                }
                attempt_len = next_len;
                continue;
            }
            if (out_reason) {
                *out_reason = IRFailReason::kGprPressure;
            }
            return 0;
        }

        lower(ctx, result);

        if (attempt > 0) {
            if (result->x87_cache.tally_ir_split != 0xFFFFU) {
                result->x87_cache.tally_ir_split++;
            }
            if (log_split) {
                std::fprintf(stderr, "[x87ir-split] split ok: consumed=%d of run=%d\n",
                             static_cast<int>(ctx.consumed), run_length);
            }
        }
        if (remat_changed && result->x87_cache.tally_ir_remat != 0xFFFFU) {
            result->x87_cache.tally_ir_remat++;
        }

        return ctx.consumed;
    }
}

}  // namespace X87IR
