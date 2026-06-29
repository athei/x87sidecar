#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "rosetta_core/Config.h"
#include "rosetta_core/CoreConfig.h"
#include "rosetta_core/IROperand.h"
#include "rosetta_core/X87IR.h"

namespace X87IR {

// pass_fma_reduce diagnostic counters.  Atomic so the runtime can
// increment them safely from worker threads.  Snapshot via
// fma_reduce_stats(); print via fma_reduce_print_stats().
namespace fma_reduce_counters {
std::atomic<uint64_t> invocations{0};
std::atomic<uint64_t> candidates_seen{0};
std::atomic<uint64_t> chains_tagged{0};
std::atomic<uint64_t> rejected_short{0};
std::atomic<uint64_t> rejected_load_kind{0};
std::atomic<uint64_t> rejected_load_use{0};
std::atomic<uint64_t> rejected_mem_shape{0};
std::atomic<uint64_t> rejected_stride{0};
}  // namespace fma_reduce_counters

// Reject reason discriminator used inside the pass to route to the right
// counter.  Order matches struct field declaration so the report renders
// the buckets left-to-right in coverage importance.
enum class FmaRejectReason : uint8_t {
    Stride = 0,
    LoadKind,
    LoadUse,
    MemShape,
};

FmaReduceStats fma_reduce_stats() {
    using namespace fma_reduce_counters;
    return FmaReduceStats{
        .invocations = invocations.load(std::memory_order_relaxed),
        .candidates_seen = candidates_seen.load(std::memory_order_relaxed),
        .chains_tagged = chains_tagged.load(std::memory_order_relaxed),
        .rejected_short = rejected_short.load(std::memory_order_relaxed),
        .rejected_load_kind = rejected_load_kind.load(std::memory_order_relaxed),
        .rejected_load_use = rejected_load_use.load(std::memory_order_relaxed),
        .rejected_mem_shape = rejected_mem_shape.load(std::memory_order_relaxed),
        .rejected_stride = rejected_stride.load(std::memory_order_relaxed),
    };
}

void fma_reduce_print_stats() {
    if (std::getenv("X87_LOG_FMA_REDUCE") == nullptr) {
        return;
    }
    const FmaReduceStats s = fma_reduce_stats();
    std::printf(
        "[fma_reduce] invocations=%llu candidates=%llu chains_tagged=%llu "
        "rej_short=%llu rej_loadkind=%llu rej_loaduse=%llu "
        "rej_memshape=%llu rej_stride=%llu\n",
        static_cast<unsigned long long>(s.invocations),
        static_cast<unsigned long long>(s.candidates_seen),
        static_cast<unsigned long long>(s.chains_tagged),
        static_cast<unsigned long long>(s.rejected_short),
        static_cast<unsigned long long>(s.rejected_load_kind),
        static_cast<unsigned long long>(s.rejected_load_use),
        static_cast<unsigned long long>(s.rejected_mem_shape),
        static_cast<unsigned long long>(s.rejected_stride));
}

}  // namespace X87IR

namespace X87IR {

// ── Pass 1: Dead Store Elimination ──────────────────────────────────────────
//
// Walk backward. A value node is dead if it has no live consumers (no other
// node references it in inputs[], and it's not in the final slot_val[]).
// Side-effect nodes (Store*, FCmp, FTst, FStsw) are never eliminated.

static void pass_dse(Context& ctx) {
    // Count uses for each node.
    int16_t use_count[kMaxNodes] = {};

    // Final stack values count as uses.
    for (short d : ctx.slot_val) {
        if (d >= 0 && d < ctx.num_nodes) {
            use_count[d]++;
        }
    }

    // Forward pass to count uses from non-dead nodes.
    for (int i = 0; i < ctx.num_nodes; i++) {
        auto& n = ctx.nodes[i];
        if (n.flags & kDead) {
            continue;
        }
        for (short input : n.inputs) {
            if (input >= 0) {
                use_count[input]++;
            }
        }
    }

    // Backward pass: mark dead nodes (pure-value nodes with zero uses).
    for (int i = ctx.num_nodes - 1; i >= 0; i--) {
        auto& n = ctx.nodes[i];
        if (n.flags & kDead) {
            continue;
        }

        // Side-effect nodes are never dead.
        switch (n.op) {
            case Op::StoreF64:
            case Op::StoreF32:
            case Op::StoreI16:
            case Op::StoreI32:
            case Op::StoreI64:
            case Op::FCmp:
            case Op::FTst:
            case Op::FStsw:
            case Op::FComI:
            case Op::StoreCW:
            case Op::LoadCW:
                continue;
            default:
                break;
        }

        if (use_count[i] == 0) {
            n.flags |= kDead;
            // Decrement use counts for this node's inputs.
            for (short input : n.inputs) {
                if (input >= 0) {
                    use_count[input]--;
                }
            }
        }
    }

    // Second backward pass to catch cascading dead nodes.
    for (int i = ctx.num_nodes - 1; i >= 0; i--) {
        auto& n = ctx.nodes[i];
        if (n.flags & kDead) {
            continue;
        }
        switch (n.op) {
            case Op::StoreF64:
            case Op::StoreF32:
            case Op::StoreI16:
            case Op::StoreI32:
            case Op::StoreI64:
            case Op::FCmp:
            case Op::FTst:
            case Op::FStsw:
            case Op::FComI:
            case Op::StoreCW:
            case Op::LoadCW:
                continue;
            default:
                break;
        }
        if (use_count[i] == 0) {
            n.flags |= kDead;
            for (short input : n.inputs) {
                if (input >= 0) {
                    use_count[input]--;
                }
            }
        }
    }
}

// ── Pass 2: FMA Detection ───────────────────────────────────────────────────
//
// Pattern: FMul(a, b) with exactly one use → FAdd(mul, c) or FAdd(c, mul)
//          → replace with FMAdd(a, b, c).
//
// Similarly:
//   FAdd(c, mul) where mul = FMul(a, b)  → FMAdd(a, b, c)  [c + a*b]
//   FSub(c, mul)                          → FMSub(a, b, c)  [c - a*b]
//   FSub(mul, c)                          → FNMSub(a, b, c) [a*b - c]

static void pass_fma(Context& ctx) {
    // Count uses first to find single-use FMul nodes.
    int16_t use_count[kMaxNodes] = {};
    for (short d : ctx.slot_val) {
        if (d >= 0 && d < ctx.num_nodes) {
            use_count[d]++;
        }
    }
    for (int i = 0; i < ctx.num_nodes; i++) {
        auto& n = ctx.nodes[i];
        if (n.flags & kDead) {
            continue;
        }
        for (short input : n.inputs) {
            if (input >= 0) {
                use_count[input]++;
            }
        }
    }

    for (int i = 0; i < ctx.num_nodes; i++) {
        auto& n = ctx.nodes[i];
        if (n.flags & kDead) {
            continue;
        }
        if (n.op != Op::FAdd && n.op != Op::FSub) {
            continue;
        }

        int16_t in0 = n.inputs[0];
        int16_t in1 = n.inputs[1];

        // Check if one input is a single-use FMul.
        auto try_fuse = [&](int mul_input_idx, int other_input_idx) -> bool {
            int16_t mul_id = (mul_input_idx == 0) ? in0 : in1;
            int16_t other_id = (other_input_idx == 0) ? in0 : in1;
            if (mul_id < 0 || mul_id >= ctx.num_nodes) {
                return false;
            }
            auto& mul_node = ctx.nodes[mul_id];
            if (mul_node.op != Op::FMul) {
                return false;
            }
            if (mul_node.flags & kDead) {
                return false;
            }
            if (use_count[mul_id] != 1) {
                return false;
            }

            // Determine FMA variant.
            Op fma_op;
            if (n.op == Op::FAdd) {
                fma_op = Op::FMAdd;  // c + a*b (or a*b + c, same thing)
            } else {
                // FSub
                if (mul_input_idx == 0) {
                    // FSub(mul, c) → a*b - c = FNMSub
                    fma_op = Op::FNMSub;
                } else {
                    // FSub(c, mul) → c - a*b = FMSub
                    fma_op = Op::FMSub;
                }
            }

            // Rewrite: n becomes FMA, mul becomes dead.
            n.op = fma_op;
            n.inputs[0] = mul_node.inputs[0];  // a
            n.inputs[1] = mul_node.inputs[1];  // b
            n.inputs[2] = other_id;            // c (addend)
            mul_node.flags |= kDead;
            use_count[mul_id] = 0;
            // Update use counts for the rewritten inputs.
            if (mul_node.inputs[0] >= 0) {
                use_count[mul_node.inputs[0]]++;
            }
            if (mul_node.inputs[1] >= 0) {
                use_count[mul_node.inputs[1]]++;
            }
            // other_id already had its use from the original node; no change needed.
            return true;
        };

        if (!try_fuse(0, 1)) {
            try_fuse(1, 0);
        }
    }
}

// ── Pass 3: FCOM + FSTSW Fusion ─────────────────────────────────────────────
//
// If a FCmp/FTst node is immediately followed by a FStsw with no intervening
// CC-modifying node, mark both with kFcomFused. The lowering keeps packed CC
// in a GPR between the two nodes, avoiding one LDRH in FStsw.

static void pass_fcom_fstsw_fusion(Context& ctx) {
    for (int i = 0; i < ctx.num_nodes; i++) {
        auto& n = ctx.nodes[i];
        if (n.flags & kDead) {
            continue;
        }
        if (n.op != Op::FStsw) {
            continue;
        }

        int16_t fcmp_id = n.inputs[0];
        if (fcmp_id < 0 || fcmp_id >= ctx.num_nodes) {
            continue;
        }
        auto& fcmp_node = ctx.nodes[fcmp_id];
        if (fcmp_node.op != Op::FCmp && fcmp_node.op != Op::FTst) {
            continue;
        }

        // Check no intervening CC-modifying instruction between fcmp and fstsw.
        bool clean = true;
        for (int j = fcmp_id + 1; j < i; j++) {
            auto& between = ctx.nodes[j];
            if (between.flags & kDead) {
                continue;
            }
            if (between.op == Op::FCmp || between.op == Op::FTst) {
                clean = false;
                break;
            }
        }
        if (clean) {
            fcmp_node.flags |= kFcomFused;
            n.flags |= kFcomFused;
        }
    }
}

// ── Pass 4: FMA Reduction Vectorisation ─────────────────────────────────────
//
// Recognise serial FMADD reduction chains of the shape:
//
//     A0 = <some node>            (initial accumulator: ReadSt, Const, …)
//     A1 = FMAdd(L0, W0, A0)
//     A2 = FMAdd(L1, W1, A1)      ← single-use chain on input[2]
//     ...
//     AN = FMAdd(L_{N-1}, W_{N-1}, A_{N-1})
//
// where every L_i and W_i is a single-use LoadF32 (or, chain-wide, LoadF64)
// with a simple base+disp MemRef, and each memory stream advances by its own
// fixed stride: disp(L_i) = disp(L_0) + stride_L*i, all sharing one base
// register (same for W_i, with an independent stride_W).  The stride is
// derived from the first step (i=1) and validated across the chain — the
// natural +element_size stride is the contiguous case, larger strides are
// the matrix-vector dot-product idiom (stride-8/16) that dominates the WoW
// workload (rank-1/2/10/12 in /tmp/epoch.prof).
//
// The pass tags the head FMAdd with kFmaReduceHead, and tags every FMAdd
// in the chain plus their LoadF32 input pair with kFmaReduceMember.  The
// lowering pass (X87IRLower::lower_fma_reduce) then emits the body as
// pair-loaded LDR D + FCVTL .2D + FMLA .2D iterations followed by a
// scalar FADDP horizontal sum and a final FADD with A_init.
//
// Gated by RosettaConfig::enable_fma_reduce (X87_ENABLE_FMA_REDUCE=1) for
// staged rollout.  When disabled, the pass returns immediately and the
// chain stays scalar via the existing per-FMAdd FMADD lowering.

static bool extract_simple_mem(const Node& n, uint8_t* base_reg, int64_t* disp) {
    if (n.mem_operand == nullptr) {
        return false;
    }
    const IROperand& op = *n.mem_operand;
    if (op.kind != IROperandKind::MemRef) {
        return false;
    }
    if ((op.mem.mem_flags & 1U) == 0) {  // no base register
        return false;
    }
    if ((op.mem.mem_flags & 2U) != 0) {  // index register present
        return false;
    }
    if (op.mem.seg_override != 0) {
        return false;
    }
    *base_reg = op.mem.base_reg;
    *disp = op.mem.disp;
    return true;
}

static void count_uses(const Context& ctx, int16_t* use_count) {
    for (int i = 0; i < kMaxNodes; i++) {
        use_count[i] = 0;
    }
    for (short d : ctx.slot_val) {
        if (d >= 0 && d < ctx.num_nodes) {
            use_count[d]++;
        }
    }
    for (int i = 0; i < ctx.num_nodes; i++) {
        const auto& n = ctx.nodes[i];
        if (n.flags & kDead) {
            continue;
        }
        for (short input : n.inputs) {
            if (input >= 0 && input < ctx.num_nodes) {
                use_count[input]++;
            }
        }
    }
}

static void pass_fma_reduce(Context& ctx) {
    if (g_rosetta_config == nullptr || !g_rosetta_config->enable_fma_reduce) {
        return;
    }
    fma_reduce_counters::invocations.fetch_add(1, std::memory_order_relaxed);

    int16_t use_count[kMaxNodes];
    count_uses(ctx, use_count);

    int16_t chain[kMaxNodes];

    for (int i = 0; i < ctx.num_nodes; i++) {
        const auto& head = ctx.nodes[i];
        if (head.flags & (kDead | kFmaReduceMember)) {
            continue;
        }
        if (head.op != Op::FMAdd) {
            continue;
        }
        // The candidate head's input[2] (= A_init) must NOT itself be an
        // FMAdd in a chain; otherwise this node would be a chain *member*
        // rather than the head.  We mark members lazily as we extend, so
        // this catches the case where a previously-tagged chain ended at
        // i-1 and i picks up an unrelated FMAdd with the same shape.
        if (head.inputs[2] >= 0 && head.inputs[2] < ctx.num_nodes) {
            const auto& a_init = ctx.nodes[head.inputs[2]];
            if (a_init.op == Op::FMAdd && (a_init.flags & kFmaReduceMember)) {
                continue;
            }
        }

        // Forward-walk: extend the chain as long as the next FMAdd in
        // node order uses our tail as input[2] AND the tail is single-use
        // (its only consumer is the next FMAdd's accumulator slot).
        int chain_len = 0;
        chain[chain_len++] = static_cast<int16_t>(i);
        auto tail = static_cast<int16_t>(i);

        while (chain_len < kMaxNodes) {
            int16_t next = -1;
            for (int j = tail + 1; j < ctx.num_nodes; j++) {
                const auto& m = ctx.nodes[j];
                if (m.flags & kDead) {
                    continue;
                }
                if (m.op != Op::FMAdd) {
                    continue;
                }
                if (m.inputs[2] != tail) {
                    continue;
                }
                next = static_cast<int16_t>(j);
                break;
            }
            if (next < 0) {
                break;
            }
            if (use_count[tail] != 1) {
                break;  // tail also used elsewhere — can't fold its FPR away
            }
            chain[chain_len++] = next;
            tail = next;
        }

        fma_reduce_counters::candidates_seen.fetch_add(1, std::memory_order_relaxed);

        if (chain_len < 2) {
            fma_reduce_counters::rejected_short.fetch_add(1, std::memory_order_relaxed);
            continue;  // need ≥2 trios for the vector path to win
        }

        // Validate every member's LoadF32 inputs are single-use, simple-
        // mem, and that each stream is +4-contiguous from its first.
        bool ok = true;
        FmaRejectReason reject_reason = FmaRejectReason::Stride;
        Op elem_op = Op::LoadF32;  // captured at k==0; all loads must match
        uint8_t base_l = 0;
        uint8_t base_w = 0;
        int64_t disp_l_first = 0;
        int64_t disp_w_first = 0;
        int64_t stride_l = 0;  // derived at k==1
        int64_t stride_w = 0;
        for (int k = 0; k < chain_len && ok; k++) {
            const auto& a = ctx.nodes[chain[k]];
            const int16_t l_id = a.inputs[0];
            const int16_t w_id = a.inputs[1];
            if (l_id < 0 || w_id < 0 || l_id >= ctx.num_nodes || w_id >= ctx.num_nodes) {
                ok = false;
                reject_reason = FmaRejectReason::LoadKind;
                break;
            }
            const auto& ln = ctx.nodes[l_id];
            const auto& wn = ctx.nodes[w_id];
            // Both streams must be the same load width — all LoadF32 (matrix-
            // vector f32) or all LoadF64 (m64 doubles, e.g. TurtleWoW). The
            // lowerer vectorises a single element width per chain.
            if (k == 0) {
                if ((ln.op != Op::LoadF32 && ln.op != Op::LoadF64) || wn.op != ln.op) {
                    ok = false;
                    reject_reason = FmaRejectReason::LoadKind;
                    break;
                }
                elem_op = ln.op;
            } else if (ln.op != elem_op || wn.op != elem_op) {
                ok = false;
                reject_reason = FmaRejectReason::LoadKind;
                break;
            }
            if ((ln.flags & kDead) || (wn.flags & kDead)) {
                ok = false;
                reject_reason = FmaRejectReason::LoadKind;
                break;
            }
            if (use_count[l_id] != 1 || use_count[w_id] != 1) {
                ok = false;
                reject_reason = FmaRejectReason::LoadUse;
                break;
            }
            if ((ln.flags & kFmaReduceMember) || (wn.flags & kFmaReduceMember)) {
                ok = false;  // already absorbed by another chain
                reject_reason = FmaRejectReason::LoadUse;
                break;
            }
            uint8_t lb = 0;
            uint8_t wb = 0;
            int64_t ldisp = 0;
            int64_t wdisp = 0;
            if (!extract_simple_mem(ln, &lb, &ldisp)) {
                ok = false;
                reject_reason = FmaRejectReason::MemShape;
                break;
            }
            if (!extract_simple_mem(wn, &wb, &wdisp)) {
                ok = false;
                reject_reason = FmaRejectReason::MemShape;
                break;
            }
            if (k == 0) {
                base_l = lb;
                base_w = wb;
                disp_l_first = ldisp;
                disp_w_first = wdisp;
            } else {
                if (k == 1) {
                    // Derive each stream's constant stride from the first
                    // step. Generalises the original +4-contiguous rule to
                    // any fixed stride (matrix-vector idioms run stride-8/16)
                    // and to LoadF64. A zero stride would alias the same
                    // element into every lane — reject as ill-formed.
                    stride_l = ldisp - disp_l_first;
                    stride_w = wdisp - disp_w_first;
                    if (stride_l == 0 || stride_w == 0) {
                        ok = false;
                        reject_reason = FmaRejectReason::Stride;
                        break;
                    }
                }
                const int64_t expected_l = disp_l_first + stride_l * static_cast<int64_t>(k);
                const int64_t expected_w = disp_w_first + stride_w * static_cast<int64_t>(k);
                if (lb != base_l || ldisp != expected_l) {
                    ok = false;
                    reject_reason = FmaRejectReason::Stride;
                    break;
                }
                if (wb != base_w || wdisp != expected_w) {
                    ok = false;
                    reject_reason = FmaRejectReason::Stride;
                    break;
                }
            }
        }

        if (!ok) {
            switch (reject_reason) {
                case FmaRejectReason::Stride:
                    fma_reduce_counters::rejected_stride.fetch_add(1, std::memory_order_relaxed);
                    break;
                case FmaRejectReason::LoadKind:
                    fma_reduce_counters::rejected_load_kind.fetch_add(1, std::memory_order_relaxed);
                    break;
                case FmaRejectReason::LoadUse:
                    fma_reduce_counters::rejected_load_use.fetch_add(1, std::memory_order_relaxed);
                    break;
                case FmaRejectReason::MemShape:
                    fma_reduce_counters::rejected_mem_shape.fetch_add(1, std::memory_order_relaxed);
                    break;
            }
            // Deep-debug: when X87_LOG_FMA_REDUCE_VERBOSE=1, print the
            // chain's address layout on stride-fail so the actual workload
            // shape (e.g. WoW's stride-16 matrix-vector) is visible.
            // stdout per feedback_logging_preferences.md.
            if (reject_reason == FmaRejectReason::Stride && chain_len >= 2) {
                static const char* verbose = std::getenv("X87_LOG_FMA_REDUCE_VERBOSE");
                if (verbose != nullptr && verbose[0] == '1') {
                    std::printf("[fma_reduce] stride-fail chain_len=%d:", chain_len);
                    for (int k = 0; k < chain_len; k++) {
                        const auto& a = ctx.nodes[chain[k]];
                        const int16_t l_id = a.inputs[0];
                        const int16_t w_id = a.inputs[1];
                        if (l_id < 0 || w_id < 0) {
                            continue;
                        }
                        uint8_t lb = 0;
                        uint8_t wb = 0;
                        int64_t ldisp = 0;
                        int64_t wdisp = 0;
                        extract_simple_mem(ctx.nodes[l_id], &lb, &ldisp);
                        extract_simple_mem(ctx.nodes[w_id], &wb, &wdisp);
                        std::printf(" [%d L:base=%u disp=%lld W:base=%u disp=%lld]", k, lb,
                                    static_cast<long long>(ldisp), wb,
                                    static_cast<long long>(wdisp));
                    }
                    // Surface input[2]'s op so the FMul-as-input[2]
                    // pair-sum shape is visible (different from FMAdd
                    // chains and ReadSt-rooted chains).
                    const int16_t a_init = ctx.nodes[chain[0]].inputs[2];
                    const char* init_op = "<oob>";
                    if (a_init >= 0 && a_init < ctx.num_nodes) {
                        switch (ctx.nodes[a_init].op) {
                            case Op::ReadSt:
                                init_op = "ReadSt";
                                break;
                            case Op::FMul:
                                init_op = "FMul";
                                break;
                            case Op::FMAdd:
                                init_op = "FMAdd";
                                break;
                            case Op::ConstZero:
                                init_op = "ConstZero";
                                break;
                            case Op::ConstOne:
                                init_op = "ConstOne";
                                break;
                            case Op::LoadF32:
                                init_op = "LoadF32";
                                break;
                            case Op::LoadF64:
                                init_op = "LoadF64";
                                break;
                            default:
                                init_op = "other";
                                break;
                        }
                    }
                    std::printf(" A_init=%s\n", init_op);
                }
            }
            continue;
        }
        fma_reduce_counters::chains_tagged.fetch_add(1, std::memory_order_relaxed);

        // Tag.  Every chain FMAdd gets kFmaReduceMember; the first
        // additionally gets kFmaReduceHead.  Each absorbed LoadF32 gets
        // kFmaReduceMember so the lowerer skips its scalar LDR S + FCVT
        // emission and peak_live_fprs treats it as zero-cost.
        ctx.nodes[chain[0]].flags |= kFmaReduceHead;
        for (int k = 0; k < chain_len; k++) {
            auto& a = ctx.nodes[chain[k]];
            a.flags |= kFmaReduceMember;
            ctx.nodes[a.inputs[0]].flags |= kFmaReduceMember;
            ctx.nodes[a.inputs[1]].flags |= kFmaReduceMember;
        }
    }
}

// ── Public API ──────────────────────────────────────────────────────────────

void optimize(Context& ctx) {
    pass_dse(ctx);
    pass_fma(ctx);
    pass_fma_reduce(ctx);
    pass_fcom_fstsw_fusion(ctx);
}

}  // namespace X87IR
