#include <cstdint>

#include "rosetta_core/X87IR.h"

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

// ── Public API ──────────────────────────────────────────────────────────────

void optimize(Context& ctx) {
    pass_dse(ctx);
    pass_fma(ctx);
    pass_fcom_fstsw_fusion(ctx);
}

}  // namespace X87IR
