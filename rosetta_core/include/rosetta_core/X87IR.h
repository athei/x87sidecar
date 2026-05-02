#pragma once

#include <cstdint>
#include <cstring>

struct TranslationResult;
struct IRInstr;
union IROperand;

namespace X87IR {

static constexpr int kMaxNodes = 64;

// ── IR opcodes ──────────────────────────────────────────────────────────────

enum class Op : uint8_t {
    // Pure values (no side effects)
    ReadSt,     // load initial stack slot (initial_depth in payload)
    LoadF64,    // load f64 from memory
    LoadF32,    // load f32 from memory, widen to f64
    LoadI16,    // load i16, sign-extend, convert to f64
    LoadI32,    // load i32, convert to f64
    LoadI64,    // load i64, convert to f64
    ConstZero,  // 0.0
    ConstOne,   // 1.0
    ConstF64,   // arbitrary f64 constant (imm_bits in payload)

    // Binary arithmetic
    FAdd,
    FSub,
    FMul,
    FDiv,

    // FMA: inputs[0]*inputs[1] + inputs[2]  (FMADD encoding)
    //       inputs[2] - inputs[0]*inputs[1]  (FMSUB encoding)
    //       inputs[0]*inputs[1] - inputs[2]  (FNMSUB encoding)
    FMAdd,
    FMSub,
    FNMSub,

    // Unary
    FNeg,
    FAbs,
    FSqrt,
    FRndInt,

    // Conditional select (FCMOV): inputs[0]=ST(0) false arm, inputs[1]=ST(i) true arm
    // imm_bits[3:0] = AArch64 condition code
    FCSel,

    // Memory stores (side effects — emitted in program order)
    StoreF64,  // store as f64 to memory
    StoreF32,  // narrow to f32, store
    StoreI16,  // convert f64 to signed i16, store
    StoreI32,  // convert f64 to signed i32, store
    StoreI64,  // convert f64 to signed i64, store

    // Compare (side effects — write CC to status_word)
    FCmp,  // compare two values, set C0/C2/C3
    FTst,  // compare value with zero, set C0/C2/C3

    // FCOMI family (side effects — write result directly to NZCV, not status_word)
    FComI,  // FCOMI/FUCOMI/FCOMIP/FUCOMIP: compare ST(0) vs ST(i), set NZCV
            // inputs[0]=ST(0) node, inputs[1]=ST(i) node
            // flags & kFcomIPopping: pop ST(0) after compare (FCOMIP/FUCOMIP)

    // Status word read
    FStsw,  // store status_word to AX (consumes CC from prior FCmp/FTst)

    // Control word
    StoreCW,  // FLDCW: load u16 from memory, write to X87State.control_word
    LoadCW,   // FNSTCW: read X87State.control_word, store u16 to memory
};

enum NodeFlags : uint8_t {
    kNone = 0,
    kDead = 1 << 0,          // eliminated by optimization pass
    kFcomFused = 1 << 1,     // FCOM+FSTSW fused: CC stays in register
    kTruncate = 1 << 2,      // StoreI*: always truncate (FISTTP), skip RC dispatch
    kFcomIPopping = 1 << 3,  // FComI: pop ST(0) after compare (FCOMIP/FUCOMIP)
};

// ── IR node ─────────────────────────────────────────────────────────────────

struct Node {
    Op op;
    uint8_t flags;
    int16_t inputs[3];  // SSA references (node indices); -1 = unused
    union {
        uint64_t imm_bits;       // ConstF64
        IROperand* mem_operand;  // Load*/Store*: pointer into original IRInstr
        int16_t initial_depth;   // ReadSt: depth relative to initial TOP
    };
};
static_assert(sizeof(Node) == 16, "X87IR::Node should be 16 bytes");

// ── IR context (all state for one run, lives on the stack) ──────────────────

struct Context {
    Node nodes[kMaxNodes];  // 1024 bytes
    int16_t num_nodes;

    // Symbolic stack.
    //   val >= 0: IR node ID
    //   val < 0:  initial slot; initial_depth = -(val + 1)
    int16_t slot_val[8];
    int16_t top_delta;  // accumulated push(-) / pop(+)

    // Dedup cache for initial-slot reads (avoids duplicate ReadSt nodes).
    int16_t initial_read[8];  // node ID for initial depth d, or -1

    // CC tracking for FCOM+FSTSW fusion.
    int16_t last_fcmp;  // most recent FCmp/FTst node ID, or -1

    // NZCV tracking for FCMOV safety gate.
    int16_t last_fcomi;  // most recent FComI node ID, or -1

    // How many x87 instructions were successfully consumed.
    int16_t consumed;

    void init() {
        num_nodes = 0;
        top_delta = 0;
        consumed = 0;
        last_fcmp = -1;
        last_fcomi = -1;
        for (int i = 0; i < 8; i++) {
            slot_val[i] = static_cast<int16_t>(-(i + 1));
            initial_read[i] = -1;
        }
    }

    // Append a new node and return its index.
    int16_t add_node(Op op, int16_t in0 = -1, int16_t in1 = -1, int16_t in2 = -1) {
        if (num_nodes >= kMaxNodes)
            return -1;
        int16_t id = num_nodes++;
        auto& n = nodes[id];
        n.op = op;
        n.flags = kNone;
        n.inputs[0] = in0;
        n.inputs[1] = in1;
        n.inputs[2] = in2;
        n.imm_bits = 0;
        return id;
    }

    // Resolve the current value at logical stack depth `d`.
    // If the slot still holds an initial value, emits a ReadSt node.
    // Returns node ID, or -1 on failure.
    int16_t resolve(int d) {
        if (d < 0 || d >= 8)
            return -1;
        int16_t val = slot_val[d];
        if (val >= 0)
            return val;

        int init_d = -(val + 1);
        if (init_d < 0 || init_d >= 8)
            return -1;

        // Dedup: reuse prior ReadSt for same initial depth
        if (initial_read[init_d] >= 0) {
            slot_val[d] = initial_read[init_d];
            return initial_read[init_d];
        }

        int16_t id = add_node(Op::ReadSt);
        if (id < 0)
            return -1;
        nodes[id].initial_depth = static_cast<int16_t>(init_d);
        initial_read[init_d] = id;
        slot_val[d] = id;
        return id;
    }

    void push(int16_t node_id) {
        for (int i = 7; i > 0; i--)
            slot_val[i] = slot_val[i - 1];
        slot_val[0] = node_id;
        top_delta--;
    }

    void pop() {
        for (int i = 0; i < 7; i++)
            slot_val[i] = slot_val[i + 1];
        slot_val[7] = -100;  // sentinel: "popped past end"
        top_delta++;
    }
};

// ── Public API ──────────────────────────────────────────────────────────────

// Build IR from a sequence of x87 instructions.
// Returns the number of instructions consumed (stored in ctx.consumed).
bool build(Context& ctx, IRInstr* instr_array, int64_t num_instrs, int64_t start_idx,
           int run_length);

// Run optimization passes on the IR (DSE, FMA detection, FCOM+FSTSW fusion).
void optimize(Context& ctx);

// Compute the peak number of simultaneously live FPR-bearing nodes that the
// lowering pass will require, accounting for transient temporaries (e.g. the
// +1 FPR spike during StoreF32 narrowing).  Used to gate lowering against the
// available scratch FPR pool.
int peak_live_fprs(const Context& ctx);

// Lower IR to AArch64 instructions, writing into result->insn_buf.
void lower(Context& ctx, TranslationResult* result);

// Reason compile_run returned 0 (i.e. IR couldn't lower the run).  Reported
// via the optional out_reason parameter so the X87_PROFILE path-tally can
// classify "single-op fall-through because IR failed for reason X" — useful
// for diagnosing why hot patterns sometimes stay on the slow path.
enum class IRFailReason : uint8_t {
    kNone = 0,         // success, or compile_run wasn't called
    kBuildFail = 1,    // build() returned false (kMaxNodes overflow, unhandled op, …)
    kFprPressure = 2,  // peak_live_fprs > available
    kGprPressure = 3,  // peak_live_gprs > available
};

// Main entry point: build + optimize + lower.
// Returns the number of x87 instructions consumed (0 = IR couldn't handle any).
// out_reason (optional): set to a non-kNone value when the return is 0,
// indicating which gate refused.  Untouched on success.
int compile_run(TranslationResult* result, IRInstr* instr_array, int64_t num_instrs,
                int64_t start_idx, int run_length, IRFailReason* out_reason = nullptr);

}  // namespace X87IR
