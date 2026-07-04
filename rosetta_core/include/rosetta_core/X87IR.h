#pragma once

#include <cstdint>
#include <cstring>

struct TranslationResult;
struct IRInstr;
union IROperand;

namespace X87IR {

// Per-Context cap on IR nodes.  Bumped 64 → 128 on 2026-05-03 after the
// fail-reason instrumentation showed the L=12 fst chain block overflowing
// at the old cap (chain + FXCH + repeated segments routinely exceed 64).
// Cost: 64 × sizeof(Node)=16 B = 1 KB extra per Context allocation; plus
// X87IRLower's per-Context arrays (node_fpr, last_use, holding) grow by
// the same ratio.  All Context lives on the stack and is short-lived.
static constexpr int kMaxNodes = 128;

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

    // Transcendentals (side-effect-free; lowered via emit_inline_<op>_core).
    // Each lowering reserves +1 transient FPR for the result and +1 transient
    // GPR for the constants table pointer (Xconst); see peak_live_fprs /
    // peak_live_gprs in X87IRLower.cpp.  fsincos is NOT a single op — build()
    // models it as two FSin + FCos nodes sharing the input.
    FSin,    // 1 in, 1 out: sin(in0)
    FCos,    // 1 in, 1 out: cos(in0)
    FPatan,  // 2 in, 1 out: atan2(in0, in1)
    FPtan,   // 1 in, 1 out: tan(in0); caller pushes ConstOne for the +1.0
    FYl2x,   // 2 in, 1 out: in0 * log2(in1)
    FScale,  // 2 in, 1 out: in0 * 2^trunc(in1); ST(1) (=in1) unchanged in slot

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

    // ── Bridge ops (run bridging v1) ────────────────────────────────────────
    // Non-x87 integer instructions carried inside an x87 IR run so the FP
    // stack stays in FPRs across short mov/lea gaps (see X87Bridge.h for the
    // eligibility predicate).  All are side-effect nodes (guest register
    // and/or memory writes) emitted strictly in program order; none touches
    // NZCV or produces an FPR value.  Guest register numbers are encoded
    // NEGATIVELY in inputs[] via bridge_encode_reg so no pass walk can
    // mistake them for SSA node references.  Operand width (32/64-bit) is
    // carried by the kBridge64 flag; W-register writes zero-extend, which is
    // exactly x86-64 semantics.
    BridgeMovRR,   // mov r,r      inputs[0]=enc(dst), inputs[1]=enc(src)
    BridgeMovRI,   // mov r,imm    inputs[0]=enc(dst), imm_bits=value
    BridgeLea,     // lea r,[m]    inputs[0]=enc(dst), mem_operand=&src
    BridgeLoadG,   // mov r,[m]    inputs[0]=enc(dst), mem_operand=&src
    BridgeStoreG,  // mov [m],r    inputs[0]=enc(src), mem_operand=&dst

    // ── Bridge ALU ops (run bridging v2) ────────────────────────────────────
    // Flag-writing integer ALU (add/sub/and/or/xor, plus inc/dec folded to
    // add/sub 1) carried inside a run ONLY when Rosetta's own liveness byte
    // proves the written flags dead (IRInstr::flag_liveness == 0; see
    // X87Bridge.h).  Lowered to NON-flag-setting ARM (ADD, not ADDS) so
    // NZCV — which may carry x86 flags live THROUGH the region, e.g. a
    // cmp's CF surviving an inc/dec — is never clobbered.  The ALU kind
    // (BridgeAluKind) rides negatively encoded in inputs[2], same guard as
    // guest registers.
    BridgeAluRR,  // alu r,r    inputs[0]=enc(dst), inputs[1]=enc(src), inputs[2]=enc(kind)
    BridgeAluRI,  // alu r,imm  inputs[0]=enc(dst), inputs[2]=enc(kind), imm_bits=value
    BridgeAluRM,  // alu r,[m]  inputs[0]=enc(dst), inputs[2]=enc(kind), mem_operand=&src
    BridgeAluMR,  // alu [m],r  inputs[0]=enc(src), inputs[2]=enc(kind), mem_operand=&dst
};

// ALU selector for BridgeAlu* nodes; stored in inputs[2] via
// bridge_encode_reg so pass walks' `input >= 0` guards skip it.
enum BridgeAluKind : uint8_t {
    kBridgeAluAdd = 0,
    kBridgeAluSub,
    kBridgeAluAnd,
    kBridgeAluOr,
    kBridgeAluXor,
};

enum NodeFlags : uint8_t {
    kNone = 0,
    kDead = 1 << 0,          // eliminated by optimization pass
    kFcomFused = 1 << 1,     // FCOM+FSTSW fused: CC stays in register
    kTruncate = 1 << 2,      // StoreI*: always truncate (FISTTP), skip RC dispatch
    kFcomIPopping = 1 << 3,  // FComI: pop ST(0) after compare (FCOMIP/FUCOMIP)

    // FMA-reduction chain tagging (set by pass_fma_reduce, consumed by lower
    // and peak_live_fprs).  Recognises serial reductions of the form
    //   A_{i+1} = FMAdd(L_i, W_i, A_i)
    // where L_i / W_i are LoadF32s with simple (base+disp) operands and the
    // two streams are independently +4-contiguous, so the body can be
    // vectorised into LDR D + FCVTL .2D + FMLA .2D pairs ending with a
    // scalar FADDP horizontal sum.
    kFmaReduceHead = 1 << 4,    // First FMAdd of a chain: lower emits whole body
    kFmaReduceMember = 1 << 5,  // FMAdd / LoadF32 absorbed by a chain (incl. head)

    kBridge64 = 1 << 6,  // Bridge*: 64-bit operand width (default 32-bit)
};

// Guest GPR numbers inside Bridge* nodes' inputs[] — negative so every pass
// walk's `input >= 0` guard skips them (they are not node references).
// -1 stays "unused"; encodings start at -2.
inline int16_t bridge_encode_reg(int guest_reg) {
    return static_cast<int16_t>(-(guest_reg + 2));
}
inline int bridge_decode_reg(int16_t enc) {
    return -(enc + 2);
}

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

    // Ordinal (0-based, within the run) of the x87 instruction that created
    // each node.  Written by build() for every node it appends; lets
    // compile_run map a pressure-overflow node back to an instruction
    // boundary when it splits an over-pressure run into a fitting prefix.
    int16_t node_src[kMaxNodes];

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

    // x87 opcode (kOpcodeName_*) at which build()'s default arm bailed on this
    // run, or 0xFFFF if no bail occurred. Diagnostic for the build-blocking-
    // opcode histogram. Even on success (consumed >= 2) this can be set when
    // the loop stopped at an unsupported opcode at position N.
    uint16_t fail_opcode;

    void init() {
        num_nodes = 0;
        top_delta = 0;
        consumed = 0;
        last_fcmp = -1;
        last_fcomi = -1;
        fail_opcode = 0xFFFFU;
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
// allow_bridges: also consume v1 bridge instructions (X87Bridge.h) between
// x87 ops, emitting Bridge* side-effect nodes at their program position.
// bridges_v2: additionally consume flag-dead ALU bridges (proof re-checked
// per instruction via x87bridge::is_bridge_v2_proven; the block-level
// flag-liveness-populated gate is the lookahead's job).
bool build(Context& ctx, IRInstr* instr_array, int64_t num_instrs, int64_t start_idx,
           int run_length, bool allow_bridges = false, bool bridges_v2 = false);

// Run optimization passes on the IR (DSE, FMA detection, FCOM+FSTSW fusion).
void optimize(Context& ctx);

// pass_fma_reduce diagnostics.  Atomic counters incremented as the pass
// walks a Context, plus a reject-reason histogram for chains the pass
// considered but didn't tag.  Survey-style instrumentation for the
// optimization toolkit — useful when measuring how an optimization
// covers a real workload vs. synthetic benches.  Mirror this shape for
// future passes (e.g. attempted-fusion counters).
//
// Counters are global atomics because translate_instruction is called
// from worker threads; tools/profile_analyze invokes it serially but the
// runtime sidecar may not.
struct FmaReduceStats {
    uint64_t invocations;         // pass_fma_reduce called this many times
    uint64_t candidates_seen;     // FMAdd nodes considered as chain heads
    uint64_t chains_tagged;       // chains successfully tagged for vector lowering
    uint64_t rejected_short;      // chain length < 2 (lone FMAdd)
    uint64_t rejected_load_kind;  // input[0] or input[1] not LoadF32
    uint64_t rejected_load_use;   // L_i / W_i is multi-use or already absorbed
    uint64_t rejected_mem_shape;  // mem_operand has index reg or seg override
    uint64_t rejected_stride;     // stream isn't +4 contiguous (e.g. WoW's
                                  // stride-16 matrix-vector idiom)
};

// Snapshot the current counter values.  Cheap (relaxed loads).
FmaReduceStats fma_reduce_stats();

// Print a one-line summary of fma_reduce_stats() to stdout.  Called by
// profile_analyze when X87_LOG_FMA_REDUCE=1 is set; no-op otherwise.
void fma_reduce_print_stats();

// Compute the peak number of simultaneously live FPR-bearing nodes that the
// lowering pass will require, accounting for transient temporaries (e.g. the
// +1/+2 FPR spike during StoreF32 narrowing).  Used to gate lowering against
// the available scratch FPR pool.
//
// budget / first_over_node: when first_over_node is non-null, it is set to
// the index of the first node whose modeled demand exceeds `budget` (-1 if
// demand never exceeds it).  compile_run uses ctx.node_src[*first_over_node]
// to pick the instruction boundary at which to split an over-pressure run.
int peak_live_fprs(const Context& ctx, int budget = 0x7FFFFFFF, int* first_over_node = nullptr);

// GPR-side equivalent of peak_live_fprs (pinned + held + per-node transient
// demand model; see X87IRLower.cpp for the model notes).  fast_round must
// match the effective fast-round decision the lowering will make
// (x87_fast_round_active) so the RC-cache pinned-GPR prediction agrees.
int peak_live_gprs(const Context& ctx, int budget = 0x7FFFFFFF, int* first_over_node = nullptr,
                   bool fast_round = false);

// Lower IR to AArch64 instructions, writing into result->insn_buf.
void lower(Context& ctx, TranslationResult* result);

// Reason compile_run returned 0 (i.e. IR couldn't lower the run).  Reported
// via the optional out_reason parameter so the X87_PROFILE path-tally can
// classify "single-op fall-through because IR failed for reason X" — useful
// for diagnosing why hot patterns sometimes stay on the slow path.
enum class IRFailReason : uint8_t {
    kNone = 0,           // success, or compile_run wasn't called
    kBuildFail = 1,      // build() returned false (kMaxNodes overflow, unhandled op, …)
    kFprPressure = 2,    // peak_live_fprs > available
    kGprPressure = 3,    // peak_live_gprs > available
    kBridgePartial = 4,  // bridged (require_full) run didn't consume the whole
                         // region — nothing was emitted; caller falls back to
                         // the plain (x87-only) dispatch
};

// Main entry point: build + optimize + lower.
// Returns the number of x87 instructions consumed (0 = IR couldn't handle any).
// out_reason (optional): set to a non-kNone value when the return is 0,
// indicating which gate refused.  Untouched on success.
// out_peak_gprs (optional): set to peak_live_gprs(ctx) whenever build()
// succeeded.  Set to 0 when build() failed (no ctx to query).  Useful for
// diagnosing why compile_run keeps refusing on hot blocks — saturating
// against the available pool tells us we need to either reduce IR pressure
// or accept the fall-through.
// out_fail_opcode (optional): set to the kOpcodeName_* value at which build()'s
// default arm bailed.  Set whenever build observed an unsupported opcode, even
// when compile_run returned a positive consumed (i.e. build succeeded on the
// prefix but stopped early at position N).  Untouched (caller-initialized
// sentinel preserved) when no bail was observed.
// bridged (optional): all-or-nothing bridged mode — build() accepts v1
// bridge instructions, pressure splitting is disabled (a partial consume
// would hand the run tail's bridges back to stock mid-run, which may
// clobber pinned scratch GPRs), and any outcome other than consuming
// exactly run_length returns 0 with kBridgePartial (nothing emitted; the
// gates run before lower()).
// bridges_v2 (optional): with bridged, also accept flag-dead ALU bridges.
int compile_run(TranslationResult* result, IRInstr* instr_array, int64_t num_instrs,
                int64_t start_idx, int run_length, IRFailReason* out_reason = nullptr,
                int* out_peak_gprs = nullptr, uint16_t* out_fail_opcode = nullptr,
                bool bridged = false, bool bridges_v2 = false);

}  // namespace X87IR
