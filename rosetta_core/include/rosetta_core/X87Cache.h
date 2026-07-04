#pragma once

#include <cstdint>

struct IRBlock;
struct IRInstr;

// OPT-1: Cross-instruction x87 base/TOP register cache.
//
// When consecutive x87 instructions appear in a block, the base address
// (X18 + x87_state_offset) and the TOP field never change between instructions
// except through our own push/pop (which update the register in-place).
// Caching these two values across instructions saves 3-4 emitted AArch64
// instructions per x87 opcode after the first in a run.
struct X87Cache {
    int8_t base_gpr = 0;        // GPR holding X87State base
    int8_t top_gpr = 0;         // GPR holding TOP
    int16_t run_remaining = 0;  // Countdown; 0 = inactive
    int8_t st_base_gpr = 0;     // GPR holding &st[0] = Xbase + kX87RegFileOff
    int8_t top_dirty = 0;       // OPT-C: 1 = push skipped store_top, TOP in memory stale
    int8_t gprs_valid = 0;      // 1 = base/top GPR numbers are meaningful
    int8_t st_base_valid = 0;   // 1 = st_base_gpr is currently pinned & meaningful.
                                // Independent of gprs_valid so that X87IRLower's
                                // epilogue can free Xst_base before the tag-batch
                                // alloc (drops peak GPR by 1) while keeping
                                // base_gpr/top_gpr pinned for the run's tail.
    int8_t tag_push_pending =
        0;  // OPT-D: 1 = push's tag-valid update deferred (cancel on next pop)
    int8_t deferred_pop_count =
        0;  // OPT-D2: number of pop tag-set-empty updates deferred to run end

    // OPT-G: Deferred FXCH — compile-time register renaming.
    // perm[i] maps logical stack depth i to physical depth offset.
    // Identity: perm[i] == i for all i.  FXCH ST(n) swaps perm[0] and perm[n].
    // Flushed at run end by emitting the minimal memory swaps (cycle decomposition).
    int8_t perm[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    int8_t perm_dirty = 0;  // 1 = permutation is non-identity

    // Run bridging: descriptor stashed at a fresh run start when
    // lookahead_bridged found a profitable region.  bridge_pending_idx is
    // the region's start instruction (the attempt fires only when the IR
    // dispatch reaches exactly that index with no deferred state);
    // bridge_pending_plain is the plain x87 run length to restore when the
    // all-or-nothing attempt falls back.
    int16_t bridge_pending_total = 0;
    int16_t bridge_pending_x87 = 0;
    int16_t bridge_pending_plain = 0;
    int16_t bridge_pending_idx = -1;

    IRBlock* prev_block = nullptr;

    // X87_PROFILE — per-block translation-path tally.  Reset on block
    // transition.  Each increment is mirrored to profile::set_block_tally
    // so the dumped value at exit reflects the final count, even though
    // the block's translate_instruction calls happen one at a time.
    // profile_bid is kOverflowId when profiling is disabled or the block
    // exceeded kMaxBlocks; in either case the bump-and-mirror is skipped.
    uint16_t tally_ir = 0;
    uint16_t tally_peep = 0;
    uint16_t tally_single = 0;
    uint16_t tally_ft = 0;
    // IR-failure reason classifiers.  Bumped per translate_instruction call
    // when the IR gate would have fired but compile_run returned 0.  Disjoint
    // from tally_single (a fall-through from IR failure also bumps single).
    uint16_t tally_ir_build_fail = 0;
    uint16_t tally_ir_fpr_fail = 0;
    uint16_t tally_ir_gpr_fail = 0;
    // Runs compile_run rescued by splitting at a register-pressure peak
    // (i.e. a would-have-been fpr/gpr refusal that lowered as a shorter
    // prefix instead).  Bumped inside compile_run itself — per-TR, so
    // per-thread-safe like the rest of the tallies.
    uint16_t tally_ir_split = 0;
    // Runs where the remat/sink pass changed the IR to relieve FPR
    // pressure before (or instead of) a split.
    uint16_t tally_ir_remat = 0;
    // Run bridging: bridge instructions consumed inside IR runs / bridged
    // compile_run attempts that fell back to plain dispatch.
    uint16_t tally_bridge = 0;
    uint16_t tally_bridge_fail = 0;
    // Max peak_live_gprs(ctx) observed across compile_run attempts in this
    // block.  Saturating at its u16 upper bound is a sufficient signal — we
    // only care about "was peak high enough to refuse?".
    uint16_t tally_max_gpr_peak = 0;
    // Per-reason IR-gate refusal counters.  Bumped each time the dispatcher
    // gate refuses entry to compile_run before checking; one counter per
    // condition.  Disjoint from tally_ir_*_fail (those count failures
    // *after* compile_run was called).  Per-reason counts (rather than a
    // single first/last-write-wins sentinel) avoid trailing-tail short_run
    // records masking a longer run's actual cause.
    uint16_t tally_ir_gate_short_run = 0;
    uint16_t tally_ir_gate_top_dirty = 0;
    uint16_t tally_ir_gate_tag_push = 0;
    uint16_t tally_ir_gate_deferred_pop = 0;
    uint16_t tally_ir_gate_perm_dirty = 0;
    // Per-reason MAX of cache.run_remaining observed at any gate refusal in
    // this block.  Distinguishes "real long-run refusal" (max_run large,
    // e.g. 14) from "trailing-tail refusal" (max_run small, e.g. 2 — the
    // dispatcher peeling one op at a time after an earlier IR failure).
    // Indexed by kIRGateReason*; surfaces as the max_run column of the
    // IR-gate-refusal histogram.
    uint16_t max_run_at_gate[5] = {0, 0, 0, 0, 0};
    // 1 = the current block contains a control-word writer (FLDCW/FLDENV/
    // FRSTOR/FXRSTOR/FINIT/FSAVE).  Scanned once per block transition when
    // X87_FAST_ROUND=2; consumed by x87_fast_round_active to keep the full
    // RC dispatch in exactly those blocks.
    int8_t block_has_cw_write = 0;
    // Last x87 opcode (kOpcodeName_*) translated in this block, or 0xFFFF
    // if none yet.  Used by the IR-gate top_dirty diagnostic to attribute
    // which preceding op left top_dirty=1 — surfaces in the analyzer's
    // "Top opcodes preceding top_dirty refusal" histogram.
    uint16_t prev_x87_opcode = 0xFFFFU;
    uint32_t profile_bid = 0xFFFFFFFFU;  // = profile::kOverflowId
    // FNV-1a IR-content hash, stable across runs (PC zeroed before hashing).
    // Populated unconditionally on block transition so the X87_*_HASH_LIST
    // rollback gate works even when X87_PROFILE is off.
    uint64_t profile_hash = 0;

    bool active() const;
    void invalidate();
    void invalidate(uint32_t& free_gpr_mask, uint32_t scratch_mask);
    void set_run(int run_length);
    void tick();
    uint32_t pinned_mask() const;

    // OPT-G: permutation helpers
    void reset_perm();
    bool perm_is_identity() const;

    // Scan forward from insn_idx counting consecutive handled x87 instructions.
    static int lookahead(IRInstr* instr_array, int64_t num_instrs, int64_t insn_idx);

    // Run bridging: a region descriptor for one IR run that spans short gaps
    // of v1 bridge instructions (X87Bridge.h) between x87 segments.
    //   total       instructions in the region (x87 + bridges); 0 = no
    //               profitable bridged region (fall back to plain dispatch)
    //   x87_count   x87 instructions within it
    //   bridges     bridge instructions within it (== total - x87_count)
    // The region always starts and ends on an x87 instruction (trailing
    // bridges are trimmed — they join nothing), contains at least one join,
    // and satisfies x87_count >= 3 so the merged run clears the IR gate.
    struct BridgedRun {
        int16_t total = 0;
        int16_t x87_count = 0;
        int16_t bridges = 0;
    };
    static BridgedRun lookahead_bridged(IRInstr* instr_array, int64_t num_instrs,
                                        int64_t insn_idx, int max_gap, int max_total_bridges);
};
