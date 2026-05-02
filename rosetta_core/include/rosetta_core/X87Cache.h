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
    int8_t gprs_valid = 0;      // 1 = base/top/st_base GPR numbers are meaningful
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
    uint32_t profile_bid = 0xFFFFFFFFU;  // = profile::kOverflowId

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
};
