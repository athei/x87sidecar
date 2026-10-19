#!/usr/bin/env bash
#
# run_tests.sh -- run all x87 test binaries under native Rosetta and the
# custom x87sidecar, checking self-reported PASS/FAIL output.
#
# Phases:
#   1. Native Rosetta (baseline)
#   2. x87sidecar (IR + fusions enabled, default config)
#   3. x87sidecar with X87_DISABLE_X87_IR=1 (direct translator only).
#      Skips EXACT_I64_COPY_SKIP: without the IR, a fild m64 to fistp m64
#      copy in a run that also holds a float op such as fstpl rounds
#      through binary64; only with X87_NATIVE_SLOTS=0, since the native
#      slots copy it exactly.
#   4. x87sidecar with X87_DISABLE_X87_IR=1 + X87_DISABLE_ALL_FUSIONS=1.
#      Skips EXACT_I64_COPY_SKIP, as phase 3, and the tests in
#      NO_FUSION_SKIP, whose exact results need a fusion.
#   5. x87sidecar with X87_DISABLE_HOOK=1 (stock translate_insn only)
#   6. x87sidecar with X87_ENABLE_FMA_CONTRACT=1 (FMA contraction is off by
#      default; this phase keeps the IR FMA nodes, the FMA-reduce lowering
#      and the fusion FMA paths under test).  Skips test_fma_pc53, which
#      asserts the unfused numerics.
#   7. x87sidecar with X87_FPR_POOL_LIMIT=5 (deterministic pressure
#      splitting / remat-sink exercise on every test)
#   8. x87sidecar with X87_ENABLE_IR_SPLIT=0 X87_ENABLE_IR_REMAT=0
#      (legacy all-or-nothing pressure gate)
#   9. x87sidecar with X87_FAST_ROUND=2 on a curated same-block-FLDCW set
#      (cross-block RC is mis-rounded by design under =2)
#  10. x87sidecar with X87_ENABLE_BRIDGE=0 (bridging defaults ON; this
#      phase keeps the unbridged dispatch under regression test)
#  11. x87sidecar with X87_BRIDGE_V2=0 (v2 defaults ON; this phase keeps
#      the v1-only bridged dispatch under regression test)
#  12. test_decoder_fcomp_st and test_decoder_arpl, three ways each: both
#      must trap under native Rosetta, pass under x87sidecar (the
#      decode_opcode hook substitutes a decodable encoding), and trap again
#      under X87_NO_DECODE_HOOK=1
#  13. x87sidecar with X87_STOCK_BCD=0 on test_fbld, test_fbstp and
#      test_flags_handoff, the only phase that reaches the inline FBLD/FBSTP
#      translators (blocks containing them go to stock by default)
#  14. a representative set under X87_FAIL_ALLOC_EVERY=N for several N,
#      without X87_STRICT_ALLOC: requests whose register allocation fails
#      are declined to stock, and every result must still match
#  15. test_x87_ftz_daz: passes under native Rosetta and under x87sidecar
#      with X87_IGNORE_FTZ_DAZ=1 (default, IR off, fusions off), and must
#      still fail under x87sidecar without the knob; test_flags_handoff must
#      pass with the knob
#  16. test_flags_handoff with X87_STOCK_OPS handing the fcomi family, then
#      the fcmov family, to stock, with the IR on and off: guest flags set
#      by one translator and read by the other.  Each run must resolve every
#      name and hand at least one block to stock
#  17. test_x87_sync_fault: passes under native Rosetta and under x87sidecar
#      with X87_PRECISE_FAULTS=1 (default, IR off, fusions off, run cache
#      off, clamped register pool, bridging off, X87_STOCK_BCD=0 and
#      X87_FAIL_ALLOC_EVERY=N), with its in-run group too under
#      X87_PRECISE_FAULTS=2 X87_NATIVE_SLOTS=1, and one of its faults must
#      still abort the process without the knob, under both the cooperative
#      and the default attach.  Its wc_cross case, an fstpt across a page
#      boundary of uncached memory, must store without a signal natively
#      and without the knob, and take the probe's SIGBUS with it
#  18. test_crash_after_release: a target that is no longer traced after
#      the release and then dies (abort, SIGSEGV, SIGKILL, SIGILL, exit(3))
#      must end with its own exit status natively and under both attaches
#  19. test_x87_cross_reply_exact with X87_NATIVE_SLOTS=1 (default, IR off,
#      fusions off, run cache off, single-op fast path off, clamped
#      register pool, bridging off, X87_PRECISE_FAULTS=1,
#      X87_FAIL_ALLOC_EVERY=N, and X87_GPR_POOL_LIMIT=4, where integer
#      stores fall back to binary64), then the tests that read or replace the
#      native x87 state with it.  The knob is on by default, so this covers
#      it also in a run that exports X87_NATIVE_SLOTS=0
#  20. every test with X87_NATIVE_SLOTS=0, the kill switch, where
#      test_x87_cross_reply_exact has to fail; exporting X87_NATIVE_SLOTS=0
#      for the whole run makes every phase run that way
#   H. test_opcode_host_table, natively: the name-derived opcode maps
#      against the pinned 26.4 table and the identity mapping; and
#      test_opcode_table_parse: the "index < N" bound on synthetic blobs
#   R. replay tests/data/geom_block_874c40.ir under --fpr-pool 8 and
#      assert the pressure splits keep it on the IR path (fpr_fail=0), then
#      replay tests/data/fist_pressure.ir the same way and assert it exits 0
#      with one split per block and no GPR refusals, then
#      tests/data/ir_dup_input.ir and assert it exits 0 with every x87
#      instruction on the IR path, then tests/data/ir_fpatan_rc.ir and
#      assert it exits 0 with the fpatan's GPR peak predicted
#
# test_x87_sync_fault runs only in phase 17, since without X87_PRECISE_FAULTS
# every one of its faults aborts the process.  Even with the knob it runs
# only the faults the sidecar recovers precisely; the ones inside longer
# floating-point runs still abort, and SYNC_FAULT_IN_RUN=1 adds them; phase
# 17 runs them with X87_PRECISE_FAULTS=2, where they pass.  Naming it,
# `run_tests.sh test_x87_sync_fault`, runs it through the per-test phases
# without the knob, where it reports CRASH from phase 2 on, and skips phase
# 17.  Run it as
# `X87_PRECISE_FAULTS=1 bash scripts/run_tests.sh test_x87_sync_fault`, or run
# the full suite.
#
# Every run of the loader is bounded by TEST_TIMEOUT; a run that does not
# finish in time is killed and reported as a failure of that test.
#
# Every phase runs with X87_STRICT_ALLOC=1, so a register allocation that
# fails asserts and crashes the test instead of being declined to stock.
# The decline keeps a program running in the field; here a pressure-model
# bug has to show up as a failure.  Phase 14 alone turns it off.
#
# Phase 1 doubles as the baseline the other phases are read against.  A
# case that fails there fails with the sidecar nowhere in the picture, so
# it is stock Rosetta's and none of our phases can make it pass; a later
# phase failing only cases Phase 1 also failed is reported XFAIL and does
# not fail the run, while any case Phase 1 passed still does.  Phase 1
# itself only gets that latitude for the tests named in
# KNOWN_STOCK_DIVERGENCE below.
#
# Usage:
#   bash scripts/run_tests.sh                # build + test (all phases)
#   bash scripts/run_tests.sh --no-build     # skip build
#   bash scripts/run_tests.sh --native-only  # Phase 1 only (no x87sidecar)
#   bash scripts/run_tests.sh test_arith     # only run specific test(s)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"
BIN="$BUILD_DIR/bin"
# Default-attach phases use task_for_pid + ptrace, which need the entitled
# build (cs.debugger + get-task-allow).  The flat `x87sidecar` ships without
# entitlements and only works in cooperative mode.
LOADER="$BIN/x87sidecar_entitled"
TESTS_BIN="$BIN/tests"

ALL_TESTS=(
    test_fldconst
    test_detach_signals
    test_fld
    test_fld_m80fp
    test_fmul
    test_fild
    test_fcom
    test_fcomp_mem
    test_peephole
    test_arith
    test_compare_unary
    test_fld_fst
    test_fst_chain_compose
    test_peephole3
    test_peephole4
    test_peephole5
    test_peephole6
    test_deep_stack
    test_single_op
    test_x87_full
    test_fstpt
    test_fxam
    test_peephole7
    test_fcom_nzcv
    test_arithp_fstp
    test_arith_faddp
    test_fstp_arith_fstp
    test_fld_arith
    test_fstp_fld
    test_fistt
    test_tag_batch
    test_ir_split
    test_ir_remat
    test_ir_split_trans
    test_ir_dup_input
    test_ir_fpatan_rc
    test_ir_split_gs
    test_bridge_mov
    test_bridge_lea
    test_bridge_flags
    test_bridge_alu
    test_bridge_ext
    test_fld_arith_arithp_fma
    test_readst_elide
    test_fxch
    test_fxch_initial
    test_fldcw
    test_fcomi
    test_frndint
    test_fistp_multi
    test_fist_indefinite
    test_fist_pressure
    test_fild_fistp_copy
    test_delphi_move
    test_fcmov
    test_flags_handoff
    test_ficom
    test_rc_recache
    test_fstpt_gs
    test_mem_bare_reg
    test_x87_gs_operand
    test_ir_gate_tag_push
    test_fstsw_pending_top
    test_fma_reduce
    test_fma_reduce_strided
    test_fma_pc53
    test_fbld
    test_fclex
    test_fdecstp
    test_fincstp
    test_ffree
    test_fdisi_feni
    test_fxtract
    test_fscale
    test_finit
    test_fbstp
    test_fldenv
    test_fstenv
    test_fclex_compose
    test_finit_compose
    test_fldenv_compose
    test_fstenv_compose
    test_frstor
    test_fsave
    test_fxrstor
    test_fxsave
    test_fsin
    test_x87_loop
    test_flags_across_x87
    test_flags_across_vex
    test_fcos
    test_f2xm1
    test_fpatan
    test_fsincos
    test_fptan
    test_fyl2x
    test_fyl2xp1
    test_fprem
    test_fprem1
    test_x87_signal_storm
    test_x87_signal_context
    test_x87_native_state
    test_m80_copy
    test_x87_cross_reply_exact
    test_bcd_exact
    test_alloc_decline
)

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

DO_BUILD=1
NATIVE_ONLY=0
SELECTED_TESTS=()
for arg in "$@"; do
    if [[ "$arg" == "--no-build" ]]; then
        DO_BUILD=0
    elif [[ "$arg" == "--native-only" ]]; then
        NATIVE_ONLY=1
    else
        SELECTED_TESTS+=("$arg")
    fi
done

if [[ ${#SELECTED_TESTS[@]} -gt 0 ]]; then
    TESTS=("${SELECTED_TESTS[@]}")
else
    TESTS=("${ALL_TESTS[@]}")
fi

export X87_STRICT_ALLOC=1

if [[ $DO_BUILD -eq 1 ]]; then
    echo -e "${CYAN}Building...${NC}"
    cmake --build "$BUILD_DIR" --parallel 2>&1 | tail -3
    echo ""
fi

# Strip runtime-internal noise lines before checking output
filter_runtime_lines() {
    # The sidecar announces X87_FAIL_ALLOC_EVERY on stdout, and the knob's
    # name would otherwise read as a FAIL line in check_output.
    # grep exits 1 when it prints nothing.  Report success then, so that with
    # pipefail the pipeline carries the run's own status (124 for a timeout)
    # instead of grep's.
    grep -v -E 'RosettaRuntimex87 built|Installing JIT|JIT Translation Hook|try_fuse_|CORE_LOG|\] X87_FAIL_ALLOC_EVERY=' ||
        [[ $? -eq 1 ]]
}

# Every run of the loader goes through run_bounded, so a target or sidecar
# that never exits is reported as a failure instead of stalling the harness.
# shellcheck source=scripts/run_bounded.sh
source "$SCRIPT_DIR/run_bounded.sh"

# Upper bound in seconds for one run of the loader.  The slowest test takes
# about 6 s on an M-series Mac, so this only ever fires on a hang.  It has to
# stay below the 90 s of silence after which scripts/diagnose_ci.py stops a
# CI run, or a hang there is never reported against its test.  Exported for
# test_x87_trace.sh, which bounds its own loader runs.
export TEST_TIMEOUT="${TEST_TIMEOUT:-60}"

TOTAL=0
PASSED=0
FAILED=0
XFAILED=0
ERRORS=0

# ── Stock-divergence baseline ────────────────────────────────────────────
# Phase 1 is native Rosetta with the sidecar nowhere in the picture, so a
# case that fails there is stock's and no phase of ours can make it pass.
# Its FAIL lines are recorded per test and every later phase is read
# against them: a failure whose lines all appear in the baseline is
# reported XFAIL and does not gate, while a line the baseline does not
# have is ours and fails as before.  Whole lines are compared, values
# included, so the same case failing differently under the sidecar still
# gates.
#
# Phase 1 itself has to allow a divergence by name.  Only the tests listed
# here may fail there without failing the run; anything else is a broken
# test or a broken machine and should stop the run as it always did.
KNOWN_STOCK_DIVERGENCE=(
    # A `sub`'s CF read wrong by a later `sbb` across a VEX/BMI2 filling,
    # which is the field case the test was written to pin rather than
    # something it asserts is fixed.  Reproduces on the macos-26-arm64 CI
    # runner in every phase including native, and on no M5 Max seen so
    # far, so it is stock Rosetta's and it is hardware-dependent.
    test_flags_across_vex
    # Stock Rosetta shifts the x87 stack by one when an asynchronous signal
    # lands in its translation of fcomp, fcompp or ficomp (the compare-and-pop
    # forms; the pop is lost or doubled).  The storm test reproduces it on
    # every run natively and in the phases that hand these ops to stock.
    test_x87_signal_storm
)

STOCK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/x87sidecar-stock.XXXXXX")"
trap 'rm -rf "$STOCK_DIR"' EXIT

# 1 only while Phase 1 runs; every later phase reads what it recorded.
BASELINE_PHASE=0

# Tests whose exact results need X87_NATIVE_SLOTS, which is on by default.
# With X87_NATIVE_SLOTS=0 every phase in which the sidecar translates them
# has to see them fail with a failure count: a pass would mean the test no
# longer reaches what the knob fixes.  Phase 20 runs them that way.
NATIVE_SLOTS_TESTS=(
    test_x87_cross_reply_exact
)
# 1 while Phase 5 runs the tests with the sidecar's translation hook off.
STOCK_EMIT_PHASE=0

# Whether this harness run has the native slots off (X87_NATIVE_SLOTS=0,
# the kill switch; any other value or none keeps them on, as in the sidecar).
native_slots_off() {
    [[ "${X87_NATIVE_SLOTS-}" == 0 ]]
}

needs_native_slots() {
    local t
    for t in "${NATIVE_SLOTS_TESTS[@]}"; do
        [[ "$t" == "$1" ]] && return 0
    done
    return 1
}

fail_lines() {
    grep -E 'FAIL' <<<"$1" | sed 's/^[[:space:]]*//; s/[[:space:]]*$//'
}

is_known_divergence() {
    local t
    for t in "${KNOWN_STOCK_DIVERGENCE[@]}"; do
        [[ "$t" == "$1" ]] && return 0
    done
    return 1
}

check_output() {
    local name="$1"
    local out="$2"
    local exit_code="$3"
    TOTAL=$((TOTAL + 1))
    if [[ "$exit_code" -eq 124 ]]; then
        echo -e "${RED}FAIL${NC}  $name  (timed out after $TEST_TIMEOUT s)"
        FAILED=$((FAILED + 1))
        tail -5 <<<"$out" | sed 's/^/      /'
        return 0
    fi
    if [[ $BASELINE_PHASE -eq 0 && $STOCK_EMIT_PHASE -eq 0 ]] && native_slots_off &&
        needs_native_slots "$name"; then
        if [[ "$exit_code" -eq 1 ]] && grep -qE '^[1-9][0-9]* failure' <<<"$out"; then
            echo -e "${GREEN}PASS${NC}  $name  (fails without X87_NATIVE_SLOTS, as expected)"
            PASSED=$((PASSED + 1))
        else
            local wanted="failures without X87_NATIVE_SLOTS"
            echo -e "${RED}FAIL${NC}  $name  (wanted $wanted, got exit=$exit_code)"
            FAILED=$((FAILED + 1))
            tail -3 <<<"$out" | sed 's/^/      /'
        fi
        return 0
    fi
    # Herestrings, not `echo "$out" | grep`: when $out exceeds the pipe
    # buffer, `grep -q` exits at the first match and echo takes EPIPE, which
    # pipefail turns into a failed pipeline — a passing test then gets
    # reported NO-PASS (seen as the flaky CI failure). Same reason the FAIL
    # excerpt uses awk instead of `grep | head`.
    if grep -qE 'FAIL' <<<"$out"; then
        local lines new count
        lines=$(fail_lines "$out")
        count=$(wc -l <<<"$lines" | tr -d ' ')
        [[ "$count" -eq 1 ]] && count="1 case" || count="$count cases"
        if [[ $BASELINE_PHASE -eq 1 ]]; then
            printf '%s\n' "$lines" >"$STOCK_DIR/$name"
            if is_known_divergence "$name"; then
                echo -e "${YELLOW}XFAIL${NC} $name  (stock divergence, $count)"
                XFAILED=$((XFAILED + 1))
            else
                echo -e "${RED}FAIL${NC}  $name"
                FAILED=$((FAILED + 1))
            fi
            awk '++n <= 10 { print "      " $0 }' <<<"$lines"
            return 0
        fi
        new="$lines"
        if [[ -f "$STOCK_DIR/$name" ]]; then
            new=$(grep -Fxv -f "$STOCK_DIR/$name" <<<"$lines" || true)
        fi
        if [[ -z "$new" ]]; then
            echo -e "${YELLOW}XFAIL${NC} $name  (stock divergence, $count, none new)"
            XFAILED=$((XFAILED + 1))
            return 0
        fi
        echo -e "${RED}FAIL${NC}  $name"
        FAILED=$((FAILED + 1))
        # Only the lines stock does not have: those are the ones that are ours.
        awk '++n <= 10 { print "      " $0 }' <<<"$new"
        return 0
    elif [[ "$exit_code" -ne 0 ]]; then
        # Silent crash — test exited non-zero with no FAIL line.
        echo -e "${RED}CRASH${NC} $name  (exit=$exit_code)"
        FAILED=$((FAILED + 1))
        tail -5 <<<"$out" | sed 's/^/      /'
    elif ! grep -qE '(PASS|[0-9]+ failure)' <<<"$out"; then
        # No PASS lines and no "N failure(s)" summary — test produced
        # nothing useful, treat as broken.
        echo -e "${RED}NO-PASS${NC} $name  (no PASS / failure summary line)"
        FAILED=$((FAILED + 1))
    else
        echo -e "${GREEN}PASS${NC}  $name"
        PASSED=$((PASSED + 1))
    fi
}

# ── Phase 1: native Rosetta ───────────────────────────────────────────────────
echo -e "${BOLD}=== Phase 1: native Rosetta ===${NC}"

BASELINE_PHASE=1
for t in "${TESTS[@]}"; do
    BINARY="$TESTS_BIN/$t"
    if [[ ! -x "$BINARY" ]]; then
        echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
        ERRORS=$((ERRORS + 1))
        continue
    fi
    EXIT=0
    OUT=$("$BINARY" 2>/dev/null) || EXIT=$?
    check_output "$t" "$OUT" "$EXIT"
done
BASELINE_PHASE=0

# ── Phase 2: x87sidecar ───────────────────────────────────────────────────
# pipefail (set -o pipefail above) makes the pipe inherit the loader's
# non-zero exit, so a silent loader/test crash propagates through
# `... | filter_runtime_lines` and we capture it in $EXIT.
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 2: x87sidecar ===${NC}"

    for t in "${TESTS[@]}"; do
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(run_bounded "$TEST_TIMEOUT" "$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) ||
            EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
fi

# ── Phase 2c: x87sidecar --cooperative (handshake smoke) ──────────────────
# Cooperative attach hands the sidecar the tracee's task port over a bootstrap
# rendezvous, with no task_for_pid, ptrace, get-task-allow or elevated
# privileges. The test binaries link the handshake shim (tests/coop_handshake.c),
# and the sidecar installs the same translate_insn and decode_opcode hooks
# after Rosetta init that the default attach installs before it, so this phase
# runs every test with the hook installed through the handshake.
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 2c: x87sidecar --cooperative (handshake smoke) ===${NC}"

    for t in "${TESTS[@]}"; do
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(run_bounded "$TEST_TIMEOUT" "$LOADER" --cooperative "$BINARY" 2>/dev/null |
            filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
fi

# fild m64 into fistp/fisttp m64 is copied exactly by a native transport
# reply when the run holds only integer loads and stores, by the IR pass
# (pass_exact_i64_copy) for longer runs mixing in other ops, and by the
# fild_fistp peephole for a lone pair, and with the native slots on (the
# default) the fild writes an exact 80-bit slot, so the copy is exact
# without the IR as well.  With X87_NATIVE_SLOTS=0 and the IR off (phases 3
# and 4) a run that also holds an fstpl goes through binary64 and rounds
# every |v| >= 2^53 by design, so the exact-copy test is skipped there.
EXACT_I64_COPY_SKIP=(
    test_fild_fistp_copy
)
exact_i64_copy_skipped() {
    local s
    native_slots_off || return 1
    for s in "${EXACT_I64_COPY_SKIP[@]}"; do
        [[ "$1" == "$s" ]] && return 0
    done
    return 1
}

# ── Phase 3: x87sidecar, IR disabled ─────────────────────────────────────
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 3: x87sidecar (IR disabled) ===${NC}"

    for t in "${TESTS[@]}"; do
        if exact_i64_copy_skipped "$t"; then
            continue
        fi
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(run_bounded "$TEST_TIMEOUT" env X87_DISABLE_X87_IR=1 "$LOADER" "$BINARY" \
            2>/dev/null | filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
fi

# ── Phase 4: x87sidecar, IR disabled + all fusions disabled ──────────────
# Some results are exact only through a fusion: a lone FLD m80 or FSTP m80
# converts through the binary64 register file, and only the adjacent pair
# copies the ten bytes.  The tests that assert those exact results are
# skipped here by design.
NO_FUSION_SKIP=(
    test_m80_copy
)
no_fusion_skipped() {
    local s
    for s in "${NO_FUSION_SKIP[@]}"; do
        [[ "$1" == "$s" ]] && return 0
    done
    return 1
}
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 4: x87sidecar (IR disabled, fusions disabled) ===${NC}"

    for t in "${TESTS[@]}"; do
        if exact_i64_copy_skipped "$t" || no_fusion_skipped "$t"; then
            continue
        fi
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(run_bounded "$TEST_TIMEOUT" env X87_DISABLE_X87_IR=1 X87_DISABLE_ALL_FUSIONS=1 \
            "$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
fi

# ── Phase 5: x87sidecar X87_DISABLE_HOOK=1 (stock translate_insn only) ───
# Validates that the deliberate-fall-through ops (fxsave, fxrstor, and
# the metadata-only set in kKnownFallThrough) compose correctly with
# stock's emit.  A FAIL here indicates an m108-style internal-offset
# bug — see project_native_rosetta_lazy_f80.md.  The compose tests
# (test_*_compose.c) are the primary target of this phase.
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 5: x87sidecar X87_DISABLE_HOOK=1 (stock emit) ===${NC}"

    STOCK_EMIT_PHASE=1
    for t in "${TESTS[@]}"; do
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(run_bounded "$TEST_TIMEOUT" env X87_DISABLE_HOOK=1 "$LOADER" "$BINARY" \
            2>/dev/null | filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
    STOCK_EMIT_PHASE=0
fi

# ── Phase 6: x87sidecar X87_ENABLE_FMA_CONTRACT=1 ────────────────────────
# FMA contraction is OFF by default (real x87 rounds the product before
# the add), so Phase 2 no longer emits a single FMA anywhere.  This phase
# turns contraction on so the IR FMAdd/FMSub/FNMSub nodes, the FMA-reduce
# lowering they feed, and the fusion FMA paths stay under continuous test.
# test_fma_pc53 asserts the unfused numerics and is skipped here by design.
FMA_CONTRACT_SKIP=(
    test_fma_pc53
)
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 6: x87sidecar X87_ENABLE_FMA_CONTRACT=1 (fused FMA paths) ===${NC}"

    for t in "${TESTS[@]}"; do
        skip=0
        for s in "${FMA_CONTRACT_SKIP[@]}"; do
            [[ "$t" == "$s" ]] && skip=1
        done
        if [[ $skip -eq 1 ]]; then
            continue
        fi
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(run_bounded "$TEST_TIMEOUT" env X87_ENABLE_FMA_CONTRACT=1 "$LOADER" "$BINARY" \
            2>/dev/null | filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
fi

# ── Phase 7: x87sidecar X87_FPR_POOL_LIMIT=5 (pressure split/remat) ──────
# Clamp the FPR gate to 5 slots so every test with a run peaking above
# that exercises the pressure-split and remat/sink rescue paths
# deterministically (the real pool depends on stock's dynamic FPR
# seeding).  Values must be identical to the unclamped phases.
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 7: x87sidecar X87_FPR_POOL_LIMIT=5 (pressure split/remat) ===${NC}"

    for t in "${TESTS[@]}"; do
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(run_bounded "$TEST_TIMEOUT" env X87_FPR_POOL_LIMIT=5 "$LOADER" "$BINARY" \
            2>/dev/null | filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
fi

# ── Phase 8: x87sidecar split/remat disabled (legacy all-or-nothing gate) ─
# Kill-switch regression phase: X87_ENABLE_IR_SPLIT=0 X87_ENABLE_IR_REMAT=0
# restores the pre-split refuse-the-whole-run behavior; results must not
# change (only codegen quality does).
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 8: x87sidecar X87_ENABLE_IR_SPLIT=0 X87_ENABLE_IR_REMAT=0 ===${NC}"

    for t in "${TESTS[@]}"; do
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(run_bounded "$TEST_TIMEOUT" env X87_ENABLE_IR_SPLIT=0 X87_ENABLE_IR_REMAT=0 \
            "$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
fi

# ── Phase 9: x87sidecar X87_FAST_ROUND=2 (smart per-block, curated set) ───
# =2 keeps the full RC dispatch in blocks containing a control-word
# writer, so same-block FLDCW idioms must pass.  Cross-block RC (FLDCW in
# one block, consumer in another) is mis-rounded BY DESIGN under =2 —
# test_frndint's standalone cases hit exactly that, so this phase runs a
# curated subset instead of the full suite.
FAST_ROUND2_TESTS=(
    test_fldcw
    test_fistp_multi
    test_rc_recache
    test_fistt
)
if [[ $NATIVE_ONLY -eq 0 && ${#SELECTED_TESTS[@]} -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 9: x87sidecar X87_FAST_ROUND=2 (same-block FLDCW idioms) ===${NC}"

    for t in "${FAST_ROUND2_TESTS[@]}"; do
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(run_bounded "$TEST_TIMEOUT" env X87_FAST_ROUND=2 "$LOADER" "$BINARY" \
            2>/dev/null | filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
fi

# ── Phase 10: x87sidecar X87_ENABLE_BRIDGE=0 (bridging kill switch) ───────
# Bridging defaults ON (phases 2/6/7 exercise the bridged lowering); this
# phase keeps the unbridged dispatch under continuous regression test.
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 10: x87sidecar X87_ENABLE_BRIDGE=0 (bridging off) ===${NC}"

    for t in "${TESTS[@]}"; do
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(run_bounded "$TEST_TIMEOUT" env X87_ENABLE_BRIDGE=0 "$LOADER" "$BINARY" \
            2>/dev/null | filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
fi

# ── Phase 11: x87sidecar X87_BRIDGE_V2=0 (v2 bridging kill switch) ────────
# v2 bridging defaults ON (phase 2 exercises the flag-dead ALU lowering);
# this phase keeps the v1-only bridged dispatch under regression test.
# test_bridge_alu exercises the v2 shapes directly.
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 11: x87sidecar X87_BRIDGE_V2=0 (v2 bridging off) ===${NC}"

    for t in "${TESTS[@]}"; do
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(run_bounded "$TEST_TIMEOUT" env X87_BRIDGE_V2=0 "$LOADER" "$BINARY" \
            2>/dev/null | filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
fi

# ── Phase 12: decode_opcode hook (DC D8 fcomp alias, 32-bit ARPL) ────────
# These two tests are the only ones whose expected result differs by phase, so
# they are not in ALL_TESTS: stock Rosetta cannot decode either encoding and
# traps, the sidecar's decode_opcode hook substitutes and they pass, and
# X87_NO_DECODE_HOOK=1 puts them back to trapping.  Assert all three for each,
# so neither a silently dead hook nor a silently dead kill switch can pass
# unnoticed.
if [[ $NATIVE_ONLY -eq 0 && ${#SELECTED_TESTS[@]} -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 12: decode hook (DC D8 fcomp alias, 32-bit ARPL) ===${NC}"

    DECODE_BIN="$TESTS_BIN/test_decoder_fcomp_st"
    ARPL_BIN="$TESTS_BIN/test_decoder_arpl"
    if [[ ! -x "$DECODE_BIN" || ! -x "$ARPL_BIN" ]]; then
        echo -e "${YELLOW}SKIP${NC}  decode hook  (binaries not found)"
        ERRORS=$((ERRORS + 1))
    else
        # $1 = label, $2 = 'pass' or 'trap' (what the run must do), $3.. = command
        expect_decode() {
            local label="$1" want="$2"
            shift 2
            TOTAL=$((TOTAL + 1))
            local out exit_code=0
            out=$(run_bounded "$TEST_TIMEOUT" "$@" 2>/dev/null | filter_runtime_lines) ||
                exit_code=$?
            # A trapping run reports the SIGILL it caught in a FAIL line and
            # exits 1; any other failure, a loader that did not start
            # included, is neither result.
            local got
            if [[ $exit_code -eq 124 ]]; then
                got="a timeout after $TEST_TIMEOUT s"
            elif [[ $exit_code -eq 0 ]] && ! grep -qE 'FAIL' <<<"$out"; then
                got="pass"
                grep -qE 'PASS' <<<"$out" || got="exit 0 without a PASS line"
            elif grep -qE 'FAIL.*SIGILL' <<<"$out"; then
                got="trap"
            else
                got="exit $exit_code without a SIGILL line"
            fi
            local verb="traps"
            [[ "$want" == "pass" ]] && verb="passes"
            if [[ "$got" == "$want" ]]; then
                echo -e "${GREEN}PASS${NC}  $label  ($verb as expected)"
                PASSED=$((PASSED + 1))
            else
                echo -e "${RED}FAIL${NC}  $label  (wanted $want, got $got)"
                FAILED=$((FAILED + 1))
                tail -6 <<<"$out" | sed 's/^/      /'
            fi
        }

        expect_decode "DC D8  native Rosetta"       trap "$DECODE_BIN"
        expect_decode "DC D8  x87sidecar"           pass "$LOADER" "$DECODE_BIN"
        expect_decode "DC D8  X87_NO_DECODE_HOOK=1" trap env X87_NO_DECODE_HOOK=1 "$LOADER" \
            "$DECODE_BIN"
        expect_decode "ARPL   native Rosetta"       trap "$ARPL_BIN"
        expect_decode "ARPL   x87sidecar"           pass "$LOADER" "$ARPL_BIN"
        expect_decode "ARPL   X87_NO_DECODE_HOOK=1" trap env X87_NO_DECODE_HOOK=1 "$LOADER" \
            "$ARPL_BIN"
    fi
fi

# ── Phase 13: x87sidecar X87_STOCK_BCD=0 (inline FBLD/FBSTP, curated set) ─
# Blocks containing FBLD or FBSTP go to stock by default, so no other phase
# reaches the sidecar's own BCD translators.  This phase keeps them under
# test on the cases they get right; test_bcd_exact needs all 18 digits and
# only passes through stock.  test_flags_handoff checks that they leave the
# guest flags alone.  Phase 17 covers a fault in an inline fbld or fbstp.
STOCK_BCD_OFF_TESTS=(
    test_fbld
    test_fbstp
    test_flags_handoff
)
if [[ $NATIVE_ONLY -eq 0 && ${#SELECTED_TESTS[@]} -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 13: x87sidecar X87_STOCK_BCD=0 (inline fbld/fbstp) ===${NC}"

    for t in "${STOCK_BCD_OFF_TESTS[@]}"; do
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(run_bounded "$TEST_TIMEOUT" env X87_STOCK_BCD=0 "$LOADER" "$BINARY" \
            2>/dev/null | filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
fi

# ── Phase 14: allocation failures declined to stock ──────────────────────
# X87_FAIL_ALLOC_EVERY=N fails every Nth scratch register allocation, which
# makes the translator drop the request it was working on and decline it.
# The N values move the failures around: N=1 declines every request at its
# first allocation, the larger ones land inside IR lowering, bridged regions,
# split runs and transcendentals.  Results must match the other phases.  A
# run at N=1 that never declined means the injection is not reaching the
# allocators, which would make the phase test nothing.
ALLOC_FAIL_TESTS=(
    test_alloc_decline
    test_x87_full
    test_ir_split
    test_bridge_alu
    test_fistp_multi
    test_fist_pressure
    test_fsincos
    test_fxch
    test_x87_native_state
    test_delphi_move
    test_flags_handoff
)
ALLOC_FAIL_EVERY=(1 3 17 43 97 251)
if [[ $NATIVE_ONLY -eq 0 && ${#SELECTED_TESTS[@]} -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 14: X87_FAIL_ALLOC_EVERY=N (allocation failures declined) ===${NC}"

    for n in "${ALLOC_FAIL_EVERY[@]}"; do
        for t in "${ALLOC_FAIL_TESTS[@]}"; do
            BINARY="$TESTS_BIN/$t"
            if [[ ! -x "$BINARY" ]]; then
                echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
                ERRORS=$((ERRORS + 1))
                continue
            fi
            EXIT=0
            OUT=$(run_bounded "$TEST_TIMEOUT" env X87_STRICT_ALLOC=0 X87_FAIL_ALLOC_EVERY=$n \
                "$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) || EXIT=$?
            if [[ $n -eq 1 && $EXIT -ne 124 ]] && ! grep -q 'declined to stock' <<<"$OUT"; then
                TOTAL=$((TOTAL + 1))
                echo -e "${RED}FAIL${NC}  $t  (N=1 declined nothing)"
                FAILED=$((FAILED + 1))
                continue
            fi
            check_output "$t" "$OUT" "$EXIT"
        done
    done
fi

# ── Phase 15: X87_IGNORE_FTZ_DAZ (x87 denormals under MXCSR FTZ/DAZ) ─────
# test_x87_ftz_daz is not in ALL_TESTS because its result depends on a knob
# that is off by default: without X87_IGNORE_FTZ_DAZ=1 the sidecar flushes
# x87 denormals when MXCSR.FTZ or DAZ is set, which the test reports as
# failures.  Here it must pass natively and with the knob on (the boundary
# code is shared by every dispatch path, so default, IR off and fusions off
# cover it), and must still fail with the knob off, so a knob that silently
# stopped working, or a default that changed, does not pass unnoticed.
if [[ $NATIVE_ONLY -eq 0 && ${#SELECTED_TESTS[@]} -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 15: X87_IGNORE_FTZ_DAZ (x87 denormals under FTZ/DAZ) ===${NC}"
    # Which register holds MXCSR.FTZ/DAZ depends on FEAT_AFP; print it so a
    # failure here can be read against the hardware it ran on.
    echo "      hw.optional.arm.FEAT_AFP=$(sysctl -n hw.optional.arm.FEAT_AFP 2>/dev/null || echo unknown)"
    "$BIN/x87sidecar" --probe 2>/dev/null | grep 'MXCSR FTZ/DAZ' || true

    FTZ_BIN="$TESTS_BIN/test_x87_ftz_daz"
    if [[ ! -x "$FTZ_BIN" ]]; then
        echo -e "${YELLOW}SKIP${NC}  test_x87_ftz_daz  (binary not found)"
        ERRORS=$((ERRORS + 1))
    else
        # $1 = label, $2 = 'pass' or 'fail' (what the run must do), $3.. = command
        expect_ftz() {
            local label="$1" want="$2"
            shift 2
            TOTAL=$((TOTAL + 1))
            local out exit_code=0
            # The test's override that forces its non-FEAT_AFP tolerance must
            # not leak in from the calling shell and soften the strict path.
            out=$(run_bounded "$TEST_TIMEOUT" env -u X87_TEST_ASSUME_APPLE_FLUSH_REGISTER "$@" \
                2>/dev/null | filter_runtime_lines) || exit_code=$?
            # A run that ends any other way (a signal, an abort, no summary
            # line) is a crash, never the expected failure.
            local got="crash (exit=$exit_code)"
            if [[ $exit_code -eq 124 ]]; then
                got="a timeout after $TEST_TIMEOUT s"
            elif [[ $exit_code -eq 0 ]] && ! grep -qE 'FAIL' <<<"$out" &&
                grep -qE '^0 failure' <<<"$out"; then
                got="pass"
            elif [[ $exit_code -eq 1 ]] && grep -qE '^[1-9][0-9]* failure' <<<"$out"; then
                got="fail"
            fi
            local verb="fails"
            [[ "$want" == "pass" ]] && verb="passes"
            if [[ "$got" == "$want" ]]; then
                echo -e "${GREEN}PASS${NC}  $label  ($verb as expected)"
                PASSED=$((PASSED + 1))
                # A noted known limitation still shows in the log.
                awk '/^NOTE/ { print "      " $0 }' <<<"$out"
            else
                echo -e "${RED}FAIL${NC}  $label  (wanted $want, got $got)"
                FAILED=$((FAILED + 1))
                awk '/FAIL/ && ++n <= 6 { print "      " $0 }' <<<"$out"
                tail -3 <<<"$out" | sed 's/^/      /'
            fi
        }

        expect_ftz "ftz_daz  native Rosetta"             pass "$FTZ_BIN"
        # With X87_NATIVE_SLOTS=0 the knob's code sits in the conversion
        # boundary of every reply and keeps its stash at +6.
        expect_ftz "ftz_daz  knob on, no native slots"   pass env X87_IGNORE_FTZ_DAZ=1 \
            X87_NATIVE_SLOTS=0 "$LOADER" "$FTZ_BIN"
        expect_ftz "ftz_daz  knob on, no native slots, IR off" pass env X87_IGNORE_FTZ_DAZ=1 \
            X87_NATIVE_SLOTS=0 X87_DISABLE_X87_IR=1 "$LOADER" "$FTZ_BIN"
        expect_ftz "ftz_daz  knob on, no native slots, IR + fusions off" pass \
            env X87_IGNORE_FTZ_DAZ=1 X87_NATIVE_SLOTS=0 X87_DISABLE_X87_IR=1 \
            X87_DISABLE_ALL_FUSIONS=1 "$LOADER" "$FTZ_BIN"
        expect_ftz "ftz_daz  knob off, no native slots (flushes)" fail env X87_NATIVE_SLOTS=0 \
            "$LOADER" "$FTZ_BIN"
        # The knob adds code to every reply boundary; the guest flags must
        # still come through it.
        expect_ftz "flags_handoff  knob on, no native slots" pass env X87_IGNORE_FTZ_DAZ=1 \
            X87_NATIVE_SLOTS=0 "$LOADER" "$TESTS_BIN/test_flags_handoff"
        # With the native slots, the default, replies have no conversion
        # boundary and keep the stash at +86 instead of +6, so they run the
        # same checks, including the storm whose handler sets FTZ|DAZ in the
        # signal context.
        expect_ftz "ftz_daz  knob on, native slots"      pass env X87_IGNORE_FTZ_DAZ=1 \
            X87_NATIVE_SLOTS=1 "$LOADER" "$FTZ_BIN"
        expect_ftz "ftz_daz  knob on, native slots, IR off" pass env X87_IGNORE_FTZ_DAZ=1 \
            X87_NATIVE_SLOTS=1 X87_DISABLE_X87_IR=1 "$LOADER" "$FTZ_BIN"
        expect_ftz "ftz_daz  knob on, native slots, IR + fusions off" pass \
            env X87_IGNORE_FTZ_DAZ=1 X87_NATIVE_SLOTS=1 X87_DISABLE_X87_IR=1 \
            X87_DISABLE_ALL_FUSIONS=1 "$LOADER" "$FTZ_BIN"
        expect_ftz "ftz_daz  knob off, native slots (flushes)" fail env X87_NATIVE_SLOTS=1 \
            "$LOADER" "$FTZ_BIN"
        expect_ftz "flags_handoff  knob on, native slots" pass env X87_IGNORE_FTZ_DAZ=1 \
            X87_NATIVE_SLOTS=1 "$LOADER" "$TESTS_BIN/test_flags_handoff"
    fi
fi

# ── Phase 16: guest flags across the stock/sidecar boundary ──────────────
# test_flags_handoff puts its fcomi-family compares and the fcmovs that read
# them in separate blocks.  Handing every block that holds a compare to
# stock leaves the fcmovs to the sidecar, and handing the fcmovs to stock
# leaves the compares to the sidecar, so each translator reads PF, ZF and CF
# where the other wrote them.  The other phases cover stock's SSE, integer,
# sahf and popf setters and its setcc, jcc, cmovcc, lahf and pushf readers.
# A misspelt name, or a list that handed no block to stock, would pass
# without testing anything, so each run must resolve every name to its
# opcode id and report at least one block handed over.
# Each entry is "<X87_STOCK_OPS value>|<opcode ids it resolves to>".
FLAGS_HANDOFF_STOCK_OPS=(
    "fcomi,fucomi,fcomip,fucomip|4"
    "fcmovu,fcmovnu,fcmovb,fcmove,fcmovbe,fcmovnb,fcmovne,fcmovnbe|8"
)
if [[ $NATIVE_ONLY -eq 0 && ${#SELECTED_TESTS[@]} -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 16: guest flags across stock and the sidecar (X87_STOCK_OPS) ===${NC}"

    BINARY="$TESTS_BIN/test_flags_handoff"
    if [[ ! -x "$BINARY" ]]; then
        echo -e "${YELLOW}SKIP${NC}  test_flags_handoff  (binary not found)"
        ERRORS=$((ERRORS + 1))
    else
        for entry in "${FLAGS_HANDOFF_STOCK_OPS[@]}"; do
            ops="${entry%%|*}"
            want_ids="${entry##*|}"
            for ir_off in 0 1; do
                echo "      X87_STOCK_OPS=$ops X87_DISABLE_X87_IR=$ir_off"
                EXIT=0
                OUT=$(run_bounded "$TEST_TIMEOUT" env X87_STOCK_OPS=$ops \
                    X87_DISABLE_X87_IR=$ir_off "$LOADER" "$BINARY" 2>/dev/null |
                    filter_runtime_lines) || EXIT=$?
                if [[ $EXIT -eq 124 ]]; then
                    check_output "test_flags_handoff" "$OUT" "$EXIT"
                    continue
                fi
                if grep -q 'X87_STOCK_OPS: unknown opcode' <<<"$OUT" ||
                    ! grep -q "X87_STOCK_OPS: $want_ids opcode ids resolved" <<<"$OUT"; then
                    TOTAL=$((TOTAL + 1))
                    echo -e "${RED}FAIL${NC}  test_flags_handoff  (X87_STOCK_OPS did not resolve to $want_ids opcode ids)"
                    FAILED=$((FAILED + 1))
                    continue
                fi
                if ! grep -q '\[x87stock\] hit #1 ' <<<"$OUT"; then
                    TOTAL=$((TOTAL + 1))
                    echo -e "${RED}FAIL${NC}  test_flags_handoff  (no block handed to stock)"
                    FAILED=$((FAILED + 1))
                    continue
                fi
                check_output "test_flags_handoff" "$OUT" "$EXIT"
            done
        done
    fi
fi

# ── Phase 17: X87_PRECISE_FAULTS=1 (memory faults in translated x87 code) ─
# test_x87_sync_fault is not in ALL_TESTS: its cases fault on purpose, and
# with X87_PRECISE_FAULTS off, the default, any guest memory fault inside
# sidecar-translated x87 code aborts the process.  Here it must pass
# natively and with the knob on in the configurations that change what a
# reply holds, including the inline BCD translators and declined requests,
# and one of its faults must still abort without the knob, so a knob that
# stopped working, or a default that changed, does not pass unnoticed.
SYNC_FAULT_CONFIGS=(
    ""
    "X87_DISABLE_X87_IR=1"
    "X87_DISABLE_X87_IR=1 X87_DISABLE_ALL_FUSIONS=1"
    "X87_DISABLE_CACHE=1"
    "X87_FPR_POOL_LIMIT=5"
    "X87_ENABLE_BRIDGE=0"
    "X87_STOCK_BCD=0"
    "X87_NATIVE_SLOTS=0"
    "X87_NATIVE_SLOTS=0 X87_DISABLE_X87_IR=1"
)
if [[ $NATIVE_ONLY -eq 0 && ${#SELECTED_TESTS[@]} -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 17: X87_PRECISE_FAULTS=1 (memory faults in translated x87 code) ===${NC}"

    SF_BIN="$TESTS_BIN/test_x87_sync_fault"
    if [[ ! -x "$SF_BIN" ]]; then
        echo -e "${YELLOW}SKIP${NC}  test_x87_sync_fault  (binary not found)"
        ERRORS=$((ERRORS + 1))
    else
        echo "      native Rosetta"
        EXIT=0
        OUT=$("$SF_BIN" 2>/dev/null) || EXIT=$?
        check_output "test_x87_sync_fault" "$OUT" "$EXIT"
        for cfg in "${SYNC_FAULT_CONFIGS[@]}"; do
            echo "      X87_PRECISE_FAULTS=1 $cfg"
            EXIT=0
            # shellcheck disable=SC2086
            OUT=$(run_bounded "$TEST_TIMEOUT" env X87_PRECISE_FAULTS=1 $cfg "$LOADER" "$SF_BIN" \
                2>/dev/null | filter_runtime_lines) || EXIT=$?
            check_output "test_x87_sync_fault" "$OUT" "$EXIT"
        done
        # X87_PRECISE_FAULTS=2 with X87_NATIVE_SLOTS ends a reply before every
        # later memory access, so there the in-run group (SYNC_FAULT_IN_RUN=1)
        # has to pass as well.
        for cfg in "" "X87_DISABLE_X87_IR=1 X87_DISABLE_ALL_FUSIONS=1" "X87_DISABLE_CACHE=1" \
            "X87_IGNORE_FTZ_DAZ=1"; do
            echo "      X87_PRECISE_FAULTS=2 X87_NATIVE_SLOTS=1 SYNC_FAULT_IN_RUN=1 $cfg"
            EXIT=0
            # shellcheck disable=SC2086
            OUT=$(run_bounded "$TEST_TIMEOUT" env X87_PRECISE_FAULTS=2 X87_NATIVE_SLOTS=1 \
                SYNC_FAULT_IN_RUN=1 $cfg "$LOADER" "$SF_BIN" 2>/dev/null | filter_runtime_lines) ||
                EXIT=$?
            check_output "test_x87_sync_fault" "$OUT" "$EXIT"
        done
        # An fstpt across a page boundary of uncached memory: stored without a
        # signal natively and without the knob; with it the probe's
        # straddling halfword is unaligned there and takes SIGBUS, as
        # docs/internals.md describes.  $1 = label, $2 = expected RESULT
        # text, $3.. = command.
        expect_wc_cross() {
            local label="$1" want="$2"
            shift 2
            TOTAL=$((TOTAL + 1))
            local out exit_code=0
            out=$(run_bounded "$TEST_TIMEOUT" env SYNC_FAULT_CASE=wc_cross "$@" 2>/dev/null) ||
                exit_code=$?
            local got
            got=$(grep -E '^(RESULT|SKIP)' <<<"$out" | head -1)
            if [[ $exit_code -eq 0 && "$got" == *"$want"* ]]; then
                echo -e "${GREEN}PASS${NC}  sync_fault wc_cross  $label  ($want)"
                PASSED=$((PASSED + 1))
            elif [[ $exit_code -eq 0 && "$got" == SKIP* ]]; then
                echo -e "${YELLOW}SKIP${NC}  sync_fault wc_cross  $label  ($got)"
                ERRORS=$((ERRORS + 1))
            else
                local reason="got exit=$exit_code ${got:-no result}"
                echo -e "${RED}FAIL${NC}  sync_fault wc_cross  $label  (wanted $want, $reason)"
                FAILED=$((FAILED + 1))
            fi
        }
        expect_wc_cross "native Rosetta" "signal 0, stored" "$SF_BIN"
        expect_wc_cross "knob off" "signal 0, stored" "$LOADER" "$SF_BIN"
        expect_wc_cross "X87_PRECISE_FAULTS=1" "signal 10, not stored" \
            X87_PRECISE_FAULTS=1 "$LOADER" "$SF_BIN"
        expect_wc_cross "X87_PRECISE_FAULTS=1 X87_NATIVE_SLOTS=0" "signal 10, not stored" \
            X87_PRECISE_FAULTS=1 X87_NATIVE_SLOTS=0 "$LOADER" "$SF_BIN"
        for n in "${ALLOC_FAIL_EVERY[@]}"; do
            echo "      X87_PRECISE_FAULTS=1 X87_FAIL_ALLOC_EVERY=$n"
            EXIT=0
            OUT=$(run_bounded "$TEST_TIMEOUT" env X87_PRECISE_FAULTS=1 X87_STRICT_ALLOC=0 \
                X87_FAIL_ALLOC_EVERY=$n "$LOADER" "$SF_BIN" 2>/dev/null | filter_runtime_lines) ||
                EXIT=$?
            check_output "test_x87_sync_fault" "$OUT" "$EXIT"
        done
        # Without the knob a store at the start of a reply has to abort the
        # process with the runtime's message and SIGABRT; anything else, a
        # pass included, means the knob or its default changed.  Through the
        # cooperative attach, which the bundles use, and through the default
        # attach.  The runtime aborts from its exception server thread while
        # the faulting thread waits for that server's reply, which is the
        # case that hung the default attach while the released target was
        # still traced (see "Releasing the default attach stop" in
        # docs/internals.md).
        # $1 = label, $2.. = loader flags
        expect_sf_abort() {
            local label="$1"
            shift
            TOTAL=$((TOTAL + 1))
            local out exit_code=0
            out=$(run_bounded "$TEST_TIMEOUT" env SYNC_FAULT_CASE=s_fstp64 "$LOADER" "$@" \
                "$SF_BIN" 2>&1) || exit_code=$?
            if [[ $exit_code -eq 134 ]] &&
                grep -q 'unexpectedly need to EmulateForward on a synchronous exception' <<<"$out"; then
                echo -e "${GREEN}PASS${NC}  sync_fault  knob off, $label (aborts as expected)"
                PASSED=$((PASSED + 1))
            else
                local reason="got exit=$exit_code"
                [[ $exit_code -eq 124 ]] && reason="timed out after $TEST_TIMEOUT s"
                echo -e "${RED}FAIL${NC}  sync_fault  knob off, $label  (wanted the EmulateForward abort, $reason)"
                FAILED=$((FAILED + 1))
                tail -3 <<<"$out" | sed 's/^/      /'
            fi
        }
        expect_sf_abort "cooperative" --cooperative
        expect_sf_abort "default attach"
    fi
fi

# ── Phase 18: fatal signals after the default attach releases the target ─
# test_crash_after_release is not in ALL_TESTS: it dies on purpose, in the
# way its argument names, after one x87 instruction.  Before dying it checks
# that it is no longer traced and prints its release line, or exits 1.  Each
# run must print that line, end with the target's own exit status, natively
# and through both attaches, within the timeout, and the attached runs must
# not have fallen back to running without the hook.
CRASH_AFTER_RELEASE_CASES=(
    "abort 134"
    "segv 139"
    "kill9 137"
    "trap 132"
    "exit 3"
)
if [[ $NATIVE_ONLY -eq 0 && ${#SELECTED_TESTS[@]} -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 18: fatal signals after the release (test_crash_after_release) ===${NC}"

    CR_BIN="$TESTS_BIN/test_crash_after_release"
    if [[ ! -x "$CR_BIN" ]]; then
        echo -e "${YELLOW}SKIP${NC}  test_crash_after_release  (binary not found)"
        ERRORS=$((ERRORS + 1))
    else
        for entry in "${CRASH_AFTER_RELEASE_CASES[@]}"; do
            mode="${entry% *}"
            want="${entry#* }"
            for how in native default cooperative; do
                case "$how" in
                native) cmd=("$CR_BIN" "$mode") ;;
                default) cmd=("$LOADER" "$CR_BIN" "$mode") ;;
                cooperative) cmd=("$LOADER" --cooperative "$CR_BIN" "$mode") ;;
                esac
                TOTAL=$((TOTAL + 1))
                EXIT=0
                OUT=$(run_bounded "$TEST_TIMEOUT" "${cmd[@]}" 2>&1) || EXIT=$?
                label="crash_after_release  $mode, $how"
                reason=""
                if [[ $EXIT -eq 124 ]]; then
                    reason="timed out after $TEST_TIMEOUT s"
                elif [[ $EXIT -ne $want ]]; then
                    reason="got exit=$EXIT"
                elif ! grep -q '^released, x87 result 1\.0, ' <<<"$OUT"; then
                    reason="no release line"
                elif [[ $how != native ]] &&
                    grep -qE 'RUNNING WITHOUT X87 ACCELERATION|x87 hook not installed' <<<"$OUT"; then
                    reason="ran without the hook"
                fi
                if [[ -z "$reason" ]]; then
                    echo -e "${GREEN}PASS${NC}  $label  (exit=$EXIT)"
                    PASSED=$((PASSED + 1))
                else
                    echo -e "${RED}FAIL${NC}  $label  ($reason)"
                    FAILED=$((FAILED + 1))
                    tail -3 <<<"$OUT" | sed 's/^/      /'
                fi
            done
        done
    fi
fi

# ── Phase 19: X87_NATIVE_SLOTS=1 (x87 registers kept in their native slots) ─
# The knob is on by default, so the per-test phases run with it unless the
# whole run exports X87_NATIVE_SLOTS=0.  Here test_x87_cross_reply_exact
# runs with it in the configurations that change which translator emits a
# reply, with precise faults, and with allocation failures declined to
# stock, followed by the tests that read or replace the native x87 state,
# so a run that exports X87_NATIVE_SLOTS=0 covers the knob too.  Naming any
# of these tests on the command line runs this phase for them.
NATIVE_SLOTS_CONFIGS=(
    ""
    "X87_DISABLE_X87_IR=1"
    "X87_DISABLE_X87_IR=1 X87_DISABLE_ALL_FUSIONS=1"
    "X87_DISABLE_CACHE=1"
    "X87_DISABLE_SINGLE_FAST=1"
    "X87_FPR_POOL_LIMIT=5"
    "X87_ENABLE_BRIDGE=0"
    "X87_PRECISE_FAULTS=1"
    "X87_STRICT_ALLOC=0 X87_FAIL_ALLOC_EVERY=3"
    "X87_STRICT_ALLOC=0 X87_FAIL_ALLOC_EVERY=17"
    "X87_STRICT_ALLOC=0 X87_FAIL_ALLOC_EVERY=97"
)
NATIVE_SLOTS_STATE_TESTS=(
    test_x87_native_state
    test_x87_signal_context
    test_x87_signal_storm
    test_delphi_move
    test_flags_handoff
    test_frstor
    test_fsave
    test_fxch
    test_m80_copy
    test_fild_fistp_copy
)
# Whether phase 19 runs test $1: always in a full run, otherwise only when
# it was named.
native_slots_phase_runs() {
    local t
    [[ ${#SELECTED_TESTS[@]} -eq 0 ]] && return 0
    for t in "${SELECTED_TESTS[@]}"; do
        [[ "$t" == "$1" ]] && return 0
    done
    return 1
}
if [[ $NATIVE_ONLY -eq 0 ]]; then
    PHASE19_TESTS=()
    for t in "${NATIVE_SLOTS_TESTS[@]}" "${NATIVE_SLOTS_STATE_TESTS[@]}"; do
        native_slots_phase_runs "$t" && PHASE19_TESTS+=("$t")
    done
fi
if [[ $NATIVE_ONLY -eq 0 && ${#PHASE19_TESTS[@]} -gt 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 19: X87_NATIVE_SLOTS=1 (x87 registers in their native slots) ===${NC}"

    SAVED_NATIVE_SLOTS="${X87_NATIVE_SLOTS-unset}"
    export X87_NATIVE_SLOTS=1
    for cfg in "${NATIVE_SLOTS_CONFIGS[@]}"; do
        for t in "${NATIVE_SLOTS_TESTS[@]}"; do
            native_slots_phase_runs "$t" || continue
            echo "      X87_NATIVE_SLOTS=1 $cfg"
            BINARY="$TESTS_BIN/$t"
            if [[ ! -x "$BINARY" ]]; then
                echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
                ERRORS=$((ERRORS + 1))
                continue
            fi
            EXIT=0
            # shellcheck disable=SC2086
            OUT=$(run_bounded "$TEST_TIMEOUT" env $cfg "$LOADER" "$BINARY" 2>/dev/null |
                filter_runtime_lines) || EXIT=$?
            check_output "$t" "$OUT" "$EXIT"
        done
    done
    # X87_GPR_POOL_LIMIT=4 leaves too few registers for the exact integer
    # store, so integer stores of 80-bit registers read binary64 and the
    # test accepts 64-bit stores rounded to 53 bits.
    for t in "${NATIVE_SLOTS_TESTS[@]}"; do
        native_slots_phase_runs "$t" || continue
        echo "      X87_NATIVE_SLOTS=1 X87_GPR_POOL_LIMIT=4 ($t --int-stores-may-round)"
        BINARY="$TESTS_BIN/$t"
        [[ -x "$BINARY" ]] || continue
        EXIT=0
        OUT=$(run_bounded "$TEST_TIMEOUT" env X87_GPR_POOL_LIMIT=4 "$LOADER" "$BINARY" \
            --int-stores-may-round 2>/dev/null | filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
    echo "      X87_NATIVE_SLOTS=1"
    for t in "${NATIVE_SLOTS_STATE_TESTS[@]}"; do
        native_slots_phase_runs "$t" || continue
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(run_bounded "$TEST_TIMEOUT" "$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) ||
            EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
    if [[ "$SAVED_NATIVE_SLOTS" == unset ]]; then
        unset X87_NATIVE_SLOTS
    else
        export X87_NATIVE_SLOTS="$SAVED_NATIVE_SLOTS"
    fi
fi

# ── Phase 20: X87_NATIVE_SLOTS=0 (kill switch) ───────────────────────────
# Every test once more with the native slots off, so the compact binary64
# register file, its conversion boundary and the paths only it uses stay
# covered while the knob defaults on.  test_x87_cross_reply_exact has to
# fail here (check_output).  Skipped when the whole run already exports
# X87_NATIVE_SLOTS=0, since phase 2 then ran the same.
if [[ $NATIVE_ONLY -eq 0 ]] && ! native_slots_off; then
    echo ""
    echo -e "${BOLD}=== Phase 20: X87_NATIVE_SLOTS=0 (kill switch) ===${NC}"
    SAVED_NATIVE_SLOTS="${X87_NATIVE_SLOTS-unset}"
    export X87_NATIVE_SLOTS=0
    for t in "${TESTS[@]}"; do
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(run_bounded "$TEST_TIMEOUT" "$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) ||
            EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
    if [[ "$SAVED_NATIVE_SLOTS" == unset ]]; then
        unset X87_NATIVE_SLOTS
    else
        export X87_NATIVE_SLOTS="$SAVED_NATIVE_SLOTS"
    fi
fi

# ── Host: name-derived opcode maps and the table parse ──────────────────
# Native arm64, no Rosetta: opcode_set_host_table fed the internal names, the
# 26.4 names and a macOS 15.7.8-shaped list; parseOpcodeTable fed synthetic
# __cstring blobs.
if [[ ${#SELECTED_TESTS[@]} -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Host: opcode host table ===${NC}"
    for HOST_TEST in opcode_host_table opcode_table_parse; do
        TOTAL=$((TOTAL + 1))
        HOST_BIN="$BIN/test_$HOST_TEST"
        if [[ ! -x "$HOST_BIN" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $HOST_TEST  (binary not found)"
            ERRORS=$((ERRORS + 1))
        elif HOUT=$("$HOST_BIN" 2>&1); then
            echo -e "${GREEN}PASS${NC}  $HOST_TEST"
            PASSED=$((PASSED + 1))
        else
            echo -e "${RED}FAIL${NC}  $HOST_TEST"
            echo "$HOUT" | sed 's/^/      /'
            FAILED=$((FAILED + 1))
        fi
    done
fi

# ── Replay: captured WoW geom block through the clamped pressure gate ────
# tests/data/geom_block_874c40.ir is the canonical FprPressure reproducer
# (169 instrs; 48 fpr_fail ops at the 8-slot pool before splitting).
# Assert the pressure-relief machinery keeps it fully on the IR path.
if [[ $NATIVE_ONLY -eq 0 && ${#SELECTED_TESTS[@]} -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Replay: geom_block_874c40.ir under --fpr-pool 8 ===${NC}"
    TOTAL=$((TOTAL + 1))
    REPLAY_BIN="$BIN/ir_pressure_replay"
    GEOM_IR="$ROOT_DIR/tests/data/geom_block_874c40.ir"
    if [[ ! -x "$REPLAY_BIN" || ! -f "$GEOM_IR" ]]; then
        echo -e "${YELLOW}SKIP${NC}  geom_replay  (tool or blob not found)"
        ERRORS=$((ERRORS + 1))
    else
        # The geom blob is a legacy capture with 26.4-host opcodes.
        ROUT=$("$REPLAY_BIN" "$GEOM_IR" --fpr-pool 8 --runtime-version 0) || true
        if grep -q '^ir_fpr_fail,0$' <<<"$ROUT" && \
           grep -qE '^ir_split,[1-9]' <<<"$ROUT"; then
            echo -e "${GREEN}PASS${NC}  geom_replay (fpr_fail=0, splits fired)"
            PASSED=$((PASSED + 1))
        else
            echo -e "${RED}FAIL${NC}  geom_replay"
            echo "$ROUT" | sed 's/^/      /'
            FAILED=$((FAILED + 1))
        fi
    fi

    # tests/data/fist_pressure.ir is test_fist_pressure's five blocks: eight
    # values live at an integer store, whose indefinite guard takes one more
    # FPR.  On the 8-slot pool the gate has to see that register and split,
    # or the lowering runs out of FPRs and asserts.  Exactly one split per
    # block shows that every block still reaches its stores.  The suffix
    # after each split starts with Xbase, Wd_top and Xst_base already held
    # by the run cache; no GPR refusals shows the GPR model does not charge
    # those to the pool a second time.
    echo ""
    echo -e "${BOLD}=== Replay: fist_pressure.ir under --fpr-pool 8 ===${NC}"
    TOTAL=$((TOTAL + 1))
    FIST_IR="$ROOT_DIR/tests/data/fist_pressure.ir"
    if [[ ! -x "$REPLAY_BIN" || ! -f "$FIST_IR" ]]; then
        echo -e "${YELLOW}SKIP${NC}  fist_pressure_replay  (tool or blob not found)"
        ERRORS=$((ERRORS + 1))
    else
        REXIT=0
        ROUT=$("$REPLAY_BIN" "$FIST_IR" --fpr-pool 8 2>&1) || REXIT=$?
        if [[ $REXIT -eq 0 ]] && grep -q '^ir_fpr_fail,0$' <<<"$ROUT" && \
           grep -q '^ir_gpr_fail,0$' <<<"$ROUT" && grep -q '^ir_split,5$' <<<"$ROUT"; then
            echo -e "${GREEN}PASS${NC}  fist_pressure_replay (fpr_fail=0, gpr_fail=0, splits fired)"
            PASSED=$((PASSED + 1))
        else
            echo -e "${RED}FAIL${NC}  fist_pressure_replay  (exit=$REXIT)"
            echo "$ROUT" | sed 's/^/      /'
            FAILED=$((FAILED + 1))
        fi
    fi

    # tests/data/ir_dup_input.ir is test_ir_dup_input's four blocks: fyl2x,
    # fpatan and fscale with both operands the same node, and just enough
    # other values live to reach the 8-slot pool if that node is counted
    # twice.  The run has to be relieved before the lowering asserts, and
    # all 38 x87 instructions have to stay on the IR path.
    echo ""
    echo -e "${BOLD}=== Replay: ir_dup_input.ir under --fpr-pool 8 ===${NC}"
    TOTAL=$((TOTAL + 1))
    DUP_IR="$ROOT_DIR/tests/data/ir_dup_input.ir"
    if [[ ! -x "$REPLAY_BIN" || ! -f "$DUP_IR" ]]; then
        echo -e "${YELLOW}SKIP${NC}  ir_dup_input_replay  (tool or blob not found)"
        ERRORS=$((ERRORS + 1))
    else
        REXIT=0
        ROUT=$("$REPLAY_BIN" "$DUP_IR" --fpr-pool 8 2>&1) || REXIT=$?
        if [[ $REXIT -eq 0 ]] && grep -q '^ir_fpr_fail,0$' <<<"$ROUT" && \
           grep -q '^ir,38$' <<<"$ROUT"; then
            echo -e "${GREEN}PASS${NC}  ir_dup_input_replay (fpr_fail=0, all on IR)"
            PASSED=$((PASSED + 1))
        else
            echo -e "${RED}FAIL${NC}  ir_dup_input_replay  (exit=$REXIT)"
            echo "$ROUT" | sed 's/^/      /'
            FAILED=$((FAILED + 1))
        fi
    fi

    # tests/data/ir_fpatan_rc.ir is test_ir_fpatan_rc's two blocks: fpatan
    # in a run that pins the cached rounding control, at a fresh run start
    # with the full 8-GPR pool.  The model has to predict 5 pinned + 4 for
    # fpatan = 9 and keep the run off the lowering, which otherwise runs out
    # of GPRs and asserts.
    echo ""
    echo -e "${BOLD}=== Replay: ir_fpatan_rc.ir on the full GPR pool ===${NC}"
    TOTAL=$((TOTAL + 1))
    FPATAN_IR="$ROOT_DIR/tests/data/ir_fpatan_rc.ir"
    if [[ ! -x "$REPLAY_BIN" || ! -f "$FPATAN_IR" ]]; then
        echo -e "${YELLOW}SKIP${NC}  ir_fpatan_rc_replay  (tool or blob not found)"
        ERRORS=$((ERRORS + 1))
    else
        REXIT=0
        ROUT=$("$REPLAY_BIN" "$FPATAN_IR" --fpr-pool 8 --gpr-pool 8 2>&1) || REXIT=$?
        if [[ $REXIT -eq 0 ]] && grep -q '^max_gpr_peak,9$' <<<"$ROUT"; then
            echo -e "${GREEN}PASS${NC}  ir_fpatan_rc_replay (gpr peak 9 seen)"
            PASSED=$((PASSED + 1))
        else
            echo -e "${RED}FAIL${NC}  ir_fpatan_rc_replay  (exit=$REXIT)"
            echo "$ROUT" | sed 's/^/      /'
            FAILED=$((FAILED + 1))
        fi
    fi
fi

echo ""
# Tracing must also survive Rosetta's recovery interpreter, not just execute
# correctly on ARM hardware. The fixture includes the selected hash in both modes.
if [[ $NATIVE_ONLY -eq 0 && " ${TESTS[*]} " == *" test_x87_signal_context "* ]]; then
    EXIT=0
    # The script bounds each of its loader runs with run_bounded and
    # TEST_TIMEOUT and exits 124 when one times out.  Wrapping the script
    # itself would not help: a kill of its group does not reach the groups
    # of the runs inside it.
    OUT=$(bash "$SCRIPT_DIR/test_x87_trace.sh" "$BUILD_DIR" 2>&1) || EXIT=$?
    check_output "x87_trace" "$OUT" "$EXIT"
fi

echo "================================================================"
echo -e "Results: ${GREEN}${PASSED} passed${NC}, ${RED}${FAILED} failed${NC}, ${YELLOW}${XFAILED} stock divergences${NC}, ${YELLOW}${ERRORS} skipped${NC} / ${TOTAL} total"

if [[ $FAILED -gt 0 ]]; then
    exit 1
fi
exit 0
