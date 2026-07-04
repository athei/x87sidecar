#!/usr/bin/env bash
#
# run_tests.sh -- run all x87 test binaries under native Rosetta and the
# custom x87sidecar, checking self-reported PASS/FAIL output.
#
# Phases:
#   1. Native Rosetta (baseline)
#   2. x87sidecar (IR + fusions enabled, default config)
#   3. x87sidecar with X87_DISABLE_X87_IR=1 (direct translator only)
#   4. x87sidecar with X87_DISABLE_X87_IR=1 + X87_DISABLE_ALL_FUSIONS=1
#   5. x87sidecar with X87_DISABLE_HOOK=1 (stock translate_insn only)
#   6. x87sidecar with X87_ENABLE_FMA_REDUCE=0 (legacy scalar FMADD path —
#      regression catch since FMA-reduce now defaults ON)
#   7. x87sidecar with X87_FPR_POOL_LIMIT=5 (deterministic pressure
#      splitting / remat-sink exercise on every test)
#   8. x87sidecar with X87_ENABLE_IR_SPLIT=0 X87_ENABLE_IR_REMAT=0
#      (legacy all-or-nothing pressure gate)
#   9. x87sidecar with X87_FAST_ROUND=2 on a curated same-block-FLDCW set
#      (cross-block RC is mis-rounded by design under =2)
#  10. x87sidecar with X87_ENABLE_BRIDGE=1 (run bridging v1 — default OFF
#      while soaking; the phase keeps the bridged lowering exercised)
#   R. replay tests/data/geom_block_874c40.ir under --fpr-pool 8 and
#      assert the pressure splits keep it on the IR path (fpr_fail=0)
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
LOADER="$BIN/x87sidecar"
TESTS_BIN="$BIN/tests"

ALL_TESTS=(
    test_fldconst
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
    test_bridge_mov
    test_bridge_lea
    test_bridge_flags
    test_fld_arith_arithp_fma
    test_readst_elide
    test_fxch
    test_fxch_initial
    test_fldcw
    test_fcomi
    test_frndint
    test_fistp_multi
    test_fcmov
    test_ficom
    test_rc_recache
    test_fstpt_gs
    test_ir_gate_tag_push
    test_fma_reduce
    test_fma_reduce_strided
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
    test_fcos
    test_f2xm1
    test_fpatan
    test_fsincos
    test_fptan
    test_fyl2x
    test_fyl2xp1
    test_fprem
    test_fprem1
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

if [[ $DO_BUILD -eq 1 ]]; then
    echo -e "${CYAN}Building...${NC}"
    cmake --build "$BUILD_DIR" --parallel 2>&1 | tail -3
    echo ""
fi

# Strip runtime-internal noise lines before checking output
filter_runtime_lines() {
    grep -v -E 'RosettaRuntimex87 built|Installing JIT|JIT Translation Hook|try_fuse_|CORE_LOG'
}

TOTAL=0
PASSED=0
FAILED=0
ERRORS=0

check_output() {
    local name="$1"
    local out="$2"
    local exit_code="$3"
    TOTAL=$((TOTAL + 1))
    if echo "$out" | grep -qE 'FAIL'; then
        echo -e "${RED}FAIL${NC}  $name"
        FAILED=$((FAILED + 1))
        echo "$out" | grep -E 'FAIL' | head -10 | sed 's/^/      /'
    elif [[ "$exit_code" -ne 0 ]]; then
        # Silent crash — test exited non-zero with no FAIL line.
        echo -e "${RED}CRASH${NC} $name  (exit=$exit_code)"
        FAILED=$((FAILED + 1))
        echo "$out" | tail -5 | sed 's/^/      /'
    elif ! echo "$out" | grep -qE '(PASS|[0-9]+ failure)'; then
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
        OUT=$("$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
fi

# ── Phase 3: x87sidecar, IR disabled ─────────────────────────────────────
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 3: x87sidecar (IR disabled) ===${NC}"

    for t in "${TESTS[@]}"; do
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(X87_DISABLE_X87_IR=1 "$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
fi

# ── Phase 4: x87sidecar, IR disabled + all fusions disabled ──────────────
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 4: x87sidecar (IR disabled, fusions disabled) ===${NC}"

    for t in "${TESTS[@]}"; do
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(X87_DISABLE_X87_IR=1 X87_DISABLE_ALL_FUSIONS=1 "$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) || EXIT=$?
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

    for t in "${TESTS[@]}"; do
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(X87_DISABLE_HOOK=1 "$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
fi

# ── Phase 6: x87sidecar X87_ENABLE_FMA_REDUCE=0 ──────────────────────────
# FMA-reduce defaults ON in production, so Phase 2 already covers the
# vector-lowered path.  This phase forces the pass OFF to keep the
# scalar FMADD path under continuous test — protects against regressions
# in the legacy lowering (or in places that take a different path when
# the pass doesn't pre-tag chain heads).
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 6: x87sidecar X87_ENABLE_FMA_REDUCE=0 (scalar FMADD) ===${NC}"

    for t in "${TESTS[@]}"; do
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(X87_ENABLE_FMA_REDUCE=0 "$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) || EXIT=$?
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
        OUT=$(X87_FPR_POOL_LIMIT=5 "$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) || EXIT=$?
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
        OUT=$(X87_ENABLE_IR_SPLIT=0 X87_ENABLE_IR_REMAT=0 "$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) || EXIT=$?
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
        OUT=$(X87_FAST_ROUND=2 "$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
fi

# ── Phase 10: x87sidecar X87_ENABLE_BRIDGE=1 (run bridging v1) ────────────
# Bridging defaults OFF while soaking; this phase keeps the bridged
# lowering under continuous test across the whole suite (regions form
# wherever mov/lea gaps join x87 segments, incl. compiler-generated ones).
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 10: x87sidecar X87_ENABLE_BRIDGE=1 (run bridging) ===${NC}"

    for t in "${TESTS[@]}"; do
        BINARY="$TESTS_BIN/$t"
        if [[ ! -x "$BINARY" ]]; then
            echo -e "${YELLOW}SKIP${NC}  $t  (binary not found)"
            ERRORS=$((ERRORS + 1))
            continue
        fi
        EXIT=0
        OUT=$(X87_ENABLE_BRIDGE=1 "$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
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
        if echo "$ROUT" | grep -q '^ir_fpr_fail,0$' && \
           echo "$ROUT" | grep -qE '^ir_split,[1-9]'; then
            echo -e "${GREEN}PASS${NC}  geom_replay (fpr_fail=0, splits fired)"
            PASSED=$((PASSED + 1))
        else
            echo -e "${RED}FAIL${NC}  geom_replay"
            echo "$ROUT" | sed 's/^/      /'
            FAILED=$((FAILED + 1))
        fi
    fi
fi

echo ""
echo "================================================================"
echo -e "Results: ${GREEN}${PASSED} passed${NC}, ${RED}${FAILED} failed${NC}, ${YELLOW}${ERRORS} skipped${NC} / ${TOTAL} total"

if [[ $FAILED -gt 0 ]]; then
    exit 1
fi
exit 0
