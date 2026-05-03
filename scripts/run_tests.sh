#!/usr/bin/env bash
#
# run_tests.sh -- run all x87 test binaries under native Rosetta and the
# custom rosettax87, checking self-reported PASS/FAIL output.
#
# Phases:
#   1. Native Rosetta (baseline)
#   2. rosettax87 (IR + fusions enabled, default config)
#   3. rosettax87 with X87_DISABLE_X87_IR=1 (direct translator only)
#   4. rosettax87 with X87_DISABLE_X87_IR=1 + X87_DISABLE_ALL_FUSIONS=1
#   5. rosettax87 with X87_DISABLE_HOOK=1 (stock translate_insn only)
#
# Usage:
#   bash scripts/run_tests.sh                # build + test (all phases)
#   bash scripts/run_tests.sh --no-build     # skip build
#   bash scripts/run_tests.sh --native-only  # Phase 1 only (no rosettax87)
#   bash scripts/run_tests.sh test_arith     # only run specific test(s)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"
BIN="$BUILD_DIR/bin"
LOADER="$BIN/rosettax87"
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

# ── Phase 2: rosettax87 ───────────────────────────────────────────────────
# pipefail (set -o pipefail above) makes the pipe inherit the loader's
# non-zero exit, so a silent loader/test crash propagates through
# `... | filter_runtime_lines` and we capture it in $EXIT.
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 2: rosettax87 ===${NC}"

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

# ── Phase 3: rosettax87, IR disabled ─────────────────────────────────────
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 3: rosettax87 (IR disabled) ===${NC}"

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

# ── Phase 4: rosettax87, IR disabled + all fusions disabled ──────────────
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 4: rosettax87 (IR disabled, fusions disabled) ===${NC}"

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

# ── Phase 5: rosettax87 X87_DISABLE_HOOK=1 (stock translate_insn only) ───
# Validates that the deliberate-fall-through ops (fxsave, fxrstor, and
# the metadata-only set in kKnownFallThrough) compose correctly with
# stock's emit.  A FAIL here indicates an m108-style internal-offset
# bug — see project_native_rosetta_lazy_f80.md.  The compose tests
# (test_*_compose.c) are the primary target of this phase.
if [[ $NATIVE_ONLY -eq 0 ]]; then
    echo ""
    echo -e "${BOLD}=== Phase 5: rosettax87 X87_DISABLE_HOOK=1 (stock emit) ===${NC}"

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

echo ""
echo "================================================================"
echo -e "Results: ${GREEN}${PASSED} passed${NC}, ${RED}${FAILED} failed${NC}, ${YELLOW}${ERRORS} skipped${NC} / ${TOTAL} total"

if [[ $FAILED -gt 0 ]]; then
    exit 1
fi
exit 0
