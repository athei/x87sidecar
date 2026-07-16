#!/usr/bin/env bash
#
# run_fusion_sweep.sh -- Systematically test fusion translations.
#
# Phase 1 — Single: enable one fusion at a time (all other fusions disabled,
#   all opcodes JITted as usual). Identifies which individual fusions have
#   broken translations.
#
# Phase 2 — Cumulative: enable fusions one-by-one in order. Identifies which
#   fusion, when added to the already-working set, causes a failure.
#
# Usage:
#   bash scripts/run_fusion_sweep.sh                   # all phases, with build
#   bash scripts/run_fusion_sweep.sh --no-build        # skip cmake build
#   bash scripts/run_fusion_sweep.sh --phase1-only     # single-fusion phase only
#   bash scripts/run_fusion_sweep.sh --phase2-only     # cumulative-fusion phase only
#   bash scripts/run_fusion_sweep.sh --stop-on-fail    # stop cumulative at first new failure
#   bash scripts/run_fusion_sweep.sh --timeout 30      # kill+retry tests that hang > 30s
#   bash scripts/run_fusion_sweep.sh --all-tests        # run all tests (not just fusion-relevant) in sweep phases

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"
BIN="$BUILD_DIR/bin"
# Default-attach (task_for_pid + ptrace) needs the entitled build; the flat
# `x87sidecar` ships without entitlements and is cooperative-only.
LOADER="$BIN/x87sidecar_entitled"
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
    test_peephole3
    test_peephole4
    test_peephole5
    test_deep_stack
    test_x87_full
    test_fstpt
    test_peephole7
    test_arith_faddp
)

ALL_FUSIONS=(
    fld_arithp
    fld_fstp
    fld_arith_fstp
    fld_fcomp_fstsw
    fxch_arithp
    fxch_fstp
    fcom_fstsw
    fld_fcompp_fstsw
    fld_fld_fucompp
    fld_fcomp
    fld_arith_arithp
    arithp_fstp
    fstp_fld
    arith_fstp
    arith_faddp
)

# Tests that exercise fusion patterns
FUSION_TESTS=(test_peephole3 test_peephole4 test_peephole5 test_peephole6 test_peephole7 test_peephole8 test_peephole test_arith test_fcomp_mem test_x87_full test_arith_faddp)

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# Parse args
DO_BUILD=1
RUN_PHASE1=1
RUN_PHASE2=1
STOP_ON_FAIL=0
STOPPED=0
TIMEOUT_SECS=0
ALL_TESTS_IN_SWEEPS=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-build)    DO_BUILD=0 ;;
        --phase1-only) RUN_PHASE2=0 ;;
        --phase2-only) RUN_PHASE1=0 ;;
        --stop-on-fail) STOP_ON_FAIL=1 ;;
        --timeout)     shift; TIMEOUT_SECS="$1" ;;
        --all-tests)   ALL_TESTS_IN_SWEEPS=1 ;;
        *) echo "Unknown arg: $1" >&2; exit 1 ;;
    esac
    shift
done

if [[ $ALL_TESTS_IN_SWEEPS -eq 1 ]]; then
    SWEEP_TESTS=("${ALL_TESTS[@]}")
else
    SWEEP_TESTS=("${FUSION_TESTS[@]}")
fi

if [[ $DO_BUILD -eq 1 ]]; then
    echo -e "${CYAN}Building...${NC}"
    cmake --build "$BUILD_DIR" --parallel 2>&1 | tail -3
    echo ""
fi

filter_runtime_lines() {
    grep -v -E 'RosettaRuntimex87 built|Installing JIT|JIT Translation Hook|try_fuse_|CORE_LOG'
}

TOTAL=0
PASSED=0
FAILED=0
ERRORS=0

run_one_test() {
    local config_name="$1"
    local envvars="$2"
    local binary_name="$3"
    local binary="$TESTS_BIN/$binary_name"

    TOTAL=$(( TOTAL + 1 ))

    if [[ ! -x "$binary" ]]; then
        echo -e "  ${YELLOW}SKIP${NC}  $binary_name  (binary not found)"
        ERRORS=$(( ERRORS + 1 ))
        return
    fi

    local attempt=0
    local timed_out=0
    local tmpout
    tmpout=$(mktemp)

    while (( attempt < 2 )); do
        attempt=$(( attempt + 1 ))
        timed_out=0

        if [[ $TIMEOUT_SECS -gt 0 ]]; then
            local rc=0
            # shellcheck disable=SC2086
            gtimeout "$TIMEOUT_SECS" env $envvars "$LOADER" "$binary" > "$tmpout" 2>/dev/null || rc=$?
            if [[ $rc -eq 124 ]]; then timed_out=1; fi
        else
            # shellcheck disable=SC2086
            env $envvars "$LOADER" "$binary" > "$tmpout" 2>/dev/null || true
        fi

        if [[ $timed_out -eq 1 && $attempt -eq 1 ]]; then
            echo -e "  ${YELLOW}TIMEOUT${NC}  $binary_name  [$config_name] — retrying..."
            continue
        fi
        break
    done

    OUT=$(filter_runtime_lines < "$tmpout" || true)
    rm -f "$tmpout"

    if [[ $timed_out -eq 1 ]]; then
        echo -e "  ${RED}TIMEOUT${NC}  $binary_name  [$config_name] — timed out on retry"
        FAILED=$(( FAILED + 1 ))
        return
    fi

    # Herestring + awk instead of `echo | grep [-q] | head`: see the EPIPE /
    # pipefail note in run_tests.sh check_output().
    if grep -qE 'FAIL' <<<"$OUT"; then
        echo -e "  ${RED}FAIL${NC}  $binary_name  [$config_name]"
        FAILED=$(( FAILED + 1 ))
        awk '/FAIL/ && ++n <= 10 { print "        " $0 }' <<<"$OUT"
    else
        echo -e "  ${GREEN}PASS${NC}  $binary_name"
        PASSED=$(( PASSED + 1 ))
    fi
}

run_config() {
    local section="$1"
    local name="$2"
    local envvars="$3"
    local -a tests=("${@:4}")

    echo -e "${BOLD}=== [$section] $name ===${NC}"
    for t in "${tests[@]}"; do
        run_one_test "$name" "$envvars" "$t"
    done
    echo ""
}

# All fusions except those in the named array, comma-separated
fusions_except_set() {
    local -n _enabled_set="$1"
    printf '%s\n' "${ALL_FUSIONS[@]}" \
        | grep -vFx "$(printf '%s\n' "${_enabled_set[@]}")" \
        | paste -s -d ',' -
}

echo -e "${CYAN}Fusion sweep: test fusions in isolation and cumulatively${NC}"
echo "================================================================"
echo ""

# ── Baseline: all fusions disabled (JIT on, no peephole) ─────────────────────
run_config sweep baseline \
    "X87_DISABLE_ALL_FUSIONS=1" \
    "${ALL_TESTS[@]}"

# ── Phase 1 — Single fusion ───────────────────────────────────────────────────
if [[ $RUN_PHASE1 -eq 1 ]]; then
    echo -e "${BOLD}━━━ Phase 1: single fusion (enable one, disable other fusions) ━━━${NC}"
    echo ""
    for f in "${ALL_FUSIONS[@]}"; do
        single=("$f")
        others=$(fusions_except_set single || true)
        prev_failed=$FAILED
        run_config sweep "single:$f" \
            "X87_DISABLE_FUSIONS=$others" \
            "${SWEEP_TESTS[@]}"
        if [[ $STOP_ON_FAIL -eq 1 && $FAILED -gt $prev_failed ]]; then
            echo -e "${RED}STOPPED: fusion '$f' has failure(s)${NC}"
            STOPPED=1
            break
        fi
    done
fi

# ── Phase 2 — Cumulative fusions ─────────────────────────────────────────────
if [[ $RUN_PHASE2 -eq 1 && $STOPPED -eq 0 ]]; then
    echo -e "${BOLD}━━━ Phase 2: cumulative fusions (grow enabled set) ━━━${NC}"
    echo ""
    enabled=()
    for f in "${ALL_FUSIONS[@]}"; do
        enabled+=("$f")
        prev_failed=$FAILED

        if [[ ${#enabled[@]} -le 3 ]]; then
            label=$(IFS=+; echo "${enabled[*]}")
        else
            label="${enabled[0]}+${enabled[1]}+${enabled[2]}+...(${#enabled[@]})"
        fi

        others=$(fusions_except_set enabled || true)
        if [[ -z "$others" ]]; then
            run_config sweep "cumul:$label" \
                "" \
                "${SWEEP_TESTS[@]}"
        else
            run_config sweep "cumul:$label" \
                "X87_DISABLE_FUSIONS=$others" \
                "${SWEEP_TESTS[@]}"
        fi

        if [[ $STOP_ON_FAIL -eq 1 && $FAILED -gt $prev_failed ]]; then
            echo -e "${RED}STOPPED: adding '$f' introduced new failure(s)${NC}"
            STOPPED=1
            break
        fi
    done
fi

# ── Summary ───────────────────────────────────────────────────────────────────
echo "================================================================"
echo -e "Results: ${GREEN}${PASSED} passed${NC}, ${RED}${FAILED} failed${NC}, ${YELLOW}${ERRORS} skipped${NC} / ${TOTAL} total"

if [[ $FAILED -gt 0 ]]; then
    exit 1
fi
exit 0
