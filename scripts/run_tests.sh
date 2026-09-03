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
#   R. replay tests/data/geom_block_874c40.ir under --fpr-pool 8 and
#      assert the pressure splits keep it on the IR path (fpr_fail=0)
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
    test_fcmov
    test_ficom
    test_rc_recache
    test_fstpt_gs
    test_ir_gate_tag_push
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
        OUT=$("$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) || EXIT=$?
        check_output "$t" "$OUT" "$EXIT"
    done
fi

# ── Phase 2c: x87sidecar --cooperative (handshake smoke) ──────────────────
# Cooperative attach hands the sidecar the tracee's task port over a bootstrap
# rendezvous — no task_for_pid / ptrace / get-task-allow / elevated privileges.
# This phase verifies --cooperative does not break execution. NOTE: cooperative
# mode does not yet reinstall the x87 JIT hook (the exec-pre-init stop it needs
# is not reproducible cooperatively — see the cooperative-attach plan), so this
# checks stock-Rosetta correctness under the handshake, not [ulp] acceleration.
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
        OUT=$("$LOADER" --cooperative "$BINARY" 2>/dev/null | filter_runtime_lines) || EXIT=$?
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
        OUT=$(X87_ENABLE_FMA_CONTRACT=1 "$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) || EXIT=$?
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
        OUT=$(X87_ENABLE_BRIDGE=0 "$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) || EXIT=$?
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
        OUT=$(X87_BRIDGE_V2=0 "$LOADER" "$BINARY" 2>/dev/null | filter_runtime_lines) || EXIT=$?
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
            out=$("$@" 2>/dev/null | filter_runtime_lines) || exit_code=$?
            local got="trap"
            if [[ $exit_code -eq 0 ]] && ! grep -qE 'FAIL' <<<"$out"; then
                got="pass"
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
fi

echo ""
echo "================================================================"
echo -e "Results: ${GREEN}${PASSED} passed${NC}, ${RED}${FAILED} failed${NC}, ${YELLOW}${XFAILED} stock divergences${NC}, ${YELLOW}${ERRORS} skipped${NC} / ${TOTAL} total"

if [[ $FAILED -gt 0 ]]; then
    exit 1
fi
exit 0
