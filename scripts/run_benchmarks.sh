#!/usr/bin/env bash
#
# run_benchmarks.sh -- Run x87 benchmarks under native Rosetta and the JIT and
# print a comparison table.
#
# Configurations:
#   1. Native Rosetta    — binary run directly (baseline)
#   2. Loader optimized  — rosettax87 with all JIT optimizations enabled
#
# Usage:
#   bash scripts/run_benchmarks.sh              # build + run
#   bash scripts/run_benchmarks.sh --no-build   # skip cmake build

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"
BIN="$BUILD_DIR/bin"
LOADER="$BIN/rosettax87"
BENCH_BIN="$BIN/bench"

ALL_BENCHMARKS=(
    bench_constants
    bench_load
    bench_add
    bench_sub
    bench_mul
    bench_div
    bench_store
    bench_compare
    bench_unary
    bench_fcmov
    bench_fusion_fld_arithp
    bench_fusion_fld_fstp
    bench_fusion_fld_arith_fstp
    bench_fusion_fld_fcomp_fstsw
    bench_fusion_fxch_arithp
    bench_fusion_fxch_fstp
    bench_fusion_fcom_fstsw
    bench_fusion_arithp_fstp
    bench_fusion_arith_fstp
    bench_fusion_arith_faddp
    bench_fusion_fld_arith_arithp
    bench_fstp_fld
    bench_round
    bench_fistt
    bench_tag_batch
    bench_fistp_multi
    bench_fbld
    bench_fclex
    bench_fdecstp
    bench_fincstp
    bench_ffree
    bench_fdisi_feni
    bench_fnop
    bench_fxtract
    bench_fscale
    bench_finit
    bench_fbstp
    bench_fldenv
    bench_fstenv
    bench_frstor
    bench_fsave
    bench_fxsave
    bench_fxrstor
    bench_fsin
    bench_fcos
    bench_f2xm1
    bench_fpatan
    bench_fsincos
    bench_fptan
    bench_fyl2x
    bench_fyl2xp1
    bench_fprem
    bench_fprem1
)

RED='\033[0;31m'
GREEN='\033[0;32m'
WHITE='\033[1;37m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

DO_BUILD=1
for arg in "$@"; do
    if [[ "$arg" == "--no-build" ]]; then
        DO_BUILD=0
    fi
done

if [[ $DO_BUILD -eq 1 ]]; then
    echo -e "${CYAN}Building...${NC}"
    cmake --build "$BUILD_DIR" --parallel 2>&1 | tail -3
    echo ""
fi

# Strip runtime-internal noise lines before checking output (same filter as run_tests.sh)
filter_runtime_lines() {
    grep -v -E 'RosettaRuntimex87 built|Installing JIT|JIT Translation Hook|try_fuse_|CORE_LOG' || true
}

# Run a benchmark binary through the loader.
#
# Both the "baseline" and "optimized" columns now go through the loader,
# differing only in X87_DISABLE_HOOK=1.  This is a deliberate apples-to-apples
# choice: with the loader attached, Apple's runtime drops out of AOT-cache
# + interpreter mode (because the loader writes g_disable_aot=1 in
# libRosettaRuntime).  Running the bare binary uses Apple's interpreter
# fast path for tiny x87 loops, which is faster than any JIT translation
# (the bench is ~0.55× of the interpreter "native" time even when our hook
# is bypassed).  X87_DISABLE_HOOK=1 strips just the translate_insn entry
# patch but keeps everything else, so both columns translate via the same
# JIT pipeline; the "optimized" column adds our inline x87 emit, the
# "baseline" column relies on stock libRosettaRuntime's JIT codegen.
#
# Usage: run_bench <binary> <mode: baseline|optimized> [extra-env=val...]
run_bench() {
    local binary="$1"
    local mode="$2"
    shift 2
    local -a extra_env=("$@")
    if [[ "$mode" == "baseline" ]]; then
        extra_env+=("X87_DISABLE_HOOK=1")
    fi
    env "${extra_env[@]}" "$LOADER" "$binary" 2>/dev/null | filter_runtime_lines
}

# Collect results: associative arrays keyed by "bench.func_name"
declare -A NATIVE
declare -A OPTIMIZED

echo -e "${BOLD}Running benchmarks...${NC}"
echo ""

for bench in "${ALL_BENCHMARKS[@]}"; do
    binary="$BENCH_BIN/$bench"
    if [[ ! -x "$binary" ]]; then
        echo -e "${YELLOW}SKIP${NC}  $bench  (binary not found)"
        continue
    fi

    echo -ne "  ${bench} ... "

    while IFS=' ' read -r keyword name ticks; do
        [[ "$keyword" == "BENCH" ]] || continue
        NATIVE["$bench.$name"]="$ticks"
    done < <(run_bench "$binary" baseline)

    while IFS=' ' read -r keyword name ticks; do
        [[ "$keyword" == "BENCH" ]] || continue
        OPTIMIZED["$bench.$name"]="$ticks"
    done < <(run_bench "$binary" optimized)

    echo -e "${GREEN}done${NC}"
done

echo ""

# ── Legend ────────────────────────────────────────────────────────────────────
echo -e "${BOLD}Columns:${NC}"
echo "  baseline  — X87_DISABLE_HOOK=1 rosettax87 (no x87 hook, AOT cache + interpreter"
echo "              both disabled — same translation environment as the optimized"
echo "              column, only without our inline x87 emit)"
echo "  JIT       — rosettax87 with full JIT optimizations enabled"
echo "  spd_gain  — speedup of JIT over baseline (apples-to-apples)"
echo ""

# ── Print comparison table ─────────────────────────────────────────────────────
COL_NAME=42
COL_NUM=10
COL_SPD=9
DIVIDER=$(printf '─%.0s' $(seq 1 $((COL_NAME + COL_NUM*2 + COL_SPD + 8))))

printf "${BOLD}%-${COL_NAME}s %${COL_NUM}s %${COL_NUM}s %${COL_SPD}s${NC}\n" \
    "benchmark" "baseline" "JIT" "spd_gain"
echo "$DIVIDER"

total_vs_native_pct=0
total_count=0

for bench in "${ALL_BENCHMARKS[@]}"; do
    binary="$BENCH_BIN/$bench"
    [[ -x "$binary" ]] || continue

    for key in "${!NATIVE[@]}"; do
        [[ "$key" == "$bench."* ]] || continue
        func_name="${key#"$bench."}"

        native_ticks="${NATIVE[$key]:-0}"
        optimized_ticks="${OPTIMIZED[$bench.$func_name]:-0}"

        # Speedup vs native: native/optimized
        if [[ $optimized_ticks -gt 0 && $native_ticks -gt 0 ]]; then
            vs_nat_int=$(( native_ticks / optimized_ticks ))
            vs_nat_frac=$(( (native_ticks * 100 / optimized_ticks) % 100 ))
            vs_nat_pct=$(( native_ticks * 100 / optimized_ticks ))
        else
            vs_nat_int=0; vs_nat_frac=0; vs_nat_pct=0
        fi

        # Color: green >= 2.00x, white 0.95–2.00x (parity), red < 0.95x (regression).
        # 0.95× tolerates run-to-run jitter on flat ops (e.g. fall-through paths
        # where baseline and optimized translate to identical bytes).
        if   [[ $vs_nat_pct -ge 200 ]]; then spd_color="$GREEN"
        elif [[ $vs_nat_pct -ge 95  ]]; then spd_color="$WHITE"
        else                                  spd_color="$RED"
        fi

        short_bench="${bench#bench_}"
        label="${short_bench}/${func_name}"
        vs_nat_str="${vs_nat_int}.$(printf '%02d' $vs_nat_frac)x"

        printf "%-${COL_NAME}s %${COL_NUM}s %${COL_NUM}s ${spd_color}%${COL_SPD}s${NC}\n" \
            "$label" \
            "$native_ticks" \
            "$optimized_ticks" \
            "$vs_nat_str"

        total_vs_native_pct=$(( total_vs_native_pct + vs_nat_pct ))
        total_count=$(( total_count + 1 ))
    done
done

echo "$DIVIDER"

# Summary
if [[ $total_count -gt 0 ]]; then
    avg_vs_nat_int=$(( total_vs_native_pct / total_count / 100 ))
    avg_vs_nat_frac=$(( (total_vs_native_pct / total_count) % 100 ))

    echo ""
    echo -e "${BOLD}Summary (${total_count} benchmarks)${NC}"
    echo -e "  Avg spd_gain (JIT vs baseline):  ${GREEN}${avg_vs_nat_int}.$(printf '%02d' $avg_vs_nat_frac)x${NC}"
fi

# ── X87_FAST_ROUND comparison ────────────────────────────────────────────────
# bench_round exercises FISTP/FIST/FRNDINT with all four rounding modes.
# Compare JIT with the full RC dispatch chain (default) vs. single-instruction
# fast path (X87_FAST_ROUND=1, nearest-only, ~27% faster).
#
# WARNING: X87_FAST_ROUND is incorrect for code that uses FLDCW to set
# non-default rounding modes (e.g. Lua, floor-based coordinate math). Only
# use it when you have verified the target binary never changes the x87
# rounding mode.
ROUND_BIN="$BENCH_BIN/bench_round"
if [[ -x "$ROUND_BIN" ]]; then
    echo ""
    echo -e "${BOLD}X87_FAST_ROUND comparison${NC}  (bench_round)"
    echo -e "  ${YELLOW}NOTE: X87_FAST_ROUND is unsafe for code using FLDCW (e.g. Lua).${NC}"
    echo ""

    declare -A ROUND_DEFAULT
    declare -A ROUND_FAST

    while IFS=' ' read -r keyword name ticks; do
        [[ "$keyword" == "BENCH" ]] || continue
        ROUND_DEFAULT["$name"]="$ticks"
    done < <(run_bench "$ROUND_BIN" optimized)

    while IFS=' ' read -r keyword name ticks; do
        [[ "$keyword" == "BENCH" ]] || continue
        ROUND_FAST["$name"]="$ticks"
    done < <(run_bench "$ROUND_BIN" optimized X87_FAST_ROUND=1)

    DIVIDER2=$(printf '─%.0s' $(seq 1 $((COL_NAME + COL_NUM*2 + COL_SPD + 8))))
    printf "${BOLD}%-${COL_NAME}s %${COL_NUM}s %${COL_NUM}s %${COL_SPD}s${NC}\n" \
        "bench_round/func" "JIT" "fast_round" "speedup"
    echo "$DIVIDER2"

    for name in "${!ROUND_DEFAULT[@]}"; do
        def_ticks="${ROUND_DEFAULT[$name]:-0}"
        fast_ticks="${ROUND_FAST[$name]:-0}"

        if [[ $fast_ticks -gt 0 && $def_ticks -gt 0 ]]; then
            spd_pct=$(( def_ticks * 100 / fast_ticks ))
            spd_int=$(( spd_pct / 100 ))
            spd_frac=$(( spd_pct % 100 ))
        else
            spd_int=0; spd_frac=0; spd_pct=0
        fi

        if   [[ $spd_pct -ge 110 ]]; then spd_color="$GREEN"
        elif [[ $spd_pct -ge 90  ]]; then spd_color="$WHITE"
        else                               spd_color="$RED"
        fi

        printf "%-${COL_NAME}s %${COL_NUM}s %${COL_NUM}s ${spd_color}%${COL_SPD}s${NC}\n" \
            "round/$name" "$def_ticks" "$fast_ticks" \
            "${spd_int}.$(printf '%02d' $spd_frac)x"
    done

    echo "$DIVIDER2"
fi
