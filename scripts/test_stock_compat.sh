#!/usr/bin/env bash
# Verify actual dispatch using the signal-context fixture's CoD2 IR stream.
# Every target-block execution counter must remain zero with the fallback,
# while other blocks still execute sidecar code. Opting out must reverse it.
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
BUILD=${1:-"$ROOT/build"}
LOADER="$BUILD/bin/x87sidecar_entitled"
FIXTURE="$BUILD/bin/tests/test_x87_signal_context"
ANALYZER="$BUILD/bin/profile_analyze"
HASH=0x129250d0f7976b3f
WORK=$(mktemp -d "${TMPDIR:-/tmp}/x87-stock-compat.XXXXXX")
trap 'rm -rf "$WORK"' EXIT

run_case() {
    local name=$1 accelerated=$2
    shift 2
    local output dump
    # Do not inherit another run's X87_PROFILE path or diagnostic knobs.
    # The prefix expansion is supported by macOS's Bash 3.2.
    output=$(
        for setting in "${!X87_@}"; do unset "$setting"; done
        export X87_NO_PREAUTH=1 X87_PROFILE="$WORK/$name.prof"
        env "$@" "$LOADER" "$FIXTURE" 2>&1
    ) || { printf '%s\n' "$output"; return 1; }
    if [[ $(grep -c '^PASS  signal context ' <<<"$output") -ne 20 ]] ||
       grep -q '^FAIL' <<<"$output"; then
        printf 'FAIL  stock_compat %s: signal-context fixture failed\n%s\n' "$name" "$output"
        return 1
    fi
    # Profiling paths include the target PID on current main.
    local profiles=("$WORK/$name.prof"*)
    if [[ ${#profiles[@]} -ne 1 || ! -f ${profiles[0]} ]]; then
        printf 'FAIL  stock_compat %s: missing or ambiguous profile\n' "$name"
        return 1
    fi
    dump=$("$ANALYZER" "${profiles[0]}" --dump-block-by-hash "$HASH" 2>&1)
    if ! awk -v accelerated="$accelerated" '
        /^# block_id=/ {
            found++
            for (i = 1; i <= NF; i++) {
                if ($i ~ /^exec_count=/) {
                    split($i, pair, "=")
                    count += pair[2]
                }
            }
        }
        END { exit !(found > 0 && (accelerated ? count > 0 : count == 0)) }
    ' <<<"$dump"; then
        printf 'FAIL  stock_compat %s: wrong dispatch\n%s\n' "$name" "$dump"
        return 1
    fi
    if ! grep -qE 'wrote [0-9]+ block counters; max=[1-9][0-9]*' <<<"$output"; then
        printf 'FAIL  stock_compat %s: no other accelerated blocks\n%s\n' "$name" "$output"
        return 1
    fi
    printf 'PASS  stock_compat %s\n' "$name"
}

run_case default 0
run_case dummy_list 0 X87_STOCK_HASH_LIST=0x1
run_case disabled 1 X87_DISABLE_STOCK_COMPAT=1
run_case explicit_list 0 X87_DISABLE_STOCK_COMPAT=1 X87_STOCK_HASH_LIST="$HASH"
