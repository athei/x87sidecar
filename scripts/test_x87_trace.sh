#!/usr/bin/env bash
# Check native-state tracing under signal recovery, ring wraps and contention.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
BUILD=${1:-"$ROOT/build"}
WORK=$(mktemp -d "${TMPDIR:-/tmp}/x87-trace-test.XXXXXX")
trap 'rm -rf "$WORK"' EXIT

for fixture in test_x87_signal_context test_x87_trace_threads test_x87_trace_freeze; do
    selected_hash=0x129250d0f7976b3f
    if [[ $fixture == test_x87_trace_freeze ]]; then
        selected_hash=0x1e7ad60db09bb6e8  # fld1; fchs, result is deliberately negative
    fi
    output=$(
        for setting in "${!X87_@}"; do unset "$setting"; done
        export X87_NO_PREAUTH=1 X87_TRACE_BLOCK="$selected_hash"
        export X87_TRACE_OUTPUT="$WORK/$fixture" X87_TRACE_STOP_NEGATIVE=1
        "$BUILD/bin/x87sidecar_entitled" "$BUILD/bin/tests/$fixture" 2>&1
    ) || { printf '%s\n' "$output"; exit 1; }
    if grep -q 'FAIL' <<<"$output" || ! grep -q '^PASS' <<<"$output"; then
        printf 'FAIL  trace fixture %s\n%s\n' "$fixture" "$output"
        exit 1
    fi
    captures=("$WORK/$fixture".*.x87trace)
    [[ ${#captures[@]} -eq 1 && -f ${captures[0]} ]]
    if [[ $fixture == test_x87_trace_freeze ]]; then
        python3 - "$ROOT" "${captures[0]}" <<'PY'
import runpy, sys
from pathlib import Path
trace = runpy.run_path(sys.argv[1] + '/tools/x87_trace_analyze.py')
header, records = trace['read_trace'](Path(sys.argv[2]))
assert header['frozen'] and header['events'] == 2 and header['retained'] == 2, header
assert records[-1]['phase'] == 'exit' and records[-1]['st0'] == -1, records
PY
    else
        python3 "$ROOT/tools/x87_trace_analyze.py" "${captures[0]}" --check
    fi
    if [[ $fixture == test_x87_trace_threads ]]; then
        python3 - "$ROOT" "${captures[0]}" <<'PY'
import runpy, sys
from pathlib import Path
trace = runpy.run_path(sys.argv[1] + '/tools/x87_trace_analyze.py')
header, records = trace['read_trace'](Path(sys.argv[2]))
assert not header['frozen'], header
assert header['events'] > 65536, header
assert len({r['context'] for r in records}) == 8, 'ring tail lost a producer'
PY
    fi
    printf 'PASS  x87_trace %s\n' "$fixture"
done
