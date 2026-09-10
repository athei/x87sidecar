#!/usr/bin/env python3
"""Read an x87 boundary trace, retaining reservation gaps and thread identity."""

import argparse
import json
import math
import struct
from pathlib import Path

MAGIC = 0x0031435254373858
PITCH_HASH = 0x129250D0F7976B3F
RECORD_BYTES = 256


def f80(mantissa, sign_exp):
    exponent = sign_exp & 0x7fff
    sign = -1 if sign_exp & 0x8000 else 1
    if exponent == 0x7fff:
        return sign * math.inf if mantissa == 0x8000000000000000 else math.nan
    try:
        return sign * math.ldexp(float(mantissa), (exponent or 1) - 16383 - 63)
    except OverflowError:
        return sign * math.inf


def decode_record(data):
    words = struct.unpack('<32Q', data)
    locked, sequence, context, site, nzcv, fpcr = words[:6]
    cw, sw, tags = struct.unpack_from('<3H', data, 48)
    top = (sw >> 11) & 7
    slots = []
    raw_slots = []
    for physical in range(8):
        mantissa, sign_exp = struct.unpack_from('<QH', data, 54 + 10 * physical)
        raw_slots.append(f'{sign_exp:04x}:{mantissa:016x}')
        tag = (tags >> (2 * physical)) & 3
        slots.append(None if tag == 3 else f80(mantissa, sign_exp))
    return dict(sequence=sequence, context=f'0x{context:x}', pc=f'0x{site >> 32:x}',
                index=(site & 0xffffffff) >> 1, phase='exit' if site & 1 else 'entry',
                nzcv=f'0x{nzcv:x}', fpcr=f'0x{fpcr:x}', cw=f'0x{cw:04x}',
                sw=f'0x{sw:04x}', tags=f'0x{tags:04x}', top=top, st0=slots[top],
                physical=slots, raw=raw_slots, arm_gpr=[f'0x{x:x}' for x in words[17:]],
                locked=locked)


def read_trace(path):
    data = path.read_bytes()
    if len(data) < RECORD_BYTES:
        raise ValueError('missing header (capture did not finish)')
    magic, version, block_hash, pid, next_id, frozen, count, dropped = struct.unpack_from('<8Q', data)
    if magic != MAGIC or version != 1 or count != 65536:
        raise ValueError('unsupported trace header')
    if len(data) != RECORD_BYTES * (count + 1):
        raise ValueError('truncated or trailing capture data')
    records = []
    floor = max(0, next_id - count)
    for slot in range(count):
        offset = RECORD_BYTES * (slot + 1)
        raw = data[offset:offset + RECORD_BYTES]
        locked, sequence = struct.unpack_from('<2Q', raw)
        if sequence == 0:
            continue
        if locked or not floor < sequence <= next_id or (sequence - 1) % count != slot:
            raise ValueError(f'invalid publication at ring slot {slot}')
        records.append(decode_record(raw))
    records.sort(key=lambda rec: rec['sequence'])
    if min(next_id, count) - len(records) != dropped:
        raise ValueError('record count does not match dropped count')
    return dict(hash=f'0x{block_hash:016x}', pid=pid, events=next_id, frozen=bool(frozen),
                retained=len(records), dropped=dropped), records


def analyze(header, records):
    pending = {}
    pairs = 0
    replaced_entries = 0
    unpaired_exits = 0
    mismatches = []
    state_changes = []
    gap_resets = 0
    previous = None
    for record in records:
        if previous is not None and record['sequence'] != previous + 1:
            # A dropped reservation could belong to any thread. Do not
            # pair across it and manufacture a numeric disagreement.
            pending.clear()
            gap_resets += 1
        previous = record['sequence']
        context = record['context']
        if record['phase'] == 'entry':
            replaced_entries += context in pending
            pending[context] = record
            continue
        entry = pending.pop(context, None)
        if entry is None or entry['pc'] != record['pc']:
            unpaired_exits += 1
            continue
        # Only the known ten-instruction exp2 chain has this numeric invariant.
        # Partial replies and arbitrary selected blocks are still decoded.
        if int(header['hash'], 16) != PITCH_HASH or entry['index'] != 0 or record['index'] != 10:
            continue
        changed = [key for key in ('arm_gpr', 'nzcv', 'fpcr', 'cw', 'top')
                   if entry[key] != record[key]]
        if changed:
            state_changes.append(dict(entry_sequence=entry['sequence'],
                                      exit_sequence=record['sequence'], fields=changed))
        x, result = entry['st0'], record['st0']
        if x is None or not math.isfinite(x) or not -1022 <= x <= 1023:
            continue
        pairs += 1
        expected = 2.0 ** x
        if result is None or not math.isfinite(result) or not math.isclose(result, expected, rel_tol=1e-12, abs_tol=0):
            mismatches.append(dict(entry=entry, exit=record, expected=expected))
    return dict(checked_pairs=pairs, replaced_entries=replaced_entries,
                unpaired_exits=unpaired_exits, pending_entries=len(pending),
                mismatches=mismatches, state_changes=state_changes, gap_resets=gap_resets)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--json', action='store_true', help='emit the decoded capture and analysis')
    parser.add_argument('--limit', type=int, default=5, help='maximum mismatch details in text output')
    parser.add_argument('--check', action='store_true', help='fail if the exp2 pairs are absent or disagree')
    args = parser.parse_args()
    header, records = read_trace(args.trace)
    analysis = analyze(header, records)
    if args.json:
        print(json.dumps(dict(header=header, analysis=analysis, records=records), indent=2))
    else:
        print(json.dumps(header, sort_keys=True))
        print(f"checked_pairs={analysis['checked_pairs']} mismatches={len(analysis['mismatches'])} "
              f"replaced_entries={analysis['replaced_entries']} unpaired_exits={analysis['unpaired_exits']} "
              f"pending_entries={analysis['pending_entries']} state_changes={len(analysis['state_changes'])} "
              f"gap_resets={analysis['gap_resets']}")
        for change in analysis['state_changes'][:args.limit]:
            print(json.dumps(change))
        for mismatch in analysis['mismatches'][:args.limit]:
            print(json.dumps(mismatch, indent=2))
    return int(args.check and (not analysis['checked_pairs'] or bool(analysis['mismatches']) or bool(analysis['state_changes'])))


if __name__ == '__main__':
    raise SystemExit(main())
