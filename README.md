# x87sidecar

Faster x87 floating-point for x86 programs running under Apple's Rosetta 2 on
Apple Silicon, plus a profiler that sees through the translation. Most
x87-heavy x86 software is Windows software, so in practice x87sidecar is
wrapped around wine and pointed at old games.

It began as a fork of
[Lifeisawful/rosettax87_jit](https://github.com/Lifeisawful/rosettax87_jit),
an in-process JIT for stock Rosetta's x87 handlers, and has since diverged in
architecture and scope far enough to warrant its own name.

## What you get

| | |
|---|---|
| Faster x87 | Runs of x87 instructions become inline ARM64 through an IR pipeline with fusions and inline transcendentals, in place of stock Rosetta's much slower translation. See [Benchmarks](#benchmarks). |
| Two encodings Rosetta rejects | `DC D8` (`fcomp st(0)` alias) and legacy-mode `ARPL` trap under stock Rosetta. x87sidecar translates both, so winerosetta.dll is no longer needed. See [Compatibility](#compatibility-and-correctness). |
| Sampling profiler | Samples a running program without stopping it and reports guest x86 addresses and stacks, not ARM ones. See [Profiling](#profiling). |
| x87 block profiler | Per-block execution counters plus `profile_analyze`, which ranks the x87 code that costs the most and names the guest addresses it lives at. |
| Notarizable | In cooperative mode the sidecar needs no entitlements, so it can ship inside a signed, notarized app bundle. See [Attaching](#attaching-to-the-target). |
| Survives Rosetta updates | Hook sites are found by anchors, every assumption is checked against the installed runtime at startup, and `x87sidecar --probe` reports the verdict. |

## Quick start

Each [release](https://github.com/athei/x87sidecar/releases) ships two
`.tar.xz` assets, `x87sidecar` (no entitlements, notarizable) and
`x87sidecar_entitled` (can attach to any x86 binary without root). The two
are byte-identical except for the signature. Downloads carry the quarantine
attribute, so clear it first.

```bash
tar xf x87sidecar_entitled.tar.xz
xattr -d com.apple.quarantine x87sidecar_entitled
./x87sidecar_entitled --probe                  # is this Rosetta supported?
./x87sidecar_entitled ./some_x86_program args  # run it faster
```

The first launch asks for developer-tools authorization once per login
session. For Windows software use the prebuilt wine from
[athei/wine-build](https://github.com/athei/wine-build): set
`ROSETTA_X87_PATH` to the flat `x87sidecar` binary and its loader re-execs
every 32-bit process through `x87sidecar --cooperative`, which needs no
entitlements and no password.

## Benchmarks

Ticks from `scripts/run_benchmarks.sh` on an Apple M5 Max, macOS 27.0,
2026-09-03. The baseline column is stock Rosetta's own JIT translation of
the same loop (`X87_DISABLE_HOOK=1`, the sidecar attached but not
translating); running the binaries bare, with no sidecar at all, gives the
same baseline figures within noise. Steady-state execution only: every
cold-translated x87 block pays one IPC round trip first.

| benchmark | stock Rosetta | x87sidecar | speedup |
|---|---:|---:|---:|
| `load/fld_m64` | 9300800 | 453800 | 20.49x |
| `add/fadd_m64` | 23969400 | 455400 | 52.63x |
| `mul/fmul_m64` | 40994600 | 493000 | 83.15x |
| `div/fdiv_m64` | 62845800 | 454600 | 138.24x |
| `store/fstp_m64` | 7268600 | 492600 | 14.75x |
| `compare/fcomi` | 42550600 | 799000 | 53.25x |
| `unary/fsqrt` | 122184400 | 495400 | 246.63x |
| `fsin/fsin_midrange` | 117391400 | 2641600 | 44.43x |
| `fyl2x/fyl2x` | 116134400 | 2627000 | 44.20x |
| `fpatan/fpatan` | 45748200 | 7251200 | 6.30x |
| `dot_product/dot_product_n4` | 147243800 | 1059800 | 138.93x |
| `fusion_fld_arith_fstp/fld_fmul_fstp` | 50546200 | 471400 | 107.22x |
| `fstp_fld/chain_8x` | 182530000 | 2082000 | 87.67x |
| `fbstp/fbstp_small` | 43726200 | 25734000 | 1.69x |
| `frstor/frstor_loop` | 4980200 | 12904600 | 0.38x |

The harness's arithmetic mean over all 179 micro-benchmarks is 61x. These
are tight loops of x87 instructions and measure the translator, not a
program: a game spends most of its time elsewhere, and the gain it sees
depends on how much of it is x87. The state-management instructions are not
the target and some are slower on their own, `frstor` above among them.
The `fbstp` row measures the sidecar's own translation, which now runs only
with `X87_STOCK_BCD=0`: by default a block containing `fbld` or `fbstp`
goes to stock, so it runs at the stock figure.

## How it works

The original in-process design mapped its own executable pages and patched
stock's `translate_insn` to branch into them. That cannot be made safe: when
a signal arrives, Rosetta looks the thread's ARM64 pc up in its own
translation tables, code from an injected dylib is unknown to them, and the
process dies with:

```
rosetta error: no code fragment associated with the given arm pc
```

The crash rate grows with the JIT's success, since a busier JIT means more
time spent at an unknown pc. So the translator runs in a separate native
arm64 process, the sidecar, and the only ARM64 left inside the target is a
small IPC stub written once at install time into the page padding at the
tail of stock's translation-output buffer: pages stock already registered as
translated code, so the reverse lookup covers them for free. The prologue of
`translate_insn` is overwritten with a branch into the stub, and the
displaced bytes are preserved there for the fall-through path.

```
┌──────────────────── wine + x86 app (under Rosetta) ─────────────────────┐
│                                                                          │
│   x86 stream ─► stock translate_insn (prologue patched)                  │
│                              │                                           │
│                       branch into our stub                               │
│                       (in page padding, written once at install)         │
│                              │                                           │
│                       opcode in x87 ranges?  ◄── two bounds checks       │
│                              │                                           │
│                       no ◄───┴───► yes                                   │
│                       │             │                                    │
│                       ▼             ▼                                    │
│                resume stock     mach_msg2 to sidecar                     │
│                via preserved          │                                  │
│                prologue bytes         │                                  │
└───────────────────────────────────────┼──────────────────────────────────┘
                                        │
                                        ▼
┌──────────────────────── x87sidecar (native arm64) ──────────────────────┐
│                                                                          │
│   IR translator ─► peephole fusion / FMA / inline transcendentals        │
│                              │                                           │
│                              ▼                                           │
│                  ARM64 bytes (handled) | "unhandled"                     │
│                              │                                           │
│                              ▼ reply                                     │
│           handled  → stub returns to translate_insn's caller             │
│           unhandled → stub falls through to stock                        │
│                                                                          │
│   per-call: mach_vm_read of source, mach_vm_write of result              │
│   shared (mach_vm_remap'd, copy=FALSE): per-block exec counters          │
└──────────────────────────────────────────────────────────────────────────┘
```

The stub's runtime work is two bounds checks against the x87 opcode ranges.
In range, it sends a Mach message to the sidecar and either returns the
sidecar's bytes to `translate_insn`'s caller or falls through to stock when
the sidecar declines. Out of range, it runs the preserved prologue and
branches back. Every cold-translated x87 block pays one IPC round trip; once
stock has installed the bytes the sidecar is off the hot path, so
steady-state speed is unaffected. The sidecar caches the block's IR and the
thread-context layout across requests and fuses each reply with the next
receive, which leaves 5 traps per request. The `x87sidecar` binary plays
both roles: it launches the target, attaches, installs the hooks, then drops
into its receive loop. [docs/internals.md](docs/internals.md) has the
details.

## Profiling

Both profilers are enabled by naming an output file; neither costs anything
when unset. Both always append `.<pid>` to the provided path, using the pid
of the process the sidecar is attached to, not the sidecar's own. For example,
`/tmp/game.prof` becomes `/tmp/game.prof.12345`. Paths are otherwise literal;
there is no placeholder or extension parsing. Wine starts one cooperative
sidecar per i386 process with the same environment, so the suffix keeps
those processes from truncating one another's profiles.

### Sampling profiler: where the guest was

```bash
X87_SAMPLE=/tmp/game.prof ./x87sidecar_entitled ./program
```

The sampler reads a running thread's ARM pc and resolves it to the guest x86
pc it was translated from, using the per-fragment instruction maps Rosetta
keeps for its own use. Nothing suspends the target: `thread_get_state` and
`mach_vm_read` both work on a running task, so the target pays only memory
read traffic and the rate is bounded by the sidecar's own CPU. It latches
onto the thread that runs the program's main image, which it finds by
reading the guest's own image headers, PE under wine or a plain Mach-O, then
follows that thread alone. Stacks come from walking the guest frame-pointer
chain; return addresses on the guest stack are already guest addresses, so
only the leaf needs resolving.

The profile is one self-describing text file, rewritten in full every report
interval so it can be read while the target runs, and written again on every
catchable exit. It carries its own module map, so symbolising needs nothing
but the program's binaries and debug info.

| section | contents |
|---|---|
| header | settings, the thread it latched onto, `effective_hz`, `missed_ticks`, `samples_dropped`, `avg_us`, `avg_depth`, cache statistics |
| `[latch_history]` | every latch, unlatch and discard, so a profile that threw samples away says so |
| `[threads]` | per thread: samples, resolved, in range, latched |
| `[modules]` | base, size, kind (`pe`, `macho`, `anon`, `slice`), state, host path of every image a sample touched |
| `[leaves]` | exclusive histogram of guest pcs |
| `[host_leaves]` | samples with no guest pc, kept as host pcs with the reason (runtime code, no fragment, before the map's first boundary) |
| `[host_syscalls]` | the guest syscall a host-only sample was blocked in, recovered from `x16` at the runtime's syscall dispatcher |
| `[stacks]` | folded inclusive stacks, `root;...;leaf count` |

A run leaves `<file>.<pid>` and `<file>.<pid>.windows`, the latter one record per report
interval holding only that interval's samples, each closed by an
`end_window` line so a record cut short by SIGKILL can be dropped. The
cumulative profile is the sum of the windows. A sample costs about 10 us,
roughly 1% of one core per kHz, and the header records the rate actually
achieved.

| knob | flag | default | effect |
|---|---|---|---|
| `X87_SAMPLE=<file>` | `--sample=<file>` | off | write the profile to `<file>.<pid>` |
| `X87_SAMPLE_HZ=N` | `--sample-hz=N` | 10000 | rate for the latched thread |
| `X87_SAMPLE_SWEEP_HZ=N` | `--sweep-hz=N` | 1000 | rate at which all threads are swept while looking for one to latch onto |
| `X87_SAMPLE_REPORT=SECS` | | 10 | rewrite interval and window size |
| `X87_SAMPLE_WINDOWS=0` | | on | stop writing `<file>.<pid>.windows` |
| `X87_GUEST_RANGE=LO-HI` | `--guest-range=LO-HI` | detected | pin the guest range that marks the thread worth profiling |
| `X87_NO_UNWIND=1` | `--no-unwind` | off | leaf pcs only, about half the per-sample cost |

The environment wins over the flags, so an app bundle can enable sampling
without touching argv.

### Block profiler: what the x87 code costs

```bash
X87_PROFILE=/tmp/game.x87 ./x87sidecar_entitled ./program
# Replace 12345 with the target pid printed in the output path.
./build/bin/profile_analyze /tmp/game.x87.12345 --rank-by emit --hot-addrs 50
```

While translating, the sidecar writes each block's IR to the file the first
time it sees it. The execution counters live in a page the sidecar allocates
and shares into the target, where the emitted code increments them with a
single atomic add, so at exit the sidecar reads its own mapping and appends
the counts. `profile_analyze` then re-translates every x87 pattern three
ways (production, IR pipeline off, IR gate forced) and ranks patterns by
execution-weighted ARM output, which separates "the translator emits too
much for this" from "this just runs a lot". `--hot-addrs N` collapses the
cost onto guest block-entry addresses for mapping onto the program's
functions, `--frag-rows N` lists blocks whose x87 runs are split by
bridgeable integer instructions, and `--dump-block N` or
`--dump-block-by-hash 0xH` prints a block's IR. Blocks are identified by a
content hash that is stable across launches and host versions; the same
hash keys the per-block knobs in [Configuration](#configuration).

## Attaching to the target

The sidecar needs the target's Mach task port to read its memory and plant
the stub. There are two ways to get it.

| | default | `--cooperative` |
|---|---|---|
| how | `task_for_pid` + `ptrace(PT_ATTACHEXC)` around the target's exec | the target hands over its task and thread ports on a per-pid Mach service named by `X87_SIDECAR_BOOTSTRAP`, then blocks until the sidecar replies |
| needs | `x87sidecar_entitled` (`cs.debugger` + `get-task-allow`, one developer-tools password per login) or root | nothing; the flat `x87sidecar` passes notarization |
| target | any x86 binary | must perform the handshake ([`coop_proto.h`](rosetta_loader/src/coop_proto.h), plain C) |
| used by | the test and benchmark harness, CI under `sudo` | the prebuilt wine, app bundles |

Both modes converge on the same install and receive loop. Stock wine does
not perform the handshake; the patch lives on the `cx-*-patched` branches of
[athei/wine](https://github.com/athei/wine), which is what
[athei/wine-build](https://github.com/athei/wine-build) packages. The
handshake reply carries the code ranges the sidecar patched, and the target
invalidates them from its own thread: a cooperative attach happens after
Rosetta init, when `translate_insn` is already hot in the instruction cache
and a cross-process flush is not reliable.

When the hook cannot be set up, the two modes differ. The default attach
refuses to launch the target, so a test or benchmark run never reports
results that were not hooked. Cooperative mode, which wine sends every
32-bit process through, lets the program run when hook setup fails. A
failure before launch, such as an unsupported Rosetta or a sidecar that
could not start, execs the target in place, keeping its pid, with the
arguments and environment wine passed: `X87_SIDECAR_BOOTSTRAP` is not set
for it and `ROSETTA_DISABLE_AOT` keeps whatever value it inherited. A
failure after the target has handed over its ports, by an error or an
exception, puts back any entry patch already written and releases the
target without the hook.

Either way a program running without the hook is easy to miss, since the
only symptom is that an x87-heavy game is slow. So both paths print a
banner on stderr:

```
################################################################
WARNING: X87SIDECAR COULD NOT HOOK ROSETTA: UNSUPPORTED ROSETTA.
PROCESS 12345 IS RUNNING WITHOUT X87 ACCELERATION: /path/to/wine
32-BIT GAMES THAT USE X87 MATH WILL BE MUCH SLOWER.
################################################################
```

The reason on the second line varies; the substring `RUNNING WITHOUT X87
ACCELERATION` does not, so logs can be searched for it. A sidecar that
gives up before the target's handshake request arrived, when the target
may not be running at all, prints `x87 hook not installed for <program>`
instead.

Two switches exercise these paths. `X87_FORCE_UNSUPPORTED=1` makes the
loader treat the installed Rosetta as unsupported. `X87_FORCE_ABANDON` makes
the sidecar give up after hooking: `entry` after the `translate_insn` entry
patch, `decode` after the `decode_opcode` step, `thread` with an exception
where the receive thread is spawned. `scripts/test_unhooked_fallback.py`
checks the first without privileges.

## Compatibility and correctness

Nothing in the tree is tied to a macOS or Rosetta build number. At startup
the loader locates what it patches by anchors that survive a rebuild, checks
the assumptions the emitted code relies on against the installed runtime,
and refuses a runtime that fails a check rather than patching guessed
addresses; [Attaching](#attaching-to-the-target) says what then happens to
the program. `x87sidecar --probe` prints that report and exits 0 only when
every feature is supported; run it first after a macOS update. CI runs it
on the current `macos-26` runner before the test suite, and it runs on
macOS 27.

x87 coverage: arithmetic, memory operands, comparisons, the full
transcendental set (`fsin`, `fcos`, `fsincos`, `fpatan`, `f2xm1`, `fyl2x`,
`fyl2xp1`, `fptan`, `fprem`, `fprem1`, `fxtract`, `fscale`), state
management (`fldenv`, `fstenv`, `fxsave`, `fxrstor`, `fsave`, `frstor`,
`fclex`, `finit`, `fldcw`, `fstsw`) and the fusion patterns 3D-game
pipelines produce. The test suite runs the same self-checking binaries under
stock Rosetta and under the sidecar; anything stock gets right, the sidecar
has to get right too. FMA contraction is opt-in (`X87_ENABLE_FMA_CONTRACT=1`) because real x87 rounds the product
before the add, and at the 53-bit precision Windows processes run at the
unfused form is the exact one.

The sidecar holds x87 values as binary64 rather than the 80-bit extended
format. At the 53-bit precision control Windows uses, arithmetic results
match the hardware as long as intermediate values stay inside the binary64
exponent range; at 64-bit or 24-bit precision control, for 64-bit
integers above 2^53 that meet arithmetic and for sequences that carry extra
precision from one instruction to the next, such as the `fyl2x`-based
`pow`, they do not. A `fild m64; fistp m64` copy moves its eight bytes
unchanged, within a block or across blocks, unless the value is still on
the stack while a reply with floating-point work runs, or sits inside a
mixed run that the fusion does not match with the IR off; then a value
above 2^53 is rounded. An adjacent `fld m80; fstp m80` pair is copied
unchanged.
[docs/internals.md](docs/internals.md#precision-and-the-floating-point-environment)
lists the measured differences, including how x87 code interacts with MXCSR.

When an asynchronous signal lands inside a translated run, Rosetta steps to the next instruction-map entry and takes
the guest state from there, so every instruction the sidecar emits is one
the runtime's decoder knows, control flow only goes forward, and a run is
answered with one reply so the map has entries only where the state is
complete. The private binary64 register file is converted to Rosetta's native
80-bit layout at those boundaries, except in replies that only move
integers, which work on the 80-bit registers directly so an int64 held on
the stack across branches stays exact. The signal tests check both arithmetic
under a SIGUSR1 storm and reading or replacing the saved x87 context in
64-bit and 32-bit compatibility mode.

A memory fault in translated code can only be rolled back to the start of
its reply, and the sidecar's replies change state before their first guest
access. So by default any guest memory fault inside sidecar-translated x87
code, such as a Windows program touching a guard page or a write-watched
buffer through an x87 load or store, aborts the process with `rosetta
error: unexpectedly need to EmulateForward on a synchronous exception`,
where stock Rosetta would call the program's handler. A program that dies
with that message can set `X87_PRECISE_FAULTS=1`. The sidecar then probes
the memory operand of each reply's first x87 instruction before the reply
changes any state, and answers runs of integer loads and stores one
instruction per reply, so a fault there reaches the program's SIGSEGV or
SIGBUS handler with the same context stock Rosetta gives it, except that a
probed store into a `PROT_NONE` page is reported as a read fault. It is off
by default because of what it costs: an x87 store at the start of a reply
into write-combined memory, as when a game fills a mapped vertex buffer,
becomes 23 to 40 times slower, loops of x87 arithmetic on memory operands
lose up to 7 percent, and nested `fild m64; fistp m64` copies
(`fild; fild; fistp; fistp`) take 4.5 times as long. Even with it on, a fault in a later instruction of a
floating-point run still aborts the process, and one in the store of an
adjacent `fild m64; fistp m64` copy is reported at the `fild`;
[docs/internals.md](docs/internals.md#memory-faults-inside-emitted-code)
explains why.

Two encodings real hardware runs are missing from Rosetta's decode tables,
so a program containing them traps under stock Rosetta. A second small stub
on `decode_opcode` substitutes an encoding the decoder accepts, entirely
inside the target and without modifying guest memory.

| encoding | what it is | seen in |
|---|---|---|
| `DC D8` | undocumented alias of `fcomp st(0)` | WoW 1.12, Lua's "table index is NaN" check |
| `63 /r` | `ARPL r/m16, r16`, legacy mode only | WoW 1.12, obfuscated code reached after login |

Both deep-dives are in [docs/internals.md](docs/internals.md).

Tested live against TurtleWoW (a World of Warcraft 1.12 client) and Call of
Duty 2 under CrossOver. It is hardened against the workloads it has seen and
may need work on others.

## Building and testing

```bash
cmake -B build
cmake --build build
```

This produces `build/bin/x87sidecar`, `build/bin/x87sidecar_entitled`, the
tools, and the x86-64 test and benchmark binaries.

```bash
bash scripts/run_tests.sh                # build + all phases
bash scripts/run_tests.sh --no-build     # skip the build
bash scripts/run_tests.sh --native-only  # stock Rosetta baseline only
bash scripts/run_tests.sh test_arith     # one test
python3 scripts/test_profile_paths.py    # concurrent profiler output paths
python3 scripts/test_unhooked_fallback.py  # unhooked fallback, no privileges
bash scripts/run_benchmarks.sh           # build + benchmark table
```

The harness runs 105 self-checking x86-64 test binaries under stock Rosetta
and then under the sidecar in ten configurations (default, IR off, fusions
off, hook bypassed, FMA contraction on, clamped register pool, pressure
relief off, fast rounding, bridging off, bridging v2 off), plus a
cooperative-attach smoke test, the two decoder tests three ways, the inline
BCD translators with `X87_STOCK_BCD=0`, an allocation fault-injection phase,
the FTZ/DAZ test with and without `X87_IGNORE_FTZ_DAZ` (`test_x87_ftz_daz`
runs only in that phase), the guest-flags test with the `fcomi` and the
`fcmov` families handed to stock, the memory-fault test with
`X87_PRECISE_FAULTS=1` in seven configurations and under allocation
failures, and once without it, where it must abort (`test_x87_sync_fault`
runs only in that phase), and an IR replay phase. The stock run is
the baseline: a case that fails there is reported XFAIL in later phases
rather than gating, and the stock run itself may only fail for tests listed
in `KNOWN_STOCK_DIVERGENCE`. The harness uses the default attach path, so
it needs `x87sidecar_entitled` or root.

When the translator runs out of scratch registers it declines the request
and stock Rosetta translates that instruction, which keeps a program running
on a register-pressure bug. The harness sets `X87_STRICT_ALLOC=1` so that
the same bug fails a test instead, and its fault-injection phase runs a set
of tests with `X87_FAIL_ALLOC_EVERY=N`, which fails every Nth allocation, to
check that the declined requests leave every result unchanged.

| tool | purpose |
|---|---|
| `profile_analyze` | rank and dump the blocks in an `X87_PROFILE` capture, see [Profiling](#profiling) |
| `aotinvoke` | translate a serialized IR module through the sidecar's translator against the installed libRosettaRuntime, in process |
| `scripts/inspect_function.sh` | extract a function from a test binary, translate it with `aotinvoke`, disassemble the result |
| `ir_pressure_replay` | replay a captured block under a clamped register-pressure gate and report splits and emit |
| `rollback_diff` | replay a captured block with the IR-gate rollback off and on and diff the ARM |
| `scripts/run_fusion_sweep.sh` | enable fusions one at a time, then cumulatively, to find the one that breaks a test |

## Configuration

Knobs are environment variables read once at startup. `x87sidecar --help`
prints the complete list with defaults; the tables list the ones worth
knowing. Bracketed values are defaults.

| variable | effect |
|---|---|
| `X87_ENABLE_FMA_CONTRACT=1` | fold `fmul` + `fadd`/`fsub` into one FMA [off, see above] |
| `X87_IGNORE_FTZ_DAZ=1` | keep x87 denormals when the program sets MXCSR FTZ or DAZ, as real x87 does; costs 6 to 13 instructions per x87 run (plus up to two taken branches when the x87 stack is not empty), and about 20 ns per run while FTZ or DAZ is set. On Macs without FEAT_AFP, a signal that lands inside an x87 run can make the rest of that run give wrong results, most likely flushed denormals [off, see [docs/internals.md](docs/internals.md#precision-and-the-floating-point-environment)] |
| `X87_FAST_ROUND=1` | skip rounding-mode dispatch; unsafe for code that uses `fldcw`. `=2` skips it only in blocks with no control-word writer, still speculative [off] |
| `X87_ENABLE_BRIDGE=0` | disable carrying one IR run across short `mov`/`lea` gaps between x87 segments [on] |
| `X87_BRIDGE_V2=0` | disable bridging across flag-writing ALU whose flags Rosetta's liveness proves dead [on] |
| `X87_ENABLE_IR_SPLIT=0`, `X87_ENABLE_IR_REMAT=0` | disable register-pressure relief: splitting over-pressure runs, sinking long-lived values [on] |
| `X87_NATIVE_TRANSPORT=0` | put replies made only of integer loads and stores (`fild`, `fistp`, ...) back through the binary64 register file, which rounds int64 values of 2^53 or more held on the stack; for bisecting [on] |
| `X87_NATIVE_SLOTS=1` | keep each x87 register's working binary64 value inside its native 80-bit slot and convert slots one at a time, so registers a reply does not use keep their 80-bit values; ignored under `X87_IGNORE_FTZ_DAZ=1` [off: every reply with floating-point work converts all eight registers to binary64 and back] |
| `X87_PRECISE_FAULTS=1` | for a program that aborts with "unexpectedly need to EmulateForward on a synchronous exception": probe the memory operand of each reply's first x87 instruction and answer runs of integer loads and stores one instruction per reply, so a memory fault there reaches the program's handler; costs up to 7 percent on x87 loops, makes an x87 store into write-combined memory 23 to 40 times slower, and faults later in a floating-point run still abort [off: any memory fault in translated x87 code aborts] |
| `X87_DISABLE_X87_IR=1` | direct translator only, no IR pipeline |
| `X87_DISABLE_ALL_FUSIONS=1`, `X87_DISABLE_FUSIONS=f1,f2` | disable every fusion, or the named ones (`--help` lists the names) |
| `X87_DISABLE_FUSIONS=fild_fistp,fld_fstp_m80` | send adjacent `fld m80; fstp m80` pairs, and `fild m64; fistp m64` pairs in runs with other x87 work, through binary64 instead of copying the bytes exactly; replies of only integer loads and stores stay exact through native transport [both on] |
| `X87_DISABLE_SINGLE_FAST=1`, `X87_DISABLE_CACHE=1` | disable the single-op fast path, the cross-instruction register cache |
| `X87_STRICT_ALLOC=1` | assert when the translator runs out of scratch registers, instead of declining the request to stock with one `failed in block hash=0xH` line [off; the test harness sets it] |

Per-block knobs, keyed by the content hash `profile_analyze` prints, for
bisecting a suspected miscompile in a live workload or working around one at
no steady-state cost:

| variable | effect |
|---|---|
| `X87_STOCK_HASH_LIST=0xH,...` | hand the listed blocks to stock Rosetta entirely |
| `X87_STOCK_OPS=f2xm1,...` | hand every block containing one of the opcodes to stock |
| `X87_STOCK_BCD=0` | translate blocks containing `fbld` or `fbstp` in the sidecar; by default they go to stock, whose 80-bit registers keep all 18 BCD digits |
| `X87_LOG_HASH_LIST=0xH,...` | log every translate request of the listed blocks with an uptime stamp |
| `X87_DIAG_DIR=<dir>` | mirror those logs to `<dir>/x87diag.<pid>.log`, for hosts that lose stdout |
| `X87_ALWAYS_NONE=1` | the sidecar declines every request; separates a JIT bug from an IPC one |
| `X87_DISABLE_HOOK=1` | skip the `translate_insn` patch, the benchmark baseline |
| `X87_NO_DECODE_HOOK=1` | skip the `decode_opcode` patch, so `DC D8` and `ARPL` trap as under stock |

For an execution trace of one IR block, set `X87_TRACE_BLOCK=0xH` and
optionally `X87_TRACE_OUTPUT=/path/prefix` (default `/tmp/x87trace`). The
sidecar records native x87 state, ARM X0 through X14, NZCV and FPCR at each
handled reply's entry and exit. A shared ring retains the last 65,536
records, with per-thread identity and dropped-reservation reporting.
It allocates 16 MiB only when the selected block is encountered, and writes
`<prefix>.<target-pid>.x87trace` at exit. Existing files are never overwritten.

`X87_TRACE_STOP_NEGATIVE=1` freezes and writes the ring when ST(0) is negative
at a reply ending the selected block. This only freezes the diagnostic
buffer; the guest continues with the same result. For the CoD2 pitch hash
`0x129250d0f7976b3f`, `python3 tools/x87_trace_analyze.py capture.x87trace`
checks complete entry/exit pairs against `2^input` and reports the first
disagreements. Other hashes are decoded without assuming that invariant.
Tracing changes execution timing; a clean trace is not proof that the live
bug is fixed. It does not change stock exclusions or repair recorded values.

Loader and sidecar diagnostics:

| variable | effect |
|---|---|
| `X87_LOGS=1` | verbose loader logging |
| `X87_LOG_THROUGHPUT=1` | requests per second, traps per request, cache hit rates and allocation declines every 2 s |
| `X87_LOG_OPS=1` | one line per translated op; high volume, for freeze bisects |
| `X87_NO_IR_CACHE=1`, `X87_NO_TCO_CACHE=1` | re-read the IR array or the thread-context layout on every request |
| `X87_NO_PREAUTH=1` | skip acquiring the developer-tools right before launch (default attach only) |
| `X87_FORCE_UNSUPPORTED=1` | treat the installed Rosetta as unsupported at launch; a cooperative target then runs without the hook |
| `X87_FAIL_ALLOC_EVERY=N` | test-only fault injection: every Nth scratch register allocation fails as if the pool were empty, to exercise the decline path [off] |
| `X87_FORCE_ABANDON=entry\|decode\|thread` | give up after the named hook step; a cooperative target is rolled back and runs without the hook |

## License

MIT.
