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
when unset.

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

A run leaves `<file>` and `<file>.windows`, the latter one record per report
interval holding only that interval's samples, each closed by an
`end_window` line so a record cut short by SIGKILL can be dropped. The
cumulative profile is the sum of the windows. A sample costs about 10 us,
roughly 1% of one core per kHz, and the header records the rate actually
achieved.

| knob | flag | default | effect |
|---|---|---|---|
| `X87_SAMPLE=<file>` | `--sample=<file>` | off | enable and name the profile |
| `X87_SAMPLE_HZ=N` | `--sample-hz=N` | 10000 | rate for the latched thread |
| `X87_SAMPLE_SWEEP_HZ=N` | `--sweep-hz=N` | 1000 | rate at which all threads are swept while looking for one to latch onto |
| `X87_SAMPLE_REPORT=SECS` | | 10 | rewrite interval and window size |
| `X87_SAMPLE_WINDOWS=0` | | on | stop writing `<file>.windows` |
| `X87_GUEST_RANGE=LO-HI` | `--guest-range=LO-HI` | detected | pin the guest range that marks the thread worth profiling |
| `X87_NO_UNWIND=1` | `--no-unwind` | off | leaf pcs only, about half the per-sample cost |

The environment wins over the flags, so an app bundle can enable sampling
without touching argv. `%p` in the `X87_SAMPLE` or `X87_PROFILE` path expands
to the pid of the process the sidecar is attached to, not the sidecar's own.
Wine starts one cooperative sidecar per i386 process and all of them inherit
the same environment, so a fixed path would let the injector's sidecar
truncate the game's profile, or the reverse.

### Block profiler: what the x87 code costs

```bash
X87_PROFILE=/tmp/game.x87 ./x87sidecar_entitled ./program
./build/bin/profile_analyze /tmp/game.x87 --rank-by emit --hot-addrs 50
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

## Compatibility and correctness

Nothing in the tree is tied to a macOS or Rosetta build number. At startup
the loader locates what it patches by anchors that survive a rebuild, checks
the assumptions the emitted code relies on against the installed runtime,
and refuses a runtime that fails a check rather than patching guessed
addresses. `x87sidecar --probe` prints that report and exits 0 only when
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

The emitted code survives asynchronous signals: when one lands inside a
translated run, Rosetta steps to the next instruction-map entry and takes
the guest state from there, so every instruction the sidecar emits is one
the runtime's decoder knows, control flow only goes forward, and a run is
answered with one reply so the map has entries only where the state is
complete. `tests/test_x87_signal_storm.c` pins this under a SIGUSR1 storm.

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
bash scripts/run_benchmarks.sh           # build + benchmark table
```

The harness runs 89 self-checking x86-64 test binaries under stock Rosetta
and then under the sidecar in ten configurations (default, IR off, fusions
off, hook bypassed, FMA contraction on, clamped register pool, pressure
relief off, fast rounding, bridging off, bridging v2 off), plus a
cooperative-attach smoke test, the two decoder tests three ways and an IR
replay phase. The stock run is the baseline: a case that fails there is
reported XFAIL in later phases rather than gating, and the stock run itself
may only fail for tests listed in `KNOWN_STOCK_DIVERGENCE`. The harness uses
the default attach path, so it needs `x87sidecar_entitled` or root.

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
| `X87_FAST_ROUND=1` | skip rounding-mode dispatch; unsafe for code that uses `fldcw`. `=2` skips it only in blocks with no control-word writer, still speculative [off] |
| `X87_ENABLE_BRIDGE=0` | disable carrying one IR run across short `mov`/`lea` gaps between x87 segments [on] |
| `X87_BRIDGE_V2=0` | disable bridging across flag-writing ALU whose flags Rosetta's liveness proves dead [on] |
| `X87_ENABLE_IR_SPLIT=0`, `X87_ENABLE_IR_REMAT=0` | disable register-pressure relief: splitting over-pressure runs, sinking long-lived values [on] |
| `X87_DISABLE_X87_IR=1` | direct translator only, no IR pipeline |
| `X87_DISABLE_ALL_FUSIONS=1`, `X87_DISABLE_FUSIONS=f1,f2` | disable every fusion, or the named ones (`--help` lists the names) |
| `X87_DISABLE_SINGLE_FAST=1`, `X87_DISABLE_CACHE=1` | disable the single-op fast path, the cross-instruction register cache |

Per-block knobs, keyed by the content hash `profile_analyze` prints, for
bisecting a suspected miscompile in a live workload or working around one at
no steady-state cost:

| variable | effect |
|---|---|
| `X87_STOCK_HASH_LIST=0xH,...` | hand the listed blocks to stock Rosetta entirely |
| `X87_STOCK_OPS=f2xm1,...` | hand every block containing one of the opcodes to stock |
| `X87_LOG_HASH_LIST=0xH,...` | log every translate request of the listed blocks with an uptime stamp |
| `X87_DIAG_DIR=<dir>` | mirror those logs to `<dir>/x87diag.<pid>.log`, for hosts that lose stdout |
| `X87_ALWAYS_NONE=1` | the sidecar declines every request; separates a JIT bug from an IPC one |
| `X87_DISABLE_HOOK=1` | skip the `translate_insn` patch, the benchmark baseline |
| `X87_NO_DECODE_HOOK=1` | skip the `decode_opcode` patch, so `DC D8` and `ARPL` trap as under stock |

Loader and sidecar diagnostics:

| variable | effect |
|---|---|
| `X87_LOGS=1` | verbose loader logging |
| `X87_LOG_THROUGHPUT=1` | requests per second, traps per request and cache hit rates every 2 s |
| `X87_LOG_OPS=1` | one line per translated op; high volume, for freeze bisects |
| `X87_NO_IR_CACHE=1`, `X87_NO_TCO_CACHE=1` | re-read the IR array or the thread-context layout on every request |
| `X87_NO_PREAUTH=1` | skip acquiring the developer-tools right before launch (default attach only) |

## License

MIT.
