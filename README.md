# x87sidecar

Faster x87 floating-point for x86 apps running under Apple's Rosetta 2 on Apple Silicon.

This project is a fork of [Lifeisawful/rosettax87_jit](https://github.com/Lifeisawful/rosettax87_jit). It started as a drop-in JIT replacement for stock Rosetta's x87 instruction handlers, but has since diverged enough — both architecturally and in scope — that upstreaming back is no longer realistic, hence the rename.

## Why fork?

The original `rosettax87` was a **dylib injected into the wine/x86 process**. It mapped its own anonymous pages with `MAP_TRANSLATED_ALLOW_EXECUTE`, dropped hand-rolled ARM64 into them, and patched stock's `translate_insn` to branch into that code.

That works on the happy path. It does not survive signals.

**The constraint:** inside a process running under Rosetta, every ARM64 instruction the CPU executes is supposed to have been produced by Rosetta itself from x86. When a signal fires (or any other event makes the runtime walk the thread's PC), Rosetta does an ARM64-PC → x86-PC reverse lookup against its own translation tables. ARM64 code from an injected dylib is unknown to those tables — the lookup misses, and Rosetta aborts with:

```
rosetta error: no code fragment associated with the given arm pc
```

The longer the JIT runs, the more time the thread spends with PC inside the injected dylib's `__TEXT`, and the more likely a signal will arrive at exactly the wrong moment. In real workloads (e.g. World of Warcraft under wine) this surfaced as random crashes proportional to JIT load — un-fixable without leaving the dylib model.

**The fix is structural:** keep all our ARM64 code out of the Rosetta'd process entirely. Run the translator in a separate native arm64 process — the **sidecar**. The only ARM64 we still need *inside* the Rosetta'd process is a tiny IPC stub, written **once at install time** into the **page padding** at the tail of stock's translation-output buffer. We don't allocate any new executable mappings of our own; the stub sits on pages stock has already allocated and registered with Rosetta as translated-from-x86, so the reverse-lookup tables cover it for free. The one place we do touch stock code is the prologue of `translate_insn` itself: we overwrite the first few bytes with a branch into the stub, and we preserve the displaced bytes inside the stub so they can run unchanged on the fall-through path. The stub is intentionally minimal — its only runtime work is two bounds checks against the contiguous x87 opcode ranges in Rosetta's enum. If the opcode is in range, the stub sends a Mach message to the sidecar; if not, it executes the preserved prologue bytes and branches back into stock. On the IPC reply, the stub either returns to translate_insn's caller with the sidecar's result (handled) or falls through to stock (the sidecar reported unhandled).

## Architecture

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

The cost is real: every cold-translated x87 block now pays a Mach IPC round-trip plus 4–6 `mach_vm_read` / `mach_vm_write` syscalls (for the IR buffer, insn buffer, and translation-result struct) that the in-process dylib didn't pay. Once stock has installed the ARM64 bytes the sidecar is no longer on the hot path, so steady-state execution speed is unaffected, but cold translation is slower than it used to be. Profile counters are already `mach_vm_remap`'d (copy=FALSE) so JIT-emitted `LDADDAL` on a parent VA hits the same backing page the sidecar reads; doing the same for the IR / insn / TR buffers is a planned change to claw back the per-call syscalls.

There are two genuine upsides relative to the dylib approach. First, the sidecar is a normal arm64 process, free to use the C++ standard library and any arm64 dependencies. The in-process dylib had to stay close to no-std discipline — every call from our `__TEXT` to a library increased the wall-clock fraction the parent thread spent with its PC inside our pages, and therefore the rate of the reverse-lookup panic. Out of process, that constraint is gone.

Second, we no longer hijack Rosetta's export table to install the hook. The dylib version rewrote `X19` mid-init so that Rosetta's loader called *our* exports instead of its own; that worked but was brittle, because it depended on undocumented loader semantics that drift between Rosetta versions. The sidecar version simply overwrites the prologue of `translate_insn` with a branch — a much smaller, more stable surface to maintain.

The `x87sidecar` binary plays both roles: when invoked with a target x86 program, it launches the target under Rosetta, attaches, patches `translate_insn`, then drops into its own receive loop serving IPC requests.

## Attach modes

The sidecar has to obtain the tracee's Mach task port (to read/write its memory and plant the stub). There are two ways it does this, and they have very different deployment consequences.

**Default — `task_for_pid` + ptrace.** The sidecar forks; the parent execs the target while a grandchild acts as the debugger, calls `task_for_pid`, and attaches via `PT_ATTACHEXC`. Unprivileged, this requires the [debugger entitlement](https://developer.apple.com/documentation/bundleresources/entitlements/com.apple.security.cs.debugger) (`com.apple.security.cs.debugger`) on the caller *and* `com.apple.security.get-task-allow` on the target — and because the port is grabbed just before the parent execs, the still-`x87sidecar` parent must itself carry `get-task-allow`. Running as root (e.g. CI under `sudo`) bypasses both. This path is used by the local test/benchmark harness against the x86-64 Mach-O sample binaries.

**Cooperative — `--cooperative`.** The sidecar `bootstrap_check_in`s a per-pid service, passes its name to the target via the `X87_SIDECAR_BOOTSTRAP` env var, and the target voluntarily hands over its task *and* thread control ports over that Mach service, then blocks on a handshake reply. No `task_for_pid`, no ptrace, and **no entitlements** — so a hardened, Developer ID-signed, cooperative binary is notarizable. This is the mode the shipping wine build uses: its Rosetta re-exec always inserts `--cooperative`, and wine's startup performs the target side of the handshake.

Both modes converge on the same install and IPC loop once the port is in hand.

## Binaries

The build emits two signed artifacts from one link:

| Artifact | Signing | Use |
|---|---|---|
| `x87sidecar` | ad-hoc, **no entitlements** | Ships in the bundle; cooperative attach only. Re-sign with Developer ID + hardened runtime for distribution, then notarize. |
| `x87sidecar_entitled` | same Mach-O + `cs.debugger` + `get-task-allow` | Local default-attach (unprivileged) for the test/benchmark harness. Never shipped. |

The two are byte-identical except for the signature. The test scripts point at `x87sidecar_entitled`; under CI's `sudo` either would work.

## Status

x87 coverage: arithmetic, memory ops, comparisons, the full transcendental set (fsin, fcos, fsincos, fpatan, f2xm1, fyl2x, fyl2xp1, fptan, fprem, fprem1, fxtract, fscale), state-management ops (fldenv, fstenv, fxsave, fxrstor, fsave, frstor, fclex, finit, fldcw, fstsw), and the typical fusion patterns produced by 3D-game pipelines.

Tested live against TurtleWoW (a x86 World of Warcraft client). Not a general-purpose drop-in for arbitrary x86 software — it's been hardened against the workloads it sees, and may need work on others.

## Building

```bash
cmake -B build
cmake --build build
```

This produces both `build/bin/x87sidecar` (flat) and `build/bin/x87sidecar_entitled` (see [Binaries](#binaries)). Tests and benchmarks are built automatically.

```bash
bash scripts/run_tests.sh                # build + test (native Rosetta & x87sidecar)
bash scripts/run_tests.sh --no-build     # skip build
bash scripts/run_tests.sh --native-only  # baseline only
bash scripts/run_tests.sh test_arith     # specific test
bash scripts/run_benchmarks.sh           # build + benchmark
```

The harness uses the default (`task_for_pid` + ptrace) attach path, so it runs `x87sidecar_entitled` — which needs the debugger entitlements when unprivileged (see [Attach modes](#attach-modes)), or root via `sudo`. Cooperative attach (`--cooperative`, the mode the shipping bundle uses) needs no entitlements at all.

## Configuration

Knobs are environment variables read at startup. The most useful ones:

| Variable | Effect |
|---|---|
| `X87_FAST_ROUND=1` | Skip rounding-mode dispatch (faster but unsafe for FLDCW-heavy code; `=2` skips it only in blocks with no control-word writer — safer, still speculative) |
| `X87_ENABLE_BRIDGE=0` | Disable run bridging (default on): carrying one IR run across short `mov`/`lea` gaps between x87 segments |
| `X87_BRIDGE_V2=0` | Disable bridging of flag-writing ALU gaps (`add`/`sub`/`and`/`or`/`xor`/`inc`/`dec`) whose written flags Rosetta's own liveness analysis proves dead (default on) |
| `X87_ENABLE_IR_SPLIT=0` / `X87_ENABLE_IR_REMAT=0` | Disable register-pressure relief (splitting over-pressure runs / sinking long-lived values) |
| `X87_DISABLE_CACHE=1` | Disable x87 translation cache |
| `X87_DISABLE_X87_IR=1` | Disable IR optimization pipeline (direct translator only) |
| `X87_DISABLE_SINGLE_FAST=1` | Disable the fused single-op fast path for isolated `fld`/`fst`/`fstp` |
| `X87_DISABLE_ALL_FUSIONS=1` | Disable all instruction fusions |
| `X87_DISABLE_FUSIONS=f1,f2,…` | Disable specific fusions |
| `X87_DISABLE_HOOK=1` | Skip the `translate_insn` patch (apples-to-apples baseline against stock Rosetta) |
| `X87_LOGS=1` | Verbose loader logging |

## License

MIT.
