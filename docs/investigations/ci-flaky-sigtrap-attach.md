# CI flaky `exit=133` (SIGTRAP) at ptrace attach — investigation & fix design

**Status:** fixed — the full `PT_ATTACHEXC` migration below is implemented
(`rosetta_loader/src/mach_exception.{hpp,cpp}` + the `MuhDebugger` rewrite).
Local: 825/825, repeatedly. CI confirmation pending.
**Date:** 2026-07-09. **Author context:** master CI has been red since PR #7.
**Confidence:** mechanism *class* high; exact trigger medium (not reproducible
locally — needs a CI-like cold/virtualized environment to confirm).

> **Update 2026-07-09 (later).** The migration surfaced a *locally-reproducible*
> post-detach `exit=133` (~1/12 full-suite runs) the original `PT_ATTACH` path
> did not have. Mechanism **(A)** — `task_swap_exception_ports(EXC_MASK_ALL)`
> displaced Rosetta's own handler and the restore wrote back a stale snapshot —
> was fixed by narrowing the task claim + a conditional restore.
>
> **Mechanism (B) — ROOT CAUSE (confirmed via crash reports), fixed by an
> instruction-cache flush.** The residual crash was *not* about exception-port
> precedence. Every `EXC_BREAKPOINT`/`exit=133` crash report
> (`~/Library/Logs/DiagnosticReports/test_*.ips`, 12+ samples) faults at an
> address whose low 12 bits are `0x91c` with ASLR-varying high bits — i.e.
> `libRosettaRuntime_base + exports_fetch`, **our own planted BRK
> (`0xD4200000`)**, trapping with no handler *after* detach. Mechanism: ARM64 I/D
> caches are not coherent. `writeMemory` was a bare `mach_vm_write` with **no
> i-cache flush**. We plant a BRK at `exports_fetch`, the thread executes it
> there once (caching the line), we restore the original instruction with
> `mach_vm_write` (D-cache only), then RESUME the held thread — which re-fetches
> the same PC (ARM64 BRK doesn't advance PC). If the stale BRK line survives in
> the I-cache to that re-fetch, it re-traps after we've detached and removed our
> handler → fatal `SIGTRAP` in the tracee. Flaky ~1/12 = whether the line was
> evicted in time; independent of exception-port level, which is why the
> thread-level change below did **not** fix it. `exports_fetch` is uniquely
> exposed because it's the one site we modify *after* it's been executed/cached
> and then re-execute; the M2 `__TEXT` patches run before their first execution.
> **Fix:** `writeMemory` now calls `mach_vm_machine_attribute(MATTR_CACHE,
> MATTR_VAL_ICACHE_FLUSH)` after every write (as lldb debugserver does).
>
> A complementary hardening also landed: catch the setup BRK at **thread level**
> (`MachExceptionSession::installThreadBreakpoint`/`removeThreadBreakpoint`)
> instead of task level, so we never share Rosetta's task-level `EXC_BREAKPOINT`
> port; task-level claim is now `EXC_MASK_SOFTWARE` only. This removes a real
> latent restore-race but was not the cause of (B).

> **Implementation note — the one thing this doc got wrong.** The migration
> sketch below says "`detach()`: `PT_DETACH` is still valid". It is, but *only
> from a held BSD signal-stop*. Under `PT_ATTACHEXC` the stops are Mach
> exceptions: the planted `BRK` arrives as a bare `EXC_BREAKPOINT` carrying no
> signal, and `PT_DETACH` rejects that state with `EBUSY` — as it does a
> running process, a task with a held exception, and even a `task_suspend`ed
> one. The working sequence (debugserver's `DoSIGSTOP` + `Detach`) is:
> `kill(pid, SIGSTOP)` → reply to the held stop so the tracee runs into the
> SIGSTOP → receive and **hold** the `EXC_SOFT_SIGNAL(SIGSTOP)` → restore the
> exception ports → `PT_DETACH` *while that stop is still held* → only then
> reply to release the thread (ptrace is no longer callable afterwards, so that
> last reply must not attempt `PT_THUPDATE`). `SIGSTOP` is used precisely
> because it cannot be caught or ignored.

---

## TL;DR

- The red CI is **not a code regression**. It is a flaky fatal `SIGTRAP`
  (`exit=133` = 128+5) that hits a **different arbitrary test each run**,
  always during the loader's short **ptrace setup window**, never in the x87
  translation itself.
- Root-cause hypothesis: the setup phase catches traps via the deprecated
  **`PT_ATTACH` + `waitpid`/`WSTOPSIG` (BSD signal) path**. Delivery of
  `EXC_BREAKPOINT` (from the one `BRK` the loader plants in libRosettaRuntime)
  is a **precedence race** between the debugger and the target's own Mach
  exception handling. When the debugger loses the race the `BRK` is delivered
  to the parent as an unhandled `SIGTRAP` → default disposition = terminate →
  `exit=133`. The codebase already documents this exact hazard
  (`stub_asm.cpp:366`: "libRosettaRuntime's `__TEXT` eats `EXC_BREAKPOINT`
  before SIGTRAP is delivered").
- **Proper fix:** port the loader's attach **fully to `PT_ATTACHEXC`** and drop
  `PT_ATTACH` + `waitpid`/`WSTOPSIG` entirely. `PT_ATTACHEXC` *is* a ptrace
  attach — the sibling of `PT_ATTACH` — that delivers debug events as **Mach
  exception messages** to the debugger's exception port instead of as BSD
  signals. A debugger-registered exception port takes **precedence** over the
  target's own handlers, so the `BRK` is caught deterministically and can never
  leak to the parent as a fatal signal. This is what lldb's `debugserver` does,
  and it unifies the loader on Mach mechanisms (it is *already* all-Mach for
  memory/registers/VM, the steady-state IPC, and exit-via-`kqueue` — the
  `waitpid`/signal path is the lone BSD holdout). The `main.cpp:106-114` comment
  already names `PT_ATTACHEXC` as the correct-but-deferred alternative.
  (Keeping `PT_ATTACH` and merely *adding* `task_set_exception_ports` is **not**
  the fix — mixing signal delivery with a hand-registered port reintroduces the
  very precedence ambiguity we are killing. Use it only as a diagnostic probe;
  see "How to confirm".)
- **Complementary simpler fix:** remove the single planted breakpoint entirely
  (discover the Exports address without a trap), shrinking the trap surface to
  just the exec-stop.

Do **not** ship the `run_tests.sh` retry band-aid (explicitly rejected). Fix the
race.

---

## Symptom / evidence

CI workflow: `.github/workflows/ci.yml:29` runs `sudo scripts/run_tests.sh
--no-build` on a headless `macos-26` runner.

| Run | Commit | Crashing test(s) — all `exit=133` |
|-----|--------|-----------------------------------|
| #7 `28708749666` | `8587e47` | `test_arithp_fstp` |
| #6 `28678720016` | `c69a1d9` | `test_peephole`, `test_peephole6`, `test_frndint`, `test_fcmov`, `test_fstenv`, `test_fxrstor`, `test_finit_compose`, `test_peephole3` |
| #4 `28402654759` | `62b73a5` | `test_fistp_multi` |

Fingerprint: **different arbitrary test(s) each run**; a deterministic logic bug
would kill the *same* test every time (the tests are pure deterministic FP
checks that print a `FAIL` line on mismatch — see `tests/test_arithp_fstp.c`).
In the #7 log the loader had already printed `[rosettax87] attached:` for the
crashing test, so **attach succeeded** and the crash is *after* attach, inside
the remaining setup steps. Locally (entitled loader, no sudo) the full suite is
**825/825**, repeatedly.

`run_tests.sh` classifies this: non-zero exit with **no `FAIL` line** →
`CRASH (exit=<code>)` (`scripts/run_tests.sh:178`).

---

## Architecture recap (so the fix is scoped correctly)

The loader is a *reverse-ptrace* debugger. All of this is in
`rosetta_loader/src/main.cpp`:

1. **Fork dance** (`main.cpp:588-646`): the **parent keeps the PID and will
   `execv` the target** (it becomes the tracee). The child double-forks; the
   orphaned **grandchild is the debugger** (double-fork avoids a process-tree
   cycle that crashes Terminal's proc walker — see the `NOTE_EXIT`/`ppid==1`
   comment at `main.cpp:624-646`).
2. **Attach** (`main.cpp:649`, `MuhDebugger::attach` at `:100`): `PT_ATTACH`
   (sends SIGSTOP), `waitForStopped()` (`:67`, `expectedSignal=0` → returns on
   first stop). Prints `[rosettax87] attached:` (`:653`), then releases the
   parent via `syncPipe` (`:656`).
3. **Exec stop** (`main.cpp:660`, `waitForExecStop` at `:130`): `PT_CONTINUE` +
   `waitpid(SIGTRAP)`, then `task_for_pid` (`:140`) to grab the parent task
   port. (Register/memory access is all Mach: `thread_get_state`,
   `mach_vm_read/write/protect` — `:266-497`.)
4. **One breakpoint** (`main.cpp:725-727`): `setBreakpoint(exports_fetch)` plants
   a `BRK #0` (`AARCH64_BREAKPOINT = 0xD4200000`, `:548`) in libRosettaRuntime's
   `__TEXT`, `continueExecution()` (`:148`, `PT_CONTINUE` + `waitpid(SIGTRAP)`)
   runs until it's hit, reads `X19` (the Exports struct addr, `:729`), then
   `removeBreakpoint`. **This is the only planted breakpoint.**
5. **Install the inline IPC stub** (`main.cpp:744-1423`): patch the TR-size MOVZ,
   snapshot translate_insn's prologue, find `__TEXT` trailing padding, install a
   Mach service port in the parent (`sidecar::installPortInParent`), write the
   handler blob + transcendental constants + optional profile counters, then
   overwrite `translate_insn[0..16]` with an abs-jump to the handler.
6. **Detach + go async** (`main.cpp:1429-1473`): capture the parent task port,
   `PT_DETACH` (`:1430`), `sidecar::spawnReceiveThread`, block on
   `kqueue`/`NOTE_EXIT` until the parent exits.

**Key consequence:** ptrace/signal trapping is used **only in steps 2–6 setup**.
Once detached, the tests run **free**, tickling the loader via **Mach IPC** only.
So the fatal `SIGTRAP` must occur in the setup window — and given attach already
succeeded, the prime suspect is **step 4's `BRK`** (or, less likely, step 3's
exec stop; see below).

Full trap/ptrace surface (confirmed by grep): `PT_ATTACH` ×1, `PT_CONTINUE` ×3
(exec, breakpoint, signal-suppress), `PT_DETACH` ×2 (hook-disable path +
normal), one `setBreakpoint`. No existing Mach-exception-port code.

---

## Where the fatal SIGTRAP comes from

`exit=133` means a `SIGTRAP` reached the tracee **with its default (fatal)
disposition** — i.e. it was *not* intercepted by the debugger.

- **Exec stop (step 3) is ordering-safe, likely not it.** After `PT_ATTACH` the
  parent is ptrace-**stopped**; the `syncPipe` write makes the pipe data
  available but the parent cannot run (let alone `execv`) until the grandchild's
  `PT_CONTINUE` in `waitForExecStop`. So the exec trap fires strictly after the
  tracer is waiting for it. (Caveat: an x86 binary under Rosetta may emit more
  than one early trap/signal during bootstrap; `waitForStopped(SIGTRAP)`
  suppresses non-SIGTRAP signals and returns on the first SIGTRAP. This is a
  correctness smell but not the obvious crash.)

- **The planted `BRK` (step 4) is the prime suspect.** When the parent executes
  the `BRK`, the CPU raises an `EXC_BREAKPOINT`. Who sees it first is a
  **precedence** question between:
  - the ptrace layer (which is supposed to convert it to a `SIGTRAP`-stop
    reported to the tracer via `waitpid`), and
  - **libRosettaRuntime's own Mach exception handling** on the parent's threads.

  The repo already learned (`stub_asm.cpp:366`,
  `feedback_brk_vs_svc_in_rosetta_text.md`) that **"libRosettaRuntime's `__TEXT`
  eats `EXC_BREAKPOINT` before SIGTRAP is delivered"** — which is exactly why the
  abort routine uses `SVC` syscalls, not `BRK`. Yet step 4 plants a `BRK` in that
  same `__TEXT`. When Rosetta's handler (or the parent default handler) wins the
  race, the breakpoint becomes an **unhandled SIGTRAP to the parent → terminate →
  133**. Whether the debugger or Rosetta wins depends on timing (has Rosetta
  finished installing its exception handler at the instant the `BRK` is hit?),
  which is precisely what changes between warm-local and cold-CI.

### Why CI and not local

CI perturbs exactly the timing this precedence race depends on:
1. **Cold Rosetta translation** — CI runs `softwareupdate --install-rosetta`
   fresh each job (`ci.yml:16`), so every test binary is translated first-time
   with an empty `/var/db/oah` AOT cache, right in the attach/breakpoint window.
2. **Virtualized, shared runner** — `macos-26` is a VM; scheduling jitter stretches
   every `waitpid`/`PT_CONTINUE`/handler-install ordering.
3. **root vs entitlement** (the `sudo`) — a *different authorization path* for
   `PT_ATTACH`/`task_for_pid`, but **not** itself a timing race. `sudo` is
   required in headless CI only because the interactive debugger-entitlement
   dialog can't appear; it is almost certainly a red herring for the crash. Do
   not "fix" sudo.

The "different test each run" fingerprint is what an environment-timing race
produces.

---

## The proper fix — port the whole attach to `PT_ATTACHEXC`

Replace the **BSD-signal trap path** (`PT_ATTACH` + `waitpid`/`WSTOPSIG`)
wholesale with `ptrace(PT_ATTACHEXC, pid, 0, 0)` + a **Mach exception receive
loop**. `PT_ATTACHEXC` is a ptrace attach that delivers *all* debug events —
the attach-stop, the exec-stop, and the `BRK` — as Mach exception messages to a
port the debugger owns, rather than as BSD signals. **`PT_ATTACH` and
`waitpid`/`WSTOPSIG` leave the codebase entirely.**

This is not "add an exception port alongside ptrace". It is the *replacement*
attach primitive. Keeping `PT_ATTACH` and merely calling
`task_set_exception_ports` yourself is the wrong end state: you then have the
ptrace signal machinery and your port both eligible for the same
`EXC_BREAKPOINT`, i.e. the precedence ambiguity we are trying to remove. (That
hybrid is still useful as a one-off *diagnostic probe* — see "How to confirm" —
just not as the shipped design.)

Why this removes the race (not just narrows it):
1. A debug event delivered as a **Mach message to a port the debugger owns
   cannot fall through to the tracee's default (fatal) signal disposition** —
   the `exit=133` path is structurally impossible.
2. A **debugger exception port takes precedence** over the target's own
   handlers, so the planted `BRK` is caught deterministically even though
   libRosettaRuntime installs its own `EXC_BREAKPOINT` handling. This directly
   neutralizes the "Rosetta eats the breakpoint" hazard.
3. It is **per-thread and message-ordered**, robust to the multi-threaded
   Rosetta runtime that the whole-process signal model handles poorly.
4. It is **architecturally consistent**: the loader is *already* all-Mach for
   memory/registers/VM (`main.cpp:266-497`), the steady-state IPC
   (`sidecar::spawnReceiveThread`), and parent-exit (`kqueue`/`NOTE_EXIT`,
   `:1461`). The `waitpid`/signal path is the lone BSD holdout; this removes it.

### Does `waitpid` disappear completely? For this loader, yes.

A fully general debugger (lldb `debugserver`) keeps a *hybrid*: a Mach exception
thread **plus** a `waitpid` thread for process lifecycle + real UNIX signals,
using `PT_THUPDATE` to choose signal pass-through. This loader does **not** need
that during its narrow setup window:
- attach-stop / exec-stop / the one `BRK` are all **exceptions**
  (signal-stops surface as `EXC_SOFTWARE` with `EXC_SOFT_SIGNAL`; the breakpoint
  as `EXC_BREAKPOINT`) → all delivered to the port.
- **parent exit** is already handled by `kqueue`/`NOTE_EXIT`, not `waitpid`, so
  no lifecycle `waitpid` is required. For the rare "parent dies mid-setup",
  request `MACH_NOTIFY_DEAD_NAME` on the task port (or just treat the next
  `mach_msg`/`mach_vm_*` failure as death). No `PT_THUPDATE` signal-forwarding is
  needed because the loader passes nothing to the tracee during setup.

### Concrete migration (surface is small — one place)

Only `MuhDebugger`'s trap plumbing changes; all Mach memory/register/VM code, the
stub-install path, and the steady-state IPC are untouched. Both entry paths
(normal and `X87_DISABLE_HOOK=1`, `main.cpp:708`) go through `MuhDebugger`, so
"the whole thing" is one rewrite.

- **New:** allocate a receive right + insert a send right; a
  `mach_exc_server`/`catch_mach_exception_raise*` handler (MIG `mach_exc.defs`),
  or a hand-rolled `mach_msg` loop decoding message ids 2401/2405 with
  `MACH_EXCEPTION_CODES` (64-bit `mach_exception_data_t`).
- **`attach()` (`main.cpp:100`):** `ptrace(PT_ATTACHEXC, pid, 0, 0)`; the
  attach-stop arrives as the first exception message.
- **Rewrite** `waitForStopped()` / `waitForExecStop()` / `continueExecution()`
  (`main.cpp:67-157`) to **block on `mach_msg` receive** instead of `waitpid`. On
  the `BRK` (`EXC_BREAKPOINT`) message: read thread state (`copyThreadState`,
  `:438`), do the work (read `X19`), then **reply to resume**.
- **Resuming past the BRK:** the loader **removes** the breakpoint after the
  single hit (`:727`) and there is only ever one, so no single-step-and-re-arm
  dance is needed — restore the instruction, set thread state, reply. (Persistent
  breakpoints would need remove→step→re-arm; not required today.)
- **`detach()` (`main.cpp:159`):** `PT_DETACH` is still valid; also deallocate the
  exception port / dead-name notification.

### Subtleties / risks to verify (validate against `debugserver`)

- **Mirror the reference implementation.** lldb
  `tools/debugserver/source/MacOSX/MachException.cpp`, `MachTask.cpp`,
  `MachProcess.cpp` is the canonical `PT_ATTACHEXC` + Mach-exception debugger for
  arm64 macOS. Do not reconstruct the message/reply flow from memory — copy its
  structure.
- **Exec re-registration / ordering.** macOS resets a task's exception ports
  across `execve`; `PT_ATTACHEXC` is designed to still deliver the exec event, but
  confirm the port is live for the `BRK` that follows the exec — (re)establish
  after the exec-stop, before continuing to the exports-fetch point.
- **Reply/resume protocol.** Returning the right `kern_return_t` from the
  catch handler is what resumes the thread; a wrong reply hangs (no resume) or
  double-resumes. Get `RetCode`/`MACH_RCV` semantics right.
- **Forwarding.** For any exception that is *not* our `BRK` (unexpected fault),
  forward to the task's previously-registered handler (save the old ports the
  kernel returns when you install yours) so real faults aren't swallowed.
- **`MACH_EXCEPTION_CODES`.** 64-bit codes; the MIG subsystem and the reply must
  agree or you get parse/`MACH_RCV` errors.
- **Entitlement.** No new entitlement — `PT_ATTACHEXC` needs the same
  `com.apple.security.cs.debugger`/root the loader already has.
- **Rosetta precedence.** The whole premise — *confirm empirically* on the runner
  that the debugger port pre-empts libRosettaRuntime's handler (see below).

---

## Complementary / cheaper fix — delete the breakpoint

The single `BRK` exists only to read `X19` = Exports struct address at a known
point in libRosettaRuntime init (`main.cpp:725-733`). The loader **already**
computes `translate_insn`'s live address *without* a trap, from
`init_library`'s runtime address (first `x87Exports[]` entry) plus an RVA delta
(`main.cpp:816-839`). If the Exports address is similarly derivable from
offsets/exports without stopping at `exports_fetch`, the planted breakpoint —
and thus the crash's prime suspect — disappears, leaving only the exec-stop.
This is smaller than the full Mach-port migration and may be sufficient on its
own; ideally do both (delete the BRK *and* move to an exception port so the
exec-stop is also robust). Worth scoping first — it could be the 80/20.

---

## How to confirm the hypothesis (not locally reproducible)

Since local runs never crash, confirmation needs a CI-like environment:

1. **Instrument the setup path** on a CI branch: add loud logging (unconditional
   `fprintf`, not `VERBOSE_LOG`) around each of: exec stop reached, breakpoint
   set, breakpoint hit / `continueExecution` return, each `PT_CONTINUE` result,
   `WSTOPSIG` value in `waitForStopped`. Re-run CI a few times; the last line
   before the `exit=133` pins the exact trap. Prediction: it dies at/around the
   `exports_fetch` breakpoint hit.
2. **Cold-cache local repro attempt:** `sudo rm -rf /var/db/oah/*` (Rosetta AOT
   cache) then run the suite under `sudo scripts/run_tests.sh` in a loop; try
   under CPU load / `taskpolicy` to mimic VM jitter. If it reproduces, iterate
   locally.
3. **Cheap disproof of the fix direction (diagnostic probe only, not the fix):**
   on a throwaway branch, keep `PT_ATTACH` but register a
   `task_set_exception_ports(EXC_MASK_BREAKPOINT, …)` on the parent right after
   the exec stop, and see if CI stops crashing. If it does, that is strong
   evidence the precedence hypothesis is correct — proceed to the *real* fix
   (full `PT_ATTACHEXC`). Do **not** ship this hybrid: `PT_ATTACH`'s signal path
   and the hand-registered port both contend for the `BRK`, which is the
   ambiguity the real fix removes.

---

## Verification plan for the fix

- Local: `bash scripts/run_tests.sh --no-build` → 825/825 (must not regress).
- Local soak: loop the suite (and with `X87_DISABLE_HOOK=1`, which also uses the
  attach+detach path, `main.cpp:708-723`) many times; expect zero crashes.
- CI: push branch; the runner (`sudo`) should go green and **stay** green across
  repeated re-runs (the flakiness only shows at CI cadence, so re-run several
  times, don't trust one green).
- Confirm the steady-state IPC + profiling still works (a normal WoW capture per
  the `howto-profile` memory) — the stub/detach path is untouched but the attach
  refactor precedes it.

---

## Scope estimate

- **Delete-the-breakpoint** route: small, localized to `main.cpp:725-733` +
  offset discovery; low risk; may fully fix if the BRK is the sole trigger.
- **Full `PT_ATTACHEXC`** route: medium; rewrites `MuhDebugger`'s ~5 trap methods
  (`main.cpp:67-166`) and adds a small MIG/`mach_msg` exception handler; removes
  `PT_ATTACH` + `waitpid`/`WSTOPSIG` entirely; touches the most delicate part of
  the loader; needs the soak above. This is the durable, provably-correct fix,
  the one the `main.cpp:106` comment already points to, and the architecturally
  consistent end state (all-Mach loader).

Recommended order: scope the breakpoint-deletion first (cheap, possibly
sufficient on its own), then do the full `PT_ATTACHEXC` migration regardless — it
makes the exec-stop robust too and permanently closes the signal-delivery race
class, not just the one breakpoint. Do both; do not ship the retry band-aid or
the `task_set_exception_ports` hybrid.

---

## Anchors

- `.github/workflows/ci.yml:16,29` — Rosetta install + `sudo` test run.
- `rosetta_loader/src/main.cpp:67` `waitForStopped`, `:100` `attach`,
  `:130` `waitForExecStop`, `:148` `continueExecution`, `:159` `detach`,
  `:168` `setBreakpoint`, `:548` `AARCH64_BREAKPOINT`, `:588-646` fork dance,
  `:725-733` the one breakpoint, `:816-839` trap-free address computation,
  `:1429-1473` detach + async steady state.
- `rosetta_loader/src/stub_asm.cpp:366` — "libRosettaRuntime's `__TEXT` eats
  `EXC_BREAKPOINT` before SIGTRAP is delivered" (the corroborating prior finding;
  refs `feedback_brk_vs_svc_in_rosetta_text.md`, not in-repo).
- `scripts/run_tests.sh:169-192` — `check_output` crash classification.
