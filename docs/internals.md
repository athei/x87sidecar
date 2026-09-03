# Internals

Notes on the parts of x87sidecar that are easy to get wrong and whose
reasoning does not fit in the README. Each section states a rule the code
follows and the evidence behind it.

## The translate path and what the sidecar caches

Every cold-translated x87 block pays a Mach IPC round trip that an in-process
JIT would not pay. Once stock has installed the ARM64 bytes the sidecar is off
the hot path, so steady-state execution speed is unaffected; what remains is
cold-translation latency, and most of the per-request syscalls that used to
accompany the round trip have been clawed back.

The sidecar caches the block's IR array across requests, revalidated with an
80-byte probe read of the current instruction, since one block generates one
request per x87 run. It caches the thread-context-offsets struct by pointer,
cross-checking every miss against the first copy it saw. And it fuses the
reply send with the next request receive into a single `mach_msg` trap, the
classic MIG-server shape. That is 5 traps per request instead of 7.
`X87_NO_TCO_CACHE=1` and `X87_NO_IR_CACHE=1` switch the caches back off for
A/B runs.

One tempting further step does not work and was reverted after testing:
`mach_vm_remap`ing (copy=FALSE) the tracee's translation buffers into the
sidecar so the remaining reads and writes become memcpys. The tracee is an
x86_64-translated task with 4 KB VM pages, and remapping its private
anonymous memory into the sidecar's 16 KB arm64 map silently degrades to copy
semantics: the two views diverge in both directions. Sharing works only in
the opposite direction, for pages the sidecar allocates and remaps into the
tracee, which is exactly how the block profiler's counter array is wired.

## Asynchronous signals inside emitted code

When a signal is delivered to a thread that is executing translated code,
Rosetta has to hand the guest handler a precise x86 context. It gets one by
stepping the translated ARM code forward to the next entry of the
translation's instruction map (there is one entry per `translate_insn`
reply), taking the guest state from the thread context at that point, and
resuming from that state once the handler returns. Two rules for the code the
sidecar emits follow from this; both were found through issue #23, a Call of
Duty 2 mixer thread that lost the effect of `fld1; faddp` in one execution
out of thousands.

The first is that every emitted instruction has to be one the runtime's own
decoder knows, and control flow may only go forward. `FMOV` (scalar,
immediate), `FCSEL` and inline literal pools (raw data words in the
instruction stream) are not decodable, and a backward branch is treated as a
loop and refused. The macOS 27 runtime aborts the process with `failed to
decode instruction` when it meets one; earlier runtimes resume with part of
the guest instruction unexecuted, which is what the report saw. Constants are
therefore materialised through a GPR, a conditional select is a branch over a
register move, and no emitter branches backwards.

The second is that everything the guest can observe must be in the thread
context at every map entry. A run of consecutive x87 instructions keeps TOP in
a register and defers its tag-word and FXCH bookkeeping to the end of the run,
so a run is answered with one reply: the map then has entries only at the
run's start and its end, where the state is complete.

`tests/test_x87_signal_storm.c` runs the reported chain and one case per x87
opcode under a SIGUSR1 storm and compares every iteration bit for bit. Stock
Rosetta itself shifts the x87 stack when a signal lands in its `fcomp`,
`fcompp` or `ficomp` translations; the harness records that as a stock
divergence.

## Encodings Rosetta's decoder rejects

Two encodings that real hardware runs are absent from Rosetta's decode
tables, so a program containing either takes an illegal-instruction trap
instead of being translated. Both occur in WoW 1.12, and handling them is
what [winerosetta.dll](https://github.com/Gcenx/winerosetta) was injected
into the guest to do:

| encoding | what it is | where |
|---|---|---|
| `DC D8` | undocumented alias of `fcomp st(0)`, in the otherwise memory-form `DC D0..DF` row | `.text:006FA876`, `luaH_set`'s "table index is NaN" check |
| `63 /r` | `ARPL r/m16, r16`, legacy mode only (`0x63` is `MOVSXD` in 64-bit mode, so the tables have no ARPL) | `63 D0` in downloaded, obfuscated code, reached after login |

x87sidecar handles both a stage earlier than `translate_insn`, so no
guest-side DLL is needed. The `translate_insn` hook structurally cannot help:
an encoding the decoder rejects never becomes an instruction to translate. So
there is a second, much smaller stub on `decode_opcode`. It calls the
original, and on an `INVALID` result it builds a substitute the decoder does
accept, points `code_base`/`code_end` at it, decodes that instead, and then
restores those fields along with `insn_start` and `cursor` (which the inner
decode re-seeds from the substitute buffer, and which the rest of the pipeline
reads as guest addresses). The guest's own memory is never modified, and the
stub decides entirely in the tracee without talking to the sidecar.

The two cases differ in how much that buys. `D8 D8` *is* `fcomp st(0)`, same
length, so substituting is the whole fix and stock translates the result.
ARPL has no equivalent encoding, so the substitute borrows `0x01`
(`ADD r/m32, r32`), whose ModRM/SIB/disp encoding is byte-identical, purely
to decode the operands and the length; the mnemonic is then forced to a
synthetic opcode id appended past Rosetta's own, which the JIT stub's filter
passes through to `TranslatorCustom::translate_arpl` rather than letting
stock emit a real `add`.

The substitute is built on the stub's own stack frame, which is per-thread
and always mapped. That is load-bearing rather than incidental. An earlier
version kept it on a page the sidecar allocated and `mach_vm_remap`'d in with
`VM_INHERIT_NONE`; it passed every test binary and wedged wine, because wine
forks and a forked child inherited the patched code but not that page,
faulting once per decode. Anything the handler touches has to be reachable in
every process that inherits the patch.

Rosetta decodes speculatively, past block ends and over data, so `INVALID`
results are routine: a test binary produces about forty of them containing
neither encoding. An `INVALID` only becomes a trap if the guest executes that
address. `X87_NO_DECODE_HOOK=1` disables the stub, which makes both encodings
trap the way they do under stock Rosetta.
