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

## Releasing the default attach stop

The default loader attaches with `PT_ATTACHEXC`, so its stops arrive as
Mach exceptions. Dropping the ptrace attachment and replying to the held
exception are separate operations. On macOS 26.6.2 (XNU 12377.161.14), the
classic sequence successfully detached, but the target stopped again after
the reply to the synthetic SIGSTOP. The sidecar then waited for the stopped
target to exit. This reproduced on unchanged master after the CI runner
moved from macOS 26.5.2 to 26.6.2.

That Tahoe kernel family now uses the same reply-before-detach sequence as
newer Golden Gate kernels: catch SIGSTOP, hold a Mach task suspend, suppress
and reply to the signal while still traced, restore exception ports,
attempt ptrace detach, then balance the task suspend. Older Tahoe builds and early Golden
Gate kernels retain their existing sequence. The Tahoe cutoff is the
verified build, not a claim about the first build to change this behavior.

`test_detach_signals` checks that a released target can receive SIGUSR1 in
its own handler without involving x87 arithmetic. CI also watches the test
harness for stalled output and captures process state, pipe descriptors and
stack samples before terminating a stalled invocation. A timeout fails the
run even if the test PID exited but a sidecar still holds stdout open.

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
decode instruction` when it meets one. The original report on an earlier
runtime lost an x87 addition without that diagnostic. The decoder fix alone
did not resolve the game crash. Constants are therefore materialised through
a GPR, a conditional select is a branch over a register move, and no emitter
branches backwards.

The second is that everything the guest can observe must be in the thread
context at every map entry. A run of consecutive x87 instructions keeps TOP in
a register and defers its tag-word and FXCH bookkeeping to the end of the run,
so a run is answered with one reply: the map then has entries only at the
run's start and its end, where the state is complete. This includes the
register payload format: Rosetta stores physical x87 slots as packed 80-bit
values at offset `0x06`, stride 10. Its signal-context import/export routines
copy those fields directly. The sidecar's compact binary64 slots at offset
`0x08`, stride 8 are private to a reply. The wrapper converts on entry and
exit, after flushing deferred TOP, tags and permutations.

An opaque save/restore round trip can hide a layout mismatch: the same
misinterpreted bytes can be copied out and back unchanged. It fails when a
handler reads or edits the saved registers. `test_x87_signal_context` checks
the actual native payload at the reported chain's end and replaces it before
resuming, in both 64-bit and LDT compatibility mode with nested x87 work in
the handler. `test_x87_native_state` checks all eight FXSAVE/FXRSTOR slots and
binary64 bit preservation across separate replies, including subnormals.

`tests/test_x87_signal_storm.c` runs the reported chain and one case per x87
opcode under a SIGUSR1 storm and compares every iteration bit for bit. Stock
Rosetta itself shifts the x87 stack when a signal lands in its `fcomp`,
`fcompp` or `ficomp` translations; the harness records that as a stock
divergence.

## Precision and the floating-point environment

The sidecar holds x87 values as binary64 (53-bit significand, 11-bit
exponent). Real x87 and stock Rosetta hold them in the 80-bit extended format
(64-bit significand, 15-bit exponent) and round arithmetic results to the
precision-control (PC) field of the control word. The sidecar never reads PC.
The differences below were measured on macOS 27.0.1 on an M5 Max by running
the same x86-64 programs under stock Rosetta and under the sidecar.

At PC=53, the Windows default control word 0x027F, rounding every result to
53 bits is what the hardware does, so `fadd`, `fsub`, `fmul`, `fdiv` and
`fsqrt` agree bit for bit as long as intermediate values stay inside the
binary64 exponent range. An intermediate below 2^-1022 becomes subnormal and
loses bits, and one above 2^1023 overflows, where an 80-bit register would
keep it normal: `(1 + 2^-52) * 2^-1060 * 2^1060` gives `1 + 2^-52` on stock and
`1.0` in the sidecar.

The other two settings cannot be matched by a binary64 register file, and
stock honours both:

| control word | expression | stock Rosetta | sidecar |
|---|---|---|---|
| 0x037F, PC=64 | `(1 + 2^-60) - 1` | 2^-60 | 0 |
| 0x037F, PC=64 | `(1 + 2^-30) * (1 - 2^-30) - 1` | -2^-60 | 0 |
| 0x007F, PC=24 | `(1 + 2^-30) - 1` | 0 | 2^-30 |
| 0x007F, PC=24 | `x / 3 * 3 - x`, x = 1 + 2^-50 | -2^-50 | 0 |
| 0x007F, PC=24 | `2^24 + 1` from `fld m32; fadd m32` | 2^24 | 2^24 + 1 |

PC=64 is the control word of x86-64 macOS processes (0x037F) and of Delphi
programs (`$1332`); their results lose up to 11 bits against the hardware.
PC=24 is what wine's d3d8 and d3d9 set, as Windows does, when a device is
created without `D3DCREATE_FPU_PRESERVE`; such programs get more precision
than the hardware would give them, which matters to code that depends on
single-precision rounding, such as lockstep simulations that have to agree
with other machines. Emulating PC=24 would mean a runtime test of the control
word and a round trip through single precision after every arithmetic result,
at each of the arithmetic emit sites in the direct translator, the fusions and
the IR lowering, roughly a hundred of them. It is not done.

A 64-bit integer loaded with `fild m64` is exact on real x87 for every value,
so `fild m64; fistp m64` copies any `int64_t`. In the sidecar a value above
2^53 in magnitude is rounded on the way: 2^53 + 1 comes back as 2^53 and
2^62 + 3 as 2^62. Compilers use that pair to move 64-bit integers atomically on
32-bit x86 without SSE2.

Precision lost between instructions is not recovered later. The pow idiom of
older C runtimes and Delphi, `fyl2x` followed by `fld st0; frndint; fsub
st(1), st; fxch; f2xm1; fld1; faddp; fscale`, relies on `y * log2(x)` keeping
64 bits so that `2^frac` is accurate to double precision. Swept over 352
integer-valued powers (`2^n`, `3^n` up to `13^n`, `10^n`, `1.5^n`, square
roots of squares) at PC=53 and PC=64, stock returns all 352 exactly and the
sidecar 160 of them: `pow(3, 2)` comes out as 8.999999999999998, so `(int)pow(3, 2)`
is 8, and `pow(10, -2)` is three units in the last place low. A correctly
rounded `fyl2x` would not fix this: with `y * log2(x)` rounded to binary64 and
everything after it exact, 184 of the 352 are still wrong. Carrying the
product as a double-double into the `fsub` makes all 352 exact, but that needs
a fused lowering of the whole sequence and a `log2` accurate to about 2^-62,
and it cannot help when the steps are spread over separate translations, for
example across a call.

Rosetta maps MXCSR.RC to FPCR.RMode, MXCSR.FTZ to FPCR.FZ and MXCSR.DAZ to
FPCR.FIZ, and the emitted x87 code runs under that FPCR. Real x87 ignores all
three. So by default a program that sets FTZ or DAZ for its SSE code also
gets its x87 denormals flushed to zero, down to a plain `fld m32; fstp m32`
copy of a denormal float, which MSVC `/arch:IA32` code uses for float
copies, and, under DAZ, a denormal compares equal to zero. Stock Rosetta
keeps them.

`X87_IGNORE_FTZ_DAZ=1` makes each translation reply clear FZ and FIZ on
entry and restore them on exit (see `emit_native_state_boundary`). It is off
by default because it has a cost for every program, not only for the ones
that set FTZ or DAZ. With neither bit set and an empty x87 stack at both
ends of the reply, a reply gains six instructions and no taken branch. With
values on the stack, entry gains nine instructions and one taken branch and
exit four instructions and one taken branch. On the benchmarks that costs up
to 36 percent on the one-instruction loops of `bench_load` and `bench_store`,
3 to 28 percent on the short chains of `bench_fstp_fld`, and up to 13
percent on the dot product, `f2xm1` and `fsin` benchmarks. With a bit set,
the two FPCR writes cost about 20 ns per reply: a loop around
`fld m32; fstp m32` goes from under 1 ns per iteration to 22 ns, where stock
Rosetta takes 10 ns, and a loop around a six-instruction arithmetic run goes
from 1.2 ns to 23 ns, where stock takes 72 ns. With the knob off the emitted
code is the same as without the feature.

With the knob on, a signal handler that interrupts an x87 run sees FTZ and
DAZ clear: `stmxcsr` in the handler reads them as off, because the handler
starts with the FPCR the run had cleared. The MXCSR saved in the signal
context is correct, and the interrupted code gets FTZ and DAZ back when the
handler returns. Under a SIGUSR1 storm against a loop of short x87 runs with
both bits set, between 0.02 and 4 percent of the handlers saw them clear,
depending on the configuration and the run.

The knob keeps the original FZ and FIZ bits in the halfword at +6 of the x87
state. That halfword belongs to native slot 0, and is free only because the
compact layout keeps its doubles at +8, stride 8, and the entry and exit
conversions read and write slot 0 before and after the halfword is used. If
the sidecar ever keeps values inside the native slots (+6, stride 10)
between the conversions, the saved bits have to move somewhere else.

`test_x87_ftz_daz` covers loads, stores, arithmetic and compares under each
bit, the guest's FTZ and DAZ still applying to SSE code later in the same
block, and a signal storm while the bits are set. The SSE checks report a
skip on a runtime that does not apply the bit to SSE at all; the x87 checks
never do. `run_tests.sh` runs it in a phase of its own: natively and with
the knob it must pass, and without the knob it must fail with its failure
count rather than crash.

The knob does not cover rounding control: x87 arithmetic rounds by MXCSR.RC
rather than by the RC field of the control word. The control word's RC is
honoured by `fist`, `fistp` and `frndint`, which dispatch on it explicitly,
but `fadd` and the other arithmetic instructions round to nearest under a
program that sets the x87 RC to chop, and round up under one that sets
MXCSR.RC up. x87 arithmetic also sets the MXCSR status flags (precision,
underflow) and leaves the x87 status word's exception flags clear, the
reverse of the hardware.

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
