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

That conversion rounds every valid slot to binary64, which loses the low
bits of an int64 of 2^53 or more held on the x87 stack from one reply to
the next. Delphi's `System.Move` copies memory that way: it loads 8-byte
chunks with `fild` and stores them with `fistp` on the other side of
conditional branches and loops, and NosTale lost file paths to it. So a
reply whose x87 instructions are all `fild`, `fist`, `fistp`, `fisttp`,
`fldcw`, `fnstcw` or no-ops is a native transport reply: it skips the
conversion, `fild` writes the exact 80-bit value straight into the native
slot, and the integer stores convert the native slot under the control
word's rounding mode. Slots such a reply does not touch are neither read
nor written, and memory holds only the native layout at both of its map
entries. Its emitters write no NZCV and branch only forward, like the
boundary. `X87_NATIVE_TRANSPORT=0` turns this off for bisecting.
`test_delphi_move` runs the routine itself, and `test_x87_native_state` and
`test_x87_signal_context` check the payload in a save area and in a signal
context.

An opaque save/restore round trip can hide a layout mismatch: the same
misinterpreted bytes can be copied out and back unchanged. It fails when a
handler reads or edits the saved registers. `test_x87_signal_context` checks
the actual native payload at the reported chain's end and replaces it before
resuming, in both 64-bit and LDT compatibility mode with nested x87 work in
the handler. `test_x87_native_state` checks all eight FXSAVE/FXRSTOR slots and
binary64 bit preservation across separate replies, including subnormals.

`tests/test_x87_signal_storm.c` runs the reported chain and one case per x87
opcode, plus two native transport runs, under a SIGUSR1 storm and compares
every iteration bit for bit. Stock Rosetta itself shifts the x87 stack when
a signal lands in its `fcomp`, `fcompp` or `ficomp` translations; the
harness records that as a stock divergence.

## Memory faults inside emitted code

A synchronous fault, such as a load from a `PROT_NONE` page, is recovered
the other way round. The faulting instruction must not complete, so the
runtime cannot step forward. It rolls back instead: it takes the ARM code
from the start of the faulting pc's instruction-map entry up to the faulting
instruction, and asks the translator library whether any of it changed
state. If nothing did, the guest handler gets the entry's x86 address as rip
and the guest state from the thread context. If something did, the runtime
aborts the process with `unexpectedly need to EmulateForward on a
synchronous exception`. As far as that check is concerned, these change
state: every store, including atomics; writes to NZCV or FPCR; writes to x0
through x15 and x17, which hold guest registers; and all scalar
floating-point data processing, `fmov` and `fcvt` included, because it can
set FPSR flags. Loads into scratch registers, writes to x16, x22 through x30,
and `bl` do not count.

Stock Rosetta has one map entry per x86 instruction, and its x87
translations access guest memory before they change anything: `fldl (%rsi)`
is `ldr x22, [x6]` followed by a call into a runtime routine, and
`fstps (%rsi)` is a call that returns the value, then `str w22, [x6]`, then
the pop. The routines' own floating-point work is out of the check's sight.

The sidecar's replies start with the conversion from the native register
layout to the compact one, which compares and stores, so a fault in any
guest access after it aborts, even in a lone `fstps`. By default nothing is
done about it: with `X87_PRECISE_FAULTS` off, any guest memory fault inside
sidecar-translated x87 code aborts the process with that message, where
stock Rosetta calls the program's SIGSEGV or SIGBUS handler. Windows
programs under wine see such faults through structured exception handling:
guard pages, write watches, `IsBadReadPtr`, deliberate access violations.
A program that aborts this way can be run with `X87_PRECISE_FAULTS=1`.

With the knob on, the memory operand of each reply's first x87 instruction
is probed before anything else in the reply (`emit_fault_probe`). A load is
probed with a load of the access width into the zero register, or with
loads of its first and last byte for the 10- and 108-byte forms; one load
of the access width also covers an unaligned access that crosses a page. A
store is probed by loading the bytes it will write and writing them back
unchanged: the full access width for 2, 4 and 8 bytes, otherwise the first
byte after a load of the last. The write-back is the probe's only state
change and comes last, so a fault in it still rolls back, and it faults
where the store would, on a read-only page (a wine write watch) as much as
on a `PROT_NONE` one. Once the probes pass, the real access cannot fault,
and a fault in a probe reaches the guest handler with that instruction as
rip and the state from before it.

The write-back has a window: a write by another thread to the same bytes
between the probe's load and its store is undone. The real store then
overwrites those bytes, so the end state is one the x86 store could have
produced, and only the other thread's value is lost for that moment. An
atomic add of zero would leave no window, but LSE atomics take a bus error
on write-combined and uncached mappings, where plain loads and stores work.
Metal buffers and IOSurfaces can be mapped write-combined (a Metal buffer
created with `MTLResourceCPUCacheModeWriteCombined`, an IOSurface with a
write-combine or inhibit-cache mode), and Vulkan translation layers may map
host-visible memory that way. A D3D game filling such a mapped vertex buffer
with `fstp` would have crashed.

For an operand with a base and a displacement the probe adds two
instructions to a load and three to a store (one and two for a bare
register); the 10- and 108-byte forms take one more. The probe
instructions are ordinary loads and stores, which the runtime steps over
for asynchronous signals like any other.

The fault kind differs in one case. Stock reports a load as a read fault
(`__err` 0x4) and a store as a write fault (0x6), which wine passes on as
`ExceptionInformation[0]`. A store the sidecar probes faults in the probe's
load when its page is `PROT_NONE`, so the handler sees a read fault there.
On a read-only page the write-back faults, and the kind matches stock.

Native transport replies need no probe: with the knob on, a run of only
integer loads and stores is answered one instruction per reply, and each
of those emitters touches guest memory before it changes any state, so its
own access rolls back, with the same fault kind as stock. Those replies
have no conversion at either end: in Delphi's `System.Move`
(`test_delphi_move`), a run of two `fild` goes from 64 instructions as one
reply to 67 as two, and a run of two `fistp` from 109 to 113. One
exception is kept: an adjacent `fild m64; fistp m64` pair stays one fused
reply of seven instructions, which as two replies would be 89, and it is
the inner loop of `System.Move`. A fault in its store rolls back to the
`fild`, with the state from before it: a handler that makes the page
accessible and returns gets the right result, one that skips the reported
instruction does not, but the process does not abort. The nested pairs of
that fusion (`fild; fild; fistp; fistp`) are not used in such runs,
because a second store behind the first would abort; split, two nested
pairs take about a hundred instructions instead of nine. With
`X87_PROFILE` or `X87_TRACE_BLOCK` on, their counter and trace stores come
ahead of a transport reply's access, and a fault there aborts.

The knob is off by default because of what it costs, measured on an M5 Max
against the same build without it. The store probe has to read the bytes
it writes back, and a read from write-combined memory is uncached, so an
x87 store at the start of a reply into a write-combined buffer becomes 23
to 40 times slower: a loop filling such a buffer with `fstps` takes 39
times as long. Loops of x87 arithmetic on memory operands lose up to 7
percent (dot products, `fyl2x`), and a loop of two nested
`fild m64; fistp m64` copies takes 4.5 times as long, since those runs are
split. With the knob off the emitted code is the same as without the
feature.

Even with the knob on, a fault in a later instruction of a floating-point
run still aborts. That instruction has no map entry of its own, and giving
it one would mean writing the x87 state back to memory in the native layout
before every memory access of the run, which is the cost the run reply
exists to avoid. Probing the later accesses at the start of the reply as
well would not help: the rollback would report the run's first instruction
as rip, so a handler that skips the faulting instruction would skip the
wrong one, and a second store probe would itself change state before the
faulting one. This needs the x87 state to stay in the native layout
throughout. A 10- or 108-byte store that crosses from a writable page into
a read-only one also still aborts, since only its first byte is written
back, and with `X87_NATIVE_TRANSPORT=0` an integer-only run is an ordinary
floating-point run again.

`tests/test_x87_sync_fault.c` checks rip, fault address and kind, control
word, status word, tags and stack contents in the handler against a
reference taken just before the instruction, then either makes the page
accessible and returns or moves rip past the instruction, and compares the
final state and memory with an unfaulted run and with a run that leaves the
instruction out. Its cases cover `PROT_NONE` and read-only pages, bare,
displacement, index, RIP-relative and `%gs` operands, and 32-bit
compatibility mode through an LDT code segment, and it stores through x87
instructions into write-combined and uncached IOSurfaces. `run_tests.sh`
runs it in a phase of its own with `X87_PRECISE_FAULTS=1`, and once
without, where a store at the start of a reply must abort with the
runtime's message. `SYNC_FAULT_IN_RUN=1` adds the cases that are not
precise yet, which pass under stock Rosetta.

## Guest flags in NZCV

Stock Rosetta keeps the guest's EFLAGS in NZCV, and a flag one translator
sets is read by the other whenever an x87 instruction and its neighbours go
to different ones: every SSE and integer instruction is stock's, and
declined requests, `X87_STOCK_OPS` and `X87_STOCK_HASH_LIST` hand x87
instructions to stock too. So the sidecar has to use stock's layout exactly:

| x86 flag | where stock keeps it |
|---|---|
| SF | N |
| ZF | Z |
| CF | C, inverted |
| OF | V |
| AF | NZCV bit 27 |
| PF | NZCV bit 26 |

Bits 26 and 27 are below the four architectural flags. Apple cores write
them on flag-setting instructions, and `MRS`/`MSR NZCV` read and write
them. Stock's translations show the layout directly: `pushf` and `lahf`
read PF with `MRS` and `UBFX #26` and AF with `UBFX #27`, `popf` and `sahf`
pack all six into one `MSR NZCV`, `jp` and `setp` test bit 26, `ucomisd`
is `FCMP; AXFLAG` with PF taken from bit 26 afterwards, and `add` is
`ADDS; CFINV` so that C holds the inverted carry when the next instruction
starts.

No AArch64 condition code tests bit 26, so the sidecar's `fcmovu` and
`fcmovnu` read NZCV into a register and branch over the move with `TBZ` or
`TBNZ` on bit 26. Its `fcomi` family writes ZF, PF and CF and leaves SF, OF
and AF clear, as the hardware does. An earlier version had `fcmovu` test V,
and `fcomi` copy the unordered result into V as well as bit 26 so that the
two agreed: a stock `ucomisd`, `sahf`, `popf` or integer instruction
followed by the sidecar's `fcmovu` read OF in place of PF, and a stock
`seto`, `jo` or `pushf` after the sidecar's `fucomi` of a NaN saw OF set.
`test_flags_handoff` runs each direction, in the same block and across a
block boundary, and the test harness runs it again with the compares and
with the `fcmov`s handed to stock.

No other x87 instruction writes EFLAGS, so code the sidecar emits for one
that sets NZCV for its own branches and selects has to put the guest's flags
back. Where a scratch register is free across that code it reads NZCV into
it first and writes it back after; where all of them are taken, as in the
80-bit conversions of `fld m80`, `fstp m80`, `fsave` and `frstor` and in
`fbstp`, it parks the flags in a floating-point register. An earlier
version did neither for those instructions or for `fxam` and `fxtract`, so
a guest `pushf` after any of them read the flags the conversion or the
classification had left, not its own. The reply boundary's own conversion
saves and restores NZCV as well. `test_flags_handoff` sets seven flag
patterns with `popf`, runs each x87 instruction the sidecar translates,
alone and in its fused and IR shapes, and checks them with `pushf`.

## When the translator runs out of registers

The IR pipeline checks a run's register pressure against the scratch pools
before lowering it, and splits the run when its model says the run does not
fit. The direct emitters need a few registers each and assume the pools have
them. Either can be wrong: the model can miscount, or stock can leave a
smaller pool than usual. A failed allocation used to assert, which killed
the sidecar and, through the stub, the game. NosTale crashed this way.

Now the allocator records the failure in the `TranslationResult` and hands
out a placeholder register, so the emitter runs to the end of the request
without touching the free masks. `Translator::translate_instruction` then
puts back what the request changed (the emitted words, the fixup lists, the
GPR and FPR masks and the x87 cache) and returns `nullopt`, and stock
translates the instruction. The sidecar logs one line naming the allocator,
the block hash and the instruction, and counts the declines in the
`X87_LOG_THROUGHPUT` line.

This is sound wherever in the request the failure happens, because a request
is a whole run (see the previous section): it starts with the cache released
and the x87 state complete in native format in memory, and none of its code
is kept, including the entry conversion. A failure after a split run has
lowered its first part, inside a bridged region or in the exit conversion is
discarded the same way. The x87 cache goes back to its state at the start of
the request with the run released, as after any declined request. When the
request had moved the cache onto a new block, or found stock translating the
block over from the start, the restored cache does not name the block, so
the next request in it enters the block afresh.

The rest of the block goes to stock as well: later requests in it are
declined without translating, until stock moves to another block or starts
this one over. The flags do not need this, since both translators keep them
in the same place (see [Guest flags in NZCV](#guest-flags-in-nzcv)); the
rule is kept because a failed request is rare and it only costs speed in
that block. Instructions translated before the failure stay. A block
declined from its first request gets no block profiler counter, and the
profiler's per-block diagnostics for a declined block can include the
declined attempt.

ARPL is the exception. Stock has no opcode for it, so a failure while
translating it still aborts, and an ARPL later in a declined block is still
translated. The register form, the only one seen in practice, needs four
scratch GPRs.

`X87_STRICT_ALLOC=1` keeps the assert, and the test harness sets it so a
pressure-model bug still fails a test. `ir_pressure_replay` and
`rollback_diff` always assert; `profile_analyze` declines and reports the
count in an `alloc_declines` row. `X87_FAIL_ALLOC_EVERY=N` fails every Nth
allocation in the process. The harness runs a set of tests with it at
several N without strict mode; `test_alloc_decline` covers long runs, split
runs, a bridged region, integer stores, compares, `fucomi` followed by
`fcmovu`, and a transcendental. When no allocation fails, the emitted code
is unchanged.

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
so `fild m64; fistp m64` copies any `int64_t`. Compilers use that pair to
move 64-bit integers atomically on 32-bit x86 without SSE2, and Delphi's
`System.Move` copies memory with it. The sidecar copies these values
exactly, within a block and across blocks. A reply made only of integer
loads and stores is a native transport reply (see
[Asynchronous signals inside emitted code](#asynchronous-signals-inside-emitted-code))
and keeps the 80-bit value in the native slot, so an int64 held on the
stack from one block to the next arrives intact. Inside a run that also
holds other instructions, the `fild_fistp` fusion (a lone pair, or up to
three nested pairs) and an IR pass copy the eight bytes as long as nothing
else reads the value. What remains is a value that meets arithmetic, or that
is on the stack while a reply with floating-point work runs, since that
reply converts its slots to binary64: above 2^53 in magnitude it is rounded,
and 2^53 + 1 comes back as 2^53 and 2^62 + 3 as 2^62. With the IR off, a
copy inside such a mixed run is exact only where the fusion matches it.

`fld m80; fstp m80`, the code for a `long double` or Delphi `Extended`
assignment, is copied the same way by the `fld_fstp_m80` fusion: the ten
bytes arrive unchanged, including values outside binary64's range, all 64
significand bits and NaN payloads. Only the adjacent pair is exact. A value
loaded with `fld m80` that meets any other instruction first, or reaches its
`fstp m80` in a later reply, is rounded to binary64. An 18-digit packed BCD
integer needs up to 60 significand bits, so by default a block containing
`fbld` or `fbstp` goes to stock Rosetta, whose 80-bit registers keep every
digit. `X87_STOCK_BCD=0` translates such blocks in the sidecar, which rounds
any BCD value above 2^53 in magnitude.

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

Rosetta maps MXCSR.RC to FPCR.RMode, and on hardware with FEAT_AFP MXCSR.FTZ
to FPCR.FZ and MXCSR.DAZ to FPCR.FIZ. Older Apple hardware has no FPCR.FIZ,
and its FPCR.FZ flushes inputs as well as outputs, so there Rosetta leaves
FPCR.FZ clear and puts FTZ and DAZ in bits 1 and 0 of the Apple register
`S3_6_C15_C2_5` instead. The runtime chooses from the commpage CPU
capabilities: bit 34 (FEAT_AFP) selects FPCR, bit 38 the Apple register. The
emitted x87 code runs under either. Real x87 ignores all three. So by
default a program that sets FTZ or DAZ for its SSE code also gets its x87
denormals flushed to zero, down to a plain `fld m32; fstp m32` copy of a
denormal float, which MSVC `/arch:IA32` code uses for float copies, and,
under DAZ, a denormal compares equal to zero. Stock Rosetta keeps them.

`X87_IGNORE_FTZ_DAZ=1` makes each translation reply clear the two bits on
entry and restore them on exit (see `emit_native_state_boundary`), in FPCR
or in the Apple register, chosen the same way as the runtime chooses;
`x87sidecar --probe` prints which. The Apple register traps at EL0 on
FEAT_AFP hardware, so the choice cannot be made at run time by trying, and
the Apple-register path cannot run on the M5 Max where the rest of this
section was measured; phase 15 of `run_tests.sh` covers it on hardware
without FEAT_AFP, such as the CI runners. It is off by default because it
has a cost for every program, not only for the ones that set FTZ or DAZ.
With neither bit set and an empty x87 stack at both ends of the reply, a
reply gains six instructions and no taken branch. With values on the stack,
entry gains nine instructions and one taken branch and exit four
instructions and one taken branch. On the benchmarks that costs up to 36
percent on the one-instruction loops of `bench_load` and `bench_store`, 3 to
28 percent on the short chains of `bench_fstp_fld`, and up to 13 percent on
the dot product, `f2xm1` and `fsin` benchmarks. With a bit set, the two FPCR
writes cost about 20 ns per reply: a loop around `fld m32; fstp m32` goes
from under 1 ns per iteration to 22 ns, where stock Rosetta takes 10 ns, and
a loop around a six-instruction arithmetic run goes from 1.2 ns to 23 ns,
where stock takes 72 ns. With the knob off the emitted code is the same as
without the feature.

With the knob on, a signal handler that interrupts an x87 run sees FTZ and
DAZ clear: `stmxcsr` in the handler reads them as off, because the handler
starts with the FPCR the run had cleared. The MXCSR saved in the signal
context is correct, and the interrupted code gets FTZ and DAZ back when the
handler returns. Under a SIGUSR1 storm against a loop of short x87 runs with
both bits set, between 0.02 and 4 percent of the handlers saw them clear,
depending on the configuration and the run.

On Macs without FEAT_AFP, a signal that lands inside an x87 reply can make
the rest of that reply give wrong results even with the knob on. The
macos-26 CI runners show wrong results in the same storm: 8 to 17 percent of
the iterations, one for every three or four signals, while MXCSR after every
iteration was still correct and every check without signals passed. That
the wrong results are flushes is inferred, not yet confirmed: the test now
prints the first wrong result, fills the outputs with a sentinel before each
iteration so a store that never happened cannot pass for +0, and on such a
runtime tolerates an iteration only when every output is either its
expected value or +0, reporting the count as a NOTE; any other value still
fails. The cause was not verified on that hardware either. Two explanations
fit the data, and both stay inferred until a CI log shows the first wrong
result. One is that the runtime's signal handling puts the guest's FTZ and
DAZ back into `S3_6_C15_C2_5` before the rest of the reply runs: its
context-restore path rewrites that register from the saved MXCSR, and the
code that steps a translation forward to the next instruction-map entry
treats that register's encoding as a special case, so a step through the
reply may not keep the value the reply wrote. The other is that the
stepper skips the `msr S3_6_C15_C2_5` at reply entry, which it may not
decode, so the reply runs with the guest's flush bits from the start. On
FEAT_AFP hardware the same storm returns no wrong results.

The knob keeps the original FZ and FIZ bits in the halfword at +6 of the x87
state. That halfword belongs to native slot 0, and is free only because the
compact layout keeps its doubles at +8, stride 8, and the entry and exit
conversions read and write slot 0 before and after the halfword is used.
Native transport replies do work on the native slots (+6, stride 10), and
are still safe: they skip the boundary, so they neither write nor read the
saved bits, and their integer conversions and `LDR D` / `STR D` copies do no
floating-point arithmetic, so FZ and FIZ do not affect them. Any other reply
that kept values inside the native slots would need the saved bits moved
somewhere else.

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
