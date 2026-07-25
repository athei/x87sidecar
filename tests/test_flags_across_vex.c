/*
 * test_flags_across_vex.c -- guest EFLAGS survival across VEX/BMI2 ops.
 *
 * Companion to test_flags_across_x87.c, which covers flags living across
 * x87 ops.  This one covers the shape that showed up in the field: a
 * carry produced by `sub`/`add` and consumed by a later `sbb`/`adc`, with
 * VEX-encoded moves (vmovd/vpextrd) and BMI2 `mulx` sitting between the
 * producer and the consumer.  None of those instructions touch EFLAGS on
 * x86, so the carry must survive them untouched.
 *
 * Why it exists: a tester's 64-bit tick subtraction, compiled for i686 as
 * `sub`/`sbb` with exactly that VEX filling between the halves, came out
 * with its high dword one too small -- the `sbb` read CF as 1 when the `sub`
 * had cleared it, so a borrow that never happened turned a high dword of 0
 * into 0xffffffff.  The error was exactly -2^32 ticks, every time, and it
 * reproduced under both the upstream dylib JIT and this sidecar, which is
 * why the sequence is pinned here as a regression test rather than left as
 * a field anecdote.
 *
 * Read the result through the phases of scripts/run_tests.sh: a failure in
 * phase 1/5 (native Rosetta, hook disabled) is stock's; a failure in phase
 * 2 that clears with X87_ENABLE_BRIDGE=0 or X87_BRIDGE_V2=0 is ours.
 *
 * Width caveat: samples build -arch x86_64, while the field case is i686.
 * The halves here are deliberately 32-bit `subl`/`sbbl` on 32-bit register
 * operands so the guest sequence matches what the i686 compiler emitted,
 * but the encoding and the translator's register mapping still differ from
 * a real 32-bit process.  A clean run here does not fully exonerate a
 * 32-bit one.
 *
 * The x87-adjacent variants matter most: with no x87 in the block the
 * sidecar never sees the code at all and only stock is under test.
 */
#include <stdio.h>

static volatile double vx = 0.5;
static double r1;
/* mulx multiplicand and source; volatile so nothing is constant-folded. */
static volatile unsigned long long vm = 0x9e3779b97f4a7c15ull;

/*
 * Section A -- a cmp's flags read back by setcc after a VEX filling.
 *
 * Same probe shape and case table as test_flags_across_x87.c:
 * (5,5) -> eq=1 lt=0; (3,9) -> eq=0 lt=1; (9,3) -> eq=0 lt=0.
 */

__attribute__((target("avx2,bmi2")))
static int probe_vmovd(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t"
        "vmovd %[a], %%xmm0\n\tvmovd %%xmm0, %%r10d\n\t"
        "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt)
        : [a] "r"(a), [b] "r"(b)
        : "cc", "xmm0", "r10");
    return (eq << 1) | lt;
}

__attribute__((target("avx2,bmi2")))
static int probe_vpextrd(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t"
        "vmovd %[a], %%xmm0\n\tvpextrd $0, %%xmm0, %%r10d\n\t"
        "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt)
        : [a] "r"(a), [b] "r"(b)
        : "cc", "xmm0", "r10");
    return (eq << 1) | lt;
}

__attribute__((target("avx2,bmi2")))
static int probe_mulx(unsigned a, unsigned b) {
    unsigned char eq, lt;
    unsigned long long lo, hi;
    __asm__ volatile("cmpl %[b], %[a]\n\t"
        "mulxq %[m], %[lo], %[hi]\n\tmulxq %[m], %[lo], %[hi]\n\t"
        "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [lo] "=&r"(lo), [hi] "=&r"(hi)
        : [a] "r"(a), [b] "r"(b), [m] "m"(vm), "d"(0x0123456789abcdefull)
        : "cc");
    return (eq << 1) | lt;
}

/* The production filling: a GPR-to-XMM round trip, then two mulx. */
__attribute__((target("avx2,bmi2")))
static int probe_field(unsigned a, unsigned b) {
    unsigned char eq, lt;
    unsigned long long lo, hi;
    __asm__ volatile("cmpl %[b], %[a]\n\t"
        "vmovd %[a], %%xmm0\n\tvpextrd $0, %%xmm0, %%r10d\n\t"
        "mulxq %[m], %[lo], %[hi]\n\tmulxq %[m], %[lo], %[hi]\n\t"
        "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [lo] "=&r"(lo), [hi] "=&r"(hi)
        : [a] "r"(a), [b] "r"(b), [m] "m"(vm), "d"(0x0123456789abcdefull)
        : "cc", "xmm0", "r10");
    return (eq << 1) | lt;
}

/* Same, with x87 in the span so the sidecar owns part of the block. */
__attribute__((target("avx2,bmi2")))
static int probe_field_x87(unsigned a, unsigned b) {
    unsigned char eq, lt;
    unsigned long long lo, hi;
    __asm__ volatile("cmpl %[b], %[a]\n\t"
        "vmovd %[a], %%xmm0\n\tvpextrd $0, %%xmm0, %%r10d\n\t"
        "fldl %[x]\n\tfstpl %[r]\n\t"
        "mulxq %[m], %[lo], %[hi]\n\tmulxq %[m], %[lo], %[hi]\n\t"
        "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [lo] "=&r"(lo), [hi] "=&r"(hi), [r] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [m] "m"(vm), [x] "m"(vx), "d"(0x0123456789abcdefull)
        : "cc", "xmm0", "r10", "st");
    return (eq << 1) | lt;
}

/* x87 arithmetic, not just a load/store pair, between producer and reader. */
__attribute__((target("avx2,bmi2")))
static int probe_x87_arith(unsigned a, unsigned b) {
    unsigned char eq, lt;
    unsigned long long lo, hi;
    __asm__ volatile("cmpl %[b], %[a]\n\t"
        "fldl %[x]\n\tfldl %[x]\n\tfaddp\n\tfstpl %[r]\n\t"
        "vmovd %[a], %%xmm0\n\tvpextrd $0, %%xmm0, %%r10d\n\t"
        "mulxq %[m], %[lo], %[hi]\n\t"
        "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [lo] "=&r"(lo), [hi] "=&r"(hi), [r] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [m] "m"(vm), [x] "m"(vx), "d"(0x0123456789abcdefull)
        : "cc", "xmm0", "r10", "st", "st(1)");
    return (eq << 1) | lt;
}

/*
 * Section B -- the field shape itself: a 64-bit value subtracted as two
 * 32-bit halves, with the borrow carried across the VEX filling.  A borrow
 * that goes missing leaves the high half one too large; one invented out of
 * nothing leaves it one too small.  Both are +/-2^32 on the whole value.
 */

__attribute__((target("avx2,bmi2")))
static unsigned long long sub64_across_vex(unsigned long long a, unsigned long long b, int with_x87) {
    unsigned al = (unsigned)a, ah = (unsigned)(a >> 32);
    unsigned bl = (unsigned)b, bh = (unsigned)(b >> 32);
    unsigned long long lo, hi;
    if (with_x87) {
        __asm__ volatile("subl %[bl], %[al]\n\t"
            "vmovd %[bl], %%xmm0\n\tvpextrd $0, %%xmm0, %%r10d\n\t"
            "fldl %[x]\n\tfstpl %[r]\n\t"
            "mulxq %[m], %[lo], %[hi]\n\tmulxq %[m], %[lo], %[hi]\n\t"
            "sbbl %[bh], %[ah]\n\t"
            : [al] "+&r"(al), [ah] "+&r"(ah), [lo] "=&r"(lo), [hi] "=&r"(hi), [r] "=m"(r1)
            : [bl] "r"(bl), [bh] "r"(bh), [m] "m"(vm), [x] "m"(vx), "d"(0x0123456789abcdefull)
            : "cc", "xmm0", "r10", "st");
    } else {
        __asm__ volatile("subl %[bl], %[al]\n\t"
            "vmovd %[bl], %%xmm0\n\tvpextrd $0, %%xmm0, %%r10d\n\t"
            "mulxq %[m], %[lo], %[hi]\n\tmulxq %[m], %[lo], %[hi]\n\t"
            "sbbl %[bh], %[ah]\n\t"
            : [al] "+&r"(al), [ah] "+&r"(ah), [lo] "=&r"(lo), [hi] "=&r"(hi)
            : [bl] "r"(bl), [bh] "r"(bh), [m] "m"(vm), "d"(0x0123456789abcdefull)
            : "cc", "xmm0", "r10");
    }
    return ((unsigned long long)ah << 32) | al;
}

/*
 * Section C -- the same for a carry: `add` low half, VEX filling, `adc`
 * high half.  This is the shape behind the other half of the field report,
 * where a spurious carry landed in a high limb of a widening conversion.
 */

__attribute__((target("avx2,bmi2")))
static unsigned long long add64_across_vex(unsigned long long a, unsigned long long b, int with_x87) {
    unsigned al = (unsigned)a, ah = (unsigned)(a >> 32);
    unsigned bl = (unsigned)b, bh = (unsigned)(b >> 32);
    unsigned long long lo, hi;
    if (with_x87) {
        __asm__ volatile("addl %[bl], %[al]\n\t"
            "vmovd %[bl], %%xmm0\n\tvpextrd $0, %%xmm0, %%r10d\n\t"
            "fldl %[x]\n\tfstpl %[r]\n\t"
            "mulxq %[m], %[lo], %[hi]\n\tmulxq %[m], %[lo], %[hi]\n\t"
            "adcl %[bh], %[ah]\n\t"
            : [al] "+&r"(al), [ah] "+&r"(ah), [lo] "=&r"(lo), [hi] "=&r"(hi), [r] "=m"(r1)
            : [bl] "r"(bl), [bh] "r"(bh), [m] "m"(vm), [x] "m"(vx), "d"(0x0123456789abcdefull)
            : "cc", "xmm0", "r10", "st");
    } else {
        __asm__ volatile("addl %[bl], %[al]\n\t"
            "vmovd %[bl], %%xmm0\n\tvpextrd $0, %%xmm0, %%r10d\n\t"
            "mulxq %[m], %[lo], %[hi]\n\tmulxq %[m], %[lo], %[hi]\n\t"
            "adcl %[bh], %[ah]\n\t"
            : [al] "+&r"(al), [ah] "+&r"(ah), [lo] "=&r"(lo), [hi] "=&r"(hi)
            : [bl] "r"(bl), [bh] "r"(bh), [m] "m"(vm), "d"(0x0123456789abcdefull)
            : "cc", "xmm0", "r10");
    }
    return ((unsigned long long)ah << 32) | al;
}

/*
 * Section C2 -- the field span verbatim, `rdtsc` included.
 *
 * The shipped build's carry pair straddles an `rdtsc`:
 *
 *     subl  %esi, %eax          ; CF out
 *     movl  %eax, 0x24(%esp)
 *     rdtsc
 *     movl/movq/movd/pextrd, mulx
 *     sbbl  0x1c(%esp), %edx    ; CF in
 *
 * `rdtsc` is the one instruction in that span with no ARM equivalent, so
 * the translator has to synthesize it (read the counter, scale, split into
 * edx:eax) rather than map it -- which makes it the likeliest place for the
 * emulated carry to be dropped, and the reason this probe exists separately
 * from the pure-VEX ones above.
 *
 * `rdtsc` writes eax/edx, so those are clobbers here and the operands land
 * elsewhere; `mulx` then multiplies whatever the counter left in rdx, which
 * is fine because only the flags are under test.
 */

static int probe_rdtsc(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t"
        "rdtsc\n\t"
        "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt)
        : [a] "r"(a), [b] "r"(b)
        : "cc", "eax", "edx");
    return (eq << 1) | lt;
}

__attribute__((target("avx2,bmi2")))
static unsigned long long sub64_across_rdtsc(unsigned long long a, unsigned long long b) {
    unsigned al = (unsigned)a, ah = (unsigned)(a >> 32);
    unsigned bl = (unsigned)b, bh = (unsigned)(b >> 32);
    unsigned long long lo, hi;
    __asm__ volatile("subl %[bl], %[al]\n\t"
        "rdtsc\n\t"
        "vmovd %[bl], %%xmm0\n\tvpextrd $0, %%xmm0, %%r10d\n\t"
        "mulxq %[m], %[lo], %[hi]\n\t"
        "sbbl %[bh], %[ah]\n\t"
        : [al] "+&r"(al), [ah] "+&r"(ah), [lo] "=&r"(lo), [hi] "=&r"(hi)
        : [bl] "r"(bl), [bh] "r"(bh), [m] "m"(vm)
        : "cc", "eax", "edx", "xmm0", "r10");
    return ((unsigned long long)ah << 32) | al;
}

/*
 * Section D -- gaps narrow enough to be bridged.
 *
 * Sections A-C fill the span with mulx/vmovd/vpextrd, none of which
 * is_bridge_v1/v2 accepts, so those blocks always split and only stock's
 * spill-around behaviour is under test.  Bridging needs a gap of at most
 * X87_BRIDGE_MAX_GAP (default 2) instructions BETWEEN two x87 runs, so
 * reaching it takes a different shape: x87 run, one-instruction gap, x87
 * run, then the flag reader.
 *
 * The hazard these cover is v2's flag-deadness proof.  A bridged ALU op
 * lowers to non-flag-setting ARM, which is only sound if nothing reads the
 * flags it should have written.  If `flag_liveness` reads 0 for a `sub`
 * whose CF a later `sbb` does consume, the `sbb` sees whatever carry was left
 * lying around instead of the one the `sub` meant to produce -- the same
 * failure shape as the field case, in either direction.
 */

/*
 * The x87 runs here are `fldl`/`fmull` pairs closed by `faddp`/`fstpl`,
 * matching test_bridge_alu.c -- a bare fldl/fstpl pair does not engage the
 * IR path, so the gap between two of those is never offered to the bridge
 * (verified with X87_LOG_BRIDGE=1: zero bridge lines).
 */
static const double bx[4] = {2.0, 3.0, 5.0, 7.0};

/*
 * A CF producer in the gap whose consumer sits after the region.  This is
 * the case test_bridge_alu.c does not cover: its gap writers all have
 * provably dead flags, and its one pass-through case (`dec`, scenario 6)
 * never writes CF.  Here the `subl`'s CF is live into the `sbbl`, so
 * flag_liveness must be nonzero and the region must NOT be v2-bridged.
 * A wrong carry shows up as a high half off by exactly one, either way.
 */
static unsigned long long sub64_bridged_gap(unsigned long long a, unsigned long long b) {
    unsigned al = (unsigned)a, ah = (unsigned)(a >> 32);
    unsigned bl = (unsigned)b, bh = (unsigned)(b >> 32);
    double out;
    __asm__ volatile(
        "fldl (%[p])\n\tfmull 8(%[p])\n\t"
        "subl %[bl], %[al]\n\t"
        "fldl 16(%[p])\n\tfmull 24(%[p])\n\t"
        "faddp\n\tfstpl %[o]\n\t"
        "sbbl %[bh], %[ah]\n\t"
        : [al] "+&r"(al), [ah] "+&r"(ah), [o] "=m"(out)
        : [bl] "r"(bl), [bh] "r"(bh), [p] "r"(bx)
        : "cc", "st", "st(1)", "memory");
    return ((unsigned long long)ah << 32) | al;
}

/*
 * Counterfactual for the probe above: identical region, but the `subl`'s
 * flags are killed by a trailing `testl` before anything reads them, so
 * the proof holds and the region IS bridged.  Its value is the bridge log,
 * not the assertion -- with X87_LOG_BRIDGE=1 this shape produces a joined
 * region while the live-carry one does not, which is what makes the
 * refusal above liveness-driven rather than never-offered.
 */
static unsigned sub32_bridged_dead(unsigned a, unsigned b) {
    unsigned al = a;
    double out;
    __asm__ volatile(
        "fldl (%[p])\n\tfmull 8(%[p])\n\t"
        "subl %[b], %[al]\n\t"
        "fldl 16(%[p])\n\tfmull 24(%[p])\n\t"
        "faddp\n\tfstpl %[o]\n\t"
        "testl %[al], %[al]\n\t"
        : [al] "+&r"(al), [o] "=m"(out)
        : [b] "r"(b), [p] "r"(bx)
        : "cc", "st", "st(1)", "memory");
    return al;
}

/* Flags passing through a v1 (never-flag-writing) bridge candidate. */
static int probe_bridge_mov(unsigned a, unsigned b) {
    unsigned char eq, lt;
    unsigned scratch;
    double out;
    __asm__ volatile("cmpl %[b], %[a]\n\t"
        "fldl (%[p])\n\tfmull 8(%[p])\n\t"
        "movl %[a], %[s]\n\t"
        "fldl 16(%[p])\n\tfmull 24(%[p])\n\t"
        "faddp\n\tfstpl %[o]\n\t"
        "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [s] "=&r"(scratch), [o] "=m"(out)
        : [a] "r"(a), [b] "r"(b), [p] "r"(bx)
        : "cc", "st", "st(1)", "memory");
    return (eq << 1) | lt;
}

/*
 * A partial writer in the gap with its own written flag live.  `inc`
 * writes ZF/SF/OF/AF and leaves CF alone, so a correct run reads the
 * inc's ZF (1 -> 2, never zero) and the cmp's CF.  Bridged, `inc` writes
 * no flags at all, and the cmp's ZF would leak into the `sete` -- so this
 * probe pins both the pass-through of CF and the refusal to bridge when
 * the gap writer's own flags are live.
 */
static int probe_bridge_inc(unsigned a, unsigned b) {
    unsigned char eq, lt;
    unsigned scratch = 1;
    double out;
    __asm__ volatile("cmpl %[b], %[a]\n\t"
        "fldl (%[p])\n\tfmull 8(%[p])\n\t"
        "incl %[s]\n\t"
        "fldl 16(%[p])\n\tfmull 24(%[p])\n\t"
        "faddp\n\tfstpl %[o]\n\t"
        "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [s] "+&r"(scratch), [o] "=m"(out)
        : [a] "r"(a), [b] "r"(b), [p] "r"(bx)
        : "cc", "st", "st(1)", "memory");
    return (eq << 1) | lt;
}

typedef int (*probe_fn)(unsigned, unsigned);

int main(void) {
    int fails = 0;
    struct { const char *name; probe_fn fn; } probes[] = {
        {"vmovd", probe_vmovd},
        {"vpextrd", probe_vpextrd},
        {"mulx", probe_mulx},
        {"field", probe_field},
        {"field_x87", probe_field_x87},
        {"x87_arith", probe_x87_arith},
        {"bridge_mov", probe_bridge_mov},
        {"rdtsc", probe_rdtsc},
    };
    struct { unsigned a, b; int want; } cases[] = {{5, 5, 2}, {3, 9, 1}, {9, 3, 0}};
    for (unsigned p = 0; p < sizeof(probes) / sizeof(probes[0]); p++) {
        for (int i = 0; i < 3; i++) {
            int g = probes[p].fn(cases[i].a, cases[i].b);
            if (g != cases[i].want) {
                printf("FAIL %-10s case %d: got %d want %d\n", probes[p].name, i, g, cases[i].want);
                fails++;
                break;
            }
        }
    }

    /*
     * The `inc` in the gap writes ZF, so a correct run reads the INC's ZF
     * (never zero here: 1 -> 2) and the CMP's CF, which `inc` leaves alone.
     * eq=1 would mean the cmp's ZF leaked through an unsound bridge.
     */
    struct { unsigned a, b; int want; } inc_cases[] = {{5, 5, 0}, {3, 9, 1}, {9, 3, 0}};
    for (int i = 0; i < 3; i++) {
        int g = probe_bridge_inc(inc_cases[i].a, inc_cases[i].b);
        if (g != inc_cases[i].want) {
            printf("FAIL %-10s case %d: got %d want %d\n", "bridge_inc", i, g, inc_cases[i].want);
            fails++;
            break;
        }
    }

    /*
     * Both polarities, because a one-sided table cannot see a one-sided bug.
     * The first three borrow out of the low half, so they catch a borrow that
     * goes missing. The last three do NOT borrow, which is the case the field
     * failure needed: there the halves came from two counter reads a few
     * microseconds apart, the high dwords were equal and the low halves did
     * not borrow, and the `sbb` subtracted a borrow anyway -- turning a high
     * dword of 0 into 0xffffffff and the whole value into ~2^64.
     */
    struct { unsigned long long a, b; } sub_cases[] = {
        {0x0000000100000000ull, 0x0000000000000001ull},
        {0xfedcba9876543210ull, 0x0000000087654321ull},
        {0x0000000200000000ull, 0x0000000100000001ull},
        {0x0000004200000010ull, 0x0000004200000001ull},
        {0xfedcba9987654321ull, 0x0000000012345678ull},
        {0x0000000100000000ull, 0x0000000100000000ull},
    };
    for (unsigned i = 0; i < sizeof(sub_cases) / sizeof(sub_cases[0]); i++) {
        unsigned long long want = sub_cases[i].a - sub_cases[i].b;
        for (int x87 = 0; x87 < 2; x87++) {
            unsigned long long got = sub64_across_vex(sub_cases[i].a, sub_cases[i].b, x87);
            if (got != want) {
                printf("FAIL sub64%-7s case %u: got %016llx want %016llx (delta %+lld * 2^32)\n",
                       x87 ? "_x87" : "", i, got, want,
                       (long long)((got >> 32) - (want >> 32)));
                fails++;
            }
        }
        unsigned long long got = sub64_bridged_gap(sub_cases[i].a, sub_cases[i].b);
        if (got != want) {
            printf("FAIL sub64%-7s case %u: got %016llx want %016llx (delta %+lld * 2^32)\n",
                   "_bridged", i, got, want, (long long)((got >> 32) - (want >> 32)));
            fails++;
        }
        got = sub64_across_rdtsc(sub_cases[i].a, sub_cases[i].b);
        if (got != want) {
            printf("FAIL sub64%-7s case %u: got %016llx want %016llx (delta %+lld * 2^32)\n",
                   "_rdtsc", i, got, want, (long long)((got >> 32) - (want >> 32)));
            fails++;
        }
        unsigned lo_want = (unsigned)sub_cases[i].a - (unsigned)sub_cases[i].b;
        unsigned lo_got = sub32_bridged_dead((unsigned)sub_cases[i].a, (unsigned)sub_cases[i].b);
        if (lo_got != lo_want) {
            printf("FAIL sub32_dead    case %u: got %08x want %08x\n", i, lo_got, lo_want);
            fails++;
        }
    }

    /* Both polarities again: the first three carry out of the low half, the
     * rest must not, so an invented carry has somewhere to show. */
    struct { unsigned long long a, b; } add_cases[] = {
        {0x00000000ffffffffull, 0x0000000000000001ull},
        {0x12345678fedcba98ull, 0x0000000123456789ull},
        {0x00000001fffffffeull, 0x0000000200000003ull},
        {0x0000004200000010ull, 0x0000000100000001ull},
        {0x12345678000000feull, 0x0000000100000001ull},
        {0x0000000100000000ull, 0x0000000100000000ull},
    };
    for (unsigned i = 0; i < sizeof(add_cases) / sizeof(add_cases[0]); i++) {
        unsigned long long want = add_cases[i].a + add_cases[i].b;
        for (int x87 = 0; x87 < 2; x87++) {
            unsigned long long got = add64_across_vex(add_cases[i].a, add_cases[i].b, x87);
            if (got != want) {
                printf("FAIL add64%-5s case %u: got %016llx want %016llx (delta %+lld * 2^32)\n",
                       x87 ? "_x87" : "", i, got, want,
                       (long long)((got >> 32) - (want >> 32)));
                fails++;
            }
        }
    }

    printf(fails ? "FAIL flags-across-vex (%d probes)\n" : "PASS flags-across-vex\n", fails);
    return fails != 0;
}
