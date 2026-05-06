/*
 * test_fma_reduce.c — bit-exact validation of the FMA-reduction vector
 * lowering pass (X87IROptimize.cpp::pass_fma_reduce + X87IRLower.cpp::
 * lower_fma_reduce).
 *
 * Each chain is a serial dot-product reduction:
 *
 *     A_init = some_value
 *     A_{i+1} = A_i + (data[i] * weight[i])     for i = 0 .. N-1
 *
 * Lowered today as N scalar FMADDs.  Under X87_ENABLE_FMA_REDUCE=1 the
 * pass folds them into pair-loaded LDR D + FCVTL .2D + FMLA .2D, with a
 * scalar FADDP horizontal sum (and an odd-trio scalar FMADD tail when N
 * is odd).
 *
 * Inputs are chosen so the result is exactly representable in f64 — small
 * non-negative integers whose products and partial sums never round.
 * Both the scalar-serial chain and the lane-pair vector chain therefore
 * compute the identical f64 bit-pattern, allowing a `memcmp`-style
 * comparison even though the vector path re-associates additions.
 *
 * Run by run_tests.sh under both Phase 2 (default — scalar FMADD path)
 * and Phase 6 (X87_ENABLE_FMA_REDUCE=1 — vector path).  Both phases must
 * produce the same expected value.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static uint64_t as_u64(double d) {
    uint64_t u;
    memcpy(&u, &d, 8);
    return u;
}

static void check_equal(const char* name, double got, double expected) {
    if (as_u64(got) != as_u64(expected)) {
        printf("FAIL  %-40s got=%.17g (0x%016llx) expected=%.17g (0x%016llx)\n", name, got,
               (unsigned long long)as_u64(got), expected, (unsigned long long)as_u64(expected));
        failures++;
    } else {
        printf("PASS  %-40s = %.17g\n", name, got);
    }
}

/* Storage: f32 streams.  __attribute__((aligned(16))) keeps base 16-aligned
 * so future LDR Q escalation finds aligned pairs. */
static float data[16] __attribute__((aligned(16)));
static float weight[16] __attribute__((aligned(16)));

/*
 * Inline-asm chain dispatchers.  Each emits exactly N trios of
 *   (fld dword [data+4*i] ; fmul dword [weight+4*i] ; faddp st1, st0)
 * preceded by `fldl init` to seed ST(0)=A_init.  The post-chain `fstpl`
 * stores the chain tail into *out and pops, leaving the x87 stack
 * balanced (no slot remains live across the asm boundary).
 */

static void run_chain_n2(double init, double* out) {
    __asm__ volatile(
        "fldl %2\n\t"          /* ST(0) = init                                */
        "flds 0(%0)\n\t"       /* push data[0]: ST(0)=L0, ST(1)=init           */
        "fmuls 0(%1)\n\t"      /* ST(0) *= weight[0]                          */
        "faddp %%st(0), %%st(1)\n\t" /* ST(1) += ST(0); pop                   */
        "flds 4(%0)\n\t"
        "fmuls 4(%1)\n\t"
        "faddp %%st(0), %%st(1)\n\t"
        "fstpl %3\n"           /* *out = ST(0); pop                            */
        :
        : "r"(data), "r"(weight), "m"(init), "m"(*out)
        : "memory");
}

static void run_chain_n3(double init, double* out) {
    __asm__ volatile(
        "fldl %2\n\t"
        "flds 0(%0)\n\t  fmuls 0(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "flds 4(%0)\n\t  fmuls 4(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "flds 8(%0)\n\t  fmuls 8(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "fstpl %3\n"
        :
        : "r"(data), "r"(weight), "m"(init), "m"(*out)
        : "memory");
}

static void run_chain_n4(double init, double* out) {
    __asm__ volatile(
        "fldl %2\n\t"
        "flds 0(%0)\n\t   fmuls 0(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
        "flds 4(%0)\n\t   fmuls 4(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
        "flds 8(%0)\n\t   fmuls 8(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
        "flds 12(%0)\n\t  fmuls 12(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "fstpl %3\n"
        :
        : "r"(data), "r"(weight), "m"(init), "m"(*out)
        : "memory");
}

static void run_chain_n8(double init, double* out) {
    __asm__ volatile(
        "fldl %2\n\t"
        "flds 0(%0)\n\t   fmuls 0(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
        "flds 4(%0)\n\t   fmuls 4(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
        "flds 8(%0)\n\t   fmuls 8(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
        "flds 12(%0)\n\t  fmuls 12(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "flds 16(%0)\n\t  fmuls 16(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "flds 20(%0)\n\t  fmuls 20(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "flds 24(%0)\n\t  fmuls 24(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "flds 28(%0)\n\t  fmuls 28(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "fstpl %3\n"
        :
        : "r"(data), "r"(weight), "m"(init), "m"(*out)
        : "memory");
}

/* Compute the expected result via straightforward f64 arithmetic.
 * Using exactly-representable integer values guarantees the scalar
 * left-to-right chain and the lane-pair vector chain produce the
 * identical f64 bit-pattern. */
static double expected_dot(double init, int n) {
    double acc = init;
    for (int i = 0; i < n; i++) {
        acc += (double)data[i] * (double)weight[i];
    }
    return acc;
}

int main(void) {
    /* Small non-negative integers — products and partial sums all stay
     * below 2^53 and are exactly representable in f64. */
    for (int i = 0; i < 16; i++) {
        data[i] = (float)(i + 1);
        weight[i] = (float)(i + 1);
    }

    /* Sanity: hand-computed sums match expected_dot. */
    double init = 100.0;
    double got;

    run_chain_n2(init, &got);
    /* 100 + 1*1 + 2*2 = 105 */
    check_equal("n2  init=100 i*i", got, expected_dot(init, 2));

    run_chain_n3(init, &got);
    /* 100 + 1+4+9 = 114 (odd-trio path) */
    check_equal("n3  init=100 i*i (odd tail)", got, expected_dot(init, 3));

    run_chain_n4(init, &got);
    /* 100 + 1+4+9+16 = 130 (two pairs, no odd tail) */
    check_equal("n4  init=100 i*i", got, expected_dot(init, 4));

    run_chain_n8(init, &got);
    /* 100 + 1+4+9+16+25+36+49+64 = 304 (four pairs) */
    check_equal("n8  init=100 i*i", got, expected_dot(init, 8));

    /* Different init values — exercises the FMOV-V_acc-from-A_init seed. */
    run_chain_n4(0.0, &got);
    check_equal("n4  init=0   i*i", got, expected_dot(0.0, 4));

    run_chain_n4(-50.0, &got);
    check_equal("n4  init=-50 i*i", got, expected_dot(-50.0, 4));

    /* Different data: ones — verifies pair-loaded f32 → f64 widening
     * doesn't drop precision when both lanes hold the same value. */
    for (int i = 0; i < 16; i++) {
        data[i] = 1.0f;
        weight[i] = 1.0f;
    }
    run_chain_n4(7.0, &got);
    /* 7 + 1+1+1+1 = 11 */
    check_equal("n4  init=7   ones", got, expected_dot(7.0, 4));

    run_chain_n8(0.0, &got);
    /* 0 + 8*1 = 8 */
    check_equal("n8  init=0   ones", got, expected_dot(0.0, 8));

    /* Mixed signs — partial sums still exact in f64 with these magnitudes. */
    for (int i = 0; i < 16; i++) {
        data[i] = (i % 2 == 0) ? (float)(i + 1) : -(float)(i + 1);
        weight[i] = (float)(i + 1);
    }
    run_chain_n4(0.0, &got);
    /* 0 + 1*1 + (-2)*2 + 3*3 + (-4)*4 = 1 - 4 + 9 - 16 = -10 */
    check_equal("n4  init=0   mixed", got, expected_dot(0.0, 4));

    run_chain_n8(0.0, &got);
    /* 1 - 4 + 9 - 16 + 25 - 36 + 49 - 64 = -36 */
    check_equal("n8  init=0   mixed", got, expected_dot(0.0, 8));

    if (failures == 0) {
        printf("\nALL PASS\n");
        return 0;
    }
    printf("\n%d FAIL(s)\n", failures);
    return 1;
}
