/*
 * test_readst_elide.c -- Demonstrate ReadSt write-back elision in the IR epilogue.
 *
 * The optimization: when the IR epilogue would store a ReadSt node back to the
 * same physical slot it was loaded from, the store is a no-op and is skipped.
 *
 * To trigger this, we need the IR to run on PRE-EXISTING stack values (not loaded
 * within the run). We achieve this by splitting x87 sequences with non-x87
 * instructions (MOV), forcing separate IR runs. The second run operates on
 * values already in the x87 register file from the first run.
 *
 * Key pattern (Run 2): FADD ST(0),ST(2) + FMUL ST(0),ST(1) + FST [mem]
 *   - resolve(1) creates ReadSt(initial_depth=1), stays in slot_val[1]
 *   - resolve(2) creates ReadSt(initial_depth=2), stays in slot_val[2]
 *   - Epilogue: d=1: 1 == 1+0 → ELIDE.  d=2: 2 == 2+0 → ELIDE.
 *   - Without optimization: 3 stores.  With: 1 store.  Saves 2 stores.
 *
 * Build: clang -arch x86_64 -O0 -g -o test_readst_elide test_readst_elide.c
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

static void check(const char* name, double got, double expected) {
    if (as_u64(got) != as_u64(expected)) {
        printf("FAIL  %-55s  got=%.17g  expected=%.17g\n", name, got, expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

/*
 * Run 1: FLD a, FLD b, FLD c  (3 pushes)
 *   Stack: ST(0)=c=5, ST(1)=b=3, ST(2)=a=2
 *   MOV breaks the run.
 *
 * Run 2: FADD ST(0),ST(2) + FMUL ST(0),ST(1) + FST [result]
 *   FADD: resolve(0)→ReadSt(0), resolve(2)→ReadSt(2). ST(0) = 5+2 = 7
 *   FMUL: resolve(1)→ReadSt(1). ST(0) = 7*3 = 21
 *   FST: non-popping store of 21 to result.
 *
 *   slot_val = [FMul, ReadSt(1), ReadSt(2)], top_delta = 0
 *   Epilogue WITHOUT optimization: stores all 3 back to x87 register file.
 *   Epilogue WITH optimization: stores only FMul (d=0). ReadSt(1) at d=1 and
 *     ReadSt(2) at d=2 are elided (initial_depth == d + 0).
 *
 *   MOV breaks the run.
 *
 * Run 3: FSTP ST(0), FSTP [r1], FSTP [r2]
 *   Reads the remaining values and verifies they survived correctly.
 */
static void test_elision_3deep(double* result, double* r1, double* r2) {
    volatile double a = 2.0, b = 3.0, c = 5.0;
    int dummy;
    __asm__ volatile(
        /* Run 1: push 3 values */
        "fldl %4\n\t" /* ST(0)=a=2 */
        "fldl %5\n\t" /* ST(0)=b=3, ST(1)=2 */
        "fldl %6\n\t" /* ST(0)=c=5, ST(1)=3, ST(2)=2 */
        /* Break: non-x87 instruction */
        "movl $0, %3\n\t"
        /* Run 2: non-popping arithmetic (ReadSt elision target) */
        "fadd %%st(2), %%st\n\t" /* ST(0) = 5+2 = 7 */
        "fmul %%st(1), %%st\n\t" /* ST(0) = 7*3 = 21 */
        "fstl %0\n\t"            /* FST (non-popping): store 21 to *result */
        /* Break */
        "movl $1, %3\n\t"
        /* Run 3: verify remaining values survived */
        "fstp %%st(0)\n\t" /* pop 21 → ST(0)=3, ST(1)=2 */
        "fstpl %1\n\t"     /* r1 = 3, pop → ST(0)=2 */
        "fstpl %2\n\t"     /* r2 = 2, pop */
        : "=m"(*result), "=m"(*r1), "=m"(*r2), "+m"(dummy)
        : "m"(a), "m"(b), "m"(c) /* %4=a, %5=b, %6=c */
    );
}

/*
 * Net-pop variant: FSTP + FADD + FST on pre-existing values.
 *
 * Run 2: FSTP ST(0) + FADD ST(0),ST(1) + FST [result]
 *   FSTP pops: top_delta = 1
 *   FADD: resolve(0) → ReadSt(1), resolve(1) → ReadSt(2).
 *     slot_val[0] = FAdd, slot_val[1] = ReadSt(2)
 *   FST: store.
 *
 *   Epilogue check (top_delta = 1):
 *     d=0: FAdd → store (needed)
 *     d=1: ReadSt(2), initial_depth=2. 2 == 1+1 → YES. Elide.
 */
static void test_pop_then_arith(double* result, double* r1) {
    volatile double a = 2.0, b = 3.0, c = 5.0;
    int dummy;
    __asm__ volatile(
        /* Run 1: push 3 values */
        "fldl %3\n\t" /* ST(0)=a=2 */
        "fldl %4\n\t" /* ST(0)=b=3, ST(1)=2 */
        "fldl %5\n\t" /* ST(0)=c=5, ST(1)=3, ST(2)=2 */
        /* Break */
        "movl $0, %2\n\t"
        /* Run 2: pop one, then arithmetic on remaining */
        "fstp %%st(0)\n\t"       /* pop c=5 → ST(0)=3, ST(1)=2 */
        "fadd %%st(1), %%st\n\t" /* ST(0) = 3+2 = 5 */
        "fstl %0\n\t"            /* FST (non-popping): store 5 */
        /* Break */
        "movl $1, %2\n\t"
        /* Run 3: verify */
        "fstp %%st(0)\n\t" /* pop 5 → ST(0)=2 */
        "fstpl %1\n\t"     /* r1 = 2, pop */
        : "=m"(*result), "=m"(*r1), "+m"(dummy)
        : "m"(a), "m"(b), "m"(c));
}

int main(void) {
    printf("=== ReadSt write-back elision tests ===\n\n");

    {
        double result, r1, r2;
        test_elision_3deep(&result, &r1, &r2);
        check("3-deep: result = (5+2)*3 = 21", result, 21.0);
        check("3-deep: remaining ST(1) = 3 (b, unchanged)", r1, 3.0);
        check("3-deep: remaining ST(2) = 2 (a, unchanged)", r2, 2.0);
    }

    printf("\n");

    {
        double result, r1;
        test_pop_then_arith(&result, &r1);
        check("pop+arith: result = 3+2 = 5", result, 5.0);
        check("pop+arith: remaining ST(1) = 2 (a, unchanged)", r1, 2.0);
    }

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
