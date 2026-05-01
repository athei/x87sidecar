/*
 * bench_fusion_arithp_fld.c -- Benchmark for arithp_fld fusion.
 * Pattern: ARITHp ST(1) + FLD src (pop+push cancel).
 *
 * IMPORTANT: For arithp_fld to fire, the faddp must NOT be preceded by a
 * fuseable fld (otherwise fld_arithp catches it first).  These benchmarks
 * use patterns where a non-popping arithmetic or other instruction precedes
 * the faddp, ensuring arithp_fld is the fusion that matches.
 *
 * NOTE: GAS AT&T: fsubp=Intel FSUBRP, fdivp=Intel FDIVRP
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 1000000
#define RUNS 5

/*
 * Non-popping multiply then faddp then fld:
 *   fld [a] | fld [b] | fmul st(0),st(1) | faddp | fld [c]
 *
 * The fmul is non-popping so it doesn't fuse with faddp via fld_arithp.
 * The faddp+fld pair is our target.
 */
static bench_ns_t bench_mul_accum_load(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 3.0, b = 4.0, c = 5.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %1\n\t"            /* ST(0) = a */
            "fldl  %2\n\t"            /* ST(0) = b, ST(1) = a */
            "fmul  %%st(1), %%st\n\t" /* ST(1) = a*b, ST(0) = b (non-popping) */
            "faddp\n\t"               /* ST(0) = b + a*b       ← arithp (NOT preceded by fld) */
            "fldl  %3\n\t"            /* ST(0) = c, ST(1) = result ← fld */
            "faddp\n\t"               /* ST(0) = result + c */
            "fstpl %0\n"
            : "=m"(r)
            : "m"(a), "m"(b), "m"(c));
    return bench_now_ns() - start;
}

/*
 * Dot product where partial sums use non-popping fadd:
 *   fld [x0] | fmul [y0] | fld [x1] | fmul [y1] | fadd st(1),st | faddp | fld [x2]
 *
 * The fadd st(1),st writes to st(1) without popping; the faddp then
 * folds and pops.  faddp is preceded by fadd (not fld), so arithp_fld fires.
 */
static bench_ns_t bench_fadd_faddp_fld(void) {
    bench_ns_t start = bench_now_ns();
    volatile double x0 = 1.0, y0 = 2.0;
    volatile double x1 = 3.0, y1 = 4.0;
    volatile double x2 = 5.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %1\n\t"            /* x0 */
            "fmull %2\n\t"            /* x0*y0 */
            "fldl  %3\n\t"            /* x1 */
            "fmull %4\n\t"            /* x1*y1, x0*y0 */
            "fadd  %%st, %%st(1)\n\t" /* ST(1) += ST(0) = x0y0+x1y1, ST(0) = x1y1 */
            "faddp\n\t"               /* ST(0) = x1y1 + (x0y0+x1y1) ← arithp */
            "fldl  %5\n\t"            /* x2, result               ← fld    */
            "faddp\n\t"
            "fstpl %0\n"
            : "=m"(r)
            : "m"(x0), "m"(y0), "m"(x1), "m"(y1), "m"(x2));
    return bench_now_ns() - start;
}

/*
 * Chain of non-popping fsub then fsubp then fld:
 *   fld [a] | fld [b] | fsub st(0),st(1) | fsubp | fld [c]
 *
 * fsubp preceded by fsub (not fld), so arithp_fld fires.
 */
static bench_ns_t bench_sub_fsubp_fld(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 10.0, b = 3.0, c = 7.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %1\n\t"            /* a */
            "fldl  %2\n\t"            /* b, a */
            "fsub  %%st(1), %%st\n\t" /* ST(0) = b-a, ST(1) = a */
            "fsubp\n\t"               /* ST(0) = (b-a) - a       ← arithp */
            "fldl  %3\n\t"            /* c, result               ← fld    */
            "faddp\n\t"
            "fstpl %0\n"
            : "=m"(r)
            : "m"(a), "m"(b), "m"(c));
    return bench_now_ns() - start;
}

/*
 * Repeated fmulp+fld chain (4 pairs).
 * Each fmulp is preceded by fmul (non-popping), not fld.
 *
 * Pattern: fld a | fld b | fmul st(0),st(1) | fmulp | fld c | fmul st(0),st(1) | fmulp | fld d |
 * ...
 */
static bench_ns_t bench_mulp_fld_chain(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 1.1, b = 1.2, c = 1.3, d = 1.4;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %1\n\t"            /* a */
            "fldl  %2\n\t"            /* b, a */
            "fmul  %%st(1), %%st\n\t" /* ST(1) = a*b, ST(0) = b */
            "fmulp\n\t"               /* ST(0) = b * (a*b)       ← arithp */
            "fldl  %3\n\t"            /* c, result               ← fld    */
            "fmul  %%st(1), %%st\n\t" /* ST(1) *= ST(0) */
            "fmulp\n\t"               /* ← arithp */
            "fldl  %4\n\t"            /* d, result               ← fld    */
            "faddp\n\t"
            "fstpl %0\n"
            : "=m"(r)
            : "m"(a), "m"(b), "m"(c), "m"(d));
    return bench_now_ns() - start;
}

/*
 * faddp + fld ST(0): arithp followed by register load of the result.
 * Tests the emit_fmov_f64_reg path in the fusion.
 */
static bench_ns_t bench_faddp_fld_st0(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 3.0, b = 4.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %1\n\t"            /* a */
            "fldl  %2\n\t"            /* b, a */
            "fadd  %%st, %%st(1)\n\t" /* ST(1) = a+b, ST(0) = b */
            "faddp\n\t"               /* ST(0) = b + (a+b)       ← arithp */
            "fld   %%st(0)\n\t"       /* dup result              ← fld ST(0) */
            "faddp\n\t"               /* ST(0) = 2 * result */
            "fstpl %0\n"
            : "=m"(r)
            : "m"(a), "m"(b));
    return bench_now_ns() - start;
}

/*
 * Mixed: fmul mem (non-popping) | faddp | fld [next]
 * This is common in game engines: multiply-accumulate where the multiply
 * is memory-operand (not preceded by fld push).
 */
static bench_ns_t bench_fmul_mem_faddp_fld(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 2.0, b = 3.0, c = 4.0, d = 5.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %1\n\t" /* a */
            "fldl  %2\n\t" /* b, a */
            "fmull %3\n\t" /* b*c, a (non-popping mem multiply) */
            "faddp\n\t"    /* a + b*c           ← arithp */
            "fldl  %4\n\t" /* d, result         ← fld    */
            "fmulp\n\t"    /* d * result */
            "fstpl %0\n"
            : "=m"(r)
            : "m"(a), "m"(b), "m"(c), "m"(d));
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"mul_accum_load", bench_mul_accum_load}, {"fadd_faddp_fld", bench_fadd_faddp_fld},
        {"sub_fsubp_fld", bench_sub_fsubp_fld},   {"mulp_fld_chain", bench_mulp_fld_chain},
        {"faddp_fld_st0", bench_faddp_fld_st0},   {"fmul_mem_faddp_fld", bench_fmul_mem_faddp_fld},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++)
            sum += benches[i].fn();
        printf("BENCH %s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
