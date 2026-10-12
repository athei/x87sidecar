/*
 * test_ir_split_gs.c: integer stores to %gs: operands in a run that is
 * split for FPR pressure.
 *
 * Eight values are loaded and then stored with fistpl, so the indefinite
 * guard of the first store needs a ninth FPR and an eight-register pool
 * splits the run before the stores.  The suffix starts with Xbase, Wd_top
 * and Xst_base held by the run cache, and with two or more rounding stores
 * it pins the cached rounding control too.  A %gs: operand needs more
 * address registers than the IR GPR model charges for a memory operand
 * (the segment base, and the TLS base routine's fixed scratch slot), so
 * such an operand has to stay out of the IR run, or the lowering runs out
 * of GPRs in the suffix.  The fresh-run case is the same shape without the
 * split.
 *
 * The %gs: slots are saved and restored around each case.
 *
 * Build: clang -arch x86_64 -O0 -o test_ir_split_gs test_ir_split_gs.c
 */
#include <stdint.h>
#include <stdio.h>

static int failures = 0;

static void check(const char* name, int32_t got, int32_t expected) {
    if (got != expected) {
        printf("FAIL  %-48s  got=%d  expected=%d\n", name, got, expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

#define CW_NEAREST 0x037F

static void set_cw(uint16_t cw) {
    __asm__ volatile("fldcw %0" : : "m"(cw));
}

static void save_gs(uint64_t* saved) {
    __asm__ volatile(
        "movq %%gs:0x300, %%rax\n\t"
        "movq %%rax, (%0)\n\t"
        :
        : "r"(saved)
        : "memory", "rax");
}

static void restore_gs(const uint64_t* saved) {
    __asm__ volatile(
        "movq (%0), %%rax\n\t"
        "movq %%rax, %%gs:0x300\n\t"
        :
        : "r"(saved)
        : "memory", "rax");
}

/* Two rounding stores to %gs: in a fresh run.  out[0..1] = in[0..1]. */
static void gs_fresh(const double* v, int32_t* out) {
    __asm__ volatile(
        "fldl   (%1)\n\t"
        "fistpl %%gs:0x300\n\t"
        "fldl   8(%1)\n\t"
        "fistpl %%gs:0x304\n\t"
        "movl   %%gs:0x300, %%eax\n\t"
        "movl   %%eax, (%0)\n\t"
        "movl   %%gs:0x304, %%eax\n\t"
        "movl   %%eax, 4(%0)\n\t"
        :
        : "r"(out), "r"(v)
        : "memory", "eax", "st", "st(1)");
}

/* Eight loads, then the first two stores to %gs: and six to out[2..7].
 * out[0] = in[7], out[1] = in[6], out[2 + k] = in[5 - k]. */
static void gs_suffix(const double* v, int32_t* out) {
    __asm__ volatile(
        "fldl   (%1)\n\t"
        "fldl   8(%1)\n\t"
        "fldl   16(%1)\n\t"
        "fldl   24(%1)\n\t"
        "fldl   32(%1)\n\t"
        "fldl   40(%1)\n\t"
        "fldl   48(%1)\n\t"
        "fldl   56(%1)\n\t"
        "fistpl %%gs:0x300\n\t"
        "fistpl %%gs:0x304\n\t"
        "fistpl 8(%0)\n\t"
        "fistpl 12(%0)\n\t"
        "fistpl 16(%0)\n\t"
        "fistpl 20(%0)\n\t"
        "fistpl 24(%0)\n\t"
        "fistpl 28(%0)\n\t"
        "movl   %%gs:0x300, %%eax\n\t"
        "movl   %%eax, (%0)\n\t"
        "movl   %%gs:0x304, %%eax\n\t"
        "movl   %%eax, 4(%0)\n\t"
        :
        : "r"(out), "r"(v)
        : "memory", "eax", "st", "st(1)", "st(2)", "st(3)", "st(4)", "st(5)", "st(6)", "st(7)");
}

int main(void) {
    /* Round-to-nearest, ties to even. */
    const double v[8] = {1.5, 2.5, 3.0, -4.5, 5.0, 6.5, 7.0, 8.5};
    const int32_t r[8] = {2, 2, 3, -4, 5, 6, 7, 8};
    char name[64];
    uint64_t saved = 0;

    set_cw(CW_NEAREST);

    {
        int32_t out[8] = {0};
        save_gs(&saved);
        gs_suffix(v, out);
        restore_gs(&saved);
        check("suffix fistpl %gs:0x300  8.5", out[0], r[7]);
        check("suffix fistpl %gs:0x304  7", out[1], r[6]);
        for (int k = 0; k < 6; k++) {
            snprintf(name, sizeof(name), "suffix fistpl out[%d]  %g", 2 + k, v[5 - k]);
            check(name, out[2 + k], r[5 - k]);
        }
    }
    {
        int32_t out[2] = {0, 0};
        save_gs(&saved);
        gs_fresh(v, out);
        restore_gs(&saved);
        check("fresh  fistpl %gs:0x300  1.5", out[0], r[0]);
        check("fresh  fistpl %gs:0x304  2.5", out[1], r[1]);
    }

    printf("%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
