/*
 * test_decoder_fcomp_st.c — the non-canonical `DC D8` encoding of FCOMP ST(0).
 *
 * FCOMP ST(0) has two encodings that real x87 silicon accepts:
 *
 *     D8 D8    canonical, what every assembler emits
 *     DC D8    alternate, an undocumented alias
 *
 * The `DC D0..DF` row is normally the memory / reversed-operand form, so the
 * register form is undocumented. Rosetta's decoder rejects it: decode_opcode
 * returns INVALID and the instruction raises an illegal-instruction trap
 * instead of translating. Shipping software does emit it (WoW 1.12 has one at
 * .text:006FA876, `fld qword [edi+8]; fcomp st; fnstsw ax`, a NaN check), which
 * is why wine users have needed winerosetta.dll injected into the guest to fix
 * it up from a vectored exception handler.
 *
 * x87sidecar handles it in the decode_opcode hook instead, so this test is
 * expected to trap under stock Rosetta and to pass under the sidecar. Phase 12
 * of scripts/run_tests.sh asserts exactly that, in both directions, plus the
 * X87_NO_DECODE_HOOK=1 kill switch.
 *
 * No assembler will emit `DC D8`, so the bytes are injected with `.byte`.
 *
 * Result convention (as in the other x87 tests): the masked status-word CC bits
 *
 *   result & 0x4500      bit 14 = C3, bit 10 = C2, bit 8 = C0
 *
 *   equal        0x4000
 *   unordered    0x4500
 *
 * Build: clang -arch x86_64 -O2 -o test_decoder_fcomp_st test_decoder_fcomp_st.c
 */

#include <setjmp.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * SIGILL guard. Where `DC D8` is not decoded the trap would otherwise kill the
 * process with no output, and the harness could not tell "the alias is
 * unsupported here" from "the binary is broken". Catch it, unwind back to the
 * per-case recovery point, and report the case as failed.
 * --------------------------------------------------------------------------- */
static sigjmp_buf g_ill_env;
static volatile sig_atomic_t g_ill_hit = 0;

static void on_sigill(int sig) {
    (void)sig;
    g_ill_hit = 1;
    siglongjmp(g_ill_env, 1);
}

static void install_sigill_guard(void) {
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_sigill;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGILL, &sa, NULL);
}

/* Read the x87 status-word CC bits after a compare. */
#define READ_SW(var)              \
    uint16_t var;                 \
    __asm__ volatile("fnstsw %%ax\n" \
                     "andw $0x4500, %%ax\n" \
                     "movw %%ax, %0\n"      \
                     : "=m"(var)            \
                     :                      \
                     : "ax")

/* FCOMP ST(0) via DC D8. Comparing a finite value with itself is equal (C3=1),
 * and the single pop leaves the stack empty, so no teardown is needed. */
static uint16_t fcomp_st0_dcd8_eq(void) {
    double st0 = 2.0;
    __asm__ volatile("fldl %0\n"
                     ".byte 0xDC, 0xD8\n"
                     :
                     : "m"(st0));
    READ_SW(cc);
    return cc;
}

/* Same encoding, NaN against itself: unordered. */
static uint16_t fcomp_st0_dcd8_un(void) {
    double nan_val = __builtin_nan("");
    __asm__ volatile("fldl %0\n"
                     ".byte 0xDC, 0xD8\n"
                     :
                     : "m"(nan_val));
    READ_SW(cc);
    return cc;
}

/* Control: the canonical D8 D8 encoding of the same instruction. It must give
 * the same answer, and it must work everywhere. If this one fails, the
 * expectations above are wrong rather than the decoder. */
static uint16_t fcomp_st0_d8d8_eq(void) {
    double st0 = 2.0;
    __asm__ volatile("fldl %0\n"
                     ".byte 0xD8, 0xD8\n"
                     :
                     : "m"(st0));
    READ_SW(cc);
    return cc;
}

typedef struct {
    const char* name;
    uint16_t (*fn)(void);
    uint16_t expected;
} TestCase;

static const TestCase g_tests[] = {
    {"fcomp ST(0)  D8 D8  EQ   2.0,2.0  (control)", fcomp_st0_d8d8_eq, 0x4000},
    {"fcomp ST(0)  DC D8  EQ   2.0,2.0", fcomp_st0_dcd8_eq, 0x4000},
    {"fcomp ST(0)  DC D8  UN   NaN,NaN", fcomp_st0_dcd8_un, 0x4500},
};

static volatile int g_case = 0;
static volatile uint16_t g_got = 0;

int main(void) {
    install_sigill_guard();

    const int n = (int)(sizeof(g_tests) / sizeof(g_tests[0]));
    int pass = 0, fail = 0;

    for (g_case = 0; g_case < n; g_case++) {
        const TestCase* tc = &g_tests[g_case];
        /* Each case starts from a clean x87 stack: a trapped case leaves the
         * value its fldl pushed behind. */
        __asm__ volatile("fninit");
        g_ill_hit = 0;
        g_got = 0;

        if (sigsetjmp(g_ill_env, 1) == 0) {
            g_got = tc->fn();
        }

        if (g_ill_hit) {
            printf("FAIL  %-44s  SIGILL: encoding not decoded here\n", tc->name);
            fail++;
            continue;
        }
        if (g_got != tc->expected) {
            printf("FAIL  %-44s  got=0x%04x  expected=0x%04x\n", tc->name, (unsigned)g_got,
                   (unsigned)tc->expected);
            fail++;
            continue;
        }
        printf("PASS  %s\n", tc->name);
        pass++;
    }

    printf("\n%d/%d passed\n", pass, n);
    return fail ? 1 : 0;
}
