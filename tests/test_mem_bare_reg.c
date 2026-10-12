// test_mem_bare_reg.c -- x87 memory operands that are a bare 64-bit register
//
// For a 64-bit "(%reg)" operand with no displacement, or an index-only
// "(,%reg,1)" operand, the address computation hands back the guest
// register itself instead of a scratch copy.  An emitter that then loads
// through that register into itself, or bumps it in place to walk a
// buffer, overwrites the guest register.  Every case here checks both the
// x87 result and that the address register still holds its value.
//
// Each operation runs twice: isolated between integer instructions, so it
// is translated on its own, and inside a run of x87 instructions, so it is
// translated with the run's register cache live.
//
// Compile:
//   clang -arch x86_64 -O2 -o test_mem_bare_reg test_mem_bare_reg.c

#include <stdint.h>
#include <stdio.h>
#include <string.h>

// A flag-producing integer instruction ends an x87 run and is never
// folded into one.
#define SEP "cmpq %%r11, %%r11\n\t"

// ── FIADD / FISUB / FISUBR / FIMUL / FIDIV / FIDIVR ────────────────────────
//
// ST(0) op= int at (%rdi).  Returns 1 when %rdi survived.

#define FI_ARITH(name, insn, itype)                                                      \
    static int name##_alone(double st0, itype m, double* out) {                           \
        uintptr_t p = (uintptr_t)&m;                                                     \
        const uintptr_t p0 = p;                                                          \
        __asm__ volatile("fninit\n\t" SEP "fldl %[st]\n\t" SEP insn " (%%rdi)\n\t" SEP     \
                         "fstpl %[out]\n\t"                                              \
                         : [out] "=m"(*out), "+D"(p)                                     \
                         : [st] "m"(st0), "m"(m)                                         \
                         : "r11", "cc", "memory");                                       \
        return p == p0;                                                                  \
    }                                                                                    \
    static int name##_run(double st0, itype m, double* out) {                             \
        uintptr_t p = (uintptr_t)&m;                                                     \
        const uintptr_t p0 = p;                                                          \
        __asm__ volatile("fninit\n\t" SEP "fldl %[st]\n\t" insn " (%%rdi)\n\t"            \
                         "fstpl %[out]\n\t"                                              \
                         : [out] "=m"(*out), "+D"(p)                                     \
                         : [st] "m"(st0), "m"(m)                                         \
                         : "r11", "cc", "memory");                                       \
        return p == p0;                                                                  \
    }

FI_ARITH(fiadds, "fiadds", int16_t)
FI_ARITH(fiaddl, "fiaddl", int32_t)
FI_ARITH(fisubs, "fisubs", int16_t)
FI_ARITH(fisubl, "fisubl", int32_t)
FI_ARITH(fisubrs, "fisubrs", int16_t)
FI_ARITH(fisubrl, "fisubrl", int32_t)
FI_ARITH(fimuls, "fimuls", int16_t)
FI_ARITH(fimull, "fimull", int32_t)
FI_ARITH(fidivs, "fidivs", int16_t)
FI_ARITH(fidivl, "fidivl", int32_t)
FI_ARITH(fidivrs, "fidivrs", int16_t)
FI_ARITH(fidivrl, "fidivrl", int32_t)

// FIADD through other bare base registers and through an index-only operand.
#define FIADD_REG(name, operand, constraint)                                             \
    static int name(double st0, int32_t m, double* out) {                                 \
        uintptr_t p = (uintptr_t)&m;                                                     \
        const uintptr_t p0 = p;                                                          \
        __asm__ volatile("fninit\n\t" SEP "fldl %[st]\n\t" SEP "fiaddl " operand "\n\t"    \
                         SEP "fstpl %[out]\n\t"                                          \
                         : [out] "=m"(*out), constraint(p)                               \
                         : [st] "m"(st0), "m"(m)                                         \
                         : "r11", "cc", "memory");                                       \
        return p == p0;                                                                  \
    }

FIADD_REG(fiaddl_rsi, "(%%rsi)", "+S")
FIADD_REG(fiaddl_rbx, "(%%rbx)", "+b")
FIADD_REG(fiaddl_rcx, "(%%rcx)", "+c")
FIADD_REG(fiaddl_index, "(,%%rdx,1)", "+d")

// ── FICOM / FICOMP ─────────────────────────────────────────────────────────
//
// Returns the status word read straight after the compare.  FICOMP pops
// the only value, so TOP is back at 0 in that status word.

#define FI_COM(name, insn, itype, pop)                                                   \
    static int name##_alone(double st0, itype m, uint16_t* sw) {                          \
        uintptr_t p = (uintptr_t)&m;                                                     \
        const uintptr_t p0 = p;                                                          \
        uint16_t ax;                                                                     \
        __asm__ volatile("fninit\n\t" SEP "fldl %[st]\n\t" SEP insn " (%%rdi)\n\t" SEP     \
                         "fnstsw %%ax\n\t" pop                                           \
                         : "=a"(ax), "+D"(p)                                             \
                         : [st] "m"(st0), "m"(m)                                         \
                         : "r11", "cc", "memory");                                       \
        *sw = ax;                                                                        \
        return p == p0;                                                                  \
    }                                                                                    \
    static int name##_run(double st0, itype m, uint16_t* sw) {                            \
        uintptr_t p = (uintptr_t)&m;                                                     \
        const uintptr_t p0 = p;                                                          \
        uint16_t ax;                                                                     \
        __asm__ volatile("fninit\n\t" SEP "fldl %[st]\n\t" insn " (%%rdi)\n\t"            \
                         "fnstsw %%ax\n\t" pop                                           \
                         : "=a"(ax), "+D"(p)                                             \
                         : [st] "m"(st0), "m"(m)                                         \
                         : "r11", "cc", "memory");                                       \
        *sw = ax;                                                                        \
        return p == p0;                                                                  \
    }

FI_COM(ficoms, "ficoms", int16_t, "fstp %%st(0)\n\t")
FI_COM(ficoml, "ficoml", int32_t, "fstp %%st(0)\n\t")
FI_COM(ficomps, "ficomps", int16_t, "")
FI_COM(ficompl, "ficompl", int32_t, "")

// ── FNSAVE / FRSTOR ────────────────────────────────────────────────────────
//
// Both walk the 108-byte image slot by slot.  Save a two-deep stack,
// restore it, and read it back.

static unsigned char g_image[128] __attribute__((aligned(16)));

static int fsave_frstor_alone(double a, double b, double out[2]) {
    uintptr_t p = (uintptr_t)g_image;
    const uintptr_t p0 = p;
    __asm__ volatile("fninit\n\t" SEP "fldl %[a]\n\t" "fldl %[b]\n\t" SEP
                     "fnsave (%%rdi)\n\t" SEP "frstor (%%rdi)\n\t" SEP
                     "fstpl %[o0]\n\t" "fstpl %[o1]\n\t"
                     : [o0] "=m"(out[0]), [o1] "=m"(out[1]), "+D"(p)
                     : [a] "m"(a), [b] "m"(b)
                     : "r11", "cc", "memory");
    return p == p0;
}

static int fsave_frstor_run(double a, double b, double out[2]) {
    uintptr_t p = (uintptr_t)g_image;
    const uintptr_t p0 = p;
    __asm__ volatile("fninit\n\t" SEP "fldl %[a]\n\t" "fldl %[b]\n\t"
                     "fnsave (%%rdi)\n\t" "frstor (%%rdi)\n\t"
                     "fstpl %[o0]\n\t" "fstpl %[o1]\n\t"
                     : [o0] "=m"(out[0]), [o1] "=m"(out[1]), "+D"(p)
                     : [a] "m"(a), [b] "m"(b)
                     : "r11", "cc", "memory");
    return p == p0;
}

// ---------------------------------------------------------------------------

static int g_pass;
static int g_fail;

static void report(const char* name, int ok) {
    printf("%-34s %s\n", name, ok ? "PASS" : "FAIL");
    if (ok) {
        g_pass++;
    } else {
        g_fail++;
    }
}

static int same(double got, double want) {
    return memcmp(&got, &want, sizeof got) == 0;
}

#define CHECK_ARITH16(name, m, want)                                                     \
    do {                                                                                 \
        double out = -1.0;                                                               \
        int kept = name##_alone(12.0, (int16_t)(m), &out);                               \
        report(#name " alone", kept && same(out, (want)));                               \
        out = -1.0;                                                                      \
        kept = name##_run(12.0, (int16_t)(m), &out);                                     \
        report(#name " run", kept && same(out, (want)));                                 \
    } while (0)

#define CHECK_ARITH32(name, m, want)                                                     \
    do {                                                                                 \
        double out = -1.0;                                                               \
        int kept = name##_alone(12.0, (int32_t)(m), &out);                               \
        report(#name " alone", kept && same(out, (want)));                               \
        out = -1.0;                                                                      \
        kept = name##_run(12.0, (int32_t)(m), &out);                                     \
        report(#name " run", kept && same(out, (want)));                                 \
    } while (0)

// sw & 0x4700: C3 C2 C1 C0.  12 > 3 sets none; 12 == 12 sets C3.
#define CHECK_COM(name, itype, m, want_cc, want_top)                                     \
    do {                                                                                 \
        uint16_t sw = 0xFFFF;                                                            \
        int kept = name##_alone(12.0, (itype)(m), &sw);                                  \
        report(#name " alone", kept && (sw & 0x4500) == (want_cc) &&                     \
                                   ((sw >> 11) & 7) == (want_top));                      \
        sw = 0xFFFF;                                                                     \
        kept = name##_run(12.0, (itype)(m), &sw);                                        \
        report(#name " run", kept && (sw & 0x4500) == (want_cc) &&                       \
                                 ((sw >> 11) & 7) == (want_top));                        \
    } while (0)

int main(void) {
    // 12 op 3, all exact in binary64.
    CHECK_ARITH16(fiadds, 3, 15.0);
    CHECK_ARITH32(fiaddl, 3, 15.0);
    CHECK_ARITH16(fisubs, 3, 9.0);
    CHECK_ARITH32(fisubl, 3, 9.0);
    CHECK_ARITH16(fisubrs, 3, -9.0);
    CHECK_ARITH32(fisubrl, 3, -9.0);
    CHECK_ARITH16(fimuls, -3, -36.0);
    CHECK_ARITH32(fimull, -3, -36.0);
    CHECK_ARITH16(fidivs, 3, 4.0);
    CHECK_ARITH32(fidivl, 3, 4.0);
    CHECK_ARITH16(fidivrs, 3, 0.25);
    CHECK_ARITH32(fidivrl, 3, 0.25);

    {
        double out = -1.0;
        report("fiaddl (%rsi)", fiaddl_rsi(12.0, 5, &out) && same(out, 17.0));
        out = -1.0;
        report("fiaddl (%rbx)", fiaddl_rbx(12.0, 5, &out) && same(out, 17.0));
        out = -1.0;
        report("fiaddl (%rcx)", fiaddl_rcx(12.0, 5, &out) && same(out, 17.0));
        out = -1.0;
        report("fiaddl (,%rdx,1)", fiaddl_index(12.0, 5, &out) && same(out, 17.0));
    }

    // FICOM leaves the value on the stack (TOP = 7); FICOMP pops it (TOP = 0).
    CHECK_COM(ficoms, int16_t, 3, 0x0000, 7);
    CHECK_COM(ficoml, int32_t, 12, 0x4000, 7);
    CHECK_COM(ficomps, int16_t, 12, 0x4000, 0);
    CHECK_COM(ficompl, int32_t, 3, 0x0000, 0);

    {
        double out[2] = {0.0, 0.0};
        int kept = fsave_frstor_alone(1.5, -2.25, out);
        report("fnsave/frstor alone", kept && same(out[0], -2.25) && same(out[1], 1.5));
        out[0] = out[1] = 0.0;
        kept = fsave_frstor_run(1.5, -2.25, out);
        report("fnsave/frstor run", kept && same(out[0], -2.25) && same(out[1], 1.5));
    }

    printf("\n%d/%d passed\n", g_pass, g_pass + g_fail);
    return g_fail ? 1 : 0;
}
