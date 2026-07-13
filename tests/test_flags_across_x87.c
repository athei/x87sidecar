/*
 * test_flags_across_x87.c — guest EFLAGS survival across x87 ops.
 *
 * Stock Rosetta keeps materialized guest flags in NZCV across x87 ops
 * whose own stock lowering preserves them; a `cmp; <x87 op>; setcc`
 * sequence must observe the cmp's flags.  Our inline transcendental
 * emitters use FCMP/FCSEL/TST/SUBS internally, which clobber NZCV —
 * they save/restore it (or use non-flag-setting forms).  This test
 * covers every x87 op class: transcendentals (the ones that clobbered),
 * plus the fcom/ftst/fistp/frndint families (whose stock lowerings also
 * clobber NZCV, so stock spills around them and no preservation is
 * needed on our side) as canaries.
 *
 * Pattern per probe: cmpl a,b ; <x87 sequence> ; sete/setb.  Wants:
 * (5,5) -> eq=1 lt=0; (3,9) -> eq=0 lt=1; (9,3) -> eq=0 lt=0.
 */
#include <stdio.h>

static volatile double vx = 0.5, vy = 1.5;
static double r1, r2; static short sw; static int i1;

static int probe_fsin(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t" "fldl %[x]\n\tfsin\n\tfstpl %[r]\n\t" "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [r] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [x] "m"(vx)
        : "cc", "st");
    return (eq << 1) | lt;
}
static int probe_fcos(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t" "fldl %[x]\n\tfcos\n\tfstpl %[r]\n\t" "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [r] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [x] "m"(vx)
        : "cc", "st");
    return (eq << 1) | lt;
}
static int probe_fsincos(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t" "fldl %[x]\n\tfsincos\n\tfstpl %[r]\n\tfstpl %[q]\n\t" "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [r] "=m"(r1), [q] "=m"(r2), [dummy] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [x] "m"(vx)
        : "cc", "st", "st(1)");
    return (eq << 1) | lt;
}
static int probe_fptan(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t" "fldl %[x]\n\tfptan\n\tfstpl %[r]\n\tfstpl %[q]\n\t" "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [r] "=m"(r1), [q] "=m"(r2), [dummy] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [x] "m"(vx)
        : "cc", "st", "st(1)");
    return (eq << 1) | lt;
}
static int probe_f2xm1(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t" "fldl %[x]\n\tf2xm1\n\tfstpl %[r]\n\t" "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [r] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [x] "m"(vx)
        : "cc", "st");
    return (eq << 1) | lt;
}
static int probe_fpatan(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t" "fldl %[y]\n\tfldl %[x]\n\tfpatan\n\tfstpl %[r]\n\t" "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [r] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [y] "m"(vy), [x] "m"(vx)
        : "cc", "st", "st(1)");
    return (eq << 1) | lt;
}
static int probe_fyl2x(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t" "fldl %[y]\n\tfldl %[x]\n\tfyl2x\n\tfstpl %[r]\n\t" "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [r] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [y] "m"(vy), [x] "m"(vx)
        : "cc", "st", "st(1)");
    return (eq << 1) | lt;
}
static int probe_fyl2xp1(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t" "fldl %[y]\n\tfldl %[x]\n\tfyl2xp1\n\tfstpl %[r]\n\t" "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [r] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [y] "m"(vy), [x] "m"(vx)
        : "cc", "st", "st(1)");
    return (eq << 1) | lt;
}
static int probe_fprem(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t" "fldl %[y]\n\tfldl %[x]\n\tfprem\n\tfstpl %[r]\n\tfstpl %[q]\n\t" "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [r] "=m"(r1), [q] "=m"(r2), [dummy] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [y] "m"(vy), [x] "m"(vx)
        : "cc", "st", "st(1)");
    return (eq << 1) | lt;
}
static int probe_fscale(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t" "fldl %[y]\n\tfldl %[x]\n\tfscale\n\tfstpl %[r]\n\tfstpl %[q]\n\t" "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [r] "=m"(r1), [q] "=m"(r2), [dummy] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [y] "m"(vy), [x] "m"(vx)
        : "cc", "st", "st(1)");
    return (eq << 1) | lt;
}
static int probe_fcom(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t" "fldl %[y]\n\tfldl %[x]\n\tfcompp\n\t" "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [dummy] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [y] "m"(vy), [x] "m"(vx)
        : "cc", "st", "st(1)");
    return (eq << 1) | lt;
}
static int probe_fcom_sw(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t" "fldl %[y]\n\tfldl %[x]\n\tfcompp\n\tfnstsw %[sw]\n\t" "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [sw] "=m"(sw), [dummy] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [y] "m"(vy), [x] "m"(vx)
        : "cc", "st", "st(1)");
    return (eq << 1) | lt;
}
static int probe_ftst(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t" "fldl %[x]\n\tftst\n\tfstpl %[r]\n\t" "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [r] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [x] "m"(vx)
        : "cc", "st");
    return (eq << 1) | lt;
}
static int probe_fistp(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t" "fldl %[x]\n\tfistpl %[i]\n\t" "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [i] "=m"(i1), [dummy] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [x] "m"(vx)
        : "cc", "st");
    return (eq << 1) | lt;
}
static int probe_frndint(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t" "fldl %[x]\n\tfrndint\n\tfstpl %[r]\n\t" "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [r] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [x] "m"(vx)
        : "cc", "st");
    return (eq << 1) | lt;
}
static int probe_fsqrt(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t" "fldl %[x]\n\tfsqrt\n\tfstpl %[r]\n\t" "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [r] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [x] "m"(vx)
        : "cc", "st");
    return (eq << 1) | lt;
}
static int probe_fadd(unsigned a, unsigned b) {
    unsigned char eq, lt;
    __asm__ volatile("cmpl %[b], %[a]\n\t" "fldl %[x]\n\tfldl %[y]\n\tfaddp\n\tfstpl %[r]\n\t" "sete %[eq]\n\tsetb %[lt]\n\t"
        : [eq] "=&r"(eq), [lt] "=&r"(lt), [r] "=m"(r1)
        : [a] "r"(a), [b] "r"(b), [x] "m"(vx), [y] "m"(vy)
        : "cc", "st", "st(1)");
    return (eq << 1) | lt;
}
int main(void) { int fails = 0;
    struct { unsigned a, b; int want; } cases[] = {{5,5,2},{3,9,1},{9,3,0}};
    for (int i = 0; i < 3; i++) {
        int g = probe_fsin(cases[i].a, cases[i].b);
        if (g != cases[i].want) { printf("FAIL %-8s case %d: got %d want %d\n", "fsin", i, g, cases[i].want); fails++; break; }
    }
    for (int i = 0; i < 3; i++) {
        int g = probe_fcos(cases[i].a, cases[i].b);
        if (g != cases[i].want) { printf("FAIL %-8s case %d: got %d want %d\n", "fcos", i, g, cases[i].want); fails++; break; }
    }
    for (int i = 0; i < 3; i++) {
        int g = probe_fsincos(cases[i].a, cases[i].b);
        if (g != cases[i].want) { printf("FAIL %-8s case %d: got %d want %d\n", "fsincos", i, g, cases[i].want); fails++; break; }
    }
    for (int i = 0; i < 3; i++) {
        int g = probe_fptan(cases[i].a, cases[i].b);
        if (g != cases[i].want) { printf("FAIL %-8s case %d: got %d want %d\n", "fptan", i, g, cases[i].want); fails++; break; }
    }
    for (int i = 0; i < 3; i++) {
        int g = probe_f2xm1(cases[i].a, cases[i].b);
        if (g != cases[i].want) { printf("FAIL %-8s case %d: got %d want %d\n", "f2xm1", i, g, cases[i].want); fails++; break; }
    }
    for (int i = 0; i < 3; i++) {
        int g = probe_fpatan(cases[i].a, cases[i].b);
        if (g != cases[i].want) { printf("FAIL %-8s case %d: got %d want %d\n", "fpatan", i, g, cases[i].want); fails++; break; }
    }
    for (int i = 0; i < 3; i++) {
        int g = probe_fyl2x(cases[i].a, cases[i].b);
        if (g != cases[i].want) { printf("FAIL %-8s case %d: got %d want %d\n", "fyl2x", i, g, cases[i].want); fails++; break; }
    }
    for (int i = 0; i < 3; i++) {
        int g = probe_fyl2xp1(cases[i].a, cases[i].b);
        if (g != cases[i].want) { printf("FAIL %-8s case %d: got %d want %d\n", "fyl2xp1", i, g, cases[i].want); fails++; break; }
    }
    for (int i = 0; i < 3; i++) {
        int g = probe_fprem(cases[i].a, cases[i].b);
        if (g != cases[i].want) { printf("FAIL %-8s case %d: got %d want %d\n", "fprem", i, g, cases[i].want); fails++; break; }
    }
    for (int i = 0; i < 3; i++) {
        int g = probe_fscale(cases[i].a, cases[i].b);
        if (g != cases[i].want) { printf("FAIL %-8s case %d: got %d want %d\n", "fscale", i, g, cases[i].want); fails++; break; }
    }
    for (int i = 0; i < 3; i++) {
        int g = probe_fcom(cases[i].a, cases[i].b);
        if (g != cases[i].want) { printf("FAIL %-8s case %d: got %d want %d\n", "fcom", i, g, cases[i].want); fails++; break; }
    }
    for (int i = 0; i < 3; i++) {
        int g = probe_fcom_sw(cases[i].a, cases[i].b);
        if (g != cases[i].want) { printf("FAIL %-8s case %d: got %d want %d\n", "fcom_sw", i, g, cases[i].want); fails++; break; }
    }
    for (int i = 0; i < 3; i++) {
        int g = probe_ftst(cases[i].a, cases[i].b);
        if (g != cases[i].want) { printf("FAIL %-8s case %d: got %d want %d\n", "ftst", i, g, cases[i].want); fails++; break; }
    }
    for (int i = 0; i < 3; i++) {
        int g = probe_fistp(cases[i].a, cases[i].b);
        if (g != cases[i].want) { printf("FAIL %-8s case %d: got %d want %d\n", "fistp", i, g, cases[i].want); fails++; break; }
    }
    for (int i = 0; i < 3; i++) {
        int g = probe_frndint(cases[i].a, cases[i].b);
        if (g != cases[i].want) { printf("FAIL %-8s case %d: got %d want %d\n", "frndint", i, g, cases[i].want); fails++; break; }
    }
    for (int i = 0; i < 3; i++) {
        int g = probe_fsqrt(cases[i].a, cases[i].b);
        if (g != cases[i].want) { printf("FAIL %-8s case %d: got %d want %d\n", "fsqrt", i, g, cases[i].want); fails++; break; }
    }
    for (int i = 0; i < 3; i++) {
        int g = probe_fadd(cases[i].a, cases[i].b);
        if (g != cases[i].want) { printf("FAIL %-8s case %d: got %d want %d\n", "fadd", i, g, cases[i].want); fails++; break; }
    }
    printf(fails ? "FAIL flags-across-x87 (%d ops)\n" : "PASS flags-across-x87\n", fails);
    return fails != 0;
}