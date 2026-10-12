/*
 * test_flags_handoff.c -- EFLAGS handed between stock Rosetta and the sidecar.
 *
 * Stock Rosetta keeps the guest's EFLAGS in NZCV: SF in N, ZF in Z, CF
 * inverted in C, OF in V, and PF and AF in NZCV bits 26 and 27, which Apple
 * cores write on flag-setting instructions and which MRS and MSR NZCV read
 * and write.  A flag one translator sets is read by the other whenever an
 * x87 instruction and its neighbours are translated by different ones:
 * every SSE and integer instruction goes to stock, and X87_STOCK_OPS,
 * X87_STOCK_HASH_LIST and declined requests hand x87 instructions to stock
 * too.
 *
 * Section A: a flag setter stock translates (popf, ucomisd, sahf, test)
 *            followed by the sidecar's fcmov, in the same block and in the
 *            next one.  fcmovu and fcmovnu read PF; the others are
 *            controls.
 * Section B: fcomi, fucomi, fcomip and fucomip followed by fcmov in the
 *            next block.  run_tests.sh also runs this test with the
 *            compares and with the fcmovs handed to stock.
 * Section C: the four compares followed by the readers stock translates
 *            (setp, setnp, seto, sets, jp, cmovp, lahf, pushf), in the same
 *            block and in the next one.
 * Section D: every flag set by popf, then an x87 run, then pushf: the run
 *            must leave all six flags as it found them.
 *
 * Every case is checked against x86 semantics, so the native run is the
 * reference.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>

static int failures = 0;

#define F_CF 0x001
#define F_PF 0x004
#define F_AF 0x010
#define F_ZF 0x040
#define F_SF 0x080
#define F_OF 0x800
#define F_ALL (F_CF | F_PF | F_AF | F_ZF | F_SF | F_OF)

static void check_int(const char* name, long got, long want) {
    if (got != want) {
        printf("FAIL  %-64s got=%#lx want=%#lx\n", name, got, want);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

/* ── fcmov readers ──────────────────────────────────────────────────────── */

/* dst is loaded as ST(0) and src as ST(1); r receives ST(0) after the fcmov.
 * The probes return 1 when the fcmov moved the source. */
struct fio {
    double dst, src, r;
};

#define FCMOV_BODY(cc)   \
    "fldl 8(%[p])\n\t"   \
    "fldl (%[p])\n\t"    \
    "fcmov" #cc          \
    " %%st(1), %%st\n\t" \
    "fstpl 16(%[p])\n\t" \
    "fstp %%st(0)\n\t"

/* popf with the six arithmetic flags replaced by fl.  The pushes go below
 * the red zone, and LEA leaves the flags alone. */
#define SET_FLAGS                 \
    "leaq -128(%%rsp), %%rsp\n\t" \
    "pushfq\n\t"                  \
    "andq $-0x8d6, (%%rsp)\n\t"   \
    "orq %[fl], (%%rsp)\n\t"      \
    "popfq\n\t"                   \
    "leaq 128(%%rsp), %%rsp\n\t"

#define NEXT_BLOCK "jmp 1f\n1:\n\t"

#define SET_UCOMISD "ucomisd %[b], %[a]\n\t"

/* fucom; fnstsw ax; sahf: CF, PF and ZF from C0, C2 and C3. */
#define SET_SAHF        \
    "fldl 8(%[q])\n\t"  \
    "fldl (%[q])\n\t"   \
    "fucom %%st(1)\n\t" \
    "fnstsw %%ax\n\t"   \
    "fstp %%st(0)\n\t"  \
    "fstp %%st(0)\n\t"  \
    "sahf\n\t"

#define SET_TEST "testl %[v], %[v]\n\t"

#define CMP_FCOMI "fcomi %%st(1), %%st\n\t"
#define CMP_FUCOMI "fucomi %%st(1), %%st\n\t"
#define CMP_FCOMIP "fcomip %%st(1), %%st\n\t"
#define CMP_FUCOMIP "fucomip %%st(1), %%st\n\t"

/* Compare q[0] (ST(0)) with q[1] (ST(1)), then fcmov in the next block.
 * The popping forms leave ST(1) behind, the others both. */
#define FCOMI_FCMOV(cmp, pops, cc)   \
    "fldl 8(%[q])\n\t"               \
    "fldl (%[q])\n\t" cmp NEXT_BLOCK \
    FCMOV_BODY(cc) "fstp %%st(0)\n\t" pops

#define DEF_FCMOV(cc)                                                                \
    static int popf_##cc(uint64_t fl, int split) {                                   \
        struct fio io = {1.0, 2.0, 0.0};                                             \
        struct fio* p = &io;                                                         \
        if (split) {                                                                 \
            __asm__ volatile(SET_FLAGS NEXT_BLOCK FCMOV_BODY(cc)                     \
                             :                                                       \
                             : [p] "r"(p), [fl] "r"(fl)                              \
                             : "cc", "memory", "st", "st(1)");                       \
        } else {                                                                     \
            __asm__ volatile(SET_FLAGS FCMOV_BODY(cc)                                \
                             :                                                       \
                             : [p] "r"(p), [fl] "r"(fl)                              \
                             : "cc", "memory", "st", "st(1)");                       \
        }                                                                            \
        return io.r == 2.0;                                                          \
    }                                                                                \
    static int ucomisd_##cc(double a, double b, int split) {                         \
        struct fio io = {1.0, 2.0, 0.0};                                             \
        struct fio* p = &io;                                                         \
        if (split) {                                                                 \
            __asm__ volatile(SET_UCOMISD NEXT_BLOCK FCMOV_BODY(cc)                   \
                             :                                                       \
                             : [p] "r"(p), [a] "x"(a), [b] "x"(b)                    \
                             : "cc", "memory", "st", "st(1)");                       \
        } else {                                                                     \
            __asm__ volatile(SET_UCOMISD FCMOV_BODY(cc)                              \
                             :                                                       \
                             : [p] "r"(p), [a] "x"(a), [b] "x"(b)                    \
                             : "cc", "memory", "st", "st(1)");                       \
        }                                                                            \
        return io.r == 2.0;                                                          \
    }                                                                                \
    static int sahf_##cc(double a, double b, int split) {                            \
        struct fio io = {1.0, 2.0, 0.0};                                             \
        struct fio* p = &io;                                                         \
        double q[2] = {a, b};                                                        \
        double* qp = q;                                                              \
        if (split) {                                                                 \
            __asm__ volatile(SET_SAHF NEXT_BLOCK FCMOV_BODY(cc)                      \
                             :                                                       \
                             : [p] "r"(p), [q] "r"(qp)                               \
                             : "cc", "memory", "rax", "st", "st(1)");                \
        } else {                                                                     \
            __asm__ volatile(SET_SAHF FCMOV_BODY(cc)                                 \
                             :                                                       \
                             : [p] "r"(p), [q] "r"(qp)                               \
                             : "cc", "memory", "rax", "st", "st(1)");                \
        }                                                                            \
        return io.r == 2.0;                                                          \
    }                                                                                \
    static int test_##cc(uint32_t v, int split) {                                    \
        struct fio io = {1.0, 2.0, 0.0};                                             \
        struct fio* p = &io;                                                         \
        if (split) {                                                                 \
            __asm__ volatile(SET_TEST NEXT_BLOCK FCMOV_BODY(cc)                      \
                             :                                                       \
                             : [p] "r"(p), [v] "r"(v)                                \
                             : "cc", "memory", "st", "st(1)");                       \
        } else {                                                                     \
            __asm__ volatile(SET_TEST FCMOV_BODY(cc)                                 \
                             :                                                       \
                             : [p] "r"(p), [v] "r"(v)                                \
                             : "cc", "memory", "st", "st(1)");                       \
        }                                                                            \
        return io.r == 2.0;                                                          \
    }                                                                                \
    static int fcomi_##cc(int cmp, double a, double b) {                             \
        struct fio io = {1.0, 2.0, 0.0};                                             \
        struct fio* p = &io;                                                         \
        double q[2] = {a, b};                                                        \
        double* qp = q;                                                              \
        switch (cmp) {                                                               \
            case 0:                                                                  \
                __asm__ volatile(FCOMI_FCMOV(CMP_FCOMI, "fstp %%st(0)\n\t", cc)      \
                                 :                                                   \
                                 : [p] "r"(p), [q] "r"(qp)                           \
                                 : "cc", "memory", "st", "st(1)", "st(2)", "st(3)"); \
                break;                                                               \
            case 1:                                                                  \
                __asm__ volatile(FCOMI_FCMOV(CMP_FUCOMI, "fstp %%st(0)\n\t", cc)     \
                                 :                                                   \
                                 : [p] "r"(p), [q] "r"(qp)                           \
                                 : "cc", "memory", "st", "st(1)", "st(2)", "st(3)"); \
                break;                                                               \
            case 2:                                                                  \
                __asm__ volatile(FCOMI_FCMOV(CMP_FCOMIP, "", cc)                     \
                                 :                                                   \
                                 : [p] "r"(p), [q] "r"(qp)                           \
                                 : "cc", "memory", "st", "st(1)", "st(2)", "st(3)"); \
                break;                                                               \
            default:                                                                 \
                __asm__ volatile(FCOMI_FCMOV(CMP_FUCOMIP, "", cc)                    \
                                 :                                                   \
                                 : [p] "r"(p), [q] "r"(qp)                           \
                                 : "cc", "memory", "st", "st(1)", "st(2)", "st(3)"); \
                break;                                                               \
        }                                                                            \
        return io.r == 2.0;                                                          \
    }

DEF_FCMOV(u)
DEF_FCMOV(nu)
DEF_FCMOV(b)
DEF_FCMOV(e)
DEF_FCMOV(be)
DEF_FCMOV(nb)
DEF_FCMOV(ne)
DEF_FCMOV(nbe)

struct fcmov_kind {
    const char* name;
    int (*popf)(uint64_t, int);
    int (*ucomisd)(double, double, int);
    int (*sahf)(double, double, int);
    int (*test)(uint32_t, int);
    int (*fcomi)(int, double, double);
};

#define KIND(cc) {"fcmov" #cc, popf_##cc, ucomisd_##cc, sahf_##cc, test_##cc, fcomi_##cc}

static const struct fcmov_kind kinds[] = {
    KIND(u), KIND(nu), KIND(b), KIND(e), KIND(be), KIND(nb), KIND(ne), KIND(nbe),
};

/* Whether fcmov<kind> moves, from x86 EFLAGS. */
static int fcmov_moves(int kind, unsigned fl) {
    const int cf = (fl & F_CF) != 0;
    const int pf = (fl & F_PF) != 0;
    const int zf = (fl & F_ZF) != 0;
    switch (kind) {
        case 0:
            return pf;
        case 1:
            return !pf;
        case 2:
            return cf;
        case 3:
            return zf;
        case 4:
            return cf || zf;
        case 5:
            return !cf;
        case 6:
            return !zf;
        default:
            return !cf && !zf;
    }
}

/* EFLAGS after fcomi or ucomisd of a against b. */
static unsigned compare_flags(double a, double b) {
    if (isnan(a) || isnan(b)) {
        return F_ZF | F_PF | F_CF;
    }
    if (a < b) {
        return F_CF;
    }
    if (a == b) {
        return F_ZF;
    }
    return 0;
}

struct pair {
    const char* name;
    double a, b;
};

static const struct pair pairs[] = {
    {"unordered", NAN, 1.0},
    {"less", 1.0, 2.0},
    {"equal", 2.0, 2.0},
    {"greater", 3.0, 2.0},
};

#define NPAIRS (int)(sizeof(pairs) / sizeof(pairs[0]))
#define NKINDS (int)(sizeof(kinds) / sizeof(kinds[0]))

static const char* const layout[] = {"same block", "next block"};

static void section_a(void) {
    static const uint64_t popf_cases[] = {
        0, F_PF, F_OF, F_PF | F_OF, F_CF, F_ZF, F_CF | F_ZF, F_SF | F_AF, F_ALL, F_ALL & ~F_PF,
    };
    static const uint32_t test_cases[] = {0, 1, 3, 0x80, 0x81, 0x100};
    char name[160];

    for (int k = 0; k < NKINDS; k++) {
        for (int split = 0; split < 2; split++) {
            for (size_t i = 0; i < sizeof(popf_cases) / sizeof(popf_cases[0]); i++) {
                const unsigned fl = (unsigned)popf_cases[i];
                snprintf(name, sizeof(name), "A/popf %#05x; %s (%s)", fl, kinds[k].name,
                         layout[split]);
                check_int(name, kinds[k].popf(fl, split), fcmov_moves(k, fl));
            }
            for (int i = 0; i < NPAIRS; i++) {
                const unsigned fl = compare_flags(pairs[i].a, pairs[i].b);
                snprintf(name, sizeof(name), "A/ucomisd %s; %s (%s)", pairs[i].name, kinds[k].name,
                         layout[split]);
                check_int(name, kinds[k].ucomisd(pairs[i].a, pairs[i].b, split),
                          fcmov_moves(k, fl));
                snprintf(name, sizeof(name), "A/fucom; fnstsw; sahf %s; %s (%s)", pairs[i].name,
                         kinds[k].name, layout[split]);
                check_int(name, kinds[k].sahf(pairs[i].a, pairs[i].b, split), fcmov_moves(k, fl));
            }
            for (size_t i = 0; i < sizeof(test_cases) / sizeof(test_cases[0]); i++) {
                const uint32_t v = test_cases[i];
                unsigned fl = 0;
                if (v == 0) {
                    fl |= F_ZF;
                }
                if (v & 0x80000000U) {
                    fl |= F_SF;
                }
                if ((__builtin_popcount(v & 0xFF) & 1) == 0) {
                    fl |= F_PF;
                }
                snprintf(name, sizeof(name), "A/test %#x; %s (%s)", v, kinds[k].name,
                         layout[split]);
                check_int(name, kinds[k].test(v, split), fcmov_moves(k, fl));
            }
        }
    }
}

static const char* const cmp_names[] = {"fcomi", "fucomi", "fcomip", "fucomip"};

static void section_b(void) {
    char name[160];
    for (int c = 0; c < 4; c++) {
        for (int k = 0; k < NKINDS; k++) {
            for (int i = 0; i < NPAIRS; i++) {
                const unsigned fl = compare_flags(pairs[i].a, pairs[i].b);
                snprintf(name, sizeof(name), "B/%s %s; %s (next block)", cmp_names[c],
                         pairs[i].name, kinds[k].name);
                check_int(name, kinds[k].fcomi(c, pairs[i].a, pairs[i].b), fcmov_moves(k, fl));
            }
        }
    }
}

/* ── Readers stock translates, after the sidecar's compares ───────────────── */

struct readers {
    uint8_t setp, setnp, seto, sets, jp;
    uint32_t cmovp;
    uint8_t lahf;
    uint64_t pushf;
};

/* All the readers leave the flags alone, so each reads what the compare
 * left.  jp writes 1 to its slot only when taken. */
#define READERS                   \
    "setp 0(%[o])\n\t"            \
    "setnp 1(%[o])\n\t"           \
    "seto 2(%[o])\n\t"            \
    "sets 3(%[o])\n\t"            \
    "jp 2f\n\t"                   \
    "jmp 3f\n"                    \
    "2:\n\t"                      \
    "movb $1, 4(%[o])\n"          \
    "3:\n\t"                      \
    "movl $7, %%ecx\n\t"          \
    "movl $0, %%edx\n\t"          \
    "cmovpl %%ecx, %%edx\n\t"     \
    "movl %%edx, 8(%[o])\n\t"     \
    "lahf\n\t"                    \
    "movb %%ah, 12(%[o])\n\t"     \
    "leaq -128(%%rsp), %%rsp\n\t" \
    "pushfq\n\t"                  \
    "popq 16(%[o])\n\t"           \
    "leaq 128(%%rsp), %%rsp\n\t"

#define CMP_READERS(cmp, mid, pops) \
    "fldl 8(%[q])\n\t"              \
    "fldl (%[q])\n\t" cmp mid READERS pops

#define CLOBBERS "cc", "memory", "rax", "rcx", "rdx", "st", "st(1)"

static void run_readers(int cmp, int split, double a, double b, struct readers* out) {
    double q[2] = {a, b};
    double* qp = q;
    *out = (struct readers){0};
    struct readers* o = out;
    if (split) {
        switch (cmp) {
            case 0:
                __asm__ volatile(
                    CMP_READERS(CMP_FCOMI, NEXT_BLOCK, "fstp %%st(0)\n\tfstp %%st(0)\n\t")
                    :
                    : [q] "r"(qp), [o] "r"(o)
                    : CLOBBERS);
                break;
            case 1:
                __asm__ volatile(
                    CMP_READERS(CMP_FUCOMI, NEXT_BLOCK, "fstp %%st(0)\n\tfstp %%st(0)\n\t")
                    :
                    : [q] "r"(qp), [o] "r"(o)
                    : CLOBBERS);
                break;
            case 2:
                __asm__ volatile(CMP_READERS(CMP_FCOMIP, NEXT_BLOCK, "fstp %%st(0)\n\t")
                                 :
                                 : [q] "r"(qp), [o] "r"(o)
                                 : CLOBBERS);
                break;
            default:
                __asm__ volatile(CMP_READERS(CMP_FUCOMIP, NEXT_BLOCK, "fstp %%st(0)\n\t")
                                 :
                                 : [q] "r"(qp), [o] "r"(o)
                                 : CLOBBERS);
                break;
        }
    } else {
        switch (cmp) {
            case 0:
                __asm__ volatile(CMP_READERS(CMP_FCOMI, "", "fstp %%st(0)\n\tfstp %%st(0)\n\t")
                                 :
                                 : [q] "r"(qp), [o] "r"(o)
                                 : CLOBBERS);
                break;
            case 1:
                __asm__ volatile(CMP_READERS(CMP_FUCOMI, "", "fstp %%st(0)\n\tfstp %%st(0)\n\t")
                                 :
                                 : [q] "r"(qp), [o] "r"(o)
                                 : CLOBBERS);
                break;
            case 2:
                __asm__ volatile(CMP_READERS(CMP_FCOMIP, "", "fstp %%st(0)\n\t")
                                 :
                                 : [q] "r"(qp), [o] "r"(o)
                                 : CLOBBERS);
                break;
            default:
                __asm__ volatile(CMP_READERS(CMP_FUCOMIP, "", "fstp %%st(0)\n\t")
                                 :
                                 : [q] "r"(qp), [o] "r"(o)
                                 : CLOBBERS);
                break;
        }
    }
}

static void section_c(void) {
    char name[160];
    for (int c = 0; c < 4; c++) {
        for (int split = 0; split < 2; split++) {
            for (int i = 0; i < NPAIRS; i++) {
                const unsigned fl = compare_flags(pairs[i].a, pairs[i].b);
                const int pf = (fl & F_PF) != 0;
                struct readers r;
                run_readers(c, split, pairs[i].a, pairs[i].b, &r);
#define CHECK_READER(label, got, want)                                                   \
    snprintf(name, sizeof(name), "C/%s %s; %s (%s)", cmp_names[c], pairs[i].name, label, \
             layout[split]);                                                             \
    check_int(name, (long)(got), (long)(want))
                CHECK_READER("setp", r.setp, pf);
                CHECK_READER("setnp", r.setnp, !pf);
                CHECK_READER("seto", r.seto, 0);
                CHECK_READER("sets", r.sets, 0);
                CHECK_READER("jp", r.jp, pf);
                CHECK_READER("cmovp", r.cmovp, pf ? 7 : 0);
                CHECK_READER("lahf", r.lahf & 0xD5, fl & 0xD5);
                CHECK_READER("pushf", r.pushf & F_ALL, fl);
#undef CHECK_READER
            }
        }
    }
}

/* ── Flags across an x87 run ──────────────────────────────────────────────── */

static volatile double vx = 0.5, vy = 1.5;

/* popf; <x87 run>; pushf.  The run is the sidecar's alone. */
#define ACROSS(fn, body, ...)                                                              \
    static uint64_t fn(uint64_t fl) {                                                      \
        uint64_t out = 0;                                                                  \
        double r = 0.0;                                                                    \
        int32_t iv = 0;                                                                    \
        uint16_t sw = 0;                                                                   \
        double x = vx, y = vy;                                                             \
        double* rp = &r;                                                                   \
        int32_t* ip = &iv;                                                                 \
        uint16_t* sp = &sw;                                                                \
        uint64_t* op = &out;                                                               \
        __asm__ volatile(SET_FLAGS body                                                    \
                         "leaq -128(%%rsp), %%rsp\n\t"                                     \
                         "pushfq\n\t"                                                      \
                         "popq (%[out])\n\t"                                               \
                         "leaq 128(%%rsp), %%rsp\n\t"                                      \
                         :                                                                 \
                         : [fl] "r"(fl), [x] "m"(x), [y] "m"(y), [r] "r"(rp), [i] "r"(ip), \
                           [sw] "r"(sp), [out] "r"(op)                                     \
                         : "cc", "memory", "st", "st(1)");                                 \
        (void)r;                                                                           \
        (void)iv;                                                                          \
        (void)sw;                                                                          \
        return out & F_ALL;                                                                \
    }

ACROSS(across_arith, "fldl %[x]\n\tfldl %[y]\n\tfaddp\n\tfmull %[x]\n\tfstpl (%[r])\n\t")
ACROSS(across_fsin, "fldl %[x]\n\tfsin\n\tfstpl (%[r])\n\t")
ACROSS(across_fpatan, "fldl %[y]\n\tfldl %[x]\n\tfpatan\n\tfstpl (%[r])\n\t")
ACROSS(across_fcom, "fldl %[y]\n\tfldl %[x]\n\tfcompp\n\tfnstsw (%[sw])\n\t")
ACROSS(across_ftst, "fldl %[x]\n\tftst\n\tfnstsw (%[sw])\n\tfstpl (%[r])\n\t")
ACROSS(across_fistp, "fldl %[y]\n\tfistpl (%[i])\n\t")
ACROSS(across_frndint, "fldl %[y]\n\tfrndint\n\tfstpl (%[r])\n\t")
ACROSS(across_fild, "fildl (%[i])\n\tfistpl (%[i])\n\t")

static void section_d(void) {
    static const struct {
        const char* name;
        uint64_t (*fn)(uint64_t);
    } runs[] = {
        {"faddp; fmul", across_arith},   {"fsin", across_fsin},         {"fpatan", across_fpatan},
        {"fcompp; fnstsw", across_fcom}, {"ftst; fnstsw", across_ftst}, {"fistp", across_fistp},
        {"frndint", across_frndint},     {"fild; fistp", across_fild},
    };
    static const uint64_t patterns[] = {
        0, F_ALL, F_PF, F_AF, F_OF, F_PF | F_OF | F_SF, F_CF | F_ZF | F_AF};
    char name[160];
    for (size_t r = 0; r < sizeof(runs) / sizeof(runs[0]); r++) {
        for (size_t i = 0; i < sizeof(patterns) / sizeof(patterns[0]); i++) {
            snprintf(name, sizeof(name), "D/popf %#05llx; %s; pushf",
                     (unsigned long long)patterns[i], runs[r].name);
            check_int(name, (long)runs[r].fn(patterns[i]), (long)patterns[i]);
        }
    }
}

int main(void) {
    section_a();
    section_b();
    section_c();
    section_d();
    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
