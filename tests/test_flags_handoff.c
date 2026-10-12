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
 * Section E: the same for every x87 instruction the sidecar translates,
 *            alone and in the fusion and IR shapes, with values held across
 *            a reply boundary, and over four input sets that reach the
 *            special-value branches of the f80 conversions.
 *
 * argv[1] selects sections by letter ("ABCDE" by default).
 *
 * Every case is checked against x86 semantics, so the native run is the
 * reference.
 */

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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
 * left.  jp writes 1 to its slot only when taken.  `movb %ah` cannot take
 * a REX prefix, so the pointers are pinned to rsi and rdi. */
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
                    : [q] "S"(qp), [o] "D"(o)
                    : CLOBBERS);
                break;
            case 1:
                __asm__ volatile(
                    CMP_READERS(CMP_FUCOMI, NEXT_BLOCK, "fstp %%st(0)\n\tfstp %%st(0)\n\t")
                    :
                    : [q] "S"(qp), [o] "D"(o)
                    : CLOBBERS);
                break;
            case 2:
                __asm__ volatile(CMP_READERS(CMP_FCOMIP, NEXT_BLOCK, "fstp %%st(0)\n\t")
                                 :
                                 : [q] "S"(qp), [o] "D"(o)
                                 : CLOBBERS);
                break;
            default:
                __asm__ volatile(CMP_READERS(CMP_FUCOMIP, NEXT_BLOCK, "fstp %%st(0)\n\t")
                                 :
                                 : [q] "S"(qp), [o] "D"(o)
                                 : CLOBBERS);
                break;
        }
    } else {
        switch (cmp) {
            case 0:
                __asm__ volatile(CMP_READERS(CMP_FCOMI, "", "fstp %%st(0)\n\tfstp %%st(0)\n\t")
                                 :
                                 : [q] "S"(qp), [o] "D"(o)
                                 : CLOBBERS);
                break;
            case 1:
                __asm__ volatile(CMP_READERS(CMP_FUCOMI, "", "fstp %%st(0)\n\tfstp %%st(0)\n\t")
                                 :
                                 : [q] "S"(qp), [o] "D"(o)
                                 : CLOBBERS);
                break;
            case 2:
                __asm__ volatile(CMP_READERS(CMP_FCOMIP, "", "fstp %%st(0)\n\t")
                                 :
                                 : [q] "S"(qp), [o] "D"(o)
                                 : CLOBBERS);
                break;
            default:
                __asm__ volatile(CMP_READERS(CMP_FUCOMIP, "", "fstp %%st(0)\n\t")
                                 :
                                 : [q] "S"(qp), [o] "D"(o)
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

/* ── Flags across every x87 instruction ───────────────────────────────────── */

/* Operands for section E, at fixed offsets the bodies use. */
struct emem {
    unsigned char x80[16];                               /*   0: f80 input */
    double a;                                            /*  16 */
    double b;                                            /*  24 */
    int32_t i32;                                         /*  32 */
    int16_t i16;                                         /*  36 */
    uint16_t cw;                                         /*  38: scratch for fnstcw/fldcw */
    int64_t i64;                                         /*  40 */
    unsigned char bcd[12];                               /*  48: packed BCD input */
    float f32;                                           /*  60 */
    unsigned char out[64];                               /*  64: results */
    unsigned char env[512] __attribute__((aligned(16))); /* 128: fsave, fxsave */
} __attribute__((aligned(16)));

_Static_assert(offsetof(struct emem, a) == 16, "a");
_Static_assert(offsetof(struct emem, i32) == 32, "i32");
_Static_assert(offsetof(struct emem, i16) == 36, "i16");
_Static_assert(offsetof(struct emem, cw) == 38, "cw");
_Static_assert(offsetof(struct emem, i64) == 40, "i64");
_Static_assert(offsetof(struct emem, bcd) == 48, "bcd");
_Static_assert(offsetof(struct emem, f32) == 60, "f32");
_Static_assert(offsetof(struct emem, out) == 64, "out");
_Static_assert(offsetof(struct emem, env) == 128, "env");

/* popf; <body>; pushf.  None of the bodies writes EFLAGS on x86, so pushf
 * must return what popf set.  Each body leaves the x87 stack empty. */
#define EPROBE(fn, body)                                                                    \
    static uint64_t fn(uint64_t fl, struct emem* m) {                                       \
        uint64_t out = 0;                                                                   \
        __asm__ volatile(SET_FLAGS body                                                     \
                         "leaq -128(%%rsp), %%rsp\n\t"                                      \
                         "pushfq\n\t"                                                       \
                         "popq %[out]\n\t"                                                  \
                         "leaq 128(%%rsp), %%rsp\n\t"                                       \
                         : [out] "=&r"(out)                                                 \
                         : [fl] "r"(fl), [m] "r"(m)                                         \
                         : "cc", "memory", "rax", "st", "st(1)", "st(2)", "st(3)", "st(4)", \
                           "st(5)", "st(6)", "st(7)");                                      \
        return out & F_ALL;                                                                 \
    }

#define LA "fldl 16(%[m])\n\t"
#define LB "fldl 24(%[m])\n\t"
#define LX "fldt (%[m])\n\t"
#define S0 "fstpl 64(%[m])\n\t"
#define S1 "fstpl 72(%[m])\n\t"
#define POP "fstp %%st(0)\n\t"

/* Constants */
EPROBE(e_fldz, "fldz\n\t" S0)
EPROBE(e_fld1, "fld1\n\t" S0)
EPROBE(e_fldl2e, "fldl2e\n\t" S0)
EPROBE(e_fldl2t, "fldl2t\n\t" S0)
EPROBE(e_fldlg2, "fldlg2\n\t" S0)
EPROBE(e_fldln2, "fldln2\n\t" S0)
EPROBE(e_fldpi, "fldpi\n\t" S0)
/* Loads and stores */
EPROBE(e_fld_m32, "flds 60(%[m])\n\tfstps 64(%[m])\n\t")
EPROBE(e_fld_m64, LA S0)
EPROBE(e_fld_m80, LX S0)
EPROBE(e_fstp_m80, LA "fstpt 80(%[m])\n\t")
EPROBE(e_fld_fstp_m80, LX "fstpt 80(%[m])\n\t")
EPROBE(e_fld_st, LA "fld %%st(0)\n\tfstp %%st(1)\n\t" S0)
EPROBE(e_fst, LA "fstl 64(%[m])\n\tfsts 72(%[m])\n\tfst %%st(0)\n\t" POP)
EPROBE(e_fstp_m32, LA "fstps 64(%[m])\n\t")
EPROBE(e_fstp_fld, LA S0 "fldl 64(%[m])\n\t" S1)
EPROBE(e_fld_m80_arith, LX "faddl 24(%[m])\n\tfstpt 80(%[m])\n\t")
/* Integer loads and stores */
EPROBE(e_fild_m16, "filds 36(%[m])\n\t" S0)
EPROBE(e_fild_m32, "fildl 32(%[m])\n\t" S0)
EPROBE(e_fild_m64, "fildll 40(%[m])\n\t" S0)
EPROBE(e_transport_m32, "fildl 32(%[m])\n\tfistpl 64(%[m])\n\t")
EPROBE(e_transport_m64, "fildll 40(%[m])\n\tfistpll 64(%[m])\n\t")
EPROBE(e_transport_m16, "filds 36(%[m])\n\tfists 64(%[m])\n\tfistps 66(%[m])\n\t")
EPROBE(e_transport_fisttp, "fildll 40(%[m])\n\tfisttpll 64(%[m])\n\t")
EPROBE(e_fist, LA "fistl 64(%[m])\n\tfistpll 72(%[m])\n\t")
EPROBE(e_fistp_m16, LA "fistps 64(%[m])\n\t")
EPROBE(e_fisttp_m32, LA "fisttpl 64(%[m])\n\t")
EPROBE(e_fisttp_m64, LA "fisttpll 64(%[m])\n\t")
EPROBE(e_fild_fistp_mixed, LA "fildll 40(%[m])\n\tfistpll 64(%[m])\n\t" S1)
EPROBE(e_fld_m80_fistp, LX "fistpll 64(%[m])\n\t")
/* Arithmetic */
EPROBE(e_fadd_m64, LA "faddl 24(%[m])\n\t" S0)
EPROBE(e_fsub_m64, LA "fsubl 24(%[m])\n\t" S0)
EPROBE(e_fsubr_m64, LA "fsubrl 24(%[m])\n\t" S0)
EPROBE(e_fmul_m64, LA "fmull 24(%[m])\n\t" S0)
EPROBE(e_fdiv_m64, LA "fdivl 24(%[m])\n\t" S0)
EPROBE(e_fdivr_m64, LA "fdivrl 24(%[m])\n\t" S0)
EPROBE(e_fadd_m32, LA "fadds 60(%[m])\n\t" S0)
EPROBE(e_fsub_m32, LA "fsubs 60(%[m])\n\t" S0)
EPROBE(e_fsubr_m32, LA "fsubrs 60(%[m])\n\t" S0)
EPROBE(e_fmul_m32, LA "fmuls 60(%[m])\n\t" S0)
EPROBE(e_fdiv_m32, LA "fdivs 60(%[m])\n\t" S0)
EPROBE(e_fdivr_m32, LA "fdivrs 60(%[m])\n\t" S0)
EPROBE(e_fadd_st, LA LB "fadd %%st(1), %%st\n\tfadd %%st, %%st(1)\n\t" S0 S1)
EPROBE(e_fsub_st, LA LB "fsub %%st(1), %%st\n\tfsub %%st, %%st(1)\n\t" S0 S1)
EPROBE(e_fsubr_st, LA LB "fsubr %%st(1), %%st\n\tfsubr %%st, %%st(1)\n\t" S0 S1)
EPROBE(e_fmul_st, LA LB "fmul %%st(1), %%st\n\tfmul %%st, %%st(1)\n\t" S0 S1)
EPROBE(e_fdiv_st, LA LB "fdiv %%st(1), %%st\n\tfdiv %%st, %%st(1)\n\t" S0 S1)
EPROBE(e_fdivr_st, LA LB "fdivr %%st(1), %%st\n\tfdivr %%st, %%st(1)\n\t" S0 S1)
EPROBE(e_faddp, LA LB "faddp\n\t" S0)
EPROBE(e_fsubp, LA LB "fsubp\n\t" S0)
EPROBE(e_fsubrp, LA LB "fsubrp\n\t" S0)
EPROBE(e_fmulp, LA LB "fmulp\n\t" S0)
EPROBE(e_fdivp, LA LB "fdivp\n\t" S0)
EPROBE(e_fdivrp, LA LB "fdivrp\n\t" S0)
EPROBE(e_fiadd, LA "fiaddl 32(%[m])\n\tfiadds 36(%[m])\n\t" S0)
EPROBE(e_fisub, LA "fisubl 32(%[m])\n\tfisubs 36(%[m])\n\t" S0)
EPROBE(e_fisubr, LA "fisubrl 32(%[m])\n\tfisubrs 36(%[m])\n\t" S0)
EPROBE(e_fimul, LA "fimull 32(%[m])\n\tfimuls 36(%[m])\n\t" S0)
EPROBE(e_fidiv, LA "fidivl 32(%[m])\n\tfidivs 36(%[m])\n\t" S0)
EPROBE(e_fidivr, LA "fidivrl 32(%[m])\n\tfidivrs 36(%[m])\n\t" S0)
EPROBE(e_fma_shape, LA "fmull 24(%[m])\n\tfaddl 16(%[m])\n\t" S0)
EPROBE(e_dot,
       "flds 60(%[m])\n\tfmuls 60(%[m])\n\tflds 60(%[m])\n\tfmuls 60(%[m])\n\tfaddp\n\t"
       "fstps 64(%[m])\n\t")
/* Unary */
EPROBE(e_fchs, LA "fchs\n\t" S0)
EPROBE(e_fabs, LA "fabs\n\t" S0)
EPROBE(e_fsqrt, LA "fsqrt\n\t" S0)
EPROBE(e_frndint, LA "frndint\n\t" S0)
EPROBE(e_fxtract, LA "fxtract\n\t" S0 S1)
EPROBE(e_fscale, LB LA "fscale\n\t" S0 POP)
EPROBE(e_fsin, LA "fsin\n\t" S0)
EPROBE(e_fcos, LA "fcos\n\t" S0)
EPROBE(e_fsincos, LA "fsincos\n\t" S0 S1)
EPROBE(e_fptan, LA "fptan\n\t" S0 S1)
EPROBE(e_fpatan, LB LA "fpatan\n\t" S0)
EPROBE(e_f2xm1, LA "f2xm1\n\t" S0)
EPROBE(e_fyl2x, LB LA "fyl2x\n\t" S0)
EPROBE(e_fyl2xp1, LB LA "fyl2xp1\n\t" S0)
EPROBE(e_fprem, LB LA "fprem\n\t" S0 POP)
EPROBE(e_fprem1, LB LA "fprem1\n\t" S0 POP)
/* Compares that write the status word */
EPROBE(e_fcom_m64, LA "fcoml 24(%[m])\n\t" POP)
EPROBE(e_fcom_m32, LA "fcoms 60(%[m])\n\t" POP)
EPROBE(e_fcomp_m64, LA "fcompl 24(%[m])\n\t")
EPROBE(e_fcom_st, LB LA "fcom %%st(1)\n\tfcomp %%st(1)\n\t" POP)
EPROBE(e_fcompp, LB LA "fcompp\n\t")
EPROBE(e_fucom, LB LA "fucom %%st(1)\n\tfucomp %%st(1)\n\t" POP)
EPROBE(e_fucompp, LB LA "fucompp\n\t")
EPROBE(e_ficom, LA "ficoml 32(%[m])\n\tficoms 36(%[m])\n\tficompl 32(%[m])\n\t")
EPROBE(e_ftst, LA "ftst\n\t" POP)
EPROBE(e_fxam, LA "fxam\n\t" POP)
EPROBE(e_fcom_fnstsw, LA "fcoml 24(%[m])\n\tfnstsw %%ax\n\t" POP)
EPROBE(e_fcomp_fnstsw, LA "fcompl 24(%[m])\n\tfnstsw %%ax\n\t")
EPROBE(e_fcompp_fnstsw, LB LA "fcompp\n\tfnstsw %%ax\n\t")
EPROBE(e_fucompp_fnstsw, LB LA "fucompp\n\tfnstsw 64(%[m])\n\t")
EPROBE(e_fnstsw_push, LA "fnstsw %%ax\n\t" POP)
/* Moves */
EPROBE(e_fxch, LA LB "fxch\n\t" S0 S1)
EPROBE(e_fxch_fstp, LA LB "fxch\n\tfstp %%st(1)\n\t" S0)
EPROBE(e_fxch_arithp, LA LB "fxch\n\tfsubrp\n\t" S0)
EPROBE(e_fcmovb, LA LB "fcmovb %%st(1), %%st\n\t" S0 POP)
EPROBE(e_fcmove, LA LB "fcmove %%st(1), %%st\n\t" S0 POP)
EPROBE(e_fcmovbe, LA LB "fcmovbe %%st(1), %%st\n\t" S0 POP)
EPROBE(e_fcmovu, LA LB "fcmovu %%st(1), %%st\n\t" S0 POP)
EPROBE(e_fcmovnb, LA LB "fcmovnb %%st(1), %%st\n\t" S0 POP)
EPROBE(e_fcmovne, LA LB "fcmovne %%st(1), %%st\n\t" S0 POP)
EPROBE(e_fcmovnbe, LA LB "fcmovnbe %%st(1), %%st\n\t" S0 POP)
EPROBE(e_fcmovnu, LA LB "fcmovnu %%st(1), %%st\n\t" S0 POP)
/* Control and stack management */
EPROBE(e_fldcw, "fnstcw 38(%[m])\n\tfldcw 38(%[m])\n\t")
EPROBE(e_fldcw_fist, "fnstcw 38(%[m])\n\tfldcw 38(%[m])\n\t" LA "fistpl 64(%[m])\n\t")
EPROBE(e_fnop, "fnop\n\t")
EPROBE(e_fdisi_feni, ".byte 0xdb, 0xe1\n\t.byte 0xdb, 0xe0\n\t")
EPROBE(e_ffree, LA LB "ffree %%st(0)\n\tfincstp\n\t" POP)
EPROBE(e_fdecstp, "fdecstp\n\tfincstp\n\t")
EPROBE(e_fnclex, "fnclex\n\t")
EPROBE(e_fninit, "fninit\n\t")
EPROBE(e_fsave, LA "fnsave 128(%[m])\n\tfrstor 128(%[m])\n\t" POP)
EPROBE(e_fstenv, LA "fnstenv 128(%[m])\n\tfldenv 128(%[m])\n\t" POP)
EPROBE(e_fxsave, LA "fxsave 128(%[m])\n\tfxrstor 128(%[m])\n\t" POP)
/* Packed BCD */
EPROBE(e_fbld, "fbld 48(%[m])\n\t" S0)
EPROBE(e_fbstp, LA "fbstp 80(%[m])\n\t")
EPROBE(e_fbld_fbstp, "fbld 48(%[m])\n\tfbstp 80(%[m])\n\t")
/* Longer runs, and values held across a reply boundary */
EPROBE(e_long_run, LA LB
       "fmul %%st(1), %%st\n\tfabs\n\tfsqrt\n\tfsin\n\tfrndint\n\t"
       "fistpl 64(%[m])\n\tftst\n\tfnstsw %%ax\n\t" POP)
EPROBE(e_across_m80, LX NEXT_BLOCK "fstpt 80(%[m])\n\t")
EPROBE(e_across_arith, LX NEXT_BLOCK LA "faddp\n\t" S0)
EPROBE(e_across_transport, "fildll 40(%[m])\n\t" NEXT_BLOCK LA S0 "fistpll 72(%[m])\n\t")
EPROBE(e_across_stack, LA LB LX NEXT_BLOCK "fstpt 80(%[m])\n\t" S0 S1)

struct eprobe {
    const char* name;
    uint64_t (*fn)(uint64_t, struct emem*);
};

#define EP(n) {#n, e_##n}

static const struct eprobe eprobes[] = {
    EP(fldz),
    EP(fld1),
    EP(fldl2e),
    EP(fldl2t),
    EP(fldlg2),
    EP(fldln2),
    EP(fldpi),
    EP(fld_m32),
    EP(fld_m64),
    EP(fld_m80),
    EP(fstp_m80),
    EP(fld_fstp_m80),
    EP(fld_st),
    EP(fst),
    EP(fstp_m32),
    EP(fstp_fld),
    EP(fld_m80_arith),
    EP(fild_m16),
    EP(fild_m32),
    EP(fild_m64),
    EP(transport_m32),
    EP(transport_m64),
    EP(transport_m16),
    EP(transport_fisttp),
    EP(fist),
    EP(fistp_m16),
    EP(fisttp_m32),
    EP(fisttp_m64),
    EP(fild_fistp_mixed),
    EP(fld_m80_fistp),
    EP(fadd_m64),
    EP(fsub_m64),
    EP(fsubr_m64),
    EP(fmul_m64),
    EP(fdiv_m64),
    EP(fdivr_m64),
    EP(fadd_m32),
    EP(fsub_m32),
    EP(fsubr_m32),
    EP(fmul_m32),
    EP(fdiv_m32),
    EP(fdivr_m32),
    EP(fadd_st),
    EP(fsub_st),
    EP(fsubr_st),
    EP(fmul_st),
    EP(fdiv_st),
    EP(fdivr_st),
    EP(faddp),
    EP(fsubp),
    EP(fsubrp),
    EP(fmulp),
    EP(fdivp),
    EP(fdivrp),
    EP(fiadd),
    EP(fisub),
    EP(fisubr),
    EP(fimul),
    EP(fidiv),
    EP(fidivr),
    EP(fma_shape),
    EP(dot),
    EP(fchs),
    EP(fabs),
    EP(fsqrt),
    EP(frndint),
    EP(fxtract),
    EP(fscale),
    EP(fsin),
    EP(fcos),
    EP(fsincos),
    EP(fptan),
    EP(fpatan),
    EP(f2xm1),
    EP(fyl2x),
    EP(fyl2xp1),
    EP(fprem),
    EP(fprem1),
    EP(fcom_m64),
    EP(fcom_m32),
    EP(fcomp_m64),
    EP(fcom_st),
    EP(fcompp),
    EP(fucom),
    EP(fucompp),
    EP(ficom),
    EP(ftst),
    EP(fxam),
    EP(fcom_fnstsw),
    EP(fcomp_fnstsw),
    EP(fcompp_fnstsw),
    EP(fucompp_fnstsw),
    EP(fnstsw_push),
    EP(fxch),
    EP(fxch_fstp),
    EP(fxch_arithp),
    EP(fcmovb),
    EP(fcmove),
    EP(fcmovbe),
    EP(fcmovu),
    EP(fcmovnb),
    EP(fcmovne),
    EP(fcmovnbe),
    EP(fcmovnu),
    EP(fldcw),
    EP(fldcw_fist),
    EP(fnop),
    EP(fdisi_feni),
    EP(ffree),
    EP(fdecstp),
    EP(fnclex),
    EP(fninit),
    EP(fsave),
    EP(fstenv),
    EP(fxsave),
    EP(fbld),
    EP(fbstp),
    EP(fbld_fbstp),
    EP(long_run),
    EP(across_m80),
    EP(across_arith),
    EP(across_transport),
    EP(across_stack),
};

/* f80 as sign/exponent and significand */
static void set_f80(unsigned char* p, uint16_t se, uint64_t sig) {
    memcpy(p, &sig, 8);
    memcpy(p + 8, &se, 2);
}

/* Input sets: ordinary values; zero and a value outside binary64's range;
 * subnormals; NaN and infinity.  They take the conversion and special-case
 * branches as well as the common ones. */
static void fill_emem(struct emem* m, int set) {
    memset(m, 0, sizeof(*m));
    static const unsigned char bcd[10] = {0x21, 0x43, 0x65, 0x87, 0x09, 0, 0, 0, 0, 0};
    memcpy(m->bcd, bcd, sizeof(bcd));
    switch (set) {
        case 0:
            set_f80(m->x80, 0x3FFF, 0xC000000000000000ULL); /* 1.5 */
            m->a = 1.5;
            m->b = 2.25;
            m->f32 = 1.5f;
            m->i32 = 7;
            m->i16 = 3;
            m->i64 = 123456789;
            break;
        case 1:
            set_f80(m->x80, 0x7FF0, 0x8000000000000000ULL); /* above binary64 */
            m->a = -2.5;
            m->b = 0.0;
            m->f32 = -0.0f;
            m->i32 = -5;
            m->i16 = 0;
            m->i64 = -1;
            break;
        case 2:
            set_f80(m->x80, 0x0001, 0x8000000000000000ULL); /* below binary64 */
            m->a = 4.9406564584124654e-324;
            m->b = -3.0;
            m->f32 = 1e-40f;
            m->i32 = INT32_MIN;
            m->i16 = INT16_MIN;
            m->i64 = INT64_MIN;
            break;
        default:
            set_f80(m->x80, 0x7FFF, 0xC000000000000000ULL); /* quiet NaN */
            m->a = NAN;
            m->b = INFINITY;
            m->f32 = NAN;
            m->i32 = 0;
            m->i16 = -1;
            m->i64 = 0;
            break;
    }
}

static void section_e(void) {
    static const uint64_t patterns[] = {
        0, F_ALL, F_PF, F_AF, F_OF | F_SF, F_CF | F_ZF, F_PF | F_AF | F_ZF};
    static struct emem m;
    for (size_t p = 0; p < sizeof(eprobes) / sizeof(eprobes[0]); p++) {
        int bad = 0;
        for (int set = 0; set < 4; set++) {
            for (size_t i = 0; i < sizeof(patterns) / sizeof(patterns[0]); i++) {
                fill_emem(&m, set);
                const uint64_t got = eprobes[p].fn(patterns[i], &m);
                if (got != patterns[i]) {
                    printf("FAIL  E/%s input %d: popf %#05llx, pushf %#05llx\n", eprobes[p].name,
                           set, (unsigned long long)patterns[i], (unsigned long long)got);
                    bad++;
                }
            }
        }
        if (bad == 0) {
            printf("PASS  E/%s\n", eprobes[p].name);
        }
        failures += bad;
    }
}

int main(int argc, char** argv) {
    /* Sections to run, as letters; all of them by default. */
    const char* only = argc > 1 ? argv[1] : "ABCDE";
    if (strchr(only, 'A')) {
        section_a();
    }
    if (strchr(only, 'B')) {
        section_b();
    }
    if (strchr(only, 'C')) {
        section_c();
    }
    if (strchr(only, 'D')) {
        section_d();
    }
    if (strchr(only, 'E')) {
        section_e();
    }
    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
