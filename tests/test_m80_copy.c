/*
 * test_m80_copy.c: FLD m80fp immediately followed by FSTP m80fp.
 *
 * This is what a long double or Delphi Extended assignment compiles to
 * (`fld tbyte [src]; fstp tbyte [dst]`, also for Extended record fields).
 * Real x87 moves the ten bytes without conversion: FLD m80 does not quiet
 * an SNaN and FSTP m80 does not round, so every encoding comes out bit for
 * bit, including values outside binary64's range, NaN payloads, and the
 * unsupported encodings (unnormals, pseudo-denormals, pseudo-NaNs).
 *
 * Each value goes through several shapes: the pair alone, the pair inside a
 * longer x87 run with an unrelated value underneath it on the stack, two
 * pairs back to back, the pair with %gs-relative operands, and the pair
 * with plain register operands and no displacement ((%rsi), (%rdi), (%rsp),
 * one base register for both operands, a bare index register).  For the
 * register shapes the address is the guest register itself, so the test
 * also checks that every guest register it names is unchanged.  The x87
 * stack has to come out as it went in, with TOP and the tag word unchanged.
 *
 * The harness builds this at -O2 (cmake/AddX86Sample.cmake), where the "m"
 * operands come out as frame-relative addresses; the register shapes name
 * their registers explicitly so the addressing modes do not depend on the
 * compiler.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

typedef struct {
    uint64_t mant;
    uint16_t sexp;
} __attribute__((packed)) f80;

typedef struct {
    f80 v;
    uint8_t pad[6];
} __attribute__((aligned(16))) f80_slot;

static void copy_pair(const f80_slot* src, f80_slot* dst) {
    __asm__ volatile(
        "fldt  %1\n\t"
        "fstpt %0\n"
        : "=m"(*dst)
        : "m"(*src)
        : "st");
}

/* fld1 underneath, the copy in the middle, then the 1.0 is stored. */
static double copy_in_run(const f80_slot* src, f80_slot* dst) {
    double below;
    __asm__ volatile(
        "fld1\n\t"
        "fldt  %2\n\t"
        "fstpt %0\n\t"
        "fstpl %1\n"
        : "=m"(*dst), "=m"(below)
        : "m"(*src)
        : "st");
    return below;
}

static void copy_twice(const f80_slot* a, f80_slot* b, const f80_slot* c, f80_slot* d) {
    __asm__ volatile(
        "fldt  %2\n\t"
        "fstpt %0\n\t"
        "fldt  %3\n\t"
        "fstpt %1\n"
        : "=m"(*b), "=m"(*d)
        : "m"(*a), "m"(*c)
        : "st");
}

/* Both operands %gs-relative (TLS), so both addresses go through the
 * segment-base path.  The slots at %gs:0x300 are the ones test_fstpt_gs
 * uses. */
static void copy_gs(const f80_slot* src, f80_slot* dst) {
    uint64_t mant;
    uint16_t sexp;
    __asm__ volatile(
        "movq  %2, %%gs:0x300\n\t"
        "movw  %3, %%gs:0x308\n\t"
        "fldt  %%gs:0x300\n\t"
        "fstpt %%gs:0x310\n\t"
        "movq  %%gs:0x310, %0\n\t"
        "movw  %%gs:0x318, %1\n"
        : "=&r"(mant), "=&r"(sexp)
        : "r"(src->v.mant), "r"(src->v.sexp)
        : "st", "memory");
    dst->v.mant = mant;
    dst->v.sexp = sexp;
}

/* Register operands with no displacement.  For these the address is the
 * guest register itself, so the copy must not use that register as scratch.
 * Each shape returns 1 when every guest register it names is unchanged. */

/* fldt (%rsi); fstpt (%rdi) */
static int copy_rsi_rdi(const f80_slot* src, f80_slot* dst) {
    uint64_t rsi_after, rdi_after;
    __asm__ volatile(
        "movq  %2, %%rsi\n\t"
        "movq  %3, %%rdi\n\t"
        "fldt  (%%rsi)\n\t"
        "fstpt (%%rdi)\n\t"
        "movq  %%rsi, %0\n\t"
        "movq  %%rdi, %1\n"
        : "=&r"(rsi_after), "=&r"(rdi_after)
        : "r"(src), "r"(dst)
        : "rsi", "rdi", "st", "memory");
    return rsi_after == (uint64_t)(uintptr_t)src && rdi_after == (uint64_t)(uintptr_t)dst;
}

/* fldt (%rsp); fstpt (%rdi).  The source is staged below the red zone. */
static int copy_from_rsp(const f80_slot* src, f80_slot* dst) {
    uint64_t rsp_before, rsp_after, rdi_after;
    __asm__ volatile(
        "subq  $144, %%rsp\n\t"
        "movq  (%4), %%rax\n\t"
        "movq  %%rax, (%%rsp)\n\t"
        "movw  8(%4), %%ax\n\t"
        "movw  %%ax, 8(%%rsp)\n\t"
        "movq  %%rsp, %0\n\t"
        "fldt  (%%rsp)\n\t"
        "fstpt (%%rdi)\n\t"
        "movq  %%rsp, %1\n\t"
        "movq  %%rdi, %2\n\t"
        "addq  $144, %%rsp\n"
        : "=&r"(rsp_before), "=&r"(rsp_after), "=&r"(rdi_after)
        : "D"(dst), "r"(src)
        : "rax", "st", "memory");
    return rsp_after == rsp_before && rdi_after == (uint64_t)(uintptr_t)dst;
}

/* fldt (%rsi); fstpt (%rsp), read back from the stack before it is released. */
static int copy_to_rsp(const f80_slot* src, f80_slot* dst) {
    uint64_t rsp_before, rsp_after, rsi_after, mant;
    uint16_t sexp;
    __asm__ volatile(
        "subq  $144, %%rsp\n\t"
        "movq  %%rsp, %0\n\t"
        "fldt  (%%rsi)\n\t"
        "fstpt (%%rsp)\n\t"
        "movq  %%rsp, %1\n\t"
        "movq  %%rsi, %2\n\t"
        "movq  (%%rsp), %3\n\t"
        "movw  8(%%rsp), %4\n\t"
        "addq  $144, %%rsp\n"
        : "=&r"(rsp_before), "=&r"(rsp_after), "=&r"(rsi_after), "=&r"(mant), "=&r"(sexp)
        : "S"(src)
        : "st", "memory");
    dst->v.mant = mant;
    dst->v.sexp = sexp;
    return rsp_after == rsp_before && rsi_after == (uint64_t)(uintptr_t)src;
}

/* fldt (%rsi); fstpt 16(%rsi): one base register for both operands, with a
 * displacement only on the store.  pair[0] is the source, pair[1] the
 * destination. */
static int copy_same_base(f80_slot pair[2]) {
    uint64_t rsi_after;
    __asm__ volatile(
        "fldt  (%%rsi)\n\t"
        "fstpt 16(%%rsi)\n\t"
        "movq  %%rsi, %0\n"
        : "=&r"(rsi_after)
        : "S"(pair)
        : "st", "memory");
    return rsi_after == (uint64_t)(uintptr_t)pair;
}

/* fldt (,%rcx,1); fstpt (,%rdx,1): index registers with no base. */
static int copy_index_only(const f80_slot* src, f80_slot* dst) {
    uint64_t rcx_after, rdx_after;
    __asm__ volatile(
        "fldt  (,%%rcx,1)\n\t"
        "fstpt (,%%rdx,1)\n\t"
        "movq  %%rcx, %0\n\t"
        "movq  %%rdx, %1\n"
        : "=&r"(rcx_after), "=&r"(rdx_after)
        : "c"(src), "d"(dst)
        : "st", "memory");
    return rcx_after == (uint64_t)(uintptr_t)src && rdx_after == (uint64_t)(uintptr_t)dst;
}

/* The pair followed by FNSTENV: TOP (status bits 13:11) and the tag word. */
static void copy_then_env(const f80_slot* src, f80_slot* dst, uint16_t* sw, uint16_t* tw) {
    uint8_t env[28];
    __asm__ volatile(
        "fldt    %2\n\t"
        "fstpt   %0\n\t"
        "fnstenv %1\n"
        : "=m"(*dst), "=m"(env)
        : "m"(*src)
        : "st");
    memcpy(sw, env + 4, 2);
    memcpy(tw, env + 8, 2);
}

static void check_f80(const char* shape, const char* name, const f80_slot* got, uint16_t exp_sexp,
                      uint64_t exp_mant) {
    int ok = got->v.sexp == exp_sexp && got->v.mant == exp_mant;
    if (ok) {
        printf("PASS  %-14s %s\n", shape, name);
        return;
    }
    printf("FAIL  %-14s %-28s got=%04x:%016llx expected=%04x:%016llx\n", shape, name, got->v.sexp,
           (unsigned long long)got->v.mant, exp_sexp, (unsigned long long)exp_mant);
    failures++;
}

static void check_regs(const char* shape, const char* name, int regs_ok) {
    if (!regs_ok) {
        printf("FAIL  %-14s %-28s guest register changed\n", shape, name);
        failures++;
    }
}

typedef struct {
    const char* name;
    uint16_t sexp;
    uint64_t mant;
} Case;

static const Case kCases[] = {
    {"one plus 2^-63", 0x3FFF, 0x8000000000000001ULL},
    {"pi to 64 bits", 0x4000, 0xC90FDAA22168C235ULL},
    {"2^4097 (above f64 max)", 0x7000, 0x8000000000000000ULL},
    {"largest finite", 0x7FFE, 0xFFFFFFFFFFFFFFFFULL},
    {"smallest normal", 0x0001, 0x8000000000000000ULL},
    {"smallest denormal", 0x0000, 0x0000000000000001ULL},
    {"-2^-16192 (below f64 min)", 0x80BF, 0x8000000000000000ULL},
    {"negative zero", 0x8000, 0x0000000000000000ULL},
    {"negative infinity", 0xFFFF, 0x8000000000000000ULL},
    {"QNaN with payload", 0x7FFF, 0xC000000000000001ULL},
    {"SNaN with payload", 0x7FFF, 0xA000000000000001ULL},
    {"default NaN", 0xFFFF, 0xC000000000000000ULL},
    {"pseudo-denormal", 0x0000, 0x8000000000000001ULL},
    {"unnormal", 0x3FFF, 0x4000000000000000ULL},
    {"pseudo-infinity", 0x7FFF, 0x0000000000000000ULL},
    {"pseudo-NaN", 0x7FFF, 0x4000000000000001ULL},
};

int main(void) {
    const int n = (int)(sizeof(kCases) / sizeof(kCases[0]));
    for (int i = 0; i < n; i++) {
        const Case* c = &kCases[i];
        f80_slot src;
        memset(&src, 0, sizeof(src));
        src.v.sexp = c->sexp;
        src.v.mant = c->mant;

        f80_slot dst;
        memset(&dst, 0x5A, sizeof(dst));
        copy_pair(&src, &dst);
        check_f80("pair", c->name, &dst, c->sexp, c->mant);

        memset(&dst, 0x5A, sizeof(dst));
        double below = copy_in_run(&src, &dst);
        check_f80("in run", c->name, &dst, c->sexp, c->mant);
        if (below != 1.0) {
            printf("FAIL  %-14s %-28s value underneath got=%g expected=1\n", "in run", c->name,
                   below);
            failures++;
        }

        f80_slot b, d, other;
        memset(&b, 0x5A, sizeof(b));
        memset(&d, 0x5A, sizeof(d));
        memset(&other, 0, sizeof(other));
        other.v.sexp = 0xBFFF;
        other.v.mant = 0xC000000000000003ULL;
        copy_twice(&src, &b, &other, &d);
        check_f80("twice first", c->name, &b, c->sexp, c->mant);
        check_f80("twice second", c->name, &d, 0xBFFF, 0xC000000000000003ULL);

        memset(&dst, 0x5A, sizeof(dst));
        copy_gs(&src, &dst);
        check_f80("gs operands", c->name, &dst, c->sexp, c->mant);

        memset(&dst, 0x5A, sizeof(dst));
        check_regs("rsi to rdi", c->name, copy_rsi_rdi(&src, &dst));
        check_f80("rsi to rdi", c->name, &dst, c->sexp, c->mant);

        memset(&dst, 0x5A, sizeof(dst));
        check_regs("rsp to rdi", c->name, copy_from_rsp(&src, &dst));
        check_f80("rsp to rdi", c->name, &dst, c->sexp, c->mant);

        memset(&dst, 0x5A, sizeof(dst));
        check_regs("rsi to rsp", c->name, copy_to_rsp(&src, &dst));
        check_f80("rsi to rsp", c->name, &dst, c->sexp, c->mant);

        f80_slot pair[2];
        memset(pair, 0x5A, sizeof(pair));
        pair[0] = src;
        check_regs("same base", c->name, copy_same_base(pair));
        check_f80("same base", c->name, &pair[1], c->sexp, c->mant);
        check_f80("same base src", c->name, &pair[0], c->sexp, c->mant);

        memset(&dst, 0x5A, sizeof(dst));
        check_regs("index only", c->name, copy_index_only(&src, &dst));
        check_f80("index only", c->name, &dst, c->sexp, c->mant);

        uint16_t sw = 0, tw = 0;
        memset(&dst, 0x5A, sizeof(dst));
        copy_then_env(&src, &dst, &sw, &tw);
        check_f80("then fnstenv", c->name, &dst, c->sexp, c->mant);
        if (((sw >> 11) & 7) != 0 || tw != 0xFFFF) {
            printf("FAIL  %-14s %-28s top=%u tag=%04x expected top=0 tag=ffff\n", "then fnstenv",
                   c->name, (sw >> 11) & 7, tw);
            failures++;
        }
    }

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
