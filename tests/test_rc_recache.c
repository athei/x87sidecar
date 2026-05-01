/*
 * test_rc_recache.c — Verify StoreCW RC re-caching optimization.
 *
 * The optimization: after FLDCW writes a new control word, the lowerer
 * extracts RC directly from the value already in a GPR (via UBFX) rather
 * than invalidating the cache and forcing the next FISTP to re-read
 * control_word from memory with LDRH+UBFX.
 *
 * Key pattern (MSVC (int)float cast sandwich):
 *   FLD val; FISTP [r1];           ← uses current RC
 *   FLDCW [new_cw];               ← StoreCW: re-caches RC
 *   FLD val; FISTP [r2];           ← should use re-cached RC (no LDRH+UBFX)
 *
 * This test has 2+ RC consumers across a StoreCW boundary, which triggers
 * allocation of Wd_rc_cached and exercises the re-cache path.
 *
 * Build: clang -arch x86_64 -O0 -o test_rc_recache test_rc_recache.c
 */
#include <stdint.h>
#include <stdio.h>

static int failures = 0;

static void check_i32(const char* name, int32_t got, int32_t expected) {
    if (got != expected) {
        printf("FAIL  %-55s  got=%d  expected=%d\n", name, got, expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

#define CW_NEAREST 0x037F
#define CW_FLOOR 0x077F
#define CW_CEIL 0x0B7F
#define CW_TRUNC 0x0F7F

static void set_cw(uint16_t cw) {
    __asm__ volatile("fldcw %0" : : "m"(cw));
}

static uint16_t get_cw(void) {
    uint16_t cw;
    __asm__ volatile("fnstcw %0" : "=m"(cw));
    return cw;
}

/* ── FISTP + FLDCW + FISTP sandwich ──────────────────────────────────────
 * 2 RC consumers (FISTP) with a StoreCW (FLDCW) between them.
 * First FISTP uses the pre-existing RC; second uses the new RC.
 * The re-cache optimization means the second FISTP skips LDRH+UBFX.
 */
static void fistp_fldcw_fistp(double a, double b, uint16_t new_cw, int32_t* r1, int32_t* r2) {
    __asm__ volatile(
        "fldl   %4\n\t" /* push b             → ST(0)=b         */
        "fldl   %5\n\t" /* push a             → ST(0)=a, ST(1)=b */
        "fistpl %0\n\t" /* pop a → *r1        (uses current RC)  */
        "fldcw  %3\n\t" /* FLDCW new_cw       (StoreCW: re-cache) */
        "fistpl %1\n"   /* pop b → *r2        (uses new RC)      */
        : "=m"(*r1), "=m"(*r2)
        : "m"(*r1), "m"(new_cw), "m"(b), "m"(a));
}

/* ── FISTP + FLDCW + FISTP + FLDCW + FISTP ───────────────────────────────
 * 3 RC consumers with 2 StoreCWs between them — tests multiple re-caches.
 */
static void fistp_fldcw_fistp_fldcw_fistp(double a, double b, double c, uint16_t cw1, uint16_t cw2,
                                          int32_t* r1, int32_t* r2, int32_t* r3) {
    __asm__ volatile(
        "fldl   %6\n\t" /* push c */
        "fldl   %7\n\t" /* push b */
        "fldl   %8\n\t" /* push a */
        "fistpl %0\n\t" /* pop a → *r1  (initial RC)    */
        "fldcw  %4\n\t" /* FLDCW cw1    (1st re-cache)  */
        "fistpl %1\n\t" /* pop b → *r2  (cw1 RC)        */
        "fldcw  %5\n\t" /* FLDCW cw2    (2nd re-cache)  */
        "fistpl %2\n"   /* pop c → *r3  (cw2 RC)        */
        : "=m"(*r1), "=m"(*r2), "=m"(*r3)
        : "m"(*r1), "m"(cw1), "m"(cw2), "m"(c), "m"(b), "m"(a));
}

/* ── Classic MSVC sandwich: FNSTCW + FLDCW trunc + FISTP + FLDCW restore ─
 * Most common real-world pattern. The FISTP between two FLDCWs uses
 * truncation; a second FISTP after the restore uses nearest.
 */
static void msvc_sandwich_x2(double a, double b, int32_t* r_trunc, int32_t* r_nearest) {
    uint16_t saved_cw;
    uint16_t trunc_cw = CW_TRUNC;
    __asm__ volatile(
        "fldl   %4\n\t" /* push b              → ST(0)=b         */
        "fldl   %5\n\t" /* push a              → ST(0)=a, ST(1)=b */
        "fnstcw %2\n\t" /* save CW             (LoadCW)           */
        "fldcw  %3\n\t" /* FLDCW trunc         (StoreCW)          */
        "fistpl %0\n\t" /* pop a → *r_trunc    (truncate mode)    */
        "fldcw  %2\n\t" /* FLDCW saved         (StoreCW: restore) */
        "fistpl %1\n"   /* pop b → *r_nearest  (nearest mode)     */
        : "=m"(*r_trunc), "=m"(*r_nearest), "=m"(saved_cw)
        : "m"(trunc_cw), "m"(b), "m"(a));
}

int main(void) {
    uint16_t saved_cw = get_cw();

    /* ── FISTP + FLDCW + FISTP: nearest → floor ─────────────────────────── */
    printf("=== FISTP + FLDCW + FISTP (nearest → floor) ===\n");
    {
        int32_t r1, r2;
        set_cw(CW_NEAREST);
        fistp_fldcw_fistp(2.7, 2.7, CW_FLOOR, &r1, &r2);
        check_i32("nearest(2.7)=3, floor(2.7)=2:  r1", r1, 3);
        check_i32("nearest(2.7)=3, floor(2.7)=2:  r2", r2, 2);

        set_cw(CW_NEAREST);
        fistp_fldcw_fistp(-2.3, -2.3, CW_FLOOR, &r1, &r2);
        check_i32("nearest(-2.3)=-2, floor(-2.3)=-3:  r1", r1, -2);
        check_i32("nearest(-2.3)=-2, floor(-2.3)=-3:  r2", r2, -3);
    }

    /* ── FISTP + FLDCW + FISTP: floor → ceil ─────────────────────────────── */
    printf("\n=== FISTP + FLDCW + FISTP (floor → ceil) ===\n");
    {
        int32_t r1, r2;
        set_cw(CW_FLOOR);
        fistp_fldcw_fistp(2.3, 2.3, CW_CEIL, &r1, &r2);
        check_i32("floor(2.3)=2, ceil(2.3)=3:  r1", r1, 2);
        check_i32("floor(2.3)=2, ceil(2.3)=3:  r2", r2, 3);
    }

    /* ── 3 FISTPs, 2 FLDCWs: nearest → trunc → floor ─────────────────── */
    printf("\n=== 3 FISTPs, 2 FLDCWs (nearest → trunc → floor) ===\n");
    {
        int32_t r1, r2, r3;
        set_cw(CW_NEAREST);
        fistp_fldcw_fistp_fldcw_fistp(2.7, 2.7, 2.7, CW_TRUNC, CW_FLOOR, &r1, &r2, &r3);
        check_i32("nearest(2.7)=3:  r1", r1, 3);
        check_i32("trunc(2.7)=2:    r2", r2, 2);
        check_i32("floor(2.7)=2:    r3", r3, 2);

        set_cw(CW_NEAREST);
        fistp_fldcw_fistp_fldcw_fistp(-2.7, -2.7, -2.7, CW_TRUNC, CW_FLOOR, &r1, &r2, &r3);
        check_i32("nearest(-2.7)=-3:  r1", r1, -3);
        check_i32("trunc(-2.7)=-2:    r2", r2, -2);
        check_i32("floor(-2.7)=-3:    r3", r3, -3);
    }

    /* ── MSVC sandwich x2: trunc + nearest ───────────────────────────────── */
    printf("\n=== MSVC sandwich (trunc + nearest) ===\n");
    {
        int32_t rt, rn;
        set_cw(CW_NEAREST);
        msvc_sandwich_x2(2.9, 2.9, &rt, &rn);
        check_i32("trunc(2.9)=2", rt, 2);
        check_i32("nearest(2.9)=3", rn, 3);

        set_cw(CW_NEAREST);
        msvc_sandwich_x2(-2.9, -2.9, &rt, &rn);
        check_i32("trunc(-2.9)=-2", rt, -2);
        check_i32("nearest(-2.9)=-3", rn, -3);

        set_cw(CW_NEAREST);
        msvc_sandwich_x2(2.5, 2.5, &rt, &rn);
        check_i32("trunc(2.5)=2", rt, 2);
        check_i32("nearest(2.5)=2", rn, 2); /* ties to even */
    }

    set_cw(saved_cw);

    printf("\n%s  (%d failure%s)\n", failures == 0 ? "ALL PASS" : "SOME FAILURES", failures,
           failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
