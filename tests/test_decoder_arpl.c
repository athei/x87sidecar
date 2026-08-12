/*
 * test_decoder_arpl.c — exercise 32-bit ARPL (63 /r) under Rosetta via a 64->32->64
 * "Heaven's Gate" transition (the same technique Wine/CrossOver WoW64 uses).
 *
 * ARPL only exists in 32-bit mode (0x63 is MOVSXD in 64-bit), so to reach the
 * rosettax87_jit ARPL path we must actually switch the CPU to 32-bit compat
 * mode, run `arpl`, and switch back.
 *
 * Mechanism:
 *   - a tiny 32-bit stub assembled directly into __TEXT (gate_stub):
 *       arpl %dx,%ax ; push %edi (=cs64) ; push %esi (=ret offset) ; lret
 *     __TEXT is the linker-placed, read-only + executable (R^X) code segment, so
 *     no mmap and no runtime W^X juggling is needed. Under the shrunken __PAGEZERO
 *     the whole image sits below 4 GB, so the stub's address fits the 32-bit
 *     m16:32 far pointer used to reach it.
 *   - the 64-bit caller sets EAX/EDX (the arpl operands), loads ESI with the
 *     32-bit return offset and EDI with the 64-bit CS, switches to a flat 32-bit
 *     stack, and far-jmps to CS sel_cs32 : gate_stub.
 *   - the stub returns to 64-bit with a far-return (lret) off the 32-bit stack —
 *     it pushes {cs64, ret_off} and lret pops them. No far pointer is stored
 *     anywhere (which would need a runtime absolute address in read-only __TEXT,
 *     forbidden as a text-relocation); the return frame is built from registers.
 *   - sel_cs64 is the process's *own* 64-bit CS (0x2B) — user code cannot mint a
 *     long-mode segment in its LDT, so the return uses the existing one.
 *
 * If this runs where ARPL is unsupported (native Rosetta, which has no ARPL), the
 * 0x63 raises SIGILL rather than translating; a SIGILL handler recovers and the
 * test reports SKIP instead of wedging the process.
 *
 * Segments: the fixed macOS GDT selectors (USER_CS32=0x1B, USER_DS=0x23,
 * USER64_CS=0x2B) do NOT work under Rosetta — a far-jump to them yields
 * "rosetta error: invalid gdt selector index". Instead we allocate our own code
 * and data segments in the task's LDT via i386_set_ldt() (the same mechanism
 * Wine/CrossOver WoW64 uses on macOS). The returned LDT index becomes a selector
 * with TI=1 (LDT) and RPL=3:  sel = (index << 3) | (1 << 2) | 3.
 *
 * Build: clang -arch x86_64 -O2 -Wl,-pagezero_size,0x4000 -o test_decoder_arpl test_decoder_arpl.c
 */
#include <errno.h>
#include <setjmp.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <architecture/i386/desc.h>
#include <architecture/i386/table.h>
#include <i386/user_ldt.h>

static int failures = 0;

#define EFL_ZF 0x40u

/* LDT selectors for our 32-bit code, 32-bit data, and 64-bit code segments,
 * filled by setup_ldt(). A far-jump/segment load needs TI=1, RPL=3. */
static uint16_t sel_cs32 = 0;
static uint16_t sel_ds32 = 0;
static uint16_t sel_cs64 = 0;

/* Build the (index<<3 | LDT | RPL3) selector for an LDT slot. */
static uint16_t ldt_sel(int index) { return (uint16_t)((index << 3) | (1 << 2) | 3); }

/* Install one flat 4 GB 32-bit segment at an explicit LDT index and return its
 * selector. Rosetta rejects LDT_AUTO_ALLOC ("i386_set_ldt(kLdtAutoAlloc) not
 * supported") and multi-selector calls, so we set exactly one descriptor at a
 * chosen index.
 *   idx      — explicit LDT slot
 *   is_code  — code (executable) vs data (writable) segment
 * Base 0, limit 0xFFFFF pages (4 GB), 32-bit (D/B=1), DPL 3, present. */
static uint16_t alloc_ldt_seg(int idx, int is_code) {
    ldt_entry_t e;
    memset(&e, 0, sizeof e);
    if (is_code) {
        e.code.limit00  = 0xFFFF;
        e.code.base00   = 0;
        e.code.base16   = 0;
        e.code.type     = DESC_CODE_READ;   /* execute/read */
        e.code.dpl      = 3;
        e.code.present  = 1;
        e.code.limit16  = 0xF;
        e.code.opsz     = DESC_CODE_32B;
        e.code.granular = DESC_GRAN_PAGE;
        e.code.base24   = 0;
    } else {
        e.data.limit00  = 0xFFFF;
        e.data.base00   = 0;
        e.data.base16   = 0;
        e.data.type     = DESC_DATA_WRITE;
        e.data.dpl      = 3;
        e.data.present  = 1;
        e.data.limit16  = 0xF;
        e.data.stksz    = DESC_DATA_32B;
        e.data.granular = DESC_GRAN_PAGE;
        e.data.base24   = 0;
    }

    int got = i386_set_ldt(idx, &e, 1);
    if (got < 0) {
        printf("FAIL  i386_set_ldt(idx=%d,%s) failed (errno=%d %s)\n",
               idx, is_code ? "code" : "data", errno, strerror(errno));
        failures++;
        return 0;
    }
    return ldt_sel(idx);
}

/* Install the LDT segments the gate needs at explicit slots. macOS reserves LDT
 * indices 0..2 (i386_set_ldt returns EINVAL for them), so the first user slot is
 * 3. Only the 32-bit code + data segments are LDT-allocated; the 64-bit return
 * segment is the process's *existing* 64-bit CS — user code is not allowed to
 * mint a long-mode (L-bit) code segment in its LDT (i386_set_ldt returns EACCES),
 * exactly as WoW64 does: it returns through the real 64-bit CS. */
static void setup_ldt(void) {
    sel_cs32 = alloc_ldt_seg(/*idx=*/3, /*is_code=*/1);
    sel_ds32 = alloc_ldt_seg(/*idx=*/4, /*is_code=*/0);
    __asm__ volatile("mov %%cs, %0" : "=r"(sel_cs64));  /* real 64-bit CS (0x2B) */
}

/* The 32-bit gate stub, assembled directly into __TEXT (R^X, linker-placed,
 * below 4 GB under the shrunken __PAGEZERO — see file header). Entered in 32-bit
 * mode with EDI = 64-bit CS, ESI = 32-bit return offset:
 *     arpl %dx,%ax ; push %edi ; push %esi ; lret
 * The lret far-returns to 64-bit off the flat 32-bit stack. */
extern uint8_t gate_stub[];
__asm__(
    ".section __TEXT,__text\n"
    ".align 4\n"
    ".globl _gate_stub\n"
    "_gate_stub:\n"
    "  .byte 0x63, 0xd0\n"   /* arpl %dx, %ax                      */
    "  .byte 0x57\n"         /* push %edi   (64-bit CS for lret)   */
    "  .byte 0x56\n"         /* push %esi   (return offset)        */
    "  .byte 0xcb\n"         /* lret        (far return -> 64-bit) */
);

struct __attribute__((packed)) farptr32 {
    uint32_t off;
    uint16_t sel;
};

/* Run `arpl ax, dx` in 32-bit mode. Returns EAX out; writes EFLAGS out. */
static uint32_t run_arpl(uint32_t eax, uint32_t edx, uint32_t *eflags_out) {
    struct farptr32 to32 = {(uint32_t)(uintptr_t)gate_stub, sel_cs32};
    uint64_t in_eax = eax, in_edx = edx;
    uint32_t in_ds32 = sel_ds32, in_cs64 = sel_cs64;
    uint32_t out_eax = 0, out_flags = 0;

    __asm__ volatile(
        "leaq 1f(%%rip), %%rsi   \n"  /* ESI = 32-bit return offset (low 32)   */
        "movl %k[incs], %%edi    \n"  /* EDI = 64-bit CS (for the stub's lret) */
        "mov  %%ds, %%bx         \n"  /* save DS/ES/SS                         */
        "mov  %%es, %%r8w        \n"
        "mov  %%ss, %%r9w        \n"
        "movl %k[inds], %%eax    \n"  /* flat 32-bit LDT data segment          */
        "mov  %%ax, %%ds         \n"
        "mov  %%ax, %%es         \n"
        "mov  %%ax, %%ss         \n"  /* flat 32-bit stack (lret pops from it) */
        "movl %k[ineax], %%eax   \n"  /* EAX = dst selector                    */
        "movl %k[inedx], %%edx   \n"  /* EDX = src selector                    */
        "ljmp *%[fp]             \n"  /* -> CS sel_cs32 : gate_stub (32-bit)   */
        "1:                      \n"  /* landing (back in 64-bit via lret)     */
        "mov  %%bx, %%ds         \n"  /* restore DS/ES/SS                      */
        "mov  %%r8w, %%es        \n"
        "mov  %%r9w, %%ss        \n"
        "movl %%eax, %[oeax]     \n"
        "pushfq                  \n"
        "popq %%rcx              \n"
        "movl %%ecx, %[oflags]   \n"
        : [oeax] "=&r"(out_eax), [oflags] "=&r"(out_flags)
        : [ineax] "r"(in_eax), [inedx] "r"(in_edx), [inds] "r"(in_ds32),
          [incs] "r"(in_cs64), [fp] "m"(to32)
        : "rax", "rbx", "rcx", "rdx", "rsi", "rdi", "r8", "r9", "cc", "memory");

    *eflags_out = out_flags;
    return out_eax;
}

static uint32_t ref_arpl(uint32_t eax, uint32_t edx, int *zf) {
    uint32_t d = eax & 3, s = edx & 3;
    if (d < s) { *zf = 1; return (eax & ~3u) | s; }
    *zf = 0; return eax;
}

static void check(uint32_t eax, uint32_t edx) {
    int exp_zf;
    uint32_t exp_eax = ref_arpl(eax, edx, &exp_zf);

    uint32_t flags;
    uint32_t got_eax = run_arpl(eax, edx, &flags);
    int got_zf = (flags & EFL_ZF) != 0;

    if (got_eax != exp_eax || got_zf != exp_zf) {
        printf("FAIL  arpl eax=%08x edx=%08x -> eax got=%08x exp=%08x  zf got=%d exp=%d\n",
               eax, edx, got_eax, exp_eax, got_zf, exp_zf);
        failures++;
    }
}

/* Recovery point for the SIGILL guard (see below). */
static sigjmp_buf g_ill_env;
static volatile sig_atomic_t g_ill_hit = 0;

/* SIGILL handler: under native Rosetta the 32-bit `arpl` (0x63) is undecodable
 * and raises an illegal-instruction trap. Rather than let the process wedge,
 * jump back out to the recovery point set in main(). siglongjmp also restores
 * the signal mask so SIGILL stays deliverable.
 *
 * Unlike upstream this reports a FAIL rather than a SKIP, so a decode hook that
 * silently stopped working cannot pass unnoticed. Phase 12 of run_tests.sh is
 * what knows which environments are expected to trap. */
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

int main(void) {
    install_sigill_guard();

    /* If any `arpl` traps SIGILL (native Rosetta — no ARPL support), we land
     * here instead of hanging, and report that the environment lacks ARPL. */
    if (sigsetjmp(g_ill_env, 1)) {
        printf("FAIL  test_decoder_arpl — arpl raised SIGILL: 32-bit ARPL is not decoded "
               "here\n");
        return 1;
    }

    setup_ldt();          /* install LDT 32-bit code/data segments + read cs64 */
    if (failures) { printf("\n%d failure(s)\n", failures); return 1; }

    int total = 0;
    const uint32_t uppers[] = {0x00000000, 0x0000AB30, 0xFFFFFFFC, 0x1234BE00};
    for (unsigned di = 0; di < 4; di++)
        for (unsigned si = 0; si < 4; si++)
            for (unsigned u = 0; u < 4; u++) {
                check((uppers[u] & ~3u) | di, 0x00550000u | si);
                total++;
            }

    if (failures == 0) printf("PASS  test_decoder_arpl  (%d cases through 32-bit gate)\n", total);
    else printf("\n%d/%d cases FAILED\n", failures, total);
    return failures ? 1 : 0;
}
