/*
 * Signal handlers must see native x87 values and be able to replace them.
 * A round trip through an opaque save buffer can hide a layout mismatch, so
 * check the actual 80-bit payload at a known instruction-map boundary.
 * Both long mode and an LDT compatibility segment run the issue #23 chain.
 */
#include <architecture/i386/desc.h>
#include <architecture/i386/table.h>
#include <errno.h>
#include <i386/user_ldt.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static int failures;
static struct {
    double x;
    uint64_t expected;
} input;
static struct {
    double value;
    uint32_t bad;
    uint32_t replaced;
} output;
static pthread_t victim;
static atomic_bool running;
static volatile sig_atomic_t signals, observations, invalid_context, rewrite_context;
extern char context32_finish[], context64_finish[], gate_stub[];

/* Rosetta uses the full mcontext after an LDT is installed. Its extra
 * segment registers move the FPU area, including in the 64-bit handler. */
static _STRUCT_X86_FLOAT_STATE64* float_state(ucontext_t* uc) {
    if (uc->uc_mcsize == sizeof(_STRUCT_MCONTEXT64_FULL) ||
        uc->uc_mcsize == sizeof(_STRUCT_MCONTEXT_AVX64_FULL) ||
        uc->uc_mcsize == sizeof(_STRUCT_MCONTEXT_AVX512_64_FULL))
        return &((_STRUCT_MCONTEXT64_FULL*)uc->uc_mcontext)->__fs;
    return &uc->uc_mcontext->__fs;
}

static void handler(int sig, siginfo_t* info, void* context) {
    (void)sig;
    (void)info;
    ucontext_t* uc = context;
    signals++;
    uintptr_t pc = uc->uc_mcontext->__ss.__rip;
    if (pc == (uintptr_t)context32_finish || pc == (uintptr_t)context64_finish) {
        _STRUCT_X86_FLOAT_STATE64* fs = float_state(uc);
        uint16_t sw;
        memcpy(&sw, &fs->__fpu_fsw, sizeof(sw));
        /* Darwin's signal context contains physical x87 slots. */
        unsigned char* slot = (unsigned char*)&fs->__fpu_stmm0 + 16 * ((sw >> 11) & 7);
        uint64_t mant;
        uint16_t exp;
        memcpy(&mant, slot, 8);
        memcpy(&exp, slot + 8, 2);
        observations++;
        /* The chain yields positive, normal doubles for these inputs. Round the
         * native significand to nearest-even without using the handler FPU. */
        uint64_t rounded = (mant >> 11) + ((mant & 0x7ff) > 0x400 ||
                                           ((mant & 0x7ff) == 0x400 && ((mant >> 11) & 1)));
        uint64_t bits = ((uint64_t)(exp - 15361) << 52) + rounded;
        if (!(mant >> 63) || exp <= 15360 || exp >= 17407 ||
            (bits != input.expected && !(rewrite_context && bits == UINT64_C(0x3ff4000000000000))))
            invalid_context++;
        if (rewrite_context) {
            /* 1.25 in native extended precision. The following FSTP must
             * observe this edit, as a guest callback/context restore would. */
            mant = UINT64_C(0xa000000000000000);
            exp = 0x3fff;
            memcpy(slot, &mant, 8);
            memcpy(slot + 8, &exp, 2);
        }
    }
    double nested;
    __asm__ volatile("fninit; fld1; fldpi; fmulp; fsin; fstpl %0" : "=m"(nested) : : "st", "st(1)");
}

static void* sender(void* unused) {
    (void)unused;
    while (atomic_load_explicit(&running, memory_order_relaxed)) {
        if (pthread_kill(victim, SIGUSR1) != 0)
            return (void*)1;
        struct timespec delay = {0, 20000};
        nanosleep(&delay, NULL);
    }
    return NULL;
}

#define CHAIN \
    "fld st(0)\nfrndint\nfsubr st(1), st\nfxch\nfchs\nf2xm1\nfld1\nfaddp\nfscale\nfstp st(1)\n"
/* EAX/EDX address static objects below 4 GB in compatibility mode. */
#define BODY(a, d, label)                                                                      \
    "mov ecx, 1000\n0: fld QWORD PTR [" a "]\njmp 1f\n1:\n" CHAIN ".globl _" label "\n_" label \
    ":\njmp 2f\n2: fstp QWORD PTR [" d                                                         \
    "]\n"                                                                                      \
    "mov ebx, DWORD PTR [" d "]\ncmp ebx, DWORD PTR [" a                                       \
    "+8]\njne 3f\n"                                                                            \
    "mov ebx, DWORD PTR [" d "+4]\ncmp ebx, DWORD PTR [" a                                     \
    "+12]\nje 5f\n"                                                                            \
    "3: cmp DWORD PTR [" d                                                                     \
    "], 0\njne 4f\n"                                                                           \
    "cmp DWORD PTR [" d                                                                        \
    "+4], 0x3ff40000\njne 4f\n"                                                                \
    "inc DWORD PTR [" d "+12]\njmp 5f\n4: inc DWORD PTR [" d                                   \
    "+8]\n"                                                                                    \
    "5: dec ecx\njnz 0b\n"

__asm__(
    ".text\n.align 4\n.globl _gate_stub\n_gate_stub:\n"
    ".intel_syntax noprefix\n.code32\n" BODY(
        "eax", "edx",
        "context32_finish") "push edi\npush esi\nretf\n.code64\n.att_syntax prefix\n");

__attribute__((noinline)) static void run_chain64(void) {
    __asm__ volatile(
        ".intel_syntax noprefix\n" BODY("rax", "rdx", "context64_finish") ".att_syntax prefix\n"
        :
        : "a"(&input), "d"(&output)
        : "rbx", "rcx", "memory", "cc", "st", "st(1)", "st(2)");
}
static uint16_t sel_cs32 = 0;
static uint16_t sel_ds32 = 0;
static uint16_t sel_cs64 = 0;

static uint16_t ldt_sel(int index) {
    return (uint16_t)((index << 3) | (1 << 2) | 3);
}

static uint16_t alloc_ldt_seg(int idx, int is_code) {
    ldt_entry_t e;
    memset(&e, 0, sizeof e);
    if (is_code) {
        e.code.limit00 = 0xFFFF;
        e.code.base00 = 0;
        e.code.base16 = 0;
        e.code.type = DESC_CODE_READ;
        e.code.dpl = 3;
        e.code.present = 1;
        e.code.limit16 = 0xF;
        e.code.opsz = DESC_CODE_32B;
        e.code.granular = DESC_GRAN_PAGE;
        e.code.base24 = 0;
    } else {
        e.data.limit00 = 0xFFFF;
        e.data.base00 = 0;
        e.data.base16 = 0;
        e.data.type = DESC_DATA_WRITE;
        e.data.dpl = 3;
        e.data.present = 1;
        e.data.limit16 = 0xF;
        e.data.stksz = DESC_DATA_32B;
        e.data.granular = DESC_GRAN_PAGE;
        e.data.base24 = 0;
    }

    int got = i386_set_ldt(idx, &e, 1);
    if (got < 0) {
        printf("FAIL  i386_set_ldt(idx=%d,%s) failed (errno=%d %s)\n", idx,
               is_code ? "code" : "data", errno, strerror(errno));
        failures++;
        return 0;
    }
    return ldt_sel(idx);
}

static void setup_ldt(void) {
    sel_cs32 = alloc_ldt_seg(3, 1);
    sel_ds32 = alloc_ldt_seg(4, 0);
    __asm__ volatile("mov %%cs, %0" : "=r"(sel_cs64));
}

struct __attribute__((packed)) farptr32 {
    uint32_t off;
    uint16_t sel;
};

static void run_chain32(void) {
    struct farptr32 to32 = {(uint32_t)(uintptr_t)gate_stub, sel_cs32};
    uint64_t in_eax = (uintptr_t)&input, in_edx = (uintptr_t)&output;
    uint32_t in_ds32 = sel_ds32, in_cs64 = sel_cs64;

    __asm__ volatile(
        "leaq 1f(%%rip), %%rsi   \n"
        "movl %k[incs], %%edi    \n"
        "mov  %%ds, %%r10w         \n"
        "mov  %%es, %%r8w        \n"
        "mov  %%ss, %%r9w        \n"
        "movl %k[inds], %%eax    \n"
        "mov  %%ax, %%ds         \n"
        "mov  %%ax, %%es         \n"
        "mov  %%ax, %%ss         \n"
        "movl %k[ineax], %%eax   \n"
        "movl %k[inedx], %%edx   \n"
        "ljmp *%[fp]             \n"
        "1:                      \n"
        "mov  %%r10w, %%ds         \n"
        "mov  %%r8w, %%es        \n"
        "mov  %%r9w, %%ss        \n"

        :
        : [ineax] "r"(in_eax), [inedx] "r"(in_edx), [inds] "r"(in_ds32), [incs] "r"(in_cs64),
          [fp] "m"(to32)
        : "rax", "rbx", "rcx", "rdx", "rsi", "rdi", "r8", "r9", "r10", "cc", "memory", "st",
          "st(1)", "st(2)");
}

static double now(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void run_case(int compat, int rewrite, double x) {
    void (*run)(void) = compat ? run_chain32 : run_chain64;
    rewrite_context = 0;
    input.x = x;
    memset(&output, 0, sizeof(output));
    __asm__ volatile("fninit");
    run();
    memcpy(&input.expected, &output.value, 8);
    if (!isfinite(output.value) || fabs(output.value - exp2(x)) > 1e-12) {
        printf("FAIL  signal context: reference x=%g got=%.17g\n", x, output.value);
        failures++;
        return;
    }
    memset(&output, 0, sizeof(output));
    signals = observations = invalid_context = 0;
    rewrite_context = rewrite;
    atomic_store(&running, 1);
    pthread_t thread;
    if (pthread_create(&thread, NULL, sender, NULL) != 0) {
        fprintf(stderr, "FAIL  signal context: pthread_create\n");
        failures++;
        return;
    }
    double deadline = now() + 0.2;
    do {
        run();
    } while (now() < deadline);
    atomic_store(&running, 0);
    void* sender_error;
    pthread_join(thread, &sender_error);
    int ok = observations > 0 && invalid_context == 0 && output.bad == 0 && sender_error == NULL &&
             (!rewrite || output.replaced > 0) && (rewrite || output.replaced == 0);
    printf(
        "%s  signal context mode=%d rewrite=%d x=%g signals=%d checked=%d invalid=%d bad=%u "
        "replaced=%u\n",
        ok ? "PASS" : "FAIL", compat ? 32 : 64, rewrite, x, (int)signals, (int)observations,
        (int)invalid_context, output.bad, output.replaced);
    if (!ok)
        failures++;
}

int main(void) {
    setvbuf(stdout, NULL, _IOLBF, 0);
    setup_ldt();
    if (failures)
        return 1;
    victim = pthread_self();
    struct sigaction sa = {0};
    sa.sa_sigaction = handler;
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGUSR1, &sa, NULL) != 0)
        return 1;
    /* Subnormal inputs also put CLZ/variable shifts in the signal-recovery
     * path while the chain's result remains normal and easy to inspect. */
    const double values[] = {-0.005, 2.3, -3.7, 0x1p-1074, -0x1p-1074};
    for (int compat = 0; compat < 2; ++compat)
        for (int rewrite = 0; rewrite < 2; ++rewrite)
            for (unsigned i = 0; i < sizeof(values) / sizeof(values[0]); ++i)
                run_case(compat, rewrite, values[i]);
    return failures ? 1 : 0;
}
