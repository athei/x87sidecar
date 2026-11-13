// test_x87_retranslate_mem.c -- memory kept per retranslation of x87 code
//
// Each case copies a short x87 block to a fresh RWX page and runs it many
// times, writing a byte on the page between runs, so that Rosetta translates
// the block again before every run (as test_x87_smc's data_write case does).
// Under stock Rosetta each translation takes its buffers from an arena that
// the next translation reclaims, so the process does not grow with the number
// of retranslations.  The test sums the untagged anonymous memory in its own
// address space (VM user tag 0, what mach_vm_allocate gives without a tag)
// before and after the loop and fails when it grew by more than kSlackBytes;
// one page kept per retranslation would add 8 MB.
//
//   gs     a %gs: operand: the BL to the TLS base routine adds a fixup to a
//          list that is empty at the start of every translation.  An integer
//          instruction comes first, so the instruction buffer is stock's;
//   first  an x87 run at the start of the block: its words are the first in
//          the translation's instruction buffer, which is empty until then.
//
// Every run's result is checked against its input, which changes per run.
//
// Compile:
//   clang -arch x86_64 -O2 -o test_x87_retranslate_mem test_x87_retranslate_mem.c

#include <mach/mach.h>
#include <mach/mach_vm.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>

// Called as fn(out, in, gs_off): out in %rdi, in in %rsi, gs_off in %rdx.
#define DEFINE64(name, body)                                                                 \
    __asm__(".text\n.p2align 4\n.globl _" #name "\n_" #name ":\n" body "ret\n.globl _" #name \
            "_end\n_" #name "_end:\n");                                                      \
    extern char name[], name##_end[];

DEFINE64(gs_operand, "mov %rdx, %rax\nfldl %gs:(%rax)\nfstpl (%rdi)\n")
DEFINE64(first_x87, "fldl (%rsi)\nfstpl (%rdi)\n")

typedef void (*BlockFn)(double* out, const double* in, uintptr_t gs_off);

enum {
    kPage = 4096,
    kDataByte = 0xc00,
    kWarmup = 200,
    kIters = 2000,
    kSlackBytes = 64 * 1024,
};

// Bytes in top-level regions with VM user tag 0.  Submaps (the shared
// cache) are skipped, not entered.
static uint64_t untagged_bytes(void) {
    uint64_t total = 0;
    mach_vm_address_t addr = 0;
    for (;;) {
        mach_vm_size_t size = 0;
        natural_t depth = 0;
        vm_region_submap_info_data_64_t info;
        mach_msg_type_number_t count = VM_REGION_SUBMAP_INFO_COUNT_64;
        if (mach_vm_region_recurse(mach_task_self(), &addr, &size, &depth,
                                   (vm_region_recurse_info_t)&info, &count) != KERN_SUCCESS) {
            break;
        }
        if (!info.is_submap && info.user_tag == 0) {
            total += size;
        }
        addr += size;
    }
    return total;
}

static pthread_key_t g_key;

static double input(unsigned i) {
    return 1.5 + (double)(i % 16);
}

// Runs the block once on input(i), placed where the block reads it (the TSD
// slot for gs); returns 1 when the block stored that value.
static int run_once(char* page, int gs, unsigned i) {
    const double in = input(i);
    double out = 0.0;
    if (gs) {
        uint64_t bits;
        memcpy(&bits, &in, 8);
        pthread_setspecific(g_key, (const void*)(uintptr_t)bits);
    }
    page[kDataByte] = (char)i;
    ((BlockFn)(uintptr_t)page)(&out, &in, (uintptr_t)g_key * 8);
    return memcmp(&out, &in, 8) == 0;
}

static int run_case(const char* name, const char* code, const char* code_end, int gs) {
    char* page =
        mmap(NULL, kPage, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANON, -1, 0);
    if (page == MAP_FAILED) {
        printf("FAIL  retranslate %s: no RWX page\n", name);
        return 1;
    }
    memset(page, 0xcc, kPage);
    memcpy(page, code, (size_t)(code_end - code));

    unsigned bad = 0;
    for (unsigned i = 0; i < kWarmup; i++) {
        bad += !run_once(page, gs, i);
    }
    const uint64_t before = untagged_bytes();
    for (unsigned i = 0; i < kIters; i++) {
        bad += !run_once(page, gs, i);
    }
    const uint64_t after = untagged_bytes();
    munmap(page, kPage);

    const long long grew = (long long)(after - before);
    if (bad != 0) {
        printf("FAIL  retranslate %s: %u of %u runs stored the wrong value\n", name, bad,
               kWarmup + kIters);
        return 1;
    }
    if (grew > kSlackBytes) {
        printf(
            "FAIL  retranslate %s: untagged memory grew by %lld bytes over %u retranslations, "
            "at most %d allowed\n",
            name, grew, kIters, kSlackBytes);
        return 1;
    }
    printf("PASS  retranslate %s (untagged memory %+lld bytes over %u retranslations)\n", name,
           grew, kIters);
    return 0;
}

int main(void) {
    setvbuf(stdout, NULL, _IOLBF, 0);
    if (pthread_key_create(&g_key, NULL) != 0) {
        printf("FAIL  retranslate: pthread_key_create failed\n");
        return 1;
    }
    int failures = 0;
    failures += run_case("gs", gs_operand, gs_operand_end, 1);
    failures += run_case("first", first_x87, first_x87_end, 0);
    printf("%d failure%s\n", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
