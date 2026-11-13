// test_x87_retranslate_mem.c -- memory kept per retranslation of x87 code
//
// Each case copies an x87 block to a fresh RWX page and runs it many times.
// Between runs it rewrites the immediate of the block's last instruction,
// `movl $imm, 8(%rdi)`, with the run's number, so that Rosetta translates the
// block again before every run, and the stored immediate shows that the run
// executed the new translation.  Under stock Rosetta each translation takes
// its buffers from an arena that the next translation reclaims, so the
// process does not grow with the number of retranslations.  The test sums
// the untagged anonymous memory in its own address space (VM user tag 0,
// what mach_vm_allocate gives without a tag) before and after the loop and
// fails when it grew by more than kSlackBytes.
//
//   gs               a %gs: operand: the BL to the TLS base routine adds a
//                    fixup to a list that is empty at the start of every
//                    translation.  An integer instruction comes first, so
//                    the instruction buffer is stock's;
//   first            an x87 run at the start of the block: its words are the
//                    first in the translation's instruction buffer, which is
//                    empty until then;
//   gs_stock_fixups  one %gs: x87 run, then 400 integer loads through %gs:,
//                    whose fixups stock appends to the same list after the
//                    sidecar's first, past the first 4 KB of it;
//   gs_many_runs     400 x87 runs with a %gs: operand, each followed by an
//                    integer %gs: load: 400 requests in one translation,
//                    with stock's fixups between the sidecar's;
//   gs_sidecar_fixups  the same 400 runs separated by a cmpq instead, so
//                    every fixup in the list is the sidecar's, and the list
//                    outgrows a region the sidecar handed over earlier in the
//                    same translation.
//
// Every run's x87 result is checked against its input, which changes per
// run.
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
// name_tag labels the closing `movl $imm, 8(%rdi)` (C7 47 08 imm32).
#define DEFINE64(name, body)                                                               \
    __asm__(".text\n.p2align 4\n.globl _" #name "\n_" #name ":\n" body ".globl _" #name    \
            "_tag\n_" #name "_tag:\nmovl $0, 8(%rdi)\nret\n.globl _" #name "_end\n_" #name \
            "_end:\n");                                                                    \
    extern char name[], name##_tag[], name##_end[];

#define R4(x) x x x x
#define R20(x) R4(x) R4(x) R4(x) R4(x) R4(x)
#define R400(x) R20(R20(x))

DEFINE64(gs_operand, "mov %rdx, %rax\nfldl %gs:(%rax)\nfstpl (%rdi)\n")
DEFINE64(first_x87, "fldl (%rsi)\nfstpl (%rdi)\n")
DEFINE64(gs_stock_fixups,
         "mov %rdx, %rax\nfldl %gs:(%rax)\nfstpl (%rdi)\n" R400("movq %gs:(%rax), %rcx\n"))
DEFINE64(gs_many_runs,
         "mov %rdx, %rax\n" R400("fldl %gs:(%rax)\nfstpl (%rdi)\nmovq %gs:(%rax), %rcx\n"))
DEFINE64(gs_sidecar_fixups,
         "mov %rdx, %rax\n" R400("fldl %gs:(%rax)\nfstpl (%rdi)\ncmpq %r11, %r11\n"))

typedef void (*BlockFn)(double* out, const double* in, uintptr_t gs_off);

enum {
    kPage = 4096,
    kTagImm = 3,  // offset of the imm32 in C7 47 08 imm32
    kSlackBytes = 64 * 1024,
};

struct test_case {
    const char* name;
    const char* code;
    const char* tag;
    const char* end;
    int gs;  // the block reads its input through %gs:
    unsigned warmup;
    unsigned iters;
};

static const struct test_case kCases[] = {
    {"gs", gs_operand, gs_operand_tag, gs_operand_end, 1, 200, 2000},
    {"first", first_x87, first_x87_tag, first_x87_end, 0, 200, 2000},
    {"gs_stock_fixups", gs_stock_fixups, gs_stock_fixups_tag, gs_stock_fixups_end, 1, 20, 200},
    {"gs_many_runs", gs_many_runs, gs_many_runs_tag, gs_many_runs_end, 1, 10, 100},
    {"gs_sidecar_fixups", gs_sidecar_fixups, gs_sidecar_fixups_tag, gs_sidecar_fixups_end, 1, 10,
     100},
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

// Run i: patches i into the block's closing immediate, places input(i) where
// the block reads it (the TSD slot for gs) and runs the block.  Counts in
// *bad a wrong x87 result and in *stale a run that stored an older
// immediate, which means it ran a translation from before the patch.
static void run_once(char* page, size_t imm_off, int gs, unsigned i, unsigned* bad,
                     unsigned* stale) {
    const double in = input(i);
    double out[2] = {0.0, 0.0};
    if (gs) {
        uint64_t bits;
        memcpy(&bits, &in, 8);
        pthread_setspecific(g_key, (const void*)(uintptr_t)bits);
    }
    const uint32_t imm = i + 1;
    memcpy(page + imm_off, &imm, 4);
    ((BlockFn)(uintptr_t)page)(out, &in, (uintptr_t)g_key * 8);
    uint32_t got;
    memcpy(&got, &out[1], 4);
    *bad += memcmp(&out[0], &in, 8) != 0;
    *stale += got != imm;
}

static int run_case(const struct test_case* c) {
    const size_t len = (size_t)(c->end - c->code);
    const size_t tag = (size_t)(c->tag - c->code);
    if (len > kPage || (uint8_t)c->tag[0] != 0xc7 || (uint8_t)c->tag[1] != 0x47 ||
        (uint8_t)c->tag[2] != 0x08) {
        printf("FAIL  retranslate %s: unexpected layout (%zu bytes)\n", c->name, len);
        return 1;
    }
    char* page =
        mmap(NULL, kPage, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANON, -1, 0);
    if (page == MAP_FAILED) {
        printf("FAIL  retranslate %s: no RWX page\n", c->name);
        return 1;
    }
    memset(page, 0xcc, kPage);
    memcpy(page, c->code, len);

    unsigned bad = 0;
    unsigned stale = 0;
    for (unsigned i = 0; i < c->warmup; i++) {
        run_once(page, tag + kTagImm, c->gs, i, &bad, &stale);
    }
    const uint64_t before = untagged_bytes();
    for (unsigned i = c->warmup; i < c->warmup + c->iters; i++) {
        run_once(page, tag + kTagImm, c->gs, i, &bad, &stale);
    }
    const uint64_t after = untagged_bytes();
    munmap(page, kPage);

    const unsigned runs = c->warmup + c->iters;
    const long long grew = (long long)(after - before);
    if (bad != 0 || stale != 0) {
        printf(
            "FAIL  retranslate %s: of %u runs, %u stored a wrong x87 result and %u ran a "
            "stale translation\n",
            c->name, runs, bad, stale);
        return 1;
    }
    if (grew > kSlackBytes) {
        printf(
            "FAIL  retranslate %s: untagged memory grew by %lld bytes over %u retranslations, "
            "at most %d allowed\n",
            c->name, grew, c->iters, kSlackBytes);
        return 1;
    }
    printf("PASS  retranslate %s (untagged memory %+lld bytes over %u retranslations)\n", c->name,
           grew, c->iters);
    return 0;
}

int main(int argc, char** argv) {
    setvbuf(stdout, NULL, _IOLBF, 0);
    const char* only = argc > 1 ? argv[1] : NULL;
    if (pthread_key_create(&g_key, NULL) != 0) {
        printf("FAIL  retranslate: pthread_key_create failed\n");
        return 1;
    }
    int failures = 0;
    for (size_t k = 0; k < sizeof(kCases) / sizeof(kCases[0]); k++) {
        if (only == NULL || strcmp(only, kCases[k].name) == 0) {
            failures += run_case(&kCases[k]);
        }
    }
    printf("%d failure%s\n", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
