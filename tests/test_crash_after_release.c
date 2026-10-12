#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// Dies after the loader has released it, in the way argv[1] names, so the
// harness can check that the shell sees the target's own exit status instead
// of a target left stopped behind a sidecar that waits for it forever:
//
//   abort  SIGABRT                      exit status 134
//   segv   unhandled store through NULL 139
//   kill9  SIGKILL sent to itself       137
//   trap   ud2, SIGILL                  132
//   exit   exit(3), the control case    3

static volatile double sink;

int main(int argc, char** argv) {
    if (argc < 2) {
        puts("usage: test_crash_after_release abort|segv|kill9|trap|exit");
        return 2;
    }
    // One x87 instruction, so the block is translated with the hook in place.
    double one;
    __asm__ volatile("fld1\n\tfstpl %0" : "=m"(one));
    sink = one;
    printf("released, x87 result %.1f, dying by %s\n", one, argv[1]);
    fflush(stdout);

    if (strcmp(argv[1], "abort") == 0) {
        abort();
    } else if (strcmp(argv[1], "segv") == 0) {
        *(volatile int*)0 = 1;
    } else if (strcmp(argv[1], "kill9") == 0) {
        raise(SIGKILL);
    } else if (strcmp(argv[1], "trap") == 0) {
        __asm__ volatile("ud2");
    } else if (strcmp(argv[1], "exit") == 0) {
        exit(3);
    } else {
        printf("unknown mode %s\n", argv[1]);
        return 2;
    }
    puts("FAIL still alive after the fatal action");
    return 1;
}
