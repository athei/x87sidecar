#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/sysctl.h>
#include <sys/types.h>
#include <unistd.h>

// Checks that the loader released this process untraced, then dies in the
// way argv[1] names, so the harness can check that the shell sees the
// target's own exit status:
//
//   abort  SIGABRT                      exit status 134
//   segv   unhandled store through NULL 139
//   kill9  SIGKILL sent to itself       137
//   trap   ud2, SIGILL                  132
//   exit   exit(3), the control case    3
//
// A target left traced after the release takes some fatal signals through
// XNU's traced path, which can leave it waiting forever instead of exiting
// (see "Releasing the default attach stop" in docs/internals.md). The deaths
// below do not reach that case on their own, so the traced flag is checked
// directly and a traced process fails with exit 1 before dying.

static volatile double sink;

// 1 if this process is traced, 0 if not, -1 if the sysctl fails.
static int traced(void) {
    struct kinfo_proc kp;
    size_t len = sizeof(kp);
    int mib[4] = {CTL_KERN, KERN_PROC, KERN_PROC_PID, getpid()};
    memset(&kp, 0, sizeof(kp));
    if (sysctl(mib, 4, &kp, &len, NULL, 0) != 0) {
        return -1;
    }
    return (kp.kp_proc.p_flag & P_TRACED) != 0;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        puts("usage: test_crash_after_release abort|segv|kill9|trap|exit");
        return 2;
    }
    // One x87 instruction, so the block is translated with the hook in place.
    double one;
    __asm__ volatile("fld1\n\tfstpl %0" : "=m"(one));
    sink = one;
    const int t = traced();
    if (t != 0) {
        printf("FAIL still traced after the release (traced=%d)\n", t);
        return 1;
    }
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
