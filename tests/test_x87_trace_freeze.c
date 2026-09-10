#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>
__attribute__((noinline)) static double negative(void) {
    double result;
    __asm__ volatile("jmp 1f; 1: fld1; fchs; jmp 2f; 2: fstpl %0" : "=m"(result)::"st");
    return result;
}
int main(void) {
    setvbuf(stdout, NULL, _IOLBF, 0);
    for (int i = 0; i < 1000; i++) {
        if (negative() != -1.0) {
            puts("FAIL negative fixture");
            return 1;
        }
    }
    puts("PASS negative fixture: 1000 executions completed");
    const char* prefix = getenv("X87_TRACE_OUTPUT");
    if (prefix && *prefix) {
        char path[4096];
        int length = snprintf(path, sizeof(path), "%s.%d.x87trace", prefix, getpid());
        if (length < 0 || (size_t)length >= sizeof(path))
            return 2;
        int complete = 0;
        for (int i = 0; i < 500; ++i) {
            struct stat info;
            if (stat(path, &info) == 0 && info.st_size == 256 * (65536 + 1)) {
                complete = 1;
                break;
            }
            usleep(10000);
        }
        if (!complete) {
            puts("FAIL trace was not written while guest remained alive");
            return 1;
        }
        puts("PASS trace visible while guest remained alive");
    }
    puts("PASS negative fixture: exiting");
    return 0;
}
