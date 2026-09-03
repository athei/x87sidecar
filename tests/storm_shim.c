// Asynchronous-signal storm for the x86_64 test samples.
//
// Compiled into every sample next to coop_handshake.c.  Inert unless
// X87_TEST_STORM is set; then a constructor installs a counting SIGUSR1
// handler and starts a thread that sends SIGUSR1 to the main thread every
// X87_TEST_STORM microseconds (a bare "1" means 20 us).  Every x87 sequence a
// test executes is thereby interrupted at random points, which is what makes
// the runtime recover a precise guest state inside sidecar-emitted code.  A
// translation that cannot survive that shows up as a FAIL line from the test
// itself or as an abort from the runtime.

#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static volatile sig_atomic_t storm_signals = 0;
static pthread_t storm_target;
static long storm_interval_us = 20;

static void storm_handler(int sig, siginfo_t* si, void* ctx) {
    (void)sig;
    (void)si;
    (void)ctx;
    storm_signals++;
}

static void* storm_thread(void* arg) {
    (void)arg;
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGUSR1);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
    struct timespec ts;
    ts.tv_sec = storm_interval_us / 1000000;
    ts.tv_nsec = (storm_interval_us % 1000000) * 1000;
    for (;;) {
        if (pthread_kill(storm_target, SIGUSR1) != 0) {
            break;  // main thread gone
        }
        nanosleep(&ts, NULL);
    }
    return NULL;
}

__attribute__((destructor)) static void x87_test_storm_report(void) {
    if (storm_target != 0) {
        fprintf(stderr, "[storm] signals=%d\n", (int)storm_signals);
    }
}

__attribute__((constructor)) static void x87_test_storm(void) {
    const char* v = getenv("X87_TEST_STORM");
    if (!v || !*v || strcmp(v, "0") == 0) {
        return;
    }
    long us = atol(v);
    if (us > 1) {
        storm_interval_us = us;
    }
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = storm_handler;
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGUSR1, &sa, NULL) != 0) {
        return;
    }
    storm_target = pthread_self();
    pthread_t th;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    pthread_create(&th, &attr, storm_thread, NULL);
    pthread_attr_destroy(&attr);
}
