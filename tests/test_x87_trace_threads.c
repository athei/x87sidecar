#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
/* Keep workers within one batch of each other so the retained ring tail
 * includes every producer, even on runners with uneven scheduling. */
static atomic_int ready, start, running = 8, errors;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t condition = PTHREAD_COND_INITIALIZER;
static int waiting, generation;
static void barrier(void) {
    pthread_mutex_lock(&mutex);
    int current = generation;
    if (++waiting == 8) {
        waiting = 0;
        generation++;
        pthread_cond_broadcast(&condition);
    } else {
        while (generation == current)
            pthread_cond_wait(&condition, &mutex);
    }
    pthread_mutex_unlock(&mutex);
}
static void handler(int sig) {
    (void)sig;
    double temporary;
    __asm__ volatile("fninit; fldpi; fsin; fstpl %0" : "=m"(temporary)::"st");
}
__attribute__((noinline)) static double pitch(double x) {
    double out;
    __asm__ volatile(
        ".intel_syntax noprefix; fld QWORD PTR [rax]; jmp 1f; 1: "
        "fld st(0); frndint; fsubr st(1),st; fxch; fchs; f2xm1; fld1; faddp; fscale; fstp st(1); "
        "jmp 2f; 2: "
        "fstp QWORD PTR [rdx]; .att_syntax prefix" ::"a"(&x),
        "d"(&out)
        : "memory", "st", "st(1)", "st(2)");
    return out;
}
static void* worker(void* arg) {
    int id = (int)(intptr_t)arg;
    atomic_fetch_add(&ready, 1);
    while (!atomic_load(&start))
        sched_yield();
    int failures = 0;
    for (int n = 0; n < 131072; n++) {
        double x = -0.005 * (id + 1) - (n & 31) * 0.00001;
        double got = pitch(x), expected = exp2(x);
        if (!isfinite(got) || fabs(got - expected) > 1e-12)
            failures++;
        if ((n & 1023) == 1023)
            barrier();
    }
    atomic_fetch_add(&errors, failures);
    atomic_fetch_sub(&running, 1);
    return 0;
}
int main(void) {
    struct sigaction action = {0};
    action.sa_handler = handler;
    sigemptyset(&action.sa_mask);
    sigaction(SIGUSR1, &action, 0);
    pthread_t threads[8];
    for (int i = 0; i < 8; i++)
        if (pthread_create(&threads[i], 0, worker, (void*)(intptr_t)i))
            return 2;
    while (atomic_load(&ready) != 8)
        sched_yield();
    atomic_store(&start, 1);
    int signals = 0;
    while (atomic_load(&running)) {
        for (int i = 0; i < 8; i++)
            if (pthread_kill(threads[i], SIGUSR1) == 0)
                signals++;
        struct timespec pause = {0, 20000};
        nanosleep(&pause, 0);
    }
    for (int i = 0; i < 8; i++)
        pthread_join(threads[i], 0);
    printf("%s threaded pitch: 1048576 executions errors=%d signals=%d\n",
           atomic_load(&errors) ? "FAIL" : "PASS", atomic_load(&errors), signals);
    return atomic_load(&errors) != 0;
}
