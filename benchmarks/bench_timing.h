/*
 * bench_timing.h — wall-clock timing helper for the bench suite.
 *
 * We use clock_gettime(CLOCK_MONOTONIC) instead of clock() because some
 * benches (transcendentals) route work through a sidecar process via IPC,
 * and clock() (= getrusage(RUSAGE_SELF) on macOS) only counts CPU time of
 * the calling process — sidecar work and IPC blocking time are invisible
 * to it. CLOCK_MONOTONIC is wall-clock and counts everything.
 *
 * On Apple Silicon macOS, clock_gettime is served from the commpage (no
 * kernel trap), so the per-call overhead is tens of ns — negligible vs.
 * the 2,000,000-iteration loops the benches use.
 */
#pragma once

#include <stdint.h>
#include <time.h>

typedef uint64_t bench_ns_t;

static inline bench_ns_t bench_now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (bench_ns_t)ts.tv_sec * 1000000000ull + (bench_ns_t)ts.tv_nsec;
}
