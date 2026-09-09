#include <fcntl.h>
#include <mach/mach_vm.h>
#include <pthread.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include "rosetta_core/Config.h"
#include "rosetta_core/CoreConfig.h"
#include "rosetta_core/X87Trace.h"
#include "sidecar.hpp"

namespace sidecar {
namespace {
struct TraceState {
    x87trace::Control* control = nullptr;
    FILE* file = nullptr;
    std::string path;
    uint64_t hash = 0;
    pid_t pid = 0;
    std::mutex mutex;
    std::atomic<bool> stop{false};
    bool written = false;
    pthread_t watcher{};
    bool watcher_started = false;
};
TraceState trace;

uint64_t acquire(uint64_t& value) {
    return std::atomic_ref<uint64_t>(value).load(std::memory_order_acquire);
}

void writeTrace() {
    std::scoped_lock guard(trace.mutex);
    if (trace.control == nullptr || trace.written)
        return;
    const uint64_t next = acquire(trace.control->next);
    const uint64_t floor = next > x87trace::kCount ? next - x87trace::kCount : 0;
    auto* source = reinterpret_cast<x87trace::Record*>(trace.control + 1);
    std::vector<x87trace::Record> records(x87trace::kCount);
    uint64_t valid = 0;
    for (size_t i = 0; i < x87trace::kCount; ++i) {
        // Take the producer's same nonblocking lock. This avoids relying
        // on speculative/torn memcpy reads while an in-flight writer finishes.
        auto lock = std::atomic_ref<uint64_t>(source[i].locked);
        if (lock.fetch_or(1, std::memory_order_acquire))
            continue;
        const uint64_t sequence = source[i].sequence;
        if (sequence > floor && sequence <= next &&
            ((sequence - 1) & (x87trace::kCount - 1)) == i) {
            records[i] = source[i];
            records[i].locked = 0;
            ++valid;
        }
        lock.store(0, std::memory_order_release);
    }
    x87trace::FileHeader header{
        .hash = trace.hash,
        .pid = static_cast<uint64_t>(trace.pid),
        .next = next,
        .frozen = acquire(trace.control->frozen),
        .dropped = std::min<uint64_t>(next, x87trace::kCount) - valid,
    };
    const bool ok = std::fwrite(&header, sizeof(header), 1, trace.file) == 1 &&
                    std::fwrite(records.data(), sizeof(records[0]), records.size(), trace.file) ==
                        records.size();
    const bool flushed = std::fflush(trace.file) == 0;
    const bool closed = std::fclose(trace.file) == 0;
    trace.file = nullptr;
    trace.written = true;
    std::fprintf(stdout, "[x87trace] %s %s: events=%llu retained=%llu dropped=%llu frozen=%llu\n",
                 ok && flushed && closed ? "wrote" : "write failed for", trace.path.c_str(), next,
                 valid, header.dropped, header.frozen);
    std::fflush(stdout);
}

void* watchTrace(void*) {
    while (!trace.stop.load(std::memory_order_acquire)) {
        if (acquire(trace.control->frozen)) {
            writeTrace();
            break;
        }
        usleep(100000);
    }
    return nullptr;
}
}  // namespace

void maybeEnableX87Trace(mach_port_t task, uint64_t hash) {
    if (g_rosetta_config == nullptr || g_rosetta_config->x87_trace_path.empty() ||
        hash != g_rosetta_config->x87_trace_hash)
        return;
    static bool attempted = false;
    if (attempted)
        return;
    attempted = true;
    std::scoped_lock guard(trace.mutex);
    if (pid_for_task(task, &trace.pid) != KERN_SUCCESS)
        return;
    trace.path = g_rosetta_config->x87_trace_path + "." + std::to_string(trace.pid) + ".x87trace";
    const int fd = open(trace.path.c_str(), O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC, 0600);
    if (fd == -1) {
        std::fprintf(stderr, "[x87trace] cannot create %s: %s\n", trace.path.c_str(),
                     strerror(errno));
        return;
    }
    trace.file = fdopen(fd, "wb");
    if (trace.file == nullptr) {
        close(fd);
        return;
    }
    mach_vm_address_t local = 0, parent = 0;
    auto kr = mach_vm_allocate(mach_task_self(), &local, x87trace::kBytes, VM_FLAGS_ANYWHERE);
    if (kr == KERN_SUCCESS) {
        vm_prot_t current, maximum;
        kr = mach_vm_remap(task, &parent, x87trace::kBytes, 0, VM_FLAGS_ANYWHERE, mach_task_self(),
                           local, FALSE, &current, &maximum, VM_INHERIT_NONE);
    }
    if (kr != KERN_SUCCESS) {
        std::fprintf(stderr, "[x87trace] shared allocation failed: %s\n", mach_error_string(kr));
        if (local)
            mach_vm_deallocate(mach_task_self(), local, x87trace::kBytes);
        std::fclose(trace.file);
        trace.file = nullptr;
        return;
    }
    trace.control = reinterpret_cast<x87trace::Control*>(local);
    trace.hash = hash;
    x87trace::set_buffer(parent, hash, g_rosetta_config->x87_trace_stop_negative);
    std::fprintf(stdout, "[x87trace] hash=0x%016llx output=%s stop_negative=%d\n", hash,
                 trace.path.c_str(), int(g_rosetta_config->x87_trace_stop_negative));
    std::fflush(stdout);
    if (pthread_create(&trace.watcher, nullptr, watchTrace, nullptr) == 0)
        trace.watcher_started = true;
    else
        std::fprintf(stderr, "[x87trace] watcher unavailable; trace will be written at exit\n");
}

void flushX87Trace() {
    trace.stop.store(true, std::memory_order_release);
    pthread_t watcher{};
    bool started;
    {
        std::scoped_lock guard(trace.mutex);
        watcher = trace.watcher;
        started = trace.watcher_started;
    }
    if (started)
        pthread_join(watcher, nullptr);
    writeTrace();
}
}  // namespace sidecar
