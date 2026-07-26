#pragma once

#include <mach/mach.h>
#include <stdint.h>

#include <string>

// Sidecar Mach IPC service.
//
// After the loader's debugger phase detaches, we transition into "sidecar
// mode": run a Mach receive loop alongside the kqueue NOTE_EXIT watch on
// the parent (wine) process. The receive loop accepts messages from the
// inline IPC stub installed in stock translate_insn and (for M2) discards
// them. M3 will add real translation work + reply.
namespace sidecar {

// Mach IPC port plumbing for the inline stub.
//
// Two ports are involved:
//   1. Service port: owned by us (loader/sidecar). Parent gets a SEND
//      right under `*outParentReqName`. Stub uses that name as the
//      msgh_remote_port of every translate_insn call.
//   2. Reply port: owned by parent. Allocated directly into parent's
//      namespace via mach_port_allocate(parentTaskPort, RECEIVE, ...).
//      Stub uses `*outParentReplyName` as msgh_local_port with
//      MAKE_SEND_ONCE so the kernel hands the sidecar a fresh
//      SEND_ONCE per call. Sidecar replies on that.
//
// `parentTaskPort` must be a send-right to the parent's task port (held
// by MuhDebugger.taskPort_). Returns true on success.
bool installPortInParent(mach_port_t parentTaskPort, mach_port_t* outServicePort,
                         uint32_t* outParentReqName, uint32_t* outParentReplyName);

// Spawn a detached worker thread that runs the Mach receive loop on
// `servicePort`. The thread also needs `parentTaskPort` to
// mach_vm_read structs in the parent's address space (TranslationResult,
// IRInstr arrays). Returns true on success (thread started); the caller
// does NOT need to join it. The thread terminates implicitly on process
// exit.
bool spawnReceiveThread(mach_port_t servicePort, mach_port_t parentTaskPort);

// X87_PROFILE: read the parent-side counter array via mach_vm_read and
// append the counter section to the .prof file, then close the file.
// Called from main.cpp once kqueue NOTE_EXIT fires for the parent
// process — parentTaskPort is still valid at that point (parent task
// struct outlives NOTE_EXIT for a brief grace window).  No-op when
// X87_PROFILE was not set or counter allocation failed.
void dumpCountersIfEnabled(mach_port_t parentTaskPort);

// Guest-pc sampler.
//
// X87_SAMPLE=<path> enables it and names the profile, exactly like X87_PROFILE.
// Everything lands in that one self-describing file: the settings it ran with,
// whether it latched onto a single thread or swept them all, the per-thread
// sample counts, the leaf histogram and the folded stacks.
struct SamplerConfig {
    std::string path;  // X87_SAMPLE; empty = disabled
    bool all_threads = false;
    uint64_t interval_us = 1000;  // X87_SAMPLE_HZ, default 1 kHz
    uint64_t guest_lo = 0;        // default: all 32-bit guest code
    uint64_t guest_hi = 0x100000000ULL;
    double duration_s = 0;  // 0 = until the tracee exits
    double report_s = 10;   // profile rewrite interval
    bool unwind = true;     // walk the guest frame-pointer chain
};

// Overlay X87_SAMPLE / X87_SAMPLE_HZ / X87_SAMPLE_FOR / X87_SAMPLE_REPORT /
// X87_GUEST_RANGE / X87_ALL_THREADS / X87_NO_UNWIND onto `cfg`.  Env is how the
// app bundle enables this: gamelauncher passes fixed arguments, but applies its
// [env] table.
void samplerConfigFromEnv(SamplerConfig& cfg);

void startSampler(mach_port_t parentTaskPort, uint64_t runtimeBase, const SamplerConfig& cfg);

}  // namespace sidecar
