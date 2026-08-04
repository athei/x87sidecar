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
// which thread it latched onto, the rate it actually achieved, the leaf
// histogram and the folded stacks.
struct SamplerConfig {
    std::string path;  // X87_SAMPLE; empty = disabled
    // X87_SAMPLE_HZ, default 10 kHz.  A sample costs ~10 us, so the rate buys
    // resolution at almost exactly 1% of one core per kHz, and it holds: 10 kHz
    // measured 9998 Hz achieved with nothing dropped and the ring under 2% full.
    // Ten because resolution is the scarce thing and cpu is not.  A capture that
    // came back too thin costs a whole session to retake, while the 10% of a core
    // this spends is not felt on any machine the target runs on.  Cadence starts
    // to get ragged above here (missed ticks are 0.1% at 8 kHz, 0.9% at 16 kHz):
    // no bias, since a late tick moves the deadline rather than bursting to
    // catch up, but there is no point paying for a rate the timer cannot hold.
    uint64_t interval_us = 100;
    // Discovery has its own, slower cadence: a sweep touches every thread in the
    // task, so it costs more than a millisecond of work on a real target and
    // must not inherit a high sampling rate.  X87_SAMPLE_SWEEP_HZ, never faster
    // than interval_us.  Left at 1 kHz while sampling went to 10, which is the
    // whole point of it being a separate knob: latching takes a couple of
    // hundred sweeps either way.
    uint64_t sweep_interval_us = 1000;
    // The guest pcs that mark the thread worth profiling.  Left unset, the
    // sampler finds the main executable image itself (see detectMainImage) and
    // uses its range; setting X87_GUEST_RANGE pins it instead.
    uint64_t guest_lo = 0;
    uint64_t guest_hi = 0x100000000ULL;
    bool guest_range_pinned = false;
    // Profile rewrite interval, and with it the window size below, which is what
    // sets this rather than durability: every catchable exit path flushes (see
    // flushSamplerIfEnabled), so it bounds only what an uncatchable kill takes
    // with it, and a rewrite is cheap (62 ms measured for a 10 MB profile).
    // Ten seconds because a window is the smallest stretch of a run that can be
    // asked about afterwards, and the questions worth asking are about phases of
    // a session (a fight, a load, a zone) rather than whole runs.  At 60 a
    // capture of a raid pull came back with the fight averaged into the login
    // screen and no way to separate them.
    double report_s = 10;
    // Also write each report interval on its own, appended to <path>.windows,
    // each record holding only the samples taken during that interval.  The
    // cumulative profile answers "where does this run spend its time" and
    // cannot answer "where does the fight spend its time", because every report
    // it has ever written covers everything since the process started.  A
    // window cannot be recovered afterwards, so it has to be written at the
    // time.  One appended file rather than the file per window this started as:
    // at a ten-second interval an hour of play left 360 of them in the
    // directory.  X87_SAMPLE_WINDOWS=0 turns it off.
    bool windows = true;
    bool unwind = true;  // walk the guest frame-pointer chain
};

// Overlay X87_SAMPLE / X87_SAMPLE_HZ / X87_SAMPLE_SWEEP_HZ / X87_SAMPLE_REPORT /
// X87_GUEST_RANGE / X87_NO_UNWIND onto `cfg`.  Env is how the app bundle enables
// this: gamelauncher passes fixed arguments, but applies its [env] table.
void samplerConfigFromEnv(SamplerConfig& cfg);

void startSampler(mach_port_t parentTaskPort, uint64_t runtimeBase, const SamplerConfig& in);

// Ask the sampler for one last profile write and wait for it.  The sampler
// thread is detached, so process exit kills it wherever it happens to be: without
// this, everything sampled since the last report interval is lost, which at a
// 60 s interval is up to a minute of the run.  No-op when sampling is off.
void flushSamplerIfEnabled();

}  // namespace sidecar
