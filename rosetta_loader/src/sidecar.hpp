#pragma once

#include <mach/mach.h>
#include <stdint.h>

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
bool installPortInParent(mach_port_t parentTaskPort,
                         mach_port_t* outServicePort,
                         uint32_t* outParentReqName,
                         uint32_t* outParentReplyName);

// Run the Mach receive loop on `servicePort`. The receive thread also
// needs `parentTaskPort` to mach_vm_read structs in the parent's address
// space (TranslationResult, IRInstr arrays). Blocks forever; caller is
// expected to set up parent-exit detection separately (e.g., kqueue +
// alternative thread).
void runReceiveLoop(mach_port_t servicePort, mach_port_t parentTaskPort);

// Spawn a detached worker thread that runs runReceiveLoop. Returns true
// on success (thread started); the caller does NOT need to join it. The
// thread terminates implicitly on process exit.
bool spawnReceiveThread(mach_port_t servicePort, mach_port_t parentTaskPort);

}  // namespace sidecar
