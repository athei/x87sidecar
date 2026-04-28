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

// Allocate a fresh Mach receive port in this process and insert a SEND
// right under a chosen 32-bit name in `parentTaskPort`'s namespace.
// On success, *outServicePort holds the local receive port and
// *outParentNameRef holds the name the stub bytes should reference.
//
// `parentTaskPort` must be a send-right to the parent's task port (today's
// MuhDebugger holds this). Returns true on success.
bool installPortInParent(mach_port_t parentTaskPort,
                         mach_port_t* outServicePort,
                         uint32_t* outParentNameRef);

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
