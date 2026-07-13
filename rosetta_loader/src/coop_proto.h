#ifndef X87SIDECAR_COOP_PROTO_H
#define X87SIDECAR_COOP_PROTO_H

// Wire protocol for the cooperative task-port handshake between a tracee (a
// binary built with the coop handshake shim, or wine with its handshake hook)
// and the x87sidecar running in `--cooperative` mode.
//
// Rendezvous: the sidecar publishes a Mach receive port in the bootstrap
// namespace under the name "x87sidecar.<tracee-pid>" (see X87_COOP_ENV). The
// tracee looks it up and sends its own task + thread control ports, then blocks
// for a reply. This replaces task_for_pid + ptrace(PT_ATTACHEXC), so the
// sidecar binary needs no com.apple.security.get-task-allow and is notarizable.
//
// This header is plain C so the x86_64 test shim (and wine's unix loader) can
// include it directly.

#include <mach/mach.h>

// Environment variable carrying the bootstrap service name to the tracee.
// The sidecar sets it (via setenv) before forking so it survives the execv.
#define X87_COOP_ENV "X87_SIDECAR_BOOTSTRAP"

// msgh_id of the handshake request (tracee -> sidecar). The reply uses +1.
#define X87_COOP_MSGH_ID 0x78383721 /* 'x87!' */

// Request: tracee -> sidecar. Complex message with two port descriptors — the
// tracee's task control port and the handshake thread's control port, both
// COPY_SEND. msgh_local_port carries a MAKE_SEND_ONCE reply right that the
// sidecar answers on to unblock the tracee.
typedef struct {
    mach_msg_header_t header;
    mach_msg_body_t body;
    mach_msg_port_descriptor_t task_port;
    mach_msg_port_descriptor_t thread_port;
} x87_coop_request_t;

// Receive buffer on the sidecar side: request + room for the receive trailer.
typedef struct {
    x87_coop_request_t req;
    mach_msg_trailer_t trailer;
} x87_coop_request_rcv_t;

// Reply: sidecar -> tracee. Its arrival unblocks the tracee. It also carries up
// to two code ranges the sidecar patched (the translate_insn entry and the
// injected handler) so the tracee can invalidate its OWN instruction cache for
// them. This is required in cooperative mode: unlike the default attach (which
// patches translate_insn before it has ever executed), the cooperative attach
// happens post-Rosetta-init, when translate_insn is already hot in the i-cache
// — and a cross-process mach_vm_machine_attribute flush does not reliably make
// the patch visible. sys_icache_invalidate() on the tracee's own thread (IC
// IVAU, broadcast inner-shareable) does. len == 0 means "unused slot".
typedef struct {
    mach_msg_header_t header;
    uint64_t icache_addr[2];
    uint64_t icache_len[2];
} x87_coop_reply_t;

#endif  // X87SIDECAR_COOP_PROTO_H
