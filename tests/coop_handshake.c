// Cooperative-attach handshake shim (tracee side).
//
// Compiled into the x86_64 test samples so `x87sidecar --cooperative <sample>`
// can be exercised without wine or the game. A constructor runs BEFORE main():
// if X87_SIDECAR_BOOTSTRAP is set and names THIS process, it looks up the
// sidecar's bootstrap port, hands over the task + this thread's control ports,
// and blocks until the sidecar replies. This is the exact logic wine's unix
// loader performs; keeping it in one place (coop_proto.h) avoids drift.
//
// If the env var is absent or names another pid (e.g. inherited by an unrelated
// child), the shim is a no-op, so the same binary still runs in default mode.

#include <libkern/OSCacheControl.h>
#include <mach/mach.h>
#include <servers/bootstrap.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "coop_proto.h"

__attribute__((constructor)) static void x87_coop_handshake(void) {
    const char* name = getenv(X87_COOP_ENV);
    if (!name || !*name) {
        return;  // not running under `--cooperative`
    }

    // Staleness guard: the service name embeds the intended tracee pid
    // ("x87sidecar.<pid>"). A grandchild that merely inherited the env var must
    // NOT try to hand over its own (wrong) task port to a dead/foreign service.
    const char* dot = strrchr(name, '.');
    if (!dot || atoi(dot + 1) != (int)getpid()) {
        return;
    }

    int verbose = getenv("X87_LOGS") != NULL;

    mach_port_t bootstrap_port = MACH_PORT_NULL;
    if (task_get_bootstrap_port(mach_task_self(), &bootstrap_port) != KERN_SUCCESS) {
        return;
    }

    mach_port_t service = MACH_PORT_NULL;
    kern_return_t kr = bootstrap_look_up(bootstrap_port, name, &service);
    if (kr != KERN_SUCCESS) {
        if (verbose) fprintf(stderr, "coop: bootstrap_look_up(%s) failed 0x%x\n", name, kr);
        return;
    }

    // Reply port we block on until the sidecar has finished its setup.
    mach_port_t reply = MACH_PORT_NULL;
    mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_RECEIVE, &reply);

    x87_coop_request_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, MACH_MSG_TYPE_MAKE_SEND_ONCE) |
                           MACH_MSGH_BITS_COMPLEX;
    msg.header.msgh_size = sizeof(msg);
    msg.header.msgh_remote_port = service;  // destination: sidecar
    msg.header.msgh_local_port = reply;     // reply right → send-once to sidecar
    msg.header.msgh_id = X87_COOP_MSGH_ID;
    msg.body.msgh_descriptor_count = 2;
    msg.task_port.name = mach_task_self();
    msg.task_port.disposition = MACH_MSG_TYPE_COPY_SEND;
    msg.task_port.type = MACH_MSG_PORT_DESCRIPTOR;
    msg.thread_port.name = mach_thread_self();
    msg.thread_port.disposition = MACH_MSG_TYPE_COPY_SEND;
    msg.thread_port.type = MACH_MSG_PORT_DESCRIPTOR;

    kr = mach_msg(&msg.header, MACH_SEND_MSG, sizeof(msg), 0, MACH_PORT_NULL,
                  MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
    if (kr != KERN_SUCCESS) {
        if (verbose) fprintf(stderr, "coop: handshake send failed 0x%x\n", kr);
        return;
    }
    if (verbose) fprintf(stderr, "coop: handed over task+thread ports; blocking for reply\n");

    // Block until the sidecar has done its install and replies. The receive
    // buffer needs room for the message header AND the trailer the kernel
    // appends, else mach_msg returns MACH_RCV_TOO_LARGE.
    struct {
        x87_coop_reply_t reply;
        mach_msg_trailer_t trailer;
        char slack[64];
    } rep;
    memset(&rep, 0, sizeof(rep));
    kr = mach_msg(&rep.reply.header, MACH_RCV_MSG, 0, sizeof(rep), reply, 30000 /*ms*/,
                  MACH_PORT_NULL);
    if (verbose) fprintf(stderr, "coop: handshake reply kr=0x%x; continuing\n", kr);

    // Invalidate our own i-cache for the code the sidecar patched. This runs on
    // the same thread that will execute translate_insn, and sys_icache_invalidate
    // issues a broadcast (inner-shareable) IC IVAU, so the patched entry becomes
    // visible on every core — which a cross-process flush from the sidecar does
    // not reliably achieve once translate_insn is already hot in the i-cache.
    if (kr == KERN_SUCCESS) {
        for (int i = 0; i < 2; i++) {
            if (rep.reply.icache_len[i] != 0) {
                sys_icache_invalidate((void*)(uintptr_t)rep.reply.icache_addr[i],
                                      (size_t)rep.reply.icache_len[i]);
            }
        }
    }

    unsetenv(X87_COOP_ENV);
}
