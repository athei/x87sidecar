#include "sidecar.hpp"

#include <cstdio>
#include <pthread.h>

#include <mach/mach.h>
#include <mach/mach_port.h>
#include <mach/mach_vm.h>

namespace sidecar {

namespace {

// Largest message we expect to receive — header (24) + 5×8 args (40) +
// trailer + future payload. 4 KB is generous.
constexpr size_t kRecvBufferSize = 4096;

struct ThreadArgs {
    mach_port_t servicePort;
    mach_port_t parentTaskPort;
};

// Receive-loop worker thread entry point.
void* threadEntry(void* arg) {
    auto* a = reinterpret_cast<ThreadArgs*>(arg);
    runReceiveLoop(a->servicePort, a->parentTaskPort);
    delete a;
    return nullptr;
}

}  // namespace

bool installPortInParent(mach_port_t parentTaskPort,
                         mach_port_t* outServicePort,
                         uint32_t* outParentReqName,
                         uint32_t* outParentReplyName) {
    // Allocate a fresh receive port in this process.
    mach_port_t servicePort = MACH_PORT_NULL;
    kern_return_t kr =
        mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_RECEIVE, &servicePort);
    if (kr != KERN_SUCCESS) {
        fprintf(stderr,
                "sidecar: mach_port_allocate(RECEIVE) failed (0x%x %s)\n",
                kr, mach_error_string(kr));
        return false;
    }

    // Insert a send right (derived from our receive right) so we can
    // hand it across the task boundary.
    kr = mach_port_insert_right(mach_task_self(), servicePort, servicePort,
                                MACH_MSG_TYPE_MAKE_SEND);
    if (kr != KERN_SUCCESS) {
        fprintf(stderr,
                "sidecar: mach_port_insert_right(SELF, MAKE_SEND) failed (0x%x %s)\n",
                kr, mach_error_string(kr));
        return false;
    }

    // Allocate a fresh name in the parent task's namespace, then plant
    // our send right under that name. The parent process can mach_msg
    // to that name and the kernel will route to our servicePort.
    mach_port_name_t parentName = MACH_PORT_NULL;
    kr = mach_port_allocate(parentTaskPort, MACH_PORT_RIGHT_DEAD_NAME, &parentName);
    if (kr != KERN_SUCCESS) {
        fprintf(stderr,
                "sidecar: mach_port_allocate(parent, DEAD_NAME) failed (0x%x %s)\n",
                kr, mach_error_string(kr));
        return false;
    }
    // Drop the placeholder so the name slot is free.
    kr = mach_port_deallocate(parentTaskPort, parentName);
    if (kr != KERN_SUCCESS) {
        fprintf(stderr,
                "sidecar: mach_port_deallocate(parent placeholder) failed (0x%x %s)\n",
                kr, mach_error_string(kr));
        return false;
    }

    // Plant our send right under that freed name. The kernel resolves
    // (mach_task_self(), servicePort) to the underlying port object and
    // installs a send right at `parentName` in `parentTaskPort`'s ns.
    kr = mach_port_insert_right(parentTaskPort, parentName, servicePort,
                                MACH_MSG_TYPE_COPY_SEND);
    if (kr != KERN_SUCCESS) {
        fprintf(stderr,
                "sidecar: mach_port_insert_right(parent, COPY_SEND) failed "
                "(0x%x %s)\n",
                kr, mach_error_string(kr));
        return false;
    }

    // Verify what's actually at parentName in parent's namespace.
    mach_port_type_t parentType = 0;
    kr = mach_port_type(parentTaskPort, parentName, &parentType);
    if (kr != KERN_SUCCESS) {
        fprintf(stderr,
                "sidecar: mach_port_type(parent,0x%x) failed 0x%x %s\n",
                parentName, kr, mach_error_string(kr));
    } else {
        fprintf(stderr,
                "sidecar: parent name 0x%x type=0x%x (SEND=%d RECV=%d "
                "SENDONCE=%d DEAD=%d)\n",
                parentName, parentType,
                !!(parentType & MACH_PORT_TYPE_SEND),
                !!(parentType & MACH_PORT_TYPE_RECEIVE),
                !!(parentType & MACH_PORT_TYPE_SEND_ONCE),
                !!(parentType & MACH_PORT_TYPE_DEAD_NAME));
    }

    *outServicePort = servicePort;
    *outParentReqName = uint32_t(parentName);

    // ── Allocate the parent-owned reply port ────────────────────────────────
    // The stub names this port as msgh_local_port + MAKE_SEND_ONCE, so the
    // kernel hands the server a fresh SEND_ONCE right per call. Sidecar
    // replies via that send-once; the reply lands here in parent's space
    // and the stub's mach_msg(RCV) drains it.
    mach_port_name_t parentReplyName = MACH_PORT_NULL;
    kr = mach_port_allocate(parentTaskPort, MACH_PORT_RIGHT_RECEIVE,
                            &parentReplyName);
    if (kr != KERN_SUCCESS) {
        fprintf(stderr,
                "sidecar: mach_port_allocate(parent, RECEIVE) for reply "
                "failed (0x%x %s)\n",
                kr, mach_error_string(kr));
        return false;
    }
    fprintf(stderr,
            "sidecar: parent reply port name 0x%x (RECEIVE in parent)\n",
            parentReplyName);
    *outParentReplyName = uint32_t(parentReplyName);
    return true;
}

void runReceiveLoop(mach_port_t servicePort, mach_port_t parentTaskPort) {
    fprintf(stderr,
            "sidecar: receive loop starting on port 0x%x (parent task=0x%x)\n",
            servicePort, parentTaskPort);

    // Counter file we update on each successful receive — survives process
    // exit so post-mortem can verify whether the loop ever drained anything.
    auto bumpCounter = [](uint64_t hits, uint64_t lastErr) {
        FILE* f = fopen("/tmp/rosettax87_jit_sidecar_count", "w");
        if (!f) return;
        fprintf(f, "hits=%llu lastErr=0x%llx\n", hits, lastErr);
        fclose(f);
    };
    bumpCounter(0, 0);

    // Receive buffer aligned for the Mach IPC header.
    struct alignas(8) {
        uint8_t bytes[kRecvBufferSize];
    } buf;

    uint64_t hits = 0;
    for (;;) {
        mach_msg_header_t* hdr = reinterpret_cast<mach_msg_header_t*>(buf.bytes);
        hdr->msgh_local_port = servicePort;
        hdr->msgh_size = sizeof(buf);

        kern_return_t kr =
            mach_msg(hdr, MACH_RCV_MSG, 0, sizeof(buf), servicePort,
                     MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
        if (kr != KERN_SUCCESS) {
            fprintf(stderr,
                    "sidecar: mach_msg(RCV) returned 0x%x (%s)\n", kr,
                    mach_error_string(kr));
            bumpCounter(hits, uint64_t(kr));
            return;
        }

        hits++;
        bumpCounter(hits, 0);

        // M3 payload: header (24 B) + 5 × 8-byte args (40 B) = 64 B.
        // Reply path: stub provided a SEND_ONCE on msgh_remote_port (via
        // its MAKE_SEND_ONCE on the local-port disposition). We use it to
        // reply with msgh_id=0 (None — fall through to stock translate_insn)
        // for now. Once the Translator is wired, msgh_id=1 + body[0]=result
        // will let the stub return Some(N) directly without falling through.
        mach_port_t replyPort = hdr->msgh_remote_port;
        if (hdr->msgh_size >= 24 + 40 && hdr->msgh_id == 0x10000001 &&
            replyPort != MACH_PORT_NULL) {
            // Build reply.
            struct ReplyMsg {
                mach_msg_header_t hdr;
                uint64_t result;
            } reply{};
            reply.hdr.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_MOVE_SEND_ONCE, 0);
            reply.hdr.msgh_size = sizeof(reply);
            reply.hdr.msgh_remote_port = replyPort;
            reply.hdr.msgh_local_port = MACH_PORT_NULL;
            reply.hdr.msgh_id = 0;          // None — stub falls through
            reply.result = 0;

            kern_return_t kr_send = mach_msg(
                &reply.hdr, MACH_SEND_MSG, sizeof(reply), 0, MACH_PORT_NULL,
                MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
            if (kr_send != KERN_SUCCESS && hits <= 5) {
                fprintf(stderr,
                        "sidecar: #%llu reply send failed 0x%x %s\n",
                        hits, kr_send, mach_error_string(kr_send));
                // mach_msg consumes the SEND_ONCE on success; on failure
                // we must drop it ourselves.
                mach_port_deallocate(mach_task_self(), replyPort);
            }
            // mach_msg(SEND) on success consumed replyPort. Don't drop it.
        } else if (replyPort != MACH_PORT_NULL &&
                   (hdr->msgh_bits & MACH_MSGH_BITS_REMOTE_MASK) ==
                       MACH_MSG_TYPE_MOVE_SEND_ONCE) {
            // Other / malformed message — discard the SEND_ONCE.
            mach_port_deallocate(mach_task_self(), replyPort);
        }
    }
}

bool spawnReceiveThread(mach_port_t servicePort, mach_port_t parentTaskPort) {
    pthread_t thr;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    auto* args = new ThreadArgs{servicePort, parentTaskPort};
    int rc = pthread_create(&thr, &attr, threadEntry, args);
    pthread_attr_destroy(&attr);
    if (rc != 0) {
        delete args;
        fprintf(stderr, "sidecar: pthread_create failed (%d)\n", rc);
        return false;
    }
    return true;
}

}  // namespace sidecar
