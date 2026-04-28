#include "sidecar.hpp"

#include <cstdio>
#include <pthread.h>

#include <mach/mach.h>
#include <mach/mach_port.h>

namespace sidecar {

namespace {

// Largest message we expect to receive in M2 — header only is 24 bytes,
// plus trailer (mach_msg_trailer_t is at least 8 bytes), plus future
// payload. 4 KB is generous.
constexpr size_t kRecvBufferSize = 4096;

// Receive-loop worker thread entry point.
void* threadEntry(void* arg) {
    mach_port_t servicePort = mach_port_t(uintptr_t(arg));
    runReceiveLoop(servicePort);
    return nullptr;
}

}  // namespace

bool installPortInParent(mach_port_t parentTaskPort,
                         mach_port_t* outServicePort,
                         uint32_t* outParentNameRef) {
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
    *outParentNameRef = uint32_t(parentName);
    return true;
}

void runReceiveLoop(mach_port_t servicePort) {
    fprintf(stderr, "sidecar: receive loop starting on port 0x%x\n", servicePort);

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

        // M2 dummy: log first few and drop. M3 will dispatch real work.
        if (hits <= 5) {
            fprintf(stderr,
                    "sidecar: received #%llu msgh_id=0x%x size=%u remote=0x%x\n",
                    hits, hdr->msgh_id, hdr->msgh_size, hdr->msgh_remote_port);
        }

        if (hdr->msgh_remote_port != MACH_PORT_NULL &&
            (hdr->msgh_bits & MACH_MSGH_BITS_REMOTE_MASK) ==
                MACH_MSG_TYPE_MOVE_SEND_ONCE) {
            mach_port_deallocate(mach_task_self(), hdr->msgh_remote_port);
        }
    }
}

bool spawnReceiveThread(mach_port_t servicePort) {
    pthread_t thr;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    int rc = pthread_create(&thr, &attr, threadEntry,
                            reinterpret_cast<void*>(uintptr_t(servicePort)));
    pthread_attr_destroy(&attr);
    if (rc != 0) {
        fprintf(stderr, "sidecar: pthread_create failed (%d)\n", rc);
        return false;
    }
    return true;
}

}  // namespace sidecar
