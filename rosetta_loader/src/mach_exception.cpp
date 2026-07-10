#include "mach_exception.hpp"

#include <mach/mach.h>
#include <mach/mig_errors.h>
#include <sys/ptrace.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

// MIG-generated exception server (message ids 2405-2407, MACH_EXCEPTION_CODES).
// Generated into the build dir from $SDK/usr/include/mach/mach_exc.defs by the
// CMake mig step; declares mach_exc_server() and the catch_* routines below.
// The generated server is C; force C linkage on its declarations so our
// definitions below and the C-compiled server agree.
extern "C" {
#include "mach_excServer.h"
}

// The MIG server dispatches to plain-C catch_* functions by name. There is only
// ever one debugger session in this process, so a file-static pointer is set
// around each mach_exc_server() call (mirrors debugserver's g_message).
static MachExceptionSession* g_activeSession = nullptr;

extern "C" kern_return_t catch_mach_exception_raise(mach_port_t /*exception_port*/,
                                                    mach_port_t thread, mach_port_t task,
                                                    exception_type_t exception,
                                                    mach_exception_data_t code,
                                                    mach_msg_type_number_t codeCnt) {
    if (g_activeSession != nullptr) {
        // mach_exception_data_t is a 64-bit array; copy through int64_t so the
        // header stays independent of MIG typedefs.
        int64_t codes[2] = {0, 0};
        const int n = codeCnt < 2 ? static_cast<int>(codeCnt) : 2;
        for (int i = 0; i < n; ++i) {
            std::memcpy(&codes[i], &code[i], sizeof(int64_t));
        }
        g_activeSession->recordFromCatch(thread, task, exception, codes, static_cast<int>(codeCnt));
    }
    // Return success so MIG builds a resume reply; we send it (or override its
    // RetCode for forwarding) later, once we have acted on the stop.
    return KERN_SUCCESS;
}

// EXCEPTION_DEFAULT delivers via catch_mach_exception_raise only; the state
// variants are never invoked but must exist for the MIG server to link.
extern "C" kern_return_t catch_mach_exception_raise_state(
    mach_port_t /*exception_port*/, exception_type_t /*exception*/,
    const mach_exception_data_t /*code*/, mach_msg_type_number_t /*codeCnt*/, int* /*flavor*/,
    const thread_state_t /*old_state*/, mach_msg_type_number_t /*old_stateCnt*/,
    thread_state_t /*new_state*/, mach_msg_type_number_t* /*new_stateCnt*/) {
    return KERN_FAILURE;
}

extern "C" kern_return_t catch_mach_exception_raise_state_identity(
    mach_port_t /*exception_port*/, mach_port_t /*thread*/, mach_port_t /*task*/,
    exception_type_t /*exception*/, mach_exception_data_t /*code*/,
    mach_msg_type_number_t /*codeCnt*/, int* /*flavor*/, thread_state_t /*old_state*/,
    mach_msg_type_number_t /*old_stateCnt*/, thread_state_t /*new_state*/,
    mach_msg_type_number_t* /*new_stateCnt*/) {
    return KERN_FAILURE;
}

void MachExceptionSession::recordFromCatch(mach_port_t thread, mach_port_t task,
                                           exception_type_t exception, const int64_t* code,
                                           int codeCnt) {
    current_.type = exception;
    current_.thread = thread;
    current_.task = task;
    current_.codeCount = codeCnt < 2 ? codeCnt : 2;
    current_.codes[0] = current_.codeCount > 0 ? code[0] : 0;
    current_.codes[1] = current_.codeCount > 1 ? code[1] : 0;
    current_.valid = true;
}

bool MachExceptionSession::swapPorts(task_t task) {
    savedCount_ = kMaxExcPorts;
    // Claim ONLY EXC_SOFTWARE at task level: it carries the ptrace signal-stops
    // (attach/exec/SIGSTOP as EXC_SOFT_SIGNAL) that PT_ATTACHEXC folds into Mach
    // exceptions and that we must receive. We deliberately do NOT claim
    // EXC_BREAKPOINT here — that is libRosettaRuntime's task-level port for its
    // JIT BRKs, and displacing-then-restoring it races the runtime (the
    // post-detach SIGTRAP / exit=133 flake). Our own planted BRK is caught at
    // THREAD level instead (installThreadBreakpoint). Taking EXC_MASK_ALL would
    // likewise disturb handlers for faults we never plant (e.g. EXC_BAD_ACCESS).
    kern_return_t kr = task_swap_exception_ports(
        task, EXC_MASK_SOFTWARE, exceptionPort_, EXCEPTION_DEFAULT | MACH_EXCEPTION_CODES,
        THREAD_STATE_NONE, savedMasks_, &savedCount_, savedPorts_, savedBehaviors_, savedFlavors_);
    if (kr != KERN_SUCCESS) {
        fprintf(stdout, "MachExc: task_swap_exception_ports failed (0x%x: %s)\n", kr,
                mach_error_string(kr));
        savedCount_ = 0;
        return false;
    }
    task_ = task;
    haveSaved_ = true;
    return true;
}

bool MachExceptionSession::install(pid_t pid, task_t task) {
    pid_ = pid;

    mach_port_t self = mach_task_self();
    kern_return_t kr = mach_port_allocate(self, MACH_PORT_RIGHT_RECEIVE, &exceptionPort_);
    if (kr != KERN_SUCCESS) {
        fprintf(stdout, "MachExc: mach_port_allocate failed (0x%x: %s)\n", kr,
                mach_error_string(kr));
        return false;
    }
    kr = mach_port_insert_right(self, exceptionPort_, exceptionPort_, MACH_MSG_TYPE_MAKE_SEND);
    if (kr != KERN_SUCCESS) {
        fprintf(stdout, "MachExc: mach_port_insert_right failed (0x%x: %s)\n", kr,
                mach_error_string(kr));
        return false;
    }
    return swapPorts(task);
}

bool MachExceptionSession::reinstall(task_t task) {
    if (exceptionPort_ == MACH_PORT_NULL) {
        fprintf(stdout, "MachExc: reinstall before install\n");
        return false;
    }
    task_ = task;
    // If exec preserved our registration, don't swap again: task_swap would
    // then record our own port as the "previous" handler and corrupt the
    // restore. Only re-register when exec reset the task's exception ports.
    if (verifyInstalled()) {
        return true;
    }
    return swapPorts(task);
}

bool MachExceptionSession::verifyInstalled() const {
    exception_mask_t masks[kMaxExcPorts];
    mach_port_t ports[kMaxExcPorts];
    exception_behavior_t behaviors[kMaxExcPorts];
    thread_state_flavor_t flavors[kMaxExcPorts];
    mach_msg_type_number_t count = kMaxExcPorts;
    kern_return_t kr = task_get_exception_ports(task_, EXC_MASK_ALL, masks, &count, ports, behaviors,
                                                flavors);
    if (kr != KERN_SUCCESS) {
        fprintf(stdout, "MachExc: task_get_exception_ports failed (0x%x: %s)\n", kr,
                mach_error_string(kr));
        return false;
    }
    for (mach_msg_type_number_t i = 0; i < count; ++i) {
        if (ports[i] == exceptionPort_) {
            return true;
        }
    }
    // Not an error on its own: reinstall() uses this to decide whether exec
    // reset the ports. Callers that require our port log their own failure.
    return false;
}

bool MachExceptionSession::installThreadBreakpoint(thread_act_t thread) {
    if (exceptionPort_ == MACH_PORT_NULL) {
        fprintf(stdout, "MachExc: installThreadBreakpoint before install\n");
        return false;
    }
    savedThreadCount_ = kMaxExcPorts;
    kern_return_t kr = thread_swap_exception_ports(
        thread, EXC_MASK_BREAKPOINT, exceptionPort_, EXCEPTION_DEFAULT | MACH_EXCEPTION_CODES,
        THREAD_STATE_NONE, savedThreadMasks_, &savedThreadCount_, savedThreadPorts_,
        savedThreadBehaviors_, savedThreadFlavors_);
    if (kr != KERN_SUCCESS) {
        fprintf(stdout, "MachExc: thread_swap_exception_ports failed (0x%x: %s)\n", kr,
                mach_error_string(kr));
        savedThreadCount_ = 0;
        return false;
    }
    bpThread_ = thread;
    haveThreadBp_ = true;
    return true;
}

void MachExceptionSession::removeThreadBreakpoint() {
    if (!haveThreadBp_) {
        return;
    }
    // Write back whatever the thread had before. A freshly-exec'd thread has no
    // EXC_BREAKPOINT handler, so savedThreadCount_ is typically 0 and this loop
    // restores the (empty) default — clearing our registration. We never
    // touched the task-level handler, so there is nothing else to undo.
    for (mach_msg_type_number_t i = 0; i < savedThreadCount_; ++i) {
        thread_set_exception_ports(bpThread_, savedThreadMasks_[i], savedThreadPorts_[i],
                                   savedThreadBehaviors_[i], savedThreadFlavors_[i]);
    }
    if (savedThreadCount_ == 0) {
        // Nothing was registered before us: explicitly drop our port so the
        // thread falls through to the task-level (Rosetta) handler again.
        thread_set_exception_ports(bpThread_, EXC_MASK_BREAKPOINT, MACH_PORT_NULL,
                                   EXCEPTION_DEFAULT | MACH_EXCEPTION_CODES, THREAD_STATE_NONE);
    }
    haveThreadBp_ = false;
    savedThreadCount_ = 0;
    bpThread_ = MACH_PORT_NULL;
}

MachExceptionSession::Event MachExceptionSession::waitForEvent(uint32_t timeoutMs) {
    current_ = Event{};
    std::memset(&excMsg_, 0, sizeof(excMsg_));
    std::memset(&replyMsg_, 0, sizeof(replyMsg_));

    kern_return_t kr = mach_msg(&excMsg_.hdr, MACH_RCV_MSG | MACH_RCV_TIMEOUT, 0, sizeof(excMsg_),
                                exceptionPort_, timeoutMs, MACH_PORT_NULL);
    if (kr != KERN_SUCCESS) {
        if (kr == MACH_RCV_TIMED_OUT) {
            fprintf(stdout, "MachExc: timed out waiting for exception (%u ms)\n", timeoutMs);
        } else {
            fprintf(stdout, "MachExc: mach_msg receive failed (0x%x: %s)\n", kr,
                    mach_error_string(kr));
        }
        return current_;  // valid == false
    }

    // Decode + build the reply. mach_exc_server invokes catch_mach_exception_raise
    // (which fills current_ via recordFromCatch) and writes the reply into
    // replyMsg_ without sending it — so the tracee stays stopped.
    g_activeSession = this;
    boolean_t handled = mach_exc_server(&excMsg_.hdr, &replyMsg_.hdr);
    g_activeSession = nullptr;
    if (!handled) {
        fprintf(stdout, "MachExc: mach_exc_server did not handle the message (id=%d)\n",
                excMsg_.hdr.msgh_id);
        current_.valid = false;
        return current_;
    }
    haveHeld_ = true;
    return current_;
}

bool MachExceptionSession::sendReply() {
    kern_return_t kr = mach_msg(&replyMsg_.hdr, MACH_SEND_MSG, replyMsg_.hdr.msgh_size, 0,
                                MACH_PORT_NULL, MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
    if (kr != KERN_SUCCESS) {
        fprintf(stdout, "MachExc: mach_msg reply send failed (0x%x: %s)\n", kr,
                mach_error_string(kr));
        return false;
    }
    haveHeld_ = false;
    return true;
}

bool MachExceptionSession::reply(int signalToForward) {
    if (!haveHeld_) {
        return true;  // nothing outstanding
    }
    // For a soft-signal stop, PT_THUPDATE selects the signal delivered to the
    // tracee on resume (0 suppresses it — the attach SIGSTOP and exec SIGTRAP
    // are pure stops we never want re-delivered). Breakpoint stops carry no
    // signal, so this is skipped for them.
    if (current_.softSignal() != 0) {
        errno = 0;
        if (ptrace(PT_THUPDATE, pid_, reinterpret_cast<caddr_t>(static_cast<uintptr_t>(
                                           current_.thread)),
                   signalToForward) != 0) {
            fprintf(stdout, "MachExc: PT_THUPDATE failed: %s\n", strerror(errno));
        }
    }
    return sendReply();
}

bool MachExceptionSession::release() {
    if (!haveHeld_) {
        return true;
    }
    return sendReply();
}

bool MachExceptionSession::forward() {
    if (!haveHeld_) {
        return true;
    }
    // Flip the reply RetCode so the kernel treats the exception as unhandled by
    // us and applies its default disposition. mach_exception_raise's reply is a
    // mig_reply_error_t (Head, NDR, RetCode).
    auto* err = reinterpret_cast<mig_reply_error_t*>(&replyMsg_);
    err->RetCode = KERN_FAILURE;
    fprintf(stdout, "MachExc: forwarding unhandled exception type=%d to default disposition\n",
            current_.type);
    return sendReply();
}

void MachExceptionSession::restoreAndTearDown() {
    // Drop any thread-level EXC_BREAKPOINT registration first (normally already
    // removed right after the BRK; this is a safety net for error paths).
    removeThreadBreakpoint();
    if (haveSaved_ && task_ != TASK_NULL) {
        for (mach_msg_type_number_t i = 0; i < savedCount_; ++i) {
            // We now only ever hold EXC_MASK_SOFTWARE at task level, but keep
            // the conditional restore: write the saved handler back ONLY if we
            // are still the registered handler for this mask. If libRosettaRuntime
            // has installed its own handler over ours since the swap, blindly
            // writing back our stale (empty) snapshot would clobber it; leave
            // the runtime's handler in place instead.
            exception_mask_t curMasks[kMaxExcPorts];
            mach_port_t curPorts[kMaxExcPorts];
            exception_behavior_t curBehaviors[kMaxExcPorts];
            thread_state_flavor_t curFlavors[kMaxExcPorts];
            mach_msg_type_number_t curCount = kMaxExcPorts;
            bool stillOurs = false;
            if (task_get_exception_ports(task_, savedMasks_[i], curMasks, &curCount, curPorts,
                                         curBehaviors, curFlavors) == KERN_SUCCESS) {
                for (mach_msg_type_number_t j = 0; j < curCount; ++j) {
                    if (curPorts[j] == exceptionPort_) {
                        stillOurs = true;
                        break;
                    }
                }
            }
            if (stillOurs) {
                task_set_exception_ports(task_, savedMasks_[i], savedPorts_[i], savedBehaviors_[i],
                                         savedFlavors_[i]);
            }
        }
        haveSaved_ = false;
        savedCount_ = 0;
    }
    if (exceptionPort_ != MACH_PORT_NULL) {
        mach_port_deallocate(mach_task_self(), exceptionPort_);
        exceptionPort_ = MACH_PORT_NULL;
    }
}
