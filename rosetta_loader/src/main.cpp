#include <mach-o/dyld.h>
#include <mach-o/dyld_images.h>
#include <mach/mach_vm.h>
#include <mach/vm_attributes.h>
#include <sched.h>
#include <servers/bootstrap.h>
#include <sys/event.h>
#include <sys/mman.h>
#include <sys/ptrace.h>
#include <sys/wait.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <exception>
#include <map>
#include <numbers>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "coop_proto.h"
#include "mach_exception.hpp"
#include "offset_finder.hpp"
#include "rosetta_core/Config.h"
#include "rosetta_core/ConfigEnv.h"
#include "rosetta_core/CoreConfig.h"
#include "rosetta_core/ProfileRuntime.h"
#include "rosetta_core/RosettaCore.h"
#include "rosetta_core/TranscendentalHelper.h"
#include "rosetta_core/TranslationResult.h"
#include "sidecar.hpp"
#include "stub_asm.hpp"
#include "types.h"

const char* logsEnabled = nullptr;

#define VERBOSE_LOG(fmt, ...)           \
    do {                                \
        if (logsEnabled) {              \
            printf(fmt, ##__VA_ARGS__); \
        }                               \
    } while (0)

using DyldProcessInfo = struct dyld_process_info_base*;

extern "C" DyldProcessInfo _dyld_process_info_create(task_t task, uint64_t timestamp,
                                                     kern_return_t* kernelError);
extern "C" void _dyld_process_info_for_each_image(DyldProcessInfo info,
                                                  void (^callback)(uint64_t machHeaderAddress,
                                                                   const uuid_t uuid,
                                                                   const char* path));
extern "C" void _dyld_process_info_release(DyldProcessInfo info);

// bootstrap_register2 is a private libSystem symbol (the non-deprecated way to
// publish a dynamic service name). It isn't in <servers/bootstrap.h>, so declare
// it like wine's server/mach.c does.
extern "C" kern_return_t bootstrap_register2(mach_port_t bp, name_t service_name, mach_port_t sp,
                                             uint64_t flags);

class MuhDebugger {
private:
    static const uint32_t AARCH64_BREAKPOINT;  // just declare here

    pid_t childPid_ = -1;
    task_t taskPort_ = MACH_PORT_NULL;
    std::map<uint64_t, uint32_t> breakpoints_;  // addr -> original instruction

    // Debug events arrive as Mach exception messages on a port we own (see
    // mach_exception.hpp). While an event is held (unreplied) the tracee is
    // stopped; replying resumes it. This replaces the PT_ATTACH +
    // waitpid/WSTOPSIG signal path, whose EXC_BREAKPOINT delivery raced
    // libRosettaRuntime's own handler and leaked our planted BRK to the parent
    // as a fatal SIGTRAP (CI exit=133 — see
    // docs/investigations/ci-flaky-sigtrap-attach.md).
    MachExceptionSession exc_;
    MachExceptionSession::Event lastEvent_{};

    // Receive the next event, suppressing soft-signal stops other than
    // expectedSignal (0 = return on any stop), mirroring the old
    // suppress-and-continue loop. An EXC_BREAKPOINT is always a stop. An
    // unexpected non-signal exception is a genuine fault: forward it to the
    // task's default disposition and fail.
    bool waitForStopped(int expectedSignal = 0) {
        while (true) {
            lastEvent_ = exc_.waitForEvent();
            if (!lastEvent_.valid) {
                return false;
            }
            if (lastEvent_.isBreakpoint()) {
                VERBOSE_LOG("Stopped at EXC_BREAKPOINT\n");
                return true;
            }
            int sig = lastEvent_.softSignal();
            if (sig != 0) {
                VERBOSE_LOG("Process stopped signal=%d\n", sig);
                if (expectedSignal == 0 || sig == expectedSignal) {
                    return true;
                }
                VERBOSE_LOG("Suppressing unexpected signal %d (waiting for %d)\n", sig,
                            expectedSignal);
                if (!exc_.reply(0)) {
                    return false;
                }
                continue;
            }
            fprintf(stdout, "Unexpected exception type=%d during setup; forwarding\n",
                    lastEvent_.type);
            exc_.forward();
            return false;
        }
    }

    // Wait specifically for the planted BRK (EXC_BREAKPOINT), suppressing any
    // soft signals that arrive first. The old code caught the BRK as a SIGTRAP
    // via ptrace; under PT_ATTACHEXC a hardware breakpoint is delivered as
    // EXC_BREAKPOINT directly.
    bool waitForBreakpoint() {
        while (true) {
            lastEvent_ = exc_.waitForEvent();
            if (!lastEvent_.valid) {
                return false;
            }
            if (lastEvent_.isBreakpoint()) {
                VERBOSE_LOG("Stopped at EXC_BREAKPOINT\n");
                return true;
            }
            int sig = lastEvent_.softSignal();
            if (sig != 0) {
                VERBOSE_LOG("Suppressing signal %d while waiting for breakpoint\n", sig);
                if (!exc_.reply(0)) {
                    return false;
                }
                continue;
            }
            fprintf(stdout, "Unexpected exception type=%d before breakpoint; forwarding\n",
                    lastEvent_.type);
            exc_.forward();
            return false;
        }
    }

public:
    ~MuhDebugger() {
        if (taskPort_ != MACH_PORT_NULL) {
            mach_port_deallocate(mach_task_self(), taskPort_);
        }
    }

    [[nodiscard]] task_t taskPort() const { return taskPort_; }
    [[nodiscard]] mach_port_t stoppedThread() const { return lastEvent_.thread; }

    // Arm/disarm the thread-level EXC_BREAKPOINT catcher on the currently
    // stopped thread — the one that will execute (and later re-execute) the
    // planted BRK. Must be armed before continuing into the BRK and disarmed
    // after the original instruction is restored. Catching the BRK at thread
    // level keeps us off libRosettaRuntime's task-level EXC_BREAKPOINT port.
    bool armThreadBreakpoint() { return exc_.installThreadBreakpoint(lastEvent_.thread); }
    void disarmThreadBreakpoint() { exc_.removeThreadBreakpoint(); }

    // ── Cooperative mode (no task_for_pid / no ptrace) ──────────────────────
    // Adopt a task port the tracee voluntarily handed over via the bootstrap
    // handshake. Takes ownership of the send right (released by ~MuhDebugger).
    // No ptrace attach, no exec-stop: the tracee is already running and is held
    // quiescent by blocking in the handshake until we reply.
    bool adopt(pid_t pid, task_t task) {
        childPid_ = pid;
        taskPort_ = task;
        return exc_.initPortOnly(pid, task);
    }

    // Like findRuntime(), but disambiguates by content: return the first
    // executable MH_MAGIC_64 region whose bytes at [matchOff, matchOff+matchLen)
    // equal `match`. In a fully-initialized process (cooperative mode) there are
    // several MH_MAGIC regions besides libRosettaRuntime, so the plain
    // first-match heuristic in findRuntime() picks the wrong one; the
    // exports_fetch instruction signature uniquely identifies the runtime.
    [[nodiscard]] auto findRuntimeMatching(uint64_t matchOff, const uint8_t* match,
                                           size_t matchLen) const -> uintptr_t {
        if (matchLen == 0 || matchLen > 64) {
            return 0;
        }
        mach_vm_address_t address = 0;
        mach_vm_size_t size = 0;
        vm_region_basic_info_data_64_t info;
        mach_port_t objectName = MACH_PORT_NULL;
        while (true) {
            mach_msg_type_number_t count = VM_REGION_BASIC_INFO_COUNT_64;
            if (mach_vm_region(taskPort_, &address, &size, VM_REGION_BASIC_INFO_64,
                               reinterpret_cast<vm_region_info_t>(&info), &count,
                               &objectName) != KERN_SUCCESS) {
                break;
            }
            if (info.protection & (VM_PROT_EXECUTE | VM_PROT_READ)) {
                uint32_t magic = 0;
                if (readMemory(address, &magic, sizeof(magic)) && magic == MH_MAGIC_64) {
                    uint8_t buf[64];
                    if (readMemory(address + matchOff, buf, matchLen) &&
                        memcmp(buf, match, matchLen) == 0) {
                        return address;
                    }
                }
            }
            address += size;
        }
        return 0;
    }

    // Scan the tracee's executable memory for a byte pattern; return the live
    // address of the first match, or 0. Locates translate_insn by its prologue
    // signature without the Exports struct (X19) — works whether the tracee is
    // stopped at exec (default) or blocked in the handshake (cooperative).
    [[nodiscard]] uint64_t scanForPattern(const uint8_t* pat, size_t patLen) const {
        if (patLen == 0) {
            return 0;
        }
        mach_vm_address_t address = 0;
        mach_vm_size_t size = 0;
        vm_region_basic_info_data_64_t info;
        mach_port_t objectName = MACH_PORT_NULL;
        std::vector<uint8_t> buf;
        while (true) {
            mach_msg_type_number_t count = VM_REGION_BASIC_INFO_COUNT_64;
            if (mach_vm_region(taskPort_, &address, &size, VM_REGION_BASIC_INFO_64,
                               reinterpret_cast<vm_region_info_t>(&info), &count,
                               &objectName) != KERN_SUCCESS) {
                break;
            }
            const uint64_t regionStart = address;
            const uint64_t regionSize = size;
            address += size;  // advance before any continue
            if ((info.protection & VM_PROT_EXECUTE) == 0) {
                continue;
            }
            if (regionSize == 0 || regionSize > (64ULL << 20)) {
                continue;  // skip absent/implausibly-large executable regions
            }
            buf.resize(regionSize);
            if (!readMemory(regionStart, buf.data(), regionSize)) {
                continue;
            }
            const auto it = std::search(buf.begin(), buf.end(), pat, pat + patLen);
            if (it != buf.end()) {
                return regionStart + static_cast<uint64_t>(std::distance(buf.begin(), it));
            }
        }
        return 0;
    }

    bool attach(pid_t pid) {
        childPid_ = pid;
        VERBOSE_LOG("Attempting to attach to %d\n", childPid_);

        // Grab the task port up front (pre-exec) so we can install our Mach
        // exception port before attaching. The loader already holds the
        // debugger entitlement / runs as root, and already task_for_pid's the
        // harder post-exec Rosetta process below.
        if (task_for_pid(mach_task_self(), childPid_, &taskPort_) != KERN_SUCCESS) {
            fprintf(stdout, "attach: task_for_pid(%d) failed\n", childPid_);
            return false;
        }
        // Install the exception port BEFORE attaching so every debug event is
        // routed to us and can never fall through to the parent's fatal signal
        // disposition.
        if (!exc_.install(childPid_, taskPort_)) {
            return false;
        }
        // PT_ATTACHEXC: a ptrace attach (sibling of PT_ATTACH) that delivers
        // debug events as Mach exceptions to our port instead of as BSD
        // signals. Sends SIGSTOP, delivered as EXC_SOFT_SIGNAL(SIGSTOP).
        if (ptrace(PT_ATTACHEXC, childPid_, nullptr, 0) == -1) {
            fprintf(stdout, "ptrace(PT_ATTACHEXC): %s\n", strerror(errno));
            return false;
        }
        if (!waitForStopped()) {  // consume the attach-stop
            return false;
        }
        VERBOSE_LOG("Attached to %d (attach-stop)\n", childPid_);
        return true;
    }

    // Resume from the attach-stop, let the parent execv, and stop at the exec
    // SIGTRAP. execve can hand back a different task port and resets thread
    // state, so re-fetch the task port and make sure our exception port covers
    // the post-exec task before the breakpoint window.
    bool waitForExecStop() {
        if (!exc_.reply(0)) {  // resume, suppressing SIGSTOP
            fprintf(stdout, "waitForExecStop: failed to resume from attach-stop\n");
            return false;
        }
        if (!waitForStopped(SIGTRAP)) {
            return false;
        }
        VERBOSE_LOG("Program stopped due to execv\n");

        if (task_for_pid(mach_task_self(), childPid_, &taskPort_) != KERN_SUCCESS) {
            fprintf(stdout, "Failed to get task port for pid %d\n", childPid_);
            return false;
        }
        if (!exc_.reinstall(taskPort_) || !exc_.verifyInstalled()) {
            fprintf(stdout, "Failed to (re)install exception port after exec\n");
            return false;
        }
        VERBOSE_LOG("Started debugging process %d using port %d\n", childPid_, taskPort_);
        return true;
    }

    bool continueExecution() {
        if (!exc_.reply(0)) {  // resume from the held stop
            fprintf(stdout, "continueExecution: resume failed\n");
            return false;
        }
        VERBOSE_LOG("continueExecution...\n");
        // ARM64 BRK does not advance PC; removeBreakpoint restores the original
        // instruction, so replying (in detach) later re-executes it correctly.
        return waitForBreakpoint();
    }

    bool detach() {
        // PT_DETACH requires the tracee to be in a BSD signal-stop. Under
        // PT_ATTACHEXC our stops arrive as Mach exceptions (the BRK is a bare
        // EXC_BREAKPOINT carrying no signal), which PT_DETACH rejects with
        // EBUSY. So synthesize a SIGSTOP job-control stop to detach from — the
        // approach debugserver takes in MachProcess::DoSIGSTOP/Detach. SIGSTOP
        // cannot be caught or ignored, so the process stays stopped until
        // PT_DETACH continues it.
        // Drive the tracee into a SIGSTOP job-control stop and hold it. SIGSTOP
        // can't be caught or ignored, and PT_DETACH only accepts a held
        // signal-stop like this one — a bare EXC_BREAKPOINT or a running
        // process both give EBUSY.
        kill(childPid_, SIGSTOP);
        if (!exc_.reply(0)) {  // release the held stop; run into the SIGSTOP
            return false;
        }
        if (!waitForStopped(SIGSTOP)) {
            fprintf(stdout, "detach: failed to reach SIGSTOP stop\n");
            return false;
        }
        // Restore the task's exception ports, PT_DETACH from the held SIGSTOP
        // stop, then release the exception (unblocking the thread). Order
        // matters: PT_DETACH must run while the SIGSTOP exception is still held,
        // and the release must run after (ptrace is no longer valid post-detach).
        exc_.restoreAndTearDown();
        bool ok = true;
        if (ptrace(PT_DETACH, childPid_, reinterpret_cast<caddr_t>(1), 0) < 0) {
            fprintf(stdout, "ptrace(PT_DETACH): %s\n", strerror(errno));
            ok = false;
        }
        exc_.release();
        if (ok) {
            VERBOSE_LOG("Debugger detached.\n");
        }
        return ok;
    }

    bool setBreakpoint(uint64_t address) {
        // Verify address is in valid range
        if (address >= MACH_VM_MAX_ADDRESS) {
            fprintf(stdout, "Invalid address 0x%llx\n", address);
            return false;
        }

        // Read the original instruction
        uint32_t original;
        if (!readMemory(address, &original, sizeof(uint32_t))) {
            fprintf(stdout, "Failed to read memory at 0x%llx\n", address);
            return false;
        }

        // First, try to adjust memory protection
        if (!adjustMemoryProtection(address, VM_PROT_READ | VM_PROT_WRITE | VM_PROT_COPY,
                                    sizeof(uint32_t))) {
            return false;
        }

        // Write breakpoint instruction
        if (!writeMemory(address, &AARCH64_BREAKPOINT, sizeof(uint32_t))) {
            fprintf(stdout, "Failed to write breakpoint at 0x%llx\n", address);
            return false;
        }

        if (!adjustMemoryProtection(address, VM_PROT_READ | VM_PROT_EXECUTE, sizeof(uint32_t))) {
            return false;
        }

        breakpoints_[address] = original;
        VERBOSE_LOG("Breakpoint set at address 0x%llx\n", address);
        return true;
    }

    bool removeBreakpoint(uint64_t address) {
        auto it = breakpoints_.find(address);
        if (it == breakpoints_.end()) {
            fprintf(stdout, "No breakpoint found at address 0x%llx\n", address);
            return false;
        }

        // First, try to adjust memory protection
        if (!adjustMemoryProtection(address, VM_PROT_READ | VM_PROT_WRITE, sizeof(uint32_t))) {
            return false;
        }

        // Restore original instruction
        if (!writeMemory(address, &it->second, sizeof(uint32_t))) {
            fprintf(stdout, "Failed to restore original instruction at 0x%llx\n", address);
            return false;
        }

        if (!adjustMemoryProtection(address, VM_PROT_READ | VM_PROT_EXECUTE, sizeof(uint32_t))) {
            return false;
        }
        breakpoints_.erase(it);
        VERBOSE_LOG("Breakpoint removed from address 0x%llx\n", address);
        return true;
    }

    enum Register : std::uint8_t {
        X0,
        X1,
        X2,
        X3,
        X4,
        X5,
        X6,
        X7,
        X8,
        X9,
        X10,
        X11,
        X12,
        X13,
        X14,
        X15,
        X16,
        X17,
        X18,
        X19,
        X20,
        X21,
        X22,
        X23,
        X24,
        X25,
        X26,
        X27,
        X28,
        FP,
        LR,
        SP,
        PC,
        CPSR
    };

    // Read a register from a specific thread port. Used to read X19 from the
    // exact thread reported by the breakpoint exception message — correct even
    // if libRosettaRuntime has spawned other threads by then (task_threads[0]
    // is not guaranteed to be the one that hit the BRK).
    [[nodiscard]] uint64_t readRegister(mach_port_t thread, Register reg) const {
        arm_thread_state64_t state;
        mach_msg_type_number_t count = ARM_THREAD_STATE64_COUNT;
        kern_return_t kr = thread_get_state(thread, ARM_THREAD_STATE64,
                                            reinterpret_cast<thread_state_t>(&state), &count);
        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to get thread state (error 0x%x: %s)\n", kr,
                    mach_error_string(kr));
            return 0;
        }

        if (reg >= X0 && reg <= X28) {
            return state.__x[reg];
        }
        switch (reg) {
            case FP:
                return state.__fp;
            case LR:
                return state.__lr;
            case SP:
                return state.__sp;
            case PC:
                return state.__pc;
            case CPSR:
                return state.__cpsr;
            default:
                fprintf(stdout, "Invalid register\n");
                return 0;
        }
    }

    [[nodiscard]] bool setRegister(Register reg, uint64_t value) const {
        thread_act_port_array_t threadList;
        mach_msg_type_number_t threadCount;

        kern_return_t kr = task_threads(taskPort_, &threadList, &threadCount);
        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to get threads (error 0x%x: %s)\n", kr, mach_error_string(kr));
            return false;
        }

        arm_thread_state64_t state;
        mach_msg_type_number_t count = ARM_THREAD_STATE64_COUNT;
        kr = thread_get_state(threadList[0], ARM_THREAD_STATE64,
                              reinterpret_cast<thread_state_t>(&state), &count);

        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to get thread state (error 0x%x: %s)\n", kr,
                    mach_error_string(kr));
            return false;
        }

        if (reg >= X0 && reg <= X28) {
            state.__x[reg] = value;
        } else {
            switch (reg) {
                case FP:
                    state.__fp = value;
                    break;
                case LR:
                    state.__lr = value;
                    break;
                case SP:
                    state.__sp = value;
                    break;
                case PC:
                    state.__pc = value;
                    break;
                case CPSR:
                    state.__cpsr = value;
                    break;
                default: {
                    fprintf(stdout, "Invalid register\n");
                    return false;
                }
            }
        }

        kr = thread_set_state(threadList[0], ARM_THREAD_STATE64,
                              reinterpret_cast<thread_state_t>(&state), ARM_THREAD_STATE64_COUNT);
        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to set thread state (error 0x%x: %s)\n", kr,
                    mach_error_string(kr));
            return false;
        }

        // Cleanup
        for (uint i = 0; i < threadCount; i++) {
            mach_port_deallocate(mach_task_self(), threadList[i]);
        }
        vm_deallocate(mach_task_self(), reinterpret_cast<vm_address_t>(threadList),
                      sizeof(thread_t) * threadCount);

        return true;
    }

    [[nodiscard]] bool adjustMemoryProtection(uint64_t address, vm_prot_t protection,
                                              mach_vm_size_t size) const {
        // 4KB page size in rosetta process
        vm_size_t pageSize = 0x1000;
        // align to page boundary
        mach_vm_address_t region = address & ~(pageSize - 1);
        size = ((address + size + pageSize - 1) & ~(pageSize - 1)) - region;

        VERBOSE_LOG("Adjusting memory protection at 0x%llx - 0x%llx\n", (uint64_t)region,
                    (uint64_t)(region + size));

        kern_return_t kr = mach_vm_protect(taskPort_, region, size, false, protection);
        if (kr != KERN_SUCCESS) {
            fprintf(stdout,
                    "Failed to adjust memory protection at 0x%llx - 0x%llx (error 0x%x: %s)\n",
                    static_cast<uint64_t>(region), (region + size), kr, mach_error_string(kr));
            return false;
        }
        return true;
    }

    bool readMemory(uint64_t address, void* buffer, size_t size) const {
        mach_vm_size_t readSize;

        kern_return_t kr = mach_vm_read_overwrite(
            taskPort_, address, size, reinterpret_cast<mach_vm_address_t>(buffer), &readSize);

        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to read memory at 0x%llx (error 0x%x: %s)\n", address, kr,
                    mach_error_string(kr));
            return false;
        }

        return readSize == size;
    }

    bool writeMemory(uint64_t address, const void* buffer, size_t size) const {
        kern_return_t kr =
            mach_vm_write(taskPort_, address, reinterpret_cast<vm_offset_t>(buffer), size);

        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to write memory at 0x%llx (error 0x%x: %s)\n", address, kr,
                    mach_error_string(kr));
            return false;
        }

        // Flush the target's instruction cache for the written range. ARM64 I/D
        // caches are not coherent: mach_vm_write lands in the D-cache path only,
        // so a core that has already executed (and cached) this line keeps
        // running the stale bytes. That is fatal for the one breakpoint we plant
        // at exports_fetch: we plant a BRK, catch it, restore the original
        // instruction, then RESUME the held thread — which re-fetches the same
        // PC (ARM64 BRK does not advance PC). If the stale BRK is still in the
        // I-cache at that re-fetch, it re-traps after we have detached and torn
        // down our exception handler → an unhandled SIGTRAP kills the tracee
        // (the flaky post-detach exit=133). Flushing here makes the restore (and
        // every M2 __TEXT patch) coherent, exactly as lldb's debugserver does
        // after writing to a tracee. Best-effort: a failure only regresses to
        // the old behaviour, so log and continue.
        vm_machine_attribute_val_t flush = MATTR_VAL_ICACHE_FLUSH;
        kern_return_t fkr = mach_vm_machine_attribute(taskPort_, address, size, MATTR_CACHE, &flush);
        if (fkr != KERN_SUCCESS) {
            fprintf(stdout, "Warning: i-cache flush at 0x%llx failed (error 0x%x: %s)\n", address,
                    fkr, mach_error_string(fkr));
        }

        return true;
    }

    bool copyThreadState(arm_thread_state64_t& state) const {
        thread_act_port_array_t threadList;
        mach_msg_type_number_t threadCount;

        kern_return_t kr = task_threads(taskPort_, &threadList, &threadCount);
        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to get threads (error 0x%x: %s)\n", kr, mach_error_string(kr));
            return false;
        }

        mach_msg_type_number_t count = ARM_THREAD_STATE64_COUNT;
        kr = thread_get_state(threadList[0], ARM_THREAD_STATE64,
                              reinterpret_cast<thread_state_t>(&state), &count);

        // Cleanup
        for (uint i = 0; i < threadCount; i++) {
            mach_port_deallocate(mach_task_self(), threadList[i]);
        }
        vm_deallocate(mach_task_self(), reinterpret_cast<vm_address_t>(threadList),
                      sizeof(thread_t) * threadCount);

        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to get thread state (error 0x%x: %s)\n", kr,
                    mach_error_string(kr));
            return false;
        }

        return true;
    }

    [[nodiscard]] bool restoreThreadState(const arm_thread_state64_t& state) const {
        thread_act_port_array_t threadList;
        mach_msg_type_number_t threadCount;

        kern_return_t kr = task_threads(taskPort_, &threadList, &threadCount);
        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to get threads (error 0x%x: %s)\n", kr, mach_error_string(kr));
            return false;
        }

        kr = thread_set_state(
            threadList[0], ARM_THREAD_STATE64,
            reinterpret_cast<thread_state_t>(const_cast<arm_thread_state64_t*>(&state)),
            ARM_THREAD_STATE64_COUNT);

        // Cleanup
        for (uint i = 0; i < threadCount; i++) {
            mach_port_deallocate(mach_task_self(), threadList[i]);
        }
        vm_deallocate(mach_task_self(), reinterpret_cast<vm_address_t>(threadList),
                      sizeof(thread_t) * threadCount);

        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to set thread state (error 0x%x: %s)\n", kr,
                    mach_error_string(kr));
            return false;
        }

        return true;
    }

    [[nodiscard]] auto findRuntime() const -> uintptr_t {
        mach_vm_address_t address = 0;
        mach_vm_size_t size;
        vm_region_basic_info_data_64_t info;
        mach_msg_type_number_t count = VM_REGION_BASIC_INFO_COUNT_64;
        mach_port_t objectName;
        kern_return_t kr;
        __block std::vector<uintptr_t> moduleList;

        auto* processInfo = _dyld_process_info_create(taskPort_, 0, &kr);
        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to get dyld process info (error 0x%x: %s)\n", kr,
                    mach_error_string(kr));
            return 0;
        }
        _dyld_process_info_for_each_image(processInfo,
                                          ^(uint64_t address, const uuid_t uuid, const char* path) {
                                            VERBOSE_LOG("Module: 0x%llx - %s\n", address, path);
                                            moduleList.push_back(address);
                                          });
        _dyld_process_info_release(processInfo);

        while (true) {
            if (mach_vm_region(taskPort_, &address, &size, VM_REGION_BASIC_INFO_64,
                               reinterpret_cast<vm_region_info_t>(&info), &count,
                               &objectName) != KERN_SUCCESS) {
                break;
            }

            if (info.protection & (VM_PROT_EXECUTE | VM_PROT_READ)) {
                if (std::ranges::find_if(moduleList, [address](const uintptr_t& moduleAddress) {
                        return address == moduleAddress;
                    }) == moduleList.end()) {
                    uint32_t magicBytes;
                    if (readMemory(address, &magicBytes, sizeof(magicBytes)) &&
                        magicBytes == MH_MAGIC_64) {
                        return address;
                    }
                }
            }

            address += size;
        }

        return 0;
    }
};

// Define the static constant outside the class
const unsigned int MuhDebugger::AARCH64_BREAKPOINT = 0xD4200000;

static void print_loader_usage(const char* prog) {
    std::printf(
        "usage: %s [--cooperative] <program> [program-args...]\n"
        "\n"
        "Flags:\n"
        "  --cooperative  attach via a voluntary task-port handshake instead of\n"
        "                 task_for_pid + ptrace. The target must hand over its\n"
        "                 task port over the bootstrap service named in the\n"
        "                 " X87_COOP_ENV " env var (set automatically). Needs no\n"
        "                 get-task-allow entitlement, so the binary is notarizable.\n"
        "                 Non-cooperative targets keep using the default path.\n"
        "  --help         print this message and exit\n"
        "\n"
        "All other configuration is via environment variables:\n"
        "\n",
        prog);
    print_env_help(stdout);
}

// Cooperative-mode bootstrap service name: "x87sidecar.<tracee-pid>".
static std::string coop_service_name(pid_t pid) {
    return std::string("x87sidecar.") + std::to_string(pid);
}


int main(int argc, char* argv[]) try {
    int argi = 1;
    if (argi < argc && std::string_view(argv[argi]) == "--help") {
        print_loader_usage(argv[0]);
        return 0;
    }
    bool cooperative = false;
    if (argi < argc && std::string_view(argv[argi]) == "--cooperative") {
        cooperative = true;
        ++argi;
    }
    if (argi >= argc) {
        std::fprintf(stderr, "%s: missing <program> argument (try --help)\n", argv[0]);
        return 2;
    }

    static RosettaConfig g_cfg = load_config_from_env();
    rosetta_set_config(&g_cfg);
    logsEnabled = g_cfg.loader_logs ? "1" : nullptr;

    VERBOSE_LOG("Launching debugger.\n");

    // Both attach modes use the same reverse fork: the ORIGINAL pid execs into
    // the target (keeps the pid for macOS dock/activation) and a double-forked
    // grandchild becomes the sidecar. They differ only in how the sidecar
    // obtains the tracee's task port (below); from "runtime ready" on it is all
    // shared.
    char** progArgv = &argv[argi];
    pid_t parentPid = getpid();
    MuhDebugger dbg;
    bool needsInitBarrier = true;
    mach_port_t coopReplyPort = MACH_PORT_NULL;

    // Force Rosetta to JIT (call translate_insn) instead of using its AOT /
    // interpreter path — libRosettaRuntime reads ROSETTA_DISABLE_AOT at init, so
    // it MUST be in the environment before the target execs. This is what makes
    // the JIT hook reachable in cooperative mode (which attaches post-init and
    // so can't win the race to write the g_disable_aot global in time); it is
    // harmless for default mode, which also wants AOT off.
    setenv("ROSETTA_DISABLE_AOT", "1", 1);

    // Cooperative mode publishes its bootstrap service name (derived from the
    // pid the parent keeps across execv) BEFORE forking, so the target inherits
    // it via the environment.
    std::string coopName;
    if (cooperative) {
        coopName = coop_service_name(parentPid);
        setenv(X87_COOP_ENV, coopName.c_str(), 1);
    }

    int syncPipe[2];
    if (pipe(syncPipe) == -1) {
        fprintf(stdout, "pipe: %s\n", strerror(errno));
        return 1;
    }

    pid_t child = fork();

    if (child == -1) {
        fprintf(stdout, "fork: %s\n", strerror(errno));
        return 1;
    }

    if (child != 0) {
        // PARENT → tracee: wait until the sidecar has attached/registered, then
        // exec the target (keeps the original pid).
        close(syncPipe[1]);
        char buf;
        read(syncPipe[0], &buf, 1);
        close(syncPipe[0]);
        waitpid(child, nullptr, WNOHANG);  // reap intermediate double-fork child
        VERBOSE_LOG("parent: launching into program: %s\n", progArgv[0]);
        execv(progArgv[0], progArgv);
        fprintf(stdout, "parent: execv: %s\n", strerror(errno));
        return 1;
    }

    // CHILD: double-fork so the sidecar reparents to launchd. (In default mode
    // this also breaks the ptrace PID cycle that crashes Terminal's
    // process-tree walker.)
    close(syncPipe[0]);
    pid_t intermediatePid = getpid();  // valid in C; G inherits via fork copy
    pid_t grandchild = fork();
    if (grandchild == -1) {
        fprintf(stdout, "fork (double-fork): %s\n", strerror(errno));
        _exit(1);
    }
    if (grandchild != 0) {
        _exit(0);  // intermediate child exits; grandchild reparented to launchd
    }

    // ── GRANDCHILD == the sidecar ───────────────────────────────────────────
    if (cooperative) {
        // Publish a receive port under the pid-based name; the target looks it
        // up, hands over its task+thread control ports, and blocks for a reply.
        mach_port_t bootstrapPort = MACH_PORT_NULL;
        task_get_bootstrap_port(mach_task_self(), &bootstrapPort);
        mach_port_t servicePort = MACH_PORT_NULL;
        mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_RECEIVE, &servicePort);
        mach_port_insert_right(mach_task_self(), servicePort, servicePort,
                               MACH_MSG_TYPE_MAKE_SEND);
        kern_return_t kr =
            bootstrap_register2(bootstrapPort, const_cast<char*>(coopName.c_str()), servicePort, 0);
        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "[rosettax87] bootstrap_register2(%s) failed: 0x%x\n", coopName.c_str(),
                    kr);
            return 1;
        }
        VERBOSE_LOG("[rosettax87] cooperative service registered: %s\n", coopName.c_str());
        write(syncPipe[1], "x", 1);
        close(syncPipe[1]);

        x87_coop_request_rcv_t rcv{};
        kr = mach_msg(&rcv.req.header, MACH_RCV_MSG | MACH_RCV_TIMEOUT, 0, sizeof(rcv), servicePort,
                      30000, MACH_PORT_NULL);
        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "[rosettax87] cooperative handshake receive failed: 0x%x (%s)\n", kr,
                    mach_error_string(kr));
            return 1;
        }
        task_t traceeTask = rcv.req.task_port.name;
        coopReplyPort = rcv.req.header.msgh_remote_port;
        VERBOSE_LOG("[rosettax87] cooperative attach: task=0x%x thread=0x%x reply=0x%x\n",
                    traceeTask, rcv.req.thread_port.name, coopReplyPort);
        if (!dbg.adopt(parentPid, traceeTask)) {
            return 1;
        }
        needsInitBarrier = false;  // cooperative attaches post-init; oah already mapped
    } else {
        // Default: wait for the intermediate child to exit (so kernel state is
        // cycle-free — see the process-tree-walker note), then task_for_pid +
        // ptrace attach + catch the exec SIGTRAP.
        {
            int kq = kqueue();
            if (kq >= 0) {
                struct kevent ev;
                EV_SET(&ev, intermediatePid, EVFILT_PROC, EV_ADD | EV_ONESHOT, NOTE_EXIT, 0,
                       nullptr);
                struct kevent out;
                struct timespec ts = {.tv_sec = 2, .tv_nsec = 0};
                (void)kevent(kq, &ev, 1, &out, 1, &ts);
                close(kq);
            }
            while (getppid() != 1) {
                sched_yield();
            }
        }
        if (!dbg.attach(parentPid)) {
            fprintf(stdout, "Failed to attach to parent process\n");
            return 1;
        }
        printf("[rosettax87] attached: %s\n", progArgv[0]);
        fflush(stdout);
        write(syncPipe[1], "x", 1);
        close(syncPipe[1]);
        if (!dbg.waitForExecStop()) {
            fprintf(stdout, "Failed to catch parent's exec\n");
            return 1;
        }
        VERBOSE_LOG("Attached successfully\n");
    }

    // Code ranges the M2 install patched, for the tracee to i-cache-invalidate
    // (cooperative mode only — see x87_coop_reply_t). Populated during M2.
    uint64_t coopIcacheAddr[2] = {0, 0};
    uint64_t coopIcacheLen[2] = {0, 0};

    // Release the tracee once the hook is installed (default: ptrace detach;
    // cooperative: reply to the handshake, carrying the patched code ranges so
    // the tracee flushes its own i-cache, then unblock it).
    auto releaseTracee = [&]() {
        if (cooperative) {
            x87_coop_reply_t reply{};
            reply.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_MOVE_SEND_ONCE, 0);
            reply.header.msgh_size = sizeof(reply);
            reply.header.msgh_remote_port = coopReplyPort;
            reply.header.msgh_local_port = MACH_PORT_NULL;
            reply.header.msgh_id = X87_COOP_MSGH_ID + 1;
            reply.icache_addr[0] = coopIcacheAddr[0];
            reply.icache_addr[1] = coopIcacheAddr[1];
            reply.icache_len[0] = coopIcacheLen[0];
            reply.icache_len[1] = coopIcacheLen[1];
            kern_return_t kr = mach_msg(&reply.header, MACH_SEND_MSG, sizeof(reply), 0,
                                        MACH_PORT_NULL, MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
            if (kr != KERN_SUCCESS) {
                fprintf(stdout, "[rosettax87] cooperative reply failed: 0x%x (%s)\n", kr,
                        mach_error_string(kr));
            }
        } else {
            (void)dbg.detach();
        }
    };

    // Set up offsets dynamically
    OffsetFinder offsetFinder;
    // Set default offsets temporarily (or just in case we need to fall back)
    offsetFinder.setDefaultOffsets();
    // Search the rosetta runtime binary for offsets.
    if (offsetFinder.determineOffsets()) {
        VERBOSE_LOG("Found rosetta runtime offsets successfully!\n");
        VERBOSE_LOG(
            "offset_exports_fetch=%llx offset_svc_call_entry=%llx offset_svc_call_ret=%llx\n",
            offsetFinder.offsetExportsFetch_, offsetFinder.offsetSvcCallEntry_,
            offsetFinder.offsetSvcCallRet_);
    }
    if (offsetFinder.determineRuntimeOffsets()) {
        VERBOSE_LOG("Found additional rosetta runtime offsets successfully!\n");
        VERBOSE_LOG("offset_translate_insn=%llx offset_transaction_result_size=%llx\n",
                    offsetFinder.offsetTranslateInsn_, offsetFinder.offsetTransactionResultSize_);
    }

    static const uint8_t kExportsFetchPat[8] = {0x62, 0x06, 0x40, 0xF9, 0x63, 0x12, 0x40, 0xB9};
    const auto runtimeBase = dbg.findRuntimeMatching(offsetFinder.offsetExportsFetch_,
                                                     kExportsFetchPat, sizeof(kExportsFetchPat));

    VERBOSE_LOG("Rosetta runtime base: 0x%lx\n", runtimeBase);

    if (runtimeBase == 0) {
        fprintf(stdout, "Failed to find Rosetta runtime by signature\n");
        return 1;
    }
    // NOTE: Apple's AOT/interpreter path is already disabled — libRosettaRuntime
    // read ROSETTA_DISABLE_AOT=1 (set unconditionally before fork) at init and
    // set the g_disable_aot global itself, before deciding AOT-vs-JIT. That is
    // the single mechanism for both modes: it works pre-init regardless of when
    // we attach, so we no longer poke the global from here (the old exec-stop
    // memory write only worked for the default attach and was redundant once the
    // env var is always set).

    // X87_DISABLE_HOOK=1 — passthrough mode for benchmarks.
    //
    // AOT/interpreter is already disabled (ROSETTA_DISABLE_AOT env), so the
    // target runs with stock translate_insn — the same translation environment
    // our normal mode produces for non-x87 instructions, but without our hook
    // landing for x87 (stock JIT codegen). This is the apples-to-apples baseline
    // run_benchmarks.sh compares against. Release without installing the hook.
    if (g_cfg.loader_disable_hook) {
        VERBOSE_LOG("X87_DISABLE_HOOK=1: passthrough mode; releasing without hook\n");
        releaseTracee();
        // Block until parent exits (mirror the post-stub-install path below).
        int kq = kqueue();
        if (kq != -1) {
            struct kevent ev;
            EV_SET(&ev, parentPid, EVFILT_PROC, EV_ADD, NOTE_EXIT, 0, nullptr);
            kevent(kq, &ev, 1, nullptr, 0, nullptr);
            kevent(kq, nullptr, 0, &ev, 1, nullptr);
            close(kq);
        }
        return 0;
    }

    // Reach the post-Rosetta-init point: libRosettaRuntime (which contains
    // translate_insn) is NOT mapped at the default exec-stop, but it is by the
    // time the runtime calls exports_fetch — and no TranslationResult has been
    // allocated yet. Plant a thread-level BRK there purely as an "init ready"
    // barrier (we no longer read X19 from it — translate_insn is found by
    // signature scan, the version comes from the on-disk runtime). Thread-level
    // so we never displace libRosettaRuntime's task-level EXC_BREAKPOINT handler
    // (the detach-time race that leaked a fatal SIGTRAP, CI exit=133).
    // Cooperative mode attaches post-init, so it skips the barrier entirely.
    if (needsInitBarrier) {
        if (!dbg.armThreadBreakpoint()) {
            fprintf(stdout, "Failed to arm thread-level breakpoint catcher\n");
            return 1;
        }
        dbg.setBreakpoint(runtimeBase + offsetFinder.offsetExportsFetch_);
        dbg.continueExecution();
        dbg.removeBreakpoint(runtimeBase + offsetFinder.offsetExportsFetch_);
        dbg.disarmThreadBreakpoint();
    }

    // Seed the runtime version (OpcodeCompatibility gates 26.4↔26.5 on it) from
    // the on-disk Exports.version — no live Exports struct (X19) needed. The
    // loader never calls rosetta_core_init(), so it must seed this itself.
    VERBOSE_LOG("Rosetta version: %llx\n", offsetFinder.runtimeVersion_);
    rosetta_core_set_runtime_version(offsetFinder.runtimeVersion_);

    // Locate translate_insn by its 36-byte prologue signature (both modes; no
    // X19 / initLibrary offset math). Valid because the barrier above (default)
    // or the post-init handshake (cooperative) guarantees oah is mapped.
    uint64_t translateInsnAddr = dbg.scanForPattern(OffsetFinder::kTranslateInsnPattern.data(),
                                                    OffsetFinder::kTranslateInsnPattern.size());
    if (translateInsnAddr == 0) {
        fprintf(stdout, "Failed to locate translate_insn by signature scan\n");
        return 1;
    }
    VERBOSE_LOG("translate_insn (scanned) = 0x%llx\n", translateInsnAddr);

    // ── M2: Inline IPC stub install ─────────────────────────────────────────
    //
    // 1. Find libRosettaRuntime's __TEXT trailing alignment padding (zero
    //    bytes between last function and segment-end page boundary). That's
    //    where OUR_HANDLER + STASH + STASH_JUMP will live.
    // 2. Allocate a Mach receive port in this process and plant a SEND
    //    right under a fresh name in the parent's port namespace.
    // 3. Assemble stub bytes (entry @ translate_insn[0..16] + handler blob
    //    @ trailing padding) referencing that port name.
    // 4. COW + mach_vm_write the bytes; restore RX.
    // 5. After detach, run the receive loop alongside kqueue parent-exit.
    {
        // translate_insn's live address came from the signature scan above.
        // No live Mach-O header walk, no Exports/init_library math, and no
        // TR-size patch: x87_cache no longer lives in the tracee's TR (the
        // sidecar keeps it in its own per-thread map), so stock's TR allocation
        // size is left untouched — which is what lets this install work
        // post-init in cooperative mode.

        // Snapshot the original prologue.
        uint8_t origPrologue[16];
        if (!dbg.readMemory(translateInsnAddr, origPrologue, sizeof(origPrologue))) {
            fprintf(stdout, "M2: failed to read translate_insn prologue at 0x%llx\n",
                    translateInsnAddr);
            return 1;
        }
        VERBOSE_LOG(
            "M2: translate_insn prologue: %02x%02x%02x%02x %02x%02x%02x%02x "
            "%02x%02x%02x%02x %02x%02x%02x%02x\n",
            origPrologue[0], origPrologue[1], origPrologue[2], origPrologue[3], origPrologue[4],
            origPrologue[5], origPrologue[6], origPrologue[7], origPrologue[8], origPrologue[9],
            origPrologue[10], origPrologue[11], origPrologue[12], origPrologue[13],
            origPrologue[14], origPrologue[15]);

        // ── Find the live executable region containing translate_insn ───────
        // dyld_shared_cache may split libRosettaRuntime segments. The
        // mach_header at runtimeBase doesn't necessarily live in the same
        // region as translate_insn's code. Use mach_vm_region to find the
        // bounds of the contiguous executable mapping that holds it.
        mach_vm_address_t regAddr = translateInsnAddr;
        mach_vm_size_t regSize = 0;
        vm_region_basic_info_data_64_t regInfo{};
        mach_msg_type_number_t regCount = VM_REGION_BASIC_INFO_COUNT_64;
        mach_port_t regObj = MACH_PORT_NULL;
        if (mach_vm_region(dbg.taskPort(), &regAddr, &regSize, VM_REGION_BASIC_INFO_64,
                           reinterpret_cast<vm_region_info_t>(&regInfo), &regCount,
                           &regObj) != KERN_SUCCESS) {
            fprintf(stdout, "M2: mach_vm_region(translate_insn=0x%llx) failed\n",
                    translateInsnAddr);
            return 1;
        }
        if (translateInsnAddr < regAddr || translateInsnAddr >= regAddr + regSize) {
            fprintf(stdout,
                    "M2: translate_insn (0x%llx) not in region [0x%llx, "
                    "0x%llx)\n",
                    translateInsnAddr, static_cast<uint64_t>(regAddr), (regAddr + regSize));
            return 1;
        }
        const auto codeRegStart = static_cast<uint64_t>(regAddr);
        const auto codeRegEnd = (regAddr + regSize);
        VERBOSE_LOG(
            "M2: code region containing translate_insn: [0x%llx, 0x%llx) "
            "size=0x%llx prot=0x%x\n",
            codeRegStart, codeRegEnd, codeRegEnd - codeRegStart, regInfo.protection);

        // ── Find trailing alignment padding inside that region ──────────────
        // Read the last 64 KB and find the last non-zero byte, then bytes
        // after that are trailing padding.
        constexpr uint64_t kScanWindow = 0x10000;  // 64 KB
        uint64_t scanWindow = std::min(kScanWindow, codeRegEnd - codeRegStart);
        uint64_t scanStart = codeRegEnd - scanWindow;
        std::vector<uint8_t> tail(scanWindow, 0);
        if (!dbg.readMemory(scanStart, tail.data(), scanWindow)) {
            fprintf(stdout, "M2: failed to read code-region tail at 0x%llx\n", scanStart);
            return 1;
        }
        ssize_t lastNonZero = -1;
        for (ssize_t i = static_cast<ssize_t>(scanWindow) - 1; i >= 0; i--) {
            if (tail[i] != 0) {
                lastNonZero = i;
                break;
            }
        }
        if (lastNonZero < 0) {
            fprintf(stdout, "M2: code-region tail is all zeros, refusing\n");
            return 1;
        }
        uint64_t padStartOff =
            (static_cast<uint64_t>(lastNonZero + 1) + 3) & ~static_cast<uint64_t>(3);
        uint64_t padStartAddr = scanStart + padStartOff;
        uint64_t padBytes = codeRegEnd - padStartAddr;
        VERBOSE_LOG("M2: __TEXT trailing padding starts at 0x%llx, %llu bytes free\n", padStartAddr,
                    padBytes);

        // ── Install Mach service port in parent ─────────────────────────────
        mach_port_t servicePort = MACH_PORT_NULL;
        uint32_t parentReqName = 0;
        uint32_t parentReplyName = 0;
        if (!sidecar::installPortInParent(dbg.taskPort(), &servicePort, &parentReqName,
                                          &parentReplyName)) {
            fprintf(stdout, "M2: sidecar::installPortInParent failed\n");
            return 1;
        }
        VERBOSE_LOG("M2: parent req port 0x%x  reply port 0x%x  local service 0x%x\n",
                    parentReqName, parentReplyName, servicePort);

        // ── Assemble stub bytes ─────────────────────────────────────────────
        // OUR_HANDLER + STASH + STASH_JUMP go to padStartAddr.
        // ENTRY (16-byte abs-jump to OUR_HANDLER) goes to translate_insn[0..16].
        auto blobs = stub_asm::build(padStartAddr, translateInsnAddr, origPrologue, parentReqName,
                                     parentReplyName);
        if (blobs.entry.size() != 16) {
            fprintf(stdout, "M2: stub_asm::build returned wrong entry size %zu\n",
                    blobs.entry.size());
            return 1;
        }
        if (blobs.handler.size() > padBytes) {
            fprintf(stdout,
                    "M2: handler blob (%zu bytes) doesn't fit in trailing "
                    "padding (%llu bytes)\n",
                    blobs.handler.size(), padBytes);
            return 1;
        }
        VERBOSE_LOG("M2: handler blob = %zu bytes (fits in %llu padding)\n", blobs.handler.size(),
                    padBytes);

        // ── Write OUR_HANDLER + STASH + STASH_JUMP into trailing padding ───
        if (!dbg.adjustMemoryProtection(padStartAddr, VM_PROT_READ | VM_PROT_WRITE | VM_PROT_COPY,
                                        blobs.handler.size())) {
            fprintf(stdout, "M2: failed to make padding writable\n");
            return 1;
        }
        if (!dbg.writeMemory(padStartAddr, blobs.handler.data(), blobs.handler.size())) {
            fprintf(stdout, "M2: failed to write handler blob\n");
            return 1;
        }
        if (!dbg.adjustMemoryProtection(padStartAddr, VM_PROT_READ | VM_PROT_EXECUTE,
                                        blobs.handler.size())) {
            fprintf(stdout, "M2: failed to restore padding protection\n");
            return 1;
        }
        VERBOSE_LOG("M2: handler installed at 0x%llx\n", padStartAddr);

        // ── Install transcendental polynomial constants ─────────────────────
        // 8 B aligned, immediately after OUR_HANDLER.  JIT-emitted inline
        // code (translate_fsin / translate_f2xm1 / etc.) loads coefficients
        // and lookup tables via `MOVZ/MOVK Xconst, addr; LDR Dt, [Xconst,
        // #off]`.  Source values:  ARM-software/optimized-routines
        // math/aarch64/advsimd/{sin,cos,exp2m1,log2,atan2,tan}.c.
        //
        // The whole struct (~4.3 KiB) is a single static-storage compile-
        // time constant initialiser — lives in __DATA_CONST of this loader
        // binary, gets `mach_vm_write`d into the parent in one call.  No
        // stack frame, no per-field assignment, no intermediate memcpy.
        static constexpr rosetta_core::TranscendentalConstants kTransConstants = {
            .inv_pi = std::numbers::inv_pi,
            .pi_1 = std::numbers::pi,
            .pi_2 = 0x1.1a62633145c06p-53,
            .pi_3 = 0x1.c1cd129024e09p-106,
            .sin_c =
                {
                    -0x1.555555555547bp-3,
                    0x1.1111111108a4dp-7,
                    -0x1.a01a019936f27p-13,
                    0x1.71de37a97d93ep-19,
                    -0x1.ae633919987c6p-26,
                    0x1.60e277ae07cecp-33,
                    -0x1.9e9540300a1p-41,
                },
            .range_val = 0x1p23,
            .half = 0.5,
            // f2xm1 polynomial coefficients (advsimd/exp2m1.c).
            .exp2m1_log2_hi = std::numbers::ln2,
            .exp2m1_log2_lo = 0x1.abc9e3b39803f3p-56,
            .exp2m1_c1 = 0x1.ebfbdff82c58ep-3,
            .exp2m1_c2 = 0x1.c6b08d71f5804p-5,
            .exp2m1_c3 = 0x1.3b2ab6fee7509p-7,
            .exp2m1_c4 = 0x1.5d1d37eb33b15p-10,
            .exp2m1_c5 = 0x1.423f35f371d9ap-13,
            .exp2m1_c6 = 0x1.e7d57ad9a5f93p-5,
            .exp2m1_shift = 0x1.8p45,
            .exp2m1_rnd2zero = -0x1p-8,
            .exp2m1_tablebound = 0x1.5bfffffffffffp-2,
            .one = 1.0,
            // 2^(j/128), j=0..127, biased-exponent form.  Source:
            // optimized-routines v_exp_data.c.
            .exp_table =
                {
                    0x3ff0000000000000ULL, 0x3feff63da9fb3335ULL, 0x3fefec9a3e778061ULL,
                    0x3fefe315e86e7f85ULL, 0x3fefd9b0d3158574ULL, 0x3fefd06b29ddf6deULL,
                    0x3fefc74518759bc8ULL, 0x3fefbe3ecac6f383ULL, 0x3fefb5586cf9890fULL,
                    0x3fefac922b7247f7ULL, 0x3fefa3ec32d3d1a2ULL, 0x3fef9b66affed31bULL,
                    0x3fef9301d0125b51ULL, 0x3fef8abdc06c31ccULL, 0x3fef829aaea92de0ULL,
                    0x3fef7a98c8a58e51ULL, 0x3fef72b83c7d517bULL, 0x3fef6af9388c8deaULL,
                    0x3fef635beb6fcb75ULL, 0x3fef5be084045cd4ULL, 0x3fef54873168b9aaULL,
                    0x3fef4d5022fcd91dULL, 0x3fef463b88628cd6ULL, 0x3fef3f49917ddc96ULL,
                    0x3fef387a6e756238ULL, 0x3fef31ce4fb2a63fULL, 0x3fef2b4565e27cddULL,
                    0x3fef24dfe1f56381ULL, 0x3fef1e9df51fdee1ULL, 0x3fef187fd0dad990ULL,
                    0x3fef1285a6e4030bULL, 0x3fef0cafa93e2f56ULL, 0x3fef06fe0a31b715ULL,
                    0x3fef0170fc4cd831ULL, 0x3feefc08b26416ffULL, 0x3feef6c55f929ff1ULL,
                    0x3feef1a7373aa9cbULL, 0x3feeecae6d05d866ULL, 0x3feee7db34e59ff7ULL,
                    0x3feee32dc313a8e5ULL, 0x3feedea64c123422ULL, 0x3feeda4504ac801cULL,
                    0x3feed60a21f72e2aULL, 0x3feed1f5d950a897ULL, 0x3feece086061892dULL,
                    0x3feeca41ed1d0057ULL, 0x3feec6a2b5c13cd0ULL, 0x3feec32af0d7d3deULL,
                    0x3feebfdad5362a27ULL, 0x3feebcb299fddd0dULL, 0x3feeb9b2769d2ca7ULL,
                    0x3feeb6daa2cf6642ULL, 0x3feeb42b569d4f82ULL, 0x3feeb1a4ca5d920fULL,
                    0x3feeaf4736b527daULL, 0x3feead12d497c7fdULL, 0x3feeab07dd485429ULL,
                    0x3feea9268a5946b7ULL, 0x3feea76f15ad2148ULL, 0x3feea5e1b976dc09ULL,
                    0x3feea47eb03a5585ULL, 0x3feea34634ccc320ULL, 0x3feea23882552225ULL,
                    0x3feea155d44ca973ULL, 0x3feea09e667f3bcdULL, 0x3feea012750bdabfULL,
                    0x3fee9fb23c651a2fULL, 0x3fee9f7df9519484ULL, 0x3fee9f75e8ec5f74ULL,
                    0x3fee9f9a48a58174ULL, 0x3fee9feb564267c9ULL, 0x3feea0694fde5d3fULL,
                    0x3feea11473eb0187ULL, 0x3feea1ed0130c132ULL, 0x3feea2f336cf4e62ULL,
                    0x3feea427543e1a12ULL, 0x3feea589994cce13ULL, 0x3feea71a4623c7adULL,
                    0x3feea8d99b4492edULL, 0x3feeaac7d98a6699ULL, 0x3feeace5422aa0dbULL,
                    0x3feeaf3216b5448cULL, 0x3feeb1ae99157736ULL, 0x3feeb45b0b91ffc6ULL,
                    0x3feeb737b0cdc5e5ULL, 0x3feeba44cbc8520fULL, 0x3feebd829fde4e50ULL,
                    0x3feec0f170ca07baULL, 0x3feec49182a3f090ULL, 0x3feec86319e32323ULL,
                    0x3feecc667b5de565ULL, 0x3feed09bec4a2d33ULL, 0x3feed503b23e255dULL,
                    0x3feed99e1330b358ULL, 0x3feede6b5579fdbfULL, 0x3feee36bbfd3f37aULL,
                    0x3feee89f995ad3adULL, 0x3feeee07298db666ULL, 0x3feef3a2b84f15fbULL,
                    0x3feef9728de5593aULL, 0x3feeff76f2fb5e47ULL, 0x3fef05b030a1064aULL,
                    0x3fef0c1e904bc1d2ULL, 0x3fef12c25bd71e09ULL, 0x3fef199bdd85529cULL,
                    0x3fef20ab5fffd07aULL, 0x3fef27f12e57d14bULL, 0x3fef2f6d9406e7b5ULL,
                    0x3fef3720dcef9069ULL, 0x3fef3f0b555dc3faULL, 0x3fef472d4a07897cULL,
                    0x3fef4f87080d89f2ULL, 0x3fef5818dcfba487ULL, 0x3fef60e316c98398ULL,
                    0x3fef69e603db3285ULL, 0x3fef7321f301b460ULL, 0x3fef7c97337b9b5fULL,
                    0x3fef864614f5a129ULL, 0x3fef902ee78b3ff6ULL, 0x3fef9a51fbc74c83ULL,
                    0x3fefa4afa2a490daULL, 0x3fefaf482d8e67f1ULL, 0x3fefba1bee615a27ULL,
                    0x3fefc52b376bba97ULL, 0x3fefd0765b6e4540ULL, 0x3fefdbfdad9cbe14ULL,
                    0x3fefe7c1819e90d8ULL, 0x3feff3c22b8f71f1ULL,
                },
            // (2^(j/128) - 1) for j=0..43 (positive x), then j=-44..-1
            // (negative x, accessed via index offset 24).  Source:
            // optimized-routines exp2m1.c scalem1[].
            .exp_scalem1 =
                {
                    0x0000000000000000ULL, 0x3f763da9fb33356eULL, 0x3f864d1f3bc03077ULL,
                    0x3f90c57a1b9fe12fULL, 0x3f966c34c5615d0fULL, 0x3f9c1aca777db772ULL,
                    0x3fa0e8a30eb37901ULL, 0x3fa3c7d958de7069ULL, 0x3fa6ab0d9f3121ecULL,
                    0x3fa992456e48fee8ULL, 0x3fac7d865a7a3440ULL, 0x3faf6cd5ffda635eULL,
                    0x3fb1301d0125b50aULL, 0x3fb2abdc06c31cc0ULL, 0x3fb429aaea92ddfbULL,
                    0x3fb5a98c8a58e512ULL, 0x3fb72b83c7d517aeULL, 0x3fb8af9388c8de9cULL,
                    0x3fba35beb6fcb754ULL, 0x3fbbbe084045cd3aULL, 0x3fbd4873168b9aa8ULL,
                    0x3fbed5022fcd91ccULL, 0x3fc031dc431466b2ULL, 0x3fc0fa4c8beee4b1ULL,
                    0x3fc1c3d373ab11c3ULL, 0x3fc28e727d9531faULL, 0x3fc35a2b2f13e6e9ULL,
                    0x3fc426ff0fab1c05ULL, 0x3fc4f4efa8fef709ULL, 0x3fc5c3fe86d6cc80ULL,
                    0x3fc6942d3720185aULL, 0x3fc7657d49f17ab1ULL, 0x3fc837f0518db8a9ULL,
                    0x3fc90b87e266c18aULL, 0x3fc9e0459320b7faULL, 0x3fcab62afc94ff86ULL,
                    0x3fcb8d39b9d54e55ULL, 0x3fcc6573682ec32cULL, 0x3fcd3ed9a72cffb7ULL,
                    0x3fce196e189d4724ULL, 0x3fcef5326091a112ULL, 0x3fcfd228256400ddULL,
                    0x3fd0582887dcb8a8ULL, 0x3fd0c7d76542a25bULL, 0xbfcb23213cc8e86cULL,
                    0xbfca96ecd0deb7c4ULL, 0xbfca09f58086c6c2ULL, 0xbfc97c3a3cd7e119ULL,
                    0xbfc8edb9f5703dc0ULL, 0xbfc85e7398737374ULL, 0xbfc7ce6612886a6dULL,
                    0xbfc73d904ed74b33ULL, 0xbfc6abf137076a8eULL, 0xbfc61987b33d329eULL,
                    0xbfc58652aa180903ULL, 0xbfc4f25100b03219ULL, 0xbfc45d819a94b14bULL,
                    0xbfc3c7e359c9266aULL, 0xbfc331751ec3a814ULL, 0xbfc29a35c86a9b1aULL,
                    0xbfc20224341286e4ULL, 0xbfc1693f3d7be6daULL, 0xbfc0cf85bed0f8b7ULL,
                    0xbfc034f690a387deULL, 0xbfbf332113d56b1fULL, 0xbfbdfaa500017c2dULL,
                    0xbfbcc0768d4175a6ULL, 0xbfbb84935fc8c257ULL, 0xbfba46f918837cb7ULL,
                    0xbfb907a55511e032ULL, 0xbfb7c695afc3b424ULL, 0xbfb683c7bf93b074ULL,
                    0xbfb53f391822dbc7ULL, 0xbfb3f8e749b3e342ULL, 0xbfb2b0cfe1266bd4ULL,
                    0xbfb166f067f25cfeULL, 0xbfb01b466423250aULL, 0xbfad9b9eb0a5ed76ULL,
                    0xbfaafd11874c009eULL, 0xbfa85ae0438b37cbULL, 0xbfa5b505d5b6f268ULL,
                    0xbfa30b7d271980f7ULL, 0xbfa05e4119ea5d89ULL, 0xbf9b5a991288ad16ULL,
                    0xbf95f134923757f3ULL, 0xbf90804a4c683d8fULL, 0xbf860f9f985bc9f4ULL,
                    0xbf761eea3847077bULL,
                },
            // log2 / fyl2x / fyl2xp1 polynomial + tables (advsimd/log2.c).
            .log2_off = 0x3fe6900900000000ULL,
            .log2_sign_exp_mask = 0xfff0000000000000ULL,
            .log2_invln2 = std::numbers::log2e,
            .log2_c0 = -0x1.71547652b83p-1,
            .log2_c1 = 0x1.ec709dc340953p-2,
            .log2_c2 = -0x1.71547651c8f35p-2,
            .log2_c3 = 0x1.2777ebe12dda5p-2,
            .log2_c4 = -0x1.ec738d616fe26p-3,
            // {invc, log2c} pairs at j=0..127, split into two parallel
            // arrays.  Source: optimized-routines v_log2_data.c.
            .log2_invc =
                {
                    std::numbers::sqrt2,  0x1.6815f2f3e42edp+0,
                    0x1.661e39be1ac9ep+0, 0x1.642bfa30ac371p+0,
                    0x1.623f1d916f323p+0, 0x1.60578da220f65p+0,
                    0x1.5e75349dea571p+0, 0x1.5c97fd387a75ap+0,
                    0x1.5abfd2981f200p+0, 0x1.58eca051dc99cp+0,
                    0x1.571e526d9df12p+0, 0x1.5554d555b3fcbp+0,
                    0x1.539015e2a20cdp+0, 0x1.51d0014ee0164p+0,
                    0x1.50148538cd9eep+0, 0x1.4e5d8f9f698a1p+0,
                    0x1.4cab0edca66bep+0, 0x1.4afcf1a9db874p+0,
                    0x1.495327136e16fp+0, 0x1.47ad9e84af28fp+0,
                    0x1.460c47b39ae15p+0, 0x1.446f12b278001p+0,
                    0x1.42d5efdd720ecp+0, 0x1.4140cfe001a0fp+0,
                    0x1.3fafa3b421f69p+0, 0x1.3e225c9c8ece5p+0,
                    0x1.3c98ec29a211ap+0, 0x1.3b13442a413fep+0,
                    0x1.399156baa3c54p+0, 0x1.38131639b4cdbp+0,
                    0x1.36987540fbf53p+0, 0x1.352166b648f61p+0,
                    0x1.33adddb3eb575p+0, 0x1.323dcd99fc1d3p+0,
                    0x1.30d129fefc7d2p+0, 0x1.2f67e6b72fe7dp+0,
                    0x1.2e01f7cf8b187p+0, 0x1.2c9f518ddc86ep+0,
                    0x1.2b3fe86e5f413p+0, 0x1.29e3b1211b25cp+0,
                    0x1.288aa08b373cfp+0, 0x1.2734abcaa8467p+0,
                    0x1.25e1c82459b81p+0, 0x1.2491eb1ad59c5p+0,
                    0x1.23450a54048b5p+0, 0x1.21fb1bb09e578p+0,
                    0x1.20b415346d8f7p+0, 0x1.1f6fed179a1acp+0,
                    0x1.1e2e99b93c7b3p+0, 0x1.1cf011a7a882ap+0,
                    0x1.1bb44b97dba5ap+0, 0x1.1a7b3e66cdd4fp+0,
                    0x1.1944e11dc56cdp+0, 0x1.18112aebb1a6ep+0,
                    0x1.16e013231b7e9p+0, 0x1.15b1913f156cfp+0,
                    0x1.14859cdedde13p+0, 0x1.135c2dc68cfa4p+0,
                    0x1.12353bdb01684p+0, 0x1.1110bf25b85b4p+0,
                    0x1.0feeafd2f8577p+0, 0x1.0ecf062c51c3bp+0,
                    0x1.0db1baa076c8bp+0, 0x1.0c96c5bb3048ep+0,
                    0x1.0b7e20263e070p+0, 0x1.0a67c2acd0ce3p+0,
                    0x1.0953a6391e982p+0, 0x1.0841c3caea380p+0,
                    0x1.07321489b13eap+0, 0x1.062491aee9904p+0,
                    0x1.05193497a7cc5p+0, 0x1.040ff6b5f5e9fp+0,
                    0x1.0308d19aa6127p+0, 0x1.0203beedb0c67p+0,
                    0x1.010037d38bcc2p+0, 1.0,
                    0x1.fc06d493cca10p-1, 0x1.f81e6ac3b918fp-1,
                    0x1.f44546ef18996p-1, 0x1.f07b10382c84bp-1,
                    0x1.ecbf7070e59d4p-1, 0x1.e91213f715939p-1,
                    0x1.e572a9a75f7b7p-1, 0x1.e1e0e2c530207p-1,
                    0x1.de5c72d8a8be3p-1, 0x1.dae50fa5658ccp-1,
                    0x1.d77a71145a2dap-1, 0x1.d41c51166623ep-1,
                    0x1.d0ca6ba0bb29fp-1, 0x1.cd847e8e59681p-1,
                    0x1.ca4a499693e00p-1, 0x1.c71b8e399e821p-1,
                    0x1.c3f80faf19077p-1, 0x1.c0df92dc2b0ecp-1,
                    0x1.bdd1de3cbb542p-1, 0x1.baceb9e1007a3p-1,
                    0x1.b7d5ef543e55ep-1, 0x1.b4e749977d953p-1,
                    0x1.b20295155478ep-1, 0x1.af279f8e82be2p-1,
                    0x1.ac5638197fdf3p-1, 0x1.a98e2f102e087p-1,
                    0x1.a6cf5606d05c1p-1, 0x1.a4197fc04d746p-1,
                    0x1.a16c80293dc01p-1, 0x1.9ec82c4dc5bc9p-1,
                    0x1.9c2c5a491f534p-1, 0x1.9998e1480b618p-1,
                    0x1.970d9977c6c2dp-1, 0x1.948a5c023d212p-1,
                    0x1.920f0303d6809p-1, 0x1.8f9b698a98b45p-1,
                    0x1.8d2f6b81726f6p-1, 0x1.8acae5bb55badp-1,
                    0x1.886db5d9275b8p-1, 0x1.8617ba567c13cp-1,
                    0x1.83c8d27487800p-1, 0x1.8180de3c5dbe7p-1,
                    0x1.7f3fbe71cdb71p-1, 0x1.7d055498071c1p-1,
                    0x1.7ad182e54f65ap-1, 0x1.78a42c3c90125p-1,
                    0x1.767d342f76944p-1, 0x1.745c7ef26b00ap-1,
                    0x1.7241f15769d0fp-1, 0x1.702d70d396e41p-1,
                    0x1.6e1ee3700cd11p-1, 0x1.6c162fc9cbe02p-1,
                },
            .log2_log2c =
                {
                    -0x1.00130d57f5fadp-1, -0x1.f802661bd725ep-2,
                    -0x1.efea1c6f73a5bp-2, -0x1.e7dd1dcd06f05p-2,
                    -0x1.dfdb4ae024809p-2, -0x1.d7e484d101958p-2,
                    -0x1.cff8ad452f6ep-2,  -0x1.c817a666c997fp-2,
                    -0x1.c04152d640419p-2, -0x1.b87595a3f64b2p-2,
                    -0x1.b0b4526c44d07p-2, -0x1.a8fd6d1a90f5ep-2,
                    -0x1.a150ca2559fc6p-2, -0x1.99ae4e62cca29p-2,
                    -0x1.9215df1a1e842p-2, -0x1.8a8761fe1f0d9p-2,
                    -0x1.8302bd1cc9a54p-2, -0x1.7b87d6fb437f6p-2,
                    -0x1.741696673a86dp-2, -0x1.6caee2b3c6fe4p-2,
                    -0x1.6550a3666c27ap-2, -0x1.5dfbc08de02a4p-2,
                    -0x1.56b022766c84ap-2, -0x1.4f6db1c955536p-2,
                    -0x1.4834579063054p-2, -0x1.4103fd2249a76p-2,
                    -0x1.39dc8c3fe6dabp-2, -0x1.32bdeed4b5c8fp-2,
                    -0x1.2ba80f41e20ddp-2, -0x1.249ad8332f4a7p-2,
                    -0x1.1d96347e7f3ebp-2, -0x1.169a0f7d6604ap-2,
                    -0x1.0fa654a221909p-2, -0x1.08baefcf8251ap-2,
                    -0x1.01d7cd14deecdp-2, -0x1.f5f9b1ad55495p-3,
                    -0x1.e853ff76a77afp-3, -0x1.dabe5d624cba1p-3,
                    -0x1.cd38a5cef4822p-3, -0x1.bfc2b38d315f9p-3,
                    -0x1.b25c61f5edd0fp-3, -0x1.a5058d18e9cacp-3,
                    -0x1.97be1113e47a3p-3, -0x1.8a85cafdf5e27p-3,
                    -0x1.7d5c97e8fc45bp-3, -0x1.704255d6486e4p-3,
                    -0x1.6336e2cedd7bfp-3, -0x1.563a1d9b0cc6ap-3,
                    -0x1.494be541aaa6fp-3, -0x1.3c6c1964dd0f2p-3,
                    -0x1.2f9a99f19a243p-3, -0x1.22d747344446p-3,
                    -0x1.1622020d4f7f5p-3, -0x1.097aabb3553f3p-3,
                    -0x1.f9c24b48014c5p-4, -0x1.e0aaa3bdc858ap-4,
                    -0x1.c7ae257c952d6p-4, -0x1.aecc960a03e58p-4,
                    -0x1.9605bb724d541p-4, -0x1.7d595ca7147cep-4,
                    -0x1.64c74165002d9p-4, -0x1.4c4f31c86d344p-4,
                    -0x1.33f0f70388258p-4, -0x1.1bac5abb3037dp-4,
                    -0x1.0381272495f21p-4, -0x1.d6de4eba2de2ap-5,
                    -0x1.a6ec4e8156898p-5, -0x1.772be542e3e1bp-5,
                    -0x1.479cadcde852dp-5, -0x1.183e4265faa5p-5,
                    -0x1.d2207fdaa1b85p-6, -0x1.742486cb4a6a2p-6,
                    -0x1.1687d77cfc299p-6, -0x1.7293623a6b5dep-7,
                    -0x1.70ec80ec8f25dp-8, 0.0,
                    0x1.704c1ca6b6bc9p-7,  0x1.6eac8ba664beap-6,
                    0x1.11e67d040772dp-5,  0x1.6bc665e2105dep-5,
                    0x1.c4f8a9772bf1dp-5,  0x1.0ebff10fbb951p-4,
                    0x1.3aaf4d7805d11p-4,  0x1.664ba81a4d717p-4,
                    0x1.9196387da6de4p-4,  0x1.bc902f2b7796p-4,
                    0x1.e73ab5f584f28p-4,  0x1.08cb78510d232p-3,
                    0x1.1dd2fe2f0dcb5p-3,  0x1.32b4784400df4p-3,
                    0x1.47706f3d49942p-3,  0x1.5c0768ee4a4dcp-3,
                    0x1.7079e86fc7c6dp-3,  0x1.84c86e1183467p-3,
                    0x1.98f377a34b499p-3,  0x1.acfb803bc924bp-3,
                    0x1.c0e10098b025fp-3,  0x1.d4a46efe103efp-3,
                    0x1.e8463f45b8d0bp-3,  0x1.fbc6e3228997fp-3,
                    0x1.079364f2e5aa8p-2,  0x1.1133306010a63p-2,
                    0x1.1ac309631bd17p-2,  0x1.24432485370c1p-2,
                    0x1.2db3b5449132fp-2,  0x1.3714ee1d7a32p-2,
                    0x1.406700ab52c94p-2,  0x1.49aa1d87522b2p-2,
                    0x1.52de746d7ecb2p-2,  0x1.5c0434336b343p-2,
                    0x1.651b8ad6c90d1p-2,  0x1.6e24a56ab5831p-2,
                    0x1.771fb04ec29b1p-2,  0x1.800cd6f19c25ep-2,
                    0x1.88ec441df11dfp-2,  0x1.91be21b7c93f5p-2,
                    0x1.9a8298f8c7454p-2,  0x1.a339d255c04ddp-2,
                    0x1.abe3f59f43db7p-2,  0x1.b48129deca9efp-2,
                    std::numbers::log10e,  0x1.c5955e23ebcbcp-2,
                    0x1.ce0ca8f4e1557p-2,  0x1.d6779a5a75774p-2,
                    0x1.ded6563550d27p-2,  0x1.e728ffafd840ep-2,
                    0x1.ef6fb96c8d739p-2,  0x1.f7aaa57907219p-2,
                },
            // fpatan / atan2 polynomial (advsimd/atan2.c).
            .atan2_neg_two = -2.0,
            .atan2_pi_over_2 = 0x1.921fb54442d18p+0,
            .atan2_c =
                {
                    -0x1.555555555552ap-2,  0x1.9999999995aebp-3,  -0x1.24924923923f6p-3,
                    0x1.c71c7184288a2p-4,   -0x1.745d11fb3d32bp-4, 0x1.3b136a18051b9p-4,
                    -0x1.110e6d985f496p-4,  0x1.e1bcf7f08801dp-5,  -0x1.ae644e28058c3p-5,
                    0x1.82eeb1fed85c6p-5,   -0x1.59d7f901566cbp-5, 0x1.2c982855ab069p-5,
                    -0x1.eb49592998177p-6,  0x1.69d8b396e3d38p-6,  -0x1.ca980345c4204p-7,
                    0x1.dc050eafde0b3p-8,   -0x1.7ea70755b8eccp-9, 0x1.ba3da3de903e8p-11,
                    -0x1.44a4b059b6f67p-13, 0x1.c4a45029e5a91p-17,
                },
            // fptan polynomial (advsimd/tan.c).
            .tan_two_over_pi = 0x1.45f306dc9c883p-1,
            .tan_half_pi_hi = 0x1.921fb54442d18p+0,
            .tan_half_pi_lo = 0x1.1a62633145c07p-54,
            .tan_shift = 0x1.8p52,
            .tan_poly =
                {
                    0x1.5555555555556p-2,
                    0x1.1111111110a63p-3,
                    0x1.ba1ba1bb46414p-5,
                    0x1.664f47e5b5445p-6,
                    0x1.226e5e5ecdfa3p-7,
                    0x1.d6c7ddbf87047p-9,
                    0x1.7ea75d05b583ep-10,
                    0x1.289f22964a03cp-11,
                    0x1.4e4fd14147622p-12,
                },
        };
        const uint64_t constsAddr =
            (padStartAddr + blobs.handler.size() + 0x7) & ~static_cast<uint64_t>(0x7);
        const uint64_t constsEnd = constsAddr + sizeof(kTransConstants);
        if (constsEnd > padStartAddr + padBytes) {
            fprintf(stdout,
                    "M2: transcendental constants (%zu B) don't fit in "
                    "trailing padding (free %llu B after handler at 0x%llx)\n",
                    sizeof(kTransConstants), padStartAddr + padBytes - constsAddr, constsAddr);
            return 1;
        }
        if (!dbg.adjustMemoryProtection(constsAddr, VM_PROT_READ | VM_PROT_WRITE | VM_PROT_COPY,
                                        sizeof(kTransConstants))) {
            fprintf(stdout, "M2: failed to make constants padding writable\n");
            return 1;
        }
        if (!dbg.writeMemory(constsAddr, &kTransConstants, sizeof(kTransConstants))) {
            fprintf(stdout, "M2: failed to write transcendental constants\n");
            return 1;
        }
        // Restore RX, not just R: the trailing pad is page-granular RX and
        // the constants live on the same 4 KB page as OUR_HANDLER.
        // Stripping EXECUTE here would kill the handler.
        if (!dbg.adjustMemoryProtection(constsAddr, VM_PROT_READ | VM_PROT_EXECUTE,
                                        sizeof(kTransConstants))) {
            fprintf(stdout, "M2: failed to restore constants padding protection\n");
            return 1;
        }
        rosetta_core::set_transcendental_constants_addr(constsAddr);
        VERBOSE_LOG("M2: transcendental constants installed at 0x%llx (%zu B)\n", constsAddr,
                    sizeof(kTransConstants));

        // ── X87_PROFILE: per-block counter array, shared with parent ──────
        // Allocate 8 MiB in OUR (loader/sidecar) address space, then
        // mach_vm_remap(copy=FALSE) to map the SAME backing pages into
        // parent's address space at a parent VA.  JIT-emitted code
        // atomically increments via the parent VA; the sidecar reads via
        // its local VA at exit.  This eliminates the post-NOTE_EXIT race
        // where mach_vm_read fails because parent's task port has gone
        // dead — our local mapping survives parent's death because the
        // pages are also referenced by our own task.  Skipped when
        // X87_PROFILE is unset.
        if (!g_rosetta_config->profile_path.empty()) {
            mach_vm_address_t local_addr = 0;
            kern_return_t kr = mach_vm_allocate(mach_task_self(), &local_addr,
                                                profile::kCounterBytes, VM_FLAGS_ANYWHERE);
            if (kr != KERN_SUCCESS) {
                fprintf(stdout,
                        "[rosettax87] X87_PROFILE: mach_vm_allocate(self, %zu B) failed "
                        "0x%x %s; Stage A IR dump still active, exec_count will be 0\n",
                        profile::kCounterBytes, kr, mach_error_string(kr));
            } else {
                mach_vm_address_t parent_addr = 0;
                vm_prot_t cur_prot = VM_PROT_NONE;
                vm_prot_t max_prot = VM_PROT_NONE;
                kr = mach_vm_remap(dbg.taskPort(), &parent_addr, profile::kCounterBytes, 0,
                                   VM_FLAGS_ANYWHERE, mach_task_self(), local_addr,
                                   /*copy=*/FALSE, &cur_prot, &max_prot, VM_INHERIT_NONE);
                if (kr != KERN_SUCCESS) {
                    fprintf(stdout,
                            "[rosettax87] X87_PROFILE: mach_vm_remap(parent) failed 0x%x %s; "
                            "exec_count will be 0\n",
                            kr, mach_error_string(kr));
                    mach_vm_deallocate(mach_task_self(), local_addr, profile::kCounterBytes);
                } else {
                    // Ensure parent-side mapping is RW so LDADDAL works.
                    if (mach_vm_protect(dbg.taskPort(), parent_addr, profile::kCounterBytes, FALSE,
                                        VM_PROT_READ | VM_PROT_WRITE) != KERN_SUCCESS) {
                        fprintf(stdout,
                                "[rosettax87] X87_PROFILE: mach_vm_protect(parent RW) "
                                "failed; counters may not increment\n");
                    }
                    profile::set_counter_array(parent_addr, local_addr);
                    VERBOSE_LOG(
                        "M2: X87_PROFILE counter array parent=0x%llx local=0x%llx (%zu B)\n",
                        parent_addr, local_addr, profile::kCounterBytes);
                }
            }
        }

        // ── Patch translate_insn[0..16] with the abs-jump ENTRY ────────────
        if (!dbg.adjustMemoryProtection(translateInsnAddr,
                                        VM_PROT_READ | VM_PROT_WRITE | VM_PROT_COPY,
                                        blobs.entry.size())) {
            fprintf(stdout, "M2: failed to make translate_insn writable\n");
            return 1;
        }
        if (!dbg.writeMemory(translateInsnAddr, blobs.entry.data(), blobs.entry.size())) {
            fprintf(stdout, "M2: failed to write translate_insn entry\n");
            return 1;
        }
        if (!dbg.adjustMemoryProtection(translateInsnAddr, VM_PROT_READ | VM_PROT_EXECUTE,
                                        blobs.entry.size())) {
            fprintf(stdout, "M2: failed to restore translate_insn protection\n");
            return 1;
        }
        VERBOSE_LOG("M2: translate_insn entry patched (abs-jump to 0x%llx)\n", padStartAddr);

        // Record the patched code ranges so the tracee can invalidate its own
        // i-cache (cooperative mode). The entry (hot in the i-cache post-init)
        // is the one that MUST be flushed; the handler lives in never-executed
        // padding, but flush it too for safety. Default mode ignores these (it
        // patches pre-init, so no stale lines exist and it detaches instead).
        coopIcacheAddr[0] = translateInsnAddr;
        coopIcacheLen[0] = blobs.entry.size();
        coopIcacheAddr[1] = padStartAddr;
        coopIcacheLen[1] = blobs.handler.size();

        // Capture the tracee's task port BEFORE releasing it. The send-right is
        // held in MuhDebugger::taskPort_ and stays valid past release (default's
        // ptrace detach or cooperative's handshake reply); only ~MuhDebugger
        // releases it. The receive thread uses it for mach_vm_read on
        // TranslationResult / IRInstr structs.
        mach_port_t parentTaskPort = dbg.taskPort();
        releaseTracee();

        // Spawn the Mach receive thread BEFORE the kqueue wait below so
        // any in-flight tickle messages from the parent get drained while
        // we sit on kqueue. Detached thread; cleaned up on process exit.
        if (!sidecar::spawnReceiveThread(servicePort, parentTaskPort)) {
            fprintf(stdout, "M2: failed to spawn receive thread\n");
            return 1;
        }
        VERBOSE_LOG("M2: receive thread running; entering kqueue wait\n");

        // Self-test: send a tickle message from this process to our own
        // service port. If the receive thread picks it up, the receive
        // plumbing is healthy (and the issue is on the parent side).
        {
            mach_msg_header_t selfMsg{};
            selfMsg.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, 0);
            selfMsg.msgh_size = sizeof(selfMsg);
            selfMsg.msgh_remote_port = servicePort;
            selfMsg.msgh_local_port = MACH_PORT_NULL;
            selfMsg.msgh_id = 0xCAFEBABE;
            kern_return_t kr = mach_msg(&selfMsg, MACH_SEND_MSG, sizeof(selfMsg), 0, MACH_PORT_NULL,
                                        MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
            VERBOSE_LOG("M2: self-test mach_msg send → 0x%x (%s)\n", kr, mach_error_string(kr));
        }
    }

    // Block until the parent (wine) exits. We can't use waitpid since
    // the parent is not our child, so use kqueue with EVFILT_PROC.
    int kq = kqueue();
    if (kq != -1) {
        struct kevent ev;
        EV_SET(&ev, parentPid, EVFILT_PROC, EV_ADD, NOTE_EXIT, 0, nullptr);
        kevent(kq, &ev, 1, nullptr, 0, nullptr);
        // Block until parent exits
        kevent(kq, nullptr, 0, &ev, 1, nullptr);
        // Window after NOTE_EXIT but before kernel reaps the parent task:
        // mach_vm_read still works against the held task-port send-right.
        // Use it to pull X87_PROFILE counters back into the .prof file.
        sidecar::dumpCountersIfEnabled(dbg.taskPort());
        close(kq);
    }

    return 0;
} catch (const std::exception& e) {
    fprintf(stderr, "rosettax87: %s\n", e.what());
    return 1;
}
