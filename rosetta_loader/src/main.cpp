#include <mach-o/dyld.h>
#include <mach-o/dyld_images.h>
#include <mach/mach_vm.h>
#include <rosetta_config/Config.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/event.h>
#include <sys/ptrace.h>
#include <sys/wait.h>
#include <sched.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include "macho_loader.hpp"
#include "offset_finder.hpp"
#include "rosetta_core/TranscendentalHelper.h"
#include "rosetta_core/TranslationResult.h"
#include "sidecar.hpp"
#include "stub_asm.hpp"
#include "types.h"

const char* logsEnabled = nullptr;

#define VERBOSE_LOG(fmt, ...)                   \
    do {                                \
        if (logsEnabled) {              \
            printf(fmt, ##__VA_ARGS__); \
        }                               \
    } while (0)

typedef const struct dyld_process_info_base* DyldProcessInfo;

extern "C" DyldProcessInfo _dyld_process_info_create(task_t task, uint64_t timestamp,
                                                     kern_return_t* kernelError);
extern "C" void _dyld_process_info_for_each_image(DyldProcessInfo info,
                                                  void (^callback)(uint64_t machHeaderAddress,
                                                                   const uuid_t uuid,
                                                                   const char* path));
extern "C" void _dyld_process_info_release(DyldProcessInfo info);

class MuhDebugger {
private:
    static const uint32_t AARCH64_BREAKPOINT;  // just declare here

    pid_t childPid_ = -1;
    task_t taskPort_ = MACH_PORT_NULL;
    std::map<uint64_t, uint32_t> breakpoints_;  // addr -> original instruction

    // Wait for the traced process to stop. If expectedSignal is non-zero,
    // loop and suppress any other signals until the expected one arrives.
    bool waitForStopped(int expectedSignal = 0) {
        while (true) {
            int status;
            if (waitpid(childPid_, &status, 0) == -1) {
                fprintf(stdout, "waitpid: %s\n", strerror(errno));
                return false;
            }
            if (!WIFSTOPPED(status)) {
                return false;
            }
            int sig = WSTOPSIG(status);
            VERBOSE_LOG("Process stopped signal=%d\n", sig);
            if (expectedSignal == 0 || sig == expectedSignal) {
                return true;
            }
            // Spurious signal; suppress it and continue
            VERBOSE_LOG("Suppressing unexpected signal %d (waiting for %d)\n", sig, expectedSignal);
            if (ptrace(PT_CONTINUE, childPid_, (caddr_t)1, 0) < 0) {
                fprintf(stdout, "ptrace(PT_CONTINUE suppress): %s\n", strerror(errno));
                return false;
            }
        }
    }

public:
    ~MuhDebugger() {
        if (taskPort_ != MACH_PORT_NULL) {
            mach_port_deallocate(mach_task_self(), taskPort_);
        }
    }

    task_t taskPort() const { return taskPort_; }

    bool attach(pid_t pid) {
        childPid_ = pid;
        VERBOSE_LOG("Attempting to attach to %d\n", childPid_);

        // Attach to the target via PT_ATTACH (sends SIGSTOP).
        if (ptrace(PT_ATTACH, childPid_, nullptr, 0) == -1) {
            fprintf(stdout, "ptrace(PT_ATTACH): %s\n", strerror(errno));
            return false;
        }
        if (!waitForStopped()) {
            return false;
        }
        VERBOSE_LOG("Attached to %d (SIGSTOP)\n", childPid_);
        return true;
    }

    // Continue the traced process and wait for it to stop at execv (SIGTRAP).
    bool waitForExecStop() {
        if (ptrace(PT_CONTINUE, childPid_, (caddr_t)1, 0) < 0) {
            fprintf(stdout, "ptrace(PT_CONTINUE for exec): %s\n", strerror(errno));
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
        VERBOSE_LOG("Started debugging process %d using port %d\n", childPid_, taskPort_);
        return true;
    }

    bool continueExecution() {
        if (ptrace(PT_CONTINUE, childPid_, (caddr_t)1, 0) < 0) {
            fprintf(stdout, "ptrace(PT_CONTINUE): %s\n", strerror(errno));
            return false;
        }

        VERBOSE_LOG("continueExecution...\n");

        return waitForStopped(SIGTRAP);
    }

    bool detach() {
        if (ptrace(PT_DETACH, childPid_, (caddr_t)1, 0) < 0) {
            fprintf(stdout, "ptrace(PT_DETACH): %s\n", strerror(errno));
            return false;
        }
        VERBOSE_LOG("Debugger detached.\n");
        return true;
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

    enum Register {
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

    uint64_t readRegister(Register reg) {
        thread_act_port_array_t threadList;
        mach_msg_type_number_t threadCount;

        kern_return_t kr = task_threads(taskPort_, &threadList, &threadCount);
        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to get threads (error 0x%x: %s)\n", kr, mach_error_string(kr));
            return 0;
        }

        arm_thread_state64_t state;
        mach_msg_type_number_t count = ARM_THREAD_STATE64_COUNT;
        kr = thread_get_state(threadList[0], ARM_THREAD_STATE64, (thread_state_t)&state, &count);

        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to get thread state (error 0x%x: %s)\n", kr,
                    mach_error_string(kr));
            return 0;
        }

        uint64_t value = 0;
        if (reg >= X0 && reg <= X28) {
            value = state.__x[reg];
        } else {
            switch (reg) {
                case FP:
                    value = state.__fp;
                    break;
                case LR:
                    value = state.__lr;
                    break;
                case SP:
                    value = state.__sp;
                    break;
                case PC:
                    value = state.__pc;
                    break;
                case CPSR:
                    value = state.__cpsr;
                    break;
                default: {
                    fprintf(stdout, "Invalid register\n");
                    return 0;
                }
            }
        }

        // Cleanup
        for (unsigned int i = 0; i < threadCount; i++) {
            mach_port_deallocate(mach_task_self(), threadList[i]);
        }
        vm_deallocate(mach_task_self(), (vm_address_t)threadList, sizeof(thread_t) * threadCount);

        return value;
    }

    bool setRegister(Register reg, uint64_t value) {
        thread_act_port_array_t threadList;
        mach_msg_type_number_t threadCount;

        kern_return_t kr = task_threads(taskPort_, &threadList, &threadCount);
        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to get threads (error 0x%x: %s)\n", kr, mach_error_string(kr));
            return false;
        }

        arm_thread_state64_t state;
        mach_msg_type_number_t count = ARM_THREAD_STATE64_COUNT;
        kr = thread_get_state(threadList[0], ARM_THREAD_STATE64, (thread_state_t)&state, &count);

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

        kr = thread_set_state(threadList[0], ARM_THREAD_STATE64, (thread_state_t)&state,
                              ARM_THREAD_STATE64_COUNT);
        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to set thread state (error 0x%x: %s)\n", kr,
                    mach_error_string(kr));
            return false;
        }

        // Cleanup
        for (uint i = 0; i < threadCount; i++) {
            mach_port_deallocate(mach_task_self(), threadList[i]);
        }
        vm_deallocate(mach_task_self(), (vm_address_t)threadList, sizeof(thread_t) * threadCount);

        return true;
    }

    bool adjustMemoryProtection(uint64_t address, vm_prot_t protection, mach_vm_size_t size) {
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
                    (uint64_t)region, (uint64_t)(region + size), kr, mach_error_string(kr));
            return false;
        }
        return true;
    }

    bool readMemory(uint64_t address, void* buffer, size_t size) {
        mach_vm_size_t readSize;

        kern_return_t kr =
            mach_vm_read_overwrite(taskPort_, address, size, (mach_vm_address_t)buffer, &readSize);

        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to read memory at 0x%llx (error 0x%x: %s)\n", address, kr,
                    mach_error_string(kr));
            return false;
        }

        return readSize == size;
    }

    bool writeMemory(uint64_t address, const void* buffer, size_t size) {
        kern_return_t kr = mach_vm_write(taskPort_, address, (vm_offset_t)buffer, size);

        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to write memory at 0x%llx (error 0x%x: %s)\n", address, kr,
                    mach_error_string(kr));
            return false;
        }

        return true;
    }

    bool copyThreadState(arm_thread_state64_t& state) {
        thread_act_port_array_t threadList;
        mach_msg_type_number_t threadCount;

        kern_return_t kr = task_threads(taskPort_, &threadList, &threadCount);
        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to get threads (error 0x%x: %s)\n", kr, mach_error_string(kr));
            return false;
        }

        mach_msg_type_number_t count = ARM_THREAD_STATE64_COUNT;
        kr = thread_get_state(threadList[0], ARM_THREAD_STATE64, (thread_state_t)&state, &count);

        // Cleanup
        for (uint i = 0; i < threadCount; i++) {
            mach_port_deallocate(mach_task_self(), threadList[i]);
        }
        vm_deallocate(mach_task_self(), (vm_address_t)threadList, sizeof(thread_t) * threadCount);

        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to get thread state (error 0x%x: %s)\n", kr,
                    mach_error_string(kr));
            return false;
        }

        return true;
    }

    bool restoreThreadState(const arm_thread_state64_t& state) {
        thread_act_port_array_t threadList;
        mach_msg_type_number_t threadCount;

        kern_return_t kr = task_threads(taskPort_, &threadList, &threadCount);
        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to get threads (error 0x%x: %s)\n", kr, mach_error_string(kr));
            return false;
        }

        kr = thread_set_state(threadList[0], ARM_THREAD_STATE64, (thread_state_t)&state,
                              ARM_THREAD_STATE64_COUNT);

        // Cleanup
        for (uint i = 0; i < threadCount; i++) {
            mach_port_deallocate(mach_task_self(), threadList[i]);
        }
        vm_deallocate(mach_task_self(), (vm_address_t)threadList, sizeof(thread_t) * threadCount);

        if (kr != KERN_SUCCESS) {
            fprintf(stdout, "Failed to set thread state (error 0x%x: %s)\n", kr,
                    mach_error_string(kr));
            return false;
        }

        return true;
    }

    auto findRuntime() -> uintptr_t {
        mach_vm_address_t address = 0;
        mach_vm_size_t size;
        vm_region_basic_info_data_64_t info;
        mach_msg_type_number_t count = VM_REGION_BASIC_INFO_COUNT_64;
        mach_port_t objectName;
        kern_return_t kr;
        __block std::vector<uintptr_t> moduleList;

        auto processInfo = _dyld_process_info_create(taskPort_, 0, &kr);
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
                               (vm_region_info_t)&info, &count, &objectName) != KERN_SUCCESS) {
                break;
            }

            if (info.protection & (VM_PROT_EXECUTE | VM_PROT_READ)) {
                if (std::find_if(moduleList.begin(), moduleList.end(),
                                 [address](const uintptr_t& moduleAddress) {
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

// Resolve a Windows-style path (e.g. "C:\foo\bar.exe") to a macOS path
// using the Wine prefix dosdevices mapping.
static std::string resolveWinePath(const char* winPath) {
    const char* prefix = getenv("WINEPREFIX");
    std::string winePrefix = prefix ? prefix : (std::string(getenv("HOME")) + "/.wine");

    // Find drive letter (e.g. "C:")
    if (strlen(winPath) < 3 || winPath[1] != ':')
        return {};

    char driveLetter = tolower(winPath[0]);
    std::string dosDevice = winePrefix + "/dosdevices/" + driveLetter + ":";

    // Resolve the symlink to get the real drive root
    char resolved[PATH_MAX];
    if (!realpath(dosDevice.c_str(), resolved))
        return {};

    // Convert the rest of the path: skip "C:", replace backslashes
    std::string result = resolved;
    const char* rest = winPath + 2;
    for (; *rest; rest++) {
        result += (*rest == '\\') ? '/' : *rest;
    }
    return result;
}

// Classify a file by reading its PE header. Returns X86 / X64 for
// recognised PE machine fields, NotPE for everything else (unreadable,
// missing MZ/PE signatures, unknown machine, or e.g. a Mach-O binary).
enum class PeArch { NotPE, X86, X64 };

static PeArch classifyPE(const std::string& path) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f)
        return PeArch::NotPE;

    uint16_t dosMagic;
    if (fread(&dosMagic, 2, 1, f) != 1 || dosMagic != 0x5A4D) {
        fclose(f);
        return PeArch::NotPE;
    }

    uint32_t peOffset;
    fseek(f, 0x3C, SEEK_SET);
    if (fread(&peOffset, 4, 1, f) != 1) {
        fclose(f);
        return PeArch::NotPE;
    }

    fseek(f, peOffset, SEEK_SET);
    uint32_t peSig;
    if (fread(&peSig, 4, 1, f) != 1 || peSig != 0x00004550) {
        fclose(f);
        return PeArch::NotPE;
    }

    uint16_t machine;
    if (fread(&machine, 2, 1, f) != 1) {
        fclose(f);
        return PeArch::NotPE;
    }
    fclose(f);

    switch (machine) {
        case 0x014C: return PeArch::X86;  // IMAGE_FILE_MACHINE_I386
        case 0x8664: return PeArch::X64;  // IMAGE_FILE_MACHINE_AMD64
        default:     return PeArch::NotPE;
    }
}

// Outcome of inspecting argv for a Windows PE. The skip flag drives
// whether the loader attaches; displayPath carries the resolved native
// .exe path (when found in argv) so the always-on summary line in main()
// can show what was actually classified — falling back to argv[1] when
// no .exe was identified (Mach-O test/bench binaries).
struct AttachDecision {
    bool        skip;         // true → execv passthrough, false → attach
    std::string displayPath;  // resolved Wine native path if found, else empty
    const char* reason;       // non-null only when skip=true (e.g. "x64 PE")
};

// Examine argv for a Windows PE. Skip only when argv positively
// identifies a 64-bit PE (Wine running an x64 .exe — x87 JIT is
// unnecessary). Anything else — Mach-O test/bench binaries, unknown
// inputs — falls through and the loader attaches by default.
static AttachDecision classifyAttachTarget(int argc, char* argv[]) {
    for (int i = 2; i < argc; i++) {
        // Windows-style path (drive letter + colon)
        if (strlen(argv[i]) >= 3 && argv[i][1] == ':') {
            std::string nativePath = resolveWinePath(argv[i]);
            if (nativePath.empty()) {
                VERBOSE_LOG("Could not resolve Wine path '%s', attaching.\n", argv[i]);
                return {false, std::string(argv[i]), nullptr};
            }
            VERBOSE_LOG("Resolved '%s' -> '%s'\n", argv[i], nativePath.c_str());
            PeArch arch = classifyPE(nativePath);
            VERBOSE_LOG("PE architecture: %s\n",
                arch == PeArch::X86   ? "x86 (32-bit)"
                : arch == PeArch::X64 ? "x64 (64-bit)"
                                      : "not a PE");
            return {arch == PeArch::X64, std::move(nativePath),
                    arch == PeArch::X64 ? "x64 PE" : nullptr};
        }

        // Bare .exe filename — resolve relative to cwd
        size_t len = strlen(argv[i]);
        if (len >= 4 && strcasecmp(argv[i] + len - 4, ".exe") == 0) {
            PeArch arch = classifyPE(argv[i]);
            if (arch == PeArch::X64) {
                VERBOSE_LOG("'%s' is x64 (64-bit), skipping\n", argv[i]);
                return {true, std::string(argv[i]), "x64 PE"};
            }
            // x86 PE or non-PE: keep scanning (file may not exist in cwd).
        }
    }
    return {false, {}, nullptr};
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stdout, "%s <path to program>\n", argv[0]);
        return 1;
    }

    logsEnabled = getenv("ROSETTA_X87_LOGS");

    AttachDecision decision = classifyAttachTarget(argc, argv);
    const std::string summaryPath =
        decision.displayPath.empty() ? std::string(argv[1]) : decision.displayPath;

    // Skip debugger attachment for 64-bit Windows programs (no x87 needed)
    if (!getenv("ROSETTA_X87_FORCE_ATTACH") && decision.skip) {
        printf("[rosettax87] skipped: %s (%s)\n", summaryPath.c_str(), decision.reason);
        fflush(stdout);
        VERBOSE_LOG("Program is x64 PE, skipping x87 JIT. Passing through.\n");
        execv(argv[1], &argv[1]);
        fprintf(stdout, "execv: %s\n", strerror(errno));
        return 1;
    }

    VERBOSE_LOG("Launching debugger.\n");

    // Reverse fork: parent execs into wine (keeps original PID for macOS
    // dock/activation tracking), child becomes the debugger.
    pid_t parentPid = getpid();
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
        // PARENT: will exec into wine-preloader (keeps original PID)
        close(syncPipe[1]);
        // Wait for child debugger to attach to us
        char buf;
        read(syncPipe[0], &buf, 1);
        close(syncPipe[0]);
        // Child has attached and will catch our exec's SIGTRAP
        waitpid(child, nullptr, WNOHANG);  // reap intermediate double-fork child
        VERBOSE_LOG("parent: launching into program: %s\n", argv[1]);
        execv(argv[1], &argv[1]);
        fprintf(stdout, "parent: execv: %s\n", strerror(errno));
        return 1;
    }

    // CHILD: double-fork to orphan the debugger process.
    // This prevents a PID cycle (child->parent->child via ptrace) in the
    // process table that crashes Terminal.app's recursive process-tree walker.
    close(syncPipe[0]);
    pid_t intermediatePid = getpid();   // valid in C; G inherits via fork copy
    pid_t grandchild = fork();
    if (grandchild == -1) {
        fprintf(stdout, "fork (double-fork): %s\n", strerror(errno));
        _exit(1);
    }
    if (grandchild != 0) {
        _exit(0);  // intermediate child exits; grandchild reparented to PID 1
    }

    // GRANDCHILD: wait for the intermediate child to exit before PT_ATTACH.
    // PT_ATTACH would otherwise reparent R under G while C is still alive
    // with G as its child, briefly creating a children-list cycle (R↔G via
    // PT_ATTACH plus G's original ppid=C still in C.children) that crashes
    // Terminal.app's proc_listchildpids walker. NOTE_EXIT fires from xnu's
    // proc_exit after G has already been reparented to launchd and C has
    // been removed from R.children — kernel state is then guaranteed
    // cycle-free.
    {
        int kq = kqueue();
        if (kq >= 0) {
            struct kevent ev;
            EV_SET(&ev, intermediatePid, EVFILT_PROC, EV_ADD | EV_ONESHOT,
                   NOTE_EXIT, 0, nullptr);
            struct kevent out;
            struct timespec ts = {2, 0};   // generous; C's path is ~2 insns
            (void)kevent(kq, &ev, 1, &out, 1, &ts);
            close(kq);
        }
        // Cover the case where C exited before kevent could register.
        while (getppid() != 1) {
            sched_yield();
        }
    }

    MuhDebugger dbg;
    if (!dbg.attach(parentPid)) {
        fprintf(stdout, "Failed to attach to parent process\n");
        return 1;
    }
    printf("[rosettax87] attached: %s\n", summaryPath.c_str());
    fflush(stdout);
    // Signal parent to proceed with execv
    write(syncPipe[1], "x", 1);
    close(syncPipe[1]);

    // Wait for parent's execv to trigger SIGTRAP
    if (!dbg.waitForExecStop()) {
        fprintf(stdout, "Failed to catch parent's exec\n");
        return 1;
    }
    VERBOSE_LOG("Attached successfully\n");

    // Set up offsets dynamically
    OffsetFinder offsetFinder;
    // Set default offsets temporarily (or just in case we need to fall back)
    offsetFinder.setDefaultOffsets();
    // Search the rosetta runtime binary for offsets.
    if (offsetFinder.determineOffsets()) {
        VERBOSE_LOG("Found rosetta runtime offsets successfully!\n");
        VERBOSE_LOG("offset_exports_fetch=%llx offset_svc_call_entry=%llx offset_svc_call_ret=%llx\n",
            offsetFinder.offsetExportsFetch_, offsetFinder.offsetSvcCallEntry_,
            offsetFinder.offsetSvcCallRet_);
    }
    if (offsetFinder.determineRuntimeOffsets()) {
        VERBOSE_LOG("Found additional rosetta runtime offsets successfully!\n");
        VERBOSE_LOG("offset_translate_insn=%llx offset_transaction_result_size=%llx\n",
            offsetFinder.offsetTranslateInsn_, offsetFinder.offsetTransactionResultSize_);
    }

    const auto runtimeBase = dbg.findRuntime();

    VERBOSE_LOG("Rosetta runtime base: 0x%lx\n", runtimeBase);

    if (runtimeBase == 0) {
        fprintf(stdout, "Failed to find Rosetta runtime\n");
        return 1;
    }
    uint8_t g_disable_aot_value = 1;

    dbg.writeMemory(runtimeBase + offsetFinder.offsetDisableAot_, &g_disable_aot_value,
                    sizeof(g_disable_aot_value));

    dbg.setBreakpoint(runtimeBase + offsetFinder.offsetExportsFetch_);
    dbg.continueExecution();
    dbg.removeBreakpoint(runtimeBase + offsetFinder.offsetExportsFetch_);

    auto rosettaRuntimeExportsAddress = dbg.readRegister(MuhDebugger::Register::X19);
    VERBOSE_LOG("Rosetta runtime exports: 0x%llx\n", rosettaRuntimeExportsAddress);

    Exports exports;
    dbg.readMemory(rosettaRuntimeExportsAddress, &exports, sizeof(exports));

    VERBOSE_LOG("Rosetta version: %llx\n", exports.version);

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
        // ── Read live Mach-O headers from parent's address space ────────────
        // The on-disk libRosettaRuntime at /Library/Apple/usr/libexec/oah is a
        // shared-cache STUB whose segment sizes (and the byte-pattern offsets
        // computed against it) do NOT match what's actually mapped in the
        // parent — the dyld_shared_cache version is repacked. Everything we
        // need (translate_insn address, __TEXT bounds) we discover from the
        // live process memory instead.
        mach_header_64 mh{};
        if (!dbg.readMemory(runtimeBase, &mh, sizeof(mh))) {
            fprintf(stdout, "M2: failed to read parent's mach_header_64\n");
            return 1;
        }
        if (mh.magic != MH_MAGIC_64) {
            fprintf(stdout, "M2: parent's mach_header magic mismatch (0x%x)\n",
                    mh.magic);
            return 1;
        }

        // Read the load-commands region following the header.
        std::vector<uint8_t> lcBuf(mh.sizeofcmds, 0);
        if (!dbg.readMemory(runtimeBase + sizeof(mh), lcBuf.data(),
                            mh.sizeofcmds)) {
            fprintf(stdout, "M2: failed to read parent's load commands\n");
            return 1;
        }

        // Walk to find __TEXT segment.
        uint64_t textVmAddr = 0;   // link-time vmaddr (offset basis for dyld
                                   // shared-cache; runtimeBase corresponds
                                   // to this)
        uint64_t textVmSize = 0;
        const uint8_t* p = lcBuf.data();
        for (uint32_t i = 0; i < mh.ncmds; i++) {
            auto* lc = (const load_command*)p;
            if (lc->cmd == LC_SEGMENT_64) {
                auto* seg = (const segment_command_64*)p;
                if (strncmp(seg->segname, "__TEXT", 16) == 0) {
                    textVmAddr = seg->vmaddr;
                    textVmSize = seg->vmsize;
                    break;
                }
            }
            p += lc->cmdsize;
        }
        if (textVmSize == 0) {
            fprintf(stdout, "M2: parent has no __TEXT segment\n");
            return 1;
        }
        const uint64_t textEndAddr = runtimeBase + textVmSize;
        VERBOSE_LOG("__TEXT range (live): [0x%lx, 0x%lx) (vmaddr=0x%llx vmsize=0x%llx)\n",
            (unsigned long)runtimeBase, (unsigned long)textEndAddr, textVmAddr,
            textVmSize);

        // ── Compute translate_insn's live address from a known function ──
        // pointer plus the file-offset delta. dyld_shared_cache may remap
        // libRosettaRuntime's segments to different addresses, so we cannot
        // just do `runtimeBase + translate_insn_rva`. Within a single
        // segment, however, function-to-function offsets are preserved, so:
        //   translate_insn_addr  =  init_library_runtime_addr
        //                         + (translate_insn_rva - init_library_rva)
        // init_library's runtime address is the first entry in
        // libRosettaRuntime's x87Exports[] array (X19 points at the
        // Exports struct; first export is init_library).
        Export initLibraryExport{};
        if (!dbg.readMemory(uint64_t(exports.x87Exports), &initLibraryExport,
                            sizeof(initLibraryExport))) {
            fprintf(stdout, "M2: failed to read init_library export entry\n");
            return 1;
        }
        uint64_t initLibraryAddr =
            uint64_t(initLibraryExport.address) & 0xFFFFFFFFFFFFULL;
        if (initLibraryAddr == 0) {
            fprintf(stdout, "M2: init_library export address is null\n");
            return 1;
        }
        if (offsetFinder.offsetInitLibrary_ == 0 ||
            offsetFinder.offsetTranslateInsn_ == 0) {
            fprintf(stdout,
                    "M2: missing init_library_rva or translate_insn_rva from "
                    "offset_finder\n");
            return 1;
        }
        uint64_t translateInsnAddr =
            initLibraryAddr +
            (uint64_t(offsetFinder.offsetTranslateInsn_) -
             uint64_t(offsetFinder.offsetInitLibrary_));
        VERBOSE_LOG("M2: init_library live=0x%llx rva=0x%llx | translate_insn rva=0x%llx → "
            "live=0x%llx\n",
            initLibraryAddr, (uint64_t)offsetFinder.offsetInitLibrary_,
            (uint64_t)offsetFinder.offsetTranslateInsn_, translateInsnAddr);

        // ── Patch stock's TR-size MOVZ so each TR holds our X87Cache ────────
        // libRosettaRuntime allocates per-thread TranslationResults sized by
        // a `MOV W0, #0x288` immediate that aotinvoke's pattern search located
        // (offsetTransactionResultSize_). We've extended TranslationResult
        // with `x87_cache` (OPT-1) appended at the end — see
        // TranslationResult.h. Patch the MOVZ in stock's __TEXT to allocate
        // sizeof(TranslationResult) bytes per TR so the cache field lives
        // inside parent's allocation and our write-back at the end of every
        // translate_insn call can persist it across calls (no heap overflow).
        //
        // Patch must run BEFORE any TR is allocated. We're paused at
        // offsetExportsFetch_ — early in libRosettaRuntime's init, before
        // any thread has called translate_insn — so we're safe.
        {
            if (offsetFinder.offsetTransactionResultSize_ == 0) {
                fprintf(stdout, "M2: missing offsetTransactionResultSize_\n");
                return 1;
            }
            const uint64_t trSizeAddr =
                initLibraryAddr +
                (uint64_t(offsetFinder.offsetTransactionResultSize_) -
                 uint64_t(offsetFinder.offsetInitLibrary_));

            constexpr uint32_t kNewTrSize = sizeof(TranslationResult);
            static_assert(kNewTrSize <= 0xFFFFu,
                          "TranslationResult must fit in MOVZ imm16");

            uint32_t origInsn = 0;
            if (!dbg.readMemory(trSizeAddr, &origInsn, sizeof(origInsn))) {
                fprintf(stdout, "M2: failed to read TR-size MOVZ at 0x%llx\n",
                        trSizeAddr);
                return 1;
            }
            // MOVZ Wd, #imm16, lsl #0 — sf=0, opc=10, hw=00.
            //   fixed bits mask 0xFFE00000, value 0x52800000
            //   imm16 lives in bits [20:5]
            if ((origInsn & 0xFFE00000u) != 0x52800000u) {
                fprintf(stdout,
                        "M2: TR-size patch site at 0x%llx is not MOVZ "
                        "Wd,#imm (got 0x%08x)\n",
                        trSizeAddr, origInsn);
                return 1;
            }
            const uint32_t origImm = (origInsn >> 5) & 0xFFFFu;
            const uint32_t newInsn =
                (origInsn & ~0x001FFFE0u) | (uint32_t(kNewTrSize) << 5);

            if (!dbg.adjustMemoryProtection(
                    trSizeAddr, VM_PROT_READ | VM_PROT_WRITE | VM_PROT_COPY,
                    sizeof(newInsn))) {
                fprintf(stdout, "M2: TR-size patch protect-RW failed\n");
                return 1;
            }
            if (!dbg.writeMemory(trSizeAddr, &newInsn, sizeof(newInsn))) {
                fprintf(stdout, "M2: TR-size patch write failed\n");
                return 1;
            }
            if (!dbg.adjustMemoryProtection(trSizeAddr,
                                            VM_PROT_READ | VM_PROT_EXECUTE,
                                            sizeof(newInsn))) {
                fprintf(stdout, "M2: TR-size patch protect-RX failed\n");
                return 1;
            }
            VERBOSE_LOG("M2: TR-size MOVZ at 0x%llx patched: 0x%x → 0x%x\n",
                trSizeAddr, origImm, kNewTrSize);
        }

        // Snapshot the original prologue.
        uint8_t origPrologue[16];
        if (!dbg.readMemory(translateInsnAddr, origPrologue,
                             sizeof(origPrologue))) {
            fprintf(stdout,
                    "M2: failed to read translate_insn prologue at 0x%llx\n",
                    translateInsnAddr);
            return 1;
        }
        VERBOSE_LOG("M2: translate_insn prologue: %02x%02x%02x%02x %02x%02x%02x%02x "
            "%02x%02x%02x%02x %02x%02x%02x%02x\n",
            origPrologue[0], origPrologue[1], origPrologue[2], origPrologue[3],
            origPrologue[4], origPrologue[5], origPrologue[6], origPrologue[7],
            origPrologue[8], origPrologue[9], origPrologue[10], origPrologue[11],
            origPrologue[12], origPrologue[13], origPrologue[14], origPrologue[15]);

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
        if (mach_vm_region(dbg.taskPort(), &regAddr, &regSize,
                           VM_REGION_BASIC_INFO_64,
                           reinterpret_cast<vm_region_info_t>(&regInfo),
                           &regCount, &regObj) != KERN_SUCCESS) {
            fprintf(stdout,
                    "M2: mach_vm_region(translate_insn=0x%llx) failed\n",
                    translateInsnAddr);
            return 1;
        }
        if (translateInsnAddr < regAddr ||
            translateInsnAddr >= regAddr + regSize) {
            fprintf(stdout,
                    "M2: translate_insn (0x%llx) not in region [0x%llx, "
                    "0x%llx)\n",
                    translateInsnAddr, (uint64_t)regAddr,
                    (uint64_t)(regAddr + regSize));
            return 1;
        }
        const uint64_t codeRegStart = uint64_t(regAddr);
        const uint64_t codeRegEnd = uint64_t(regAddr + regSize);
        VERBOSE_LOG("M2: code region containing translate_insn: [0x%llx, 0x%llx) "
            "size=0x%llx prot=0x%x\n",
            codeRegStart, codeRegEnd, codeRegEnd - codeRegStart,
            regInfo.protection);

        // ── Find trailing alignment padding inside that region ──────────────
        // Read the last 64 KB and find the last non-zero byte, then bytes
        // after that are trailing padding.
        constexpr uint64_t kScanWindow = 0x10000;  // 64 KB
        uint64_t scanWindow =
            std::min(kScanWindow, codeRegEnd - codeRegStart);
        uint64_t scanStart = codeRegEnd - scanWindow;
        std::vector<uint8_t> tail(scanWindow, 0);
        if (!dbg.readMemory(scanStart, tail.data(), scanWindow)) {
            fprintf(stdout,
                    "M2: failed to read code-region tail at 0x%llx\n",
                    scanStart);
            return 1;
        }
        ssize_t lastNonZero = -1;
        for (ssize_t i = ssize_t(scanWindow) - 1; i >= 0; i--) {
            if (tail[i] != 0) {
                lastNonZero = i;
                break;
            }
        }
        if (lastNonZero < 0) {
            fprintf(stdout, "M2: code-region tail is all zeros, refusing\n");
            return 1;
        }
        uint64_t padStartOff = (uint64_t(lastNonZero + 1) + 3) & ~uint64_t(3);
        uint64_t padStartAddr = scanStart + padStartOff;
        uint64_t padBytes = codeRegEnd - padStartAddr;
        VERBOSE_LOG("M2: __TEXT trailing padding starts at 0x%llx, %llu bytes free\n",
            padStartAddr, padBytes);

        // ── Install Mach service port in parent ─────────────────────────────
        mach_port_t servicePort = MACH_PORT_NULL;
        uint32_t parentReqName = 0;
        uint32_t parentReplyName = 0;
        if (!sidecar::installPortInParent(dbg.taskPort(), &servicePort,
                                           &parentReqName,
                                           &parentReplyName)) {
            fprintf(stdout, "M2: sidecar::installPortInParent failed\n");
            return 1;
        }
        VERBOSE_LOG("M2: parent req port 0x%x  reply port 0x%x  local service 0x%x\n",
            parentReqName, parentReplyName, servicePort);

        // ── Assemble stub bytes ─────────────────────────────────────────────
        // OUR_HANDLER + STASH + STASH_JUMP go to padStartAddr.
        // ENTRY (16-byte abs-jump to OUR_HANDLER) goes to translate_insn[0..16].
        auto blobs = stub_asm::build(padStartAddr, translateInsnAddr,
                                      origPrologue, parentReqName,
                                      parentReplyName);
        if (blobs.entry.size() != 16) {
            fprintf(stdout,
                    "M2: stub_asm::build returned wrong entry size %zu\n",
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
        VERBOSE_LOG("M2: handler blob = %zu bytes (fits in %llu padding)\n",
            blobs.handler.size(), padBytes);

        // ── Write OUR_HANDLER + STASH + STASH_JUMP into trailing padding ───
        if (!dbg.adjustMemoryProtection(
                padStartAddr, VM_PROT_READ | VM_PROT_WRITE | VM_PROT_COPY,
                blobs.handler.size())) {
            fprintf(stdout, "M2: failed to make padding writable\n");
            return 1;
        }
        if (!dbg.writeMemory(padStartAddr, blobs.handler.data(),
                             blobs.handler.size())) {
            fprintf(stdout, "M2: failed to write handler blob\n");
            return 1;
        }
        if (!dbg.adjustMemoryProtection(padStartAddr,
                                        VM_PROT_READ | VM_PROT_EXECUTE,
                                        blobs.handler.size())) {
            fprintf(stdout, "M2: failed to restore padding protection\n");
            return 1;
        }
        VERBOSE_LOG("M2: handler installed at 0x%llx\n", padStartAddr);

        // ── Install transcendental-IPC trampoline ───────────────────────────
        // Lives in the same trailing pad, immediately following the
        // translate-time handler (16-byte aligned).  JIT-emitted code
        // BLRs into this for fsin/fcos/etc.; the trampoline does a
        // separate Mach IPC roundtrip (msgh_id=0x10000002) to the sidecar
        // which runs <cmath>'s sin/cos/etc. and replies with the result.
        const uint64_t transHelperAddr =
            (padStartAddr + blobs.handler.size() + 0xF) & ~uint64_t(0xF);
        std::vector<uint8_t> transBlob =
            stub_asm::buildTranscendentalHelper(parentReqName,
                                                parentReplyName);
        const uint64_t transHelperEnd = transHelperAddr + transBlob.size();
        if (transHelperEnd > padStartAddr + padBytes) {
            fprintf(stdout,
                    "M2: transcendental trampoline (%zu B) doesn't fit in "
                    "trailing padding (free %llu B after handler %zu B)\n",
                    transBlob.size(),
                    padBytes - blobs.handler.size(),
                    blobs.handler.size());
            return 1;
        }
        if (!dbg.adjustMemoryProtection(
                transHelperAddr, VM_PROT_READ | VM_PROT_WRITE | VM_PROT_COPY,
                transBlob.size())) {
            fprintf(stdout,
                    "M2: failed to make trampoline padding writable\n");
            return 1;
        }
        if (!dbg.writeMemory(transHelperAddr, transBlob.data(),
                             transBlob.size())) {
            fprintf(stdout, "M2: failed to write trampoline blob\n");
            return 1;
        }
        if (!dbg.adjustMemoryProtection(transHelperAddr,
                                        VM_PROT_READ | VM_PROT_EXECUTE,
                                        transBlob.size())) {
            fprintf(stdout,
                    "M2: failed to restore trampoline padding protection\n");
            return 1;
        }
        rosetta_core::set_transcendental_helper_addr(transHelperAddr);
        VERBOSE_LOG("M2: transcendental helper installed at 0x%llx (%zu B)\n",
                    transHelperAddr, transBlob.size());

        // ── Install transcendental polynomial constants ─────────────────────
        // 8 B aligned, immediately after the IPC trampoline.  JIT-emitted
        // inline code (translate_fsin etc.) loads coefficients via
        // `MOVZ/MOVK Xconst, addr; LDR Dt, [Xconst, #off]`.  Source values:
        // ARM-software/optimized-routines math/aarch64/advsimd/sin.c.
        const uint64_t constsAddr =
            (transHelperEnd + 0x7) & ~uint64_t(0x7);
        rosetta_core::TranscendentalConstants consts = {};
        consts.inv_pi = 0x1.45f306dc9c883p-2;
        consts.pi_1   = 0x1.921fb54442d18p+1;
        consts.pi_2   = 0x1.1a62633145c06p-53;
        consts.pi_3   = 0x1.c1cd129024e09p-106;
        consts.sin_c[0] = -0x1.555555555547bp-3;
        consts.sin_c[1] =  0x1.1111111108a4dp-7;
        consts.sin_c[2] = -0x1.a01a019936f27p-13;
        consts.sin_c[3] =  0x1.71de37a97d93ep-19;
        consts.sin_c[4] = -0x1.ae633919987c6p-26;
        consts.sin_c[5] =  0x1.60e277ae07cecp-33;
        consts.sin_c[6] = -0x1.9e9540300a1p-41;
        consts.range_val = 0x1p23;
        consts.half = 0.5;
        // f2xm1 polynomial coefficients (advsimd/exp2m1.c).
        consts.exp2m1_log2_hi    = 0x1.62e42fefa39efp-1;
        consts.exp2m1_log2_lo    = 0x1.abc9e3b39803f3p-56;
        consts.exp2m1_c1         = 0x1.ebfbdff82c58ep-3;
        consts.exp2m1_c2         = 0x1.c6b08d71f5804p-5;
        consts.exp2m1_c3         = 0x1.3b2ab6fee7509p-7;
        consts.exp2m1_c4         = 0x1.5d1d37eb33b15p-10;
        consts.exp2m1_c5         = 0x1.423f35f371d9ap-13;
        consts.exp2m1_c6         = 0x1.e7d57ad9a5f93p-5;
        consts.exp2m1_shift      = 0x1.8p45;
        consts.exp2m1_rnd2zero   = -0x1p-8;
        consts.exp2m1_tablebound = 0x1.5bfffffffffffp-2;
        consts.one               = 1.0;
        // 2^(j/128), j=0..127, biased-exponent form.  Source:
        // optimized-routines v_exp_data.c.
        static constexpr uint64_t kExpTable[128] = {
            0x3ff0000000000000ull, 0x3feff63da9fb3335ull, 0x3fefec9a3e778061ull,
            0x3fefe315e86e7f85ull, 0x3fefd9b0d3158574ull, 0x3fefd06b29ddf6deull,
            0x3fefc74518759bc8ull, 0x3fefbe3ecac6f383ull, 0x3fefb5586cf9890full,
            0x3fefac922b7247f7ull, 0x3fefa3ec32d3d1a2ull, 0x3fef9b66affed31bull,
            0x3fef9301d0125b51ull, 0x3fef8abdc06c31ccull, 0x3fef829aaea92de0ull,
            0x3fef7a98c8a58e51ull, 0x3fef72b83c7d517bull, 0x3fef6af9388c8deaull,
            0x3fef635beb6fcb75ull, 0x3fef5be084045cd4ull, 0x3fef54873168b9aaull,
            0x3fef4d5022fcd91dull, 0x3fef463b88628cd6ull, 0x3fef3f49917ddc96ull,
            0x3fef387a6e756238ull, 0x3fef31ce4fb2a63full, 0x3fef2b4565e27cddull,
            0x3fef24dfe1f56381ull, 0x3fef1e9df51fdee1ull, 0x3fef187fd0dad990ull,
            0x3fef1285a6e4030bull, 0x3fef0cafa93e2f56ull, 0x3fef06fe0a31b715ull,
            0x3fef0170fc4cd831ull, 0x3feefc08b26416ffull, 0x3feef6c55f929ff1ull,
            0x3feef1a7373aa9cbull, 0x3feeecae6d05d866ull, 0x3feee7db34e59ff7ull,
            0x3feee32dc313a8e5ull, 0x3feedea64c123422ull, 0x3feeda4504ac801cull,
            0x3feed60a21f72e2aull, 0x3feed1f5d950a897ull, 0x3feece086061892dull,
            0x3feeca41ed1d0057ull, 0x3feec6a2b5c13cd0ull, 0x3feec32af0d7d3deull,
            0x3feebfdad5362a27ull, 0x3feebcb299fddd0dull, 0x3feeb9b2769d2ca7ull,
            0x3feeb6daa2cf6642ull, 0x3feeb42b569d4f82ull, 0x3feeb1a4ca5d920full,
            0x3feeaf4736b527daull, 0x3feead12d497c7fdull, 0x3feeab07dd485429ull,
            0x3feea9268a5946b7ull, 0x3feea76f15ad2148ull, 0x3feea5e1b976dc09ull,
            0x3feea47eb03a5585ull, 0x3feea34634ccc320ull, 0x3feea23882552225ull,
            0x3feea155d44ca973ull, 0x3feea09e667f3bcdull, 0x3feea012750bdabfull,
            0x3fee9fb23c651a2full, 0x3fee9f7df9519484ull, 0x3fee9f75e8ec5f74ull,
            0x3fee9f9a48a58174ull, 0x3fee9feb564267c9ull, 0x3feea0694fde5d3full,
            0x3feea11473eb0187ull, 0x3feea1ed0130c132ull, 0x3feea2f336cf4e62ull,
            0x3feea427543e1a12ull, 0x3feea589994cce13ull, 0x3feea71a4623c7adull,
            0x3feea8d99b4492edull, 0x3feeaac7d98a6699ull, 0x3feeace5422aa0dbull,
            0x3feeaf3216b5448cull, 0x3feeb1ae99157736ull, 0x3feeb45b0b91ffc6ull,
            0x3feeb737b0cdc5e5ull, 0x3feeba44cbc8520full, 0x3feebd829fde4e50ull,
            0x3feec0f170ca07baull, 0x3feec49182a3f090ull, 0x3feec86319e32323ull,
            0x3feecc667b5de565ull, 0x3feed09bec4a2d33ull, 0x3feed503b23e255dull,
            0x3feed99e1330b358ull, 0x3feede6b5579fdbfull, 0x3feee36bbfd3f37aull,
            0x3feee89f995ad3adull, 0x3feeee07298db666ull, 0x3feef3a2b84f15fbull,
            0x3feef9728de5593aull, 0x3feeff76f2fb5e47ull, 0x3fef05b030a1064aull,
            0x3fef0c1e904bc1d2ull, 0x3fef12c25bd71e09ull, 0x3fef199bdd85529cull,
            0x3fef20ab5fffd07aull, 0x3fef27f12e57d14bull, 0x3fef2f6d9406e7b5ull,
            0x3fef3720dcef9069ull, 0x3fef3f0b555dc3faull, 0x3fef472d4a07897cull,
            0x3fef4f87080d89f2ull, 0x3fef5818dcfba487ull, 0x3fef60e316c98398ull,
            0x3fef69e603db3285ull, 0x3fef7321f301b460ull, 0x3fef7c97337b9b5full,
            0x3fef864614f5a129ull, 0x3fef902ee78b3ff6ull, 0x3fef9a51fbc74c83ull,
            0x3fefa4afa2a490daull, 0x3fefaf482d8e67f1ull, 0x3fefba1bee615a27ull,
            0x3fefc52b376bba97ull, 0x3fefd0765b6e4540ull, 0x3fefdbfdad9cbe14ull,
            0x3fefe7c1819e90d8ull, 0x3feff3c22b8f71f1ull,
        };
        memcpy(consts.exp_table, kExpTable, sizeof(kExpTable));
        // (2^(j/128) - 1) for j=0..43 (positive x), then j=-44..-1
        // (negative x, accessed via index offset 24).  Source:
        // optimized-routines exp2m1.c scalem1[].
        static constexpr uint64_t kExpScalem1[88] = {
            0x0000000000000000ull, 0x3f763da9fb33356eull, 0x3f864d1f3bc03077ull,
            0x3f90c57a1b9fe12full, 0x3f966c34c5615d0full, 0x3f9c1aca777db772ull,
            0x3fa0e8a30eb37901ull, 0x3fa3c7d958de7069ull, 0x3fa6ab0d9f3121ecull,
            0x3fa992456e48fee8ull, 0x3fac7d865a7a3440ull, 0x3faf6cd5ffda635eull,
            0x3fb1301d0125b50aull, 0x3fb2abdc06c31cc0ull, 0x3fb429aaea92ddfbull,
            0x3fb5a98c8a58e512ull, 0x3fb72b83c7d517aeull, 0x3fb8af9388c8de9cull,
            0x3fba35beb6fcb754ull, 0x3fbbbe084045cd3aull, 0x3fbd4873168b9aa8ull,
            0x3fbed5022fcd91ccull, 0x3fc031dc431466b2ull, 0x3fc0fa4c8beee4b1ull,
            0x3fc1c3d373ab11c3ull, 0x3fc28e727d9531faull, 0x3fc35a2b2f13e6e9ull,
            0x3fc426ff0fab1c05ull, 0x3fc4f4efa8fef709ull, 0x3fc5c3fe86d6cc80ull,
            0x3fc6942d3720185aull, 0x3fc7657d49f17ab1ull, 0x3fc837f0518db8a9ull,
            0x3fc90b87e266c18aull, 0x3fc9e0459320b7faull, 0x3fcab62afc94ff86ull,
            0x3fcb8d39b9d54e55ull, 0x3fcc6573682ec32cull, 0x3fcd3ed9a72cffb7ull,
            0x3fce196e189d4724ull, 0x3fcef5326091a112ull, 0x3fcfd228256400ddull,
            0x3fd0582887dcb8a8ull, 0x3fd0c7d76542a25bull, 0xbfcb23213cc8e86cull,
            0xbfca96ecd0deb7c4ull, 0xbfca09f58086c6c2ull, 0xbfc97c3a3cd7e119ull,
            0xbfc8edb9f5703dc0ull, 0xbfc85e7398737374ull, 0xbfc7ce6612886a6dull,
            0xbfc73d904ed74b33ull, 0xbfc6abf137076a8eull, 0xbfc61987b33d329eull,
            0xbfc58652aa180903ull, 0xbfc4f25100b03219ull, 0xbfc45d819a94b14bull,
            0xbfc3c7e359c9266aull, 0xbfc331751ec3a814ull, 0xbfc29a35c86a9b1aull,
            0xbfc20224341286e4ull, 0xbfc1693f3d7be6daull, 0xbfc0cf85bed0f8b7ull,
            0xbfc034f690a387deull, 0xbfbf332113d56b1full, 0xbfbdfaa500017c2dull,
            0xbfbcc0768d4175a6ull, 0xbfbb84935fc8c257ull, 0xbfba46f918837cb7ull,
            0xbfb907a55511e032ull, 0xbfb7c695afc3b424ull, 0xbfb683c7bf93b074ull,
            0xbfb53f391822dbc7ull, 0xbfb3f8e749b3e342ull, 0xbfb2b0cfe1266bd4ull,
            0xbfb166f067f25cfeull, 0xbfb01b466423250aull, 0xbfad9b9eb0a5ed76ull,
            0xbfaafd11874c009eull, 0xbfa85ae0438b37cbull, 0xbfa5b505d5b6f268ull,
            0xbfa30b7d271980f7ull, 0xbfa05e4119ea5d89ull, 0xbf9b5a991288ad16ull,
            0xbf95f134923757f3ull, 0xbf90804a4c683d8full, 0xbf860f9f985bc9f4ull,
            0xbf761eea3847077bull,
        };
        memcpy(consts.exp_scalem1, kExpScalem1, sizeof(kExpScalem1));
        const uint64_t constsEnd = constsAddr + sizeof(consts);
        if (constsEnd > padStartAddr + padBytes) {
            fprintf(stdout,
                    "M2: transcendental constants (%zu B) don't fit in "
                    "trailing padding (free %llu B after trampoline at 0x%llx)\n",
                    sizeof(consts),
                    padStartAddr + padBytes - transHelperEnd, transHelperEnd);
            return 1;
        }
        if (!dbg.adjustMemoryProtection(
                constsAddr, VM_PROT_READ | VM_PROT_WRITE | VM_PROT_COPY,
                sizeof(consts))) {
            fprintf(stdout,
                    "M2: failed to make constants padding writable\n");
            return 1;
        }
        if (!dbg.writeMemory(constsAddr, &consts, sizeof(consts))) {
            fprintf(stdout, "M2: failed to write transcendental constants\n");
            return 1;
        }
        // Restore RX, not just R: the trailing pad is page-granular RX and
        // the constants live on the same 4 KB page as the IPC trampoline
        // tail.  Stripping EXECUTE here would kill the trampoline.
        if (!dbg.adjustMemoryProtection(constsAddr,
                                        VM_PROT_READ | VM_PROT_EXECUTE,
                                        sizeof(consts))) {
            fprintf(stdout,
                    "M2: failed to restore constants padding protection\n");
            return 1;
        }
        rosetta_core::set_transcendental_constants_addr(constsAddr);
        VERBOSE_LOG("M2: transcendental constants installed at 0x%llx (%zu B)\n",
                    constsAddr, sizeof(consts));

        // ── DIAGNOSTIC (temporary): replace ENTRY with `mov x0, #0xCAFE; ret`
        // padded with nops so we can tell whether translate_insn is even
        // being called after our patch lands. If test_arith behaves
        // differently from a no-patch baseline, the patch IS being hit.
        if (getenv("ROSETTA_X87_DIAG_ENTRY_RET")) {
            blobs.entry.clear();
            // movz x0, #0xCAFE
            uint32_t movzInsn = 0xD2800000u | (uint32_t(0xCAFE) << 5) | 0;
            // ret  (= BR x30)
            uint32_t retInsn  = 0xD65F03C0u;
            uint32_t nopInsn  = 0xD503201Fu;
            for (uint32_t insn : {movzInsn, retInsn, nopInsn, nopInsn}) {
                blobs.entry.push_back(uint8_t(insn));
                blobs.entry.push_back(uint8_t(insn >> 8));
                blobs.entry.push_back(uint8_t(insn >> 16));
                blobs.entry.push_back(uint8_t(insn >> 24));
            }
            VERBOSE_LOG("M2: DIAG_ENTRY_RET active — entry replaced with mov x0,#0xCAFE; ret\n");
        }

        // ── Patch translate_insn[0..16] with the abs-jump ENTRY ────────────
        if (!dbg.adjustMemoryProtection(
                translateInsnAddr, VM_PROT_READ | VM_PROT_WRITE | VM_PROT_COPY,
                blobs.entry.size())) {
            fprintf(stdout, "M2: failed to make translate_insn writable\n");
            return 1;
        }
        if (!dbg.writeMemory(translateInsnAddr, blobs.entry.data(),
                             blobs.entry.size())) {
            fprintf(stdout, "M2: failed to write translate_insn entry\n");
            return 1;
        }
        if (!dbg.adjustMemoryProtection(translateInsnAddr,
                                        VM_PROT_READ | VM_PROT_EXECUTE,
                                        blobs.entry.size())) {
            fprintf(stdout, "M2: failed to restore translate_insn protection\n");
            return 1;
        }
        VERBOSE_LOG("M2: translate_insn entry patched (abs-jump to 0x%llx)\n",
            padStartAddr);

        // Read-back verification: confirm the patch actually landed, and
        // dump the FULL handler so we can decode each instruction post-mortem.
        uint8_t verifyEntry[16];
        if (dbg.readMemory(translateInsnAddr, verifyEntry, 16)) {
            VERBOSE_LOG("M2: post-patch translate_insn[0..16]: %02x%02x%02x%02x "
                "%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x\n",
                verifyEntry[0], verifyEntry[1], verifyEntry[2], verifyEntry[3],
                verifyEntry[4], verifyEntry[5], verifyEntry[6], verifyEntry[7],
                verifyEntry[8], verifyEntry[9], verifyEntry[10],
                verifyEntry[11], verifyEntry[12], verifyEntry[13],
                verifyEntry[14], verifyEntry[15]);
        }
        std::vector<uint8_t> verifyHandler(blobs.handler.size(), 0);
        if (dbg.readMemory(padStartAddr, verifyHandler.data(),
                            verifyHandler.size())) {
            VERBOSE_LOG("M2: handler full dump (%zu bytes), 4 insns/line:\n",
                verifyHandler.size());
            for (size_t i = 0; i < verifyHandler.size(); i += 16) {
                VERBOSE_LOG("  +0x%03zx: %02x%02x%02x%02x %02x%02x%02x%02x "
                    "%02x%02x%02x%02x %02x%02x%02x%02x\n",
                    i,
                    verifyHandler[i + 0], verifyHandler[i + 1],
                    verifyHandler[i + 2], verifyHandler[i + 3],
                    verifyHandler[i + 4], verifyHandler[i + 5],
                    verifyHandler[i + 6], verifyHandler[i + 7],
                    verifyHandler[i + 8], verifyHandler[i + 9],
                    verifyHandler[i + 10], verifyHandler[i + 11],
                    verifyHandler[i + 12], verifyHandler[i + 13],
                    verifyHandler[i + 14], verifyHandler[i + 15]);
            }
        }

        // Marker file so external smoke tests can confirm M2 actually ran.
        if (FILE* f = fopen("/tmp/rosettax87_jit_m2_marker", "w")) {
            fprintf(f, "translate_insn=0x%llx\n", translateInsnAddr);
            fprintf(f, "handler=0x%llx\n", padStartAddr);
            fprintf(f, "handler_bytes=%zu\n", blobs.handler.size());
            fprintf(f, "padding_bytes=%llu\n", padBytes);
            fprintf(f, "parent_req_name=0x%x\n", parentReqName);
            fprintf(f, "parent_reply_name=0x%x\n", parentReplyName);
            fclose(f);
        }

        // Capture the parent's task port BEFORE detach. The send-right is
        // held in MuhDebugger::taskPort_ and stays valid past dbg.detach()
        // (only ~MuhDebugger releases it). The receive thread will use it
        // for mach_vm_read on TranslationResult / IRInstr structs.
        mach_port_t parentTaskPort = dbg.taskPort();
        dbg.detach();

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
            kern_return_t kr =
                mach_msg(&selfMsg, MACH_SEND_MSG, sizeof(selfMsg), 0,
                         MACH_PORT_NULL, MACH_MSG_TIMEOUT_NONE,
                         MACH_PORT_NULL);
            VERBOSE_LOG("M2: self-test mach_msg send → 0x%x (%s)\n", kr,
                mach_error_string(kr));
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
        close(kq);
    }

    return 0;
}
