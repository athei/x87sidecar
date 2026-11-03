#!/usr/bin/env python3
"""Check that the default attach leaves its target running after the release.

A detach from a held SIGSTOP stop leaves the target stopped on some kernels:
it sits in state T, the sidecar waits for it to exit, and the launch hangs
without output. This runs test_fldconst under the entitled loader, three times
per mode, and watches for that stop as the target's real parent.

    python3 scripts/test_release_not_stopped.py hook
    python3 scripts/test_release_not_stopped.py passthrough
    python3 scripts/test_release_not_stopped.py sigcont-blocked

hook runs with the translation hook, passthrough with X87_DISABLE_HOOK=1, and
sigcont-blocked with the hook and SIGCONT blocked in the inherited signal mask.
That last path leaves the target traced, so it checks only the exit status and
the output.

Exits 0 on a pass, 1 on a wrong result, 2 when the target stopped and 124 when
a run timed out. It uses the default attach, so it needs the same privileges
as run_tests.sh.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


ROOT = Path(__file__).resolve().parent.parent
BIN = ROOT / "build" / "bin"
LOADER = BIN / "x87sidecar_entitled"
BINARY = BIN / "tests" / "test_fldconst"
MODES = ("hook", "passthrough", "sigcont-blocked")
RUNS = 3
# Seconds for one run, the same bound run_tests.sh gives run_bounded.
TIMEOUT = float(os.environ.get("TEST_TIMEOUT") or 60)
# Seconds for a process group to go after its SIGKILL.
KILL_WAIT = 5

EXIT_PASS = 0
EXIT_WRONG_RESULT = 1
EXIT_STOPPED = 2
EXIT_TIMEOUT = 124


def watch(process, mode, deadline):
    """Poll the target until it exits, stops or runs out of time.

    Returns EXIT_PASS once it has exited, with process.returncode set. This
    does not check the result of the run.

    PT_ATTACH reparents the target to the sidecar, so this process gets ECHILD
    for the whole traced window. Popen.poll() and Popen.wait() turn ECHILD into
    status 0, so neither is called here. A stop is the only failure: in a tiny
    window wait4 can report W_STOPCODE(SIGCONT), which Darwin decodes as
    continued.
    """
    while True:
        if time.monotonic() > deadline:
            return EXIT_TIMEOUT
        try:
            pid, status = os.waitpid(
                process.pid, os.WNOHANG | os.WUNTRACED | os.WCONTINUED)
        except ChildProcessError:
            pid = 0  # still the sidecar's child
        if pid and os.WIFSTOPPED(status):
            if mode != "sigcont-blocked":
                return EXIT_STOPPED
        elif pid and not os.WIFCONTINUED(status):
            process.returncode = os.waitstatus_to_exitcode(status)
            return EXIT_PASS
        time.sleep(0.01)


def reap(process):
    """Collect the target's exit status once it is available."""
    if process.returncode is not None:
        return
    try:
        pid, status = os.waitpid(process.pid, os.WNOHANG)
    except ChildProcessError:
        return
    if pid:
        process.returncode = os.waitstatus_to_exitcode(status)


def kill_group(group):
    try:
        os.killpg(group, signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
        pass


def group_is_gone(group):
    try:
        os.killpg(group, 0)
    except (ProcessLookupError, PermissionError):
        # kill(2) can report EPERM when only zombies are left in the group.
        return True
    return False


def wait_for_group(process, deadline):
    """Return whether the target is reaped and its group empty by deadline."""
    while time.monotonic() <= deadline:
        reap(process)
        if process.returncode is not None and group_is_gone(process.pid):
            return True
        time.sleep(0.01)
    return False


def finish(process, deadline):
    """End the run, so that two sidecars never overlap.

    The sidecar exits once it sees the target exit, so after an exit it gets
    the rest of the run's time, as under run_bounded. Anything else gets
    SIGKILL for the whole group, because a traced target can survive SIGKILL
    until its tracer is gone. Returns whether nothing of the run is left.
    """
    if process.returncode is not None and wait_for_group(process, deadline):
        return True
    kill_group(process.pid)
    return wait_for_group(process, time.monotonic() + KILL_WAIT)


def run_once(mode, env):
    """Run the target once and return the exit code for that run."""
    # A temporary file, not a pipe: the sidecar holds stdout past the target's
    # exit. The target has to stay this process's child, so run_bounded, which
    # would put perl in between, is not used; watch and finish keep its
    # contract: a session of its own, TEST_TIMEOUT and a SIGKILL of the group.
    with tempfile.TemporaryFile() as out:
        process = subprocess.Popen([str(LOADER), str(BINARY)], env=env, stdout=out,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + TIMEOUT
        try:
            result = watch(process, mode, deadline)
        finally:
            finished = finish(process, deadline)
        out.seek(0)
        output = out.read().decode(errors="replace")

    reason = None
    if result == EXIT_STOPPED:
        reason = "target stopped after the release"
    elif result == EXIT_TIMEOUT:
        reason = f"timed out after {TIMEOUT:g} s"
    elif not finished:
        reason = "process group still present after SIGKILL"
        result = EXIT_WRONG_RESULT
    elif process.returncode != 0:
        reason = f"target exited with status {process.returncode}"
        result = EXIT_WRONG_RESULT
    elif "PASS" not in output or "FAIL" in output:
        reason = "output has no PASS line or has a FAIL line"
        result = EXIT_WRONG_RESULT
    if reason is not None:
        print(output, end="" if output.endswith("\n") else "\n")
        print(reason)
    return result


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in MODES:
        sys.exit(f"usage: {sys.argv[0]} {'|'.join(MODES)}")
    mode = sys.argv[1]
    for path in (LOADER, BINARY):
        if not path.exists():
            sys.exit(f"{path} not found; run cmake --build build first")
    env = {key: value for key, value in os.environ.items()
           if not key.startswith("X87_")}
    if mode == "passthrough":
        env["X87_DISABLE_HOOK"] = "1"
    elif mode == "sigcont-blocked":
        signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGCONT})
    for run in range(1, RUNS + 1):
        result = run_once(mode, env)
        if result != EXIT_PASS:
            print(f"{mode}: run {run} of {RUNS} failed")
            return result
    print(f"ok  {mode}: {RUNS} runs exited 0 and printed PASS")
    return EXIT_PASS


if __name__ == "__main__":
    sys.exit(main())
