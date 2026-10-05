#!/usr/bin/env python3
"""Check the cooperative unhooked fallback after a normal CMake build.

With X87_FORCE_UNSUPPORTED=1, `x87sidecar --cooperative <program>` must exec
the program in place without forking a sidecar: one warning banner on
stderr, the program's own output and exit status, the same pid, and none of
the variables the loader adds for a hooked launch. This uses the flat binary and
never attaches, so it needs no entitlements, root or authorization.
"""

import os
from pathlib import Path
import select
import signal
import subprocess
import sys


ROOT = Path(__file__).resolve().parent.parent
BIN = ROOT / "build" / "bin"
LOADER = BIN / "x87sidecar"
TESTS = ["test_arith", "test_fld", "test_fcom", "test_fxch"]
FALLBACK = "RUNNING WITHOUT X87 ACCELERATION"
LOADER_VARS = ("X87_SIDECAR_BOOTSTRAP", "ROSETTA_DISABLE_AOT")


def base_env():
    env = {key: value for key, value in os.environ.items()
           if not key.startswith("X87_") and key not in LOADER_VARS}
    return env


def run_watched(command, env):
    """Run command and record the execs and forks of its process.

    The process stops itself before exec'ing the loader, so the kqueue watch
    is in place before anything happens. Returns (returncode, stdout, stderr,
    pid, execs, forks).
    """
    wrapper = ["/bin/sh", "-c", 'kill -STOP $$; exec "$@"', "sh"] + command
    process = subprocess.Popen(wrapper, env=env, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True)
    try:
        while True:
            _, status = os.waitpid(process.pid, os.WUNTRACED)
            if os.WIFSTOPPED(status):
                break
        kq = select.kqueue()
        kq.control([select.kevent(
            process.pid, filter=select.KQ_FILTER_PROC,
            flags=select.KQ_EV_ADD | select.KQ_EV_CLEAR,
            fflags=select.KQ_NOTE_EXEC | select.KQ_NOTE_FORK | select.KQ_NOTE_EXIT)], 0)
        os.kill(process.pid, signal.SIGCONT)
        execs = forks = 0
        exited = False
        while not exited:
            events = kq.control(None, 8, 20)
            assert events, "timed out waiting for the target"
            for event in events:
                execs += bool(event.fflags & select.KQ_NOTE_EXEC)
                forks += bool(event.fflags & select.KQ_NOTE_FORK)
                exited |= bool(event.fflags & select.KQ_NOTE_EXIT)
        kq.close()
        stdout, stderr = process.communicate(timeout=20)
        return process.returncode, stdout, stderr, process.pid, execs, forks
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


def check_test_binary(name):
    binary = BIN / "tests" / name
    env = base_env()
    direct = subprocess.run([str(binary)], env=env, capture_output=True,
                            text=True, timeout=60)
    env["X87_FORCE_UNSUPPORTED"] = "1"
    code, stdout, stderr, _, execs, forks = run_watched(
        [str(LOADER), "--cooperative", str(binary)], env)
    assert stderr.count(FALLBACK) == 1, stderr
    assert code == direct.returncode, (code, direct.returncode, stdout)
    assert stdout == direct.stdout, stdout
    assert "PASS" in stdout and "FAIL" not in stdout, stdout
    # sh -> loader -> target, all in one process.
    assert execs == 2 and forks == 0, (execs, forks)
    print(f"ok  {name}: exit {code}, one process, warning banner on stderr")


def check_pid_env_and_status():
    env = base_env()
    env["X87_FORCE_UNSUPPORTED"] = "1"
    # A stale bootstrap name must not reach the target; an inherited
    # ROSETTA_DISABLE_AOT must arrive unchanged.
    env["X87_SIDECAR_BOOTSTRAP"] = "x87sidecar.1"
    env["ROSETTA_DISABLE_AOT"] = "inherited"
    script = ('echo "pid=$$"; echo "args=$*"; '
              'echo "bootstrap=${X87_SIDECAR_BOOTSTRAP-unset}"; '
              'echo "aot=${ROSETTA_DISABLE_AOT-unset}"; exit 3')
    code, stdout, stderr, pid, execs, forks = run_watched(
        [str(LOADER), "--cooperative", "/usr/bin/arch", "-x86_64", "/bin/sh",
         "-c", script, "sh", "a b", "c"], env)
    assert stderr.count(FALLBACK) == 1, stderr
    assert code == 3, (code, stdout, stderr)
    assert f"pid={pid}\n" in stdout, (pid, stdout)
    assert "args=a b c\n" in stdout, stdout
    assert "bootstrap=unset\n" in stdout, stdout
    assert "aot=inherited\n" in stdout, stdout
    assert forks == 0, forks
    print(f"ok  x86_64 sh: exit {code}, pid {pid} kept, argv and environment as passed")


def main():
    if not LOADER.exists():
        sys.exit(f"{LOADER} not found; run cmake --build build first")
    for name in TESTS:
        check_test_binary(name)
    check_pid_env_and_status()


if __name__ == "__main__":
    main()
