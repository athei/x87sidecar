#!/usr/bin/env python3
"""Run a command with a deadline and capture its processes before cleanup."""

import argparse
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time


def capture(path, command, timeout=10):
    try:
        result = subprocess.run(command, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=timeout)
        path.write_bytes(result.stdout)
        return result.stdout.decode(errors="replace")
    except subprocess.TimeoutExpired as error:
        path.write_bytes((error.stdout or b"") + b"\nDiagnostic command timed out\n")
    except OSError as error:
        path.write_text(str(error) + "\n")
    return ""


def processes(group):
    result = subprocess.run(
        ["ps", "-axo", "pid=,ppid=,pgid=,stat=,etime=,command="],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5)
    result.check_returncode()
    return [line for line in result.stdout.splitlines()
            if len(line.split()) >= 3 and line.split()[2] == str(group)]


def diagnose(output, group):
    try:
        rows = processes(group)
    except (OSError, subprocess.SubprocessError) as error:
        (output / "processes.txt").write_text(str(error) + "\n")
        return
    (output / "processes.txt").write_text("\n".join(rows) + "\n")
    print("PID PPID PGID STAT ELAPSED COMMAND", flush=True)
    for row in rows:
        print(row, flush=True)
    # The double-forked sidecar keeps its process group after launchd adopts
    # it. Looking only for descendants of the harness would miss that process.
    for row in rows:
        pid = row.split()[0]
        capture(output / f"fds-{pid}.txt", ["lsof", "-a", "-p", pid, "-d", "0,1,2"])
        print(f"Sampling pid {pid}", flush=True)
        capture(output / f"sample-{pid}-status.txt",
                ["sample", pid, "1", "1", "-file", str(output / f"sample-{pid}.txt")])


def kill_group(group):
    try:
        os.killpg(group, signal.SIGKILL)
    except ProcessLookupError:
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--idle-timeout", type=float, default=90)
    parser.add_argument("--total-timeout", type=float, default=600)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command
    if command[:1] == ["--"]:
        command = command[1:]
    if not command or args.idle_timeout <= 0 or args.total_timeout <= 0:
        parser.error("a command and positive timeouts are required")
    output = args.output_dir.resolve()
    # Do not overwrite evidence from an earlier invocation.
    output.mkdir(parents=True)
    capture(output / "os-version.txt", ["sw_vers"])
    capture(output / "kernel.txt", ["uname", "-a"])
    (output / "command.json").write_text(json.dumps(command) + "\n")

    started = last_output = last_progress = time.monotonic()
    process = subprocess.Popen(command, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, start_new_session=True)
    print(f"Watching pid/group {process.pid}; evidence: {output}", flush=True)
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    eof = False
    reason = "completed"
    status_before_cleanup = None
    try:
        with (output / "output.log").open("wb", buffering=0) as log:
            while True:
                status_before_cleanup = process.poll()
                if eof and status_before_cleanup is not None:
                    break
                now = time.monotonic()
                if now - started >= args.total_timeout:
                    reason = "total_timeout"
                    break
                if now - last_output >= args.idle_timeout:
                    reason = "idle_timeout"
                    break
                if now - last_progress >= 15:
                    print(f"Still watching pid/group {process.pid}: "
                          f"exit={status_before_cleanup}, stdout_eof={eof}, "
                          f"idle={now - last_output:.1f}s", flush=True)
                    last_progress = now
                for key, _ in selector.select(0.1):
                    data = os.read(key.fd, 65536)
                    if data:
                        last_output = time.monotonic()
                        log.write(data)
                        sys.stdout.buffer.write(data)
                        sys.stdout.buffer.flush()
                    else:
                        eof = True
                        selector.unregister(key.fileobj)
        if reason != "completed":
            print(f"Diagnostic failure: {reason}; direct exit="
                  f"{status_before_cleanup}, stdout_eof={eof}", flush=True)
            diagnose(output, process.pid)
    finally:
        # Only this invocation's group is owned by the watchdog. Capture its
        # state before ending it, including when the original PID exited but
        # an orphan still holds the output pipe open.
        if reason != "completed" or sys.exc_info()[0] is not None:
            kill_group(process.pid)
        selector.close()
        process.stdout.close()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            reason = "cleanup_timeout"
            print("Process did not exit after cleanup", flush=True)

    (output / "result.json").write_text(json.dumps({
        "pid": process.pid,
        "reason": reason,
        "exit_before_cleanup": status_before_cleanup,
        "returncode": process.returncode,
        "stdout_eof": eof,
        "seconds": time.monotonic() - started,
    }, indent=2) + "\n")
    if reason != "completed":
        return 124
    return process.returncode if process.returncode >= 0 else 128 - process.returncode


if __name__ == "__main__":
    sys.exit(main())
