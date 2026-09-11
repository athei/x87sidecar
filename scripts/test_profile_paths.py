#!/usr/bin/env python3
"""Check concurrent profiler outputs on macOS after a normal CMake build."""

import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parent.parent
BIN = ROOT / "build" / "bin"


def check_paths(directory, name):
    sample = directory / name
    profile = directory / (name + ".x87")
    # Include the old window-series naming pattern: cleanup must operate on
    # the suffixed path, never on the shared configured path.
    sentinels = [Path(str(base) + suffix) for base in (sample, profile)
                 for suffix in ("", ".windows", ".0001", ".99999.windows")]
    for path in sentinels:
        path.write_bytes(b"preserve me\n")
    env = {key: value for key, value in os.environ.items()
           if not key.startswith("X87_") and key != "STORM_CASE"}
    env.update(X87_PROFILE=str(profile), X87_SAMPLE_REPORT="0.25")
    processes = []
    completed = set()
    try:
        # One environment-configured sampler and one CLI-configured sampler
        # use the same paths at the same time. The original loader PID becomes
        # the target PID across exec; the sidecar is a separate process.
        for use_env in (True, False):
            run_env = env.copy()
            command = [str(BIN / "x87sidecar_entitled")]
            if use_env:
                run_env["X87_SAMPLE"] = str(sample)
            else:
                command.append(f"--sample={sample}")
            command += [str(BIN / "tests" / "test_x87_signal_storm"), "none", "1"]
            processes.append(subprocess.Popen(
                command, env=run_env, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, start_new_session=True))
        for process in processes:
            # EOF also waits for the sidecar to finish its final counter dump.
            output, _ = process.communicate(timeout=20)
            completed.add(process.pid)
            assert process.returncode == 0, output
            assert "FAIL" not in output, output
            assert re.search(r"wrote [1-9][0-9]* block counters", output), output
            block_path = Path(f"{profile}.{process.pid}")
            sample_path = Path(f"{sample}.{process.pid}")
            assert block_path.stat().st_size > 0, output
            assert "[leaves]" in sample_path.read_text(), output
            assert "end_window " in Path(f"{sample_path}.windows").read_text(), output
        for path in sentinels:
            assert path.read_bytes() == b"preserve me\n", path
        expected = set(sentinels)
        for process in processes:
            expected.update((Path(f"{profile}.{process.pid}"),
                             Path(f"{sample}.{process.pid}"),
                             Path(f"{sample}.{process.pid}.windows")))
        assert set(directory.iterdir()) == expected, list(directory.iterdir())
        print(f"PASS {name}: two target PIDs, both profilers, windows and preserved files")
    finally:
        for process in processes:
            if process.pid not in completed:
                # Only this test's process group, including its detached sidecar.
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.communicate(timeout=5)


def main():
    with tempfile.TemporaryDirectory(prefix="x87-profile-paths-") as temporary:
        for index, name in enumerate(("game.prof", "profile", "literal-%p-%p.prof")):
            directory = Path(temporary) / str(index)
            directory.mkdir()
            check_paths(directory, name)


if __name__ == "__main__":
    main()
