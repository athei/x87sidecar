# RosettaHack x87 + JIT

## Overview

An experimental project that hooks into Apple's Rosetta to replace x87 instruction handlers with faster implementations, and patches the translation pipeline to emit AArch64 instructions directly for improved performance.

## Prerequisites

- macOS 15 or later
- C compiler (clang)
- CMake

## Building

### Main Project

```
cmake -B build
cmake --build build
```

### Testing & Benchmarks

Tests and benchmarks are built automatically as part of the CMake build.

Run the test suite:
```bash
bash scripts/run_tests.sh              # build + test (native Rosetta & rosettax87)
bash scripts/run_tests.sh --no-build   # skip build
bash scripts/run_tests.sh --native-only # native Rosetta only
bash scripts/run_tests.sh test_arith   # run a specific test
```

Run benchmarks (compares native Rosetta, loader with optimizations disabled, and loader with full optimizations):
```bash
bash scripts/run_benchmarks.sh            # build + benchmark
bash scripts/run_benchmarks.sh --no-build # skip build
```

You will see a popup asking you to authorize debugging. Once approved, the process is granted a debug session.
Reference: [Debugging tool entitlement](https://developer.apple.com/documentation/bundleresources/entitlements/com.apple.security.cs.debugger)

Alternatively (Not Recommended), you can disable `Debugging Restrictions` part of System Integrity Protection (SIP) by running `csrutil enable --without debug` in macOS Recovery.

Warning: This reduces system security. NOT recommended.

## Configuration

All flags are set via environment variables and read at runtime.

### Optimization & Performance

| Variable | Description |
|----------|-------------|
| `X87_FAST_ROUND=1` | Skip rounding mode dispatch (faster but unsafe for FLDCW-heavy code) |

### Debugging & Troubleshooting

These flags are primarily useful for narrowing down bugs by selectively disabling features.

| Variable | Description |
|----------|-------------|
| `X87_DISABLE_CACHE=1` | Disable x87 translation cache |
| `X87_DISABLE_DEFERRED_FXCH=1` | Disable deferred FXCH optimization |
| `X87_DISABLE_X87_IR=1` | Disable IR optimization pipeline |
| `X87_DISABLE_ALL_FUSIONS=1` | Disable all instruction fusions |
| `X87_DISABLE_FUSIONS=f1,f2,...` | Disable specific fusions (comma-separated) |
| `X87_DISABLE_HOOK=1` | Skip the translate_insn entry patch (passthrough — apples-to-apples bench baseline) |
| `X87_LOGS=1` | Enable verbose logging output from the loader |

## Usage with Wine

> **Deprecated:** `wine@devel` removed support for the `ROSETTA_X87_PATH` environment variable, so the integration described below no longer works. There is no replacement at this time. The section is retained for historical reference.

### ~~Windows Applications~~

~~You can use the brew `wine@devel` cask with RosettaHack x87+JIT. It supports launching Windows applications through Wine with an environment variable `ROSETTA_X87_PATH`.~~

~~1. Install `wine@devel` using [Homebrew](https://brew.sh/)~~

~~`brew install --cask wine@devel`~~

~~2. To permanently set the environment variable, add the following to your `~/.bashrc` or `~/.zshrc` file:~~

~~`export ROSETTA_X87_PATH=/Path/To/rosettax87`~~

3. Run the Windows application

```bash
wine PATH_TO_BINARY.exe
```

## License

This project is licensed under `MIT`.
