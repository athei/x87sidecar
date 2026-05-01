# Top-level wrapper for build, format, lint, and test workflows.
# All real work happens via cmake / clang-format / clang-tidy / scripts/.
# Run `make` (or `make help`) for a target list.

BUILD_DIR    := build
CMAKE        ?= cmake

# Prefer the Homebrew LLVM toolchain when available — clang-tidy needs to
# match the host triple (arm64 macOS), and a Windows-targeting llvm-mingw
# binary in PATH will choke on system headers. Override via env if needed:
#   make tidy CLANG_TIDY=/some/other/clang-tidy
HOMEBREW_LLVM_BIN := /opt/homebrew/opt/llvm/bin
CLANG_FORMAT ?= $(shell test -x $(HOMEBREW_LLVM_BIN)/clang-format && \
                       echo $(HOMEBREW_LLVM_BIN)/clang-format || echo clang-format)
CLANG_TIDY   ?= $(shell test -x $(HOMEBREW_LLVM_BIN)/clang-tidy && \
                       echo $(HOMEBREW_LLVM_BIN)/clang-tidy || echo clang-tidy)

# Every tracked C/C++ source or header (any depth). git ls-files's `*`
# matches `/`, so a single pattern covers all subdirectories.
FORMAT_FILES := $(shell git ls-files '*.c' '*.cpp' '*.h' '*.hpp')

# Host-code translation units only. The x86_64 .c samples in tests/ and
# benchmarks/ are excluded — clang-tidy mostly flags noise on them.
TIDY_FILES := $(shell git ls-files \
    'rosetta_core/*.cpp' \
    'rosetta_loader/*.cpp' \
    'rosetta_config/*.cpp' \
    'aotinvoke/*.cpp')

.PHONY: help build configure format format-check tidy tidy-fix \
        test bench fusion-sweep clean

help:
	@echo "Targets:"
	@echo "  build         - configure and build (cmake -B build && cmake --build build)"
	@echo "  configure     - cmake configure only (re-emits compile_commands.json)"
	@echo "  format        - clang-format -i on all tracked C/C++ sources"
	@echo "  format-check  - dry-run format check; non-zero exit if anything is dirty"
	@echo "  tidy          - clang-tidy diagnostics on host C++ TUs"
	@echo "  tidy-fix      - clang-tidy --fix on host C++ TUs"
	@echo "  test          - scripts/run_tests.sh"
	@echo "  bench         - scripts/run_benchmarks.sh"
	@echo "  fusion-sweep  - scripts/run_fusion_sweep.sh"
	@echo "  clean         - rm -rf $(BUILD_DIR)"

build: configure
	$(CMAKE) --build $(BUILD_DIR)

configure:
	$(CMAKE) -B $(BUILD_DIR)

format:
	$(CLANG_FORMAT) -i $(FORMAT_FILES)

format-check:
	$(CLANG_FORMAT) --dry-run -Werror $(FORMAT_FILES)

# Apple's /usr/bin/c++ adds the macOS SDK implicitly, so the entries in
# compile_commands.json don't carry -isysroot. Homebrew clang-tidy isn't
# Apple, so it can't find <cstdint> & friends without being told. Inject
# the active SDK path via --extra-arg.
TIDY_EXTRA := --extra-arg=-isysroot$(shell xcrun --show-sdk-path)

tidy: configure
	$(CLANG_TIDY) -p $(BUILD_DIR) $(TIDY_EXTRA) $(TIDY_FILES)

tidy-fix: configure
	$(CLANG_TIDY) -p $(BUILD_DIR) $(TIDY_EXTRA) --fix --fix-errors $(TIDY_FILES)

test:
	scripts/run_tests.sh

bench:
	scripts/run_benchmarks.sh

fusion-sweep:
	scripts/run_fusion_sweep.sh

clean:
	rm -rf $(BUILD_DIR)
