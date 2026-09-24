# ZPNet Teensy 4.1 -- fast development builds and optional diagnostics.
#
#   make                          Fast incremental build, all available CPU cores.
#   make MODE=diagnostic          Clean build with full diagnostic reports.
#   make PROFILE=noalias          Incremental build with -fno-strict-aliasing.
#   make MODE=diagnostic PROFILE=noalias  Clean aliasing investigation build.
#   make PROFILE=debug            Incremental -Og optimization experiment.
#   make JOBS=2                   Limit concurrent compiler processes.
#   make CPU=1008                 Explicit ZPNet 1.008 GHz board selection.
#   make flash                    Build once, then flash that archived image.
#   make flash-only               Flash the last successful archive; never build.
#   make unused                   Existing dead-code audit reports.
#
# MODE and PROFILE are independent: MODE controls build cost; PROFILE controls
# optimization. Arduino CLI reuses objects at the fixed build/objects path and
# rebuilds them when the board or compiler overrides change. Diagnostic mode
# always rebuilds. Every successful build archives its exact ELF/HEX and hashes.
# clean/clobber retain build-history/. HalfKay and services belong to flash-teensy.

SHELL := /bin/bash
.SHELLFLAGS := -eu -o pipefail -c
.ONESHELL:
.DEFAULT_GOAL := all
.NOTPARALLEL:

NAME := teensy
ARDUINO_CLI ?= ../../bin/arduino-cli
TEENSY_LOADER ?= teensy_loader_cli
USB_MODE := serial
# Keep all menu selections in one FQBN; expand CPU after its default below.
BOARD = teensy:avr:teensy41:usb=$(USB_MODE),speed=$(CPU)
BUILD_DIR := $(CURDIR)/build
HISTORY_DIR := $(CURDIR)/build-history

MODE ?= fast
PROFILE ?= baseline
JOBS ?= 0
# Match maximize_dwt_determinism() in teensy.cpp. CPU selects compile-time
# F_CPU; a different runtime speed also requires changing that function.
CPU ?= 1008
STACK_WARN ?= 2048
EXTRA_CFLAGS ?=
EXTRA_CXXFLAGS ?=
AUDIT_GC ?= 0
IMAGE_DIR ?=

# These produce diagnostics/debug information; they add no runtime checks.
# The warnings are investigative leads, not proof of a defect or a clean bill
# of health. Stack reports describe individual functions, not nested call/ISR
# totals. No -Werror: installed core/library warnings remain visible.
DIAG_FLAGS := -g3 -fstack-usage -Wall -Wextra -Wformat=2 \
              -Wstrict-aliasing=3 -Wcast-align=strict \
              -Warray-bounds=2 -Wstringop-overflow=2 \
              -Wuninitialized -Wmaybe-uninitialized \
              -Wframe-larger-than=$(STACK_WARN) -Wstack-usage=$(STACK_WARN) \
              -fdiagnostics-color=never
CXX_DIAG_FLAGS := -Wclass-memaccess -Wplacement-new=2

all: build

build:
	@case '$(MODE)' in fast|diagnostic) ;; *)
	    echo 'MODE must be fast or diagnostic.' >&2; exit 2;;
	esac
	case '$(PROFILE)' in baseline|noalias|debug) ;; *)
	    echo 'PROFILE must be baseline, noalias, or debug.' >&2; exit 2;;
	esac
	if [[ '$(AUDIT_GC)' == 1 && '$(MODE)' != diagnostic ]]; then
	    echo 'AUDIT_GC=1 requires MODE=diagnostic (or make unused).' >&2; exit 2
	fi
	mkdir -p '$(BUILD_DIR)' '$(HISTORY_DIR)'
	# A failed new build must not leave an older image selected for flash-only.
	rm -f '$(BUILD_DIR)/last-successful-build.txt'
	# Retire only generated reports, so a failure archive cannot inherit old data.
	rm -f '$(BUILD_DIR)'/{base-properties,build-properties,build-info,build-command,build-log,symbols,disassembly,stack-usage,diagnostics}.txt
	rm -f '$(BUILD_DIR)/teensy.map'
	archive="$$(mktemp -d '$(HISTORY_DIR)'/"$$(date -u +%Y%m%dT%H%M%SZ)-$(MODE)-$(PROFILE)-XXXXXX")"
	printf 'Build evidence: %s\n' "$$archive"
	# Keep failure transcripts too. No failure can reach the flashing stage.
	trap 'rc=$$?; if (( rc != 0 )); then cp -f "$(BUILD_DIR)"/*.txt "$$archive/" 2>/dev/null || :; printf "Build failed; evidence: %s\n" "$$archive" >&2; fi' EXIT
	cli=('$(ARDUINO_CLI)' compile --fqbn '$(BOARD)' \
	     --build-path '$(BUILD_DIR)/objects' --output-dir '$(BUILD_DIR)' --jobs '$(JOBS)')
	# An explicit build path keeps core objects here too; CLI's build.options.json
	# invalidates this working cache when the FQBN or custom build properties change.
	# Read the INSTALLED platform's flags before appending our diagnostics.
	# Teensy consumes build.flags.c/cpp/ld; generic compiler.*.extra_flags are
	# not used by the verified Teensy platform. Preserve its ABI, C++ standard,
	# libraries and linker script rather than hardcoding replacements.
	"$${cli[@]}" --show-properties=expanded . > '$(BUILD_DIR)/base-properties.txt'
	declare -A props
	while IFS= read -r line; do
	    [[ "$$line" == *=* ]] || continue
	    props["$${line%%=*}"]="$${line#*=}"
	done < '$(BUILD_DIR)/base-properties.txt'
	opt="$${props[build.flags.optimize]}"
	case '$(PROFILE)' in
	    noalias) opt+=' -fno-strict-aliasing' ;;
	    debug)   opt+=' -Og' ;;
	esac
	c_flags="$${props[build.flags.c]}"
	cpp_flags="$${props[build.flags.cpp]}"
	ld_flags="$${props[build.flags.ld]}"
	if [[ '$(MODE)' == diagnostic ]]; then
	    c_flags+=' $(DIAG_FLAGS)'
	    cpp_flags+=' $(DIAG_FLAGS) $(CXX_DIAG_FLAGS)'
	    ld_flags+=' "-Wl,-Map=$(BUILD_DIR)/teensy.map" -Wl,--cref'
	    if [[ '$(AUDIT_GC)' == 1 ]]; then ld_flags+=' -Wl,--print-gc-sections'; fi
	fi
	cli+=(--build-property "build.flags.optimize=$$opt" \
	      --build-property "build.flags.c=$$c_flags $(EXTRA_CFLAGS)" \
	      --build-property "build.flags.cpp=$$cpp_flags $(EXTRA_CXXFLAGS)" \
	      --build-property "build.flags.ld=$$ld_flags")
	if [[ '$(MODE)' == diagnostic ]]; then
	    "$${cli[@]}" --show-properties=expanded . > '$(BUILD_DIR)/build-properties.txt'
	    cli+=(--clean --verbose --warnings all)
	fi
	cxx="$${props[compiler.path]}$${props[build.toolchain]}$${props[build.command.g++]}"
	tool_prefix="$${cxx%g++}"
	{
	    printf 'UTC=%s\nMODE=%s\nPROFILE=%s\nJOBS=%s\nBOARD=%s\nCPU_MENU_OVERRIDE=%s\n' \
	        "$$(date -u +%FT%TZ)" '$(MODE)' '$(PROFILE)' '$(JOBS)' '$(BOARD)' '$(CPU)'
	    printf 'F_CPU=%s\nOPTIMIZATION_FLAGS=%s\n' "$${props[build.fcpu]}" "$$opt"
	    '$(ARDUINO_CLI)' version
	    "$$cxx" --version
	    if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
	        printf '\nGit HEAD: '; git rev-parse HEAD
	        git status --short -- .
	        git diff --binary HEAD -- . > "$$archive/source-changes.patch"
	    fi
	} > '$(BUILD_DIR)/build-info.txt'
	printf '%q ' "$${cli[@]}" . > '$(BUILD_DIR)/build-command.txt'
	printf '\n' >> '$(BUILD_DIR)/build-command.txt'
	cat '$(BUILD_DIR)/build-info.txt'
	# Fast builds keep unchanged objects; diagnostic builds explicitly use --clean.
	"$${cli[@]}" . 2>&1 | tee '$(BUILD_DIR)/build-log.txt'
	test -s '$(BUILD_DIR)/$(NAME).ino.elf'
	test -s '$(BUILD_DIR)/$(NAME).ino.hex'
	if [[ '$(MODE)' == diagnostic ]]; then
	    test -s '$(BUILD_DIR)/teensy.map'
	    "$${tool_prefix}nm" -n -S -C '$(BUILD_DIR)/$(NAME).ino.elf' > '$(BUILD_DIR)/symbols.txt'
	    "$${tool_prefix}objdump" -d -C -l '$(BUILD_DIR)/$(NAME).ino.elf' > '$(BUILD_DIR)/disassembly.txt'
	    find '$(BUILD_DIR)/objects' -type f -name '*.su' -exec cat {} + \
	        | LC_ALL=C sort -t $$'\t' -k2,2nr > '$(BUILD_DIR)/stack-usage.txt'
	    # Empty output means the stack-report option did not reach compilation.
	    test -s '$(BUILD_DIR)/stack-usage.txt'
	    cp '$(BUILD_DIR)/teensy.map' "$$archive/"
	    (
	        cd '$(BUILD_DIR)/objects'
	        find . -type f -name '*.su' -print0 | tar --null -T - -czf "$$archive/stack-usage.tar.gz"
	    )
	fi
	grep -E '(warning:|error:)' '$(BUILD_DIR)/build-log.txt' > '$(BUILD_DIR)/diagnostics.txt' || :
	cp '$(BUILD_DIR)/$(NAME).ino.elf' '$(BUILD_DIR)/$(NAME).ino.hex' \
	   '$(BUILD_DIR)'/*.txt "$$archive/"
	(
	    cd "$$archive"
	    sha256sum '$(NAME).ino.elf' '$(NAME).ino.hex' > SHA256SUMS
	)
	printf '%s\n' "$$archive" > '$(BUILD_DIR)/last-successful-build.txt'
	trap - EXIT
	printf '\nSuccessful build: %s\n' "$$archive"
	cat "$$archive/SHA256SUMS"
	if [[ '$(MODE)' == diagnostic ]]; then
	    printf '\nLargest individual stack frames (bytes; not call-chain totals):\n'
	    head -n 15 '$(BUILD_DIR)/stack-usage.txt'
	fi

# flash is convenient for direct use; flash-teensy calls flash-only instead.
# Keeping the flash action in the recipe prevents make -j from racing the build.
flash: build
	@$(MAKE) --no-print-directory flash-only IMAGE_DIR="$$(cat '$(BUILD_DIR)/last-successful-build.txt')"

flash-only:
	@image_dir='$(IMAGE_DIR)'
	if [[ -z "$$image_dir" ]]; then image_dir="$$(cat '$(BUILD_DIR)/last-successful-build.txt')"; fi
	cd "$$image_dir"
	sha256sum --check SHA256SUMS
	printf 'Flashing archived image: %s\n' "$$image_dir/$(NAME).ino.hex"
	sudo '$(TEENSY_LOADER)' -mmcu=TEENSY41 -w "$$image_dir/$(NAME).ino.hex"

# Keep the original audit interface and output names. Use the same build path
# and installed linker flags as ordinary builds, with GC diagnostics appended.
unused:
	@$(MAKE) --no-print-directory build MODE=diagnostic AUDIT_GC=1
	cp '$(BUILD_DIR)/build-log.txt' unused-full.txt
	cp '$(BUILD_DIR)/teensy.map' teensy.map
	grep -q -- '--print-gc-sections' unused-full.txt
	test -s teensy.map
	grep -E '\[-Wunused-(function|variable|const-variable|but-set-variable)[^]]*\]' \
	    unused-full.txt > unused-compiler.txt || :
	grep 'removing unused section' unused-full.txt > unused-linker.txt || :
	grep '/sketch/' unused-linker.txt | grep -E '\.(text|data|bss|rodata)\.' \
	    > unused-linker-project.txt || :
	{
	    echo 'COMPILER UNUSED DIAGNOSTICS'
	    cat unused-compiler.txt
	    echo
	    echo 'PROJECT SECTIONS DISCARDED BY LINKER GC'
	    cat unused-linker-project.txt
	} > unused.txt
	cat unused.txt

clean:
	rm -rf '$(BUILD_DIR)'
	rm -f teensy.map unused.txt unused-compiler.txt unused-linker.txt \
	      unused-linker-project.txt unused-full.txt
	@echo 'Working build removed; build-history retained.'

clobber: clean
	@echo 'Working objects are gone; the next build starts fresh. Global Arduino packages are retained.'

.PHONY: all build flash flash-only unused clean clobber
