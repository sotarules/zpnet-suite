cmake_minimum_required(VERSION 3.22)

# =============================================================
# Cross-compilation: bare-metal ARM (skip link test)
# =============================================================
#
# The ARM cross-compiler cannot link a native executable, so
# CMake's compiler test would fail without these settings.
# CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY tells CMake to
# test compilation only (no link step).
# =============================================================

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# =============================================================
# ZPNet Teensy 4.1 — CLion Indexing Project
#
# PURPOSE:
#   This CMake project exists ONLY for CLion code insight.
#   It does NOT replace the real Arduino CLI / Makefile build.
#   Authoritative build + flash remains on the Raspberry Pi.
#
# REAL BUILD:
#   make build       (on Pi)
#   make flash       (on Pi)
#
# TOOLCHAIN:
#   CLion remote toolchain pointed at the Pi's ARM cross-compiler:
#     ~/.arduino15/packages/teensy/tools/teensy-compile/11.3.1/arm/bin/
#   Teensy core headers resolved from the Pi's Arduino installation:
#     ~/.arduino15/packages/teensy/hardware/avr/1.59.0/cores/teensy4/
# =============================================================

project(zpnet_teensy_index LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -------------------------------------------------------------
# Teensy core paths (on the Pi)
# -------------------------------------------------------------

set(TEENSY_ROOT "$ENV{HOME}/.arduino15/packages/teensy/hardware/avr/1.59.0")
set(TEENSY_CORE "${TEENSY_ROOT}/cores/teensy4")
set(TEENSY_WIRE "${TEENSY_ROOT}/libraries/Wire")
set(TEENSY_SPI  "${TEENSY_ROOT}/libraries/SPI")

set(ADC_LIB "$ENV{HOME}/Arduino/libraries/ADC")

# -------------------------------------------------------------
# Source set
# -------------------------------------------------------------

set(ZPNET_SOURCES
    bootloader.cpp
    build_fingerprint.cpp
    cpu_usage.cpp
    debug.cpp
    memory_info.cpp
    payload.cpp
    process.cpp
    process_clocks_alpha.cpp
    process_clocks_beta.cpp
    process_events.cpp
    process_laser.cpp
    process_performance.cpp
    process_photodiode.cpp
    process_pubsub.cpp
    process_system.cpp
    process_timepop.cpp
    publish.cpp
    teensy.cpp
    timebase.cpp
    transport.cpp
    util.cpp
    teensy.ino
    time.cpp
    ad5693r.cpp
    process_interrupt.cpp
)

set(ZPNET_HEADERS
    bootloader.h
    config.h
    cpu_usage.h
    debug.h
    events.h
    memory_info.h
    payload.h
    process.h
    process_clocks.h
    process_clocks_internal.h
    process_events.h
    process_laser.h
    process_performance.h
    process_photodiode.h
    process_pubsub.h
    process_system.h
    process_timepop.h
    publish.h
    timebase.h
    timepop.h
    transport.h
    util.h
    zpnet_pins.h
    time.h
    tdc_correction.h
    ad5693r.h
    process_interrupt.h
)

# -------------------------------------------------------------
# Target
# -------------------------------------------------------------

add_executable(zpnet_teensy_index EXCLUDE_FROM_ALL
    ${ZPNET_SOURCES}
    ${ZPNET_HEADERS}
)

set_source_files_properties(teensy.ino PROPERTIES LANGUAGE CXX)

# -------------------------------------------------------------
# Compile-time defines
# -------------------------------------------------------------
#
# These match what the Arduino CLI passes during a real build.
# They enable the correct code paths in Teensy core headers
# (imxrt.h, core_pins.h, etc.).
# -------------------------------------------------------------

target_compile_definitions(zpnet_teensy_index PRIVATE
    ZPNET_TRANSPORT_SERIAL
    ARDUINO=10819
    TEENSYDUINO=160
    ARDUINO_TEENSY41
    __IMXRT1062__
    F_CPU=600000000
)

# -------------------------------------------------------------
# Include paths
# -------------------------------------------------------------
#
# The Teensy core directory is where all the hardware register
# definitions live: imxrt.h (GPT1_CNT, IMXRT_TMR1, ARM_DWT_CYCCNT),
# core_pins.h (digitalWriteFast, attachInterrupt, pinMode),
# Arduino.h, Wire.h, etc.
# -------------------------------------------------------------

target_include_directories(zpnet_teensy_index PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${TEENSY_CORE}
    ${TEENSY_WIRE}
    ${TEENSY_SPI}
    ${ADC_LIB}
)

# -------------------------------------------------------------
# Warning suppression
# -------------------------------------------------------------
#
# This target exists only for indexing/navigation.  Suppress
# all warnings — the authoritative build is the Pi makefile.
# -------------------------------------------------------------

target_compile_options(zpnet_teensy_index PRIVATE -w)

# -------------------------------------------------------------
# Target properties
# -------------------------------------------------------------

set_target_properties(zpnet_teensy_index PROPERTIES
    LINKER_LANGUAGE CXX
)

# -------------------------------------------------------------
# Notes
# -------------------------------------------------------------
#
# Real build:   make build   (on Pi, uses Arduino CLI)
# Real flash:   make flash   (on Pi, uses teensy_loader_cli)
#
# This file exists only to give CLion a usable project model.
# The remote toolchain resolves headers from the Pi filesystem.
# =============================================================