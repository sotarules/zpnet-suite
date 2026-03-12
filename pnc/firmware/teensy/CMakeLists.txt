cmake_minimum_required(VERSION 3.22)

# =============================================================
# ZPNet Teensy 4.1 — CLion Indexing Project
#
# PURPOSE:
#   • This CMake project exists ONLY for CLion code insight
#   • It does NOT replace the real Arduino CLI / Makefile build
#   • Authoritative build + flash remains on the Raspberry Pi
#
# REAL BUILD:
#   • make build
#   • make flash
#
# DESIGN:
#   • Mirror important compile-time defines from makefile
#   • Present the firmware sources as a single executable target
#   • Provide enough structure for navigation / completion
#   • Suppress local warning noise because this is NOT the real build
# =============================================================

project(zpnet_teensy_index LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -------------------------------------------------------------
# Configuration mirrored from makefile
# -------------------------------------------------------------

set(ZPNET_USB_MODE "serial" CACHE STRING "USB mode mirrored from makefile")
set(ZPNET_BOARD "teensy41" CACHE STRING "Board identifier for documentation only")

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
    process_clocks.cpp
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
)

# -------------------------------------------------------------
# Target
# -------------------------------------------------------------
#
# EXCLUDE_FROM_ALL keeps this target from acting like a normal
# "real" build target in the local environment.
# -------------------------------------------------------------

add_executable(zpnet_teensy_index EXCLUDE_FROM_ALL
    ${ZPNET_SOURCES}
    ${ZPNET_HEADERS}
)

# Treat the .ino as C++ for CLion's benefit.
set_source_files_properties(teensy.ino PROPERTIES LANGUAGE CXX)

# -------------------------------------------------------------
# Compile-time defines mirrored from makefile
# -------------------------------------------------------------
#
# These extra Arduino/Teensy-style defines are here purely to
# reduce parser confusion and IDE noise.
# -------------------------------------------------------------

target_compile_definitions(zpnet_teensy_index PRIVATE
    ZPNET_TRANSPORT_SERIAL
    ARDUINO=10819
    TEENSYDUINO=160
    ARDUINO_TEENSY41
    __IMXRT1062__
)

# -------------------------------------------------------------
# Include paths
# -------------------------------------------------------------
#
# For your current tree, local headers are in this directory.
# Add more include paths here later if needed.
# -------------------------------------------------------------

target_include_directories(zpnet_teensy_index PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# -------------------------------------------------------------
# Warning suppression
# -------------------------------------------------------------
#
# This target exists only for indexing/navigation. We suppress
# warning noise aggressively because the authoritative build is
# still the Raspberry Pi Makefile/Arduino CLI path.
# -------------------------------------------------------------

if (MSVC)
    target_compile_options(zpnet_teensy_index PRIVATE
        /W0
        /wd4068
        /wd4100
        /wd4189
        /wd4200
        /wd4244
        /wd4267
        /wd4305
        /wd4324
        /wd4505
        /wd4996
    )
else()
    target_compile_options(zpnet_teensy_index PRIVATE
        -w
    )
endif()

# -------------------------------------------------------------
# Target properties
# -------------------------------------------------------------
#
# Keep CLion centered on parsing/indexing rather than pretending
# this is a meaningful host-built firmware executable.
# -------------------------------------------------------------

set_target_properties(zpnet_teensy_index PROPERTIES
    LINKER_LANGUAGE CXX
)

# -------------------------------------------------------------
# Notes
# -------------------------------------------------------------
#
# Real build:
#   make build
#
# Real flash:
#   make flash
#
# This file exists only to give CLion a usable project model.
# =============================================================