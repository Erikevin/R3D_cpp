cmake_minimum_required(VERSION 3.13)
project(r3d_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Threads REQUIRED)

# ---- GPIO backend (wiringPi, Orange Pi) ------------------------------------
# When libwiringPi is found, gpio.cpp is compiled with USE_WIRINGPI; otherwise
# it falls back to the logging Mock backend (so host builds still work).
option(USE_WIRINGPI "Enable wiringPi GPIO backend (Orange Pi)" ON)

# ---- Tier-A core library (no ROS) ------------------------------------------
add_library(r3d_core STATIC
    src/gpio.cpp
    src/hardware.cpp
    src/serial.cpp
    src/serial_protocol.cpp
    src/protection.cpp
    src/log_rotator.cpp
    src/performance_monitor.cpp
    src/ipc.cpp
    src/system.cpp
    src/bridge_launcher.cpp
)
target_include_directories(r3d_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(r3d_core PUBLIC Threads::Threads)

# wiringPi discovery — when found, compile gpio.cpp with USE_WIRINGPI and link
# libwiringPi; otherwise gpio.cpp uses the Mock backend (host development).
if(USE_WIRINGPI)
    find_library(WIRINGPI_LIB wiringPi
        PATHS /usr/lib /usr/local/lib
              /usr/lib/arm-linux-gnueabihf
              /usr/lib/aarch64-linux-gnu)
    find_path(WIRINGPI_INCLUDE_DIR wiringPi.h
        PATHS /usr/include /usr/local/include)
    if(WIRINGPI_LIB AND WIRINGPI_INCLUDE_DIR)
        message(STATUS "✅ 找到 wiringPi: ${WIRINGPI_LIB}")
        target_compile_definitions(r3d_core PUBLIC USE_WIRINGPI)
        target_include_directories(r3d_core PUBLIC ${WIRINGPI_INCLUDE_DIR})
        target_link_libraries(r3d_core PUBLIC ${WIRINGPI_LIB})
    else()
        message(WARNING "⚠️  未找到 wiringPi，GPIO 使用 Mock 后端")
    endif()
endif()

# ---- executables -----------------------------------------------------------
add_executable(r3d_main      src/main.cpp)
target_link_libraries(r3d_main PRIVATE r3d_core)

add_executable(r3d_daemon    src/daemon.cpp)
target_link_libraries(r3d_daemon PRIVATE r3d_core Threads::Threads)

add_executable(r3d_send_cmd  src/send_cmd.cpp)
target_link_libraries(r3d_send_cmd PRIVATE r3d_core)

# ---- Cross-compile (aarch64 / Jetson) --------------------------------------
# Configure with the toolchain file (see scripts/cross-aarch64.sh):
#   cmake -B build-aarch64 -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-linux-gnu.cmake -DUSE_WIRINGPI=ON
# wiringPi needs libwiringPi in the sysroot; without it gpio.cpp falls back to
# the Mock backend at compile time.
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    message(STATUS "R3D: cross-compiling for aarch64 — USE_WIRINGPI=${USE_WIRINGPI}")
endif()

# ---- Tier-B ROS 2 package --------------------------------------------------
# Built separately inside ros2_ws/ with colcon (see ros2_ws/src/r3d_bringup) —
# NOT part of this standalone Tier-A build.
