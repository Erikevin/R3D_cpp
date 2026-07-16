cmake_minimum_required(VERSION 3.13)
project(r3d_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Threads REQUIRED)

# ---- GPIO backend (compile-time select) ------------------------------------
set(R3D_GPIO_BACKEND "MOCK" CACHE STRING "GPIO backend: MOCK | SYSFS | LIBGPIOD")
set_property(CACHE R3D_GPIO_BACKEND PROPERTY STRINGS MOCK SYSFS LIBGPIOD)
add_compile_definitions(R3D_GPIO_BACKEND_${R3D_GPIO_BACKEND})

# ---- Tier-A core library (no ROS) ------------------------------------------
add_library(r3d_core STATIC
    src/gpio.cpp
    src/hardware.cpp
    src/serial.cpp
    src/serial_protocol.cpp
    src/protection.cpp
    src/ipc.cpp
    src/system.cpp
    src/bridge_launcher.cpp
)
target_include_directories(r3d_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(r3d_core PUBLIC Threads::Threads)

# Optional libgpiod link (header presence is auto-detected in gpio.cpp, so a
# missing header silently falls back to MOCK at compile time).
if(R3D_GPIO_BACKEND STREQUAL "LIBGPIOD")
    find_library(R3D_LIBGPIOD gpiod)
    if(R3D_LIBGPIOD)
        target_link_libraries(r3d_core PUBLIC ${R3D_LIBGPIOD})
    else()
        message(WARNING "libgpiod not found; GPIO will fall back to MOCK.")
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
#   cmake -B build-aarch64 -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-linux-gnu.cmake -DR3D_GPIO_BACKEND=LIBGPIOD
# LIBGPIOD needs libgpiod-dev:aarch64 in the sysroot; without it gpio.cpp falls
# back to MOCK at compile time (via __has_include).
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    message(STATUS "R3D: cross-compiling for aarch64 — GPIO backend=${R3D_GPIO_BACKEND}")
endif()

# ---- Tier-B ROS 2 package --------------------------------------------------
# Built separately inside ros2_ws/ with colcon (see ros2_ws/src/r3d_bringup) —
# NOT part of this standalone Tier-A build.
