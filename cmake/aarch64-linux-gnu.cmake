# CMake toolchain file for cross-compiling R3D_cpp to the Jetson (AArch64).
#
#   cmake -B build-aarch64 -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-linux-gnu.cmake \
#         -DR3D_GPIO_BACKEND=LIBGPIOD
#   cmake --build build-aarch64 -j
#
# Verify the produced binary:
#   file build-aarch64/r3d_main   -> ELF 64-bit LSB executable, ARM aarch64

set(CMAKE_SYSTEM_NAME      Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CROSS_PREFIX aarch64-linux-gnu-)

set(CMAKE_C_COMPILER   ${CROSS_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${CROSS_PREFIX}g++)

# Standard Ubuntu cross sysroot (libc / libstdc++ / libpthread for the target).
set(CMAKE_SYSROOT /usr/aarch64-linux-gnu)
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})

# Only search headers/libraries under the target sysroot, but keep build tools
# (cmake, make) from the host.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
