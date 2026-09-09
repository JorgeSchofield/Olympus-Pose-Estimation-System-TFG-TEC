cmake_minimum_required(VERSION 3.10)
project(CMAES_Linux C)
include(GNUInstallDirs)

# Native POSIX/pthreads port of the CMAES multi-agent framework (originally
# FreeRTOS-only) plus a smoke-test demo, for building on Linux / Raspberry
# Pi OS. The original FreeRTOS and libcsp backends under CMAES/ are left
# untouched and are not part of this build.

add_subdirectory(libCMAES_pthreads)
add_subdirectory(linux_demo)
