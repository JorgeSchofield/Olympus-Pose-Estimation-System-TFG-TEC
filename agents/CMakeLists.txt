cmake_minimum_required(VERSION 3.10)
project(olympus_pose C)

# olympus-pose: CMAES multi-agent pose estimation for the Olympus HLC
# (agents/DESIGN.md). C99 everywhere (PE-RNF-007); only src/port and the
# CMAES library touch the OS.
#
#   cmake -S agents -B build && cmake --build build          # RPi / Linux
#   cmake -S agents -B build-core -DPE_CORE_ONLY=ON           # any host: core + tests
#   cmake -S agents -B build -DPE_BUILD_TESTS=ON && ctest     # Linux with tests

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
include(GNUInstallDirs)

option(PE_CORE_ONLY    "Build only the portable core and its tests (no Linux, no CMAES)" OFF)
option(PE_BUILD_TESTS  "Build the unit tests" OFF)
if(PE_CORE_ONLY)
    set(PE_BUILD_TESTS ON)
endif()

set(PE_WARN -Wall -Wextra -Werror)

# --- portable core: parsers, statistics, datagram, parameters, generated EKF
file(GLOB PE_GEN_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/core/gen/*.c)
add_library(pe_core STATIC
    src/core/params.c
    src/core/raw_parser.c
    src/core/nmea_parser.c
    src/core/stats.c
    src/core/pose_dgram.c
    src/core/line_buf.c
    ${PE_GEN_SOURCES}
)
target_include_directories(pe_core PUBLIC include src/core src/core/gen)
target_compile_options(pe_core PRIVATE ${PE_WARN})
if(NOT WIN32)
    target_link_libraries(pe_core PUBLIC m)
endif()

if(NOT PE_CORE_ONLY)
    find_package(Threads REQUIRED)

    # CMAES built from the same commit as the application (DESIGN.md §15.3)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../CMAES/libCMAES_pthreads
                     ${CMAKE_CURRENT_BINARY_DIR}/cmaes EXCLUDE_FROM_ALL)

    add_library(pe_port STATIC src/port/port_io.c src/port/port_log.c)
    target_include_directories(pe_port PUBLIC src/port)
    target_compile_definitions(pe_port PRIVATE _GNU_SOURCE)
    target_compile_options(pe_port PRIVATE ${PE_WARN})
    target_link_libraries(pe_port PUBLIC Threads::Threads)

    add_executable(olympus-pose
        src/main.c
        src/agent_acquisition.c
        src/agent_fusion.c
        src/agent_estimation.c
        src/agent_comm.c
        src/agent_gps.c
    )
    target_include_directories(olympus-pose PRIVATE src)
    target_compile_options(olympus-pose PRIVATE ${PE_WARN})
    target_link_libraries(olympus-pose PRIVATE pe_core pe_port cmaes_pthreads)

    add_executable(llcmux tools/llcmux/llcmux.c)
    target_compile_definitions(llcmux PRIVATE _GNU_SOURCE)
    target_compile_options(llcmux PRIVATE ${PE_WARN})
    target_link_libraries(llcmux PRIVATE pe_core pe_port)

    install(TARGETS olympus-pose llcmux RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(FILES config/pose.conf DESTINATION ${CMAKE_INSTALL_SYSCONFDIR}/olympus-pose)
    install(PROGRAMS tools/pose_listen.py tools/pose_ctl.py tools/llc_emulator.py
            DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

if(PE_BUILD_TESTS)
    enable_testing()
    add_executable(pe_tests tests/test_core.c)
    target_compile_options(pe_tests PRIVATE ${PE_WARN})
    target_compile_definitions(pe_tests PRIVATE
        PE_TEST_DATA="${CMAKE_CURRENT_SOURCE_DIR}/tests/data"
        PE_TEST_CONF="${CMAKE_CURRENT_SOURCE_DIR}/config/pose.conf")
    target_link_libraries(pe_tests PRIVATE pe_core)
    add_test(NAME pe_tests COMMAND pe_tests)
endif()
