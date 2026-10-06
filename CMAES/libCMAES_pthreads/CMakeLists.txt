cmake_minimum_required(VERSION 3.10)
project(cmaes_pthreads C)
include(GNUInstallDirs)

# C99, as required for the HLC code (PE-RNF-007). Extensions off gives
# -std=c99 rather than gnu99; the POSIX/GNU APIs come from _GNU_SOURCE below.
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_library(cmaes_pthreads STATIC
    src/Agent.c
    src/SysVars.c
    src/USER_DEF_COND.c
    src/CyclicBehaviour.c
    src/OneShotBehaviour.c
    src/MAES_Queue.c
    src/Agent_Msg.c
    src/Agent_Organization.c
    src/Agent_Platform.c
)

target_include_directories(cmaes_pthreads PUBLIC include)
target_link_libraries(cmaes_pthreads PUBLIC Threads::Threads)

# pthread_cancel/pthread_setschedparam and friends are GNU/POSIX extensions
# not exposed under the default strict-ISO feature test macros.
target_compile_definitions(cmaes_pthreads PUBLIC _GNU_SOURCE)

# install(TARGETS) must come after add_library: CMake looks the target up when
# the install() command runs and fails with "given target ... which does not
# exist" if it has not been created yet.
install(TARGETS cmaes_pthreads ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES include/CMAES.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cmaes)
