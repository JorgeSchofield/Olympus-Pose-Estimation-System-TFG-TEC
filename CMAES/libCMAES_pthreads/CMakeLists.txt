cmake_minimum_required(VERSION 3.10)
project(cmaes_pthreads C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

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
