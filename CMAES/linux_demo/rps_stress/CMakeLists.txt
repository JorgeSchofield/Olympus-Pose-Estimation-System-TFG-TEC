add_executable(cmaes_rps_stress_demo Main.c)
target_link_libraries(cmaes_rps_stress_demo PRIVATE cmaes_pthreads)
install(TARGETS cmaes_rps_stress_demo RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
