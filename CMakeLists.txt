cmake_minimum_required(VERSION 3.14)
project(rov_depth_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(rovsim
    src/rov_model.cpp
    src/pressure_sensor.cpp
    src/pid_controller.cpp
    src/battery_model.cpp
    src/safety_monitor.cpp
    src/simulation.cpp)
target_include_directories(rovsim PUBLIC include)
if(NOT MSVC)
    target_compile_options(rovsim PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(run_sim src/main.cpp)
target_link_libraries(run_sim PRIVATE rovsim)

# ---- Tests (GoogleTest) ----
option(ROVSIM_BUILD_TESTS "Build unit tests" ON)
if(ROVSIM_BUILD_TESTS)
    enable_testing()
    find_package(GTest QUIET)
    if(NOT GTest_FOUND)
        include(FetchContent)
        FetchContent_Declare(googletest
            URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
            DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(googletest)
    endif()

    add_executable(rovsim_tests
        tests/test_rov_model.cpp
        tests/test_pressure_sensor.cpp
        tests/test_pid_controller.cpp
        tests/test_battery_model.cpp
        tests/test_safety_monitor.cpp
        tests/test_closed_loop.cpp)
    target_link_libraries(rovsim_tests PRIVATE rovsim GTest::gtest GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(rovsim_tests)
endif()
