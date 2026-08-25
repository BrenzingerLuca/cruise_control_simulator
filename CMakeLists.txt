cmake_minimum_required(VERSION 3.10)

# Set policy CMP0135 to NEW to handle timestamps for FetchContent (avoids warnings)
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

project(Cruise VERSION 1.0 LANGUAGES CXX)

# Global C++ Standard Configuration
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Locate Python3 components required for matplotlib-cpp integration
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Locate yaml-cpp for external configuration parsing
find_package(yaml-cpp REQUIRED)

# Encapsulate core logic into a static library to facilitate unit testing 
# and modularity. This library excludes the application entry point (main.cpp).
add_library(cruise_lib STATIC
    src/car.cpp
    src/pid.cpp
    src/csv.cpp
    src/visualizer.cpp
    src/simulation.cpp
    src/metrics.cpp
)

# Define include directories and propagate them to all dependent targets.
# The 'SYSTEM' flag suppresses warnings from third-party headers.
target_include_directories(cruise_lib SYSTEM PUBLIC include)

# Define preprocessor macros and compile-time definitions.
# PROJECT_ROOT_DIR is used for absolute path resolution within the simulation.
target_compile_definitions(cruise_lib PUBLIC
    PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}"
    WITHOUT_NUMPY
)

# Link shared dependencies to the core library.
# Dependencies are marked PUBLIC to ensure availability for linking executables.
target_link_libraries(cruise_lib PUBLIC 
    Python3::Python 
    yaml-cpp
)

# Define the primary executable and link it against the core library.
add_executable(cruise_control src/main.cpp)
target_link_libraries(cruise_control PRIVATE cruise_lib)

# Integrate GoogleTest framework using FetchContent for automated dependency management.
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/heads/main.zip
)

# Ensure GoogleTest is downloaded and configured.
# Force the use of shared runtime libraries on Windows if applicable.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Define the unit test runner and link against the core library and gtest_main.
# 'gtest_main' provides a default main() function for the test suite.
add_executable(run_unit_tests tests/pid_test.cpp tests/car_test.cpp tests/simulation_test.cpp tests/metrics_test.cpp)
target_link_libraries(run_unit_tests PRIVATE 
    cruise_lib 
    gtest_main
)