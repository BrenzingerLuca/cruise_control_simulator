cmake_minimum_required(VERSION 3.10)
project(Cruise VERSION 1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
add_executable(cruise_control src/main.cpp src/car.cpp src/pid.cpp src/csv.cpp src/input_validator.cpp src/visualizer.cpp)
target_compile_definitions(cruise_control PRIVATE
    PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}"
)