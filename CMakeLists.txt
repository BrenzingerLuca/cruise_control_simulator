cmake_minimum_required(VERSION 3.10)
project(Cruise VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the executable
add_executable(cruise_control 
    src/main.cpp 
    src/car.cpp 
    src/pid.cpp 
    src/csv.cpp 
    src/input_validator.cpp 
    src/visualizer.cpp
    src/simulation.cpp
)

# Tell the target where the headers are 
target_include_directories(cruise_control SYSTEM PUBLIC include)

target_compile_definitions(cruise_control PRIVATE
    PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}"
)

# Find python3. Needed for matplotlib
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(cruise_control PRIVATE Python3::Python)
target_compile_definitions(cruise_control PRIVATE WITHOUT_NUMPY)

# Find YAML needed for config loading 
find_package(yaml-cpp REQUIRED)
target_link_libraries(cruise_control PRIVATE yaml-cpp Python3::Python)