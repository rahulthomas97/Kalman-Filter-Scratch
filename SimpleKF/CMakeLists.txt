cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(Kalman1D)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
find_package(Eigen3 REQUIRED)
find_package(matplotlib_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core)
find_package(toml11 REQUIRED)
# add the executable
add_executable(Kalman1D
    "${CMAKE_SOURCE_DIR}/src/main.cpp"
    "${CMAKE_SOURCE_DIR}/src/filter.cpp"
    "${CMAKE_SOURCE_DIR}/src/helper.cpp"
    "${CMAKE_SOURCE_DIR}/include/filter.h"
    "${CMAKE_SOURCE_DIR}/include/helper.h")
target_include_directories(Kalman1D PUBLIC
    "${CMAKE_SOURCE_DIR}/include"
)

target_link_libraries(Kalman1D PRIVATE matplotlib_cpp::matplotlib_cpp toml11::toml11)


target_link_libraries(Kalman1D PRIVATE Qt5::Core)

set(CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/configFiles")
target_compile_definitions(Kalman1D PRIVATE CONFIG_DIR="${CONFIG_DIR}")
