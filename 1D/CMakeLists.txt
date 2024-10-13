cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(Kalman1D)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 REQUIRED)
find_package(matplotlib_cpp REQUIRED)
# add the executable
add_executable(Kalman1D ${CMAKE_SOURCE_DIR}/src/main.cpp ${CMAKE_SOURCE_DIR}/src/filter.cpp ${CMAKE_SOURCE_DIR}/include/filter.h)
target_include_directories(Kalman1D PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)


#for the purpose of making matplotlibcpp work
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(Kalman1D PRIVATE
    Python3::Python
    Python3::Module
    Python3::NumPy
)

