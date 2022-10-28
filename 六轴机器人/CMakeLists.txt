cmake_minimum_required(VERSION 3.10)
project(Six)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories("C:/Program Files (x86)/Eigen3/include/eigen3")

add_executable(Six main.cpp)
