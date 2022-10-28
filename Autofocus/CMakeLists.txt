cmake_minimum_required(VERSION 3.10)
project(Autofocus)

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

ADD_EXECUTABLE(Autofocus main.cpp Image_Process.h Image_Process.cpp Strategy.h)
target_link_libraries(Autofocus ${OpenCV_LIBRARIES})
