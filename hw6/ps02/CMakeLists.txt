cmake_minimum_required(VERSION 2.8)

project(ps02)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")


add_executable(ps02 optical_flow.cpp)
target_link_libraries(ps02 ${OpenCV_LIBS})