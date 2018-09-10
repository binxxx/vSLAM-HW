cmake_minimum_required(VERSION 2.8)
project(ps04)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

add_executable(ps04 disparity_compare.cpp)
target_link_libraries(ps04 ${OpenCV_LIBS} ${Sophus_LIBRARIES})