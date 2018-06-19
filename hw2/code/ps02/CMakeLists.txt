cmake_minimum_required(VERSION 2.8)

project(ps02)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS )

include_directories("/usr/include/eigen3")

add_executable(ps02 ps02.cpp)