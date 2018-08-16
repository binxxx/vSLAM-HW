cmake_minimum_required(VERSION 2.8)

project(ps02)
set( CMAKE_CXX_FLAGS "-std=c++11" )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

# find Qt
find_package( Qt5 REQUIRED
        OpenGL
        Test
        Concurrent)

add_executable( ps02 undistort_image.cpp)

target_link_libraries( ps02
        ${OpenCV_LIBS})