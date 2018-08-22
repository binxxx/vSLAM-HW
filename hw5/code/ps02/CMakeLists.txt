cmake_minimum_required( VERSION 2.8 )

project( ps02 )

set( CMAKE_CXX_FLAGS "-std=c++11" )

find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( ps02 computeORB.cpp )
target_link_libraries( ps02 ${OpenCV_LIBS} )

find_package( Qt5 REQUIRED
        OpenGL
        Concurrent
        Test
        )