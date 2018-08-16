cmake_minimum_required( VERSION 2.8 )
project( ps03 )

set( CMAKE_CXX_FLAGS "-std=c++11" )

find_package( Pangolin )
include_directories(
        ${Pangolin_INCLUDE_DIRS}
        "/usr/include/eigen3" )

find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( Qt5 REQUIRED
        OpenGL
        Concurrent
        Test
        )

add_executable( ps03 disparity.cpp)
target_link_libraries( ps03
        ${Pangolin_LIBRARIES}
        ${OpenCV_LIBS})