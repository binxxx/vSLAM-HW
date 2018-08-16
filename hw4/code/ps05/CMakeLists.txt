cmake_minimum_required( VERSION 2.8 )
project( ps05 )

set( CMAKE_CXX_FLAGS "-std=c++11" )

find_package( OpenCV )
include_directories(
        "/usr/include/eigen3"
        ${OpenCV_INCLUDE_DIRS}
)

add_executable( ps05 gaussnewton.cpp )
target_link_libraries( ps05
        ${OpenCV_LIBS})

find_package( Qt5 REQUIRED
        OpenGL
        Concurrent
        Test
        )