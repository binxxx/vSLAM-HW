cmake_minimum_required( VERSION 2.8 )
project( ps03 )

set( CMAKE_CXX_FLAGS "-std=c++11" )

include_directories("/usr/include/eigen3")

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable(ps03 E2Rt.cpp)
target_link_libraries(ps03 ${Sophus_LIBRARIES})
