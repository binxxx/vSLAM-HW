cmake_minimum_required(VERSION 3.5)

project(hello)
include_directories(${PROJECT_SOURCE_DIR}/include)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(hello SHARED src/hello.cpp)

add_executable(sayhello useHello.cpp)
target_link_libraries(sayhello hello)
