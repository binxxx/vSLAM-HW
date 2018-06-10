INCLUDE_DIRECTORIES(/usr/include/hello)
ADD_EXECUTABLE(main main.c)
TARGET_LINK_LIBRARIES(main hello)
# or 
# TARGET_LINK_LIBRARIES(main libhello.so)
# or link to the static lib
# TARGET_LINK_LIBRARIES(main libhello.a)