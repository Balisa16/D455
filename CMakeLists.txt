cmake_minimum_required(VERSION 3.10)

project(D455Sample)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_subdirectory(thirdparty/glfw)
add_subdirectory(thirdparty/glm)
add_subdirectory(thirdparty/librealsense)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(DEPENDENCIES realsense2)

include_directories(
    thirdparty/glfw/include
    thirdparty/glfw/src
    thirdparty/glm/glm
    thirdparty/librealsense/include
    include
    src)

add_library(ADDED_LIBS STATIC
    src/window.cc
    src/shader.cc
    src/glad.c
    src/stb_image.cc)

set(SOURCES
    test/testlib.cpp
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

add_executable(testlib ${SOURCES})
target_link_libraries(testlib PRIVATE glfw glm realsense2)
target_include_directories(testlib PRIVATE thirdparty/glfw/include)


add_executable(main test/main.cpp)
target_link_libraries(main PRIVATE glfw glm realsense2 ADDED_LIBS)
target_include_directories(main PRIVATE
    thirdparty/glfw/include
    thirdparty/glfw/src
)