cmake_minimum_required(VERSION 3.10)

project(D455Sample)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(DEPENDENCIES realsense2)

find_package(glfw3 REQUIRED)
find_package(Pangolin 0.8 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

set(SOURCES include/glad/glad.h)

add_executable(ex1 src/main.cpp)
target_link_libraries(ex1 PRIVATE realsense2 pango_display OpenGL GL GLU glfw ${PCL_LIBRARIES})