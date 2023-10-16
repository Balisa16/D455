cmake_minimum_required(VERSION 3.10)

project(D455Sample)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(DEPENDENCIES realsense2)

find_package(glfw3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCES include/glad/glad.h)

add_executable(ex1 src/main.cpp)
target_link_libraries(ex1 PRIVATE realsense2 ${Pangolin_LIBRARIES} glfw ${PCL_LIBS} ${EIGEN3_LIBS})