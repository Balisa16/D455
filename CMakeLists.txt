cmake_minimum_required(VERSION 3.10)

project(D455Sample VERSION 1.0.0)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

add_subdirectory(thirdparty/glfw)
add_subdirectory(thirdparty/glm)
add_subdirectory(thirdparty/librealsense)
add_subdirectory(thirdparty/jsoncpp)
# add_subdirectory(thirdparty/gtest)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(DEPENDENCIES realsense2)

include_directories(
    ${PCL_INCLUDE_DIRS}
    thirdparty/glfw/include
    thirdparty/glm/glm
    thirdparty/librealsense/include
    thirdparty/jsoncpp/include
    ${PROJECT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} STATIC
    src/callback.cc
    src/writer_pcd.cc
    src/tcp_client.cc
    src/window.cc
    src/device.cc
    src/shader.cc
    src/UART.cc
    src/mat.cc
    src/glad.c
    src/stb_image.cc)

link_directories(
    ${PCL_LIBRARY_DIRS}
    # thirdparty/jsoncpp/build/lib
)

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} PRIVATE 
    glfw
    pthread
    jsoncpp
    Boost::system 
    Boost::filesystem)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/release/test)

add_subdirectory(example)