cmake_minimum_required(VERSION 3.10)

project(d455 VERSION 1.0.0)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

add_subdirectory(thirdparty/glfw)
add_subdirectory(thirdparty/glm)
add_subdirectory(thirdparty/librealsense)
add_subdirectory(thirdparty/jsoncpp)
# add_subdirectory(thirdparty/gtest)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(DEPENDENCIES realsense2)


add_library(${PROJECT_NAME} SHARED
src/writer_pcd.cc
src/tcp_client.cc
src/tcpserver.cc
src/device.cc
src/UART.cc
src/mat.cc)

message("Found PCL: ${PCL_INCLUDE_DIRS}")

target_include_directories(
    ${PROJECT_NAME} PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    /usr/include/eigen3
    # FOR Jetson Nano
    /usr/include/pcl-1.8

    # FOR Ubuntu Desktop
    # /usr/include/pcl-1.10
    
    thirdparty/glfw/include
    thirdparty/glm/glm
    thirdparty/librealsense/include
    thirdparty/jsoncpp/include
    ${PROJECT_SOURCE_DIR}/include)

link_directories(
    ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} PRIVATE 
    glfw
    pthread
    jsoncpp
    Eigen3::Eigen
    Boost::system 
    Boost::filesystem)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/release/test)

add_subdirectory(example)