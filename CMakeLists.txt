cmake_minimum_required(VERSION 3.0)

project(D455Sample)

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

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(DEPENDENCIES realsense2)

include_directories(
    ${PCL_INCLUDE_DIRS}
    thirdparty/glfw/include
    thirdparty/glm/glm
    thirdparty/librealsense/include
    ${PROJECT_SOURCE_DIR}/include)

add_library(ADDED_LIBS STATIC
    src/callback.cc
    src/tcp_client.cc
    src/window.cc
    src/device.cc
    src/shader.cc
    src/UART.cc
    src/mat.cc
    src/glad.c
    src/stb_image.cc
    thirdparty/jsoncpp/src/lib_json/json_reader.cpp
    thirdparty/jsoncpp/src/lib_json/json_value.cpp
    thirdparty/jsoncpp/src/lib_json/json_writer.cpp
    thirdparty/jsoncpp/src/jsontestrunner/main.cpp)

link_directories(
    ${PCL_LIBRARY_DIRS}
    thirdparty/jsoncpp/build/lib
)
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(ADDED_LIBS PRIVATE 
    glfw
    pthread
    jsoncpp
    Boost::system 
    Boost::filesystem)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/release)

file(GLOB SRC_FILES test/*.cpp)
message("\nDetect target")
foreach(SRC_FILE ${SRC_FILES})
    get_filename_component(TARGET_NAME ${SRC_FILE} NAME_WE)

    message("\tFound : ${TARGET_NAME}")

    add_executable(${TARGET_NAME} ${SRC_FILE})
    target_link_libraries(${TARGET_NAME} PRIVATE
        glfw 
        glm
        ADDED_LIBS
        ${PCL_LIBRARIES}
        realsense2)
    target_include_directories(${TARGET_NAME} PRIVATE
        thirdparty/glfw/include)
endforeach()
message("")
