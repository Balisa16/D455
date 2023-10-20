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
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src)

add_library(ADDED_LIBS STATIC
    src/callback.cc
    src/window.cc
    src/shader.cc
    src/glad.c
    src/stb_image.cc)

target_link_libraries(ADDED_LIBS PRIVATE glfw)

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
        realsense2)
    target_include_directories(${TARGET_NAME} PRIVATE
        thirdparty/glfw/include
    )
endforeach()
message("")
