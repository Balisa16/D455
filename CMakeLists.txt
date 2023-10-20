cmake_minimum_required(VERSION 3.10)

project(D455Sample)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_subdirectory(thirdparty/glfw)
add_subdirectory(thirdparty/glm)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(DEPENDENCIES realsense2)

set(SOURCES
    test/testlib.cpp
)

add_executable(test1 ${SOURCES})

target_link_libraries(test1 PRIVATE glfw glm)

target_include_directories(test1 PRIVATE thirdparty/glfw/include)


# find_package(OpenGL REQUIRED)
# find_package(X11 REQUIRED)
# find_package(Eigen3 REQUIRED)

# include_directories(
#     ${CMAKE_CURRENT_SOURCE_DIR}/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
#     ${CMAKE_CURRENT_SOURCE_DIR}/deps
#     ${CMAKE_SOURCE_DIR}/include/glm
#     )

# set(MYLIBS 
#     ${GLFW3_LIBRARY} 
#     glfw
#     X11 
#     Xrandr GL dl pthread
#     # ${ASSIMP_LIBRARY}
#     # ${Pangolin_LIBRARIES}
#     # ${PCL_LIBS} 
#     ${EIGEN3_LIBS})

# add_library(ADDED_LIBS
#     src/window.cc
#     src/shader.cc
#     src/glad.c
#     src/stb_image.cc)

# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# add_executable(ex1 test/main.cpp)
# target_link_libraries(ex1 PRIVATE realsense2 ADDED_LIBS ${MYLIBS} ${GLFW3_LIBRARY})