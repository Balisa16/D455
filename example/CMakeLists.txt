file(GLOB SRC_FILES *.cpp)
message("\nDetect TEST")
foreach(SRC_FILE ${SRC_FILES})
    get_filename_component(TARGET_NAME ${SRC_FILE} NAME_WE)

    message("\tFound : ${TARGET_NAME}")
    
    add_executable(${TARGET_NAME} ${SRC_FILE})
    target_include_directories(
        ${TARGET_NAME} PRIVATE
        ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(${TARGET_NAME} PRIVATE
        glfw 
        glm
        jsoncpp
        /usr/include/eigen3
        /usr/include/pcl-1.8
        /usr/include/pcl-1.8/pcl
        ${PROJECT_NAME}
        ${PCL_LIBRARIES}
        realsense2)
    target_include_directories(${TARGET_NAME} PRIVATE
        thirdparty/glfw/include)
endforeach()
message("")
