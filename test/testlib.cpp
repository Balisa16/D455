#include <iostream>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <librealsense2/rs.hpp>

int main()
{
    rs2::pointcloud pc;
    
    glm::vec3 cameraUp    = glm::vec3(0.0f, 1.0f, 0.0f);
    
	glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *win = glfwCreateWindow(800, 600, "Test Window", nullptr, nullptr);
    glfwMakeContextCurrent(win);

	std::cout << "Test 1" << std::endl;
    while(!glfwWindowShouldClose(win))
    {
    	glfwPollEvents();
    }
    glfwTerminate();
	return 0;
}