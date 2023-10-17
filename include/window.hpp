#ifndef WINDOW_HPP
#define WINDOW_HPP

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

// extern const char *vertexShaderSource;
// extern const char *fragmentShaderSource;

// camera
extern glm::vec3 cameraPos;
extern glm::vec3 cameraFront;
extern glm::vec3 cameraUp;

extern bool first_mouse;
extern float yaw;
extern float pitch;
extern float lastX;
extern float lastY;
extern float fov;

class window
{
private:
    GLFWwindow* win;
    std::string title;
    int width, height;
    unsigned int shader_program, VAO, VBO;

    static void fbuff_callback(GLFWwindow* window, int width, int height);
    static void mouse_callback(GLFWwindow* window, double xpos, double ypos);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
public:
    window(std::string title = "D455", int width = 640, int height = 480);
    void init_shader(const char * vert_source, const char * frag_source);
    void conf_vertex();
    bool is_open();
    void processinput();
    void clear_buffer();
    void draw();
    void swap_poll();
    ~window();
};

#endif