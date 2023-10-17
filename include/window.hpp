#ifndef WINDOW_HPP
#define WINDOW_HPP

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

// extern const char *vertexShaderSource;
// extern const char *fragmentShaderSource;

class window
{
private:
    GLFWwindow* win;
    std::string title;
    int width, height;
    unsigned int shader_program, VAO, VBO;

    static void fbuff_callback(GLFWwindow* window, int width, int height);
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