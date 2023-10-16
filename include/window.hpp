#ifndef WINDOW_HPP
#define WINDOW_HPP

#include <librealsense2/rs.hpp>
#include <GLFW/glfw3.h>
#include <linmath.h>

struct app_state {
    app_state(float yaw = 15.0, float pitch = 15.0) : 
        yaw(yaw), pitch(pitch), last_x(0.0), last_y(0.0),
        ml(false), offset_x(2.f), offset_y(2.f) {}
    double yaw;
    double pitch;
    double last_x;
    double last_y;
    bool ml;
    float offset_x;
    float offset_y;
};

class window
{
private:
    GLFWwindow* win;
    int _width, _height;
    app_state _state;

public:
    // operator GLFWwindow* () { return win; }
    // Mouse callback function
    std::function<void(bool)>           on_left_mouse = [](bool) {};
    std::function<void(double, double)> on_mouse_scroll = [](double, double) {};
    std::function<void(double, double)> on_mouse_move = [](double, double) {};
    std::function<void(int)>            on_key_release = [](int) {};
    void mouse_callbacks();
    window(std::string title, int width = 640, int height = 480);
    bool is_open();
    void refresh();
    ~window();
};

void window::mouse_callbacks()
{
    on_left_mouse = [&](bool pressed)
    {
        _state.ml = pressed;
    };

    on_mouse_scroll = [&](double xoffset, double yoffset)
    {
        _state.offset_x -= static_cast<float>(xoffset);
        _state.offset_y -= static_cast<float>(yoffset);
    };

    on_mouse_move = [&](double x, double y)
    {
        if (_state.ml)
        {
            _state.yaw -= (x - _state.last_x);
            _state.yaw = std::max(_state.yaw, -120.0);
            _state.yaw = std::min(_state.yaw, +120.0);
            _state.pitch += (y - _state.last_y);
            _state.pitch = std::max(_state.pitch, -80.0);
            _state.pitch = std::min(_state.pitch, +80.0);
        }
        _state.last_x = x;
        _state.last_y = y;
    };

    on_key_release = [&](int key)
    {
        if (key == 32) // Escape
        {
            _state.yaw = _state.pitch = 0; 
            _state.offset_x = _state.offset_y = 0.0;
        }
    };
}

bool window::is_open(){
    return glfwWindowShouldClose(win);
}

void window::refresh()
{   
    /*glClearColor(0.9f, 0.9f, 0.9f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);*/
    glfwSwapBuffers(win);
    glfwPollEvents();
}

window::window(std::string title, int width, int height) : 
    _width(width), _height(height)
{
    glfwInit();
    win = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!win)
        throw std::runtime_error("Could not open OpenGL window, please check your graphic drivers or use the textual SDK tools");
    glfwMakeContextCurrent(win);

    glfwSetWindowUserPointer(win, this);
    glfwSetMouseButtonCallback(win, [](GLFWwindow* w, int button, int action, int mods)
        {
            auto s = (window*)glfwGetWindowUserPointer(w);
            if (button == 0) s->on_left_mouse(action == GLFW_PRESS);
        });

    glfwSetScrollCallback(win, [](GLFWwindow* w, double xoffset, double yoffset)
        {
            auto s = (window*)glfwGetWindowUserPointer(w);
            s->on_mouse_scroll(xoffset, yoffset);
        });

    glfwSetCursorPosCallback(win, [](GLFWwindow* w, double x, double y)
        {
            auto s = (window*)glfwGetWindowUserPointer(w);
            s->on_mouse_move(x, y);
        });

    glfwSetKeyCallback(win, [](GLFWwindow* w, int key, int scancode, int action, int mods)
        {
            auto s = (window*)glfwGetWindowUserPointer(w);
            if (0 == action) // on key release
            {
                s->on_key_release(key);
            }
        });

    mouse_callbacks();

    glViewport(0, 0, _width, _height);
    mat4x4 projection;
    float ratio = (float)_width / (float)_height;
    mat4x4_perspective(projection,
                       60.f * (float) M_PI / 180.f,
                       ratio,
                       1.f, 1024.f);
    glLoadMatrixf((const GLfloat*) projection);
}

window::~window()
{
    glfwDestroyWindow(win);
    glfwTerminate();
}


// Struct for managing rotation of pointcloud view
/*struct glfw_state {
    glfw_state(float yaw = 15.0, float pitch = 15.0) : yaw(yaw), pitch(pitch), last_x(0.0), last_y(0.0),
        ml(false), offset_x(2.f), offset_y(2.f), tex() {}
    double yaw;
    double pitch;
    double last_x;
    double last_y;
    bool ml;
    float offset_x;
    float offset_y;
    texture tex;
};*/




#endif