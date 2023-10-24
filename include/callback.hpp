#ifndef CALLBACK_HPP
#define CALLBACK_HPP

#include <GLFW/glfw3.h>
#include <glm.hpp>
#include <iomanip>
#include <utility>
#include <iostream>

struct MousePos{
    bool pressed, changed;
    float x, y;

    MousePos(float width = 800, float height = 600):
    	pressed(false), x(width/2.0f), y(height/2.0f){}

    friend std::ostream& operator<<(std::ostream& os, const MousePos mouse) {
        os << "\n\tpress\t: " << (const char *)(mouse.pressed ? "Yes" : "No") << "\n\tx\t: " << std::fixed << std::setprecision(2) << mouse.x << "\n\ty\t: " << mouse.y  << std::defaultfloat << '\n';
        return os;
    }

    MousePos& operator=(const std::pair<float, float>& xy) {
        x = xy.first/2.0f;
        y = xy.second/2.0f;
        return *this;
    }
};

// camera
extern glm::vec3 CAMERA_POS;
extern glm::vec3 CAMERA_FRONT;
extern glm::vec3 CAMERA_UP;

extern bool first_mouse;
extern float yaw;
extern float pitch;
extern float FOV;

extern bool right_pressed;
extern MousePos LPos_press;

void fbuff_callback(GLFWwindow* window, int width, int height);
   
void mouse_callback(GLFWwindow* window, double xpos, double ypos);

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

void mouse_btn_callback(GLFWwindow* window, int button, int action, int mods);

#endif