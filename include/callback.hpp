#ifndef CALLBACK_HPP
#define CALLBACK_HPP

#include <GLFW/glfw3.h>
#include <glm.hpp>
#include <iomanip>
#include <iostream>

// camera
extern glm::vec3 CAMERA_POS;
extern glm::vec3 CAMERA_FRONT;
extern glm::vec3 CAMERA_UP;

extern bool first_mouse;
extern float yaw;
extern float pitch;
extern float LAST_X;
extern float LAST_Y;
extern float FOV;

extern bool left_pressed;
extern bool right_pressed;

void fbuff_callback(GLFWwindow* window, int width, int height);
   
void mouse_callback(GLFWwindow* window, double xpos, double ypos);

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

void mouse_btn_callback(GLFWwindow* window, int button, int action, int mods);

#endif