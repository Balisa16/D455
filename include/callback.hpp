#ifndef CALLBACK_HPP
#define CALLBACK_HPP

#include <GLFW/glfw3.h>
#include <glm.hpp>
#include <iomanip>
#include <iostream>

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

extern bool left_pressed;
extern bool right_pressed;

void fbuff_callback(GLFWwindow* window, int width, int height);
   
void mouse_callback(GLFWwindow* window, double xpos, double ypos);

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

void mouse_btn_callback(GLFWwindow* window, int button, int action, int mods);

#endif