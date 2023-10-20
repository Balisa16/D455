#include <callback.hpp>

// camera
glm::vec3 CAMERA_POS   = glm::vec3(0.0f, 0.0f, 3.0f);
glm::vec3 CAMERA_FRONT = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 CAMERA_UP    = glm::vec3(0.0f, 1.0f, 0.0f);

bool first_mouse = true;
bool left_pressed = false;
bool right_pressed = false;
float YAW   = -90.0f;
float PITCH =  0.0f;
float LAST_X =  800.0f / 2.0f;
float LAST_Y =  600.0f / 2.0f;
float FOV   =  45.0f;

void fbuff_callback(GLFWwindow* window, int width, int height)
{
    // glViewport(0, 0, width, height);
}

void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    if(left_pressed)
    {
        float xpos = static_cast<float>(xposIn);
        float ypos = static_cast<float>(yposIn);

        std::cout << std::fixed << std::setprecision(2) << "x: " << xpos << ",\ty: " << ypos << '\n';

        if (first_mouse)
        {
            LAST_X = xpos;
            LAST_Y = ypos;
            first_mouse = false;
        }

        float xoffset = xpos - LAST_X;
        float yoffset = LAST_Y - ypos; // reversed since y-coordinates go from bottom to top
        LAST_X = xpos;
        LAST_Y = ypos;

        float sensitivity = 0.1f; // change this value to your liking
        xoffset *= sensitivity;
        yoffset *= sensitivity;

        YAW += xoffset;
        PITCH += yoffset;

        // make sure that when PITCH is out of bounds, screen doesn't get flipped
        if (PITCH > 89.0f)
            PITCH = 89.0f;
        if (PITCH < -89.0f)
            PITCH = -89.0f;

        glm::vec3 front;
        front.x = cos(glm::radians(YAW)) * cos(glm::radians(PITCH));
        front.y = sin(glm::radians(PITCH));
        front.z = sin(glm::radians(YAW)) * cos(glm::radians(PITCH));
        CAMERA_FRONT = glm::normalize(front);
    }
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    FOV -= (float)yoffset;
    if (FOV < 1.0f)
        FOV = 1.0f;
    if (FOV > 45.0f)
        FOV = 45.0f;
}

void mouse_btn_callback(GLFWwindow* window, int button, int action, int mods)
{
    if(button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        if(action == GLFW_PRESS)
            right_pressed = true;
        else if(action == GLFW_RELEASE)
            right_pressed = false;
    }
    else if(button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if(action == GLFW_PRESS)
            left_pressed = true;
        else if(action == GLFW_RELEASE)
            left_pressed = false;
    }
}