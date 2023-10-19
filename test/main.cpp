#include <window.hpp>
#include <shader.hpp>
#include <glm/glm.hpp>

int main()
{
    window win("Test");
    Shader shader;
    shader.init("../../resources/shader/main.vert", "../../resources/shader/main.frag");
    shader.use();
    win.conf_vertex();
    shader.use();
    shader.setI("texture1", 0);
    shader.setI("texture2", 1);
    while (win.is_open())
    {
        win.processinput();
        win.clear_buffer();
        win.bind_texture();
        glm::mat4 projection = glm::perspective(glm::radians(win.get_fov()), win.get_width() / win.get_height(), 0.1f, 100.0f);
        glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
        shader.use();
        shader.setM4("projection", projection);
        shader.setM4("view", view);
        win.draw(std::make_shared<Shader>(shader));
        win.swap_poll();
    }
    return 0;
}