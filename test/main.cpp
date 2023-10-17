#include <window.hpp>
#include <shader.hpp>

int main()
{
    window win("Test");
    Shader shader;
    shader.init("../test/main.vert", "../test/main.frag");
    shader.use();
    win.conf_vertex();
    while (win.is_open())
    {
    	win.processinput();
        win.clear_buffer();
        shader.use();
	    win.draw();
	    win.swap_poll();
    }
    return 0;
}