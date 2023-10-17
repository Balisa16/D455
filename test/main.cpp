#include <window.hpp>
#include <shader.hpp>

int main()
{
    window win("Test");
    const char *vert_char = file2buf("../test/main.vert");
    const char *frag_char = file2buf("../test/main.frag");
    win.init_shader(vert_char, frag_char);
    win.conf_vertex();
    while (win.is_open())
    {
    	win.processinput();
        win.clear_buffer();
	    win.draw();
	    win.swap_poll();
    }
    return 0;
}