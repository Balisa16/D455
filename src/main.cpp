#include <window.hpp>

int main()
{
    window win("Test");
    while (win.is_open())
    {
    	win.processinput();
        win.clear_buffer();
	    win.draw();
	    win.swap_poll();
    }
    return 0;
}