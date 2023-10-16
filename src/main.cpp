#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

#include <axis.hpp>
// #include <rs-utils.hpp>
#include <window.hpp>
// #include <librealsense2/rs.hpp>
// #include <GLFW/glfw3.h>

int main( int /*argc*/, char** /*argv*/ )
{
	window app("Test");
	// glfw_state app_state;
	// register_glfw_callbacks(app, app_state);
	while(!app.is_open())
	{
		app.refresh();
	}
    return 0;
}
