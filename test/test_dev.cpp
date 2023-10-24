#include <device.hpp>
#include <iomanip>
#include <chrono>

int main()
{
	Device dev;
	int counter = 10;
	rs2::points pc;
	rs2::frame f;
	rs2::video_frame frame(f);

	std::cout << std::fixed << std::setprecision(2);
	std::chrono::time_point<std::chrono::high_resolution_clock> t_now, t_past = std::chrono::high_resolution_clock::now();
	while(counter)
	{
		dev.get_pc(pc, frame);
		auto vert = pc.get_vertices();
		t_now = std::chrono::high_resolution_clock::now();
		std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(t_now-t_past);
		t_past = t_now;
		float _fps = 1000000 / duration.count();
		std::cout << "FPS : " << _fps << "   \r";
		std::cout.flush();

		/*for (int i = 0; i < pc.size(); ++i)
		{
			std::cout << '(' << vert[i].x << ' ' << vert[i].y << ' ' << vert[i].z << "), ";
		}
		std::cout.flush();*/

		/*std::cout << "Size : " << pc.size() << std::endl;
		auto tex_coord = pc.get_texture_coordinates();
		std::cout << counter << "V :\n\tx : " << vert->x << 
			"\n\ty : " << vert->y << 
			"\n\tz : " << vert->z << std::endl;
		std::cout << counter << "TC : " << tex_coord->v << std::endl;*/

		counter--;
	}
	std::cout << std::endl;
	return 0;
}