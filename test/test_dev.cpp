#include <device.hpp>
#include <iomanip>

int main()
{
	Device dev;
	int counter = 10;
	rs2::points pc;
	rs2::video_frame frame;

	while(counter)
	{
		dev.get_pc(pc, frame);
		std::cout << std::fixed << std::setprecision(2) << "\n\n";
		auto vert = pc.get_vertices();
		for (int i = 0; i < pc.size(); ++i)
		{
			std::cout << '(' << vert[i].x << ' ' << vert[i].y << ' ' << vert[i].z << "), ";
		}
		std::cout.flush();
		/*std::cout << "Size : " << pc.size() << std::endl;
		auto tex_coord = pc.get_texture_coordinates();
		std::cout << counter << "V :\n\tx : " << vert->x << 
			"\n\ty : " << vert->y << 
			"\n\tz : " << vert->z << std::endl;
		std::cout << counter << "TC : " << tex_coord->v << std::endl;*/
		counter--;
	}
	return 0;
}