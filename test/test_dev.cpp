#include <device.hpp>
#include <iomanip>
#include <chrono>

int main()
{
	EMIRO::Device dev;
	int counter = 5;
	rs2::points pc;
	rs2::frame f;
	rs2::video_frame frame(f);

	pcl::PointCloud<pcl::PointXYZRGB> pcl_pc;

	std::cout << std::fixed << std::setprecision(2);
	std::chrono::time_point<std::chrono::high_resolution_clock> t_now, t_past = std::chrono::high_resolution_clock::now();

	while(counter)
	{
		Quaternion quat;
		dev.get_pc(pc, frame);
		auto vert = pc.get_vertices();

		dev.convert_to_PCL(pc, frame, pcl_pc, 3.0f);
		Eigen::Vector3f p = {1.0f, 2.0f, 3.0f};
		Quaternion q = {0.75f, 0.76f, 0.77f, 0.78f};
		dev.savePCD(pcl_pc, p, q);

		t_now = std::chrono::high_resolution_clock::now();
		std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(t_now-t_past);
		t_past = t_now;
		float _fps = 1000000 / duration.count();


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
	return 0;
}