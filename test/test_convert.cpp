#include <mat.hpp>
#include <device.hpp>

int main()
{
	EMIRO::Device dev;
	int counter = 5;
	rs2::points pc;
	rs2::frame f;
	rs2::video_frame frame(f);

	pcl::PointCloud<pcl::PointXYZRGB> pcl_pc;

	dev.get_pc(pc, frame);
	auto vert = pc.get_vertices();

	dev.convert_to_PCL(pc, frame, pcl_pc, 3.0f);
	Position p = {0.0f, 0.0f, 0.0f};
	Euler euler = {0.0f, 90.0f, 0.0f};

	pcl::PointCloud<pcl::PointXYZRGB> out_pc;
	EMIRO::pclConvert(p, euler, &pcl_pc, &out_pc);
	return 0;
}