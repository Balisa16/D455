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

	PointCloud pc_main, pc_temp;

	int cnt = 100;
	while(cnt)
	{
		dev.get_pc(pc, frame);

		pc_temp.clear();

		dev.make_pointcloud(&pc, &frame, &pc_temp);

		dev.store_pc(&pc_temp, &pc_main);

		std::cout << "Pos size : " << pc_main.position.size() << '\n';

		/*dev.convert_to_PCL(pc, frame, pcl_pc, 3.0f);
		
		Position p = {0.0f, 0.0f, 0.0f};
		Euler euler = {0.0f, 90.0f, 0.0f};

		pcl::PointCloud<pcl::PointXYZRGB> out_pc;
		EMIRO::pclConvert(p, euler, &pcl_pc, &out_pc);*/
		cnt--;
	
	}
	return 0;
}