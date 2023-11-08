#include <mat.hpp>
#include <device.hpp>

int main()
{
	EMIRO::Device dev;
	EMIRO::Mat mat;
	int counter = 5;
	rs2::points pc;
	rs2::frame f;
	rs2::video_frame frame(f);

	pcl::PointCloud<pcl::PointXYZRGB> pcl_pc;

	PointCloud pc_main, pc_temp, pc_temp2;

	std::this_thread::sleep_for(std::chrono::seconds(2));

	int cnt = 6;
	while(cnt)
	{
		Eigen::Vector3f p = {0.0f, 0.0f, 0.0f};
		Euler euler = {0.0f, cnt * 90.0f, 0.0f};
		Quaternion quat;

		dev.get_pc(pc, frame, euler);

		std::cout << "\nr : " << euler.roll << ",\tp : " << euler.pitch << ",\ty : " << euler.yaw << '\n';

		pc_temp.clear();

		dev.make_pointcloud(&pc, &frame, &pc_temp);

		// dev.get_orientation(&euler);

		mat.transform_pc(p, euler, &pc_temp, &pc_temp2);
		// mat.transform_pc(p, quat, &pc_temp, &pc_temp2);
		
		dev.store_pc(&pc_temp2, &pc_main);


		std::cout << "\nWait\n";

		std::this_thread::sleep_for(std::chrono::seconds(2));

		std::cout << "\nOk\n";

		cnt--;
	
	}
	pcl::PointCloud<pcl::PointXYZRGB> out_pc;

	mat.convert_to_pcl(&pc_main, &out_pc);

	dev.savePCD(out_pc);
	return 0;
}