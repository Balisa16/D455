#include <mat.hpp>
#include <device.hpp>
#include <writer_pcd.hpp>

int main()
{
	EMIRO::Device dev;
	EMIRO::Mat mat;
	rs2::points pc;
	rs2::frame f;
	rs2::video_frame frame(f);

	PointCloud pc_main, pc_temp;

	std::this_thread::sleep_for(std::chrono::seconds(2));

	int cnt = 8;

	PCDWriter::set_folder("pc");
	while (cnt)
	{
		Eigen::Vector3f p = {0.0f, 5.0f, 0.0f};
		Eigen::Quaternionf q = {0.707f, 0.707f, 0.0f, 0.0f};
		Euler euler = {0.0f, cnt * 90.0f, 0.0f}, _temp_euler;
		Quaternion quat;

		dev.get_pc(pc, frame, &euler);

		pc_temp.clear();

		dev.make_pointcloud(&pc, &frame, &pc_temp);

		dev.store_pc(&pc_temp, &pc_main);

		dev.get_orientation(&_temp_euler);

		std::cout << "roll : " << _temp_euler.roll << " pitch : " << _temp_euler.pitch << " yaw : " << _temp_euler.yaw << std::endl;

		if (pc_main.size > 500000)
		{
			pcl::PointCloud<pcl::PointXYZRGB> out_pc;

			mat.convert_to_pcl(&pc_main, &out_pc);

			PCDWriter::write("output", out_pc, p, q);

			// Remove old data
			pc_main.clear();
		}

		std::this_thread::sleep_for(std::chrono::seconds(2));

		cnt--;
	}

	if (pc_main.size)
	{
		Eigen::Vector3f p = {0.0f, 5.0f, 0.0f};
		Eigen::Quaternionf q = {0.707f, 0.707f, 0.0f, 0.0f};
		pcl::PointCloud<pcl::PointXYZRGB> out_pc;

		mat.convert_to_pcl(&pc_main, &out_pc);

		PCDWriter::write("output", out_pc, p, q);

		pc_main.clear();
	}
	return 0;
}