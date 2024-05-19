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
		Eigen::Vector3f p = {.0f, .0f, .0f};
		Eigen::Quaternionf q = {.0f, .0f, .0f, .0f};
		Quaternion quat;

		dev.get_pc(pc, frame);

		pc_temp.clear();

		dev.make_pointcloud(&pc, &frame, &pc_temp);

		dev.store_pc(&pc_temp, &pc_main);

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