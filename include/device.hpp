#ifndef DEVICE_HPP
#define DEVICE_HPP

#include <librealsense2/rs.hpp>
#include <boost/filesystem.hpp>
#include <pcl/pcl_exports.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <cmath>
#include <json/json.h>
#include <memory>
#include <chrono>
#include <ctime>
#include <iomanip> 
#include <thread>
#include <mutex>
#include <functional>

namespace EMIRO
{
	extern std::string serial;
	extern rs2::pipeline pipe;
	extern rs2::config cfg;

	extern std::shared_ptr<rs2::frameset> frames;

	extern std::chrono::time_point<std::chrono::high_resolution_clock> t_now, t_past;
	extern std::mutex mtx;

	void frames_update();

	struct RGB{
		float r, g, b;
	};

	typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
	typedef point_cloud::Ptr cloud_pointer;

	class Device
	{
	private:
		rs2::pointcloud pc;

		std::string out_folder, pc_folder;

		int filename_idx = 1;

		void check_dir(std::string folder = "../output");
		void RGB_Texture(rs2::video_frame& texture, rs2::texture_coordinate Texture_XY, RGB& out_RGB);

		// JSON Writer
		Json::Value root;
		Json::StreamWriterBuilder builder;
		std::unique_ptr<Json::StreamWriter> writer;
		std::ofstream output_file;

		// Frame thread
		std::thread th;

	public:

		Device();
		void get_pc(rs2::points& p, rs2::video_frame& c, int loop = 10);
		void convert_to_PCL(rs2::points& in_points, rs2::video_frame& in_color, pcl::PointCloud<pcl::PointXYZRGB>& output, float depth_lim = 5.0f);
		void savePCD(pcl::PointCloud<pcl::PointXYZRGB>& pc, std::string file_name = "pointcloud");
		~Device();
		
	};
}

#endif