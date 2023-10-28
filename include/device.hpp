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

#include <atomic>


enum class TStatus{
	None,
	Init,
	Available,
	Exit
};

typedef struct{
	float x, y, z;
}Position;

struct D455Data
{
private:
    const rs2::frame f;

public:
    bool thread_en;
    TStatus status;
    std::atomic_flag lock;

    std::string serial;
    rs2::config cfg;
    rs2::pipeline pipe;
    rs2::pointcloud pc;
    rs2::points point;
    rs2::frame color;
    rs2::frameset frames;
    std::chrono::time_point<std::chrono::high_resolution_clock> t_now;
    std::chrono::time_point<std::chrono::high_resolution_clock> t_past;

    D455Data() :
        thread_en(true),  // Initialize boolean member
        status(TStatus::None),  // Initialize 'status' with TStatus::None
        lock(ATOMIC_FLAG_INIT)
    {
    }
};

namespace EMIRO
{
	void frames_update(D455Data* data);

	struct RGB{
		float r, g, b;
	};

	typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
	typedef point_cloud::Ptr cloud_pointer;

	class Device
	{
	private:
	    D455Data data;

		std::string out_folder, pc_folder;

		int filename_idx = 1;

		void check_dir(std::string folder = "../output");
		void RGB_Texture(rs2::video_frame& texture, rs2::texture_coordinate Texture_XY, RGB& out_RGB);
		void progress_bar(int i, int maks = 100);

		// JSON Writer
		Json::Value root;
		Json::StreamWriterBuilder builder;
		std::unique_ptr<Json::StreamWriter> writer;
		std::ofstream output_file;

		// Frame thread
		std::thread th;

	public:

		Device();
		void get_pc(rs2::points& p, rs2::video_frame& c);
		void convert_to_PCL(rs2::points& in_points, rs2::video_frame& in_color, pcl::PointCloud<pcl::PointXYZRGB>& output, float depth_lim = 5.0f);
		void savePCD(pcl::PointCloud<pcl::PointXYZRGB>& pc, Position pos = {0.0f, 0.0f, 0.0f}, std::string file_name = "pointcloud");
		~Device();
		
	};
}

#endif