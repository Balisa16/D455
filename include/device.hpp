#ifndef DEVICE_HPP
#define DEVICE_HPP

#include <values.h>
#include <librealsense2/rs.hpp>
#include <boost/filesystem.hpp>
#include <mat.hpp>
#include <unistd.h>
#include <iostream>
#include <string>
#include <cmath>
#include <memory>
#include <chrono>
#include <iomanip>
#include <thread>
#include <functional>
#include <atomic>
#include <stdexcept>
#include <tcp_client.hpp>

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

	Euler euler;

	D455Data() : thread_en(true),		// Initialize boolean member
				 status(TStatus::None), // Initialize 'status' with TStatus::None
				 lock(ATOMIC_FLAG_INIT)
	{
	}
};

namespace EMIRO
{
	void frames_update(D455Data *data);

	void send_thread(std::shared_ptr<EMIRO::TCP> tcp_class, std::string filename, std::atomic_flag *lock_flag);

	typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
	typedef point_cloud::Ptr cloud_pointer;

	class Device
	{
	private:
		D455Data data;

		std::string out_folder, pc_folder;

		int filename_idx = 1;

		void check_dir(std::string folder = "../output");
		void RGB_Texture(rs2::video_frame &texture, rs2::texture_coordinate Texture_XY, Color &out_RGB);
		void progress_bar(int i, int maks = 100);

		// Frame thread
		std::thread th;

		// File sender
		std::shared_ptr<EMIRO::TCP> tcp_cl = std::make_shared<EMIRO::TCP>();
		std::thread tcp_th;
		std::atomic_flag transfer_lock;

	public:
		Device();

		bool check_imu();

		void get_pc(rs2::points &p, rs2::video_frame &c, Euler *euler);

		void convert_to_PCL(rs2::points &in_points, rs2::video_frame &in_color, pcl::PointCloud<pcl::PointXYZRGB> &output, float depth_lim = 5.0f);

		void make_pointcloud(rs2::points *in_points, rs2::video_frame *in_color, PointCloud *pc);

		void store_pc(PointCloud *src, PointCloud *dest);

		void get_orientation(Euler *euler);

		std::string savePCD(pcl::PointCloud<pcl::PointXYZRGB> &pc, Eigen::Vector3f pos = {0.0f, 0.0f, 0.0f}, Quaternion quat = {0.0f, 0.0f, 0.0f, 0.0f}, std::string file_name = "pointcloud");

		void sendPCD(std::string file_name = "pointcloud");

		rs2::points clean_pc(rs2::points &in_points);

		~Device();
	};
}

#endif