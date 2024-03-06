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

struct float3
{
	float x, y, z;

	void add(float x, float y, float z)
	{
		this->x += x;
		this->y += y;
		this->z += z;
	}

	float3 operator*(float s)
	{
		return {x * s, y * s, z * s};
	}
};

class rotation_estimator
{
	// theta is the angle of camera rotation in x, y and z components
	float3 theta;
	/* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
	values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
	float alpha = 0.98f;
	bool firstGyro = true;
	bool firstAccel = true;
	// Keeps the arrival time of previous gyro frame
	double last_ts_gyro = 0;

public:
	// Function to calculate the change in angle of motion based on data from gyro
	void process_gyro(rs2_vector gyro_data, double ts)
	{
		if (firstGyro) // On the first iteration, use only data from accelerometer to set the camera's initial position
		{
			firstGyro = false;
			last_ts_gyro = ts;
			return;
		}
		// Holds the change in angle, as calculated from gyro
		float3 gyro_angle;

		// Initialize gyro_angle with data from gyro
		gyro_angle.x = gyro_data.x; // Pitch
		gyro_angle.y = gyro_data.y; // Yaw
		gyro_angle.z = gyro_data.z; // Roll

		// Compute the difference between arrival times of previous and current gyro frames
		double dt_gyro = (ts - last_ts_gyro) / 1000.0;
		last_ts_gyro = ts;

		// Change in angle equals gyro measures * time passed since last measurement
		gyro_angle = gyro_angle * static_cast<float>(dt_gyro);
		theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
	}

	// Returns the current rotation angle
	float3 get_theta()
	{
		return theta;
	}
};

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

		int filename_idx = 1;

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

		void sendPCD(std::string file_name = "pointcloud");

		rs2::points clean_pc(rs2::points &in_points);

		~Device();
	};
}

#endif