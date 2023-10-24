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
#include <cmath>

struct RGB{
	float r, g, b;
};

typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
typedef point_cloud::Ptr cloud_pointer;

class Device
{
private:
	std::string serial;
	rs2::pipeline pipe;
	rs2::config cfg;

	rs2::pointcloud pc;
	rs2::frameset frames;

	std::string pc_folder = "pointcloud";

	void check_dir(std::string folder = "pointcloud");
	void RGB_Texture(rs2::video_frame& texture, rs2::texture_coordinate Texture_XY, RGB& out_RGB);
public:
	Device();
	void get_pc(rs2::points& in_points, rs2::video_frame& in_color, int loop = 10);
	void convert_to_PCL(rs2::points& in_points, rs2::video_frame& in_color, pcl::PointCloud<pcl::PointXYZRGB>& output);
	void savePCD(pcl::PointCloud<pcl::PointXYZRGB>& pc, std::string file_name = "D455.pcd");
	~Device();
	
};

#endif