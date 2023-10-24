#ifndef DEVICE_HPP
#define DEVICE_HPP

#include <librealsense2/rs.hpp>
#include <pcl/pcl_exports.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <unistd.h>
#include <iostream>

typedef pcl::PointXYZRGB RGB_Cloud;
typedef pcl::PointCloud<RGB_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;

class Device
{
private:
	std::string serial;
	rs2::pipeline pipe;
	rs2::config cfg;

	rs2::pointcloud pc;
	rs2::frameset frames;
public:
	Device();
	void get_pc(rs2::points& in_points, rs2::video_frame& in_color);
	void convert_to_PCL(rs2::points& in_points, rs2::video_frame& in_color, cloud_pointer& cloud_ptr);
	void savePCD(pcl::PointCloud<pcl::PointXYZRGB>& pc);
	~Device();
	
};

#endif