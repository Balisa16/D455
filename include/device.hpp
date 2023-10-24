#ifndef DEVICE_HPP
#define DEVICE_HPP

#include <librealsense2/rs.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <unistd.h>
#include <iostream>

typedef pcl::PointCloud<pcl::PointXYZRGB> pcl_pc;
typedef pcl_pc::Ptr pcPointer;

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
	void convert_to_PCL(rs2::points& in_points, rs2::video_frame& in_color, pcPointer& cloud_pointer);
	~Device();
	
};

#endif