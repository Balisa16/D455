#include <cmath>
#include <Eigen/Eigen>
#include <pcl/pcl_exports.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <iomanip>

#ifndef MAT_HPP
#define MAT_HPP

enum class TStatus{
	None,
	Init,
	Available,
	Exit
};

typedef struct{
	float x, y, z;
}Position;

typedef struct{
	float w, x, y, z;
}Quaternion;

typedef struct{
	float roll, pitch, yaw;
}Euler;

namespace EMIRO
{
	extern Eigen::Quaternionf euler_to_quaternion(Euler euler);

	extern void pclConvert(Position position, Euler euler, pcl::PointCloud<pcl::PointXYZRGB>* in_pointcloud, pcl::PointCloud<pcl::PointXYZRGB>* out_pointcloud);
}

#endif