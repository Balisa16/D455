#include <cmath>
#include <Eigen/Eigen>

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
	Eigen::Quaternionf euler_to_quaternion(Euler euler)
	{
		float roll2 = euler.roll/2.0f;
		float pitch2 = euler.pitch/2.0f;
		float yaw2 = euler.yaw/2.0f;
	    float qx = sin(roll2) * cos(pitch2) * cos(yaw2) - cos(roll2) * sin(pitch2) * sin(yaw2);
	    float qy = cos(roll2) * sin(pitch2) * cos(yaw2) + sin(roll2) * cos(pitch2) * sin(yaw2);
	    float qz = cos(roll2) * cos(pitch2) * sin(yaw2) - sin(roll2) * sin(pitch2) * cos(yaw2);
	    float qw = cos(roll2) * cos(pitch2) * cos(yaw2) + sin(roll2) * sin(pitch2) * sin(yaw2);
	    return {qx, qy, qz, qw};
	}

	void pclConvert(Position position, Euler euler, pcl::PointCloud<pcl::PointXYZRGB>* in_pointcloud, pcl::PointCloud<pcl::PointXYZRGB>* out_pointcloud)
	{
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();

	    transform.translation() << position.x, position.y, position.z;
	    transform.rotate(euler_to_quaternion(euler));

	    // Apply the transformation to the second point cloud
	    pcl::transformPointCloud(*out_pointcloud, *in_pointcloud, transform);
	}
}