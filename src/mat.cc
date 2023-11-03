#include <mat.hpp>

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
		out_pointcloud->points = in_pointcloud->points;
	    transform.translation() << position.x, position.y, position.z;
	    transform.rotate(euler_to_quaternion(euler));

	    // pcl::transformPointCloud(*out_pointcloud, *in_pointcloud, transform);

	    // Apply the transformation only to the XYZ coordinates while keeping RGB values unchanged
	    pcl::PointXYZRGB point;
	    std::cout << std::fixed << std::setprecision(2);
	    for (int i = 0; i < in_pointcloud->size(); ++i)
	    {
	        Eigen::Vector3f pt(in_pointcloud->points[i].x, in_pointcloud->points[i].y, in_pointcloud->points[i].z);
	        Eigen::Vector3f transformed_pt = transform * pt;
	        out_pointcloud->points[i].x = transformed_pt.x();
	        out_pointcloud->points[i].y = transformed_pt.y();
	        out_pointcloud->points[i].z = transformed_pt.z();
	        std::cout << "x:" << in_pointcloud->points[i].x << ", y:" << in_pointcloud->points[i].y << ", z:" << in_pointcloud->points[i].z <<
	        	"\tx:" << out_pointcloud->points[i].x << ", y:" << out_pointcloud->points[i].y << ", z:" << out_pointcloud->points[i].z << '\n';
	    }
	}
}