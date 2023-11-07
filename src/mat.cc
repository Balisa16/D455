#include <mat.hpp>

namespace EMIRO
{
	Mat::Mat(){}

	Eigen::Quaternionf Mat::euler_to_quaternion(Euler euler)
	{
		/*float roll2 = euler.roll/2.0f;
		float pitch2 = euler.pitch/2.0f;
		float yaw2 = euler.yaw/2.0f;
	    float qx = sin(roll2) * cos(pitch2) * cos(yaw2) - cos(roll2) * sin(pitch2) * sin(yaw2);
	    float qy = cos(roll2) * sin(pitch2) * cos(yaw2) + sin(roll2) * cos(pitch2) * sin(yaw2);
	    float qz = cos(roll2) * cos(pitch2) * sin(yaw2) - sin(roll2) * sin(pitch2) * cos(yaw2);
	    float qw = cos(roll2) * cos(pitch2) * cos(yaw2) + sin(roll2) * sin(pitch2) * sin(yaw2);*/

	    euler.roll *= (M_PI/180.0f);
	    euler.pitch *= (M_PI/180.0f);
	    euler.yaw *= (M_PI/180.0f);

	    double cr = cos(euler.roll * 0.5f);
	    double sr = sin(euler.roll * 0.5f);
	    double cp = cos(euler.pitch * 0.5f);
	    double sp = sin(euler.pitch * 0.5f);
	    double cy = cos(euler.yaw * 0.5f);
	    double sy = sin(euler.yaw * 0.5f);

	    float qw = cr * cp * cy + sr * sp * sy;
	    float qx = sr * cp * cy - cr * sp * sy;
	    float qy = cr * sp * cy + sr * cp * sy;
	    float qz = cr * cp * sy - sr * sp * cy;
	    // std::cout << "w:" << qw << "\tx:" << qx << "\ty:" << qy << "\tz:" << qz << '\n';

	    return {qw, qx, qy, qz};
	}

	void Mat::pclConvert(Eigen::Vector3f position, Euler euler, pcl::PointCloud<pcl::PointXYZRGB>* in_pointcloud, pcl::PointCloud<pcl::PointXYZRGB>* out_pointcloud)
	{
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		out_pointcloud->points = in_pointcloud->points;
	    transform.translation() << position.x(), position.y(), position.z();
	    Eigen::Quaternionf eu = euler_to_quaternion(euler);
	    transform.rotate(eu);

	    // pcl::transformPointCloud(*out_pointcloud, *in_pointcloud, transform);

	    // Apply the transformation only to the XYZ coordinates while keeping RGB values unchanged
	    pcl::PointXYZRGB point;
	    for (int i = 0; i < in_pointcloud->size(); ++i)
	    {
	        Eigen::Vector3f pt(in_pointcloud->points[i].x, in_pointcloud->points[i].y, in_pointcloud->points[i].z);
	        Eigen::Vector3f transformed_pt = transform * pt;
	        out_pointcloud->points[i].x = transformed_pt.x();
	        out_pointcloud->points[i].y = transformed_pt.y();
	        out_pointcloud->points[i].z = transformed_pt.z();
	        /*std::cout << "x:" << in_pointcloud->points[i].x << ", y:" << in_pointcloud->points[i].y << ", z:" << in_pointcloud->points[i].z <<
	        	"\tx:" << out_pointcloud->points[i].x << ", y:" << out_pointcloud->points[i].y << ", z:" << out_pointcloud->points[i].z << '\n';*/
	    }
	}

	void Mat::transform_pc(Eigen::Vector3f position, Euler euler, PointCloud* src, PointCloud* dst)
	{
		dst->position = src->position;
		dst->color = src->color;
		dst->width = src->width;
		dst->height = src->height;
		dst->size = src->size;

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	    transform.translation() << position;
	    transform.rotate(euler_to_quaternion(euler));

	    for (int i = 0; i < src->size; ++i)
	        dst->position[i] = transform * src->position[i];
		
	}

	void Mat::transform_pc(Eigen::Vector3f position, Quaternion quat, PointCloud* src, PointCloud* dst)
	{
		dst->position = src->position;
		dst->color = src->color;
		dst->width = src->width;
		dst->height = src->height;
		dst->size = src->size;

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	    transform.translation() << position;
	    transform.rotate(Eigen::Quaternionf(quat.w, quat.x, quat.y, quat.z));

	    for (int i = 0; i < src->size; ++i)
	        dst->position[i] = transform * src->position[i];
		
	}

	void Mat::convert_to_pcl(PointCloud* src, pcl::PointCloud<pcl::PointXYZRGB>* dst)
	{
		// Get minimal size
		uint64_t data_size = src->size;

		// Input width and height
		dst->width = src->width;
		dst->height = src->height;
		dst->is_dense = false;
		dst->points.resize(data_size);

		// Save into PCL data
		for (int i = 0; i < data_size; ++i)
		{
			dst->points[i].x = src->position[i].x();
			dst->points[i].y = src->position[i].y();
			dst->points[i].z = src->position[i].z();

			dst->points[i].r = src->color[i].r;
			dst->points[i].g = src->color[i].g;
			dst->points[i].b = src->color[i].b;
		}
	}

	Mat::~Mat(){}
}