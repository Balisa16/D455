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

typedef struct{
	uint8_t r, g, b;
}Color;

struct PointCloud{
	std::vector<Eigen::Vector3f> position;
	std::vector<Color> color;
	uint64_t size;
	int width, height;

	PointCloud():size(0){}

	void add(const Eigen::Vector3f pos, const Color col)
	{
		position.push_back(pos);
		color.push_back(col);
		size++;
	}

	PointCloud& operator=(const PointCloud& other) {
        if (this == &other) {
            return *this; // Self-assignment, no need to do anything
        }

        position = other.position;
        color = other.color;
        size = other.size;

        return *this;
    }

    PointCloud& operator+(const PointCloud& other) {
    	for (int i = 0; i < other.size; ++i)
    	{
    		position.push_back(other.position[i]);
    		color.push_back(other.color[i]);
    		size++;
    	}
        return *this;;
    }

    PointCloud& operator+=(const PointCloud& other) {
    	for (int i = 0; i < other.size; ++i)
    	{
    		position.push_back(other.position[i]);
    		color.push_back(other.color[i]);
    		size++;
    	}
        return *this;;
    }

    void clear()
    {
    	position.clear();
    	color.clear();
    	size = 0;
    	width = 0;
    	height = 0;
    }
};

namespace EMIRO
{
	class Mat
	{
	public:
		Mat();

		Eigen::Quaternionf euler_to_quaternion(Euler euler);

		void pclConvert(Eigen::Vector3f position, Euler euler, pcl::PointCloud<pcl::PointXYZRGB>* in_pointcloud, pcl::PointCloud<pcl::PointXYZRGB>* out_pointcloud);

		void transform_pc(Eigen::Vector3f position, Euler euler, PointCloud* src, PointCloud* dst);
		
		void convert_to_pcl(PointCloud* src, pcl::PointCloud<pcl::PointXYZRGB>* dst);

		~Mat();
	};

}

#endif