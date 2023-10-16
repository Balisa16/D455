#ifndef AXIS_HPP
#define AXIS_HPP

#include <Eigen/Core>
#include <math.h>
#include <pangolin/pangolin.h>

void create_axis(std::shared_ptr<pangolin::Axis> axis, float x, float y, float z, float x_rad, float y_rad, float z_rad)
{
	// https://en.wikipedia.org/wiki/Rotation_matrix

	if(x_rad > M_PI || y_rad > M_PI || z_rad > M_PI)	// if user using degree unit
	{
		x_rad *= M_PI/180.0f;
		y_rad *= M_PI/180.0f;
		z_rad *= M_PI/180.0f;
	}

	Eigen::Matrix4f transf_mat = Eigen::Matrix4f::Identity();

   	transf_mat.block<3, 3>(0, 0) << 
   		std::cos(z_rad)*std::cos(y_rad), std::cos(z_rad)*std::sin(y_rad)*std::sin(x_rad) - std::sin(z_rad)*std::cos(x_rad), std::cos(z_rad)*std::sin(y_rad)*std::cos(x_rad) + std::sin(z_rad)*std::sin(x_rad),
   		std::sin(z_rad)*std::cos(y_rad), std::sin(z_rad)*std::sin(y_rad)*std::sin(x_rad) + std::cos(z_rad)*std::cos(x_rad), std::sin(z_rad)*std::sin(y_rad)*std::cos(x_rad) - std::cos(z_rad)*std::sin(x_rad),
   		-std::sin(y_rad), std::cos(y_rad)*std::sin(x_rad), std::cos(y_rad)*std::cos(x_rad),

   	transf_mat.block<3, 1>(0, 3) << x, y, z;

    axis->T_pc = pangolin::OpenGlMatrix(transf_mat);
}

#endif