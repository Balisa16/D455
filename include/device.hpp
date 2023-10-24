#ifndef DEVICE_HPP
#define DEVICE_HPP

#include <librealsense2/rs.hpp>

class Device
{
private:
	std::string serial;
	rs2::pipeline d_pipe;
	rs2::config d_cfg
public:
	Device();
	~Device();
	
};

#endif