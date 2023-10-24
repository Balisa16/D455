#include <device.hpp>


Device::Device()
{
	cfg.enable_stream(RS2_STREAM_COLOR);
    cfg.enable_stream(RS2_STREAM_INFRARED);
    cfg.enable_stream(RS2_STREAM_DEPTH);

    rs2::pipeline_profile selection = pipe.start(cfg);

    rs2::device selected_device = selection.get_device();

    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
        pipe.wait_for_frames();
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
    }

    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
        sleep(1);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
    }
	std::cout << "D455 Camera is ON\n";
}

void Device::get_pc(rs2::points& points, rs2::video_frame& color)
{
	for(int i = 0; i < 10; i++)
        frames = pipe.wait_for_frames();
    color = frames.get_color_frame();
    if (!color)
        color = frames.get_infrared_frame();
    pc.map_to(color);
    auto depth = frames.get_depth_frame();
    points = pc.calculate(depth);
}

void Device::convert_to_PCL(rs2::points& in_points, rs2::video_frame& in_color, cloud_pointer& cloud_ptr)
{

}

void Device::savePCD(pcl::PointCloud<pcl::PointXYZRGB>& pc)
{
	int ret = pcl::io::savePCDFile("test", pc);
}

Device::~Device()
{

}