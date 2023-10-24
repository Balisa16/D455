#include <device.hpp>


Device::Device()
{
    // Check folder for pointcloud
    check_dir();

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

void Device::get_pc(rs2::points& points, rs2::video_frame& color, int loop)
{
	for(int i = 0; i < loop; i++)
    {
        frames = pipe.wait_for_frames();
        std::cout << "Frame : [";
        float _div = loop/10.0f;
        for(int j = 0; j < loop; j += _div)
            std::cout << "█";
        std::cout << "] " << (int)(i*100/loop) << "%     \r";
        std::cout.flush();
    }
    std::cout << '\n';
    color = frames.get_color_frame();
    if (!color)
        color = frames.get_infrared_frame();
    pc.map_to(color);
    auto depth = frames.get_depth_frame();
    points = pc.calculate(depth);
}

void Device::check_dir(std::string folder)
{
    pc_folder = folder;
    if(!boost::filesystem::exists(pc_folder))
    {
        std::cout << "Create pointcloud folder\n";
        if(!boost::filesystem::create_directory(pc_folder))
        {
            std::cout << "Failed create pointcloud folder\n";
            throw;
        }
    }

    bool is_empty = true;
    for (boost::filesystem::directory_iterator it(pc_folder); it != boost::filesystem::directory_iterator(); ++it)
    {
        is_empty =  false;
        break;
    }

    if(!is_empty)
    {
        std::cout << "Pointcloud folder isn't empty. Delete all file ? [y/n] ";
        std::cout.flush();
        char in_char;
        std::cin >> in_char;
        if(in_char == 'y' || in_char == 'Y')
        {
            std::cout << "\033[32mDelete all\033[0m file in pointcloud folder\n";
            for (boost::filesystem::directory_iterator it(pc_folder); it != boost::filesystem::directory_iterator(); ++it) {
                if (boost::filesystem::is_regular_file(it->status())) {
                    boost::filesystem::remove(it->path());
                }
            }
        }
    }
}

void Device::RGB_Texture(rs2::video_frame& texture, rs2::texture_coordinate Texture_XY, RGB& out_RGB)
{
    int width  = texture.get_width();
    int height = texture.get_height();
    
    int x_value = std::min(std::max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();
    int strides = y_value * texture.get_stride_in_bytes();
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    // RGB components to save in tuple
    out_RGB.b = New_Texture[Text_Index];
    out_RGB.g = New_Texture[Text_Index + 1];
    out_RGB.r = New_Texture[Text_Index + 2];
}

void Device::convert_to_PCL(rs2::points& in_points, rs2::video_frame& in_color, pcl::PointCloud<pcl::PointXYZRGB>& output)
{
    auto sp = in_points.get_profile().as<rs2::video_stream_profile>();
    
    output.width  = static_cast<uint32_t>( sp.width()  );   
    output.height = static_cast<uint32_t>( sp.height() );
    output.is_dense = false;
    output.points.resize( in_points.size() );

    auto Texture_Coord = in_points.get_texture_coordinates();
    auto Vertex = in_points.get_vertices();

    RGB temp_rgb;
    for (int i = 0; i < in_points.size(); i++)
    {
        output.points[i].x = Vertex[i].x;
        output.points[i].y = Vertex[i].y;
        output.points[i].z = Vertex[i].z;

        RGB_Texture(in_color, Texture_Coord[i], temp_rgb);

        output.points[i].r = temp_rgb.r;
        output.points[i].g = temp_rgb.g;
        output.points[i].b = temp_rgb.b;
    }
}

void Device::savePCD(pcl::PointCloud<pcl::PointXYZRGB>& pc, std::string file_name)
{
	int ret = pcl::io::savePCDFile((pc_folder + "/" + file_name).c_str(), pc);
    std::cout << "Save status : " << ret << '\n';
}

Device::~Device()
{

}