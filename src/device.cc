#include <device.hpp>


namespace EMIRO
{
    std::mutex mtx;

    void frames_update(
        rs2::pipeline pipe,
        rs2::pointcloud pc,
        rs2::points point,
        rs2::video_frame color,
        std::chrono::time_point<std::chrono::high_resolution_clock> t_now,
        std::chrono::time_point<std::chrono::high_resolution_clock> t_past)
    {
        rs2::frameset frames = pipe.wait_for_frames();
        color = frames.get_color_frame();
        if (!color)
            color = frames.get_infrared_frame();
        pc.map_to(color);
        rs2::depth_frame depth = frames.get_depth_frame();
        point = pc.calculate(depth);

        t_now = std::chrono::high_resolution_clock::now();
        std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(t_now - t_past);
        t_past = t_now;
        float _fps = 1000000 / duration.count();

        /*
        for(int i = 0; i <= loop; i++)
        {
            std::cout << "Frame : [";

            // Bar color
            if((i/(float)loop) < 0.3f)
                std::cout << "\033[31m";
            else if((i/(float)loop) > 0.7f)
                std::cout << "\033[32m";
            else
                std::cout << "\033[33m";

            float _div = loop/30.0f;
            for(int j = 0; j <= i; j += _div)
                std::cout << "▇";
            for(int j = i+1; j <= loop; j += _div)
                std::cout << " ";

            std::cout << "\033[0m] " << (int)(i*100/loop);
            if(i < loop)
                std::cout << "%  \r";
            else
                std::cout << "% \033[32m[OK]\033[0m\n";
            std::cout.flush();
        }
        */
        printf("FPS : %.2f  \r", _fps);
    }

    Device::Device() : 
        builder(),
        writer(builder.newStreamWriter())
    {
        t_past = std::chrono::high_resolution_clock::now();
        check_dir();

        cfg.enable_stream(RS2_STREAM_DEPTH, 0, 848, 480, RS2_FORMAT_Z16, 30);

    	cfg.enable_stream(RS2_STREAM_COLOR);
        // cfg.enable_stream(RS2_STREAM_INFRARED);
        // cfg.enable_stream(RS2_STREAM_DEPTH);

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

        // Take first
        rs2::frameset frames = pipe.wait_for_frames();
        color = frames.get_color_frame();;
        th = std::thread(frames_update, pipe, pc, point, color, t_now, t_past);
        th.detach();
    }

    void Device::get_pc(rs2::points& p, rs2::video_frame& c, int loop)
    {
        p = point;
        c = color;
    }

    void Device::check_dir(std::string folder)
    {
        out_folder = folder;
        std::vector<std::string> f;
        f.push_back(out_folder);
        f.push_back(out_folder + "/pointcloud");
        f.push_back(out_folder + "/resume");

        for (int i = 0; i < f.size(); ++i)
        {
            if(!boost::filesystem::exists(f[i]))
                if(!boost::filesystem::create_directory(f[i]))
                {
                    std::cout << "Failed create " << f[i] << '\n';
                    throw;
                }
        }


        // Check poitcloud folder
        int pc_folder_idx = 0;
        while(true)
        {
            bool en = false, empty = true;
            for (const boost::filesystem::directory_entry& entry : boost::filesystem::directory_iterator(out_folder + "/pointcloud")) {
                empty = false;
                if (boost::filesystem::is_directory(entry.path()) && std::atoi(entry.path().filename().string().c_str()) == pc_folder_idx)
                {
                    en = true;
                    break;
                }
                    
            }
            if(!en || empty)
                break;
            pc_folder_idx++;
        }

        boost::filesystem::create_directory(out_folder + "/pointcloud/" + std::to_string(pc_folder_idx));

        pc_folder = out_folder + "/pointcloud/" + std::to_string(pc_folder_idx) + '/';
        std::cout << "Pointcloud output in " <<  pc_folder << '\n';

        // Check folder for pointcloud
        output_file.open(out_folder + "/resume/resume" + std::to_string(pc_folder_idx) + ".json");
        if(!output_file.is_open())throw;
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

    void Device::convert_to_PCL(rs2::points& in_points, rs2::video_frame& in_color, pcl::PointCloud<pcl::PointXYZRGB>& output, float depth_lim)
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
            output.points[i].z = Vertex[i].x;
            output.points[i].y = Vertex[i].y;
            output.points[i].x = Vertex[i].z < depth_lim ? Vertex[i].z : depth_lim;
            
            RGB_Texture(in_color, Texture_Coord[i], temp_rgb);

            output.points[i].r = temp_rgb.r;
            output.points[i].g = temp_rgb.g;
            output.points[i].b = temp_rgb.b;
        }
    }

    void Device::savePCD(pcl::PointCloud<pcl::PointXYZRGB>& pc, std::string file_name)
    {
        std::string formatted_name = file_name + std::to_string(filename_idx) + ".pcd";

        // Get current time
        auto currentTime = std::chrono::system_clock::now();
        std::time_t time = std::chrono::system_clock::to_time_t(currentTime);
        std::tm timeInfo;
        localtime_r(&time, &timeInfo);
        char buffer[20];
        std::strftime(buffer, sizeof(buffer), "%Y/%m/%d %H:%M:%S", &timeInfo);

        root["filename"] = formatted_name.c_str();
        root["time"] = buffer;
        if(filename_idx > 1)
            output_file << ",\n";
        writer->write(root, &output_file);
        filename_idx++;

    	int ret = pcl::io::savePCDFile((pc_folder + formatted_name).c_str(), pc);
        if(ret == 0)
            std::cout << "\033[32mSaved " << formatted_name << "\033[0m\n";
        else
            std::cout << "\033[31mPCD Export FAILED\033[0m. Status : " << ret << '\n';
    }

    Device::~Device()
    {
        output_file.close();
    }
}