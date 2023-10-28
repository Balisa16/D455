#include <device.hpp>


namespace EMIRO
{
    void frames_update(D455Data* data)
    {
        data->status = TStatus::Init;
        std::thread::id th_id = std::this_thread::get_id();
        std::cout << "Thread Detach in " << th_id << '\n';
        float maks_fps = 40.0f;
        std::cout << std::fixed << std::setprecision(2);

        while(data->thread_en)
        {
            while (data->lock.test_and_set(std::memory_order_acquire));

            data->frames = data->pipe.wait_for_frames();
            /*data->color = frames.get_color_frame();
            if (!data->color)
                data->color = frames.get_infrared_frame();
            data->pc.map_to(data->color);
            rs2::depth_frame depth = frames.get_depth_frame();
            data->point = data->pc.calculate(depth);*/
            data->status = TStatus::Available;
            
            // Release the lock
            data->lock.clear(std::memory_order_release);

            data->t_now = std::chrono::high_resolution_clock::now();
            std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>((*data).t_now - ((*data).t_past));
            data->t_past = data->t_now;
            float _fps = 1000000 / (float)duration.count();

            if(_fps > maks_fps) _fps = maks_fps;

            std::cout << "\033[1mFPS : [";

            // Bar color
            if((_fps/(float)maks_fps) < 0.3f)
                std::cout << "\033[31m";
            else if((_fps/(float)maks_fps) > 0.7f)
                std::cout << "\033[32m";
            else
                std::cout << "\033[33m";

            float _div = maks_fps/30.0f;
            for(float j = 0.f; j <= _fps; j += _div)
                std::cout << "▇";
            for(float j = _fps+.001f; j <= maks_fps; j += _div)
                std::cout << " ";

            std::cout << ' ' << _fps << '/' << maks_fps << "\033[0m]     \r";
            std::cout.flush();
        }
        std::cout << "Thread Finished" << std::string(45, ' ') << '\n';
        data->status = TStatus::Exit;
    }

    void Device::progress_bar(int i, int maks)
    {
        if((i/(float)maks) < 0.3f)
            std::cout << "\033[31m";
        else if((i/(float)maks) > 0.7f)
            std::cout << "\033[32m";
        else
            std::cout << "\033[33m";

        float _div = maks/30.0f;
        for(int j = 0; j <= i; j += _div)
            std::cout << "▇";
        for(int j = i+1; j <= maks; j += _div)
            std::cout << " ";

        std::cout << "\033[0m] " << (int)(i*100/maks);
        if(i < maks)
            std::cout << "%  \r";
        else
            std::cout << "% \033[32m[OK]\033[0m\n";
        std::cout.flush();
}

    Device::Device() : 
        builder(),
        writer(builder.newStreamWriter())
    {
        data.t_past = std::chrono::high_resolution_clock::now();
        check_dir();

        data.cfg.enable_stream(RS2_STREAM_DEPTH, 0, 848, 480, RS2_FORMAT_Z16, 30);

    	data.cfg.enable_stream(RS2_STREAM_COLOR);
        // cfg.enable_stream(RS2_STREAM_INFRARED);
        // cfg.enable_stream(RS2_STREAM_DEPTH);

        rs2::pipeline_profile selection = data.pipe.start(data.cfg);

        rs2::device selected_device = selection.get_device();

        auto depth_sensor = selected_device.first<rs2::depth_sensor>();

        if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        {
            depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
            data.pipe.wait_for_frames();
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
        th = std::thread(frames_update, &data);
        th.detach();
        while(data.status != TStatus::Available);
    }

    void Device::get_pc(rs2::points& p, rs2::video_frame& c)
    {
        while (data.lock.test_and_set(std::memory_order_acquire));
        data.color = data.frames.get_color_frame();
        if (!data.color)
            data.color = data.frames.get_infrared_frame();
        data.pc.map_to(data.color);
        rs2::depth_frame depth = data.frames.get_depth_frame();
        data.point = data.pc.calculate(depth);
        p = data.point;
        c = data.color;
        data.lock.clear(std::memory_order_release);
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
            if(Vertex[i].x > 5.0f || 
                Vertex[i].y > 5.0f ||
                Vertex[i].x > 5.0f) continue;
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
            std::cout << "\033[32mSaved " << formatted_name << "\033[0m";
        else
            std::cout << "\033[31mPCD Export FAILED\033[0m. Status : " << ret << '\n';
        std::cout << std::string(30, ' ') << '\n';
    }

    Device::~Device()
    {
        data.thread_en = false;
        while(data.status != TStatus::Exit);
        output_file.close();
    }
}