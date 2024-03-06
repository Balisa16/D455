#include <device.hpp>

namespace EMIRO
{
    void send_thread(std::shared_ptr<EMIRO::TCP> tcp_class, std::string filename, std::atomic_flag *lock_flag)
    {
        while (lock_flag->test_and_set(std::memory_order_acquire))
            ;
        tcp_class->send(filename);
        lock_flag->clear(std::memory_order_release);
    }

    void frames_update(D455Data *data)
    {
        data->status = TStatus::Init;
        std::thread::id th_id = std::this_thread::get_id();
        float maks_fps = 40.0f;
        std::cout << std::fixed << std::setprecision(2);
        rotation_estimator rot;

        while (data->thread_en)
        {
            while (data->lock.test_and_set(std::memory_order_acquire))
                ;

            data->frames = data->pipe.wait_for_frames();
            data->status = TStatus::Available;

            // Release the lock
            data->lock.clear(std::memory_order_release);

            data->t_now = std::chrono::high_resolution_clock::now();
            std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(data->t_now - data->t_past);
            data->t_past = data->t_now;
            float _fps = 1000000 / (float)duration.count();

            if (_fps > maks_fps)
                _fps = maks_fps;

            std::cout << "\033[2K\r\033[1mFPS : [";

            // Bar color
            if ((_fps / (float)maks_fps) < 0.3f)
                std::cout << "\033[31m";
            else if ((_fps / (float)maks_fps) > 0.7f)
                std::cout << "\033[32m";
            else
                std::cout << "\033[33m";

            float _div = maks_fps / 30.0f;
            for (float j = 0.f; j <= _fps; j += _div)
                std::cout << "▇";
            for (float j = _fps + .001f; j <= maks_fps; j += _div)
                std::cout << " ";

            std::cout << ' ' << _fps << '/' << maks_fps << "\033[0m]";
            std::cout.flush();
        }
        std::cout << "\033[2K\rThread Finished" << std::string(45, ' ') << '\n';
        data->status = TStatus::Exit;
    }

    void Device::progress_bar(int i, int maks)
    {
        std::cout << "\033[2K\r";
        if ((i / (float)maks) < 0.3f)
            std::cout << "\033[31m";
        else if ((i / (float)maks) > 0.7f)
            std::cout << "\033[32m";
        else
            std::cout << "\033[33m";

        float _div = maks / 30.0f;
        for (int j = 0; j <= i; j += _div)
            std::cout << "▇";
        for (int j = i + 1; j <= maks; j += _div)
            std::cout << " ";

        std::cout << "\033[0m] " << (int)(i * 100 / maks);
        if (i < maks)
            std::cout << "%";
        else
            std::cout << "% \033[32m[OK]\033[0m\n";
        std::cout.flush();
    }

    bool Device::check_imu()
    {
        bool found_gyro = false;
        bool found_accel = false;
        rs2::context ctx;
        for (auto dev : ctx.query_devices())
        {
            // The same device should support gyro and accel
            found_gyro = false;
            found_accel = false;
            for (auto sensor : dev.query_sensors())
            {
                for (auto profile : sensor.get_stream_profiles())
                {
                    if (profile.stream_type() == RS2_STREAM_GYRO)
                        found_gyro = true;

                    if (profile.stream_type() == RS2_STREAM_ACCEL)
                        found_accel = true;
                }
            }
            if (found_gyro && found_accel)
                break;
        }
        return found_gyro && found_accel;
    }

    Device::Device() : transfer_lock(ATOMIC_FLAG_INIT)
    {
        data.t_past = std::chrono::high_resolution_clock::now();

        if (!check_imu())
            throw std::runtime_error("\033[31mIMU is not Support\033[0m");

        data.cfg.enable_stream(RS2_STREAM_DEPTH, 0, 848, 480, RS2_FORMAT_Z16, 30);
        data.cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

        // data.cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

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
        while (data.status != TStatus::Available)
            ;

        // Configure file sender
        tcp_cl->connection("127.0.0.1", 8888);
    }

    void Device::get_pc(rs2::points &p, rs2::video_frame &c, Euler *euler)
    {
        while (data.lock.test_and_set(std::memory_order_acquire))
            ;
        data.color = data.frames.get_color_frame();
        if (!data.color)
            data.color = data.frames.get_infrared_frame();
        data.pc.map_to(data.color);
        rs2::depth_frame depth = data.frames.get_depth_frame();
        data.point = data.pc.calculate(depth);
        p = data.point;
        c = data.color;
        std::cout << std::fixed << std::setprecision(2);

        /*euler->roll = data.euler.roll;
        euler->pitch = data.euler.pitch;
        euler->yaw = data.euler.yaw;*/
        data.lock.clear(std::memory_order_release);
    }

    void Device::RGB_Texture(rs2::video_frame &texture, rs2::texture_coordinate Texture_XY, Color &out_RGB)
    {
        int width = texture.get_width();
        int height = texture.get_height();

        int x_value = std::min(std::max(int(Texture_XY.u * width + .5f), 0), width - 1);
        int y_value = std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);

        int bytes = x_value * texture.get_bytes_per_pixel();
        int strides = y_value * texture.get_stride_in_bytes();
        int Text_Index = (bytes + strides);

        const auto New_Texture = reinterpret_cast<const uint8_t *>(texture.get_data());

        // RGB components to save in tuple
        out_RGB.b = New_Texture[Text_Index];
        out_RGB.g = New_Texture[Text_Index + 1];
        out_RGB.r = New_Texture[Text_Index + 2];
    }

    void Device::convert_to_PCL(rs2::points &in_points, rs2::video_frame &in_color, pcl::PointCloud<pcl::PointXYZRGB> &output, float depth_lim)
    {
        auto sp = in_points.get_profile().as<rs2::video_stream_profile>();

        output.width = static_cast<uint32_t>(sp.width());
        output.height = static_cast<uint32_t>(sp.height());
        output.is_dense = false;
        output.points.resize(in_points.size());

        auto Texture_Coord = in_points.get_texture_coordinates();
        auto Vertex = in_points.get_vertices();

        Color temp_rgb;
        for (int i = 0; i < in_points.size(); i++)
        {
            if (Vertex[i].x > 5.0f ||
                Vertex[i].y > 5.0f ||
                Vertex[i].z > 5.0f)
                continue;
            output.points[i].x = Vertex[i].x;
            output.points[i].y = Vertex[i].y;
            output.points[i].z = Vertex[i].z < depth_lim ? Vertex[i].z : depth_lim;

            RGB_Texture(in_color, Texture_Coord[i], temp_rgb);

            output.points[i].r = temp_rgb.r;
            output.points[i].g = temp_rgb.g;
            output.points[i].b = temp_rgb.b;
        }
    }

    void Device::make_pointcloud(rs2::points *in_points, rs2::video_frame *in_color, PointCloud *pc)
    {
        // Remove temporary data
        pc->position.clear();
        pc->color.clear();
        pc->size = 0;

        auto sp = in_points->get_profile().as<rs2::video_stream_profile>();
        pc->height = sp.height();

        const rs2::texture_coordinate *texcoord = in_points->get_texture_coordinates();
        const rs2::vertex *vert = in_points->get_vertices();

        Color temp_rgb;
        for (int i = 0; i < in_points->size(); i++)
        {
            // Filter data
            if (std::abs(vert[i].x) > 5.0f || std::abs(vert[i].x) < 0.01f)
                continue;

            // Get match color
            RGB_Texture(*in_color, texcoord[i], temp_rgb);

            // Save position and color
            pc->add(
                {vert[i].x, vert[i].y, vert[i].z},
                {temp_rgb.r, temp_rgb.g, temp_rgb.b});
        }
        pc->width = std::ceil(pc->size / (float)pc->height);
    }

    void Device::store_pc(PointCloud *src, PointCloud *dest)
    {
        dest->width = src->width;
        dest->height = src->height;

        (*dest) += (*src);
    }

    void Device::get_orientation(Euler *euler)
    {
        while (data.lock.test_and_set(std::memory_order_acquire))
            ;

        rs2::pose_frame pose_frame = data.frames.get_pose_frame();

        if (pose_frame)
        {
            std::cout << "Orientation: ";
            // rs2_vector gyro_data = motion.get_motion_data();

            // euler->roll = gyro_data.z;
            // euler->pitch = gyro_data.x;
            // euler->yaw = gyro_data.y;
        }
        else
            std::cout << "Failed update orientation\n";

        data.lock.clear(std::memory_order_release);
    }

    void Device::sendPCD(std::string file_path)
    {
        // Sending file into GCS
        tcp_th = std::thread(send_thread, tcp_cl, file_path, &transfer_lock);
        tcp_th.detach();
    }

    rs2::points Device::clean_pc(rs2::points &in_points)
    {
        rs2::points out_pc;

        return out_pc;
    }

    Device::~Device()
    {
        data.thread_en = false;
        while (data.status != TStatus::Exit)
            ;

        // Make sure transfer data is done before ending the class
        while (transfer_lock.test_and_set(std::memory_order_acquire))
            ;
        transfer_lock.clear(std::memory_order_release);
    }
}