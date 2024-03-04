#include <writer_pcd.hpp>

std::string PCDWriter::Iwrite(std::string &filename, pcl::PointCloud<pcl::PointXYZRGB> &cloud, Eigen::Vector3f &position, Eigen::Quaternionf &orientation)
{
    int current_size = cloud.size();
    cloud.width = std::ceil(current_size / (float)cloud.height);

    // Fill blank data
    int est_data = cloud.height * cloud.width;
    int need_data = est_data - current_size;
    cloud.points.resize(est_data);
    for (int i = 0; i < need_data; i++)
    {
        int pos = current_size + i;
        cloud.points[pos] = cloud.points[pos - cloud.height];
    }

    // Set sample position and sample quaternion
    cloud.sensor_origin_ = {position.x(), position.y(), position.z(), 1.0f};
    cloud.sensor_orientation_ = {orientation.w(), orientation.x(), orientation.y(), orientation.z()};

    std::string formatted_name = filename + std::to_string(file_index) + ".pcd";

    std::string pcd_path = folder_path + '/' + formatted_name;
    int ret = pcl::io::savePCDFile(pcd_path.c_str(), cloud);

    if (ret == 0)
        std::cout << "\033[2K\r\033[32mSaved " << formatted_name << "\033[0m\n";
    else
        std::cout << "\033[2K\r\033[31mPCD Export FAILED\033[0m. Status : " << ret << '\n';

    file_index++;
    prev_name = filename;
    return pcd_path;
}