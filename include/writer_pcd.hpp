#include <Eigen/Eigen>
#include <pcl/pcl_exports.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <librealsense2/rs.hpp>
#include <boost/filesystem.hpp>

class PCDWriter
{
public:
    static PCDWriter *getInstance()
    {
        static PCDWriter instance;
        return &instance;
    }

    static std::string write(std::string filename, pcl::PointCloud<pcl::PointXYZRGB> &cloud, Eigen::Vector3f position = Eigen::Vector3f(.0f, .0f, .0f), Eigen::Quaternionf orientation = Eigen::Quaternionf(.0f, .0f, .0f, .0f))
    {
        return PCDWriter::getInstance()->Iwrite(filename, cloud, position, orientation);
    }

    static void set_folder(std::string path)
    {
        if (!boost::filesystem::exists(path))
            boost::filesystem::create_directory(path);
        PCDWriter::getInstance()->folder_path = path;
    }

private:
    std::string Iwrite(std::string &filename, pcl::PointCloud<pcl::PointXYZRGB> &cloud, Eigen::Vector3f &position, Eigen::Quaternionf &orientation);
    PCDWriter() = default;
    PCDWriter(PCDWriter const &) = delete;
    PCDWriter(PCDWriter const &&) = delete;
    void operator=(PCDWriter const &) = delete;
    void operator=(PCDWriter const &&) = delete;
    ~PCDWriter() = default;

private:
    std::string folder_path = "pointcloud";
    uint32_t file_index = 1;
    std::string prev_name;
};