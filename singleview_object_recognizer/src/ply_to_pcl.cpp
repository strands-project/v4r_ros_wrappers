#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "");
    ros::NodeHandle n("~");
    std::string file_name;
    if (! (n.getParam("file", file_name)))
    {
        ROS_ERROR("No filename with argument \"file\" given.");
        return -1;
    }
    else
    {
        std::cout << "Converting ply file " << file_name << " to a point cloud. " << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pModel(new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::PLYReader plyReader;
    plyReader.read(file_name, *pModel);
    pcl::visualization::PCLVisualizer vis;
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> random_handler (pModel);

    vis.addPointCloud<pcl::PointXYZ>(pModel, "model");
    vis.spin();
    return 0;
}
