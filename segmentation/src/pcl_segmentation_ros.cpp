#include "pcl_segmentation_ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <time.h>

#include <iostream>

#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <v4r/common/miscellaneous.h>
#include <v4r/common/normals.h>
#include <v4r/common/pcl_opencv.h>
#include <v4r/segmentation/plane_utils.h>
#include <v4r/io/filesystem.h>

namespace v4r
{

template<typename PointT> bool
SegmenterROS<PointT>::do_segmentation_ROS(segmentation_srv_definitions::segment::Request & req,
                    segmentation_srv_definitions::segment::Response & response)
{
    cloud_.reset(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(req.cloud, *cloud_);

    segmenter_.segment(cloud_);
    found_clusters_ = segmenter_.getClusters();
    return respondSrvCall(req, response);

}

template<typename PointT> bool
SegmenterROS<PointT>::respondSrvCall(segmentation_srv_definitions::segment::Request &req,
                            segmentation_srv_definitions::segment::Response &response) const
{
    typename pcl::PointCloud<PointT>::Ptr colored_cloud (new pcl::PointCloud<PointT>(*cloud_));

    for( PointT &p : colored_cloud->points)
        p.r = p.g = p.b = 0.f;

    for(const std::vector<int> &indices : found_clusters_)
    {
        const uint8_t r = rand()%255;
        const uint8_t g = rand()%255;
        const uint8_t b = rand()%255;

        std_msgs::Int32MultiArray indx;
        indx.data.reserve( indices.size() );

        for( int idx : indices )
        {
            PointT &p1 = colored_cloud->points[idx];
            p1.r = r;
            p1.g = g;
            p1.b = b;
            indx.data.push_back(idx);
        }
        response.clusters_indices.push_back(indx);
    }

    sensor_msgs::PointCloud2 colored_cloud_ros;
    pcl::toROSMsg (*colored_cloud, colored_cloud_ros);
    colored_cloud_ros.header.frame_id = req.cloud.header.frame_id;
    colored_cloud_ros.header.stamp = req.cloud.header.stamp;
    vis_pc_pub_.publish(colored_cloud_ros);

    v4r::PCLOpenCVConverter<PointT> img_conv;
    img_conv.setInputCloud(colored_cloud); //assumes organized cloud
    cv::Mat colored_img = img_conv.getRGBImage();

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", colored_img).toImageMsg();
    image_pub_.publish(msg);
    return true;
}

template<typename PointT> void
SegmenterROS<PointT>::initialize (int argc, char ** argv)
{
    ros::init (argc, argv, "pcl_segmentation_service");
    n_.reset( new ros::NodeHandle ( "~" ) );
    google::InitGoogleLogging(argv[0]);
    std::vector<std::string> arguments(argv + 1, argv + argc);
    segmenter_ .initialize(arguments);

    vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "segmented_cloud_colored", 1 );
    it_.reset(new image_transport::ImageTransport(*n_));
    image_pub_ = it_->advertise("segmented_cloud_colored_img", 1, true);
    segment_srv_ = n_->advertiseService ("pcl_segmentation", &SegmenterROS<PointT>::do_segmentation_ROS, this);
    std::cout << "Ready to get service calls..." << std::endl;
    ros::spin ();
}

}

int
main (int argc, char ** argv)
{
    srand (time(NULL));
    v4r::SegmenterROS<pcl::PointXYZRGB> s;
    s.initialize(argc, argv);
    return 0;
}
