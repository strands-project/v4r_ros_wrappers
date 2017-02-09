#include "pcl_segmentation_ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <time.h>

#include <iostream>

#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <v4r/common/normals.h>
#include <v4r/common/pcl_opencv.h>
#include <v4r/io/filesystem.h>

#include <boost/any.hpp>
#include <boost/program_options.hpp>
#include <glog/logging.h>

namespace po = boost::program_options;

namespace v4r
{

template<typename PointT> bool
SegmenterROS<PointT>::do_segmentation_ROS(segmentation_srv_definitions::segment::Request & req,
                    segmentation_srv_definitions::segment::Response & response)
{
    cloud_.reset(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(req.cloud, *cloud_);

    cast_segmenter->setInputCloud(cloud_);
    if(cast_segmenter->getRequiresNormals())
    {
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        v4r::computeNormals<PointT>(cloud_, normals, normal_computation_method_);
        cast_segmenter->setNormalsCloud( normals );
    }

    cast_segmenter->segment();
    cast_segmenter->getSegmentIndices( found_clusters_ );
    return respondSrvCall(req, response);

}

template<typename PointT> bool
SegmenterROS<PointT>::respondSrvCall(segmentation_srv_definitions::segment::Request &req,
                            segmentation_srv_definitions::segment::Response &response) const
{
    typename pcl::PointCloud<PointT>::Ptr colored_cloud (new pcl::PointCloud<PointT>(*cloud_));

    for( PointT &p : colored_cloud->points)
        p.r = p.g = p.b = 0.f;

    for(size_t i=0; i < found_clusters_.size(); i++)
    {
        const pcl::PointIndices &indices = found_clusters_[i];
        const uint8_t r = rand()%255;
        const uint8_t g = rand()%255;
        const uint8_t b = rand()%255;

        std_msgs::Int32MultiArray indx;
        for( int idx : indices.indices )
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

    int method = v4r::SegmentationType::DominantPlane;

    google::InitGoogleLogging(argv[0]);

    po::options_description desc("Point Cloud Segmentation Example\n======================================\n**Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("method", po::value<int>(&method)->default_value(method), "segmentation method used")
        ("normal_computation_method,n", po::value<int>(&normal_computation_method_)->default_value(normal_computation_method_), "normal computation method (if needed by segmentation approach)")
    ;
    po::variables_map vm;
    po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
    std::vector<std::string> to_pass_further = po::collect_unrecognized(parsed.options, po::include_positional);
    po::store(parsed, vm);
    if (vm.count("help")) { std::cout << desc << std::endl; }
    try { po::notify(vm); }
    catch(std::exception& e) { std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;  }

    cast_segmenter = v4r::initSegmenter<PointT>( method, to_pass_further );

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
    /* initialize random seed: */
    srand (time(NULL));
    v4r::SegmenterROS<pcl::PointXYZRGB> s;
    s.initialize(argc, argv);
    return 0;
}
