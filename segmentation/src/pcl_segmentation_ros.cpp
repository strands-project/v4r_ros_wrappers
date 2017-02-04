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
    typename pcl::PointCloud<PointT>::Ptr colored_cloud (new pcl::PointCloud<PointT>());

    for(size_t i=0; i < found_clusters_.size(); i++)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>());
        pcl::copyPointCloud(*cloud_, found_clusters_[i], *cluster);

        const uint8_t r = rand()%255;
        const uint8_t g = rand()%255;
        const uint8_t b = rand()%255;
        for(size_t pt_id=0; pt_id<cluster->points.size(); pt_id++)
        {
            cluster->points[pt_id].r = r;
            cluster->points[pt_id].g = g;
            cluster->points[pt_id].b = b;
        }
        *colored_cloud += *cluster;

        std_msgs::Int32MultiArray indx;
        for(size_t k=0; k < found_clusters_[i].indices.size(); k++)
        {
            indx.data.push_back(found_clusters_[i].indices[k]);
        }
        response.clusters_indices.push_back(indx);
    }

    sensor_msgs::PointCloud2 segmented_cloud_colored;
    pcl::toROSMsg (*colored_cloud, segmented_cloud_colored);
    segmented_cloud_colored.header.frame_id = req.cloud.header.frame_id;
    vis_pc_pub_.publish(segmented_cloud_colored);

    //v4r::PCLOpenCVConverter<PointT> img_conv;
    //img_conv.setInputCloud(cloud_); //assumes organized cloud
    //cv::Mat colored_img = img_conv.getRGBImage();

    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", colored_img).toImageMsg();
    //image_pub_.publish(msg);
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


    if(method == v4r::SegmentationType::DominantPlane)
    {
        typename v4r::DominantPlaneSegmenter<PointT>::Parameter param;
        to_pass_further = param.init(to_pass_further);
        typename v4r::DominantPlaneSegmenter<PointT>::Ptr seg (new v4r::DominantPlaneSegmenter<PointT> (param));
        cast_segmenter = boost::dynamic_pointer_cast<v4r::Segmenter<PointT> > (seg);
    }
    else if(method == v4r::SegmentationType::MultiPlane)
    {
        typename v4r::MultiplaneSegmenter<PointT>::Parameter param;
        to_pass_further = param.init(to_pass_further);
        typename v4r::MultiplaneSegmenter<PointT>::Ptr seg (new v4r::MultiplaneSegmenter<PointT> (param));
        cast_segmenter = boost::dynamic_pointer_cast<v4r::Segmenter<PointT> > (seg);
    }
    else if(method == v4r::SegmentationType::EuclideanSegmentation)
    {
        typename v4r::EuclideanSegmenter<PointT>::Parameter param;
        to_pass_further = param.init(to_pass_further);
        typename v4r::EuclideanSegmenter<PointT>::Ptr seg (new v4r::EuclideanSegmenter<PointT> (param));
        cast_segmenter = boost::dynamic_pointer_cast<v4r::Segmenter<PointT> > (seg);
    }
    else if(method == v4r::SegmentationType::SmoothEuclideanClustering)
    {
        typename v4r::SmoothEuclideanSegmenter<PointT>::Parameter param;
        to_pass_further = param.init(to_pass_further);
        typename v4r::SmoothEuclideanSegmenter<PointT>::Ptr seg (new v4r::SmoothEuclideanSegmenter<PointT> (param));
        cast_segmenter = boost::dynamic_pointer_cast<v4r::Segmenter<PointT> > (seg);
    }


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
