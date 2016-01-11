#include "pcl_segmentation_ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/common/common.h>
#include <pcl/io/io.h>
#include <v4r/common/pcl_opencv.h>
#include <time.h>

namespace v4r
{

template<typename PointT> bool
PCLSegmenterROS<PointT>::do_segmentation_ROS(segmentation_srv_definitions::segment::Request & req,
                    segmentation_srv_definitions::segment::Response & response)
{
    pcl::fromROSMsg(req.cloud, *input_cloud_);
    do_segmentation(found_clusters_);
    return respondSrvCall(req, response);

}

template<typename PointT> bool
PCLSegmenterROS<PointT>::respondSrvCall(segmentation_srv_definitions::segment::Request &req,
                            segmentation_srv_definitions::segment::Response &response) const
{
    typename pcl::PointCloud<PointT>::Ptr colored_cloud (new pcl::PointCloud<PointT>());

    for(size_t i=0; i < found_clusters_.size(); i++)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>());
        pcl::copyPointCloud(*input_cloud_, found_clusters_[i], *cluster);

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

    cv::Mat colored_img = ConvertUnorganizedPCLCloud2Image(*colored_cloud);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", colored_img).toImageMsg();
    image_pub_.publish(msg);
    return true;
}

template<typename PointT> void
PCLSegmenterROS<PointT>::initialize (int argc, char ** argv)
{
    ros::init (argc, argv, "pcl_segmentation_service");

    n_.reset( new ros::NodeHandle ( "~" ) );
    param_.seg_type_ = 1;
    n_->getParam ( "seg_type", param_.seg_type_ );
    n_->getParam ( "min_cluster_size", param_.min_cluster_size_ );
    n_->getParam ( "max_vertical_plane_size", param_.max_vertical_plane_size_ );
    n_->getParam ( "num_plane_inliers", param_.num_plane_inliers_ );
    n_->getParam ( "max_angle_plane_to_ground", param_.max_angle_plane_to_ground_ );
    n_->getParam ( "sensor_noise_max", param_.sensor_noise_max_ );
    n_->getParam ( "table_range_min", param_.table_range_min_ );
    n_->getParam ( "table_range_max", param_.table_range_max_ );
    n_->getParam ( "chop_z", param_.chop_at_z_ );
    n_->getParam ( "angular_threshold_deg", param_.angular_threshold_deg_ );

//    n_->getParam ( "camera_frame", camera_frame_ );
//    n_->getParam ( "base_frame", base_frame_ );

    vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "segmented_cloud_colored", 1 );
    it_.reset(new image_transport::ImageTransport(*n_));
    image_pub_ = it_->advertise("segmented_cloud_colored_img", 1, true);

    this->printParams();
    segment_srv_ = n_->advertiseService ("pcl_segmentation", &PCLSegmenterROS<PointT>::do_segmentation_ROS, this);
    std::cout << "Ready to get service calls..." << std::endl;
    ros::spin ();
}
}
int
main (int argc, char ** argv)
{
    /* initialize random seed: */
    srand (time(NULL));
    v4r::PCLSegmenterROS<pcl::PointXYZRGB> s;
    s.initialize(argc, argv);

    return 0;
}
