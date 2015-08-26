#ifndef V4R_PCL_SEGMENTATION_ROS_H__
#define V4R_PCL_SEGMENTATION_ROS_H__

#include <v4r/segmentation/pcl_segmentation_methods.h>

#include <ros/ros.h>

#include "segmentation_srv_definitions/segment.h"
#include <image_transport/image_transport.h>

namespace v4r
{
template<typename PointT>
class PCLSegmenterROS : public PCLSegmenter<PointT>
{
    using PCLSegmenter<PointT>::param_;
    using PCLSegmenter<PointT>::input_cloud_;
    using PCLSegmenter<PointT>::do_segmentation;

private:
    ros::ServiceServer segment_srv_;
    boost::shared_ptr<ros::NodeHandle> n_;
    std::vector<pcl::PointIndices> found_clusters_;
    ros::Publisher vis_pc_pub_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher image_pub_;

public:
    void initialize(int argc, char ** argv);

    bool do_segmentation_ROS(segmentation_srv_definitions::segment::Request & req,
                             segmentation_srv_definitions::segment::Response & response);

    bool respondSrvCall(segmentation_srv_definitions::segment::Request &req,
                                segmentation_srv_definitions::segment::Response &response) const;
};

}

#endif
