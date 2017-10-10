#pragma once

#include <ros/ros.h>

#include "segmentation_srv_definitions/segment.h"
#include <image_transport/image_transport.h>

#include <v4r/apps/CloudSegmenter.h>
#include <v4r/segmentation/all_headers.h>
#include <v4r/segmentation/types.h>
#include <v4r/common/normals.h>

namespace v4r
{
template<typename PointT>
class SegmenterROS
{

private:
    typename pcl::PointCloud<PointT>::Ptr cloud_;
    ros::ServiceServer segment_srv_;
    boost::shared_ptr<ros::NodeHandle> n_;
    std::vector<std::vector<int> > found_clusters_;
    ros::Publisher vis_pc_pub_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher image_pub_;

    v4r::apps::CloudSegmenter<PointT> segmenter_;


public:
    SegmenterROS()
    { }

    void initialize(int argc, char ** argv);

    bool do_segmentation_ROS(segmentation_srv_definitions::segment::Request & req,
                             segmentation_srv_definitions::segment::Response & response);

    bool respondSrvCall(segmentation_srv_definitions::segment::Request &req,
                                segmentation_srv_definitions::segment::Response &response) const;
};

}
