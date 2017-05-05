#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "classifier_srv_definitions/segment_and_classify.h"
#include "classifier_srv_definitions/classify.h"
#include "object_perception_msgs/classification.h"
#include <v4r/recognition/global_recognizer.h>

namespace v4r
{

template<typename PointT>
class ClassifierROS
{
private:
    typename pcl::PointCloud<PointT>::Ptr cloud_; ///< input cloud
    typename GlobalRecognizer<PointT>::Ptr rec_; ///< recognizer

    // ROS stuff
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::ServiceServer classify_service_;
    ros::Publisher vis_pub_, vis_pc_pub_;
    visualization_msgs::MarkerArray markerArray_;

public:
    ClassifierROS()
    { }

    bool classify(classifier_srv_definitions::classify::Request &req,
                     classifier_srv_definitions::classify::Response &response);

    void initialize (int argc, char ** argv);

};

}
