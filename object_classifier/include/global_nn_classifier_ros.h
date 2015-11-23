#ifndef _V4R_GLOBAL_NN_CLASSIFIER_ROS_H__
#define _V4R_GLOBAL_NN_CLASSIFIER_ROS_H__

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
#include "segmentation_srv_definitions/segment.h"

#include <v4r/recognition/global_nn_classifier.h>
#include <v4r/segmentation/pcl_segmentation_methods.h>

namespace v4r
{

template<template<class > class Distance, typename PointT>
class GlobalNNClassifierROS : public GlobalNNClassifier<Distance, PointT>
{
private:
    using GlobalNNClassifier<Distance, PointT>::indices_;
    using GlobalNNClassifier<Distance, PointT>::input_;
    using GlobalNNClassifier<Distance, PointT>::training_dir_;

//    boost::shared_ptr<PCLSegmenter<pcl::PointXYZRGB> > seg_;  ///@brief segmentation object
    std::string models_dir_;

    double chop_at_z_;  /// @brief maximum depth to be considered when processing the input cloud (with respect to the camera coordinate system)

    // ROS stuff
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::ServiceServer segment_and_classify_service_;
    ros::ServiceServer classify_service_;
    ros::Publisher vis_pub_, vis_pc_pub_;
    visualization_msgs::MarkerArray markerArray_;

public:
    GlobalNNClassifierROS()
    {
    }

    bool classifyROS(classifier_srv_definitions::classify::Request &req,
                     classifier_srv_definitions::classify::Response &response);

    void initializeROS (int argc, char ** argv);

};

}

#endif
