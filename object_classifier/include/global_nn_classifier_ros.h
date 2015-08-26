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

namespace v4r
{

template<template<class > class Distance, typename PointT, typename FeatureT>
  class GlobalNNPipelineROS : public GlobalNNPipeline<Distance, PointT, FeatureT>
  {
  private:
      boost::shared_ptr<ros::NodeHandle> n_;

    std::string models_dir_;
    double chop_at_z_;
//    pcl::PointCloud<PointT>::Ptr frame_;
//    std::vector<pcl::PointIndices> cluster_indices_;
//    std::vector < std::string > categories_;
//    std::vector<float> conf_;
    ros::ServiceServer segment_and_classify_service_;
    ros::ServiceServer classify_service_;
    ros::Publisher vis_pub_, vis_pc_pub_;
    visualization_msgs::MarkerArray markerArray_;
    std::string camera_frame_;

  public:
    GlobalNNPipelineROS()
    {
        camera_frame_ = "/head_xtion_depth_optical_frame";
    }

    bool classifyROS(classifier_srv_definitions::classify::Request & req,
                  classifier_srv_definitions::classify::Response & response);

    void initializeROS (int argc, char ** argv);

  };

}

#endif
