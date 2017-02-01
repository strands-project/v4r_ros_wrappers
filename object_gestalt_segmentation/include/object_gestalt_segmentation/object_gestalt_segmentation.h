/*
 * object_gestalt_segmentation.hpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Ekaterina Potapova (with adaptions from Markus Suchi for v4r-ros-wrappers)
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
 
#include <sstream>

#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>

#include <v4r/attention_segmentation/PCLUtils.h>
#include <v4r/attention_segmentation/segmentation.h>
#include <v4r/attention_segmentation/EPUtils.h>
#include <v4r/attention_segmentation/AttentionModule.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "segmentation_srv_definitions/segment.h"

#ifndef OBJECT_GESTALT_SEGMENTATION_HPP_
#define OBJECT_GESTALT_SEGMENTATION_HPP_

class SegmenterComplete
{
private:
  typedef pcl::PointXYZRGB PointT;
  ros::ServiceServer Segment_;
  ros::NodeHandle *n_;
  boost::shared_ptr<v4r::Segmenter> segmenter_;
  std::string model_filename_, scaling_filename_;

  bool
  segment (segmentation_srv_definitions::segment::Request & req, segmentation_srv_definitions::segment::Response & response);
  
public:
  SegmenterComplete ();
  ~SegmenterComplete ();

  void
  initialize (int argc, char ** argv);
};

#endif //OBJECT_GESTALT_SEGMENTATION_HPP_
