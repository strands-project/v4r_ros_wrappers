#ifndef CAMERA_TRACKER_ROS_H__
#define CAMERA_TRACKER_ROS_H__

/******************************************************************************
 * Copyright (c) 2014 Aitor Aldoma, Prankl Hannes, Thomas Faeulhammer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ******************************************************************************/


#include "camera_srv_definitions/start_tracker.h"
#include "camera_srv_definitions/stop_tracker.h"
#include "camera_srv_definitions/visualize_compound.h"
#include "camera_srv_definitions/get_tracking_results.h"
#include "camera_srv_definitions/save_tracking_results_to_file.h"
#include "camera_srv_definitions/do_ba.h"
#include "camera_srv_definitions/cleanup.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>

#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <v4r/reconstruction/KeypointSlamRGBD2.h>
#include <v4r/reconstruction/ProjBundleAdjuster.h>


//#define USE_PCL_GRABBER
#ifdef USE_PCL_GRABBER
    #include <pcl/io/grabber.h>
    #include <pcl/io/openni2_grabber.h>
#endif

class CamTracker
{
private:
    double conf_;
    Eigen::Matrix4f pose_;
    boost::shared_ptr<pcl::visualization::CloudViewer> viewer_;
    typedef pcl::PointXYZRGB PointT;
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::ServiceServer cam_tracker_start_;
    ros::ServiceServer cam_tracker_stop_;
    ros::ServiceServer cam_tracker_vis_compound_;
    ros::ServiceServer cam_tracker_do_ba_;
    ros::ServiceServer cam_tracker_get_tracking_results_;
    ros::ServiceServer cam_tracker_save_tracking_results_to_file_;
    ros::ServiceServer cam_tracker_cleanup_;

    ros::Subscriber camera_topic_subscriber_;
    ros::Subscriber camera_info_subscriber_;
    ros::Publisher confidence_publisher_;
    ros::Publisher trajectory_publisher_;
    ros::Publisher keyframe_publisher_;

    boost::shared_ptr<image_transport::ImageTransport> debug_image_transport_;
    image_transport::Publisher debug_image_publisher_;

    visualization_msgs::Marker trajectory_marker_;
    visualization_msgs::Marker keyframes_marker_;

    v4r::KeypointSlamRGBD2::Parameter param;
    v4r::KeypointSlamRGBD2::Ptr camtracker;

    // Distance threshold to the last position for adding new visualization markers to the trajectory
    double trajectory_threshold_;

    double cos_min_delta_angle_;
    double sqr_min_cam_distance_;
    bool use_cam_params_from_ROS_topic_;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > cameras_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> keyframes_;
    pcl::PointCloud<PointT>::Ptr scene_;
    int saved_clouds_;
    boost::posix_time::ptime last_cloud_;
    ros::Time last_cloud_ros_time_;
    std::string camera_topic_;
    sensor_msgs::CameraInfo camera_info_;
    bool got_camera_info_;
    tf::TransformBroadcaster cameraTransformBroadcaster;

    v4r::ProjBundleAdjuster ba;

#ifdef USE_PCL_GRABBER
    boost::shared_ptr<pcl::Grabber> interface;
#endif

    void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg)
    {
        camera_info_ = *msg;
        got_camera_info_=true;
    }

    void drawConfidenceBar(cv::Mat &im, const double &conf);

    int selectFrames(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, int cam_id, const Eigen::Matrix4f &pose);

    void trackNewCloud(const sensor_msgs::PointCloud2Ptr& msg);

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);

    bool
    cleanup ();

    bool
    cleanup (camera_srv_definitions::cleanup::Request & req,
             camera_srv_definitions::cleanup::Response & response)
    {
        (void)req;
        (void)response;
        cleanup();
    }


    bool
    start (camera_srv_definitions::start_tracker::Request & req,
           camera_srv_definitions::start_tracker::Response & response);


    bool
    stop (camera_srv_definitions::start_tracker::Request & req,
          camera_srv_definitions::start_tracker::Response & response);


    void createObjectCloudFiltered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & octree_cloud);


    bool
    doBA (camera_srv_definitions::do_ba::Request & req,
          camera_srv_definitions::do_ba::Response & response);


    bool
    getTrackingResults (camera_srv_definitions::get_tracking_results::Request & req,
                        camera_srv_definitions::get_tracking_results::Response & response);


    bool
    saveTrackingResultsToFile(camera_srv_definitions::save_tracking_results_to_file::Request &req,
                              camera_srv_definitions::save_tracking_results_to_file::Response &response);


    bool
    visCompound (camera_srv_definitions::visualize_compound::Request & req,
                 camera_srv_definitions::visualize_compound::Response & response);



public:
    CamTracker () : got_camera_info_(false), use_cam_params_from_ROS_topic_(true)
    {
        conf_ = 0;
        pose_ = Eigen::Matrix4f::Identity();
        cos_min_delta_angle_ = cos(15*M_PI/180.);
        sqr_min_cam_distance_ = 1.*1.;

        trajectory_threshold_ = 0.02;

        param.det_param.nfeatures = 150;
        param.kt_param.plk_param.use_ncc = true;
        param.kt_param.plk_param.ncc_residual = .5; //   (default .2)

        param.kd_param.rt_param.inl_dist = 0.01; //e.g. 0.01 .. table top, 0.03 ..rooms
        param.lk_param.rt_param.inl_dist = 0.01; //e.g. 0.01 .. table top, 0.03 ..rooms
        param.kt_param.rt_param.inl_dist = 0.03;  //e.g. 0.04 .. table top, 0.1 ..room
        param.om_param.kd_param.rt_param.inl_dist = 0.01; //e.g. 0.01 .. table top, 0.03 ..rooms
        param.om_param.kt_param.rt_param.inl_dist = 0.03;  //e.g. 0.04 .. table top, 0.1 ..room

        camera_topic_ = "/camera/depth_registered";
    }


    void
    initialize (int argc, char ** argv);


    void
    initializeVisualizationMarkers();
};

#endif
