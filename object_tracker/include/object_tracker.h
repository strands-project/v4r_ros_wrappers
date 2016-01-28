#ifndef OBJECT_TRACKER_ROS_H__
#define OBJECT_TRACKER_ROS_H__

/******************************************************************************
 * Copyright (c) 2015 Thomas Faeulhammer, Prankl Hannes,
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


#include "object_tracker_srv_definitions/start_tracker.h"
#include "object_tracker_srv_definitions/stop_tracker.h"
#include "object_tracker_srv_definitions/cleanup.h"
#include "object_tracker_srv_definitions/change_tracking_model.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <v4r/keypoints/ArticulatedObject.h>
#include <v4r/tracking/ObjectTrackerMono.h>

#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/grabber.h>
//#include <pcl/io/openni2_grabber.h>


namespace v4r
{

class ObjTrackerMono
{
private:

    boost::shared_ptr<ros::NodeHandle> n_;
    ros::ServiceServer cam_tracker_start_;
    ros::ServiceServer cam_tracker_stop_;
    ros::ServiceServer cam_tracker_cleanup_;
    ros::ServiceServer cam_tracker_change_model_;
    ros::Subscriber camera_topic_subscriber_;
    ros::Subscriber camera_info_subscriber_;
    ros::Publisher confidence_publisher_;
    ros::Publisher object_pose_publisher_;
    boost::shared_ptr<image_transport::ImageTransport> debug_image_transport_;
    image_transport::Publisher debug_image_publisher_;
    sensor_msgs::CameraInfo camera_info_;
    ros::Time last_cloud_ros_time_;
    boost::posix_time::ptime last_cloud_;
    std::string camera_topic_;
    bool got_camera_info_;
    bool use_cam_params_from_ROS_topic_;

    std::string log_dir_;
    int dbg_;

    int ao_sleep_;
    cv::Mat_<cv::Vec3b> image_;
    cv::Mat_<cv::Vec3b> im_draw_;

    cv::Mat_<double> dist_coeffs_;
    cv::Mat_<double> intrinsic_;
    cv::Mat_<double> src_dist_coeffs_;
    cv::Mat_<double> src_intrinsic_;
    Eigen::Matrix4f pose_;
    double conf_;
    v4r::ArticulatedObject::Ptr model_;

    std::string model_file_, cam_file_, cam_file_model_;
    std::vector<Eigen::Vector3f> cam_trajectory;
    bool log_cams_;
    bool log_files_;
    v4r::ObjectTrackerMono::Parameter param_;

    v4r::ObjectTrackerMono::Ptr tracker_;

    void
    drawCoordinateSystem(cv::Mat &im, const Eigen::Matrix4f &pose, const cv::Mat_<double> &intrinsic, const cv::Mat_<double> &dist_coeffs, double size, int thickness) const;


    void
    drawConfidenceBar(cv::Mat &im, const double &conf) const;

    bool
    start (object_tracker_srv_definitions::start_tracker::Request & req,
           object_tracker_srv_definitions::start_tracker::Response & response);


    bool
    stop (object_tracker_srv_definitions::stop_tracker::Request & req,
          object_tracker_srv_definitions::stop_tracker::Response & response);

    bool
    cleanup (object_tracker_srv_definitions::cleanup::Request & req,
             object_tracker_srv_definitions::cleanup::Response & response)
    {
        (void)req;
        (void)response;
        cleanup();
    }


    bool
    changeTrackingModel (object_tracker_srv_definitions::change_tracking_model::Request & req,
                           object_tracker_srv_definitions::change_tracking_model::Response & response);

    void
    cleanup()
    {
        conf_ = 0;
        pose_ = Eigen::Matrix4f::Identity();
    }

    void
    camera_info_cb(const sensor_msgs::CameraInfoPtr& msg)
    {
        camera_info_ = *msg;
        got_camera_info_=true;
    }

    void
    cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);

    void
    trackNewCloud(const sensor_msgs::PointCloud2Ptr& msg);


public:
    ObjTrackerMono()
    {
        log_dir_ = "/tmp/log/";
        dbg_ = 0;
        ao_sleep_ = 1;

        intrinsic_ = cv::Mat_<double>::eye(3,3);
        dist_coeffs_ = cv::Mat::zeros(4, 1, CV_64F);
        log_cams_ = false;
        log_files_ = false;

        param_.kt_param.plk_param.use_ncc = true;
        param_.kt_param.plk_param.ncc_residual = .5;

        tracker_.reset(new v4r::ObjectTrackerMono(param_));
        model_.reset(new ArticulatedObject());
        camera_topic_ = "/camera/depth_registered";
        use_cam_params_from_ROS_topic_ = true;
        got_camera_info_ = false;

        conf_ = 0;
        pose_ = Eigen::Matrix4f::Identity();
    }

    void setup(int argc, char **argv);

    void run();
};

}
#endif
