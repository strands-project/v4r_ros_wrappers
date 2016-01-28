#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include "object_tracker.h"

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <geometry_msgs/Transform.h>

#include <v4r/common/impl/ScopeTime.hpp>
#include <v4r/common/pcl_opencv.h>
#include <v4r/io/filesystem.h>
#include <v4r/keypoints/io.h>
#include <v4r/reconstruction/impl/projectPointToImage.hpp>

#include <boost/program_options.hpp>
#include <glog/logging.h>

namespace po = boost::program_options;

namespace v4r
{


void
ObjTrackerMono::drawConfidenceBar(cv::Mat &im, const double &conf) const
{
    int bar_start = 50, bar_end = 200;
    int diff = bar_end-bar_start;
    int draw_end = diff*conf;
    double col_scale = 255./(double)diff;
    cv::Point2f pt1(0,30);
    cv::Point2f pt2(0,30);
    cv::Vec3b col(0,0,0);

    if (draw_end<=0)
        draw_end = 1;

    for (int i=0; i<draw_end; i++)
    {
        col = cv::Vec3b(255-(i*col_scale), i*col_scale, 0);
        pt1.x = bar_start+i;
        pt2.x = bar_start+i+1;
        cv::line(im, pt1, pt2, CV_RGB(col[0],col[1],col[2]), 8);
    }
}


void
ObjTrackerMono::drawCoordinateSystem(cv::Mat &im, const Eigen::Matrix4f &pose, const cv::Mat_<double> &intrinsic, const cv::Mat_<double> &dist_coeffs, double size, int thickness) const
{
    Eigen::Matrix3f R = pose.topLeftCorner<3,3>();
    Eigen::Vector3f t = pose.block<3, 1>(0,3);

    Eigen::Vector3f pt0  = R * Eigen::Vector3f(0,0,0) + t;
    Eigen::Vector3f pt_x = R * Eigen::Vector3f(size,0,0) + t;
    Eigen::Vector3f pt_y = R * Eigen::Vector3f(0,size,0) + t;
    Eigen::Vector3f pt_z = R * Eigen::Vector3f(0,0,size) +t ;

    cv::Point2f im_pt0, im_pt_x, im_pt_y, im_pt_z;

    if (!dist_coeffs.empty())
    {
        v4r::projectPointToImage(&pt0 [0], &intrinsic(0), &dist_coeffs(0), &im_pt0.x );
        v4r::projectPointToImage(&pt_x[0], &intrinsic(0), &dist_coeffs(0), &im_pt_x.x);
        v4r::projectPointToImage(&pt_y[0], &intrinsic(0), &dist_coeffs(0), &im_pt_y.x);
        v4r::projectPointToImage(&pt_z[0], &intrinsic(0), &dist_coeffs(0), &im_pt_z.x);
    }
    else
    {
        v4r::projectPointToImage(&pt0 [0], &intrinsic(0), &im_pt0.x );
        v4r::projectPointToImage(&pt_x[0], &intrinsic(0), &im_pt_x.x);
        v4r::projectPointToImage(&pt_y[0], &intrinsic(0), &im_pt_y.x);
        v4r::projectPointToImage(&pt_z[0], &intrinsic(0), &im_pt_z.x);
    }

    cv::line(im, im_pt0, im_pt_x, CV_RGB(255,0,0), thickness);
    cv::line(im, im_pt0, im_pt_y, CV_RGB(0,255,0), thickness);
    cv::line(im, im_pt0, im_pt_z, CV_RGB(0,0,255), thickness);
}



void
ObjTrackerMono::setup(int argc, char **argv)
{
    po::options_description desc("Monocular Object Tracker\n======================================\n**Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("model_file,m", po::value<std::string>(&model_file_), "model file (.ao)  (pointcloud or stored model with .bin)")
            ("camera_topic,t", po::value<std::string>(&camera_topic_)->default_value(camera_topic_), "ROS camera topic")
            ("use_camera_info", po::value<bool>(&use_cam_params_from_ROS_topic_)->default_value(use_cam_params_from_ROS_topic_), "if set, listens to the camera info topic and sets the intrinsics accordingly. Otherwise, it uses default Kinect parameters.")
            ("cam_file,a", po::value<std::string>(&cam_file_), "camera calibration files for the tracking sequence (.yml) (opencv format)")
            ("log_dir", po::value<std::string>(&log_dir_)->default_value(log_dir_), "directory for logging output")
            ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help"))  {  std::cout << desc << std::endl;  return; }
    try { po::notify(vm); }
    catch(std::exception& e)  { std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl; return; }

    n_.reset( new ros::NodeHandle ( "~" ) );

    intrinsic_(0,0)=intrinsic_(1,1)=525;
    intrinsic_(0,2)=320, intrinsic_(1,2)=240;

    if (!cam_file_.empty())
    {
        cv::FileStorage fs( cam_file_, cv::FileStorage::READ );
        fs["camera_matrix"] >> intrinsic_;
        fs["distortion_coefficients"] >> dist_coeffs_;
    }
    if (!cam_file_model_.empty())
    {
        cv::FileStorage fs( cam_file_model_, cv::FileStorage::READ );
        fs["camera_matrix"] >> src_intrinsic_;
        fs["distortion_coefficients"] >> src_dist_coeffs_;
    }

    if(use_cam_params_from_ROS_topic_)
    {
        camera_info_subscriber_ = n_->subscribe(camera_topic_ +"/camera_info", 1, &ObjTrackerMono::camera_info_cb, this);

        ROS_INFO_STREAM("Wating for camera info...topic=" << camera_topic_ << "/camera_info...");
        while (!got_camera_info_) {
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        ROS_INFO("got it.");
        camera_info_subscriber_.shutdown();

        dist_coeffs_ = cv::Mat(4, 1, CV_64F, camera_info_.D.data());
        intrinsic_ = cv::Mat(3, 3, CV_64F, camera_info_.K.data());
    }

    std::cout << "Intrinsic camera parameters: " << std::endl << intrinsic_ << std::endl << std::endl <<
                 "Distortion camera parameters: " << std::endl << dist_coeffs_ << std::endl << std::endl;

    if(!src_intrinsic_.empty())
        tracker_->setObjectCameraParameter(src_intrinsic_, src_dist_coeffs_);

    tracker_->setCameraParameter(intrinsic_, dist_coeffs_);

    // -------------------- load model ---------------------------
    std::cout << "Load: "<< model_file_ << std::endl;

    if( v4r::io::read(model_file_, model_) ) {
        tracker_->setObjectModel(model_);
        cam_tracker_start_  = n_->advertiseService ("start_recording", &ObjTrackerMono::start, this);
    }
    else
        std::cout << "Tracking model file not set/found! Please call the service to /change_tracking_model to change the model filename." << std::endl;

    cam_tracker_change_model_ = n_->advertiseService("change_tracking_model", &ObjTrackerMono::changeTrackingModel, this);
    debug_image_transport_.reset(new image_transport::ImageTransport(*n_));
    debug_image_publisher_ = debug_image_transport_->advertise("debug_images", 1);
    ros::spin ();
}

bool
ObjTrackerMono::start (object_tracker_srv_definitions::start_tracker::Request & req,
                   object_tracker_srv_definitions::start_tracker::Response & response)
{
    (void) req;
    (void) response;

    cam_tracker_stop_  = n_->advertiseService ("stop_recording", &ObjTrackerMono::stop, this);
    cam_tracker_cleanup_  = n_->advertiseService ("cleanup", &ObjTrackerMono::cleanup, this);
    camera_topic_subscriber_ = n_->subscribe(camera_topic_ +"/points", 1, &ObjTrackerMono::trackNewCloud, this);
    confidence_publisher_ = n_->advertise<std_msgs::Float32>("object_tracker_confidence", 1);
    object_pose_publisher_ = n_->advertise<geometry_msgs::Transform>("object_pose", 1);

    cv::namedWindow( "image", CV_WINDOW_AUTOSIZE );
    return true;
}


bool
ObjTrackerMono::changeTrackingModel (object_tracker_srv_definitions::change_tracking_model::Request & req,
                                     object_tracker_srv_definitions::change_tracking_model::Response & response)
{
    (void) response;

    camera_topic_subscriber_.shutdown();
    confidence_publisher_.shutdown();
    object_pose_publisher_.shutdown();

    model_file_ = req.filename;

    std::cout << "Changing tracking model to " << model_file_ << std::endl;

    conf_ = 0;
    pose_ = Eigen::Matrix4f::Identity();

    if(v4r::io::read(model_file_, model_)) {
        tracker_->reset();
        if(!src_intrinsic_.empty())
            tracker_->setObjectCameraParameter(src_intrinsic_, src_dist_coeffs_);

        tracker_->setCameraParameter(intrinsic_, dist_coeffs_);
        tracker_->setObjectModel(model_);

        cam_tracker_start_.shutdown();
        cam_tracker_start_  = n_->advertiseService ("start_recording", &ObjTrackerMono::start, this);
        return true;
    }
    else {
        std::cerr << "Tracking model file not found!" << std::endl;
        return false;
    }
}

bool
ObjTrackerMono::stop (object_tracker_srv_definitions::stop_tracker::Request & req,
      object_tracker_srv_definitions::stop_tracker::Response & response)
{
    camera_topic_subscriber_.shutdown();
    confidence_publisher_.shutdown();
    object_pose_publisher_.shutdown();
    return true;
}

void
ObjTrackerMono::trackNewCloud(const sensor_msgs::PointCloud2Ptr& msg)
{
    double time;

    pcl::ScopeTime t("trackNewCloud");
    pcl::PointCloud<pcl::PointXYZRGB> cloud_tmp;
    pcl::moveFromROSMsg (*msg, cloud_tmp);
    image_ = v4r::ConvertPCLCloud2Image(cloud_tmp);

    image_.copyTo(im_draw_);

    if (dbg_)
        tracker_->dbg = im_draw_;

    bool is_ok;
    {
        v4r::ScopeTime t("overall time");
        is_ok = tracker_->track(image_, pose_, conf_);
        time = t.getTime();
    }

    std_msgs::Float32 confROS;
    confROS.data = conf_;
    confidence_publisher_.publish(confROS);

    if(debug_image_publisher_.getNumSubscribers())
    {
        drawConfidenceBar(im_draw_, conf_);

        if (conf_>0.05)
        {
            geometry_msgs::Transform tt;
            tt.translation.x = pose_(0,3);
            tt.translation.y = pose_(1,3);
            tt.translation.z = pose_(2,3);

            Eigen::Matrix3f rotation = pose_.block<3,3>(0,0);
            Eigen::Quaternionf q(rotation);
            tt.rotation.x = q.x();
            tt.rotation.y = q.y();
            tt.rotation.z = q.z();
            tt.rotation.w = q.w();
            object_pose_publisher_.publish(tt);

            if (log_cams_)
                cam_trajectory.push_back(pose_.block<3, 1>(0,3));

            drawCoordinateSystem(im_draw_, pose_, intrinsic_, dist_coeffs_, 0.1, 4);
        }

        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(msg->header, "bgr8", im_draw_).toImageMsg();
        debug_image_publisher_.publish(image_msg);
    }

    std::cout << "tracker conf = " << conf_ << std::endl << std::endl;

    if (is_ok)
        std::cout << "Status: stable tracking" << std::endl;
    else if (conf_ > 0.05)
        std::cout << "Status: unstable tracking" << std::endl;
    else
        std::cout << "Status: object lost!" << std::endl;
}

}


int
main (int argc, char ** argv)
{
    ros::init (argc, argv, "object_tracker");
    v4r::ObjTrackerMono ot;
    ot.setup (argc, argv);
    return 0;
}
