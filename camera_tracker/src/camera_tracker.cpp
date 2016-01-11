/*
 * camera_tracker.cpp
 *
 *  Created on: June, 2015
 *      Author: Aitor Aldoma, Thomas Faeulhammer
 */

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include "camera_tracker.h"

#include <thread>

#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>

#include <v4r/common/impl/ScopeTime.hpp>
#include <v4r/common/noise_models.h>
#include <v4r/common/convertCloud.h>
#include <v4r/common/convertImage.h>
#include <v4r/common/PointTypes.h>
#include <v4r/registration/noise_model_based_cloud_integration.h>
#include <v4r/io/filesystem.h>
#include <v4r/keypoints/impl/invPose.hpp>


void
CamTracker::drawConfidenceBar(cv::Mat &im, const double &conf)
{
    int bar_start = 50, bar_end = 200;
    int diff = bar_end-bar_start;
    int draw_end = diff*conf;
    double col_scale = 255./(double)diff;
    cv::Point2f pt1(0,30);
    cv::Point2f pt2(0,30);
    cv::Vec3b col(0,0,0);

    if (draw_end<=0) draw_end = 1;

    for (int i=0; i<draw_end; i++)
    {
        col = cv::Vec3b(255-(i*col_scale), i*col_scale, 0);
        pt1.x = bar_start+i;
        pt2.x = bar_start+i+1;
        cv::line(im, pt1, pt2, CV_RGB(col[0],col[1],col[2]), 8);
    }
}

int CamTracker::selectFrames(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                             int cam_id, const Eigen::Matrix4f &pose)
{
    int type = 0;

    //        if (cam_id>=0)
    {
        type = 1;

        Eigen::Matrix4f inv_pose;
        v4r::invPose(pose, inv_pose);

        bool close_view_exists = false;
        for (size_t z=0; z<cameras_.size(); z++)
        {
            if ( (inv_pose.block<3,1>(0,2).dot(cameras_[z].block<3,1>(0,2)) > cos_min_delta_angle_) &&
                 (inv_pose.block<3,1>(0,3)-cameras_[z].block<3,1>(0,3)).squaredNorm() < sqr_min_cam_distance_ )
            {
                close_view_exists = true;
                break;
            }
        }

        if ( !close_view_exists )
        {
            type = 2;
            cameras_.push_back(inv_pose);
            pcl::PointCloud<pcl::PointXYZRGB> new_keyframe;
            pcl::copyPointCloud(cloud, new_keyframe);

            new_keyframe.sensor_origin_[0] = inv_pose(0,3);
            new_keyframe.sensor_origin_[1] = inv_pose(1,3);
            new_keyframe.sensor_origin_[2] = inv_pose(2,3);

            Eigen::Matrix3f rotation = inv_pose.block<3,3>(0,0);
            Eigen::Quaternionf q(rotation);
            new_keyframe.sensor_orientation_ = q;
            keyframes_.push_back(new_keyframe);
            std::cout << "Added new keyframe**********************************************************" << std::endl;

            geometry_msgs::Point p;
            p.x = -pose(0,3);
            p.y = -pose(1,3);
            p.z = -pose(2,3);
            keyframes_marker_.points.push_back(p);
            keyframes_marker_.header.stamp = ros::Time::now();

            keyframe_publisher_.publish(keyframes_marker_);
        }
        //            else
        //                std::cout << "--- not adding keyframe " << std::endl;
    }
    //        else
    //            std::cout << "00000  camID 000000 " << std::endl;
    return type;
}

void
CamTracker::trackNewCloud(const sensor_msgs::PointCloud2Ptr& msg)
{
    ros::Time start_time_stamp = msg->header.stamp;

    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time ();
    //std::cout << (start_time - last_cloud_).total_nanoseconds () * 1.0e-9 << std::endl;

    float time_ms = (start_time_stamp - last_cloud_ros_time_).toSec() * 1e3;

    last_cloud_ = start_time;
    last_cloud_ros_time_ = start_time_stamp;

    pcl::ScopeTime t("trackNewCloud");
    scene_.reset(new pcl::PointCloud<PointT>);
    pcl::moveFromROSMsg (*msg, *scene_);

    v4r::DataMatrix2D<Eigen::Vector3f> kp_cloud;
    cv::Mat_<cv::Vec3b> image;

    v4r::convertCloud(*scene_, kp_cloud, image);

    int cam_idx=-1;

    bool is_ok = camtracker->track(image, kp_cloud, pose_, conf_, cam_idx);

    if(debug_image_publisher_.getNumSubscribers())
    {
        drawConfidenceBar(image, conf_);
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();
        debug_image_publisher_.publish(image_msg);
    }

    std::cout << time_ms << " conf:" << conf_ << std::endl;

    if(is_ok)
    {
        selectFrames(*scene_, cam_idx, pose_);
        tf::Transform transform;

        //v4r::invPose(pose, inv_pose);
        transform.setOrigin(tf::Vector3(pose_(0,3), pose_(1,3), pose_(2,3)));
        tf::Matrix3x3 R(pose_(0,0), pose_(0,1), pose_(0,2),
                        pose_(1,0), pose_(1,1), pose_(1,2),
                        pose_(2,0), pose_(2,1), pose_(2,2));
        tf::Quaternion q;
        R.getRotation(q);
        transform.setRotation(q);
        ros::Time now_sync = ros::Time::now();
        cameraTransformBroadcaster.sendTransform(tf::StampedTransform(transform, now_sync, "camera_rgb_optical_frame", "world"));

        bool publish_trajectory = true;
        if (!trajectory_marker_.points.empty())
        {
            const geometry_msgs::Point& last = trajectory_marker_.points.back();
            Eigen::Vector3f v_last(last.x, last.y, last.z);
            Eigen::Vector3f v_curr(-pose_.col(3).head<3>());
            if ((v_last - v_curr).norm() < trajectory_threshold_)
                publish_trajectory = false;
        }

        if (publish_trajectory)
        {
            geometry_msgs::Point p;
            p.x = -pose_(0,3);
            p.y = -pose_(1,3);
            p.z = -pose_(2,3);
            std_msgs::ColorRGBA c;
            c.a = 1.0;
            c.g = conf_;
            trajectory_marker_.points.push_back(p);
            trajectory_marker_.colors.push_back(c);
            trajectory_marker_.header.stamp = msg->header.stamp;
            trajectory_publisher_.publish(trajectory_marker_);
        }
    }
    else
        std::cout << "cam tracker not ready!" << std::endl;

    /*std_msgs::Float32 conf_mesage;
      conf_mesage.data = conf;
      confidence_publisher_.publish(conf_mesage);*/
}

void
CamTracker::cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
    sensor_msgs::PointCloud2Ptr pMsg (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud, *pMsg);
    trackNewCloud(pMsg);
}

bool
CamTracker::cleanup ()
{
    cameras_.clear();
    keyframes_.clear();
    saved_clouds_ = 0;
    conf_ = 0;
    pose_ = Eigen::Matrix4f::Identity();

    return true;
}

bool
CamTracker::start (camera_srv_definitions::start_tracker::Request & req,
                   camera_srv_definitions::start_tracker::Response & response)
{
    (void) req;
    (void) response;
    cleanup();
    initializeVisualizationMarkers();

    cv::Mat_<double> distCoeffs = cv::Mat_<double>::zeros(4,1);
    cv::Mat_<double> intrinsic = cv::Mat::zeros(3, 3, CV_64F);
    intrinsic.at<double>(0,0) = 525.f;
    intrinsic.at<double>(1,1) = 525.f;
//    intrinsic.at<double>(0,2) = 320.f;
//    intrinsic.at<double>(1,2) = 240.f;
    intrinsic.at<double>(0,2) = 319.5f;
    intrinsic.at<double>(1,2) = 239.0f;
    intrinsic.at<double>(2,2) = 1.f;

#ifdef USE_PCL_GRABBER
    try
    {
        interface.reset( new pcl::io::OpenNI2Grabber() );
    }
    catch (pcl::IOException e)
    {
        std::cout << "PCL threw error " << e.what()
                  << ". Could not start camera..." << std::endl;
        return false;
    }
    camtracker.reset( new v4r::KeypointSlamRGBD2(param) );
    camtracker->setCameraParameter(intrinsic,distCoeffs);

    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
            boost::bind (&CamTracker::cloud_cb_, this, _1);
    interface->registerCallback (f);
    interface->start ();
#else
    if(use_cam_params_from_ROS_topic_)
    {
        camera_info_subscriber_ = n_->subscribe(camera_topic_ +"/camera_info", 1, &CamTracker::camera_info_cb, this);

        ROS_INFO_STREAM("Wating for camera info...topic=" << camera_topic_ << "/camera_info...");
        while (!got_camera_info_) {
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        ROS_INFO("got it.");
        camera_info_subscriber_.shutdown();

        distCoeffs = cv::Mat(4, 1, CV_64F, camera_info_.D.data());
        intrinsic = cv::Mat(3, 3, CV_64F, camera_info_.K.data());
    }
    std::cout << "Intrinsic camera parameters: " << std::endl << intrinsic << std::endl << std::endl <<
                 "Distortion camera parameters: " << std::endl << distCoeffs << std::endl << std::endl;

    camtracker.reset( new v4r::KeypointSlamRGBD2(param) );
    camtracker->setCameraParameter(intrinsic, distCoeffs);

    confidence_publisher_ = n_->advertise<std_msgs::Float32>("cam_tracker_confidence", 1);
    camera_topic_subscriber_ = n_->subscribe(camera_topic_ +"/points", 1, &CamTracker::trackNewCloud, this);
#endif
    std::cout << "Camera started..." << std::endl;
    last_cloud_ = boost::posix_time::microsec_clock::local_time ();
    last_cloud_ros_time_ = ros::Time::now();
    return true;
}

bool
CamTracker::stop (camera_srv_definitions::start_tracker::Request & req,
      camera_srv_definitions::start_tracker::Response & response)
{
#ifdef USE_PCL_GRABBER
    if(interface.get())
        interface->stop();
#else
    camera_topic_subscriber_.shutdown();
#endif
    if (!camtracker.empty())
        camtracker->stopObjectManagement();
    return true;
}


void
CamTracker::createObjectCloudFiltered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & octree_cloud)
{
    v4r::NguyenNoiseModel<pcl::PointXYZRGB> nm;
    std::vector<std::vector<std::vector<float> > > pt_properties (keyframes_.size());
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > ptr_clouds(keyframes_.size());
    std::vector< pcl::PointCloud<pcl::Normal>::Ptr > normals(keyframes_.size());

    if (!keyframes_.empty())
    {
        for (unsigned i=0; i<keyframes_.size(); i++)
        {

            ptr_clouds[i].reset(new pcl::PointCloud<PointT>(keyframes_[i]));
            pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
            ne.setRadiusSearch(0.01f);
            ne.setInputCloud (ptr_clouds[i]);
            normals[i].reset(new pcl::PointCloud<pcl::Normal>());
            ne.compute (*normals[i]);

            nm.setInputCloud(ptr_clouds[i]);
            nm.setInputNormals(normals[i]);
            nm.compute();
            pt_properties[i] = nm.getPointProperties();
        }

        v4r::NMBasedCloudIntegration<pcl::PointXYZRGB>::Parameter nmParam;
        nmParam.octree_resolution_ = 0.005f;
        nmParam.min_points_per_voxel_ = 1;
        v4r::NMBasedCloudIntegration<pcl::PointXYZRGB> nmIntegration (nmParam);
        octree_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        nmIntegration.setInputClouds(ptr_clouds);
        nmIntegration.setTransformations(cameras_);
        nmIntegration.setInputNormals(normals);
        nmIntegration.setPointProperties(pt_properties);
        nmIntegration.compute(octree_cloud);
    }
}

bool
CamTracker::doBA (camera_srv_definitions::do_ba::Request & req,
                  camera_srv_definitions::do_ba::Response & response)
{
    if(cameras_.empty())
    {
        ROS_WARN("Called bundle adjusment but no camera poses available\n");
        return false;
    }

    v4r::Object &model = camtracker->getModel();
    ba.optimize(model);

    for(size_t i=0; i < cameras_.size(); i++)
    {
        Eigen::Matrix4f inv_pose_after_ba;
        v4r::invPose(model.cameras[i], inv_pose_after_ba);
        cameras_[i] = inv_pose_after_ba;
    }
    return true;
}

bool
CamTracker::getTrackingResults (camera_srv_definitions::get_tracking_results::Request & req,
                                camera_srv_definitions::get_tracking_results::Response & response)
{
    for(size_t i=0; i < cameras_.size(); i++)
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(keyframes_[i], msg);
        response.keyframes.push_back(msg);

        Eigen::Matrix4f trans = cameras_[i];

        geometry_msgs::Transform tt;
        tt.translation.x = trans(0,3);
        tt.translation.y = trans(1,3);
        tt.translation.z = trans(2,3);

        Eigen::Matrix3f rotation = trans.block<3,3>(0,0);
        Eigen::Quaternionf q(rotation);
        tt.rotation.x = q.x();
        tt.rotation.y = q.y();
        tt.rotation.z = q.z();
        tt.rotation.w = q.w();
        response.transforms.push_back(tt);
    }

    return true;
}

bool
CamTracker::saveTrackingResultsToFile(camera_srv_definitions::save_tracking_results_to_file::Request &req,
                                      camera_srv_definitions::save_tracking_results_to_file::Response &response)
{
    using namespace boost::filesystem;
    try
    {
        file_status s = status(req.dir_name.data);
        if(is_regular_file(s))
        {
            ROS_ERROR("Failed to save tracking results: expected directory name, got filename");
            return false;
        }
        else
        {
            v4r::io::createDirIfNotExist(req.dir_name.data);
            for(size_t i=0; i < cameras_.size(); i++)
            {
                std::stringstream fn; fn << req.dir_name.data << "/cloud_" << std::setw(5) << std::setfill('0') << i << ".pcd";
                std::cout << "Writing file to " << fn.str() << "." << std::endl;
                pcl::io::savePCDFileBinary(fn.str(), keyframes_[i]);
            }
        }
    }
    catch(filesystem_error& e)
    {
        ROS_ERROR("Failed to save tracking results: %s", e.what());
        return false;
    }

    return true;
}

bool
CamTracker::visCompound (camera_srv_definitions::visualize_compound::Request & req,
                         camera_srv_definitions::visualize_compound::Response & response)
{
    (void)response;
    if(cameras_.empty())
        return false;

    bool do_ba_ = req.do_ba.data;
    std::cout << "!Number of keyframes: " << cameras_.size() << " " << do_ba_ << std::endl;
    pcl::visualization::PCLVisualizer vis("compound cloud");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr compound(new pcl::PointCloud<pcl::PointXYZRGB>);

    //Bundle adjustment
    if(do_ba_)
    {
        v4r::Object &model = camtracker->getModel();
        ba.optimize(model);

        for(size_t i=0; i < cameras_.size(); i++)
        {
            Eigen::Matrix4f inv_pose_after_ba;
            v4r::invPose(model.cameras[i], inv_pose_after_ba);
            cameras_[i] = inv_pose_after_ba;
            std::cout << cameras_[i] << std::endl << std::endl;
        }
    }

    for(size_t i=0; i < cameras_.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB> c;
        pcl::transformPointCloud( keyframes_[i], c, cameras_[i]);
        *compound += c;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered;
    createObjectCloudFiltered(filtered);

    int v1, v2;
    vis.createViewPort(0,0,0.5,1,v1);
    vis.createViewPort(0.5,0,1,1,v2);

    vis.addPointCloud(compound, "unfiltered", v1);
    vis.addPointCloud(filtered, "filtered", v2);

    vis.spin();

    return true;
}

void
CamTracker::initialize (int argc, char ** argv)
{
    n_.reset( new ros::NodeHandle ( "~" ) );

    confidence_publisher_ = n_->advertise<visualization_msgs::Marker>("confidence", 1);
    trajectory_publisher_ = n_->advertise<visualization_msgs::Marker>("trajectory", 1);
    keyframe_publisher_ = n_->advertise<visualization_msgs::Marker>("keyframes", 1);

    cam_tracker_start_  = n_->advertiseService ("start_recording", &CamTracker::start, this);
    cam_tracker_stop_  = n_->advertiseService ("stop_recording", &CamTracker::stop, this);
    cam_tracker_vis_compound_  = n_->advertiseService ("vis_compound", &CamTracker::visCompound, this);
    cam_tracker_do_ba_  = n_->advertiseService ("do_ba", &CamTracker::doBA, this);
    cam_tracker_get_tracking_results_  = n_->advertiseService ("get_results", &CamTracker::getTrackingResults, this);
    cam_tracker_save_tracking_results_to_file_  = n_->advertiseService ("save_results_to_file", &CamTracker::saveTrackingResultsToFile, this);
    cam_tracker_cleanup_  = n_->advertiseService ("cleanup", &CamTracker::cleanup, this);

    n_->getParam ( "camera_topic", camera_topic_ );
    n_->getParam ( "trajectory_threshold", trajectory_threshold_ );
    n_->getParam ( "use_cam_params_from_ROS_topic", use_cam_params_from_ROS_topic_ );

    double delta_angle_deg;
    if( n_->getParam ( "delta_angle_deg", delta_angle_deg ) )
        cos_min_delta_angle_ = cos( delta_angle_deg *M_PI/180.);

    debug_image_transport_.reset(new image_transport::ImageTransport(*n_));
    debug_image_publisher_ = debug_image_transport_->advertise("debug_images", 1);

    ros::spin ();
}

void
CamTracker::initializeVisualizationMarkers()
{
    trajectory_marker_.header.frame_id = "world";
    trajectory_marker_.ns = "trajectory";
    trajectory_marker_.id = 0;
    trajectory_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_marker_.scale.x = 0.005;
    trajectory_marker_.scale.y = 0.005;
    trajectory_marker_.scale.z = 0.005;
    trajectory_marker_.color.a = 1.0;
    trajectory_marker_.pose.orientation.w = 1.0;
    trajectory_marker_.points.clear();
    trajectory_marker_.colors.clear();
    trajectory_marker_.action = 3; // delete all
    trajectory_publisher_.publish(trajectory_marker_);
    trajectory_marker_.action = visualization_msgs::Marker::ADD;

    keyframes_marker_.header.frame_id = "world";
    keyframes_marker_.ns = "keyframes";
    keyframes_marker_.id = 0;
    keyframes_marker_.type = visualization_msgs::Marker::CUBE_LIST;
    keyframes_marker_.scale.x = 0.05;
    keyframes_marker_.scale.y = 0.05;
    keyframes_marker_.scale.z = 0.05;
    keyframes_marker_.color.a = 0.7;
    keyframes_marker_.color.r = 1.0;
    keyframes_marker_.color.g = 0.0;
    keyframes_marker_.pose.orientation.w = 1.0;
    keyframes_marker_.points.clear();
    keyframes_marker_.action = 3; // delete all
    keyframe_publisher_.publish(keyframes_marker_);
    keyframes_marker_.action = visualization_msgs::Marker::ADD;
}

int
main (int argc, char ** argv)
{
    ros::init (argc, argv, "camera_tracker");

    CamTracker m;
    m.initialize (argc, argv);

    return 0;
}
