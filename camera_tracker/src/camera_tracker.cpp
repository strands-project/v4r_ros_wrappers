/*
 * shape_simple_classifier_node.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: aitor
 */

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <thread>

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>

#include "camera_srv_definitions/start_tracker.h"
#include "camera_srv_definitions/stop_tracker.h"
#include "camera_srv_definitions/visualize_compound.h"
#include "camera_srv_definitions/get_tracking_results.h"
#include "camera_srv_definitions/save_tracking_results_to_file.h"
#include "camera_srv_definitions/do_ba.h"
#include "camera_srv_definitions/cleanup.h"

#include <v4r/common/impl/ScopeTime.hpp>
#include <v4r/common/noise_model_based_cloud_integration.h>
#include <v4r/common/noise_models.h>
#include <v4r/keypoints/impl/convertCloud.hpp>
#include <v4r/keypoints/impl/convertImage.hpp>
#include <v4r/keypoints/impl/invPose.hpp>
#include <v4r/keypoints/impl/PointTypes.hpp>
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

    void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg) {
        camera_info_ = *msg;
        got_camera_info_=true;
    }

    void drawConfidenceBar(cv::Mat &im, const double &conf)
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

    int selectFrames(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
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

    void trackNewCloud(const sensor_msgs::PointCloud2Ptr& msg)
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

    void getCloud(const sensor_msgs::PointCloud2Ptr& msg)
    {
        trackNewCloud(msg);
    }


    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
        sensor_msgs::PointCloud2Ptr pMsg (new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cloud, *pMsg);
        trackNewCloud(pMsg);
    }

    bool
    cleanup (camera_srv_definitions::cleanup::Request & req,
             camera_srv_definitions::cleanup::Response & response)
    {
        cameras_.clear();
        keyframes_.clear();
        saved_clouds_ = 0;
        conf_=0;
        pose_ = Eigen::Matrix4f::Identity();

        return true;
    }

    bool
    start (camera_srv_definitions::start_tracker::Request & req,
           camera_srv_definitions::start_tracker::Response & response)
    {
        cameras_.clear();
        keyframes_.clear();
        saved_clouds_ = 0;
        conf_=0;
        pose_ = Eigen::Matrix4f::Identity();

        initializeVisualizationMarkers();

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

        cv::Mat_<double> distCoeffs;
        cv::Mat_<double> intrinsic = cv::Mat::zeros(3, 3, CV_64F);
        intrinsic.at<double>(0,0) = 525.f;
        intrinsic.at<double>(1,1) = 525.f;
        intrinsic.at<double>(0,2) = 320.f;
        intrinsic.at<double>(1,2) = 240.f;
        intrinsic.at<double>(2,2) = 1.f;
        std::cout << intrinsic << std::endl << std::endl;

        camtracker.reset( new v4r::KeypointSlamRGBD2(param) );
        camtracker->setCameraParameter(intrinsic,distCoeffs);

        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
          boost::bind (&CamTracker::cloud_cb_, this, _1);
        interface->registerCallback (f);
        interface->start ();

        std::cout << "Camera started..." << std::endl;
#else
        camera_info_subscriber_ = n_->subscribe(camera_topic_ +"/camera_info", 1, &CamTracker::camera_info_cb, this);

        ROS_INFO_STREAM("Wating for camera info...topic=" << camera_topic_ << "/camera_info...");
        while (!got_camera_info_) {
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        ROS_INFO("got it.");
        camera_info_subscriber_.shutdown();

        cv::Mat_<double> distCoeffs = cv::Mat(4, 1, CV_64F, camera_info_.D.data());
        cv::Mat_<double> intrinsic = cv::Mat(3, 3, CV_64F, camera_info_.K.data());

        camtracker.reset( new v4r::KeypointSlamRGBD2(param) );
        camtracker->setCameraParameter(intrinsic,distCoeffs);

        confidence_publisher_ = n_->advertise<std_msgs::Float32>("cam_tracker_confidence", 1);
        camera_topic_subscriber_ = n_->subscribe(camera_topic_ +"/points", 1, &CamTracker::getCloud, this);
#endif
        last_cloud_ = boost::posix_time::microsec_clock::local_time ();
        last_cloud_ros_time_ = ros::Time::now();
        return true;
    }

    bool
    stop (camera_srv_definitions::start_tracker::Request & req,
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


    void createObjectCloudFiltered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & octree_cloud)
    {
        double max_angle = 70.f;
        double lateral_sigma = 0.0015f;
        bool depth_edges = true;
        float nm_integration_min_weight_ = 0.75f;

        v4r::noise_models::NguyenNoiseModel<pcl::PointXYZRGB> nm;
        std::vector< std::vector<float> > weights(keyframes_.size());
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > ptr_clouds(keyframes_.size());
        std::vector< pcl::PointCloud<pcl::Normal>::Ptr > normals(keyframes_.size());

        nm.setLateralSigma(lateral_sigma);
        nm.setMaxAngle(max_angle);
        nm.setUseDepthEdges(depth_edges);

        if (keyframes_.size()>0)
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
                nm.getWeights(weights[i]);
            }

            v4r::NMBasedCloudIntegration<pcl::PointXYZRGB> nmIntegration;
            octree_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

            nmIntegration.setInputClouds(ptr_clouds);
            nmIntegration.setWeights(weights);
            nmIntegration.setTransformations(cameras_);
            nmIntegration.setMinWeight(nm_integration_min_weight_);
            nmIntegration.setInputNormals(normals);
            nmIntegration.setMinPointsPerVoxel(1);
            nmIntegration.setResolution(0.005f);
            nmIntegration.setFinalResolution(0.005f);
            nmIntegration.compute(octree_cloud);
        }
    }

    bool
    doBA (camera_srv_definitions::do_ba::Request & req,
          camera_srv_definitions::do_ba::Response & response)
    {
        if(cameras_.size() == 0)
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
    getTrackingResults (camera_srv_definitions::get_tracking_results::Request & req,
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
    saveTrackingResultsToFile(camera_srv_definitions::save_tracking_results_to_file::Request &req,
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
            if(!is_directory(s))
            {
                path p(req.dir_name.data);
                create_directories(req.dir_name.data);
                for(size_t i=0; i < cameras_.size(); i++)
                {
                    path f = p / boost::str(boost::format("cloud_%i.pcd") % i);
                    std::string filename = f.string();
                    std::cout << "Writing file to " << filename << "." << std::endl;
                    pcl::io::savePCDFileBinary(filename, keyframes_[i]);
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
    visCompound (camera_srv_definitions::visualize_compound::Request & req,
                 camera_srv_definitions::visualize_compound::Response & response)
    {
        if(cameras_.size() == 0)
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

public:
    CamTracker () : got_camera_info_(false)
    {
        conf_=0;
        pose_ = Eigen::Matrix4f::Identity();
        cos_min_delta_angle_ = cos(15*M_PI/180.);
        sqr_min_cam_distance_ = 1.*1.;

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
    initialize (int argc, char ** argv)
    {
        double delta_angle_deg;
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

        if(!n_->getParam ( "camera_topic", camera_topic_ ))
            camera_topic_ = "/camera/depth_registered";

        if( n_->getParam ( "delta_angle_deg", delta_angle_deg ) )
            cos_min_delta_angle_ = cos( delta_angle_deg *M_PI/180.);

        if(!n_->getParam ( "trajectory_threshold_", trajectory_threshold_ ) )
            trajectory_threshold_ = 0.02;

        debug_image_transport_.reset(new image_transport::ImageTransport(*n_));
        debug_image_publisher_ = debug_image_transport_->advertise("debug_images", 1);

        ros::spin ();
    }

    void
    initializeVisualizationMarkers()
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

};

int
main (int argc, char ** argv)
{
    ros::init (argc, argv, "camera_tracker");

    CamTracker m;
    m.initialize (argc, argv);

    return 0;
}
