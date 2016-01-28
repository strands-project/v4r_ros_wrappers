/*
 * main.cpp
 *
 *  Created on: Feb, 2015
 *      Author: Thomas Faeulhammer
 */

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <v4r/common/pcl_opencv.h>
#include <v4r/io/filesystem.h>

#include "recognition_srv_definitions/recognize.h"
#include "object_tracker_srv_definitions/change_tracking_model.h"
#include "object_tracker_srv_definitions/cleanup.h"
#include "object_tracker_srv_definitions/start_tracker.h"
#include "object_tracker_srv_definitions/stop_tracker.h"
#include "object_tracker_srv_definitions/detect_and_track.h"

class DetectAndTrackDemo
{
private:
    typedef pcl::PointXYZRGB PointT;
    ros::NodeHandle *n_;
    ros::ServiceClient sv_rec_client_, obj_track_start_, obj_track_stop_, obj_track_cleanup_, obj_track_change_model_;
    ros::ServiceServer detect_and_track_;
    ros::Subscriber sub_pc_;
    std::string models_dir_;
    std::string topic_;
    bool KINECT_OK_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher image_pub_;

public:
    DetectAndTrackDemo()
    {}

    void callUsingCam(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        std::cout << "Received point cloud.\n" << std::endl;
        recognition_srv_definitions::recognize srv;
        srv.request.cloud = *msg;

        if (!sv_rec_client_.call(srv)) { ROS_ERROR("Failed to call /recognition_service/sv_recognition"); return; }
        else {
            std::vector<std_msgs::String>  detected_ids = srv.response.ids;

            if(detected_ids.empty())
                std::cout << "I did not detect any object from the model database in the current scene." << std::endl;
            else {
                sub_pc_.shutdown();
                std::cout << "I detected: " << std::endl;
                for(size_t i=0; i<detected_ids.size(); i++)
                    std::cout << "  " << detected_ids[i].data << std::endl;

                // Take first model to track
                std::string new_tracking_model = detected_ids[0].data + "/tracking_model.ao";
                object_tracker_srv_definitions::start_tracker srv_obj_track_start;
                object_tracker_srv_definitions::change_tracking_model srv_obj_track_ch_m;
                srv_obj_track_ch_m.request.filename = models_dir_ + "/" + new_tracking_model;

                if ( !obj_track_change_model_.call(srv_obj_track_ch_m) ) { std::cerr<<"Failed to change object tracker model!" << std::endl; return; }
                else
                {
                    std::cout << "Changed tracking model to " << new_tracking_model << "." << std::endl;
                    if (obj_track_start_.call(srv_obj_track_start))
                        std::cout << "started object tracker" << std::endl;
                    else  {
                        std::cerr<<"Failed to start object tracker with new model!" << std::endl;
                        return;
                    }
                }
            }
            std::cout << std::endl;
        }
    }

    void checkCloudArrive (const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        KINECT_OK_ = true;
    }

    bool checkKinect ()
    {
        ros::Subscriber sub_check_kinect = n_->subscribe (topic_, 1, &DetectAndTrackDemo::checkCloudArrive, this);
        ros::Rate loop_rate (1);
        size_t kinect_trials_ = 0;

        while (!KINECT_OK_ && ros::ok () && kinect_trials_ < 30)
        {
            std::cout << "Checking kinect status..." << std::endl;
            ros::spinOnce ();
            loop_rate.sleep ();
            kinect_trials_++;
        }

        return KINECT_OK_;
    }

    bool
    detectAndTrack( object_tracker_srv_definitions::detect_and_track::Request & req,
                    object_tracker_srv_definitions::detect_and_track::Response & response)
    {
        object_tracker_srv_definitions::stop_tracker srv_obj_track_stop;
        object_tracker_srv_definitions::cleanup srv_obj_track_cleanup;
        if ( !obj_track_stop_.call(srv_obj_track_stop) ) { std::cerr<<"Failed to stop object tracker!" << std::endl; }
        if ( !obj_track_stop_.call(srv_obj_track_cleanup) ) { std::cerr<<"Failed to cleanup object tracker!" << std::endl; }

        sub_pc_ = n_->subscribe (topic_, 1, &DetectAndTrackDemo::callUsingCam, this);
        return true;
    }


    bool initialize(int argc, char ** argv)
    {
        ros::init (argc, argv, "detect_and_track_object");
        n_ = new ros::NodeHandle ( "~" );

        std::string service_name_sv_rec = "/recognition_service/";
        sv_rec_client_ = n_->serviceClient<recognition_srv_definitions::recognize>(service_name_sv_rec + "sv_recognition");

        std::string service_name_obj_tracker = "/object_tracker/";
        obj_track_start_ = n_->serviceClient<object_tracker_srv_definitions::start_tracker>(service_name_obj_tracker + "start_recording");
        obj_track_stop_ = n_->serviceClient<object_tracker_srv_definitions::stop_tracker>(service_name_obj_tracker + "stop_recording");
        obj_track_change_model_ = n_->serviceClient<object_tracker_srv_definitions::change_tracking_model>(service_name_obj_tracker + "change_tracking_model");
        obj_track_cleanup_ = n_->serviceClient<object_tracker_srv_definitions::cleanup>(service_name_obj_tracker + "cleanup");
        detect_and_track_  = n_->advertiseService ("detect_and_track", &DetectAndTrackDemo::detectAndTrack, this);

        it_.reset(new image_transport::ImageTransport(*n_));
//        image_pub_ = it_->advertise("/mp_recognition/debug_image", 1, true);

        if(!n_->getParam ( "models_dir", models_dir_ )){
            std::cerr << "No model directory set. Please set it using parameter models_dir." << std::endl;
            return false;
        }

        if(!n_->getParam ( "topic", topic_ ))
            topic_ = "/camera/depth_registered/points";


        std::cout << "Trying to connect to camera on topic " << topic_ <<
                     ". You can change the topic with param topic " << std::endl;

        if ( checkKinect() )
        {
            std::cout << "Camera (topic: " << topic_ << ") is up and running." << std::endl;
            ros::spin();
        }
        else
        {
            std::cerr << "Camera (topic: " << topic_ << ") is not working." << std::endl;
            return false;
        }
        return true;
    }
};

int
main (int argc, char ** argv)
{
    DetectAndTrackDemo d;
    d.initialize(argc, argv);
    return 0;
}
