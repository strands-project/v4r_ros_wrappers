/*
 * main.cpp
 *
 *  Created on: Aug 20, 2014
 *      Author: Thomas Faeulhammer
 */

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <v4r/io/filesystem.h>
#include "segmentation_srv_definitions/segment.h"
#include "classifier_srv_definitions/classify.h"

class SegmenationAndClassifyDemo
{
private:
    typedef pcl::PointXYZRGB PointT;
    ros::NodeHandle *n_;
    ros::ServiceClient srv_client_seg;
    ros::ServiceClient srv_client_classify;
    std::string directory_;
    std::string topic_;
    bool KINECT_OK_;
    int input_method_; // defines the test input (0... camera topic, 1... file)

public:
    SegmenationAndClassifyDemo()
    {
        input_method_ = 0;
    }

    void
    callUsingCam(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        std::cout << "Received point cloud.\n" << std::endl;
        segmentation_srv_definitions::segment srv_seg;
        srv_seg.request.cloud = *msg;

        if (!srv_client_seg.call(srv_seg))
            std::cerr << "Error calling segmentation service!" << std::endl;
        else
        {
            classifier_srv_definitions::classify srv_classify;
            srv_classify.request.cloud = srv_seg.request.cloud;
            srv_classify.request.clusters_indices = srv_seg.response.clusters_indices;

            if (!srv_client_classify.call(srv_classify))
                std::cerr << "Error calling classification service!" << std::endl;
        }
    }

    void checkCloudArrive (const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        KINECT_OK_ = true;
    }

    bool checkKinect ()
    {
        ros::Subscriber sub_pc = n_->subscribe (topic_, 1, &SegmenationAndClassifyDemo::checkCloudArrive, this);
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

    bool callUsingFiles()
    {
        std::vector<std::string> test_cloud = v4r::io::getFilesInDirectory(directory_, ".*.pcd", false);
        for(size_t i=0; i < test_cloud.size(); i++)
        {
            pcl::PointCloud<PointT> cloud;
            pcl::io::loadPCDFile(directory_ + "/" + test_cloud[i], cloud);
            sensor_msgs::PointCloud2 cloud_ros;
            pcl::toROSMsg(cloud, cloud_ros);
            segmentation_srv_definitions::segment srv_seg;
            srv_seg.request.cloud = cloud_ros;

            if (!srv_client_seg.call(srv_seg))
            {
                std::cerr << "Error calling segmentation service!" << std::endl;
                return false;
            }
            else
            {
                classifier_srv_definitions::classify srv_classify;
                srv_classify.request.cloud = srv_seg.request.cloud;
                srv_classify.request.clusters_indices = srv_seg.response.clusters_indices;

                if (!srv_client_classify.call(srv_classify))
                {
                    std::cerr << "Error calling classification service!" << std::endl;
                    return false;
                }
            }
        }
        return true;
    }

    bool initialize(int argc, char ** argv)
    {
        ros::init (argc, argv, "SegmenationAndClassificationDemo");
        n_ = new ros::NodeHandle ( "~" );

        std::string service_name_seg = "/pcl_segmentation_service/pcl_segmentation";
        std::string service_name_classify = "/classifier_service/classify";
        srv_client_seg = n_->serviceClient<segmentation_srv_definitions::segment>(service_name_seg);
        srv_client_classify = n_->serviceClient<classifier_srv_definitions::classify>(service_name_classify);

        n_->getParam ( "input_method", input_method_ );

        if ( input_method_ == 0 )
        {
            if(!n_->getParam ( "topic", topic_ ))
            {
                topic_ = "/camera/depth_registered/points";
            }
            std::cout << "Trying to connect to camera on topic " <<
                         topic_ << ". You can change the topic with param topic or " <<
                         " test pcd files from a directory by using input_method=1 and specifying param directory. " << std::endl;

            if ( checkKinect() )
            {
                std::cout << "Camera (topic: " << topic_ << ") is up and running." << std::endl;
                ros::Subscriber sub_pc = n_->subscribe (topic_, 1, &SegmenationAndClassifyDemo::callUsingCam, this);
                ros::spin();
            }
            else
            {
                std::cerr << "Camera (topic: " << topic_ << ") is not working." << std::endl;
                return false;
            }
        }
        else //input_method==1
        {
            if(n_->getParam ( "directory", directory_ ) && directory_.length())
            {
                callUsingFiles();
            }
            else
            {
                std::cout << "No test directory (param directory) specified. " << std::endl;
                return false;
            }
        }
        return true;
    }
};

int
main (int argc, char ** argv)
{
    SegmenationAndClassifyDemo m;
    m.initialize(argc, argv);
    return 0;
}
