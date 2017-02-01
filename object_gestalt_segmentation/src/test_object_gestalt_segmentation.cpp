/*
 * main.cpp
 *
 *  Created on: Nov 09, 2016
 *      Author: Markus Suchi
 */

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <v4r/io/filesystem.h>
#include "segmentation_srv_definitions/segment.h"
#include <cv_bridge/cv_bridge.h>
#include <v4r/attention_segmentation/EPUtils.h>

class SegmenationDemo
{
private:
    typedef pcl::PointXYZRGB PointT;
    ros::NodeHandle *n_;
    ros::ServiceClient srv_client;
    std::string directory_;
    std::string topic_;
    bool KINECT_OK_;
    int input_method_; // defines the test input (0... camera topic, 1... file)
    bool visualize_;  // visualize respond (true, false)
    cv::Mat RGB;
    ros::Publisher SegmentationPub_;

public:
    SegmenationDemo()
    {
        input_method_ = 0;        
    }

    void callUsingCam(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        std::cout << "Received point cloud.\n" << std::endl;
        segmentation_srv_definitions::segment srv;
        srv.request.cloud = *msg;

        if (srv_client.call(srv))
            std::cout << "Call done..." << std::endl;
            if(visualize_)
            {
               visualizePointcloud(srv);
            }
        else
            ROS_ERROR("Failed to call service");
    }

    void checkCloudArrive (const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        KINECT_OK_ = true;
    }

    bool checkKinect ()
    {
        ros::Subscriber sub_pc = n_->subscribe (topic_, 1, &SegmenationDemo::checkCloudArrive, this);
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

    void visualizePointcloud(const segmentation_srv_definitions::segment& srv)
    {
          std::cout << "Going to show " << srv.response.clusters_indices.size() << " object(s)" << std::endl;



          //get point cloud
          pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
          pcl::fromROSMsg (srv.request.cloud, *scene);
          ROS_INFO ("Number of points in the scene: %ld", scene->points.size());

          // create image ot publish
          pcl::PointCloud<pcl::PointXYZ>::Ptr scene_xyz (new pcl::PointCloud<pcl::PointXYZ>);
          v4r::pointCloudXYZRGB_2_cloudXYZimageRGB(scene,scene_xyz,RGB,scene->width,scene->height);
          srand (time(NULL));
          for(size_t i=0; i < srv.response.clusters_indices.size(); i++)
          {
              uchar r = std::rand()%255;
              uchar g = std::rand()%255;
              uchar b = std::rand()%255;
              for(size_t k=0; k < srv.response.clusters_indices[i].data.size(); k++)
              {
               int idx = srv.response.clusters_indices[i].data[k];

               cv::Vec3b &cvp = RGB.at<cv::Vec3b> (idx/RGB.cols,idx%RGB.cols);
               cvp[0] = r;
               cvp[1] = g;
               cvp[2] = b;
              }
          }

          //publish color image
          cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
          ros::Time time = ros::Time::now();
          // convert OpenCV image to ROS message
          cv_ptr->header.stamp = time;
          cv_ptr->header.frame_id = "image";
          cv_ptr->encoding = "bgr8";
          cv_ptr->image = RGB;

          sensor_msgs::Image im;
          cv_ptr->toImageMsg(im);

          SegmentationPub_.publish(im);
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
            segmentation_srv_definitions::segment srv;
            srv.request.cloud = cloud_ros;
            if (!srv_client.call(srv))
            {
                std::stringstream mm;
                mm << "Error calling service. "<< std::endl;
                std::cerr << mm.str() << std::endl;
                return false;
            }

            if(visualize_)
            {
               visualizePointcloud(srv);
            }
        }
        std::cout << "Call completed." << std::endl;

        return true;
    }

    bool initialize(int argc, char ** argv)
    {
        ros::init (argc, argv, "SegmentationDemo");
        n_ = new ros::NodeHandle ( "~" );

        SegmentationPub_ = n_->advertise<sensor_msgs::Image>("/object_gestalt_segmentation_visualization", 1000, true);

        std::string service_name_sv_rec = "/object_gestalt_segmentation";
        srv_client = n_->serviceClient<segmentation_srv_definitions::segment>(service_name_sv_rec);

        n_->getParam ( "input_method", input_method_ );

        if( !n_->getParam( "visualize", visualize_) )
        {
           visualize_ = false;
        }
        else
        {
           visualize_ = true;
        }

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
                ros::Subscriber sub_pc = n_->subscribe (topic_, 1, &SegmenationDemo::callUsingCam, this);
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
            std::cout << "Trying to read pointcloud from file... " <<std::endl;
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
    SegmenationDemo m;
    m.initialize(argc, argv);
    return 0;
}
