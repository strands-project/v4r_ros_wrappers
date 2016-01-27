/*
 * main.cpp
 *
 *  Created on: July, 2015
 *      Author: Thomas Fäulhammer
 */

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "incremental_object_learning_srv_definitions/clear.h"
#include "incremental_object_learning_srv_definitions/learn_object.h"
#include "incremental_object_learning_srv_definitions/save_model.h"
#include "incremental_object_learning_srv_definitions/visualize.h"
#include <opencv2/opencv.hpp>
#include <v4r/io/filesystem.h>

class IOLDemoFromFiles
{
private:
    typedef pcl::PointXYZRGB PointT;
    ros::NodeHandle *n_;
    std::string directory_,
                models_dir_,
                object_name_,
                mask_file_;
    bool visualize_;

public:
    bool callIOL()
    {
        std::vector<std::string> keyframes_str = v4r::io::getFilesInDirectory(directory_, ".*.pcd", false);
        std::vector<std::string> object_indices_str = v4r::io::getFilesInDirectory(directory_, ".*object_indices.*.pcd", false);;
        std::vector<std::string> poses_str = v4r::io::getFilesInDirectory(directory_, ".*pose.*.txt", false);;

        std::string service_name_clear = "/incremental_object_learning/clear_cached_model";
        ros::ServiceClient IOL_clear_client = n_->serviceClient<incremental_object_learning_srv_definitions::clear>(service_name_clear);
        incremental_object_learning_srv_definitions::clear srv_clear;

        if ( ! IOL_clear_client.call ( srv_clear ) )
        {
            std::stringstream mm;
            mm << "Error calling service: " << service_name_clear << std::endl;
            std::cerr << mm.str() << std::endl;
        }

        std::string service_name_learn = "/incremental_object_learning/learn_object";
        ros::ServiceClient IOLclient = n_->serviceClient<incremental_object_learning_srv_definitions::learn_object>(service_name_learn);
        incremental_object_learning_srv_definitions::learn_object srv_learn;

        std::string service_name_save = "/incremental_object_learning/save_model";
        ros::ServiceClient IOLclient_save = n_->serviceClient<incremental_object_learning_srv_definitions::save_model>(service_name_save);
        incremental_object_learning_srv_definitions::save_model srv_save;

        std::string service_name_vis = "/incremental_object_learning/visualize";
        ros::ServiceClient IOLclient_vis = n_->serviceClient<incremental_object_learning_srv_definitions::visualize>(service_name_vis);
        incremental_object_learning_srv_definitions::visualize srv_vis;

        if ( !mask_file_.compare("") )
            mask_file_ = directory_ + "/mask.txt";

        std::ifstream initial_mask_file;
        initial_mask_file.open( mask_file_ );

        int idx_tmp;
        pcl::PointIndices pind;
        while (initial_mask_file >> idx_tmp)
        {
            srv_learn.request.intial_object_indices.push_back(idx_tmp);
            pind.indices.push_back(idx_tmp);
        }
        initial_mask_file.close();

        for(size_t i=0; i < keyframes_str.size(); i++)
        {
            pcl::PointCloud<PointT>::Ptr pCloud(new pcl::PointCloud<PointT>());
            std::stringstream str;
            str << directory_ << "/" << keyframes_str[i];
            std::cout << str.str() << std::endl;
            pcl::io::loadPCDFile(str.str(), *pCloud);

            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(*pCloud, msg_cloud);

            srv_learn.request.keyframes.push_back(msg_cloud);

            Eigen::Matrix4f trans;
            geometry_msgs::Transform tt;
            tt.translation.x = pCloud->sensor_origin_[0];
            tt.translation.y = pCloud->sensor_origin_[1];
            tt.translation.z = pCloud->sensor_origin_[2];

            Eigen::Matrix3f rotation = trans.block<3,3>(0,0);
            Eigen::Quaternionf q(rotation);
            tt.rotation.x = pCloud->sensor_orientation_.x();
            tt.rotation.y = pCloud->sensor_orientation_.y();
            tt.rotation.z = pCloud->sensor_orientation_.z();
            tt.rotation.w = pCloud->sensor_orientation_.w();
            srv_learn.request.transforms.push_back(tt);
        }

        if ( ! IOLclient.call(srv_learn) )
        {
            std::stringstream mm;
            mm << "Error calling service: " << service_name_learn << std::endl;
            std::cerr << mm.str() << std::endl;
            return false;
        }

        // Saving model
        srv_save.request.object_name.data = object_name_;
        srv_save.request.models_folder.data = models_dir_;

        if ( ! IOLclient_save.call(srv_save) )
        {
            std::stringstream mm;
            mm << "Error calling service: " << service_name_save << std::endl;
            std::cerr << mm << std::endl;
            return false;
        }

        if (visualize_)
        {
            if ( ! IOLclient_vis.call ( srv_vis ) )
            {
                std::stringstream mm;
                mm << "Error calling service: " << service_name_vis << std::endl;
                std::cerr << mm.str() << std::endl;
            }
        }
        return true;
    }

public:
    IOLDemoFromFiles()
    {
        visualize_ = true;
        models_dir_ = "/tmp/IOL/models/",
        object_name_ = "my_incremental_object";
    }

    bool initialize(int argc, char ** argv)
    {
        ros::init (argc, argv, "IOLDemoFromFiles");
        if (sizeof(int) != 4)
        {
            ROS_WARN("PC Architectur does not use 32bit for integer - check conflicts with pcl indices.");
        }
        n_ = new ros::NodeHandle ( "~" );

        if(!n_->getParam ( "directory", directory_ ))
        {
            ROS_ERROR("Specify a directory using param directory.\n");
            return false;
        }

        n_->getParam( "models_dir", models_dir_);
        n_->getParam( "object_name", object_name_);
        n_->getParam( "mask_file", mask_file_);
        n_->getParam("visualize", visualize_);

        return true;
    }
};

int
main (int argc, char ** argv)
{
    IOLDemoFromFiles m;
    if (m.initialize(argc, argv))
        m.callIOL();
    else
        return -1;
    return 0;
}
