/*
 * main.cpp
 *
 *  Created on: Feb 20, 2014
 *      Author: Thomas Fäulhammer
 */

#include <pcl/common/common.h>
#include <pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "do_learning_srv_definitions/learn_object.h"
#include "do_learning_srv_definitions/save_model.h"
#include "do_learning_srv_definitions/visualize.h"
#include <opencv2/opencv.hpp>
#include <v4r/common/io/filesystem_utils.h>

struct IndexPoint
{
  int idx;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (IndexPoint,
(int, idx, idx)
)

class DOLDemoFromFiles
{
private:
    typedef pcl::PointXYZRGB PointT;
    ros::NodeHandle *n_;
    std::string directory_,
                models_dir_,
                recognition_structure_dir_;
    bool visualize_;

public:
    bool callDOL()
    {

        //read files from directory
        std::string so_far = "";

        std::vector<std::string> keyframes_str;
        std::vector<std::string> object_indices_str;
        std::vector<std::string> poses_str;

        {
            std::string pattern = ".*cloud.*.pcd";
            v4r::common::io::getFilesInDirectory(directory_, keyframes_str, so_far, pattern, false);
        }

        {
            std::string pattern = ".*object_indices.*.pcd";
            v4r::common::io::getFilesInDirectory(directory_, object_indices_str, so_far, pattern, false);
        }

        {
            std::string pattern = ".*pose.*.txt";
            v4r::common::io::getFilesInDirectory(directory_, poses_str, so_far, pattern, false);
        }

        std::sort(keyframes_str.begin(), keyframes_str.end());
        std::sort(poses_str.begin(), poses_str.end());
        std::sort(object_indices_str.begin(), object_indices_str.end());

        std::string service_name_learn = "/dynamic_object_learning/learn_object";
        ros::ServiceClient DOLclient = n_->serviceClient<do_learning_srv_definitions::learn_object>(service_name_learn);
        do_learning_srv_definitions::learn_object srv_learn;


        std::string service_name_save = "/dynamic_object_learning/save_model";
        ros::ServiceClient DOLclient_save = n_->serviceClient<do_learning_srv_definitions::save_model>(service_name_save);
        do_learning_srv_definitions::save_model srv_save;

        std::string service_name_vis = "/dynamic_object_learning/visualize";
        ros::ServiceClient DOLclient_vis = n_->serviceClient<do_learning_srv_definitions::visualize>(service_name_vis);
        do_learning_srv_definitions::visualize srv_vis;


        //create request
#ifdef USE_PCL_INDICES
        std::stringstream str;
        str << directory_ << "/" << object_indices_str[0];

        pcl::PointCloud<IndexPoint> obj_indices_cloud;
        pcl::io::loadPCDFile (str.str(), obj_indices_cloud);

        for(size_t i=0; i < obj_indices_cloud.points.size(); i++)
        {
	   srv.request.intial_object_indices.push_back(obj_indices_cloud.points[i].idx);
        }
#else
        std::ifstream initial_mask_file;
        initial_mask_file.open(directory_ + "/mask.txt");

        int idx_tmp;
        pcl::PointIndices pind;
        while (initial_mask_file >> idx_tmp)
        {
            srv_learn.request.intial_object_indices.push_back(idx_tmp);
            pind.indices.push_back(idx_tmp);
        }
        initial_mask_file.close();

#endif

        for(size_t i=0; i < keyframes_str.size(); i++)
        {
            pcl::PointCloud<PointT>::Ptr pCloud(new pcl::PointCloud<PointT>());
            std::stringstream str;
            str << directory_ << "/" << keyframes_str[i];
            pcl::io::loadPCDFile(str.str(), *pCloud);

            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(*pCloud, msg_cloud);

            srv_learn.request.keyframes.push_back(msg_cloud);


            Eigen::Matrix4f trans;

//            {
//                std::stringstream str;
//                str << directory_ << "/" << poses_str[i];
//                v4r::utils::readMatrixFromFile (str.str(), trans);
//            }

            geometry_msgs::Transform tt;
            //            tt.translation.x = trans(0,3);
            //            tt.translation.y = trans(1,3);
            //            tt.translation.z = trans(2,3);
            tt.translation.x = pCloud->sensor_origin_[0];
            tt.translation.y = pCloud->sensor_origin_[1];
            tt.translation.z = pCloud->sensor_origin_[2];

            Eigen::Matrix3f rotation = trans.block<3,3>(0,0);
            Eigen::Quaternionf q(rotation);
//            tt.rotation.x = q.x();
//            tt.rotation.y = q.y();
//            tt.rotation.z = q.z();
//            tt.rotation.w = q.w();
            tt.rotation.x = pCloud->sensor_orientation_.x();
            tt.rotation.y = pCloud->sensor_orientation_.y();
            tt.rotation.z = pCloud->sensor_orientation_.z();
            tt.rotation.w = pCloud->sensor_orientation_.w();

            srv_learn.request.transforms.push_back(tt);
        }

        if ( ! DOLclient.call(srv_learn) )
        {
            std::stringstream mm;
            mm << "Error calling service: " << service_name_learn << std::endl;
            ROS_ERROR(mm.str().c_str());
            return false;
        }

        // Saving model
        srv_save.request.object_name.data = "my_dynamic_object.pcd";
        srv_save.request.models_folder.data = models_dir_;
        srv_save.request.recognition_structure_folder.data = recognition_structure_dir_;

        if ( ! DOLclient_save.call(srv_save) )
        {
            std::stringstream mm;
            mm << "Error calling service: " << service_name_save << std::endl;
            ROS_ERROR(mm.str().c_str());
            return false;
        }

        if (visualize_)
        {
            if ( ! DOLclient_vis.call ( srv_vis ) )
            {
                std::stringstream mm;
                mm << "Error calling service: " << service_name_vis << std::endl;
                ROS_ERROR(mm.str().c_str());
            }
        }

        return true;
    }

public:
    DOLDemoFromFiles()
    {
        visualize_ = true;
    }

    bool initialize(int argc, char ** argv)
    {
        ros::init (argc, argv, "DOLDemoFromFiles");
        if (sizeof(int) != 4)
        {
            ROS_WARN("PC Architectur does not use 32bit for integer - check conflicts with pcl indices.");
        }
        n_ = new ros::NodeHandle ( "~" );

        n_->getParam("visualize", visualize_);

        if(!n_->getParam ( "directory", directory_ ))
        {
            //directory_ = "/media/aitor14/DATA/STRANDS_MODELS/recognition_structure/playstation_turn_table.pcd/";
            ROS_ERROR("Specify a directory using param directory.\n");
            exit(-1);
        }

        if(!n_->getParam( "models_dir", models_dir_) )
        {
            ROS_ERROR("Specify a model directory using param models_dir.\n");
            exit(-1);
        }

        if(!n_->getParam( "recognition_dir", recognition_structure_dir_) )
        {
            ROS_ERROR("Specify a model directory using param recognition_dir.\n");
            exit(-1);
        }
    }
};

int
main (int argc, char ** argv)
{
    DOLDemoFromFiles m;
    m.initialize(argc, argv);
    m.callDOL();
    return 0;
}
