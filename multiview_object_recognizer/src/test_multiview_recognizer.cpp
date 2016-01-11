/*
 * main.cpp
 *
 *  Created on: June, 2014
 *      Author: Thomas Faeulhammer
 */

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "recognition_srv_definitions/recognize.h"
#include <v4r/io/filesystem.h>

class MultiViewRecognizerDemo
{
private:
    typedef pcl::PointXYZRGB PointT;
    ros::NodeHandle *n_;
    ros::ServiceClient sv_rec_client_;
    std::string directory_;
    std::string topic_;
    bool KINECT_OK_;
    int input_method_; // defines te test input (0... camera topic, 1... file)

public:
    MultiViewRecognizerDemo()
    {
        input_method_ = 0;
    }

    void callMvRecognizerUsingCam(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        std::cout << "Received point cloud.\n" << std::endl;
        recognition_srv_definitions::recognize srv;
        srv.request.cloud = *msg;

        if (sv_rec_client_.call(srv))
        {
            std::vector<std::string> model_ids;
            std::vector<Eigen::Matrix4f> transforms;

            for(size_t i=0; i < srv.response.ids.size(); i++)
            {
                model_ids.push_back(srv.response.ids[i].data);

                Eigen::Quaternionf q(srv.response.transforms[i].rotation.w,
                                     srv.response.transforms[i].rotation.x,
                                     srv.response.transforms[i].rotation.y,
                                     srv.response.transforms[i].rotation.z);

                Eigen::Vector3f translation(srv.response.transforms[i].translation.x,
                                            srv.response.transforms[i].translation.y,
                                            srv.response.transforms[i].translation.z);


                Eigen::Matrix4f trans;
                trans.block<3,3>(0,0) = q.toRotationMatrix();
                trans.block<3,1>(0,3) = translation;
                transforms.push_back(trans);
            }

            std::cout << "Called done..." << std::endl;
        }
        else
        {
            ROS_ERROR("Failed to call /recognition_service/sv_recognition");
        }
    }

    void checkCloudArrive (const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        KINECT_OK_ = true;
    }

    bool checkKinect ()
    {
        ros::Subscriber sub_pc = n_->subscribe (topic_, 1, &MultiViewRecognizerDemo::checkCloudArrive, this);
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

    bool callMvRecognizerUsingFiles()
    {
        std::vector<std::string> test_cloud = v4r::io::getFilesInDirectory(directory_, ".*.pcd", false);
        for(size_t i=0; i < test_cloud.size(); i++)
        {
            pcl::PointCloud<PointT> cloud;
            pcl::io::loadPCDFile(directory_ + "/" + test_cloud[i], cloud);
            sensor_msgs::PointCloud2 cloud_ros;
            pcl::toROSMsg(cloud, cloud_ros);
            cloud_ros.header.stamp = ros::Time::now();
            recognition_srv_definitions::recognize srv_rec;
            srv_rec.request.cloud = cloud_ros;
            std::stringstream view_name_ss;
            view_name_ss << "view_" << i << std::endl;
            std::cout << "####### " << view_name_ss.str() << std::endl;
            srv_rec.request.view_name.data = view_name_ss.str();

            std::string pose_file = test_cloud[i];
            boost::replace_all(pose_file, ".pcd", ".txt");

#ifdef USE_WILLOW_DATASET_FOR_EVAL
            boost::replace_all(pose_file, "cloud_", "pose_");
#else
            pose_file = "transformation_" + pose_file;
#endif
            std::cout << "Checking if path " << directory_ << "/" << pose_file << " for transform exists. " << std::endl;

            if ( boost::filesystem::exists( directory_ + "/" + pose_file ) )
            {
                std::cout << "File exists." << std::endl;
                std::ifstream is( (directory_ + "/" + pose_file).c_str() );

#ifdef USE_WILLOW_DATASET_FOR_EVAL
                std::string s;
                std::vector<std::string> file_parts;
                std::getline( is, s );
                std::istringstream ss( s );
                std::vector<double> numbers;
                while (ss)
                {
                  std::string s;
                  if (!std::getline( ss, s, ' ' )) break;
                  file_parts.push_back( s );
                  if(file_parts.size()>1)
                      numbers.push_back(atof(s.c_str()));
                }
#else
                std::istream_iterator<double> start(is), end;
                std::vector<float> numbers(start, end);
                std::cout << "Read " << numbers.size() << " numbers" << std::endl;
#endif
                // print the numbers to stdout
                std::cout << "Transform to world coordinate system: " << std::endl;
                for(size_t i=0; i<numbers.size(); i++)
                {
                    std::cout << numbers[i] << " ";
                    srv_rec.request.transform.push_back(numbers[i]);
                }
                std::cout << std::endl;
            }

            if (!sv_rec_client_.call(srv_rec))
            {
                ROS_ERROR("Error calling multiview recognition service. ");
                return false;
            }
        }
        return true;
    }

    bool initialize(int argc, char ** argv)
    {
        ros::init (argc, argv, "MultiViewRecognizerDemoFromFiles");
        n_ = new ros::NodeHandle ( "~" );

        std::string service_name_sv_rec = "/multiview_recognition_service/multiview_recognition_service";
        sv_rec_client_ = n_->serviceClient<recognition_srv_definitions::recognize>(service_name_sv_rec);

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
                ros::Subscriber sub_pc = n_->subscribe (topic_, 1, &MultiViewRecognizerDemo::callMvRecognizerUsingCam, this);
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
                callMvRecognizerUsingFiles();
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
    MultiViewRecognizerDemo m;
    m.initialize(argc, argv);
    return 0;
}
