/*
 * main.cpp
 *
 *  Created on: Feb 20, 2014
 *      Author: Thomas Fäulhammer
 */

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "recognition_srv_definitions/recognize.h"
#include "recognition_srv_definitions/retrain_recognizer.h"
#include <v4r/recognition/model_only_source.h>
#include <opencv2/opencv.hpp>
#include <v4r/common/pcl_opencv.h>

class SOCDemo
{
private:
    typedef pcl::PointXYZ PointT;
    int kinect_trials_;
    int service_calls_;
    std::string topic_;
    bool KINECT_OK_;
    bool all_required_services_okay_;
    ros::NodeHandle *n_;
    bool visualize_output_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
    std::string models_dir_;
    boost::shared_ptr < v4r::rec_3d_framework::ModelOnlySource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>
            > models_source_;
    std::string file_name_;
    int input_; // defines the test input (0... camera topic, 1... file)

    std::vector<std::string> model_ids_;
    std::vector<Eigen::Matrix4f> transforms_;
    std::vector<float> confidences_;
    std::vector<std::pair<cv::Point, cv::Point> > box_rectangles_;

    bool new_models_added_;
    bool extended_request_;
    std::vector<std::string> model_ids_to_be_loaded_;

    void
    checkCloudArrive (const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        KINECT_OK_ = true;
    }

    void
    checkKinect ()
    {
        ros::Subscriber sub_pc = n_->subscribe (topic_, 1, &SOCDemo::checkCloudArrive, this);
        ros::Rate loop_rate (1);
        kinect_trials_ = 0;
        while (!KINECT_OK_ && ros::ok ())
        {
            std::cout << "Checking kinect status..." << std::endl;
            ros::spinOnce ();
            loop_rate.sleep ();
            kinect_trials_++;
            if(kinect_trials_ >= 30)
            {
                std::cout << "Kinect is not working..." << std::endl;
                return;
            }
        }

        KINECT_OK_ = true;
        std::cout << "Kinect is up and running, new_models:" << new_models_added_ << std::endl;

        if(new_models_added_)
        {
            std::cout << "NEW MODELS HAVE BEEN ADDED... call retrain..." << std::endl;

            ros::ServiceClient retrainClient = n_->serviceClient<recognition_srv_definitions::retrain_recognizer>("/recognition_service/mp_recognition_retrain");
            recognition_srv_definitions::retrain_recognizer srv;
            if(retrainClient.call(srv))
            {
                std::cout << "called retrain succesfull" << std::endl;
            }
            else
            {
                ROS_ERROR("Failed to call /recognition_service/mp_recognition_retrain");
                exit(-1);
            }
        }

        /*if(model_ids_to_be_loaded_.size() > 0)
        {*/
        ros::ServiceClient retrainClient = n_->serviceClient<recognition_srv_definitions::retrain_recognizer>("/recognition_service/mp_recognition_retrain");
        recognition_srv_definitions::retrain_recognizer srv;
        for(size_t k=0; k < model_ids_to_be_loaded_.size(); k++)
        {
            std_msgs::String a;
            a.data = model_ids_to_be_loaded_[k];
            srv.request.load_ids.push_back(a);
        }

        if(retrainClient.call(srv))
        {
            std::cout << "called retrain succesfull" << std::endl;
        }
        else
        {
            ROS_ERROR("Failed to call /recognition_service/mp_recognition_retrain");
            exit(-1);
        }
        //}
    }

    bool callSegAndClassifierService(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        ros::ServiceClient segAndClassifierClient = n_->serviceClient<recognition_srv_definitions::recognize>("/recognition_service/mp_recognition");
        recognition_srv_definitions::recognize srv;
        srv.request.cloud = *msg;
        srv.request.complex_result.data = extended_request_;

        if (segAndClassifierClient.call(srv))
        {
            model_ids_.clear();
            transforms_.clear();

            if(extended_request_)
            {
                confidences_.clear();
                box_rectangles_.clear();
            }

            for(size_t i=0; i < srv.response.ids.size(); i++)
            {
                model_ids_.push_back(srv.response.ids[i].data);

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
                transforms_.push_back(trans);

                if(extended_request_)
                {
                    confidences_.push_back(srv.response.confidence[i]);
                    pcl::PointCloud<pcl::PointXYZRGB> model;
                    pcl::fromROSMsg(srv.response.models_cloud[i], model);

                    int cx_, cy_;
                    cx_ = 640;
                    cy_ = 480;
                    float focal_length_ = 525.f;

                    float cx, cy;
                    cx = static_cast<float> (cx_) / 2.f; //- 0.5f;
                    cy = static_cast<float> (cy_) / 2.f; // - 0.5f;

                    int min_u, min_v, max_u, max_v;
                    min_u = min_v = cx_;
                    max_u = max_v = 0;

                    for(size_t j=0; j < model.points.size(); j++)
                    {

                        float x = model.points[j].x;
                        float y = model.points[j].y;
                        float z = model.points[j].z;
                        int u = static_cast<int> (focal_length_ * x / z + cx);
                        int v = static_cast<int> (focal_length_ * y / z + cy);

                        if (u >= cx_ || v >= cy_ || u < 0 || v < 0)
                          continue;

                        if(u < min_u)
                            min_u = u;

                        if(v < min_v)
                            min_v = v;

                        if(u > max_u)
                            max_u = u;

                        if(v > max_v)
                            max_v = v;
                    }

                    cv::Point min, max;
                    min.x = min_u;
                    min.y = min_v;

                    max.x = max_u;
                    max.y = max_v;

                    std::cout << min_u << " " << min_v << " .... " << max_u << " " << max_v << std::endl;

                    box_rectangles_.push_back(std::make_pair(min,max));
                }
            }

            std::cout << "Called done..." << std::endl;
        }
        else
        {
            ROS_ERROR("Failed to call /recognition_service/mp_recognition");
            return false;
        }
        return true;
    }

    void
    callService (const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
      std::cout << "Received point cloud.\n" << std::endl;
        // if any service is not available, wait for 5 sec and check again
        if( all_required_services_okay_ || ( !all_required_services_okay_ && (service_calls_ % (1 * 5)) == 0))
        {
            std::cout << "going to call service..." << std::endl;

            all_required_services_okay_ = callSegAndClassifierService(msg);

            if (visualize_output_ && all_required_services_okay_)
            {
                pcl::fromROSMsg(*msg, *scene_);
                visualize_output();
            }
        }
        service_calls_++;

    }
public:
    SOCDemo()
    {
        scene_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        KINECT_OK_ = false;
        topic_ = "/camera/depth_registered/points";
        kinect_trials_ = 5;
        all_required_services_okay_ = false;
        new_models_added_ = false;
        visualize_output_ = false;
        models_dir_ = "";
        extended_request_ = true;
        input_ = 0;
        file_name_ = "";
    }

    bool initialize(int argc, char ** argv)
    {
        ros::init (argc, argv, "classifier_demo");
        if (sizeof(int) != 4)
        {
            ROS_WARN("PC Architectur does not use 32bit for integer - check conflicts with pcl indices.");
        }
        n_ = new ros::NodeHandle ( "~" );

        if(!n_->getParam ( "topic", topic_ ))
            topic_ = "/camera/depth_registered/points";

        if(!n_->getParam ( "visualize_output", visualize_output_ ))
            visualize_output_ = false;

        if(!n_->getParam ( "models_dir", models_dir_ ))
            models_dir_ = "";

        if(!n_->getParam ( "new_models", new_models_added_ ))
            new_models_added_ = false;

        n_->getParam ( "file_name", file_name_ );
        n_->getParam ( "input_mode", input_ );


        std::string test_models_to_load;
        if(!n_->getParam ( "load_models", test_models_to_load ))
            test_models_to_load = "";

        std::cout << "aaa" << test_models_to_load << "aaa" << std::endl;

        if(test_models_to_load.compare("") != 0)
        {
            boost::split(model_ids_to_be_loaded_, test_models_to_load, boost::is_any_of(","), boost::token_compress_on);
        }
        else
        {
            std::cout << "aaa" << test_models_to_load << "aaa" << std::endl;
            model_ids_to_be_loaded_.clear();
        }

        if (input_ == 0)
        {
            checkKinect();
        }

        if(models_dir_.compare("") != 0)
        {
            models_source_.reset (new v4r::rec_3d_framework::ModelOnlySource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>);
            models_source_->setPath (models_dir_);
            models_source_->setLoadViews (false);
            models_source_->setModelScale(1);
            models_source_->setLoadIntoMemory(false);

            std::string test = "irrelevant";
            models_source_->setExtension("pcd");
            models_source_->generate (test);
        }

        return KINECT_OK_;
    }

    void run()
    {
        if ( input_ == 0)     // Reading point cloud from Kinect and calling recognition
        {
            ros::Subscriber sub_pc = n_->subscribe (topic_, 1, &SOCDemo::callService, this);
            ros::spin();
        }
        else
        {
            testFromFile();
        }
    }

    void testFromFile()
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::io::loadPCDFile(file_name_, *scene_);

        sensor_msgs::PointCloud2::Ptr pMsg (new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*scene_, *pMsg);

        all_required_services_okay_ = callSegAndClassifierService( pMsg );

    }

    void visualize_output()
    {

        if(extended_request_)
        {
            cv::Mat_<cv::Vec3b> image;
            PCLOpenCV::ConvertPCLCloud2Image<pcl::PointXYZRGB>(scene_, image);

            for(size_t kk=0; kk < transforms_.size(); kk++)
            {

                std::string model_id(model_ids_[kk], 0, 15);
                cv::putText(image, model_id, box_rectangles_[kk].first,
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 0.65, cv::Scalar(255,0,0), 1, CV_AA);

                cv::rectangle(image, box_rectangles_[kk].first, box_rectangles_[kk].second, cv::Scalar( 0, 255, 255 ), 2);
            }

            cv::imshow("results", image);
            cv::waitKey(0);
        }
        else
        {
            if(!vis_)
            {
                vis_.reset(new pcl::visualization::PCLVisualizer("classifier visualization"));
            }

            int v1, v2;
            vis_->createViewPort(0,0,0.5,1,v1);
            vis_->createViewPort(0.5,0,1,1,v2);

            vis_->addCoordinateSystem(0.2f);

            std::cout << scene_->points.size() << std::endl;
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler(scene_);
            vis_->addPointCloud(scene_, handler, "scene", v1);

            std::cout << models_dir_ << " " << transforms_.size() << std::endl;

            if(models_dir_.compare("") != 0 && transforms_.size() > 0)
            {
                //show models

                for(size_t kk=0; kk < transforms_.size(); kk++)
                {
                    boost::shared_ptr<v4r::rec_3d_framework::Model<pcl::PointXYZRGB> > model;

                    models_source_->getModelById(model_ids_[kk], model);
                    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr model_cloud = model->getAssembled (0.003f);
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_aligned (new pcl::PointCloud<pcl::PointXYZRGB>);
                    pcl::transformPointCloud (*model_cloud, *model_aligned, transforms_[kk]);

                    std::stringstream name;
                    name << "hypotheses_" << kk;

                    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler(model_aligned);
                    vis_->addPointCloud<pcl::PointXYZRGB> (model_aligned, handler, name.str (), v2);

                    std::cout << "adding " << name.str() << std::endl;
                }
            }
            vis_->spin();

            vis_->removeAllPointClouds();
        }
    }
};

int
main (int argc, char ** argv)
{
    SOCDemo m;
    m.initialize (argc, argv);
    m.run();
    return 0;
}
