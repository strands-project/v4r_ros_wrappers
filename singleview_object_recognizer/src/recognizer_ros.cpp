#include "recognizer_ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <v4r/common/pcl_opencv.h>

#include <iostream>
#include <sstream>

namespace po = boost::program_options;

namespace v4r
{
template<typename PointT>
bool
RecognizerROS<PointT>::respondSrvCall(recognition_srv_definitions::recognize::Request &req,
                                      recognition_srv_definitions::recognize::Response &response) const
{
    typename pcl::PointCloud<PointT>::Ptr pRecognizedModels (new pcl::PointCloud<PointT>);

    // convert point cloud
    v4r::PCLOpenCVConverter<PointT> img_conv;
    img_conv.setInputCloud(scene_);

    if(!scene_->isOrganized())
    {
        ROS_WARN("Input cloud is not organized!");
        if( !camera_ )
        {
            ROS_WARN("Camera is not defined. Defaulting to Kinect parameters!");
            float focal_length = 525.f;
            size_t img_width = 640;
            size_t img_height = 480;
            float cx = 319.5f;
            float cy = 239.5f;
            camera_.reset( new v4r::Camera(focal_length, img_width, img_height, cx, cy) );
        }
        img_conv.setCamera( camera_ );
    }
    cv::Mat annotated_img = img_conv.getRGBImage();

    for(size_t ohg_id=0; ohg_id<object_hypotheses_.size(); ohg_id++)
    {
        for(const v4r::ObjectHypothesis::Ptr &oh : object_hypotheses_[ohg_id].ohs_)
        {
            if( ! oh->is_verified_ )
                continue;

            std_msgs::String ss_tmp;
            ss_tmp.data = oh->model_id_;
            response.ids.push_back(ss_tmp);

            Eigen::Matrix4f trans = oh->transform_;
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

            typename pcl::PointCloud<PointT>::ConstPtr model_cloud = mrec_->getModel( oh->model_id_, 5 );
            typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
            pcl::transformPointCloud (*model_cloud, *model_aligned, oh->transform_);
            *pRecognizedModels += *model_aligned;
            sensor_msgs::PointCloud2 rec_model;
            pcl::toROSMsg(*model_aligned, rec_model);
            response.models_cloud.push_back(rec_model);


            //        pcl::PointCloud<pcl::Normal>::ConstPtr normal_cloud = model->getNormalsAssembled ( resolution_ );

            //        pcl::PointCloud<pcl::Normal>::Ptr normal_aligned (new pcl::PointCloud<pcl::Normal>);
            //        transformNormals(*normal_cloud, *normal_aligned, oh->transform_);
            //      VisibilityReasoning<pcl::PointXYZRGB> vr (focal_length, img_width, img_height);
            //      vr.setThresholdTSS (0.01f);
            //      /*float fsv_ratio =*/ vr.computeFSVWithNormals (scene_, model_aligned, normal_aligned);
            //      confidence = 1.f - vr.getFSVUsedPoints() / static_cast<float>(model_aligned->points.size());
            //      response.confidence.push_back(confidence);

            //centroid and BBox
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*model_aligned, centroid);
            geometry_msgs::Point32 centroid_msg;
            centroid_msg.x = centroid[0];
            centroid_msg.y = centroid[1];
            centroid_msg.z = centroid[2];
            response.centroid.push_back(centroid_msg);

            Eigen::Vector4f min;
            Eigen::Vector4f max;
            pcl::getMinMax3D (*model_aligned, min, max);

            object_perception_msgs::BBox bbox;
            geometry_msgs::Point32 pt;
            pt.x = min[0]; pt.y = min[1]; pt.z = min[2]; bbox.point.push_back(pt);
            pt.x = min[0]; pt.y = min[1]; pt.z = max[2]; bbox.point.push_back(pt);
            pt.x = min[0]; pt.y = max[1]; pt.z = min[2]; bbox.point.push_back(pt);
            pt.x = min[0]; pt.y = max[1]; pt.z = max[2]; bbox.point.push_back(pt);
            pt.x = max[0]; pt.y = min[1]; pt.z = min[2]; bbox.point.push_back(pt);
            pt.x = max[0]; pt.y = min[1]; pt.z = max[2]; bbox.point.push_back(pt);
            pt.x = max[0]; pt.y = max[1]; pt.z = min[2]; bbox.point.push_back(pt);
            pt.x = max[0]; pt.y = max[1]; pt.z = max[2]; bbox.point.push_back(pt);
            response.bbox.push_back(bbox);

            if(!camera_)
            {
                ROS_WARN("Camera is not defined. Defaulting to Kinect parameters!");
                float focal_length = 525.f;
                size_t img_width = 640;
                size_t img_height = 480;
                float cx = 319.5f;
                float cy = 239.5f;
                camera_.reset( new v4r::Camera(focal_length, img_width, img_height, cx, cy) );
            }

            int min_u, min_v, max_u, max_v;
            min_u = annotated_img.cols;
            min_v = annotated_img.rows;
            max_u = max_v = 0;

            for(size_t m_pt_id=0; m_pt_id < model_aligned->points.size(); m_pt_id++)
            {
                const float x = model_aligned->points[m_pt_id].x;
                const float y = model_aligned->points[m_pt_id].y;
                const float z = model_aligned->points[m_pt_id].z;
                const int u = static_cast<int> ( camera_->getFocalLength() * x / z + camera_->getCx());
                const int v = static_cast<int> ( camera_->getFocalLength()  * y / z +  camera_->getCy());

                if (u >= annotated_img.cols || v >= annotated_img.rows || u < 0 || v < 0)
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

            cv::Point text_start;
            text_start.x = min_u;
            text_start.y = std::max(0, min_v - 10);
            cv::putText(annotated_img, oh->model_id_, text_start, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,0,255), 1, CV_AA);
            cv::rectangle(annotated_img, cv::Point(min_u, min_v), cv::Point(max_u, max_v), cv::Scalar( 0, 255, 255 ), 2);
        }
    }

    sensor_msgs::PointCloud2 recognizedModelsRos;
    pcl::toROSMsg (*pRecognizedModels, recognizedModelsRos);
    recognizedModelsRos.header.frame_id = req.cloud.header.frame_id;
    vis_pc_pub_.publish(recognizedModelsRos);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", annotated_img).toImageMsg();
    image_pub_.publish(msg);

    return true;
}

template<typename PointT>
bool
RecognizerROS<PointT>::recognizeROS(recognition_srv_definitions::recognize::Request &req,
                                    recognition_srv_definitions::recognize::Response &response)
{
    scene_.reset(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg (req.cloud, *scene_);
    object_hypotheses_ = mrec_->recognize( scene_ );

    for(size_t ohg_id=0; ohg_id<object_hypotheses_.size(); ohg_id++)
    {
        for(const v4r::ObjectHypothesis::Ptr &oh : object_hypotheses_[ohg_id].ohs_)
        {
            const std::string &model_id = oh->model_id_;
            const Eigen::Matrix4f &tf = oh->transform_;

            if( oh->is_verified_ )
            {
                LOG(INFO) << "********************" << model_id << std::endl << tf << std::endl << std::endl;
            }
        }
    }

    return respondSrvCall(req, response);
}

template<typename PointT>
bool
RecognizerROS<PointT>::initialize (int argc, char ** argv)
{
    n_.reset( new ros::NodeHandle ( "~" ) );

    std::vector<std::string> arguments(argv + 1, argv + argc);

    if(arguments.empty())
    {
        std::string models_dir;
        if( n_->getParam ( "models_dir", models_dir ) && !models_dir.empty() )
        {
            arguments.push_back("-m");
            arguments.push_back(models_dir);
        }
        else
        {
            ROS_ERROR("Models directory is not set. Must be set with param \"m\"!");
            return false;
        }

//        std::string hv_config_xml;
//        if( n_->getParam ( "hv_config_xml", hv_config_xml ) )
//        {
//            arguments.push_back("--hv_config_xml");
//            arguments.push_back(hv_config_xml);
//        }
//        std::string sift_config_xml;
//        if( n_->getParam ( "sift_config_xml", sift_config_xml ) )
//        {
//            arguments.push_back("--sift_config_xml");
//            arguments.push_back(sift_config_xml);
//        }
//        std::string shot_config_xml;
//        if( n_->getParam ( "shot_config_xml", shot_config_xml ) )
//        {
//            arguments.push_back("--shot_config_xml");
//            arguments.push_back(shot_config_xml);
//        }
//        std::string esf_config_xml;
//        if( n_->getParam ( "esf_config_xml", esf_config_xml ) )
//        {
//            arguments.push_back("--esf_config_xml");
//            arguments.push_back(esf_config_xml);
//        }
//        std::string alexnet_config_xml;
//        if( n_->getParam ( "alexnet_config_xml", alexnet_config_xml ) )
//        {
//            arguments.push_back("--alexnet_config_xml");
//            arguments.push_back(alexnet_config_xml);
//        }
//        std::string camera_xml;
//        if( n_->getParam ( "camera_xml", camera_xml ) )
//        {
//            arguments.push_back("--camera_xml");
//            arguments.push_back(camera_xml);
//        }
        std::string additional_arguments;
        if( n_->getParam ( "arg", additional_arguments ) )
        {
            std::vector<std::string> strs;
            boost::split( strs, additional_arguments, boost::is_any_of("\t ") );
            arguments.insert( arguments.end(), strs.begin(), strs.end() );
        }
    }

    std::cout << "Initializing recognizer with: " << std::endl;
    for( auto arg : arguments )
        std::cout << arg << " ";
    std::cout << std::endl;

    mrec_param_.load("cfg/multipipeline_config.xml");
    mrec_.reset(new v4r::apps::ObjectRecognizer<PointT>(mrec_param_));
    mrec_->initialize(arguments);

    ROS_INFO("Ready to get service calls.");

    vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "sv_recogniced_object_instances", 1 );
    recognize_  = n_->advertiseService ("sv_recognition", &RecognizerROS::recognizeROS, this);

    it_.reset(new image_transport::ImageTransport(*n_));
    image_pub_ = it_->advertise("sv_recogniced_object_instances_img", 1, true);

    ROS_INFO("Ready to get service calls.");
    return true;
}

}

int
main (int argc, char ** argv)
{
    ros::init (argc, argv, "recognition_service");
    v4r::RecognizerROS<pcl::PointXYZRGB> m;
    m.initialize (argc, argv);
    ros::spin ();
    return 0;
}
