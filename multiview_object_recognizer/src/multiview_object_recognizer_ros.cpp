#include "multiview_object_recognizer_ros.h"
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <v4r_config.h>
#include <v4r/common/miscellaneous.h>
#include <v4r/common/visibility_reasoning.h>
#include <v4r/common/pcl_opencv.h>

#include <boost/program_options.hpp>
#include <glog/logging.h>

namespace po = boost::program_options;

namespace v4r
{
template<typename PointT>
bool
multiviewRecognizerROS<PointT>::respondSrvCall(recognition_srv_definitions::recognize::Request &req,
                            recognition_srv_definitions::recognize::Response &response) const
{
    typename pcl::PointCloud<PointT>::Ptr pRecognizedModels (new pcl::PointCloud<PointT>);
    cv::Mat annotated_img = ConvertPCLCloud2Image(*scene_);

    std::vector<ModelTPtr> models_verified = mv_r_->getVerifiedModels();
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_verified = mv_r_->getVerifiedTransforms();

    for (size_t j = 0; j < models_verified.size(); j++)
    {
      std_msgs::String ss_tmp;
      ss_tmp.data = models_verified[j]->id_;
      response.ids.push_back(ss_tmp);

      Eigen::Matrix4f trans = transforms_verified[j];
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

      typename pcl::PointCloud<PointT>::ConstPtr model_cloud = models_verified[j]->getAssembled ( resolution_ );
      typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
      pcl::transformPointCloud (*model_cloud, *model_aligned, transforms_verified[j]);
      *pRecognizedModels += *model_aligned;
      sensor_msgs::PointCloud2 rec_model;
      pcl::toROSMsg(*model_aligned, rec_model);
      response.models_cloud.push_back(rec_model);

      pcl::PointCloud<pcl::Normal>::ConstPtr normal_cloud = models_verified[j]->getNormalsAssembled ( resolution_ );

      pcl::PointCloud<pcl::Normal>::Ptr normal_aligned (new pcl::PointCloud<pcl::Normal>);
      transformNormals(*normal_cloud, *normal_aligned, transforms_verified[j]);

      //ratio of inlier points
      float confidence = 0;
      const float focal_length = 525.f;
      const int img_width = 640;
      const int img_height = 480;

      VisibilityReasoning<pcl::PointXYZRGB> vr (focal_length, img_width, img_height);
      vr.setThresholdTSS (0.01f);
      /*float fsv_ratio =*/ vr.computeFSVWithNormals (scene_, model_aligned, normal_aligned);
      confidence = 1.f - vr.getFSVUsedPoints() / static_cast<float>(model_aligned->points.size());
      response.confidence.push_back(confidence);

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

      const float cx = static_cast<float> (img_width) / 2.f; //- 0.5f;
      const float cy = static_cast<float> (img_height) / 2.f; // - 0.5f;

      int min_u, min_v, max_u, max_v;
      min_u = img_width;
      min_v = img_height;
      max_u = max_v = 0;

      for(size_t m_pt_id=0; m_pt_id < model_aligned->points.size(); m_pt_id++)
      {
          const float x = model_aligned->points[m_pt_id].x;
          const float y = model_aligned->points[m_pt_id].y;
          const float z = model_aligned->points[m_pt_id].z;
          const int u = static_cast<int> (focal_length * x / z + cx);
          const int v = static_cast<int> (focal_length * y / z + cy);

          if (u >= img_width || v >= img_width || u < 0 || v < 0)
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
      cv::putText(annotated_img, models_verified[j]->id_, text_start, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,0,255), 1, CV_AA);
      cv::rectangle(annotated_img, cv::Point(min_u, min_v), cv::Point(max_u, max_v), cv::Scalar( 0, 255, 255 ), 2);
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
multiviewRecognizerROS<PointT>::recognizeROS (recognition_srv_definitions::recognize::Request & req, recognition_srv_definitions::recognize::Response & response)
{
    pcl::fromROSMsg (req.cloud, *scene_);
    Eigen::Matrix4f tf = RotTrans2Mat4f(scene_->sensor_orientation_, scene_->sensor_origin_);

    // reset view point otherwise pcl visualization is potentially messed up
    scene_->sensor_orientation_ = Eigen::Quaternionf::Identity();
    scene_->sensor_origin_ = Eigen::Vector4f::Zero();

    mv_r_->setInputCloud (scene_);
    mv_r_->setCameraPose(tf);
    mv_r_->recognize();

    respondSrvCall(req, response);
    return true;
}

template<typename PointT>
bool
multiviewRecognizerROS<PointT>::initialize(int argc, char **argv)
{
    n_.reset( new ros::NodeHandle ( "~" ) );
    mv_r_.reset( new v4r::MultiviewRecognizer<PointT>(argc, argv));

    vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "multiview_recognized_objects", 0 );
    recognition_serv_ = n_->advertiseService("multiview_recognition_service", &multiviewRecognizerROS::recognizeROS, this);
    it_.reset(new image_transport::ImageTransport(*n_));
    image_pub_ = it_->advertise("multiview_recogniced_object_instances_img", 1, true);

    return 1;
}
}

int
main (int argc, char ** argv)
{
  ros::init (argc, argv, "multiview_recognition_service");
  v4r::multiviewRecognizerROS<pcl::PointXYZRGB> m;
  m.initialize (argc, argv);
  ros::spin ();
  return 0;
}
