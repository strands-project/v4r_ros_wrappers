#include "recognizer_ros.h"
#include "pcl_conversions.h"
#include <cv_bridge/cv_bridge.h>

#include <v4r/common/visibility_reasoning.h>
#include <v4r/common/miscellaneous.h>
#include <v4r/common/pcl_opencv.h>

namespace v4r
{

bool
RecognizerROS::retrainROS (recognition_srv_definitions::retrain_recognizer::Request & req,
         recognition_srv_definitions::retrain_recognizer::Response & response)
{
      std::vector<std::string> model_ids;
      std::cout << "Number of ids:" << req.load_ids.size() << std::endl;

      for(size_t i=0; i < req.load_ids.size(); i++)
      {
          model_ids.push_back(req.load_ids[i].data);
          std::cout << req.load_ids[i].data << std::endl;
      }
      retrain(model_ids);
      return true;
}

bool RecognizerROS::respondSrvCall(recognition_srv_definitions::recognize::Request &req,
                            recognition_srv_definitions::recognize::Response &response) const
{
    pcl::PointCloud<PointT>::Ptr pRecognizedModels (new pcl::PointCloud<PointT>);
    cv::Mat_<cv::Vec3b> annotated_img;

    PCLOpenCV::ConvertPCLCloud2Image<PointT>(pInputCloud_, annotated_img);

    for (size_t j = 0; j < models_verified_.size(); j++)
    {
      std_msgs::String ss_tmp;
      ss_tmp.data = models_verified_[j]->id_;
      response.ids.push_back(ss_tmp);

      Eigen::Matrix4f trans = transforms_verified_[j];
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

      ConstPointInTPtr model_cloud = models_verified_[j]->getAssembled ( resolution_ );
      typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
      pcl::transformPointCloud (*model_cloud, *model_aligned, transforms_verified_[j]);
      *pRecognizedModels += *model_aligned;
      sensor_msgs::PointCloud2 rec_model;
      pcl::toROSMsg(*model_aligned, rec_model);
      response.models_cloud.push_back(rec_model);

      pcl::PointCloud<pcl::Normal>::ConstPtr normal_cloud = models_verified_[j]->getNormalsAssembled ( resolution_ );

      typename pcl::PointCloud<pcl::Normal>::Ptr normal_aligned (new pcl::PointCloud<pcl::Normal>);
      v4r::transformNormals(normal_cloud, normal_aligned, transforms_verified_[j]);

      //ratio of inlier points
      float confidence = 0;
      const float focal_length = 525.f;
      const int img_width = 640;
      const int img_height = 480;

      v4r::VisibilityReasoning<pcl::PointXYZRGB> vr (focal_length, img_width, img_height);
      vr.setThresholdTSS (0.01f);
      /*float fsv_ratio =*/ vr.computeFSVWithNormals (pInputCloud_, model_aligned, normal_aligned);
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
      cv::putText(annotated_img, models_verified_[j]->id_, text_start, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,0,255), 1, CV_AA);
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

bool RecognizerROS::recognizeROS(recognition_srv_definitions::recognize::Request &req,
                                 recognition_srv_definitions::recognize::Response &response)
{
    pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg (req.cloud, *scene);
    setInputCloud(scene);
    recognize();
    return respondSrvCall(req, response);
}

void
RecognizerROS::initialize (int argc, char ** argv)
{
    n_.reset( new ros::NodeHandle ( "~" ) );
    n_->getParam ( "models_dir", models_dir_);
    n_->getParam ( "training_dir_sift", training_dir_sift_);
    n_->getParam ( "training_dir_shot", training_dir_shot_);
    n_->getParam ( "recognizer_structure_sift", sift_structure_);
    n_->getParam ( "training_dir_ourcvfh", training_dir_ourcvfh_);
    n_->getParam ( "chop_z", sv_params_.chop_at_z_ );
    n_->getParam ( "icp_iterations", sv_params_.icp_iterations_);
    n_->getParam ( "do_sift", sv_params_.do_sift_);
    n_->getParam ( "do_shot", sv_params_.do_shot_);
    n_->getParam ( "do_ourcvfh", sv_params_.do_ourcvfh_);
    n_->getParam ( "knn_sift", sv_params_.knn_sift_);
    n_->getParam ( "publish_debug", debug_publish_);

    n_->getParam ( "cg_size_thresh", cg_params_.cg_size_threshold_);
    n_->getParam ( "cg_size", cg_params_.cg_size_);
    n_->getParam ( "cg_ransac_threshold", cg_params_.ransac_threshold_);
    n_->getParam ( "cg_dist_for_clutter_factor", cg_params_.dist_for_clutter_factor_);
    n_->getParam ( "cg_max_taken", cg_params_.max_taken_);
    n_->getParam ( "cg_max_time_for_cliques_computation", cg_params_.max_time_for_cliques_computation_);
    n_->getParam ( "cg_dot_distance", cg_params_.dot_distance_);
    n_->getParam ( "use_cg_graph", cg_params_.use_cg_graph_);

    n_->getParam ( "hv_resolution", hv_params_.resolution_);
    n_->getParam ( "hv_inlier_threshold", hv_params_.inlier_threshold_);
    n_->getParam ( "hv_radius_clutter", hv_params_.radius_clutter_);
    n_->getParam ( "hv_regularizer", hv_params_.regularizer_);
    n_->getParam ( "hv_clutter_regularizer", hv_params_.clutter_regularizer_);
    n_->getParam ( "hv_occlusion_threshold", hv_params_.occlusion_threshold_);
    n_->getParam ( "hv_optimizer_type", hv_params_.optimizer_type_);
    n_->getParam ( "hv_color_sigma_l", hv_params_.color_sigma_l_);
    n_->getParam ( "hv_color_sigma_ab", hv_params_.color_sigma_ab_);
    n_->getParam ( "hv_use_supervoxels", hv_params_.use_supervoxels_);
    n_->getParam ( "hv_detect_clutter", hv_params_.detect_clutter_);
    n_->getParam ( "hv_ignore_color", hv_params_.ignore_color_);

    std::cout << sv_params_.chop_at_z_ << ", " << hv_params_.ignore_color_ << ", do_shot:" << sv_params_.do_shot_ << std::endl;
  if (models_dir_.compare ("") == 0)
  {
    PCL_ERROR ("Set -models_dir option in the command line, ABORTING");
    return;
  }

  if ( sv_params_.do_sift_ && training_dir_sift_.compare ("") == 0)
  {
    std::cout << "do_sift is activated but training_dir_sift_ is empty! Set -training_dir_sift option in the command line if you want to keep your trained models. " << std::endl;
    return;
  }

  if ( sv_params_.do_ourcvfh_ && training_dir_ourcvfh_.compare ("") == 0)
  {
    std::cout << "do_ourcvfh is activated but training_dir_ourcvfh_ is empty! Set -training_dir_ourcvfh option in the command lineif you want to keep your trained models. " << std::endl;
    return;
  }

  if ( sv_params_.do_shot_ && training_dir_shot_.compare ("") == 0)
  {
    std::cout << "do_shot is activated but training_dir_shot_ is empty! Set -training_dir_shot option in the command line if you want to keep your trained models. " << std::endl;
    return;
  }
  vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "sv_recogniced_object_instances", 1 );
  recognize_  = n_->advertiseService ("sv_recognition", &RecognizerROS::recognizeROS, this);

  it_.reset(new image_transport::ImageTransport(*n_));
  image_pub_ = it_->advertise("sv_recogniced_object_instances_img", 1, true);

  initialize();
  std::cout << "Initialized single-view recognizer with these settings:" << std::endl
            << "==========================================================" << std::endl;
  printParams();
}

}

int
main (int argc, char ** argv)
{
  ros::init (argc, argv, "recognition_service");

  v4r::RecognizerROS m;
  m.initialize (argc, argv);
  ros::spin ();

  return 0;
}
