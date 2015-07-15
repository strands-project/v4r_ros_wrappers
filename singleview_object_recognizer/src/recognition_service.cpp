/*
 * shape_simple_classifier_node.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: aitor
 */

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float32.h"
#include "object_perception_msgs/BBox.h"
#include <pcl/apps/dominant_plane_segmentation.h>
#include <pcl/common/common.h>
#include <pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include "segmenter.h"
#include <v4r/ORFramework/color_ourcvfh_estimator.h>
#include <v4r/ORFramework/global_nn_recognizer_cvfh.h>
#include <v4r/ORFramework/local_recognizer.h>
#include <v4r/ORFramework/metrics.h>
#include <v4r/ORFramework/multi_pipeline_recognizer.h>
#include <v4r/ORFramework/multiplane_segmentation.h>
#include <v4r/ORFramework/organized_color_ourcvfh_estimator.h>
#include <v4r/ORFramework/shot_local_estimator_omp.h>
#include <v4r/ORFramework/ourcvfh_estimator.h>
#include <v4r/ORFramework/partial_pcd_source.h>
#include <v4r/ORFramework/registered_views_source.h>
#include <v4r/ORFramework/sift_local_estimator.h>
#include <v4r/Registration/correspondence_grouping.h>
#include <v4r/Registration/graph_geometric_consistency.h>
#include <v4r/ORRecognition/hv_go_3D.h>
#include <v4r/ORUtils/miscellaneous.h>
#include "recognition_srv_definitions/recognize.h"
#include "recognition_srv_definitions/retrain_recognizer.h"
#include "recognition_srv_definitions/get_configuration.h"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include <v4r/Registration/VisibilityReasoning.h>
#include <v4r/ORUtils/miscellaneous.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//#define USE_SIFT_GPU
//#define SOC_VISUALIZE

#ifdef SOC_VISUALIZE
#include <pcl/visualization/pcl_visualizer.h>
#endif

#ifndef USE_SIFT_GPU
    //WARNING: The SIFT feature is not included in the opencv bundled within ROS
    //make sure to link to the proper opencv version including nonfree features
    #include <v4r/ORFramework/opencv_sift_local_estimator.h>
#endif

bool USE_SEGMENTATION_ = false;

struct camPosConstraints
{
    bool
    operator() (const Eigen::Vector3f & pos) const
    {
        if (pos[2] > 0)
            return true;

        return false;
    }
    ;
};

class Recognizer
{
private:
  size_t num_recorded_clouds_;
  typedef pcl::PointXYZRGB PointT;
  std::string models_dir_;
  std::string training_dir_sift_;
  std::string training_dir_shot_;
  std::string sift_structure_;
  std::string training_dir_ourcvfh_;
  bool do_sift_;
  bool do_ourcvfh_;
  bool do_shot_;
  double chop_at_z_;
  int icp_iterations_;
  std::vector<std::string> text_3d_;
  boost::shared_ptr<faat_pcl::rec_3d_framework::MultiRecognitionPipeline<PointT> > multi_recog_;
  int v1_,v2_, v3_;
  ros::ServiceServer recognize_;
  ros::ServiceServer retrain_recognizer_;
  ros::ServiceServer get_configuration_;

  boost::shared_ptr<ros::NodeHandle> n_;
  ros::Publisher vis_pc_pub_;

  std::string idx_flann_fn_sift_;
  std::string idx_flann_fn_shot_;

  //Parameters for recognition
  int knn_sift_;
  int cg_size_;
  bool use_cg_graph_;

  //Parameters for hypothesis verification
  bool ignore_color_;

  //PUblishing image
  bool debug_publish_;
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher image_pub_;

#ifdef SOC_VISUALIZE
  boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
#endif

  void createDebugImageAndPublish(recognition_srv_definitions::recognize::Response & response,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & scene_)
  {

      std::vector<std::string> model_ids_;
      std::vector<Eigen::Matrix4f> transforms_;
      std::vector<float> confidences_;
      std::vector<std::pair<cv::Point, cv::Point> > box_rectangles_;

      bool extended_request_ = true;

      for(size_t i=0; i < response.ids.size(); i++)
      {
          model_ids_.push_back(response.ids[i].data);

          Eigen::Quaternionf q(response.transforms[i].rotation.w,
                               response.transforms[i].rotation.x,
                               response.transforms[i].rotation.y,
                               response.transforms[i].rotation.z);

          Eigen::Vector3f translation(response.transforms[i].translation.x,
                                      response.transforms[i].translation.y,
                                      response.transforms[i].translation.z);


          Eigen::Matrix4f trans;
          trans.block<3,3>(0,0) = q.toRotationMatrix();
          trans.block<3,1>(0,3) = translation;
          transforms_.push_back(trans);

          if(extended_request_)
          {
              confidences_.push_back(response.confidence[i]);
              pcl::PointCloud<pcl::PointXYZRGB> model;
              pcl::fromROSMsg(response.models_cloud[i], model);

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

      cv::Mat_<cv::Vec3b> image;
      PCLOpenCV::ConvertPCLCloud2Image<pcl::PointXYZRGB>(scene_, image);

      for(size_t kk=0; kk < transforms_.size(); kk++)
      {


          cv::Point text_start;
          text_start.x = box_rectangles_[kk].first.x;
          text_start.y = std::max(0, box_rectangles_[kk].first.y - 10);
          std::string model_id(model_ids_[kk], 0, 15);
          cv::putText(image, model_id, text_start,
              cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,0,255), 1, CV_AA);

          cv::rectangle(image, box_rectangles_[kk].first, box_rectangles_[kk].second, cv::Scalar( 0, 255, 255 ), 2);
      }

      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      image_pub_.publish(msg);
  }

  bool
  getConfig (recognition_srv_definitions::get_configuration::Request & req,
           recognition_srv_definitions::get_configuration::Response & response)
  {
        response.models_folder.data = models_dir_;
        response.recognition_structure_folder.data = sift_structure_;
  }

  bool
  retrain (recognition_srv_definitions::retrain_recognizer::Request & req,
           recognition_srv_definitions::retrain_recognizer::Response & response)
  {
        //delete .idx files from recognizers
        { //sift flann idx
            bf::path file(idx_flann_fn_sift_);
             if(bf::exists(file))
                bf::remove(file);
        }

        { //shot flann idx
          bf::path file(idx_flann_fn_shot_);
           if(bf::exists(file))
              bf::remove(file);
        }

        //call multi_recog_->reinitialize()

        std::vector<std::string> model_ids;
        std::cout << "Number of ids:" << req.load_ids.size() << std::endl;

        for(size_t i=0; i < req.load_ids.size(); i++)
        {
            model_ids.push_back(req.load_ids[i].data);
            std::cout << req.load_ids[i].data << std::endl;
        }

        std::cout << "Never finishes this..." << std::endl;

        if(model_ids.empty())
        {
            std::cout << "Number of ids:" << req.load_ids.size() << " model_ids.empty()" << std::endl;
            multi_recog_->reinitialize();
        }
        else
        {
            std::cout << "Number of ids:" << req.load_ids.size() << std::endl;
            multi_recog_->reinitialize(model_ids);
        }

        return true;
  }

  bool
  recognize (recognition_srv_definitions::recognize::Request & req, recognition_srv_definitions::recognize::Response & response)
  {
    typedef faat_pcl::rec_3d_framework::Model<PointT> ModelT;
    typedef boost::shared_ptr<ModelT> ModelTPtr;
    typedef typename pcl::PointCloud<PointT>::ConstPtr ConstPointInTPtr;

    pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg (req.cloud, *scene);

    std::stringstream filepath_ss;
    filepath_ss << "/tmp/recorded_cloud_" << num_recorded_clouds_ << ".pcd";
    pcl::io::savePCDFileBinary(filepath_ss.str(), *scene);
    num_recorded_clouds_++;

    float go_resolution_ = 0.005f;
    bool add_planes = true;

    //initialize go
    boost::shared_ptr<faat_pcl::GHV<PointT, PointT> > go (
                    new faat_pcl::GHV<PointT,
                    PointT>);

    go->setSmoothSegParameters(0.1, 0.035, 0.005);
    //go->setRadiusNormals(0.03f);
    go->setResolution (go_resolution_);
    go->setInlierThreshold (0.015);
    go->setRadiusClutter (0.03f);
    go->setRegularizer (3);
    go->setClutterRegularizer (5);
    go->setDetectClutter (true);
    go->setOcclusionThreshold (0.01f);
    go->setOptimizerType(0);
    go->setUseReplaceMoves(true);
    go->setRadiusNormals(0.02);
    go->setRequiresNormals(false);
    go->setInitialStatus(false);
    go->setIgnoreColor(true);
    go->setColorSigma(0.5f, 0.5f);
    go->setHistogramSpecification(true);
    go->setVisualizeGoCues(0);
    go->setUseSuperVoxels(false);

    typename pcl::PointCloud<PointT>::Ptr occlusion_cloud (new pcl::PointCloud<PointT>(*scene));
    if(chop_at_z_ > 0)
    {
        pcl::PassThrough<PointT> pass_;
        pass_.setFilterLimits (0.f, static_cast<float>(chop_at_z_));
        pass_.setFilterFieldName ("z");
        pass_.setInputCloud (scene);
        pass_.setKeepOrganized (true);
        pass_.filter (*scene);
    }

    if (scene->points.size()==0)
{
	std::cerr << "Input cloud is empty. No models found." << std::endl;
	return false;
}
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    ne.setRadiusSearch(0.02f);
    ne.setInputCloud (scene);
    ne.compute (*normal_cloud);

    //Multiplane segmentation
    faat_pcl::MultiPlaneSegmentation<PointT> mps;
    mps.setInputCloud(scene);
    mps.setMinPlaneInliers(1000);
    mps.setResolution(go_resolution_);
    mps.setNormals(normal_cloud);
    mps.setMergePlanes(true);
    std::vector<faat_pcl::PlaneModel<PointT> > planes_found;
    mps.segment();
    planes_found = mps.getModels();

    if(USE_SEGMENTATION_)
    {
        std::vector<pcl::PointIndices> indices;
        Eigen::Vector4f table_plane;
        doSegmentation<PointT>(scene, normal_cloud, indices, table_plane);

        std::vector<int> indices_above_plane;
        for (int k = 0; k < scene->points.size (); k++)
        {
            Eigen::Vector3f xyz_p = scene->points[k].getVector3fMap ();
            if (!pcl_isfinite (xyz_p[0]) || !pcl_isfinite (xyz_p[1]) || !pcl_isfinite (xyz_p[2]))
                continue;

            float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];
            if (val >= 0.01)
                indices_above_plane.push_back (static_cast<int> (k));
        }

        multi_recog_->setSegmentation(indices);
        multi_recog_->setIndices(indices_above_plane);

    }

    multi_recog_->setSceneNormals(normal_cloud);
    multi_recog_->setInputCloud (scene);
    {
        pcl::ScopeTime ttt ("Recognition");
        multi_recog_->recognize ();
    }

    //HV
    //transforms models
    boost::shared_ptr < std::vector<ModelTPtr> > models = multi_recog_->getModels ();
    boost::shared_ptr < std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms = multi_recog_->getTransforms ();

    ROS_DEBUG("Number of recognition hypotheses %d\n", static_cast<int>(models->size()));
    if(models->size() == 0)
    {
        ROS_INFO("No models to verify, returning from service.");
        return true;
    }

    std::vector<typename pcl::PointCloud<PointT>::ConstPtr> aligned_models;
    aligned_models.resize (models->size ());
    std::vector<std::string> model_ids;
    for (size_t kk = 0; kk < models->size (); kk++)
    {
        ConstPointInTPtr model_cloud = models->at (kk)->getAssembled (go_resolution_);
        typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
        pcl::transformPointCloud (*model_cloud, *model_aligned, transforms->at (kk));
        aligned_models[kk] = model_aligned;
        model_ids.push_back(models->at (kk)->id_);
    }

    go->setSceneCloud (scene);
    go->setNormalsForClutterTerm(normal_cloud);
    go->setOcclusionCloud (occlusion_cloud);
    //addModels
    go->addModels (aligned_models, true);
    //append planar models
    if(add_planes)
    {
        go->addPlanarModels(planes_found);
        for(size_t kk=0; kk < planes_found.size(); kk++)
        {
            std::stringstream plane_id;
            plane_id << "plane_" << kk;
            model_ids.push_back(plane_id.str());
        }
    }

    go->setObjectIds(model_ids);
    //verify
    {
        pcl::ScopeTime t("Go verify");
        go->verify ();
    }
    std::vector<bool> mask_hv;
    go->getMask (mask_hv);

    std::vector<int> coming_from;
    coming_from.resize(aligned_models.size() + planes_found.size());
    for(size_t j=0; j < aligned_models.size(); j++)
        coming_from[j] = 0;

    for(size_t j=0; j < planes_found.size(); j++)
        coming_from[aligned_models.size() + j] = 1;

#ifdef SOC_VISUALIZE
    vis_->removeAllPointClouds();
    vis_->addPointCloud(scene, "scene", v1_);

    for(size_t kk=0; kk < planes_found.size(); kk++)
    {
        std::stringstream pname;
        pname << "plane_" << kk;
        pcl::visualization::PointCloudColorHandlerRandom<PointT> scene_handler(planes_found[kk].plane_cloud_);
        vis_->addPointCloud<PointT> (planes_found[kk].plane_cloud_, scene_handler, pname.str(), v2_);
        pname << "chull";
        vis_->addPolygonMesh (*planes_found[kk].convex_hull_, pname.str(), v2_);
    }
#endif

    boost::shared_ptr<std::vector<ModelTPtr> > verified_models(new std::vector<ModelTPtr>);
    boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > verified_transforms;
    verified_transforms.reset(new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);

#ifdef SOC_VISUALIZE
    if(models)
    {
        for (size_t j = 0; j < mask_hv.size (); j++)
        {
            std::stringstream name;
            name << "cloud_" << j;

            if(!mask_hv[j])
            {
                if(coming_from[j] == 0)
                {
                    ConstPointInTPtr model_cloud = models->at (j)->getAssembled (0.005f);
                    typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
                    pcl::transformPointCloud (*model_cloud, *model_aligned, transforms->at (j));
                    pcl::visualization::PointCloudColorHandlerRGBField<PointT> random_handler (model_aligned);
                    vis_->addPointCloud<PointT> (model_aligned, random_handler, name.str (), v2_);
                }
                continue;
            }

            if(coming_from[j] == 0)
            {
                verified_models->push_back(models->at(j));
                verified_transforms->push_back(transforms->at(j));

                ConstPointInTPtr model_cloud = models->at (j)->getAssembled (-1);
                typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
                pcl::transformPointCloud (*model_cloud, *model_aligned, transforms->at (j));
                std::cout << models->at (j)->id_ << std::endl;

                pcl::visualization::PointCloudColorHandlerRGBField<PointT> random_handler (model_aligned);
                vis_->addPointCloud<PointT> (model_aligned, random_handler, name.str (), v3_);
            }
            else
            {
                std::stringstream pname;
                pname << "plane_v2_" << j;
                pcl::visualization::PointCloudColorHandlerRandom<PointT> scene_handler(planes_found[j - models->size()].plane_cloud_);
                vis_->addPointCloud<PointT> (planes_found[j - models->size()].plane_cloud_, scene_handler, pname.str(), v3_);
                pname << "chull_v2";
                vis_->addPolygonMesh (*planes_found[j - models->size()].convex_hull_, pname.str(), v3_);
            }
        }
    }
#else
    if(models)
    {
        for (size_t j = 0; j < mask_hv.size (); j++)
        {
            if(!mask_hv[j])
                continue;

            if(coming_from[j] == 0)
            {
                verified_models->push_back(models->at(j));
                verified_transforms->push_back(transforms->at(j));
            }
            else
            {
                /*std::stringstream pname;
                pname << "plane_v2_" << j;
                pcl::visualization::PointCloudColorHandlerRandom<PointT> scene_handler(planes_found[j - models->size()].plane_cloud_);
                vis_->addPointCloud<PointT> (planes_found[j - models->size()].plane_cloud_, scene_handler, pname.str(), v3_);
                pname << "chull_v2";
                vis_->addPolygonMesh (*planes_found[j - models->size()].convex_hull_, pname.str(), v3_);*/
            }
        }
    }
#endif

    std::cout << "Number of models:" << models->size() << std::endl;
    std::cout << "Number of verified models:" << verified_models->size() << std::endl;

    //parse verified_models and generate response to service call
      //vector of id + pose

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pRecognizedModels (new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t j = 0; j < verified_models->size (); j++)
    {
      std_msgs::String ss;
      ss.data = verified_models->at(j)->id_;
      response.ids.push_back(ss);

      Eigen::Matrix4f trans = verified_transforms->at(j);
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


      ConstPointInTPtr model_cloud = verified_models->at(j)->getAssembled (0.01);
      typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
      pcl::transformPointCloud (*model_cloud, *model_aligned, verified_transforms->at(j));
      *pRecognizedModels += *model_aligned;
    }
    sensor_msgs::PointCloud2 recognizedModelsRos;
    pcl::toROSMsg (*pRecognizedModels, recognizedModelsRos);
    recognizedModelsRos.header.frame_id = "camera_rgb_optical_frame";
    vis_pc_pub_.publish(recognizedModelsRos);

    if(req.complex_result.data)
    {
        //fill the rest of the output

/*
#ratio of visible points
float32[] confidence

# centroid of the cluster
geometry_msgs/Point32[] centroid

# bounding box of the cluster
object_perception_msgs/BBox[] bbox

# point cloud of the model transformed into camera coordinates
sensor_msgs/PointCloud2[] cloud
*/

        float res = 0.005f;

        for (size_t j = 0; j < verified_models->size (); j++)
        {
            ConstPointInTPtr model_cloud = verified_models->at(j)->getAssembled (res);
            typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
            pcl::transformPointCloud (*model_cloud, *model_aligned, verified_transforms->at(j));

            pcl::PointCloud<pcl::Normal>::ConstPtr normal_cloud = verified_models->at(j)->getNormalsAssembled (res);

            typename pcl::PointCloud<pcl::Normal>::Ptr normal_aligned (new pcl::PointCloud<pcl::Normal>);
            v4r::ORUtils::miscellaneous::transformNormals(normal_cloud, normal_aligned, transforms->at (j));

            //ratio of inlier points
            float confidence = 0;

            v4r::registration::VisibilityReasoning<pcl::PointXYZRGB> vr (525.f, 640, 480);
            vr.setThresholdTSS (0.01f);
            /*float fsv_ratio =*/ vr.computeFSVWithNormals (scene, model_aligned, normal_aligned);
            int ss = vr.getFSVUsedPoints();
            confidence = 1.f - ss / static_cast<float>(model_aligned->points.size());

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

            sensor_msgs::PointCloud2 model_msg;
            pcl::toROSMsg(*model_aligned, model_msg);

            response.models_cloud.push_back(model_msg);
        }

        if(debug_publish_)
            createDebugImageAndPublish(response, scene);

    }

#ifdef SOC_VISUALIZE
    vis_->spin ();
#endif

    return true;
  }
public:
  Recognizer ()
  {
    //default values
    chop_at_z_ = 1.5;
    do_sift_ = true;
    do_shot_ = false;
    do_ourcvfh_ = false;
    icp_iterations_ = 0;
    cg_size_ = 3;
    idx_flann_fn_sift_ = "sift_flann.idx";
    idx_flann_fn_shot_ = "shot_flann.idx";
    num_recorded_clouds_ = 0;

#ifdef SOC_VISUALIZE
    vis_.reset (new pcl::visualization::PCLVisualizer ("classifier visualization"));
    vis_->createViewPort(0,0,0.33,1.f, v1_);
    vis_->createViewPort(0.33,0,0.66,1.f, v2_);
    vis_->createViewPort(0.66,0,1,1.f, v3_);
#endif
  }

  void
  initialize (int argc, char ** argv)
  {

      n_.reset( new ros::NodeHandle ( "~" ) );
      n_->getParam ( "models_dir", models_dir_);
      n_->getParam ( "training_dir_sift", training_dir_sift_);
      n_->getParam ( "training_dir_shot", training_dir_shot_);
      n_->getParam ( "recognizer_structure_sift", sift_structure_);
      n_->getParam ( "training_dir_ourcvfh", training_dir_ourcvfh_);
      n_->getParam ( "chop_z", chop_at_z_ );
      n_->getParam ( "icp_iterations", icp_iterations_);
      n_->getParam ( "do_sift", do_sift_);
      n_->getParam ( "do_shot", do_shot_);
      n_->getParam ( "do_ourcvfh", do_ourcvfh_);
      n_->getParam ( "ignore_color", ignore_color_);
      n_->getParam ( "cg_size", cg_size_);
      n_->getParam ( "knn_sift", knn_sift_);
      n_->getParam ( "use_cg_graph", use_cg_graph_);
      n_->getParam ( "publish_debug", debug_publish_);

      std::cout << chop_at_z_ << ", " << ignore_color_ << ", do_shot:" << do_shot_ << std::endl;
    if (models_dir_.compare ("") == 0)
    {
      PCL_ERROR ("Set -models_dir option in the command line, ABORTING");
      return;
    }

    if (do_sift_ && training_dir_sift_.compare ("") == 0)
    {
      PCL_ERROR ("do_sift is activated but training_dir_sift_ is empty! Set -training_dir_sift option in the command line, ABORTING");
      return;
    }

    if (do_ourcvfh_ && training_dir_ourcvfh_.compare ("") == 0)
    {
      PCL_ERROR ("do_ourcvfh is activated but training_dir_ourcvfh_ is empty! Set -training_dir_ourcvfh option in the command line, ABORTING");
      return;
    }

    if (do_shot_ && training_dir_shot_.compare ("") == 0)
    {
      PCL_ERROR ("do_shot is activated but training_dir_shot_ is empty! Set -training_dir_shot option in the command line, ABORTING");
      return;
    }

    boost::function<bool (const Eigen::Vector3f &)> campos_constraints;
    campos_constraints = camPosConstraints ();

    multi_recog_.reset (new faat_pcl::rec_3d_framework::MultiRecognitionPipeline<PointT>);
    boost::shared_ptr <faat_pcl::CorrespondenceGrouping<PointT, PointT> > cast_cg_alg;
    boost::shared_ptr < faat_pcl::GraphGeometricConsistencyGrouping<PointT, PointT> > gcg_alg (
                                                                                               new faat_pcl::GraphGeometricConsistencyGrouping<
                                                                                                   PointT, PointT>);

    gcg_alg->setGCThreshold (cg_size_);
    gcg_alg->setGCSize (0.015);
    gcg_alg->setRansacThreshold (0.015);
    gcg_alg->setUseGraph (use_cg_graph_);
    gcg_alg->setDistForClusterFactor (0);
    gcg_alg->setMaxTaken(2);
    gcg_alg->setMaxTimeForCliquesComputation(100);
    gcg_alg->setDotDistance (0.2);
    cast_cg_alg = boost::static_pointer_cast<faat_pcl::CorrespondenceGrouping<PointT, PointT> > (gcg_alg);

    if (do_sift_)
    {

      std::string desc_name = "sift";

      boost::shared_ptr < faat_pcl::rec_3d_framework::RegisteredViewsSource<pcl::PointXYZRGBNormal, PointT, PointT>
          > mesh_source (new faat_pcl::rec_3d_framework::RegisteredViewsSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB, pcl::PointXYZRGB>);
      mesh_source->setPath (models_dir_);
      mesh_source->setModelStructureDir (sift_structure_);
      mesh_source->setLoadViews (false);
      mesh_source->generate (training_dir_sift_);

      boost::shared_ptr < faat_pcl::rec_3d_framework::Source<PointT> > cast_source;
      cast_source = boost::static_pointer_cast<faat_pcl::rec_3d_framework::RegisteredViewsSource<pcl::PointXYZRGBNormal, PointT, PointT> > (mesh_source);

#ifdef USE_SIFT_GPU
      boost::shared_ptr < faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, pcl::Histogram<128> > > estimator;
      estimator.reset (new faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, pcl::Histogram<128> >);

      boost::shared_ptr < faat_pcl::rec_3d_framework::LocalEstimator<PointT, pcl::Histogram<128> > > cast_estimator;
      cast_estimator = boost::dynamic_pointer_cast<faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, pcl::Histogram<128> > > (estimator);
#else
      boost::shared_ptr < faat_pcl::rec_3d_framework::OpenCVSIFTLocalEstimation<PointT, pcl::Histogram<128> > > estimator;
      estimator.reset (new faat_pcl::rec_3d_framework::OpenCVSIFTLocalEstimation<PointT, pcl::Histogram<128> >);

      boost::shared_ptr < faat_pcl::rec_3d_framework::LocalEstimator<PointT, pcl::Histogram<128> > > cast_estimator;
      cast_estimator = boost::dynamic_pointer_cast<faat_pcl::rec_3d_framework::OpenCVSIFTLocalEstimation<PointT, pcl::Histogram<128> > > (estimator);
#endif

      boost::shared_ptr<faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<128> > > new_sift_local_;
      new_sift_local_.reset (new faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<128> > (idx_flann_fn_sift_));
      new_sift_local_->setDataSource (cast_source);
      new_sift_local_->setTrainingDir (training_dir_sift_);
      new_sift_local_->setDescriptorName (desc_name);
      new_sift_local_->setICPIterations (0);
      new_sift_local_->setFeatureEstimator (cast_estimator);
      new_sift_local_->setUseCache (true);
      new_sift_local_->setCGAlgorithm (cast_cg_alg);
      new_sift_local_->setKnn (knn_sift_);
      new_sift_local_->setUseCache (true);
      new_sift_local_->initialize (false);

      boost::shared_ptr < faat_pcl::rec_3d_framework::Recognizer<PointT> > cast_recog;
      cast_recog = boost::static_pointer_cast<faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<128> > > (
                                                                                                                                        new_sift_local_);
      multi_recog_->addRecognizer (cast_recog);
    }

    if(do_shot_)
    {
        std::cout << "do_shot_ is activated, should do something..." << std::endl;
        boost::shared_ptr<faat_pcl::rec_3d_framework::UniformSamplingExtractor<PointT> > uniform_keypoint_extractor ( new faat_pcl::rec_3d_framework::UniformSamplingExtractor<PointT>);
        uniform_keypoint_extractor->setSamplingDensity (0.01f);
        uniform_keypoint_extractor->setFilterPlanar (true);
        uniform_keypoint_extractor->setThresholdPlanar(0.05);
        uniform_keypoint_extractor->setMaxDistance(chop_at_z_);

        boost::shared_ptr<faat_pcl::rec_3d_framework::KeypointExtractor<PointT> > keypoint_extractor;
        keypoint_extractor = boost::static_pointer_cast<faat_pcl::rec_3d_framework::KeypointExtractor<PointT> > (uniform_keypoint_extractor);

        boost::shared_ptr<faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<PointT, pcl::Normal> > normal_estimator;
        normal_estimator.reset (new faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<PointT, pcl::Normal>);
        normal_estimator->setCMR (false);
        normal_estimator->setDoVoxelGrid (true);
        normal_estimator->setRemoveOutliers (true);
        normal_estimator->setValuesForCMRFalse (0.003f, 0.02f);

        std::string desc_name("shot");
        boost::shared_ptr<faat_pcl::rec_3d_framework::SHOTLocalEstimationOMP<PointT, pcl::Histogram<352> > > estimator;
        estimator.reset (new faat_pcl::rec_3d_framework::SHOTLocalEstimationOMP<PointT, pcl::Histogram<352> >);
        estimator->setNormalEstimator (normal_estimator);
        estimator->addKeypointExtractor (keypoint_extractor);
        estimator->setSupportRadius (0.04);
        estimator->setAdaptativeMLS (false);

        boost::shared_ptr<faat_pcl::rec_3d_framework::LocalEstimator<PointT, pcl::Histogram<352> > > cast_estimator;
        cast_estimator = boost::dynamic_pointer_cast<faat_pcl::rec_3d_framework::LocalEstimator<PointT, pcl::Histogram<352> > > (estimator);

        boost::shared_ptr<faat_pcl::rec_3d_framework::RegisteredViewsSource<pcl::PointXYZRGBNormal, PointT, PointT> >
                source (
                    new faat_pcl::rec_3d_framework::RegisteredViewsSource<
                    pcl::PointXYZRGBNormal,
                    pcl::PointXYZRGB,
                    pcl::PointXYZRGB>);
        source->setPath (models_dir_);
        source->setModelStructureDir (sift_structure_);
        source->generate (training_dir_shot_);

        boost::shared_ptr<faat_pcl::rec_3d_framework::Source<pcl::PointXYZRGB> > cast_source;
        cast_source = boost::static_pointer_cast<faat_pcl::rec_3d_framework::RegisteredViewsSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB> > (source);

        boost::shared_ptr<faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<352> > > local;
        local.reset(new faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<352> > (idx_flann_fn_shot_));
        local->setDataSource (cast_source);
        local->setTrainingDir (training_dir_shot_);
        local->setDescriptorName (desc_name);
        local->setFeatureEstimator (cast_estimator);
        local->setCGAlgorithm (cast_cg_alg);
        local->setKnn(1);
        local->setUseCache (static_cast<bool> (true));
        uniform_keypoint_extractor->setSamplingDensity (0.01f);
        local->setICPIterations (0);
        local->setKdtreeSplits (512);
        local->setICPType(0);
        local->setUseCodebook(false);
        local->initialize (static_cast<bool> (false));

        boost::shared_ptr < faat_pcl::rec_3d_framework::Recognizer<PointT> > cast_recog =
            boost::static_pointer_cast<faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<352> > > (local);

        multi_recog_->addRecognizer(cast_recog);
    }

    if(do_ourcvfh_ && USE_SEGMENTATION_)
    {
      boost::shared_ptr<faat_pcl::rec_3d_framework::PartialPCDSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB> >
                          source (
                              new faat_pcl::rec_3d_framework::PartialPCDSource<
                              pcl::PointXYZRGBNormal,
                              pcl::PointXYZRGB>);
      source->setPath (models_dir_);
      source->setModelScale (1.f);
      source->setRadiusSphere (1.f);
      source->setTesselationLevel (1);
      source->setDotNormal (-1.f);
      source->setUseVertices(false);
      source->setLoadViews (false);
      source->setCamPosConstraints (campos_constraints);
      source->setLoadIntoMemory(false);
      source->setGenOrganized(true);
      source->setWindowSizeAndFocalLength(640, 480, 575.f);
      source->generate (training_dir_ourcvfh_);

      boost::shared_ptr<faat_pcl::rec_3d_framework::Source<pcl::PointXYZRGB> > cast_source;
      cast_source = boost::static_pointer_cast<faat_pcl::rec_3d_framework::PartialPCDSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB> > (source);

      //configure normal estimator
      boost::shared_ptr<faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<PointT, pcl::Normal> > normal_estimator;
      normal_estimator.reset (new faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<PointT, pcl::Normal>);
      normal_estimator->setCMR (false);
      normal_estimator->setDoVoxelGrid (false);
      normal_estimator->setRemoveOutliers (false);
      normal_estimator->setValuesForCMRFalse (0.001f, 0.02f);
      normal_estimator->setForceUnorganized(true);

      //boost::shared_ptr<faat_pcl::rec_3d_framework::ColorOURCVFHEstimator<PointT, pcl::Histogram<1327> > > vfh_estimator;
      //vfh_estimator.reset (new faat_pcl::rec_3d_framework::ColorOURCVFHEstimator<PointT, pcl::Histogram<1327> >);

      boost::shared_ptr<faat_pcl::rec_3d_framework::OrganizedColorOURCVFHEstimator<PointT, pcl::Histogram<1327> > > vfh_estimator;
      vfh_estimator.reset (new faat_pcl::rec_3d_framework::OrganizedColorOURCVFHEstimator<PointT, pcl::Histogram<1327> >);
      vfh_estimator->setNormalEstimator (normal_estimator);
      vfh_estimator->setNormalizeBins(true);
      vfh_estimator->setUseRFForColor (true);
      //vfh_estimator->setRefineClustersParam (2.5f);
      vfh_estimator->setRefineClustersParam (100.f);
      vfh_estimator->setAdaptativeMLS (false);

      vfh_estimator->setAxisRatio (1.f);
      vfh_estimator->setMinAxisValue (1.f);

      {
          //segmentation parameters for training
          std::vector<float> eps_thresholds, cur_thresholds, clus_thresholds;
          eps_thresholds.push_back (0.15);
          cur_thresholds.push_back (0.015f);
          cur_thresholds.push_back (1.f);
          clus_thresholds.push_back (10.f);

          vfh_estimator->setClusterToleranceVector (clus_thresholds);
          vfh_estimator->setEpsAngleThresholdVector (eps_thresholds);
          vfh_estimator->setCurvatureThresholdVector (cur_thresholds);
      }

      std::string desc_name = "rf_our_cvfh_color_normalized";

      boost::shared_ptr<faat_pcl::rec_3d_framework::OURCVFHEstimator<pcl::PointXYZRGB, pcl::Histogram<1327> > > cast_estimator;
      cast_estimator = boost::dynamic_pointer_cast<faat_pcl::rec_3d_framework::OrganizedColorOURCVFHEstimator<pcl::PointXYZRGB, pcl::Histogram<1327> > > (vfh_estimator);

      boost::shared_ptr<faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<faat_pcl::Metrics::HistIntersectionUnionDistance, PointT, pcl::Histogram<1327> > > rf_color_ourcvfh_global_;
      rf_color_ourcvfh_global_.reset(new faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<faat_pcl::Metrics::HistIntersectionUnionDistance, PointT, pcl::Histogram<1327> >);
      rf_color_ourcvfh_global_->setDataSource (cast_source);
      rf_color_ourcvfh_global_->setTrainingDir (training_dir_ourcvfh_);
      rf_color_ourcvfh_global_->setDescriptorName (desc_name);
      rf_color_ourcvfh_global_->setFeatureEstimator (cast_estimator);
      rf_color_ourcvfh_global_->setNN (50);
      rf_color_ourcvfh_global_->setICPIterations (0);
      rf_color_ourcvfh_global_->setNoise (0.0f);
      rf_color_ourcvfh_global_->setUseCache (true);
      rf_color_ourcvfh_global_->setMaxHyp(15);
      rf_color_ourcvfh_global_->setMaxDescDistance(0.75f);
      rf_color_ourcvfh_global_->initialize (false);
      rf_color_ourcvfh_global_->setDebugLevel(2);
      {
          //segmentation parameters for recognition
          std::vector<float> eps_thresholds, cur_thresholds, clus_thresholds;
          eps_thresholds.push_back (0.15);
          cur_thresholds.push_back (0.015f);
          cur_thresholds.push_back (0.02f);
          cur_thresholds.push_back (1.f);
          clus_thresholds.push_back (10.f);

          vfh_estimator->setClusterToleranceVector (clus_thresholds);
          vfh_estimator->setEpsAngleThresholdVector (eps_thresholds);
          vfh_estimator->setCurvatureThresholdVector (cur_thresholds);

          vfh_estimator->setAxisRatio (0.8f);
          vfh_estimator->setMinAxisValue (0.8f);

          vfh_estimator->setAdaptativeMLS (false);
      }

      boost::shared_ptr < faat_pcl::rec_3d_framework::Recognizer<PointT> > cast_recog;
      cast_recog = boost::static_pointer_cast<faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<faat_pcl::Metrics::HistIntersectionUnionDistance, PointT, pcl::Histogram<1327> > > (rf_color_ourcvfh_global_);
      multi_recog_->addRecognizer(cast_recog);
    }

    multi_recog_->setCGAlgorithm(gcg_alg);
    multi_recog_->setVoxelSizeICP(0.005f);
    multi_recog_->setICPType(1);
    multi_recog_->setICPIterations(icp_iterations_);
    multi_recog_->initialize();

    recognize_  = n_->advertiseService ("mp_recognition", &Recognizer::recognize, this);
    retrain_recognizer_  = n_->advertiseService ("mp_recognition_retrain", &Recognizer::retrain, this);
    get_configuration_  = n_->advertiseService ("get_configuration", &Recognizer::getConfig, this);

    vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "sv_recogniced_object_instances_", 1 );

    it_.reset(new image_transport::ImageTransport(*n_));

    if(debug_publish_)
        image_pub_ = it_->advertise("/mp_recognition/debug_image", 1, true);

    std::cout << "Ready to get service calls..." << std::endl;
    ros::spin ();
  }
};

int
main (int argc, char ** argv)
{
  ros::init (argc, argv, "recognition_service");

  Recognizer m;
  m.initialize (argc, argv);

  return 0;
}
