/*
 * shape_simple_classifier_node.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: aitor
 */


#include <sstream>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include <Eigen/Eigenvalues>
#include <pcl/apps/dominant_plane_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <v4r/recognition/mesh_source.h>
#include <v4r/features/vfh_estimator.h>
#include <v4r/features/esf_estimator.h>
#include <v4r/features/cvfh_estimator.h>
#include <v4r/recognition/metrics.h>

#include "global_nn_classifier_ros.h"

//class ShapeClassifier
//{
//  private:
//    typedef pcl::PointXYZ PointT;
//    std::string models_dir_;
//    std::string training_dir_;
//    std::string desc_name_;
//    int NN_;
//    double chop_at_z_;
//    pcl::PointCloud<PointT>::Ptr frame_;
//    std::vector<pcl::PointIndices> cluster_indices_;
//    std::vector < std::string > categories_;
//    std::vector<float> conf_;

//    boost::shared_ptr<v4r::GlobalNNPipeline<flann::L1, PointT, pcl::ESFSignature640> > classifier_;
//    ros::ServiceServer segment_and_classify_service_;
//    ros::ServiceServer classify_service_;
//    ros::NodeHandle *n_;
//    ros::Publisher vis_pub_, vis_pc_pub_;
//    visualization_msgs::MarkerArray markerArray_;
//    std::string camera_frame_;

//    bool segmentAndClassify(classifier_srv_definitions::segment_and_classify::Request & req,
//                               classifier_srv_definitions::segment_and_classify::Response & response)
//    {
//      //------- Segmentation------------------
//      /*Eigen::Vector4f table_plane;
//      doSegmentation<PointT>(frame_, cluster_indices_, table_plane, chop_at_z_);*/
//      //std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
//      /*
//      float tolerance = 0.005f;
//      pcl::apps::DominantPlaneSegmentation<pcl::PointXYZ> dps;
//      dps.setInputCloud (frame_);
//      dps.setMaxZBounds (chop_at_z_);
//      dps.setObjectMinHeight (tolerance);
//      dps.setMinClusterSize (1000);
//      dps.setWSize (9);
//      dps.setDistanceBetweenClusters (0.1f);

//      dps.setDownsamplingSize (0.01f);
//      dps.compute_fast (clusters);
//      dps.getIndicesClusters (cluster_indices_);
//      Eigen::Vector4f table_plane;
//      dps.getTableCoefficients (table_plane);*/
//      //-------------------------------------

//      ros::ServiceClient segmentation_client = n_->serviceClient<segmentation_srv_definitions::segment>("/object_segmenter_service/object_segmenter");
//      segmentation_srv_definitions::segment seg_srv;
//      seg_srv.request.cloud = req.cloud;
//      seg_srv.request.transform = req.transform;
//      if (segmentation_client.call(seg_srv))
//      {
//          std::cout << "Number of clusters:" << static_cast<int>(seg_srv.response.clusters_indices.size()) << std::endl;
//          classifier_srv_definitions::classify srv;
//          srv.request.cloud = req.cloud;
//          srv.request.clusters_indices = seg_srv.response.clusters_indices;
//          classify(srv.request, srv.response);
//          response.class_results = srv.response.class_results;
//          response.clusters_indices = srv.response.clusters_indices;
//          response.centroid = srv.response.centroid;
//          response.bbox = srv.response.bbox;
//          response.cloud = srv.response.cloud;
//      }
//      else
//      {
//          ROS_ERROR("Failed to call segmentation service.");
//          return false;
//      }
//      return true;
//    }

//  public:
//    ShapeClassifier()
//    {
//      //default values
//      desc_name_ = "esf";
//      NN_ = 50;
//      chop_at_z_ = 1.f;
//      frame_.reset(new pcl::PointCloud<PointT>());
//    }
//};

namespace v4r
{

template<template<class > class Distance, typename PointT, typename FeatureT> bool
GlobalNNPipelineROS<Distance,PointT,FeatureT>::classifyROS (classifier_srv_definitions::classify::Request & req, classifier_srv_definitions::classify::Response & response)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>());
        pcl::fromROSMsg(req.cloud, *cloud);
        this->input_ = cloud;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pClusteredPCl (new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::copyPointCloud(*cloud, *pClusteredPCl);

        //clear all data from previous visualization steps and publish empty markers/point cloud
        for (size_t i=0; i < markerArray_.markers.size(); i++)
        {
            markerArray_.markers[i].action = visualization_msgs::Marker::DELETE;
        }
        vis_pub_.publish( markerArray_ );
        sensor_msgs::PointCloud2 scenePc2;
        vis_pc_pub_.publish(scenePc2);
        markerArray_.markers.clear();

        for(size_t cluster_id=0; cluster_id < req.clusters_indices.size(); cluster_id++)
        {
          std::vector<int> cluster_indices_int;
          Eigen::Vector4f centroid;
          Eigen::Matrix3f covariance_matrix;

          const float r = std::rand() % 255;
          const float g = std::rand() % 255;
          const float b = std::rand() % 255;

          this->indices_.resize(req.clusters_indices[cluster_id].data.size());
          for(size_t kk=0; kk < req.clusters_indices[cluster_id].data.size(); kk++)
          {
                this->indices_[kk] = static_cast<int>(req.clusters_indices[cluster_id].data[kk]);
                pClusteredPCl->at(req.clusters_indices[cluster_id].data[kk]).r = 0.8*r;
                pClusteredPCl->at(req.clusters_indices[cluster_id].data[kk]).g = 0.8*g;
                pClusteredPCl->at(req.clusters_indices[cluster_id].data[kk]).b = 0.8*b;
          }

          this->classify ();

          std::cout << "for cluster " << cluster_id << " with size " << cluster_indices_int.size() << ", I have following hypotheses: " << std::endl;

          object_perception_msgs::classification class_tmp;
          for(size_t kk=0; kk < this->categories_.size(); kk++)
          {
            std::cout << this->categories_[kk] << " with confidence " << this->confidences_[kk] << std::endl;
            std_msgs::String str_tmp;
            str_tmp.data = this->categories_[kk];
            class_tmp.class_type.push_back(str_tmp);
            class_tmp.confidence.push_back( this->confidences_[kk] );
          }
          response.class_results.push_back(class_tmp);

          typename pcl::PointCloud<PointT>::Ptr pClusterPCl_transformed (new pcl::PointCloud<PointT>());
          pcl::computeMeanAndCovarianceMatrix(*this->input_, cluster_indices_int, covariance_matrix, centroid);
          Eigen::Matrix3f eigvects;
          Eigen::Vector3f eigvals;
          pcl::eigen33(covariance_matrix, eigvects,  eigvals);

          Eigen::Vector3f centroid_transformed = eigvects.transpose() * centroid.topRows(3);

          Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Zero(4,4);
          transformation_matrix.block<3,3>(0,0) = eigvects.transpose();
          transformation_matrix.block<3,1>(0,3) = -centroid_transformed;
          transformation_matrix(3,3) = 1;

          pcl::transformPointCloud(*this->input_, cluster_indices_int, *pClusterPCl_transformed, transformation_matrix);

          //pcl::transformPointCloud(*frame_, cluster_indices_int, *frame_eigencoordinates_, eigvects);
          PointT min_pt, max_pt;
          pcl::getMinMax3D(*pClusterPCl_transformed, min_pt, max_pt);
          std::cout << "Elongations along eigenvectors: " << max_pt.x - min_pt.x << ", " << max_pt.y - min_pt.y
                    << ", " << max_pt.z - min_pt.z << std::endl;
          geometry_msgs::Point32 centroid_ros;
          centroid_ros.x = centroid[0];
          centroid_ros.y = centroid[1];
          centroid_ros.z = centroid[2];
          response.centroid.push_back(centroid_ros);

          // calculating the bounding box of the cluster
          Eigen::Vector4f min;
          Eigen::Vector4f max;
          pcl::getMinMax3D (*this->input_, cluster_indices_int, min, max);

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

          // getting the point cloud of the cluster
          typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
          pcl::copyPointCloud(*this->input_, cluster_indices_int, *cluster);

          sensor_msgs::PointCloud2  pc2;
          pcl::toROSMsg (*cluster, pc2);
          response.cloud.push_back(pc2);

          // visualize the result as ROS topic
            visualization_msgs::Marker marker;
            marker.header.frame_id = camera_frame_;
            marker.header.stamp = ros::Time::now();
            //marker.header.seq = ++marker_seq_;
            marker.ns = "object_classification";
            marker.id = cluster_id;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = centroid_ros.x;
            marker.pose.position.y = centroid_ros.y - 0.1;
            marker.pose.position.z = centroid_ros.z - 0.1;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 1.0;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = r/255.f;
            marker.color.g = g/255.f;
            marker.color.b = b/255.f;
            std::stringstream marker_text;
            marker_text.precision(2);
            marker_text << this->categories_[0] << this->confidences_[0];
            marker.text = marker_text.str();
            markerArray_.markers.push_back(marker);
        }

        pcl::toROSMsg (*pClusteredPCl, scenePc2);
        vis_pc_pub_.publish(scenePc2);
        vis_pub_.publish( markerArray_ );

        response.clusters_indices = req.clusters_indices;
        return true;
    }

template<template<class > class Distance, typename PointT, typename FeatureT>
void GlobalNNPipelineROS<Distance,PointT,FeatureT>::initializeROS(int argc, char ** argv)
{
    ros::init (argc, argv, "classifier_service");
    n_.reset( new ros::NodeHandle ( "~" ) );
    n_->getParam ( "models_dir", models_dir_ );
    n_->getParam ( "training_dir", this->training_dir_ );
    n_->getParam ( "descriptor_name", this->descr_name_ );
    n_->getParam ( "nn", this->NN_ );
    n_->getParam ( "chop_z", chop_at_z_ );

    if(!n_->getParam ( "camera_frame", camera_frame_ ))
        camera_frame_ = "/head_xtion_depth_optical_frame";

    ROS_INFO("models_dir, training dir, desc, camera_frame:  %s, %s, %s, %s",  models_dir_.c_str(), this->training_dir_.c_str(), this->descr_name_.c_str(), camera_frame_.c_str());

  if(models_dir_.compare("") == 0)
  {
    PCL_ERROR("Set -models_dir option in the command line, ABORTING");
    return;
  }

  if(this->training_dir_.compare("") == 0)
  {
    PCL_ERROR("Set -training_dir option in the command line, ABORTING");
    return;
  }

  boost::shared_ptr<MeshSource<pcl::PointXYZ> > mesh_source (new MeshSource<pcl::PointXYZ>);
  mesh_source->setPath (models_dir_);
  mesh_source->setResolution (150);
  mesh_source->setTesselationLevel (0);
  mesh_source->setViewAngle (57.f);
  mesh_source->setRadiusSphere (1.f);
  mesh_source->setModelScale (1.f);
  mesh_source->generate (this->training_dir_);

  this->source_ = boost::static_pointer_cast<v4r::MeshSource<pcl::PointXYZ> > (mesh_source);

  boost::shared_ptr<v4r::ESFEstimation<PointT, pcl::ESFSignature640> > estimator;
  estimator.reset (new v4r::ESFEstimation<PointT, pcl::ESFSignature640>);

  boost::shared_ptr<v4r::GlobalEstimator<PointT, pcl::ESFSignature640> > cast_estimator;
  cast_estimator = boost::dynamic_pointer_cast<v4r::ESFEstimation<PointT, pcl::ESFSignature640> > (estimator);

  this->setFeatureEstimator (cast_estimator);
  this->initialize (false);

//  segment_and_classify_service_ = n_->advertiseService("segment_and_classify", &ShapeClassifier::segmentAndClassify, this);
  classify_service_ = n_->advertiseService("classify", &v4r::GlobalNNPipelineROS<Distance,PointT,FeatureT>::classifyROS, this);
  vis_pub_ = n_->advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );
  vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "clusters", 1 );
  ros::spin();
}

}

int
main (int argc, char ** argv)
{
  v4r::GlobalNNPipelineROS<flann::L1, pcl::PointXYZ, pcl::ESFSignature640> ros_classifier;
  ros_classifier.initializeROS (argc, argv);

  return 0;
}
