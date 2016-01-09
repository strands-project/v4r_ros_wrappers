/*
 * esf_classifier
 *
 *  Created on: Nov, 2015
 *      Author: Thomas Faeulhammer
 */


#include <sstream>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include <Eigen/Eigenvalues>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <v4r/recognition/mesh_source.h>
#include <v4r/features/esf_estimator.h>
#include <v4r/recognition/metrics.h>

#include "global_nn_classifier_ros.h"


namespace v4r
{

template<template<class > class Distance, typename PointT>
bool
GlobalNNClassifierROS<Distance,PointT>::classifyROS (classifier_srv_definitions::classify::Request &req, classifier_srv_definitions::classify::Response &response)
{
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(req.cloud, *cloud);
    this->input_ = cloud;

    //clear all data from previous visualization steps and publish empty markers/point cloud
    for (size_t i=0; i < markerArray_.markers.size(); i++)
        markerArray_.markers[i].action = visualization_msgs::Marker::DELETE;

    vis_pub_.publish( markerArray_ );
    sensor_msgs::PointCloud2 scenePc2;
    vis_pc_pub_.publish(scenePc2);
    markerArray_.markers.clear();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *cluster);

    for(size_t cluster_id=0; cluster_id < req.clusters_indices.size(); cluster_id++)
    {
        const float r = std::rand() % 255;
        const float g = std::rand() % 255;
        const float b = std::rand() % 255;

        this->indices_.resize(req.clusters_indices[cluster_id].data.size());

        for(size_t kk=0; kk < req.clusters_indices[cluster_id].data.size(); kk++)
        {
            this->indices_[kk] = static_cast<int>(req.clusters_indices[cluster_id].data[kk]);
            cluster->at(req.clusters_indices[cluster_id].data[kk]).r = 0.8*r;
            cluster->at(req.clusters_indices[cluster_id].data[kk]).g = 0.8*g;
            cluster->at(req.clusters_indices[cluster_id].data[kk]).b = 0.8*b;
        }

        this->classify ();

        std::cout << "for cluster " << cluster_id << " with size " << this->indices_.size() << ", I have following hypotheses: " << std::endl;

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

        Eigen::Vector4f centroid;
        Eigen::Matrix3f covariance_matrix;
        typename pcl::PointCloud<PointT>::Ptr pClusterPCl_transformed (new pcl::PointCloud<PointT>());
        pcl::computeMeanAndCovarianceMatrix(*this->input_, this->indices_, covariance_matrix, centroid);
        Eigen::Matrix3f eigvects;
        Eigen::Vector3f eigvals;
        pcl::eigen33(covariance_matrix, eigvects,  eigvals);

        Eigen::Vector3f centroid_transformed = eigvects.transpose() * centroid.topRows(3);

        Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Zero(4,4);
        transformation_matrix.block<3,3>(0,0) = eigvects.transpose();
        transformation_matrix.block<3,1>(0,3) = -centroid_transformed;
        transformation_matrix(3,3) = 1;

        pcl::transformPointCloud(*this->input_, this->indices_, *pClusterPCl_transformed, transformation_matrix);

        //pcl::transformPointCloud(*frame_, cluster_indices_int, *frame_eigencoordinates_, eigvects);
        PointT min_pt, max_pt;
        pcl::getMinMax3D(*pClusterPCl_transformed, min_pt, max_pt);
        std::cout << "Elongations along eigenvectors: "
                  << max_pt.x - min_pt.x << ", " << max_pt.y - min_pt.y << ", " << max_pt.z - min_pt.z << std::endl;

        geometry_msgs::Point32 centroid_ros;
        centroid_ros.x = centroid[0];
        centroid_ros.y = centroid[1];
        centroid_ros.z = centroid[2];
        response.centroid.push_back(centroid_ros);

        // calculating the bounding box of the cluster
        Eigen::Vector4f min;
        Eigen::Vector4f max;
        pcl::getMinMax3D (*this->input_, this->indices_, min, max);

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
        pcl::copyPointCloud(*this->input_, this->indices_, *cluster);

        sensor_msgs::PointCloud2  pc2;
        pcl::toROSMsg (*cluster, pc2);
        response.cloud.push_back(pc2);

        // visualize the result as ROS topic
        visualization_msgs::Marker marker;
        marker.header.frame_id = req.cloud.header.frame_id;
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

    pcl::toROSMsg (*cluster, scenePc2);
    vis_pc_pub_.publish(scenePc2);
    vis_pub_.publish( markerArray_ );

    response.clusters_indices = req.clusters_indices;
    return true;
}

template<template<class > class Distance, typename PointT>
void GlobalNNClassifierROS<Distance, PointT>::initializeROS(int argc, char ** argv)
{
    //    PCLSegmenter<pcl::PointXYZRGB>::Parameter seg_param;
    ros::init (argc, argv, "classifier_service");
    n_.reset( new ros::NodeHandle ( "~" ) );
    n_->getParam ( "chop_z", chop_at_z_ );
    //    n_->getParam ( "chop_z", seg_param.chop_at_z_ );
    //    n_->getParam ( "seg_type", seg_param.seg_type_ );
    //    n_->getParam ( "min_cluster_size", seg_param.min_cluster_size_ );
    //    n_->getParam ( "max_vertical_plane_size", seg_param.max_vertical_plane_size_ );
    //    n_->getParam ( "num_plane_inliers", seg_param.num_plane_inliers_ );
    //    n_->getParam ( "max_angle_plane_to_ground", seg_param.max_angle_plane_to_ground_ );
    //    n_->getParam ( "sensor_noise_max", seg_param.sensor_noise_max_ );
    //    n_->getParam ( "table_range_min", seg_param.table_range_min_ );
    //    n_->getParam ( "table_range_max", seg_param.table_range_max_ );
    //    n_->getParam ( "angular_threshold_deg", seg_param.angular_threshold_deg_ );

    ROS_INFO("models_dir, training dir:  %s, %s",  models_dir_.c_str(), this->training_dir_.c_str());

    if(!n_->getParam ( "models_dir", models_dir_ ))
    {
        PCL_ERROR("Set -models_dir option in the command line, ABORTING");
        return;
    }

    if(!n_->getParam ( "training_dir", this->training_dir_ ))
    {
        PCL_ERROR("Set -training_dir option in the command line, ABORTING");
        return;
    }

    int nn = static_cast<int>(this->NN_);
    n_->getParam ( "nn", nn);
    this->NN_ = static_cast<size_t>(nn);


    //  seg_.reset( new PCLSegmenter<pcl::PointXYZRGB>(seg_param));

    boost::shared_ptr<MeshSource<PointT> > mesh_source (new MeshSource<PointT>);
    mesh_source->setPath (models_dir_);
    mesh_source->setMeshDir(this->training_dir_);
    mesh_source->setResolution (150);
    mesh_source->setTesselationLevel (0);
    //  mesh_source->setViewAngle (57.f);
    mesh_source->setRadiusSphere (1.f);
    mesh_source->setModelScale (1.f);
    mesh_source->generate ();

    this->source_ = boost::static_pointer_cast<MeshSource<PointT> > (mesh_source);

    boost::shared_ptr<ESFEstimation<PointT> > estimator;
    estimator.reset (new ESFEstimation<PointT>);

    boost::shared_ptr<GlobalEstimator<PointT> > cast_estimator;
    cast_estimator = boost::dynamic_pointer_cast<ESFEstimation<PointT> > (estimator);

    this->setDescriptorName("esf");
    this->setFeatureEstimator (cast_estimator);
    this->initialize (false);

    //  segment_and_classify_service_ = n_->advertiseService("segment_and_classify", ShapeClassifier::segmentAndClassify, this);
    classify_service_ = n_->advertiseService("classify", &GlobalNNClassifierROS<Distance,PointT>::classifyROS, this);
    vis_pub_ = n_->advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );
    vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "clusters", 1 );
    ros::spin();
}

}

int
main (int argc, char ** argv)
{
    v4r::GlobalNNClassifierROS<flann::L1, pcl::PointXYZ> ros_classifier;
    ros_classifier.initializeROS (argc, argv);

    return 0;
}
