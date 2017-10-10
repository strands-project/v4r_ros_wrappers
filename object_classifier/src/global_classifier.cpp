/*
 * esf_classifier
 *
 *  Created on: Nov, 2015
 *      Author: Thomas Faeulhammer
 */


#include <sstream>

#include <Eigen/Eigenvalues>
#include <pcl_conversions/pcl_conversions.h>

#include "global_classifier.h"

#include <v4r/features/esf_estimator.h>
#include <v4r/io/filesystem.h>
#include <v4r/ml/nearestNeighbor.h>
#include <v4r/ml/svmWrapper.h>
#include <v4r/recognition/source.h>
#include <v4r/segmentation/all_headers.h>

#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <glog/logging.h>

#include <fstream>
#include <sstream>


namespace po = boost::program_options;

namespace v4r
{

template<typename PointT>
bool
ClassifierROS<PointT>::classify (classifier_srv_definitions::classify::Request &req, classifier_srv_definitions::classify::Response &response)
{
    cloud_.reset (new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(req.cloud, *cloud_);

    //clear all data from previous visualization steps and publish empty markers/point cloud
    for (size_t i=0; i < markerArray_.markers.size(); i++)
        markerArray_.markers[i].action = visualization_msgs::Marker::DELETE;

    vis_pub_.publish( markerArray_ );
    sensor_msgs::PointCloud2 scenePc2;
    vis_pc_pub_.publish(scenePc2);
    markerArray_.markers.clear();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_, *cluster_rgb);

    for(size_t cluster_id=0; cluster_id < req.clusters_indices.size(); cluster_id++)
    {
        const float r = std::rand() % 255;
        const float g = std::rand() % 255;
        const float b = std::rand() % 255;

        std::vector<int> cluster_indices (req.clusters_indices[cluster_id].data.size());
        for(size_t kk=0; kk < req.clusters_indices[cluster_id].data.size(); kk++)
        {
            cluster_indices[kk] = static_cast<int>(req.clusters_indices[cluster_id].data[kk]);
            cluster_rgb->at(req.clusters_indices[cluster_id].data[kk]).r = 0.8*r;
            cluster_rgb->at(req.clusters_indices[cluster_id].data[kk]).g = 0.8*g;
            cluster_rgb->at(req.clusters_indices[cluster_id].data[kk]).b = 0.8*b;
        }

        typename GlobalRecognizer<PointT>::Cluster::Ptr cluster (
                    new typename GlobalRecognizer<PointT>::Cluster (*cloud_, cluster_indices, false ) );
        rec_->setInputCloud( cloud_ );
        rec_->setCluster( cluster );
        rec_->recognize();
        std::vector<typename ObjectHypothesis::Ptr > ohs = rec_->getHypotheses();

        std::cout << "for cluster " << cluster_id << " with size " << cluster_indices.size() << ", I have following hypotheses: " << std::endl;

        object_perception_msgs::classification class_tmp;
        for(typename ObjectHypothesis::Ptr oh : ohs)
        {
            std::cout << oh->model_id_ << " with confidence " << oh->confidence_ << std::endl;
            std_msgs::String str_tmp;
            str_tmp.data = oh->model_id_;
            class_tmp.class_type.push_back(str_tmp);
            class_tmp.confidence.push_back( oh->confidence_ );
        }
        response.class_results.push_back(class_tmp);

        Eigen::Vector4f centroid;
        Eigen::Matrix3f covariance_matrix;
        typename pcl::PointCloud<PointT>::Ptr pClusterPCl_transformed (new pcl::PointCloud<PointT>());
        pcl::computeMeanAndCovarianceMatrix(*cloud_, cluster_indices, covariance_matrix, centroid);
        Eigen::Matrix3f eigvects;
        Eigen::Vector3f eigvals;
        pcl::eigen33(covariance_matrix, eigvects,  eigvals);

        Eigen::Vector3f centroid_transformed = eigvects.transpose() * centroid.topRows(3);

        Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Zero(4,4);
        transformation_matrix.block<3,3>(0,0) = eigvects.transpose();
        transformation_matrix.block<3,1>(0,3) = -centroid_transformed;
        transformation_matrix(3,3) = 1;

        pcl::transformPointCloud(*cloud_, cluster_indices, *pClusterPCl_transformed, transformation_matrix);

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
        pcl::getMinMax3D (*cloud_, cluster_indices, min, max);

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
        typename pcl::PointCloud<PointT>::Ptr cluster_pcl (new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*cloud_, cluster_indices, *cluster_pcl);

        sensor_msgs::PointCloud2  pc2;
        pcl::toROSMsg (*cluster_pcl, pc2);
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
        marker_text << ohs[0]->model_id_ << ohs[0]->confidence_;
        marker.text = marker_text.str();
        markerArray_.markers.push_back(marker);
    }

    pcl::toROSMsg (*cluster_rgb, scenePc2);
    vis_pc_pub_.publish(scenePc2);
    vis_pub_.publish( markerArray_ );

    response.clusters_indices = req.clusters_indices;
    return true;
}

template<typename PointT>
void ClassifierROS<PointT>::initialize(int argc, char ** argv)
{
    //    PCLSegmenter<pcl::PointXYZRGB>::Parameter seg_param;
    ros::init (argc, argv, "classifier_service");
    n_.reset( new ros::NodeHandle ( "~" ) );

    std::string models_dir;
    int knn = 5;
    bool retrain = false;

    google::InitGoogleLogging(argv[0]);

    po::options_description desc("Depth-map and point cloud Rendering from mesh file\n======================================\n**Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("models_dir,m", po::value<std::string>(&models_dir)->required(), "model directory ")
            ("kNN,k", po::value<int>(&knn)->default_value(knn), "defines the number k of nearest neighbor for classification")
            ("retrain", po::bool_switch(&retrain), "if true, retrains the model database no matter if they already exist")
            ;
    po::variables_map vm;
    po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
    std::vector<std::string> to_pass_further = po::collect_unrecognized(parsed.options, po::include_positional);
    po::store(parsed, vm);
    if (vm.count("help"))
    { std::cout << desc << std::endl; return; }

    try {po::notify(vm);}
    catch(std::exception& e) { std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl; return; }

    ROS_INFO("models_dir:  %s",  models_dir.c_str());


    // ==== SETUP RECOGNIZER ======
    typename Source<PointT>::Ptr model_database (new Source<PointT> ( models_dir, true ) );
    typename ESFEstimation<PointT>::Ptr estimator (new ESFEstimation<PointT>);
    typename GlobalEstimator<PointT>::Ptr cast_estimator = boost::dynamic_pointer_cast<ESFEstimation<PointT> > (estimator);

    rec_.reset(new GlobalRecognizer<PointT>);
    rec_->setModelDatabase( model_database );
    rec_->setFeatureEstimator( cast_estimator );

//    NearestNeighborClassifier::Ptr classifier (new NearestNeighborClassifier);
    SVMParameter svmParam;
    svmParam.svm_.kernel_type = ::RBF;
    svmParam.svm_.gamma = 1./640.;
    svmParam.svm_.probability = 1;
    svmParam.knn_ = 3;

    svmClassifier::Ptr classifier (new svmClassifier (svmParam));
    rec_->setClassifier(classifier);
    rec_->initialize(models_dir, retrain);

    //  segment_and_classify_service_ = n_->advertiseService("segment_and_classify", ShapeClassifier::segmentAndClassify, this);
    classify_service_ = n_->advertiseService("classify", &ClassifierROS<PointT>::classify, this);
    vis_pub_ = n_->advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );
    vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "clusters", 1 );

    ROS_INFO("Ready to get service calls.");
    ros::spin();
}

}

int
main (int argc, char ** argv)
{
    v4r::ClassifierROS<pcl::PointXYZ> ros_classifier;
    ros_classifier.initialize (argc, argv);

    return 0;
}
