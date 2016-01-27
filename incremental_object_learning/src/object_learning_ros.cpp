#include "object_learning_ros.h"
#include <pcl_conversions/pcl_conversions.h>

#include <v4r/common/miscellaneous.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>


namespace v4r
{
namespace object_modelling
{


bool
IOL_ROS::clear_cached_model (incremental_object_learning_srv_definitions::clear::Request & req,
                 incremental_object_learning_srv_definitions::clear::Response & response)
{
    clear();
    return true;
}

bool
IOL_ROS::save_model (incremental_object_learning_srv_definitions::save_model::Request & req,
                 incremental_object_learning_srv_definitions::save_model::Response & response)
{
    bool save_views = true;
    return IOL::save_model(req.models_folder.data, req.object_name.data, save_views);
}

bool
IOL_ROS::visualizeROS(incremental_object_learning_srv_definitions::visualize::Request & req,
                    incremental_object_learning_srv_definitions::visualize::Response & response)
{
    visualize();
    return true;
}


bool
IOL_ROS::writeImagesToDiskROS(incremental_object_learning_srv_definitions::write_debug_images_to_disk::Request & req,
                    incremental_object_learning_srv_definitions::write_debug_images_to_disk::Response & response)
{
    writeImagesToDisk(req.path.data, req.crop_images);
    return true;
}

bool
IOL_ROS::learn_object (incremental_object_learning_srv_definitions::learn_object::Request & req,
                   incremental_object_learning_srv_definitions::learn_object::Response & response)
{
    bool ok = true;
    assert(req.keyframes.size() == req.transforms.size());

    for (size_t i=0; i<req.keyframes.size(); i++)
    {
        incremental_object_learning_srv_definitions::learn_object_inc srv_learn_inc;
        srv_learn_inc.request.cloud = req.keyframes[i];
        srv_learn_inc.request.transform = req.transforms[i];

        if (i==0)
            srv_learn_inc.request.object_indices = req.intial_object_indices;

        ok = learn_object_inc(srv_learn_inc.request, srv_learn_inc.response);

        if (!ok)
            break;

        if(visualize_intermediate_results_)
            visualize();
    }
    return ok;
}


bool IOL_ROS::learn_object_inc (incremental_object_learning_srv_definitions::learn_object_inc::Request & req,
                       incremental_object_learning_srv_definitions::learn_object_inc::Response & response)
{
    pcl::PointCloud<PointT> cloud;
    pcl::fromROSMsg(req.cloud, cloud);

    Eigen::Matrix4f tf = fromGMTransform(req.transform);

    std::vector<size_t> initial_indices;
    initial_indices.resize( req.object_indices.size() );

    for(size_t i=0; i < req.object_indices.size(); i++)
    {
        initial_indices[i] = req.object_indices[i];
    }
    return IOL::learn_object(cloud, tf, initial_indices);
}

void
IOL_ROS::initSIFT (int argc, char ** argv)
{
    int min_plane_points, min_smooth_points;
    visualize_intermediate_results_ = false;

    n_.reset( new ros::NodeHandle ( "~" ) );
    n_->getParam ( "radius", param_.radius_);
    n_->getParam ( "dot_product", param_.eps_angle_);
    n_->getParam ( "dist_threshold_growing", param_.dist_threshold_growing_);
    n_->getParam ( "seed_res", param_.seed_resolution_);
    n_->getParam ( "voxel_res", param_.voxel_resolution_);
    n_->getParam ( "ratio", param_.ratio_supervoxel_);
    n_->getParam ( "do_erosion", param_.do_erosion_);
    n_->getParam ( "do_mst_refinement", param_.do_mst_refinement_);
    n_->getParam ( "do_sift_based_camera_pose_estimation", param_.do_sift_based_camera_pose_estimation_);
    n_->getParam ( "transfer_latest_only", param_.transfer_indices_from_latest_frame_only_);
    n_->getParam ( "chop_z", param_.chop_z_);
    n_->getParam ( "normal_method", param_.normal_method_);
    n_->getParam ( "smooth_clustering", p_param_.smooth_clustering);
    n_->getParam ( "ratio_cluster_obj_supported", param_.ratio_cluster_obj_supported_);
    n_->getParam ( "ratio_cluster_occluded", param_.ratio_cluster_occluded_);
    n_->getParam ( "stat_outlier_removal_meanK", sor_params_.meanK_);
    n_->getParam ( "stat_outlier_removal_std_mul", sor_params_.std_mul_);
    n_->getParam ( "inlier_threshold_plane_seg", p_param_.inlDist);
    n_->getParam ( "visualize_intermediate_results", visualize_intermediate_results_);

    if ( n_->getParam ( "min_plane_points", min_plane_points) )
        p_param_.minPoints = static_cast<unsigned> (min_plane_points);

    if ( n_->getParam ( "min_points_smooth_cluster", min_smooth_points) )
        p_param_.minPointsSmooth = static_cast<unsigned> (min_smooth_points);


    IOL::initSIFT();

    clear_cached_model_  = n_->advertiseService ("clear_cached_model", &IOL_ROS::clear_cached_model, this);
    learn_object_  = n_->advertiseService ("learn_object", &IOL_ROS::learn_object, this);
    learn_object_inc_  = n_->advertiseService ("learn_object_incremental", &IOL_ROS::learn_object_inc, this);
    save_model_  = n_->advertiseService ("save_model", &IOL_ROS::save_model, this);
    vis_model_  = n_->advertiseService ("visualize", &IOL_ROS::visualizeROS, this);
    write_images_to_disk_srv_ = n_->advertiseService("write_debug_images_to_disk", &IOL_ROS::writeImagesToDiskROS, this);
    vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "learned_model", 1 );

    std::cout << "Started incremental object learning with parameters: " << std::endl
              << "===================================================" << std::endl;
    printParams(std::cout);
    std::cout << "===================================================" << std::endl << std::endl;

    ros::spin ();
}
}
}



int
main (int argc, char ** argv)
{
    ros::init (argc, argv, "incremental_object_learning");
    v4r::object_modelling::IOL_ROS m;
    m.initSIFT (argc, argv);

    return 0;
}
