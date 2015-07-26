#include "object_learning_ros.h"
#include "pcl_conversions.h"

#include <v4r/common/miscellaneous.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>


namespace v4r
{
namespace object_modelling
{


bool
DOL_ROS::clear_cached_model (do_learning_srv_definitions::clear::Request & req,
                 do_learning_srv_definitions::clear::Response & response)
{
    clear();
    return true;
}

bool
DOL_ROS::save_model (do_learning_srv_definitions::save_model::Request & req,
                 do_learning_srv_definitions::save_model::Response & response)
{
    return DOL::save_model(req.models_folder.data, req.recognition_structure_folder.data, req.object_name.data);
}

bool
DOL_ROS::visualizeROS(do_learning_srv_definitions::visualize::Request & req,
                    do_learning_srv_definitions::visualize::Response & response)
{
    visualize();
    return true;
}

bool
DOL_ROS::learn_object (do_learning_srv_definitions::learn_object::Request & req,
                   do_learning_srv_definitions::learn_object::Response & response)
{
    bool ok = true;
    assert(req.keyframes.size() == req.transforms.size());

    for (size_t i=0; i<req.keyframes.size(); i++)
    {
        do_learning_srv_definitions::learn_object_inc srv_learn_inc;
        srv_learn_inc.request.cloud = req.keyframes[i];
        srv_learn_inc.request.transform = req.transforms[i];

        if (i==0)
            srv_learn_inc.request.object_indices = req.intial_object_indices;

        ok = learn_object_inc(srv_learn_inc.request, srv_learn_inc.response);

        if (!ok)
            break;
    }

    return ok;
}


bool DOL_ROS::learn_object_inc (do_learning_srv_definitions::learn_object_inc::Request & req,
                       do_learning_srv_definitions::learn_object_inc::Response & response)
{
    pcl::PointCloud<PointT> cloud;
    pcl::fromROSMsg(req.cloud, cloud);

    Eigen::Matrix4f tf = fromGMTransform(req.transform);
//    v4r::common::setCloudPose(tf, cloud);

    std::vector<size_t> initial_indices;
    initial_indices.resize( req.object_indices.size() );

    for(size_t i=0; i < req.object_indices.size(); i++)
    {
        initial_indices[i] = req.object_indices[i];
    }
    return DOL::learn_object(cloud, tf, initial_indices);
}

void
DOL_ROS::initialize (int argc, char ** argv)
{
    double inlDist, minPoints;

    n_.reset( new ros::NodeHandle ( "~" ) );
    n_->getParam ( "radius", param_.radius_);
    n_->getParam ( "dot_product", param_.eps_angle_);
    n_->getParam ( "seed_res", param_.seed_resolution_);
    n_->getParam ( "voxel_res", param_.voxel_resolution_);
    n_->getParam ( "ratio", param_.ratio_);
    n_->getParam ( "do_erosion", param_.do_erosion_);
    n_->getParam ( "do_mst_refinement", param_.do_mst_refinement_);
    n_->getParam ( "do_sift_based_camera_pose_estimation", param_.do_sift_based_camera_pose_estimation_);
    n_->getParam ( "transfer_latest_only", param_.transfer_indices_from_latest_frame_only_);
    n_->getParam ( "chop_z", param_.chop_z_);
    n_->getParam ( "normal_method", param_.normal_method_);

    if( n_->getParam ( "inlier_threshold_plane_seg", inlDist) )
        p_param_.inlDist = static_cast<float> (inlDist);

    if ( n_->getParam ( "min_plane_points", minPoints) )
        p_param_.minPoints = static_cast<float> (minPoints);


    DOL::initialize(argc, argv);

    clear_cached_model_  = n_->advertiseService ("clear_cached_model", &DOL_ROS::clear_cached_model, this);
    learn_object_  = n_->advertiseService ("learn_object", &DOL_ROS::learn_object, this);
    learn_object_inc_  = n_->advertiseService ("learn_object_incremental", &DOL_ROS::learn_object_inc, this);
    save_model_  = n_->advertiseService ("save_model", &DOL_ROS::save_model, this);
    vis_model_  = n_->advertiseService ("visualize", &DOL_ROS::visualizeROS, this);
    vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "learned_model", 1 );

    std::cout << "Started dynamic object learning with parameters: " << std::endl
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
    ros::init (argc, argv, "dynamic_object_learning");
    v4r::object_modelling::DOL_ROS m;
    m.initialize (argc, argv);

    return 0;
}
