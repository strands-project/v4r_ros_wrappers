#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "incremental_object_learning_srv_definitions/create_3D_model.h"
#include <opencv2/opencv.hpp>
#include <v4r/io/filesystem.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <limits>
#include <stdio.h>
#include <opencv2/opencv.hpp>

#include <v4r/io/eigen.h>
#include <v4r/io/filesystem.h>
#include <v4r/registration/noise_model_based_cloud_integration.h>
#include <v4r/common/noise_models.h>
#include <v4r/common/normals.h>
#include <v4r/common/pcl_opencv.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <glog/logging.h>


namespace po = boost::program_options;
using namespace v4r;


template <typename PointT>
class NMBasedCloudIntegrationROS
{
private:
    std::string test_dir, out_dir, view_prefix, obj_indices_prefix, pose_prefix;
    float chop_z;
    bool use_object_mask, use_pose_file;

    typename NMBasedCloudIntegration<PointT>::Parameter nm_int_param;
    typename NguyenNoiseModel<PointT>::Parameter nm_param;
    int normal_method;

public:
    NMBasedCloudIntegrationROS()
    {
        nm_int_param.min_points_per_voxel_ = 1;
        nm_int_param.octree_resolution_ = 0.002f;
        normal_method = 2;
        chop_z = std::numeric_limits<float>::max();

        use_object_mask = use_pose_file = true;
        view_prefix = "cloud_";
        obj_indices_prefix = "object_indices_";
        pose_prefix = "pose_";
    }

    bool init (int argc, char ** argv)
    {
        po::options_description desc("Noise model based cloud integration\n======================================\n**Allowed options");
        desc.add_options()
                ("help,h", "produce help message")
                ("view_prefix", po::value<std::string>(&view_prefix)->default_value(view_prefix), "view filename prefix for each point cloud (used when using object mask)")
                ("obj_indices_prefix", po::value<std::string>(&obj_indices_prefix)->default_value(obj_indices_prefix), "filename prefix for each object mask file(used when using object mask)")
                ("pose_prefix", po::value<std::string>(&pose_prefix)->default_value(pose_prefix), "filename prefix for each camera pose (used when using use_pose_file)")
                ("resolution,r", po::value<float>(&nm_int_param.octree_resolution_)->default_value(nm_int_param.octree_resolution_), "")
                ("min_points_per_voxel", po::value<int>(&nm_int_param.min_points_per_voxel_)->default_value(nm_int_param.min_points_per_voxel_), "")
    //            ("threshold_explained", po::value<float>(&nm_int_param.threshold_explained_)->default_value(nm_int_param.threshold_explained_), "")
                ("use_depth_edges", po::value<bool>(&nm_param.use_depth_edges_)->default_value(nm_param.use_depth_edges_), "")
                ("edge_radius", po::value<int>(&nm_param.edge_radius_)->default_value(nm_param.edge_radius_), "")
                ("normal_method,n", po::value<int>(&normal_method)->default_value(normal_method), "method used for normal computation")
                ("chop_z,z", po::value<float>(&chop_z)->default_value(chop_z), "cut of distance in m ")
                ("use_object_mask,m", po::value<bool>(&use_object_mask)->default_value(use_object_mask), "reads mask file and only extracts those indices (only if file exists)")
                ("use_pose_file,p", po::value<bool>(&use_pose_file)->default_value(use_pose_file), "reads pose from seperate pose file instead of extracting it directly from .pcd file (only if file exists)")
          ;
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if (vm.count("help"))
        {
            std::cout << desc << std::endl;
            return false;
        }

        try
        { po::notify(vm); }
        catch(std::exception& e)
        {
            std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
            return false;
        }

        nm_int_param.edge_radius_px_ = nm_param.edge_radius_;
    }

    bool create3DModel(incremental_object_learning_srv_definitions::create_3D_model::Request &req,
                       incremental_object_learning_srv_definitions::create_3D_model::Response &response)
    {
        std::string input_dir = req.input_dir.data;
        std::string output_dir = req.output_dir.data;
        int normal_method = 3;

        std::vector< std::string > views = io::getFilesInDirectory(input_dir, ".*.pcd", false);

        if(views.empty())
        {
            std::cerr << "No .pcd file in given directory (" << input_dir << "). Aborting. " << std::endl;
            return false;
        }

        typename pcl::PointCloud<PointT>::Ptr big_cloud_unfiltered (new pcl::PointCloud<PointT>);
        std::vector< typename pcl::PointCloud<PointT>::Ptr > clouds (views.size());
        std::vector< pcl::PointCloud<pcl::Normal>::Ptr > normals (views.size());
        std::vector<std::vector<std::vector<float> > > pt_properties (views.size());
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > camera_transforms (views.size());
        std::vector<std::vector<int> > obj_indices (views.size());

        for (size_t v_id=0; v_id<views.size(); v_id++)
        {
            std::stringstream txt;
            txt << "processing view " << v_id;
            pcl::ScopeTime t(txt.str().c_str());
            clouds[v_id].reset ( new pcl::PointCloud<PointT>);
            normals[v_id].reset ( new pcl::PointCloud<pcl::Normal>);

            pcl::io::loadPCDFile(input_dir + "/" + views[ v_id ], *clouds[v_id]);

            std::string obj_fn (views[v_id]);
            boost::replace_first (obj_fn, view_prefix, obj_indices_prefix);
            boost::replace_last (obj_fn, ".pcd", ".txt");

            if(io::existsFile(input_dir + "/" + obj_fn) && use_object_mask) {
                std::ifstream f((input_dir+"/"+obj_fn).c_str());
                int idx;
                while (f >> idx)
                    obj_indices[v_id].push_back(idx);
                f.close();
            }

            std::string pose_fn (views[v_id]);
            boost::replace_first (pose_fn, view_prefix, pose_prefix);
            boost::replace_last (pose_fn, ".pcd", ".txt");

            if(io::existsFile(input_dir + "/" + pose_fn) && use_pose_file) {
                camera_transforms[v_id] = io::readMatrixFromFile(input_dir + "/" + pose_fn);
            }
            else
                camera_transforms[v_id] = RotTrans2Mat4f(clouds[v_id]->sensor_orientation_, clouds[v_id]->sensor_origin_);

            // reset view point otherwise pcl visualization is potentially messed up
            clouds[v_id]->sensor_orientation_ = Eigen::Quaternionf::Identity();
            clouds[v_id]->sensor_origin_ = Eigen::Vector4f::Zero();

            {
                pcl::ScopeTime tt("Computing normals");
                computeNormals<PointT>( clouds[v_id], normals[v_id], normal_method);
            }

            {
                pcl::ScopeTime tt("Computing noise model parameter for cloud");
                NguyenNoiseModel<PointT> nm (nm_param);
                nm.setInputCloud(clouds[v_id]);
                nm.setInputNormals(normals[v_id]);
                nm.compute();
                pt_properties[v_id] = nm.getPointProperties();

            }

            pcl::PointCloud<PointT> object_cloud, object_aligned;
            pcl::copyPointCloud(*clouds[v_id], obj_indices[v_id], object_cloud);
            pcl::transformPointCloud( object_cloud, object_aligned, camera_transforms[v_id]);
            *big_cloud_unfiltered += object_aligned;
        }

        typename pcl::PointCloud<PointT>::Ptr octree_cloud(new pcl::PointCloud<PointT>);
        NMBasedCloudIntegration<PointT> nmIntegration (nm_int_param);
        nmIntegration.setInputClouds(clouds);
        nmIntegration.setPointProperties(pt_properties);
        nmIntegration.setTransformations(camera_transforms);
        nmIntegration.setInputNormals(normals);
        nmIntegration.setIndices(obj_indices);
        nmIntegration.compute(octree_cloud);
        std::vector< typename pcl::PointCloud<PointT>::Ptr > clouds_used;
        nmIntegration.getInputCloudsUsed(clouds_used);

        std::cout << "Size cloud unfiltered: " << big_cloud_unfiltered->points.size() << ", filtered: " << octree_cloud->points.size() << std::endl;

        io::createDirIfNotExist(output_dir);
        pcl::io::savePCDFileBinary(output_dir + "/3D_model.pcd", *octree_cloud);

        return true;
    }
};


int
main (int argc, char ** argv)
{
    ros::init (argc, argv, "NoiseModelBasedCloudIntegration");
    ros::NodeHandle n( "~" );

    NMBasedCloudIntegrationROS<pcl::PointXYZRGB> nm;
    nm.init(argc, argv);

    ros::ServiceServer create3dmodel = n.advertiseService ("create3DModel", &NMBasedCloudIntegrationROS<pcl::PointXYZRGB>::create3DModel, &nm);
    ros::spin ();
    return 0;
}
