#include "singleview_object_recognizer.h"
#include "world_representation.h"

#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <string>
#include <v4r/ORUtils/filesystem_utils.h>

#include <pcl/io/pcd_io.h>
ros::ServiceClient client_;

int main (int argc, char **argv)
{
    boost::shared_ptr<ros::NodeHandle> n;
    std::string models_dir;
    bool visualize_output;
    bool scene_to_scene;
    int icp_iter;
    int icp_type;
    double icp_voxel_size;
    int opt_type;

    // HV Params
    double resolution;
    double inlier_threshold;
    double radius_clutter;
    double regularizer;
    double clutter_regularizer;
    double occlusion_threshold;
    int optimizer_type;
    double color_sigma_l;
    double color_sigma_ab;


    // CG PARAMS
    int cg_size_threshold;
    double cg_size;
    double ransac_threshold;
    double dist_for_clutter_factor;
    int max_taken;
    double max_time_for_cliques_computation;
    double dot_distance;

    double chop_at_z;
    double distance_keypoints_get_discarded;
    std::string training_dir_sift, training_dir_shot, sift_structure, training_dir_ourcvfh;
    bool do_sift=false, do_ourcvfh=false, do_shot=false, ignore_color;
    int max_vertices_in_graph;

    ros::init ( argc, argv, "multiview_object_recognizer_eval_node" );
    n.reset( new ros::NodeHandle ( "~" ) );

    if ( ! n->getParam ( "models_dir", models_dir ))
    {
        std::cout << "No models_dir specified. " << std::endl;
    }

    n->getParam ( "training_dir_sift", training_dir_sift);
    n->getParam ( "training_dir_shot", training_dir_shot);
    n->getParam ( "recognizer_structure_sift", sift_structure);
    n->getParam ( "training_dir_ourcvfh", training_dir_ourcvfh);

    std::cout << chop_at_z << ", " << ignore_color << std::endl;
    if (models_dir.compare ("") == 0)
    {
        PCL_ERROR ("Set -models_dir option in the command line, ABORTING");
        return -1;
    }


    //-----Init-SIFT-GPU-Context--------
    static char kw[][16] = {"-m", "-fo", "-1", "-s", "-v", "1", "-pack"};
    char * argvv[] = {kw[0], kw[1], kw[2], kw[3],kw[4],kw[5],kw[6], NULL};

    int argcc = sizeof(argvv) / sizeof(char*);
    cv::Ptr<SiftGPU> sift = new SiftGPU ();
    sift->ParseParam (argcc, argvv);

    //create an OpenGL context for computation
    if (sift->CreateContextGL () != SiftGPU::SIFTGPU_FULL_SUPPORTED)
        throw std::runtime_error ("PSiftGPU::PSiftGPU: No GL support!");


    boost::shared_ptr<Recognizer> pSingleview_recognizer (new Recognizer());
    pSingleview_recognizer->setTraining_dir_ourcvfh(training_dir_ourcvfh);
    pSingleview_recognizer->setTraining_dir_sift(training_dir_sift);
    pSingleview_recognizer->setTraining_dir_shot(training_dir_shot);
    pSingleview_recognizer->setModels_dir(models_dir);
    pSingleview_recognizer->setSift_structure(sift_structure);


    if(n->getParam ( "icp_iterations", icp_iter))
        pSingleview_recognizer->set_icp_iterations(icp_iter);

    if(n->getParam ( "icp_type", icp_type))
        pSingleview_recognizer->set_icp_type(icp_type);

    if(n->getParam ( "icp_voxel_size", icp_voxel_size))
        pSingleview_recognizer->set_icp_type(icp_voxel_size);

    if(n->getParam ( "do_sift", do_sift))
        pSingleview_recognizer->set_do_sift(do_sift);

    if(n->getParam ( "do_shot", do_shot))
        pSingleview_recognizer->set_do_shot(do_shot);

    if(do_sift)
        pSingleview_recognizer->set_sift(sift);

    if(n->getParam ( "do_ourcvfh", do_ourcvfh))
        pSingleview_recognizer->set_do_ourcvfh(do_ourcvfh);

    if (do_sift && training_dir_sift.compare ("") == 0)
    {
        PCL_ERROR ("do_sift is activated but training_dir_sift_ is empty! Set -training_dir_sift option in the command line, ABORTING");
        return -1;
    }

    if (do_ourcvfh && training_dir_ourcvfh.compare ("") == 0)
    {
        PCL_ERROR ("do_ourcvfh is activated but training_dir_ourcvfh_ is empty! Set -training_dir_ourcvfh option in the command line, ABORTING");
        return -1;
    }

    if (do_shot && training_dir_shot.compare ("") == 0)
    {
        PCL_ERROR ("do_shot is activated but training_dir_ourcvfh_ is empty! Set -training_dir_shot option in the command line, ABORTING");
        return -1;
    }

    if(n->getParam ( "cg_size_thresh", cg_size_threshold)) // For correspondence grouping (default: 5, minimum: 3), the higher, the fewer hypotheses are constructed
        pSingleview_recognizer->set_cg_size_threshold(cg_size_threshold);

    if(n->getParam ( "cg_size", cg_size))
        pSingleview_recognizer->set_cg_size(cg_size);

    if(n->getParam ( "cg_ransac_threshold", ransac_threshold))
        pSingleview_recognizer->set_cg_ransac_threshold(ransac_threshold);

    if(n->getParam ( "cg_dist_for_clutter_factor", dist_for_clutter_factor))
        pSingleview_recognizer->set_cg_dist_for_clutter_factor(dist_for_clutter_factor);

    if(n->getParam ( "cg_max_taken", max_taken))
        pSingleview_recognizer->set_cg_max_taken(max_taken);

    if(n->getParam ( "cg_max_time_for_cliques_computation", max_time_for_cliques_computation))
        pSingleview_recognizer->set_cg_max_time_for_cliques_computation(max_time_for_cliques_computation);

    if(n->getParam ( "cg_dot_distance", dot_distance))
        pSingleview_recognizer->set_cg_dot_distance(dot_distance);

    if(n->getParam ( "hv_resolution", resolution))
        pSingleview_recognizer->set_hv_resolution(resolution);

    if(n->getParam ( "hv_inlier_threshold", inlier_threshold))
        pSingleview_recognizer->set_hv_inlier_threshold(inlier_threshold);

    if(n->getParam ( "hv_radius_clutter", radius_clutter))
        pSingleview_recognizer->set_hv_radius_clutter(radius_clutter);

    if(n->getParam ( "hv_regularizer", regularizer))
        pSingleview_recognizer->set_hv_regularizer(regularizer);

    if(n->getParam ( "hv_clutter_regularizer", clutter_regularizer))
        pSingleview_recognizer->set_hv_clutter_regularizer(clutter_regularizer);

    if(n->getParam ( "hv_occlusion_threshold", occlusion_threshold))
        pSingleview_recognizer->set_hv_occlusion_threshold(occlusion_threshold);

    if(n->getParam ( "hv_optimizer_type", optimizer_type))
        pSingleview_recognizer->set_hv_optimizer_type(optimizer_type);

    if(n->getParam ( "hv_color_sigma_l", color_sigma_l))
        pSingleview_recognizer->set_hv_color_sigma_L(color_sigma_l);

    if(n->getParam ( "hv_color_sigma_ab", color_sigma_ab))
        pSingleview_recognizer->set_hv_color_sigma_AB(color_sigma_ab);



    pSingleview_recognizer->initialize();


    worldRepresentation myWorld;
    myWorld.setModels_dir(models_dir);
    myWorld.setPSingleview_recognizer(pSingleview_recognizer);
    myWorld.setSift(sift);

    if(n->getParam ( "opt_type", opt_type))
        myWorld.setOpt_type(opt_type);

    if(n->getParam ( "chop_z", chop_at_z))
        myWorld.setChop_at_z(chop_at_z);

    if(n->getParam ( "scene_to_scene", scene_to_scene))
        myWorld.set_scene_to_scene(scene_to_scene);

    if(n->getParam ( "visualize_output", visualize_output))
        myWorld.set_visualize_output(visualize_output);

    if(n->getParam ( "max_vertices_in_graph", max_vertices_in_graph))
        myWorld.set_max_vertices_in_graph(max_vertices_in_graph);

    if(n->getParam ( "distance_keypoints_get_discarded", distance_keypoints_get_discarded))
        myWorld.set_distance_keypoints_get_discarded(distance_keypoints_get_discarded);


    std::string dataset_path, sequence_name;
    const std::string eval_path = "/home/thomas/Projects/thomas.faeulhammer/eval/";
    const std::string transform_prefix_ = "transformation_";

    if(!n->getParam("dataset_path", dataset_path))
        ROS_ERROR("No dataset path given (arg \"dataset_path\"). ");

    if(!n->getParam("sequence_name", sequence_name))
        ROS_ERROR("No sequence name given (arg \"sequence_name\"). ");

    std::stringstream scenes_dir_ss;
    scenes_dir_ss << dataset_path << "/" << sequence_name;
    std::string scenes_dir = scenes_dir_ss.str();

    boost::filesystem::path scenes_dir_bf = scenes_dir;
    if (boost::filesystem::exists (scenes_dir_bf)) //no hypothesis exist yet --> create
    {
        std::cout << "Starting eval for " << scenes_dir_ss.str();
        std::vector < std::string > files_intern;
        faat_pcl::utils::getFilesInDirectory (scenes_dir_bf, files_intern, "", ".*.pcd", true);

        if (files_intern.size())
        {
            size_t sub_id_mv = 0;
            std::stringstream or_folderpath_mv_ss;
            boost::filesystem::path or_folderpath_mv;
            do
            {
                or_folderpath_mv_ss.str(std::string());
                or_folderpath_mv_ss << eval_path << sequence_name << "_" << sub_id_mv << "_mv/";
                or_folderpath_mv = or_folderpath_mv_ss.str();
                sub_id_mv++;
            }while(boost::filesystem::exists(or_folderpath_mv) );
            boost::filesystem::create_directory(or_folderpath_mv);


            size_t sub_id_sv = 0;
            std::stringstream or_folderpath_sv_ss;
            boost::filesystem::path or_folderpath_sv;
            do
            {
                or_folderpath_sv_ss.str(std::string());
                or_folderpath_sv_ss << eval_path << sequence_name << "_" << sub_id_sv << "_sv/";
                or_folderpath_sv = or_folderpath_sv_ss.str();
                sub_id_sv++;
            }while(boost::filesystem::exists(or_folderpath_sv) );
            boost::filesystem::create_directory(or_folderpath_sv);


            std::stringstream param_file_text;
            param_file_text << "cg_size_thresh: " << cg_size_threshold << std::endl
                            << "cg_size: " << cg_size << std::endl
                            << "cg_ransac_threshold: " << ransac_threshold << std::endl
                            << "cg_dist_for_clutter_factor: " << dist_for_clutter_factor << std::endl
                            << "cg_max_taken: " << max_taken << std::endl
                            << "cg_max_time_for_cliques_computation: " << max_time_for_cliques_computation << std::endl
                            << "cg_dot_distance: " << dot_distance << std::endl
                            << "hv_resolution: " << resolution << std::endl
                            << "hv_inlier_threshold: " << inlier_threshold << std::endl
                            << "hv_radius_clutter: " << radius_clutter << std::endl
                            << "hv_regularizer: " << regularizer << std::endl
                            << "hv_clutter_regularizer: " << clutter_regularizer << std::endl
                            << "hv_occlusion_threshold: " << occlusion_threshold << std::endl
                            << "hv_optimizer_type: " << optimizer_type << std::endl
                            << "hv_color_sigma_l: " << color_sigma_l << std::endl
                            << "hv_color_sigma_ab: " << color_sigma_ab << std::endl
                            << "opt_type: " << opt_type << std::endl
                            << "chop_z: " << chop_at_z << std::endl
                            << "scene_to_scene: " << scene_to_scene << std::endl
                            << "visualize_output: " << visualize_output << std::endl
                            << "max_vertices_in_graph: " << max_vertices_in_graph << std::endl
                            << "distance_keypoints_get_discarded: " << distance_keypoints_get_discarded << std::endl
                            << "icp_iterations: " << icp_iter << std::endl
                            << "icp_type: " << icp_type << std::endl
                            << "icp_voxel_size: " << icp_voxel_size << std::endl
                            << "do_sift: " << do_sift << std::endl
                            << "do_shot: " << do_shot << std::endl
                            << "do_ourcvfh: " << do_ourcvfh << std::endl;

            std::stringstream or_filepath_parameter_ss_sv;
            or_filepath_parameter_ss_sv << or_folderpath_sv_ss.str() << "parameter.nfo";
            ofstream or_file;
            or_file.open (or_filepath_parameter_ss_sv.str());
            or_file << param_file_text.str();
            or_file.close();

            std::stringstream or_filepath_parameter_ss_mv;
            or_filepath_parameter_ss_mv << or_folderpath_mv_ss.str() << "parameter.nfo";
            or_file.open (or_filepath_parameter_ss_mv.str());
            or_file << param_file_text.str();
            or_file.close();

            for (size_t file_id=0; file_id < files_intern.size(); file_id++)
            {
                std::stringstream full_file_name;
                full_file_name << scenes_dir << "/" << files_intern[file_id];
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pScene (new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::io::loadPCDFile(full_file_name.str(), *pScene);

                std::vector<double> transform;
                transform.clear();

                std::stringstream transform_ss;
                transform_ss << scenes_dir << "/" << transform_prefix_ << files_intern[file_id].substr(0, files_intern[file_id].length() - ext.length()) << "txt";
                std::cout << "Checking if path " << transform_ss.str() << " for transform exists. " << std::endl;

                if ( boost::filesystem::exists( transform_ss.str() ) )
                {
                    std::cout << "File exists." << std::endl;
                    std::ifstream is(transform_ss.str());
                    std::istream_iterator<double> start(is), end;
                    std::vector<double> numbers(start, end);
                    std::cout << "Read " << numbers.size() << " numbers" << std::endl;

                    // print the numbers to stdout
                    std::cout << "Transform to world coordinate system: " << std::endl;
                    for(size_t i=0; i<numbers.size(); i++)
                    {
                        std::cout << numbers[i] << " ";
                        transform.push_back(numbers[i]);
                    }
                    std::cout << std::endl;
                }
                else
                {
                    std::cout << "File does not exist. Using it without world transform." << std::endl;
                }

                std::vector<ModelTPtr> models_mv;
                std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_mv;
                myWorld.recognize(pScene,
                                  sequence_name,
                                  files_intern[file_id].substr(0, files_intern[file_id].length() - ext.length() - 1),
                                  ros::Time::now().nsec,
                                  transform,
                                  models_mv,
                                  transforms_mv,
                                  or_folderpath_mv_ss.str(),
                                  or_folderpath_sv_ss.str());

                if ( models_mv.size() == 0 )
                {
                    std::cout << "I didn't detect any object in the current scene." << std::endl;
                }
                else
                {
                    for ( size_t i=0; i < models_mv.size(); i++ )
                    {
                        std::cout << "I detected object " << models_mv.at(i)->id_ << " in the scene." << std::endl;
                    }
                }
            }
        }
    }
    return 0;
}
