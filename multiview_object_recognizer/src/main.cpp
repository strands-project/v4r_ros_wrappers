
//#define COMPILE_ME
#ifdef COMPILE_ME
#include "world_representation_ros.h"

#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <string>
#include <v4r/utils/filesystem_utils.h>
#include <v4r/ORRecognition/include/singleview_object_recognizer.h>
#include <v4r/ORRecognition/include/world_representation.h>
#include <pcl/io/pcd_io.h>

typedef faat_pcl::rec_3d_framework::Model<PointT> ModelT;
typedef boost::shared_ptr<ModelT> ModelTPtr;

//#define USE_WILLOW_DATASET_FOR_EVAL

ros::ServiceClient client_;

#endif
int main (int argc, char **argv)
{
#ifdef COMPILE_ME
    std::map<std::string, v4r::multiviewGraph> mv_environment_;

    boost::shared_ptr<ros::NodeHandle> nh;
    std::string models_dir;
    bool visualize_output;
    bool play_sequence_randomly = false;
    bool do_eval = true;
//    v4r::worldRepresentationROS myWorld;
    boost::shared_ptr<v4r::multiviewGraph> pMultiview_recognizer (new v4r::multiviewGraph());
//    boost::shared_ptr<v4r::Recognizer> pSingleview_recognizer (new v4r::Recognizer());

    ros::init ( argc, argv, "multiview_object_recognizer_node" );
    nh.reset( new ros::NodeHandle ( "~" ) );


    if(!(nh->getParam ( "do_eval", do_eval)))
        do_eval = false;


//    if(nh->getParam ( "visualize_output", visualize_output))
//        myWorld.set_visualize_output(visualize_output);
//    myWorld.setModels_dir(models_dir);
//    myWorld.setSift(sift);


    if(!do_eval)
    {
        ROS_INFO("Multiview object recognizer is ready to get service callsssss.");
        ros::spin();
    }
    else    // do some offline evaluation (with files saved locally)
    {
        int id = 0;

//        for (pMultiview_recognizer->hv_params_.regularizer_ = 1; pMultiview_recognizer->hv_params_.regularizer_ <= 7; pMultiview_recognizer->hv_params_.regularizer_+=2)
        {
//        for (pMultiview_recognizer->hv_params_.color_sigma_l_ = 0.2; pMultiview_recognizer->hv_params_.color_sigma_l_ <= 0.4; pMultiview_recognizer->hv_params_.color_sigma_l_ +=0.2)
        {
//            for (pMultiview_recognizer->hv_params_.clutter_regularizer_ = 1;
//                 pMultiview_recognizer->hv_params_.clutter_regularizer_ <= 5;
//                 pMultiview_recognizer->hv_params_.clutter_regularizer_ += 2)
            {

//                if ((pMultiview_recognizer->hv_params_.regularizer_ == 5 &&
//                     pMultiview_recognizer->hv_params_.color_sigma_l_ == 0.2 &&
//                     pMultiview_recognizer->hv_params_.clutter_regularizer_ <= 1.5))
////                    || (pMultiview_recognizer->cg_params_.cg_size_ ==0.01 && pMultiview_recognizer->cg_params_.cg_size_threshold_ == 4
////                       && pMultiview_recognizer->hv_params_.clutter_regularizer_ < 1.5))
//                    continue;


        std::string dataset_path, sequence_name_provided;
        const std::string transform_prefix_ = "transformation_";

        if(!nh->getParam("dataset_path", dataset_path))
            ROS_ERROR("No dataset path given (arg \"dataset_path\"). ");

        if(!nh->getParam("sequence_name", sequence_name_provided))
            ROS_ERROR("No sequence name given (arg \"sequence_name\"). ");

        std::stringstream eval_path_ss;
        boost::filesystem::path eval_folderpath;
        do
        {
            eval_path_ss.str("");
            eval_path_ss << "/home/thomas/Projects/thomas.faeulhammer/" << "eval_" << id << "/";
            eval_folderpath = eval_path_ss.str();
            id++;
        }while(boost::filesystem::exists(eval_folderpath) );
        boost::filesystem::create_directory(eval_folderpath);

        std::string eval_path = eval_path_ss.str();

        std::vector < std::string > scene_folder;
        std::string start = "";
        v4r::utils::getFoldersInDirectory(dataset_path, start, scene_folder);

        std::cout << "There are " << scene_folder.size() << " folders in directory " << dataset_path << "." << std::endl;

        for(size_t seq_id=0; seq_id < scene_folder.size(); seq_id++)
        {
            std::stringstream seq_path_ss;
            //seq_name_ss << "set_" << setw(5) << setfill('0') << seq_id;
            //seq_name_ss << "T_" << setw(2) << setfill('0') << seq_id << "_willow_dataset";
            seq_path_ss << dataset_path << "/" << scene_folder[seq_id];

            if(sequence_name_provided.length() && sequence_name_provided.compare(scene_folder[seq_id])!=0)
                continue;

//            std::stringstream scenes_dir_ss;
//            scenes_dir_ss << dataset_path << "/" << seq_name_ss.str();
//            std::string scenes_dir = scenes_dir_ss.str();


            std::cout << "Starting eval for " << seq_path_ss.str() << std::endl;
            std::vector < std::string > files_intern;
            v4r::utils::getFilesInDirectory (seq_path_ss.str(), files_intern, "", ".*.pcd", true);

            if(play_sequence_randomly)
                std::random_shuffle(files_intern.begin(), files_intern.end());
            else
                std::sort(files_intern.begin(), files_intern.end());

            if (files_intern.size())
            {
                size_t sub_id_mv = 0;
                std::stringstream or_folderpath_mv_ss;
                boost::filesystem::path or_folderpath_mv;
                do
                {
                    or_folderpath_mv_ss.str(std::string());
                    or_folderpath_mv_ss << eval_path << scene_folder[seq_id]  << "_" << sub_id_mv << "_mv/";
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
                    or_folderpath_sv_ss << eval_path << scene_folder[seq_id]  << "_" << sub_id_sv << "_sv/";
                    or_folderpath_sv = or_folderpath_sv_ss.str();
                    sub_id_sv++;
                }while(boost::filesystem::exists(or_folderpath_sv) );
                boost::filesystem::create_directory(or_folderpath_sv);

                std::stringstream or_filepath_parameter_ss_sv;
                or_filepath_parameter_ss_sv << or_folderpath_sv_ss.str() << "parameter.nfo";
                ofstream or_file;

                std::stringstream or_filepath_parameter_ss_mv;
                or_filepath_parameter_ss_mv << or_folderpath_mv_ss.str() << "parameter.nfo";
                or_file.open (or_filepath_parameter_ss_mv.str().c_str());

                or_filepath_parameter_ss_mv.str(std::string());
                or_filepath_parameter_ss_mv << or_folderpath_mv_ss.str() << "view_temporal_order.nfo";
                or_file.open (or_filepath_parameter_ss_mv.str().c_str());
                for (size_t file_id=0; file_id < files_intern.size(); file_id++)
                {
                    or_file << files_intern[file_id] << std::endl;
                }
                or_file.close();

                for (size_t file_id=0; file_id < files_intern.size(); file_id++)
                {
                    std::stringstream full_file_name;
                    full_file_name << dataset_path << "/" << scene_folder[seq_id] << "/" << files_intern[file_id];
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pScene (new pcl::PointCloud<pcl::PointXYZRGB>);
                    pcl::io::loadPCDFile(full_file_name.str(), *pScene);

                    std::vector<double> transform;
                    transform.clear();

                    std::stringstream transform_ss;
#ifndef USE_WILLOW_DATASET_FOR_EVAL
                    transform_ss << dataset_path << "/" << scene_folder[seq_id] << "/" << transform_prefix_ << files_intern[file_id].substr(0, files_intern[file_id].length() - 3) << "txt";
#else
                    transform_ss << dataset_path << "/" << scene_folder[seq_id] << "/" << "pose_" << files_intern[file_id].substr(6, files_intern[file_id].length() - 3 - 6 ) << "txt";
#endif
                    std::cout << "Checking if path " << transform_ss.str() << " for transform exists. " << std::endl;

                    if ( boost::filesystem::exists( transform_ss.str() ) )
                    {
                        std::cout << "File exists." << std::endl;
                        std::ifstream is(transform_ss.str().c_str());

#ifdef USE_WILLOW_DATASET_FOR_EVAL
                        std::string s;
                        std::vector<std::string> file_parts;
                        std::getline( is, s );
                        std::istringstream ss( s );
                        std::vector<double> numbers;
                        while (ss)
                        {
                          std::string s;
                          if (!std::getline( ss, s, ' ' )) break;
                          file_parts.push_back( s );
                          if(file_parts.size()>1)
                              numbers.push_back(atof(s.c_str()));
                        }
#else
                        std::istream_iterator<double> start(is), end;
                        std::vector<double> numbers(start, end);
                        std::cout << "Read " << numbers.size() << " numbers" << std::endl;
#endif
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
                                      scene_folder[seq_id],
                                      files_intern[file_id].substr(0, files_intern[file_id].length() - 3 - 1),
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
        myWorld.clear();    // to reduce memory load
    }
    }
    }
}
    }
#endif
    return 0;
}
