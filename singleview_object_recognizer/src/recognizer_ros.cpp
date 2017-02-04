#include "recognizer_ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>

#include <v4r/common/camera.h>
#include <v4r/common/miscellaneous.h>
#include <v4r/common/normals.h>
#include <v4r/common/pcl_opencv.h>
#include <v4r/io/filesystem.h>
#include <v4r/features/esf_estimator.h>
#include <v4r/features/shot_local_estimator.h>
#include <v4r/features/sift_local_estimator.h>
#include <v4r/keypoints/uniform_sampling_extractor.h>
#include <v4r/ml/nearestNeighbor.h>
#include <v4r/ml/svmWrapper.h>
#include <v4r/recognition/local_recognition_pipeline.h>
#include <v4r/recognition/global_recognition_pipeline.h>
#include <v4r/segmentation/all_headers.h>

#include <pcl/common/time.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/passthrough.h>

#include <iostream>
#include <sstream>

#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <glog/logging.h>

namespace po = boost::program_options;

namespace v4r
{
template<typename PointT>
bool
RecognizerROS<PointT>::respondSrvCall(recognition_srv_definitions::recognize::Request &req,
                                      recognition_srv_definitions::recognize::Response &response) const
{
    typename pcl::PointCloud<PointT>::Ptr pRecognizedModels (new pcl::PointCloud<PointT>);

    // convert point cloud
    v4r::PCLOpenCVConverter<PointT> img_conv;
    img_conv.setInputCloud(scene_);
    if(!scene_->isOrganized())
    {
        v4r::Camera::Ptr kinect (  new v4r::Camera(525.f,640,480,319.5f,239.5f) );
        img_conv.setCamera( kinect );
    }
    cv::Mat annotated_img = img_conv.getRGBImage();

    for (typename ObjectHypothesis<PointT>::Ptr oh : verified_hypotheses_)
    {
        std_msgs::String ss_tmp;
        ss_tmp.data = oh->model_id_;
        response.ids.push_back(ss_tmp);

        Eigen::Matrix4f trans = oh->transform_;
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

        bool found;
        typename Model<PointT>::ConstPtr model = model_database_->getModelById("", oh->model_id_, found);
        typename pcl::PointCloud<PointT>::ConstPtr model_cloud = model->getAssembled( resolution_ );
        typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
        pcl::transformPointCloud (*model_cloud, *model_aligned, oh->transform_);
        *pRecognizedModels += *model_aligned;
        sensor_msgs::PointCloud2 rec_model;
        pcl::toROSMsg(*model_aligned, rec_model);
        response.models_cloud.push_back(rec_model);

        pcl::PointCloud<pcl::Normal>::ConstPtr normal_cloud = model->getNormalsAssembled ( resolution_ );

        pcl::PointCloud<pcl::Normal>::Ptr normal_aligned (new pcl::PointCloud<pcl::Normal>);
        transformNormals(*normal_cloud, *normal_aligned, oh->transform_);

        //ratio of inlier points
        float confidence = 0;
        const float focal_length = 525.f;
        const int img_width = 640;
        const int img_height = 480;

        //      VisibilityReasoning<pcl::PointXYZRGB> vr (focal_length, img_width, img_height);
        //      vr.setThresholdTSS (0.01f);
        //      /*float fsv_ratio =*/ vr.computeFSVWithNormals (scene_, model_aligned, normal_aligned);
        //      confidence = 1.f - vr.getFSVUsedPoints() / static_cast<float>(model_aligned->points.size());
        //      response.confidence.push_back(confidence);

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
        cv::putText(annotated_img, oh->model_id_, text_start, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,0,255), 1, CV_AA);
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

template<typename PointT>
bool
RecognizerROS<PointT>::recognizeROS(recognition_srv_definitions::recognize::Request &req,
                                    recognition_srv_definitions::recognize::Response &response)
{
    scene_.reset(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg (req.cloud, *scene_);

    pcl::PointCloud<pcl::Normal>::Ptr normals;

    if( mrec_->needNormals() || hv_)
    {
        pcl::ScopeTime t("Computing normals");
        normals.reset (new pcl::PointCloud<pcl::Normal>);
        pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
        ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
        ne.setMaxDepthChangeFactor(0.02f);
        ne.setNormalSmoothingSize(10.0f);
        ne.setInputCloud(scene_);
        ne.compute(*normals);
        mrec_->setSceneNormals( normals );
    }

    // ==== FILTER POINTS BASED ON DISTANCE =====
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (scene_);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, chop_z_);
    pass.setKeepOrganized(true);
    pass.filter (*scene_);

    if( chop_z_ > 0)
    {
        pcl::PassThrough<PointT> pass;
        pass.setFilterLimits ( 0.f, chop_z_ );
        pass.setFilterFieldName ("z");
        pass.setInputCloud (scene_);
        pass.setKeepOrganized (true);
        pass.filter (*scene_);
    }

    mrec_->setInputCloud ( scene_ );
    mrec_->recognize();
    generated_object_hypotheses_ = mrec_->getObjectHypothesis();

    hv_->setSceneCloud( scene_ );
    hv_->setNormals( normals );
    hv_->setHypotheses( generated_object_hypotheses_ );
    hv_->verify();
    verified_hypotheses_ = hv_->getVerifiedHypotheses();

    for ( const typename ObjectHypothesis<PointT>::Ptr &voh : verified_hypotheses_ )
    {
        const std::string &model_id = voh->model_id_;
        const Eigen::Matrix4f &tf = voh->transform_;
        LOG(INFO) << "********************" << model_id << std::endl << tf << std::endl << std::endl;
    }

    return respondSrvCall(req, response);
}

template<typename PointT>
bool
RecognizerROS<PointT>::initialize (int argc, char ** argv)
{
    n_.reset( new ros::NodeHandle ( "~" ) );

    bool visualize = false;

    // model database folder structure
    std::string models_dir;
    std::string cloud_fn_prefix;
    std::string indices_fn_prefix;
    std::string pose_fn_prefix;
    std::string transformation_fn;

    // pipeline setup
    bool do_sift = true;
    bool do_shot = true;
    bool do_esf = true;
    bool do_alexnet = false;
    double chop_z = std::numeric_limits<double>::max();
    std::string hv_config_xml = "cfg/hv_config.xml";
    std::string shot_config_xml = "cfg/shot_config.xml";
    std::string alexnet_config_xml  = "cfg/alexnet_config.xml";
    std::string esf_config_xml = "cfg/esf_config.xml";
    std::string camera_config_xml = "cfg/camera.xml";
    std::string depth_img_mask = "cfg/xtion_depth_mask.png";
    std::string sift_config_xml = "cfg/sift_config.xml";

    // Correspondence grouping parameters for local recognition pipeline
    float cg_size = 0.01f; // Size for correspondence grouping.
    int cg_thresh = 7; // Threshold for correspondence grouping. The lower the more hypotheses are generated, the higher the more confident and accurate. Minimum 3.

    google::InitGoogleLogging(argv[0]);

    po::options_description desc("Single-View Object Instance Recognizer\n======================================\n**Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("model_dir,m", po::value<std::string>(&models_dir)->default_value(models_dir), "Models directory")
            ("cloud_fn_prefix", po::value<std::string>(&cloud_fn_prefix)->default_value("cloud_"), "Prefix of cloud filename")
            ("indices_fn_prefix", po::value<std::string>(&indices_fn_prefix)->default_value("object_indices_"), "Prefix of object indices filename")
            ("transformation_fn", po::value<std::string>(&transformation_fn)->default_value(""), "Transformation to apply to each pose")
            ("pose_fn_prefix", po::value<std::string>(&pose_fn_prefix)->default_value("pose_"), "Prefix for the output pose filename")
            ("chop_z,z", po::value<double>(&chop_z)->default_value(chop_z, boost::str(boost::format("%.2e") % chop_z) ), "points with z-component higher than chop_z_ will be ignored (low chop_z reduces computation time and false positives (noise increase with z)")
            ("cg_thresh,c", po::value<int>(&cg_thresh)->default_value(cg_thresh), "Threshold for correspondence grouping. The lower the more hypotheses are generated, the higher the more confident and accurate. Minimum 3.")
            ("cg_size,g", po::value<float>(&cg_size)->default_value(cg_size, boost::str(boost::format("%.2e") % cg_size) ), "Size for correspondence grouping.")
            ("do_sift", po::value<bool>(&do_sift)->default_value(do_sift), "if true, enables SIFT feature matching")
            ("do_shot", po::value<bool>(&do_shot)->default_value(do_shot), "if true, enables SHOT feature matching")
            ("do_esf", po::value<bool>(&do_esf)->default_value(do_esf), "if true, enables ESF global matching")
            ("do_alexnet", po::value<bool>(&do_alexnet)->default_value(do_alexnet), "if true, enables AlexNet global matching")
            ("depth_img_mask", po::value<std::string>(&depth_img_mask)->default_value(depth_img_mask), "filename for image registration mask. This mask tells which pixels in the RGB image can have valid depth pixels and which ones are not seen due to the phsysical displacement between RGB and depth sensor.")
            ("hv_config_xml", po::value<std::string>(&hv_config_xml)->default_value(hv_config_xml), "Filename of Hypotheses Verification XML configuration file.")
            ("sift_config_xml", po::value<std::string>(&sift_config_xml)->default_value(sift_config_xml), "Filename of SIFT XML configuration file.")
            ("shot_config_xml", po::value<std::string>(&shot_config_xml)->default_value(shot_config_xml), "Filename of SHOT XML configuration file.")
            ("alexnet_config_xml", po::value<std::string>(&alexnet_config_xml)->default_value(alexnet_config_xml), "Filename of Alexnet XML configuration file.")
            ("esf_config_xml", po::value<std::string>(&esf_config_xml)->default_value(esf_config_xml), "Filename of ESF XML configuration file.")
            ("camera_xml", po::value<std::string>(&camera_config_xml)->default_value(camera_config_xml), "Filename of camera parameter XML file.")
            ("visualize,v", po::bool_switch(&visualize), "visualize recognition results")
            ;
    po::variables_map vm;
    po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
    std::vector<std::string> to_pass_further = po::collect_unrecognized(parsed.options, po::include_positional);
    po::store(parsed, vm);
    if (vm.count("help")) { std::cout << desc << std::endl; to_pass_further.push_back("-h"); }
    try { po::notify(vm); }
    catch(std::exception& e) { std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;  }

    // ====== DEFINE CAMERA =======
    Camera::Ptr xtion (new Camera(camera_config_xml) );

    cv::Mat_<uchar> img_mask = cv::imread(depth_img_mask, CV_LOAD_IMAGE_GRAYSCALE);
    if( img_mask.data )
        xtion->setCameraDepthRegistrationMask( img_mask );
    else
        std::cout << "No camera depth registration mask provided. Assuming all pixels have valid depth." << std::endl;


    // ==== Fill object model database ==== ( assumes each object is in a seperate folder named after the object and contains and "views" folder with the training views of the object)
    model_database_.reset (new Source<PointT> (models_dir));

    // ====== SETUP MULTI PIPELINE RECOGNIZER ======
    mrec_.reset(new MultiRecognitionPipeline<PointT>);
    typename LocalRecognitionPipeline<PointT>::Ptr local_recognition_pipeline (new LocalRecognitionPipeline<PointT>);
    {
        // ====== SETUP LOCAL RECOGNITION PIPELINE =====
        if(do_sift || do_shot)
        {
            local_recognition_pipeline->setModelDatabase( model_database_ );
            boost::shared_ptr< pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> > gc_clusterer
                    (new pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ>);
            gc_clusterer->setGCSize( cg_size );
            gc_clusterer->setGCThreshold( cg_thresh );
            local_recognition_pipeline->setCGAlgorithm( gc_clusterer );

            if(do_sift)
            {
                LocalRecognizerParameter sift_param(sift_config_xml);
                typename LocalFeatureMatcher<PointT>::Ptr sift_rec (new LocalFeatureMatcher<PointT>(sift_param));
                typename SIFTLocalEstimation<PointT>::Ptr sift_est (new SIFTLocalEstimation<PointT>);
                sift_est->setMaxDistance(std::numeric_limits<float>::max());
                sift_rec->setFeatureEstimator( sift_est );
                local_recognition_pipeline->addLocalFeatureMatcher(sift_rec);
            }
            if(do_shot)
            {
                typename SHOTLocalEstimation<PointT>::Ptr shot_est (new SHOTLocalEstimation<PointT>);
                typename UniformSamplingExtractor<PointT>::Ptr extr (new UniformSamplingExtractor<PointT>(0.02f));
                typename KeypointExtractor<PointT>::Ptr keypoint_extractor = boost::static_pointer_cast<KeypointExtractor<PointT> > (extr);

                LocalRecognizerParameter shot_param(shot_config_xml);
                typename LocalFeatureMatcher<PointT>::Ptr shot_rec (new LocalFeatureMatcher<PointT>(shot_param));
                shot_rec->addKeypointExtractor( keypoint_extractor );
                shot_rec->setFeatureEstimator( shot_est );
                local_recognition_pipeline->addLocalFeatureMatcher(shot_rec);
            }

            typename RecognitionPipeline<PointT>::Ptr rec_pipeline_tmp = boost::static_pointer_cast<RecognitionPipeline<PointT> > (local_recognition_pipeline);
            mrec_->addRecognitionPipeline(rec_pipeline_tmp);
        }

        // ====== SETUP GLOBAL RECOGNITION PIPELINE =====
        if(do_esf || do_alexnet)
        {
            typename GlobalRecognitionPipeline<PointT>::Ptr global_recognition_pipeline (new GlobalRecognitionPipeline<PointT>);
            // choose segmentation type
            typename DominantPlaneSegmenter<PointT>::Parameter param;
            std::vector<std::string> not_used_params = param.init(to_pass_further);
            typename DominantPlaneSegmenter<PointT>::Ptr seg (new DominantPlaneSegmenter<PointT> (param));
            //        typename EuclideanSegmenter<PointT>::Ptr seg (new EuclideanSegmenter<PointT> (argc, argv));
            //        typename SmoothEuclideanSegmenter<PointT>::Ptr seg (new SmoothEuclideanSegmenter<PointT> (argc, argv));

            if(do_esf)
            {
                typename ESFEstimation<PointT>::Ptr esf_estimator (new ESFEstimation<PointT>);

                // choose classifier
                //                 NearestNeighborClassifier::Ptr classifier (new NearestNeighborClassifier);
                svmClassifier::Ptr classifier (new svmClassifier);
                //                classifier->setInFilename(esf_svm_model_fn);

                GlobalRecognizerParameter esf_param (esf_config_xml);
                typename GlobalRecognizer<PointT>::Ptr global_r (new GlobalRecognizer<PointT>(esf_param));
                global_r->setFeatureEstimator(esf_estimator);
                global_r->setClassifier(classifier);
                global_recognition_pipeline->addRecognizer(global_r);
                global_recognition_pipeline->setSegmentationAlgorithm(seg);
            }

            if (do_alexnet)
            {
                std::cerr << "Not implemented right now!" << std::endl;
            }

            typename RecognitionPipeline<PointT>::Ptr rec_pipeline_tmp = boost::static_pointer_cast<RecognitionPipeline<PointT> > (global_recognition_pipeline);
            mrec_->addRecognitionPipeline( rec_pipeline_tmp );
        }

        mrec_->setModelDatabase( model_database_ );
        mrec_->initialize( models_dir, false );
    }


    // ====== SETUP HYPOTHESES VERIFICATION =====
    HV_Parameter paramHV (hv_config_xml);
    hv_.reset (new HypothesisVerification<PointT, PointT> (xtion, paramHV) );
    hv_->setModelDatabase(model_database_);

    vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "sv_recogniced_object_instances", 1 );
    recognize_  = n_->advertiseService ("sv_recognition", &RecognizerROS::recognizeROS, this);

    it_.reset(new image_transport::ImageTransport(*n_));
    image_pub_ = it_->advertise("sv_recogniced_object_instances_img", 1, true);

    ROS_INFO("Ready to get service calls.");
    return true;
}

}

int
main (int argc, char ** argv)
{
    ros::init (argc, argv, "recognition_service");
    v4r::RecognizerROS<pcl::PointXYZRGB> m;
    m.initialize (argc, argv);
    ros::spin ();
    return 0;
}
