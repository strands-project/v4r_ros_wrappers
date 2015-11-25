#include "multiview_object_recognizer_ros.h"
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <v4r_config.h>
#include <v4r/common/miscellaneous.h>
#include <v4r/common/visibility_reasoning.h>
#include <v4r/common/pcl_opencv.h>
#include <v4r/recognition/ghv.h>
#include <v4r/recognition/hv_go_3D.h>
#include <v4r/recognition/local_recognizer.h>
#include <v4r/recognition/registered_views_source.h>

#ifdef HAVE_SIFTGPU
#include <v4r/features/sift_local_estimator.h>
#else
#include <v4r/features/opencv_sift_local_estimator.h>
#endif

#include <v4r/features/shot_local_estimator_omp.h>

namespace v4r
{
template<typename PointT>
bool
multiviewRecognizerROS<PointT>::respondSrvCall(recognition_srv_definitions::recognize::Request &req,
                            recognition_srv_definitions::recognize::Response &response) const
{
    typename pcl::PointCloud<PointT>::Ptr pRecognizedModels (new pcl::PointCloud<PointT>);
    cv::Mat_<cv::Vec3b> annotated_img;

    ConvertPCLCloud2Image<PointT>(scene_, annotated_img);

    std::vector<ModelTPtr> models_verified = mv_r_->getVerifiedModels();
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_verified = mv_r_->getVerifiedTransforms();

    for (size_t j = 0; j < models_verified.size(); j++)
    {
      std_msgs::String ss_tmp;
      ss_tmp.data = models_verified[j]->id_;
      response.ids.push_back(ss_tmp);

      Eigen::Matrix4f trans = transforms_verified[j];
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

      typename pcl::PointCloud<PointT>::ConstPtr model_cloud = models_verified[j]->getAssembled ( resolution_ );
      typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
      pcl::transformPointCloud (*model_cloud, *model_aligned, transforms_verified[j]);
      *pRecognizedModels += *model_aligned;
      sensor_msgs::PointCloud2 rec_model;
      pcl::toROSMsg(*model_aligned, rec_model);
      response.models_cloud.push_back(rec_model);

      pcl::PointCloud<pcl::Normal>::ConstPtr normal_cloud = models_verified[j]->getNormalsAssembled ( resolution_ );

      pcl::PointCloud<pcl::Normal>::Ptr normal_aligned (new pcl::PointCloud<pcl::Normal>);
      transformNormals(*normal_cloud, *normal_aligned, transforms_verified[j]);

      //ratio of inlier points
      float confidence = 0;
      const float focal_length = 525.f;
      const int img_width = 640;
      const int img_height = 480;

      VisibilityReasoning<pcl::PointXYZRGB> vr (focal_length, img_width, img_height);
      vr.setThresholdTSS (0.01f);
      /*float fsv_ratio =*/ vr.computeFSVWithNormals (scene_, model_aligned, normal_aligned);
      confidence = 1.f - vr.getFSVUsedPoints() / static_cast<float>(model_aligned->points.size());
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
      cv::putText(annotated_img, models_verified[j]->id_, text_start, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,0,255), 1, CV_AA);
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
multiviewRecognizerROS<PointT>::recognizeROS (recognition_srv_definitions::recognize::Request & req, recognition_srv_definitions::recognize::Response & response)
{
    pcl::fromROSMsg (req.cloud, *scene_);

    Eigen::Matrix4f tf = RotTrans2Mat4f(scene_->sensor_orientation_, scene_->sensor_origin_);

    // reset view point otherwise pcl visualization is potentially messed up
    Eigen::Vector4f zero_origin; zero_origin[0] = zero_origin[1] = zero_origin[2] = zero_origin[3] = 0.f;
    scene_->sensor_orientation_ = Eigen::Quaternionf::Identity();
    scene_->sensor_origin_ = zero_origin;

    mv_r_->setInputCloud (scene_);
    mv_r_->setCameraPose(tf);
    mv_r_->recognize();

    respondSrvCall(req, response);
    return true;
}

template<typename PointT>
bool
multiviewRecognizerROS<PointT>::initialize(int argc, char **argv)
{
    n_.reset( new ros::NodeHandle ( "~" ) );
    bool do_sift = true;
    bool do_shot = false;
    bool use_go3d = false;

    float resolution = 0.003f;
    std::string models_dir, training_dir;

    typename GO3D<PointT, PointT>::Parameter paramGO3D;
    typename GraphGeometricConsistencyGrouping<PointT, PointT>::Parameter paramGgcg;
    typename LocalRecognitionPipeline<flann::L1, PointT, FeatureT >::Parameter paramLocalRecSift;
    typename LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<352> >::Parameter paramLocalRecShot;
    typename MultiRecognitionPipeline<PointT>::Parameter paramMultiPipeRec;
    typename SHOTLocalEstimationOMP<PointT, pcl::Histogram<352> >::Parameter paramLocalEstimator;
    typename MultiviewRecognizer<PointT>::Parameter paramMultiView;

    paramLocalRecSift.use_cache_ = paramLocalRecShot.use_cache_ = true;
    paramLocalRecSift.save_hypotheses_ = paramLocalRecShot.save_hypotheses_ = true;
    paramLocalRecShot.kdtree_splits_ = 128;

    paramMultiPipeRec.save_hypotheses_ = true;

    n_->getParam ( "visualize", visualize_);
    n_->getParam ( "models_dir", models_dir);
    n_->getParam ( "training_dir", training_dir);
    n_->getParam ( "do_sift", do_sift);
    n_->getParam ( "do_shot", do_shot);
    n_->getParam ( "use_go3d", use_go3d);
    n_->getParam ( "knn_sift", paramLocalRecSift.knn_);
    n_->getParam ( "knn_shot", paramLocalRecShot.knn_);

    n_->getParam ( "transfer_feature_matches", paramMultiPipeRec.save_hypotheses_);

    int normal_computation_method;
    if(n_->getParam ( "normal_method", normal_computation_method) != -1)
    {
        paramLocalRecSift.normal_computation_method_ =
                paramLocalRecShot.normal_computation_method_ =
                paramMultiPipeRec.normal_computation_method_ =
                paramLocalEstimator.normal_computation_method_ =
                normal_computation_method;
    }

    int icp_iterations;
    if(n_->getParam ( "icp_iterations", icp_iterations) != -1)
        paramLocalRecSift.icp_iterations_ = paramLocalRecShot.icp_iterations_ = paramMultiPipeRec.icp_iterations_ = icp_iterations;

    n_->getParam ( "chop_z", paramMultiView.chop_z_ );
    n_->getParam ( "max_vertices_in_graph", paramMultiView.max_vertices_in_graph_ );
    n_->getParam ( "compute_mst", paramMultiView.compute_mst_ );

    n_->getParam ( "cg_size_thresh", paramGgcg.gc_threshold_);
    n_->getParam ( "cg_size", paramGgcg.gc_size_);
    n_->getParam ( "cg_ransac_threshold", paramGgcg.ransac_threshold_);
    n_->getParam ( "cg_dist_for_clutter_factor", paramGgcg.dist_for_cluster_factor_);
    n_->getParam ( "cg_max_taken", paramGgcg.max_taken_correspondence_);
    n_->getParam ( "cg_max_time_for_cliques_computation", paramGgcg.max_time_allowed_cliques_comptutation_);
    n_->getParam ( "cg_dot_distance", paramGgcg.thres_dot_distance_);
    n_->getParam ( "cg_use_graph", paramGgcg.use_graph_);
    n_->getParam ( "hv_clutter_regularizer", paramGO3D.clutter_regularizer_);
    n_->getParam ( "hv_color_sigma_ab", paramGO3D.color_sigma_ab_);
    n_->getParam ( "hv_color_sigma_l", paramGO3D.color_sigma_l_);
    n_->getParam ( "hv_detect_clutter", paramGO3D.detect_clutter_);
    n_->getParam ( "hv_duplicity_cm_weight", paramGO3D.w_occupied_multiple_cm_);
    n_->getParam ( "hv_histogram_specification", paramGO3D.use_histogram_specification_);
    n_->getParam ( "hv_hyp_penalty", paramGO3D.active_hyp_penalty_);
    n_->getParam ( "hv_ignore_color", paramGO3D.ignore_color_even_if_exists_);
    n_->getParam ( "hv_initial_status", paramGO3D.initial_status_);
    n_->getParam ( "hv_inlier_threshold", paramGO3D.inliers_threshold_);
    n_->getParam ( "hv_occlusion_threshold", paramGO3D.occlusion_thres_);
    n_->getParam ( "hv_optimizer_type", paramGO3D.opt_type_);
    n_->getParam ( "hv_radius_clutter", paramGO3D.radius_neighborhood_clutter_);
    n_->getParam ( "hv_radius_normals", paramGO3D.radius_normals_);
    n_->getParam ( "hv_regularizer", paramGO3D.regularizer_);
    n_->getParam ( "hv_plane_method", paramGO3D.plane_method_);
    n_->getParam ( "hv_add_planes", paramGO3D.add_planes_);
    n_->getParam ( "hv_regularizer", paramGO3D.regularizer_);
    n_->getParam ( "hv_plane_method", paramGO3D.plane_method_);
    n_->getParam ( "hv_add_planes", paramGO3D.add_planes_);
    n_->getParam ( "knn_plane_clustering_search", paramGO3D.knn_plane_clustering_search_);

    int min_plane_inliers;
    if(n_->getParam ( "hv_min_plane_inliers", min_plane_inliers))
        paramGO3D.min_plane_inliers_ = static_cast<size_t>(min_plane_inliers);

//        n_->getParam ( "hv_requires_normals", r_.hv_params_.requires_normals_);

    rr_.reset(new MultiRecognitionPipeline<PointT>(paramMultiPipeRec));

    boost::shared_ptr < GraphGeometricConsistencyGrouping<PointT, PointT> > gcg_alg (
                new GraphGeometricConsistencyGrouping<PointT, PointT> (paramGgcg));

    boost::shared_ptr <Source<PointT> > cast_source;
    if (do_sift || do_shot ) // for local recognizers we need this source type / training data
    {
        boost::shared_ptr < RegisteredViewsSource<pcl::PointXYZRGBNormal, PointT, PointT> > src
                (new RegisteredViewsSource<pcl::PointXYZRGBNormal, PointT, PointT>(resolution));
        src->setPath (models_dir);
        src->setModelStructureDir (training_dir);
        src->generate ();
//        src->createVoxelGridAndDistanceTransform(resolution);
        cast_source = boost::static_pointer_cast<RegisteredViewsSource<pcl::PointXYZRGBNormal, PointT, PointT> > (src);
    }

    if (do_sift)
    {
#ifdef HAVE_SIFTGPU
    static char kw[][16] = {"-m", "-fo", "-1", "-s", "-v", "1", "-pack"};
    char * argvv[] = {kw[0], kw[1], kw[2], kw[3],kw[4],kw[5],kw[6], NULL};

    int argcc = sizeof(argvv) / sizeof(char*);
    sift_ = new SiftGPU ();
    sift_->ParseParam (argcc, argvv);

    //create an OpenGL context for computation
    if (sift_->CreateContextGL () != SiftGPU::SIFTGPU_FULL_SUPPORTED)
      throw std::runtime_error ("PSiftGPU::PSiftGPU: No GL support!");

  boost::shared_ptr < SIFTLocalEstimation<PointT, FeatureT > > estimator (new SIFTLocalEstimation<PointT, FeatureT >(sift_));
  boost::shared_ptr < LocalEstimator<PointT, FeatureT > > cast_estimator = boost::dynamic_pointer_cast<SIFTLocalEstimation<PointT, FeatureT > > (estimator);
#else
  boost::shared_ptr < OpenCVSIFTLocalEstimation<PointT, FeatureT > > estimator (new OpenCVSIFTLocalEstimation<PointT, FeatureT >);
  boost::shared_ptr < LocalEstimator<PointT, FeatureT > > cast_estimator = boost::dynamic_pointer_cast<OpenCVSIFTLocalEstimation<PointT, FeatureT > > (estimator);
#endif

        boost::shared_ptr<LocalRecognitionPipeline<flann::L1, PointT, FeatureT > > sift_r;
        sift_r.reset (new LocalRecognitionPipeline<flann::L1, PointT, FeatureT > (paramLocalRecSift));
        sift_r->setDataSource (cast_source);
        sift_r->setTrainingDir (training_dir);
        sift_r->setFeatureEstimator (cast_estimator);
        sift_r->initialize (false);

        boost::shared_ptr < Recognizer<PointT> > cast_recog;
        cast_recog = boost::static_pointer_cast<LocalRecognitionPipeline<flann::L1, PointT, FeatureT > > (sift_r);
        std::cout << "Feature Type: " << cast_recog->getFeatureType() << std::endl;
        rr_->addRecognizer (cast_recog);
    }
    if (do_shot)
    {
        boost::shared_ptr<UniformSamplingExtractor<PointT> > uniform_kp_extractor ( new UniformSamplingExtractor<PointT>);
        uniform_kp_extractor->setSamplingDensity (0.01f);
        uniform_kp_extractor->setFilterPlanar (true);
        uniform_kp_extractor->setThresholdPlanar(0.1);
        uniform_kp_extractor->setMaxDistance( 1000.0 ); // for training we want to consider all points (except nan values)

        boost::shared_ptr<KeypointExtractor<PointT> > keypoint_extractor = boost::static_pointer_cast<KeypointExtractor<PointT> > (uniform_kp_extractor);
        boost::shared_ptr<SHOTLocalEstimationOMP<PointT, pcl::Histogram<352> > > estimator (new SHOTLocalEstimationOMP<PointT, pcl::Histogram<352> >(paramLocalEstimator));
        estimator->addKeypointExtractor (keypoint_extractor);

        boost::shared_ptr<LocalEstimator<PointT, pcl::Histogram<352> > > cast_estimator;
        cast_estimator = boost::dynamic_pointer_cast<LocalEstimator<PointT, pcl::Histogram<352> > > (estimator);

        boost::shared_ptr<LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<352> > > local;
        local.reset(new LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<352> > (paramLocalRecShot));
        local->setDataSource (cast_source);
        local->setTrainingDir(training_dir);
        local->setFeatureEstimator (cast_estimator);
        local->initialize (false);

        uniform_kp_extractor->setMaxDistance( paramMultiView.chop_z_ ); // for training we do not want this restriction

        boost::shared_ptr<Recognizer<PointT> > cast_recog;
        cast_recog = boost::static_pointer_cast<LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<352> > > (local);
        std::cout << "Feature Type: " << cast_recog->getFeatureType() << std::endl;
        rr_->addRecognizer(cast_recog);
    }


    if(!paramMultiPipeRec.save_hypotheses_)
        rr_->setCGAlgorithm( gcg_alg );

    boost::shared_ptr<HypothesisVerification<PointT,PointT> > cast_hv_pointer;
    if(use_go3d) {
        boost::shared_ptr<GO3D<PointT, PointT> > hyp_verification_method (new GO3D<PointT, PointT>(paramGO3D));
        cast_hv_pointer = boost::static_pointer_cast<GO3D<PointT, PointT> > (hyp_verification_method);
    }
    else {
        typename GHV<PointT, PointT>::Parameter paramGHV2 = paramGO3D;
        boost::shared_ptr<GHV<PointT, PointT> > hyp_verification_method (new GHV<PointT, PointT>(paramGHV2));
        cast_hv_pointer = boost::static_pointer_cast<GHV<PointT, PointT> > (hyp_verification_method);
    }

    mv_r_.reset(new MultiviewRecognizer<PointT>(paramMultiView));
    mv_r_->setSingleViewRecognizer(rr_);
    mv_r_->setCGAlgorithm( gcg_alg );
    mv_r_->setHVAlgorithm( cast_hv_pointer );
    mv_r_->set_sift(sift_);

    vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "multiview_recognized_objects", 0 );
    recognition_serv_ = n_->advertiseService("multiview_recognition_service", &multiviewRecognizerROS::recognizeROS, this);
    it_.reset(new image_transport::ImageTransport(*n_));
    image_pub_ = it_->advertise("multiview_recogniced_object_instances_img", 1, true);

    return 1;
}
}

int
main (int argc, char ** argv)
{
  ros::init (argc, argv, "multiview_recognition_service");

  v4r::multiviewRecognizerROS<pcl::PointXYZRGB> m;
  m.initialize (argc, argv);
  ros::spin ();

  return 0;
}
