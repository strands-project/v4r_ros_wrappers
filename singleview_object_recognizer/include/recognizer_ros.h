#include "recognition_srv_definitions/get_configuration.h"
#include "recognition_srv_definitions/recognize.h"
#include "recognition_srv_definitions/retrain_recognizer.h"

#include <image_transport/image_transport.h>
#include <v4r/recognition/hypotheses_verification.h>
#include <v4r/recognition/multi_pipeline_recognizer.h>
#include <v4r/recognition/source.h>

namespace v4r
{
template<typename PointT>
class RecognizerROS
{
private:
//    typedef Model<PointT> ModelT;
//    typedef boost::shared_ptr<ModelT> ModelTPtr;

//    boost::shared_ptr<MultiRecognitionPipeline<PointT> > rr_;
//    bool visualize_;
//    pcl::visualization::PCLVisualizer::Ptr vis_;
    double chop_z_;
//    bool debug_publish_;


    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher image_pub_;
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::Publisher vis_pc_pub_;
    ros::ServiceServer recognize_;

    std::vector<ObjectHypothesesGroup<PointT> > generated_object_hypotheses_;
    std::vector<typename ObjectHypothesis<PointT>::Ptr > verified_hypotheses_;

    typename MultiRecognitionPipeline<PointT>::Ptr mrec_;
    typename HypothesisVerification<PointT, PointT>::Ptr hv_;
    typename Source<PointT>::Ptr model_database_;
    float resolution_; ///< resolution of the object model to be transferred

    typename pcl::PointCloud<PointT>::Ptr scene_;

    bool respondSrvCall(recognition_srv_definitions::recognize::Request &req, recognition_srv_definitions::recognize::Response &response) const;

public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RecognizerROS()
    {
//        visualize_ = false;
        chop_z_ = std::numeric_limits<float>::max();
//        debug_publish_ = false;
//        resolution_ = 0.005f;
    }

    bool recognizeROS (recognition_srv_definitions::recognize::Request & req,
                       recognition_srv_definitions::recognize::Response & response);

    bool initialize (int argc, char ** argv);
};

}
