#include "recognition_srv_definitions/get_configuration.h"
#include "recognition_srv_definitions/recognize.h"
#include "recognition_srv_definitions/retrain_recognizer.h"

#include <image_transport/image_transport.h>
#include <pcl/visualization/cloud_viewer.h>
#include <v4r/recognition/multi_pipeline_recognizer.h>

namespace v4r
{
template<typename PointT>
class RecognizerROS
{
private:
    typedef Model<PointT> ModelT;
    typedef boost::shared_ptr<ModelT> ModelTPtr;

    boost::shared_ptr<MultiRecognitionPipeline<PointT> > rr_;
    bool visualize_;
    pcl::visualization::PCLVisualizer::Ptr vis_;
    double chop_z_;
    bool debug_publish_;

    std::vector<ModelTPtr> models_verified_;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_verified_;

    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher image_pub_;
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::Publisher vis_pc_pub_;
    ros::ServiceServer recognize_;
    float resolution_;

    typename pcl::PointCloud<PointT>::Ptr scene_;

    bool respondSrvCall(recognition_srv_definitions::recognize::Request &req, recognition_srv_definitions::recognize::Response &response) const;

public:
    RecognizerROS()
    {
        visualize_ = false;
        chop_z_ = std::numeric_limits<float>::max();
        debug_publish_ = false;
        resolution_ = 0.005f;
    }

    bool recognizeROS (recognition_srv_definitions::recognize::Request & req,
                       recognition_srv_definitions::recognize::Response & response);

    bool initialize (int argc, char ** argv);
};

}
