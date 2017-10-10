#include "recognition_srv_definitions/get_configuration.h"
#include "recognition_srv_definitions/recognize.h"
#include "recognition_srv_definitions/retrain_recognizer.h"

#include <image_transport/image_transport.h>
#include <v4r/apps/ObjectRecognizer.h>
#include <v4r/recognition/object_hypothesis.h>

namespace v4r
{
template<typename PointT>
class RecognizerROS
{
private:
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher image_pub_;
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::Publisher vis_pc_pub_;
    ros::ServiceServer recognize_;

    std::vector< ObjectHypothesesGroup > object_hypotheses_; ///< recognized objects
    typename boost::shared_ptr<v4r::apps::ObjectRecognizer<PointT> > mrec_; ///< recognizer
    v4r::apps::ObjectRecognizerParameter mrec_param_;
    typename pcl::PointCloud<PointT>::Ptr scene_; ///< input cloud
    mutable v4r::Camera::Ptr camera_; ///< camera (if cloud is not organized)

    bool respondSrvCall(recognition_srv_definitions::recognize::Request &req, recognition_srv_definitions::recognize::Response &response) const;

public:
    RecognizerROS()
    {}

    bool recognizeROS (recognition_srv_definitions::recognize::Request & req,
                       recognition_srv_definitions::recognize::Response & response);

    bool initialize (int argc, char ** argv);

    /**
     * @brief setCamera
     * @param cam camera parameters (for visualization and in case input cloud is not organized)
     */
    void
    setCamera(const v4r::Camera::Ptr &cam)
    {
        camera_ = cam;
    }
};

}
