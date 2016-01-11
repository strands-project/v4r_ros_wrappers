#include <v4r_config.h>
#include <v4r/recognition/multiview_object_recognizer.h>
#include "recognition_srv_definitions/recognize.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>

namespace v4r
{

template<typename PointT>
class multiviewRecognizerROS
{
private:
    typedef Model<PointT> ModelT;
    typedef boost::shared_ptr<ModelT> ModelTPtr;
    typedef pcl::Histogram<128> FeatureT;

    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher image_pub_;
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::Publisher vis_pc_pub_;
    ros::ServiceServer recognition_serv_;
    float resolution_;

    boost::shared_ptr<MultiRecognitionPipeline<PointT> > rr_;
    boost::shared_ptr<MultiviewRecognizer<PointT> > mv_r_;

    bool visualize_;

    std::string models_dir_;

    cv::Ptr<SiftGPU> sift_;


    typename pcl::PointCloud<PointT>::Ptr scene_;

    bool respondSrvCall (recognition_srv_definitions::recognize::Request & req, recognition_srv_definitions::recognize::Response & response) const;

public:
    multiviewRecognizerROS()
    {
        scene_.reset(new pcl::PointCloud<PointT>);
        resolution_ = 0.005f;
    }

    bool recognizeROS (recognition_srv_definitions::recognize::Request & req, recognition_srv_definitions::recognize::Response & response);
    bool initialize (int argc, char ** argv);
};

}
