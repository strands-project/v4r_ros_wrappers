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

    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher image_pub_;
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::Publisher vis_pc_pub_;
    ros::ServiceServer recognition_serv_;

    boost::shared_ptr<MultiviewRecognizer<PointT> > mv_r_;

    bool visualize_;

    std::string models_dir_;

    float resolution_;

    typename pcl::PointCloud<PointT>::Ptr scene_;

    bool respondSrvCall (recognition_srv_definitions::recognize::Request & req, recognition_srv_definitions::recognize::Response & response) const;

public:
    multiviewRecognizerROS()
    {
        resolution_ = 0.005f;
        scene_.reset(new pcl::PointCloud<PointT>);
    }

    bool recognizeROS (recognition_srv_definitions::recognize::Request & req, recognition_srv_definitions::recognize::Response & response);
    bool initialize (int argc, char ** argv);
};

}
