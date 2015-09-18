#include <v4r/recognition/multiview_object_recognizer.h>
#include "recognition_srv_definitions/recognize.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>

namespace v4r
{

class multiviewGraphROS : public MultiviewRecognizer
{
private:
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher image_pub_;
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::Publisher vis_pc_pub_;
    ros::ServiceServer recognition_serv_;
    float resolution_;
    size_t view_counter_;

    bool respondSrvCall (recognition_srv_definitions::recognize::Request & req, recognition_srv_definitions::recognize::Response & response) const;

public:
    multiviewGraphROS() : MultiviewRecognizer()
    {
        resolution_ = 0.005f;
        view_counter_ = 0;
    }

    bool recognizeROS (recognition_srv_definitions::recognize::Request & req, recognition_srv_definitions::recognize::Response & response);
    bool initializeMV (int argc, char ** argv);
};

}
