#include <v4r/ORRecognition/include/world_representation.h>
#include "ros/ros.h"

#include "recognition_srv_definitions/multiview_recognize.h"

namespace v4r
{
class worldRepresentationROS : public worldRepresentation
{
private:
    ros::Publisher vis_pc_pub_;
    bool respondSrvCall (recognition_srv_definitions::multiview_recognize::Request & req, recognition_srv_definitions::multiview_recognize::Response & response);

public:
    bool recognizeROSWrapper (recognition_srv_definitions::multiview_recognize::Request & req, recognition_srv_definitions::multiview_recognize::Response & response);

    void set_vis_pc_pub(const ros::Publisher &vis_pc_pub)
    {
        vis_pc_pub_ = vis_pc_pub;
    }
};
}
