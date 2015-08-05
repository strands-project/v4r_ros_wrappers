#include "recognition_srv_definitions/get_configuration.h"
#include "recognition_srv_definitions/recognize.h"
#include "recognition_srv_definitions/retrain_recognizer.h"

#include <v4r/recognition/singleview_object_recognizer.h>

#include <image_transport/image_transport.h>

namespace v4r
{
class RecognizerROS : public SingleViewRecognizer
{
    using v4r::SingleViewRecognizer::initialize;

private:
    bool debug_publish_;

    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher image_pub_;
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::Publisher vis_pc_pub_;
    ros::ServiceServer recognize_;
    float resolution_;

    bool respondSrvCall(recognition_srv_definitions::recognize::Request &req, recognition_srv_definitions::recognize::Response &response) const;


public:

    RecognizerROS() : SingleViewRecognizer()
    {
        debug_publish_ = false;
        resolution_ = 0.005f;
    }

    bool
    getConfig (recognition_srv_definitions::get_configuration::Request & req,
             recognition_srv_definitions::get_configuration::Response & response)
    {
          response.models_folder.data = models_dir_;
          response.recognition_structure_folder.data = sift_structure_;
          return true;
    }


    bool retrainROS (recognition_srv_definitions::retrain_recognizer::Request & req,
             recognition_srv_definitions::retrain_recognizer::Response & response);

    bool recognizeROS (recognition_srv_definitions::recognize::Request & req,
                    recognition_srv_definitions::recognize::Response & response);

    void initialize (int argc, char ** argv);
};

}
