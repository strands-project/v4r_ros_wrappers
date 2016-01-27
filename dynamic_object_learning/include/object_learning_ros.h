#ifndef OBJECT_LEARNING_ROS_H__
#define OBJECT_LEARNING_ROS_H__

#include <v4r/object_modelling/do_learning.h>

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>

#include "do_learning_srv_definitions/clear.h"
#include "do_learning_srv_definitions/learn_object.h"
#include "do_learning_srv_definitions/learn_object_inc.h"
#include "do_learning_srv_definitions/save_model.h"
#include "do_learning_srv_definitions/visualize.h"
#include "do_learning_srv_definitions/write_debug_images_to_disk.h"

namespace v4r
{
namespace object_modelling
{
class DOL_ROS : public DOL
{
private:
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::ServiceServer clear_cached_model_,
                       learn_object_,
                       learn_object_inc_,
                       save_model_,
                       vis_model_,
                       write_images_to_disk_srv_;
    ros::Publisher vis_pc_pub_;

    bool visualize_intermediate_results_;

public:
    void initSIFT (int argc, char ** argv);

    bool clear_cached_model (do_learning_srv_definitions::clear::Request & req,
                     do_learning_srv_definitions::clear::Response & response);

    bool save_model (do_learning_srv_definitions::save_model::Request & req,
                     do_learning_srv_definitions::save_model::Response & response);

    bool visualizeROS(do_learning_srv_definitions::visualize::Request & req,
                        do_learning_srv_definitions::visualize::Response & response);

    bool writeImagesToDiskROS(do_learning_srv_definitions::write_debug_images_to_disk::Request & req,
                        do_learning_srv_definitions::write_debug_images_to_disk::Response & response);

    bool learn_object (do_learning_srv_definitions::learn_object::Request & req,
                       do_learning_srv_definitions::learn_object::Response & response);

    bool learn_object_inc (do_learning_srv_definitions::learn_object_inc::Request & req,
                           do_learning_srv_definitions::learn_object_inc::Response & response);

    static Eigen::Matrix4f fromGMTransform(const geometry_msgs::Transform & gm_trans)
    {
        Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();

        Eigen::Quaternionf q(gm_trans.rotation.w,
                             gm_trans.rotation.x,
                             gm_trans.rotation.y,
                             gm_trans.rotation.z);

        Eigen::Vector3f translation(gm_trans.translation.x,
                                    gm_trans.translation.y,
                                    gm_trans.translation.z);


        trans.block<3,3>(0,0) = q.toRotationMatrix();
        trans.block<3,1>(0,3) = translation;
        return trans;
    }
};


}
}

#endif //OBJECT_LEARNING_ROS_H__
