#include "world_representation_ros.h"
#include "pcl_conversions.h"

namespace v4r
{
bool worldRepresentationROS::respondSrvCall (recognition_srv_definitions::multiview_recognize::Request & req, recognition_srv_definitions::multiview_recognize::Response & response)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pRecognizedModels (new pcl::PointCloud<pcl::PointXYZRGB>);
//    for(size_t i = 0; i < models_mv.size(); i++)
//    {
//        std::string model_id = models_mv.at(i)->id_;
//        Eigen::Matrix4f tf = transforms_mv[i];

//        ConstPointInTPtr pModelCloud = models_mv.at (i)->getAssembled (0.005f);
//        typename pcl::PointCloud<PointT>::Ptr pModelAligned (new pcl::PointCloud<PointT>);
//        if(req.transform.size() == 16)
//        {
//            Eigen::Matrix4f global_trans;
//            for (size_t row=0; row <4; row++)
//            {
//                for(size_t col=0; col<4; col++)
//                {
//                    global_trans(row, col) = global_trans_v[4*row + col];
//                }
//            }
//            pcl::transformPointCloud (*pModelCloud, *pModelAligned, global_trans * transforms_mv[i]);
//        }
//        else
//            pcl::transformPointCloud (*pModelCloud, *pModelAligned, transforms_mv[i]);
//        *pRecognizedModels += *pModelAligned;

//        std_msgs::String ros_string;
//        ros_string.data = model_id;
//        response.ids.push_back(ros_string);

//        geometry_msgs::Transform tt;
//        tt.translation.x = tf(0,3);
//        tt.translation.y = tf(1,3);
//        tt.translation.z = tf(2,3);

//        Eigen::Matrix3f rotation = tf.block<3,3>(0,0);
//        Eigen::Quaternionf q(rotation);
//        tt.rotation.x = q.x();
//        tt.rotation.y = q.y();
//        tt.rotation.z = q.z();
//        tt.rotation.w = q.w();
//        response.transforms.push_back(tt);
//    }

//    sensor_msgs::PointCloud2  pc2;
//    pcl::toROSMsg (*pRecognizedModels, pc2);
//    pc2.header.frame_id = "map";
//    pc2.header.stamp = req.timestamp.data;
//    pc2.is_dense = false;
//    vis_pc_pub_.publish(pc2);
}

bool worldRepresentationROS::recognizeROSWrapper (recognition_srv_definitions::multiview_recognize::Request & req, recognition_srv_definitions::multiview_recognize::Response & response)
{
    if (req.cloud.data.size()==0)
    {
        ROS_ERROR("Point cloud is empty!");
        return false;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pInputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(req.cloud, *pInputCloud );
    const std::string scene_name = req.scene_name.data;
    const std::string view_name = req.view_name.data;
    const size_t timestamp = req.timestamp.data.toNSec();

//    std::stringstream save_filepath;
//    save_filepath << "/media/Data/datasets/TUW/test_set/set_00017/" << view_name << ".pcd";
//    pcl::io::savePCDFileBinary(save_filepath.str(), *pInputCloud);

    std::vector<double> global_trans_v;

    for(size_t i=0; i < req.transform.size(); i++)
        global_trans_v.push_back(req.transform[i]);

    std::vector<ModelTPtr> models_mv;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_mv;
    bool rec_error = recognize(pInputCloud, scene_name, view_name, timestamp, global_trans_v, models_mv, transforms_mv);
    respondSrvCall(req, response);


    return rec_error;
}
}
