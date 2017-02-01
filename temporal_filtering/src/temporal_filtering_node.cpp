#include <iostream>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>

#include <v4r/keypoints/temporal_smoothing_filter.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>



#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <std_srvs/SetBool.h>

boost::mutex toggle_mutex; // The iostreams are not guaranteed to be thread-safe!

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloud2;

v4r::TemporalSmoothingFilter filter;

Eigen::Matrix4f pose, inv_pose;

pcl::PointCloud<pcl::PointXYZRGBNormal> filtered_cloud;

int frameCount=0;
string point_cloud_frame_id = "/temporal_filtered_cloud";
ros::Time point_cloud_time;
bool isFiltering=false;
ros::NodeHandlePtr pnh=NULL;
ros::Subscriber sub;
ros::Publisher filtered_pub;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud){
    //pcl_conversions::toPCL
    frameCount++;
    //if(frameCount%100==0){
    //    ROS_INFO("Retrieved %d frames since start",frameCount);
    //}

    //
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud,pcl_pc2);
    point_cloud_time=cloud->header.stamp;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    //Run the filter
    filter.filter(*temp_cloud,filtered_cloud,pose);

    //time to publish
    sensor_msgs::PointCloud2 output;
    //publish something
    filtered_pub = pnh->advertise<sensor_msgs::PointCloud2>("/temporal_filtered_cloud", 1);//do not buffer this that much please... (aint nobody got RAM for that)

    pcl::toROSMsg(filtered_cloud,output);
    output.header.frame_id = point_cloud_frame_id; // Set the header values of the ROS message
    output.header.stamp = point_cloud_time;
    output.header.frame_id = "filtered_cloud";

    filtered_pub.publish(output);
}


void startFiltering()
{
    //subscribe already
    ROS_INFO("Start temporal filtering.");
    sub=pnh->subscribe("/camera/depth_registered/points",1,callback);
}

void stopFiltering()
{
    sub.shutdown();
    ROS_INFO("Stop temporal filtering.");
    frameCount=0;
}

bool service_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response& response)
{
    boost::mutex::scoped_lock(toggle_mutex);
    if(req.data==true)
    {
        startFiltering();
        response.success=true;
        response.message="enabled";
    }
    else
    {
        stopFiltering();
        response.success=true;
        response.message="disabled";
    }
    return true;
}


//camera calibration:
//http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
//http://wiki.ros.org/image_transport#Subscribed_Topics

int main (int argc, char** argv) {
    ros::init(argc, argv, "temporal_filtering");
    pnh = ros::NodeHandlePtr(new ros::NodeHandle("~"));
    bool startFilter=false;
    //start filtering at startup
    pnh->getParam("start", startFilter);
    


    //setup the filter:
    //these parameters should come from launch files
    cv::Mat_<double> intrinsic = cv::Mat_<double>::eye(3,3);
    intrinsic(0,0)=intrinsic(1,1)=570.34;
    intrinsic(0,2)=314.5, intrinsic(1,2)=235.5;
    filter.setCameraParameter(intrinsic);
    v4r::TemporalSmoothingFilter::Parameter param;
    param.global_map_size = 0;//why should this be 0? maybe increase this value?
    //start service
    ros::ServiceServer service=pnh->advertiseService("enableFilter", service_callback);

    if(startFilter)
    {
       startFiltering();
       ROS_INFO("Temporal filtering service is up. Disable filtering via call.");
    }
    else
    {
       ROS_INFO("Temporal filtering service is running. Enable filtering via call.");
    }
    
    ros::spin();
    return 0;
}
