#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sensor_msgs/PointCloud2.h>
#include <singleview_object_recognizer/CheckObjectPresenceAction.h>
#include <recognition_srv_definitions/recognize.h>
#include "mongodb_store/message_store.h"
#include "mongodb_store_msgs/StringPairList.h"
#include <std_msgs/Int32.h>
#include <scitos_ptu/PtuGotoAction.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions.h>

using namespace std_msgs;
using namespace mongodb_store;
using namespace mongodb_store_msgs;
using namespace std;

class CheckObjectPresenceAction
{

protected:

  boost::shared_ptr<actionlib::SimpleActionServer<singleview_object_recognizer::CheckObjectPresenceAction> > as_;
  std::string action_name_;
  ros::NodeHandle nh_;

  singleview_object_recognizer::CheckObjectPresenceFeedback feedback_;
  singleview_object_recognizer::CheckObjectPresenceResult result_;
  ros::Subscriber sub_;
  sensor_msgs::PointCloud2ConstPtr cloud_;
  bool got_cloud_;
  std::string topic_;  
  bool with_ptu_;
public:
    
  CheckObjectPresenceAction(std::string name)
  {
    as_.reset(new actionlib::SimpleActionServer<singleview_object_recognizer::CheckObjectPresenceAction>
            (nh_, name, boost::bind(&CheckObjectPresenceAction::executeCB, this, _1), false));

    //std::string topic_default = "/head_xtion/depth_registered/points";
    std::string topic_default = "/camera/depth_registered/points";
    nh_.param ("camera_topic", topic_, topic_default);

    as_->start();
    ROS_INFO("Action server started %s %s\n", name.c_str(), topic_.c_str());

    with_ptu_ = true;
  }

  ~CheckObjectPresenceAction(void)
  {

  }

  void
  getCloud (const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
      cloud_ = msg;
      got_cloud_ = true;
  }

  bool movePTU(float pan, float tilt)
  {
      ROS_INFO("Moving PTU to %f %f", pan, tilt);
      actionlib::SimpleActionClient<scitos_ptu::PtuGotoAction> ptu("/SetPTUState", true);
      ptu.waitForServer();
      ROS_INFO("PTU server is active\n");
      scitos_ptu::PtuGotoGoal ptuGoal;
      ptuGoal.pan = pan;
      ptuGoal.tilt = tilt;
      ptuGoal.pan_vel = 20; // 20 is a reasonable default choice
      ptuGoal.tilt_vel = 20; // 20 is a reasonable default choice
      ptu.sendGoal(ptuGoal);
      bool finished_before_timeout = ptu.waitForResult(ros::Duration(30.0));
      if (!finished_before_timeout)
      {
        ROS_ERROR("Failed to move the PTU.");
        feedback_.status = "Unable to move PTU";
        as_->publishFeedback(feedback_);
        result_.found = 0;
        as_->setAborted(result_);
        return false;
      }
      else
      {
          ROS_DEBUG("Managed to move PTU\n");
          return true;
      }
  }

  void executeCB(const singleview_object_recognizer::CheckObjectPresenceGoalConstPtr &goal)
  {

    //with_ptu_ = false;

    // helper variables
    got_cloud_ = false;
    result_.found = 0;

    //move pan-tilt to goal view
    if(with_ptu_)
    {
        if(!movePTU(goal->ptu_pan, goal->ptu_tilt))
            return;
    }

    //get point cloud
    feedback_.status = "Getting cloud";
    as_->publishFeedback(feedback_);

    ros::Subscriber sub_pc = nh_.subscribe (topic_, 1, &CheckObjectPresenceAction::getCloud, this);
    ros::Rate loop_rate (0.5);

    while (!got_cloud_ && ros::ok ())
    {
        if(as_->isPreemptRequested())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_->setPreempted();
            return;
        }

        ROS_INFO("Trying to get cloud from topic: %s\n", topic_.c_str());
        loop_rate.sleep ();
    }

    ROS_DEBUG("Got cloud, going to call service\n");

    //call recognition service
    feedback_.status = "Calling service";
    as_->publishFeedback(feedback_);
    ros::ServiceClient client = nh_.serviceClient<recognition_srv_definitions::recognize>("/recognition_service/mp_recognition");
    recognition_srv_definitions::recognize srv;
    srv.request.cloud = *cloud_;

    //parse service call
    std::string object_searched = goal->object_id;

    std::cout << "Looking for object with id:" << object_searched << std::endl;

    if (client.call(srv))
    {
        //parse result checking if the desired goal object is there or not, return true or false
        feedback_.status = "Parsing results from successful recognition service call";
        as_->publishFeedback(feedback_);

        std::cout << "Object ids found:" << static_cast<int>(srv.response.ids.size()) << std::endl;

        for(size_t i=0; i < srv.response.ids.size(); i++)
        {
          std::cout << "   => " << srv.response.ids[i] << std::endl;
          std::string id = srv.response.ids[i].data;
          if(object_searched.compare(id) == 0)
          {
              result_.found = 1;
              break;
          }
        }
    }
    else
    {
        std::cout << "there was an error calling the service" << std::endl;
        feedback_.status = "There was an error calling the service\n";
        as_->publishFeedback(feedback_);

    }

    if(with_ptu_)
    {
        //move pan-tilt to (0,0)
        movePTU(0,0);
    }

    feedback_.status = "Logging data";
    as_->publishFeedback(feedback_);

    std::stringstream cloud_name, result_name;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg (*cloud_, *scene);

    std::time_t result = std::time(NULL);
    cloud_name << "cloud_" << std::asctime(std::localtime(&result)) << ".pcd";
    result_name << "result_" << std::asctime(std::localtime(&result)) << ".txt";

    pcl::io::savePCDFileBinary(cloud_name.str(), *scene);

    std::ofstream outputFile(result_name.str().c_str());
    outputFile << goal->object_id << "\t" << result_.found << std::endl;
    outputFile.close();

    //log point cloud, result and object_id
    /*mongodb_store::MessageStoreProxy messageStore(nh_, "checkObjectPresence");

    std::vector< std::pair<std::string, std::string> > stored;
    // now add objects and store ids with the addition of type strings for safety. The types are not necessary unless you want to do some kind of reflection on this data later.

    std_msgs::Int32 found_result;
    found_result.data = result_.found;

    std_msgs::String object_id_ros_msg;
    object_id_ros_msg.data = goal->object_id;

    stored.push_back( std::make_pair(get_ros_type(*cloud_), messageStore.insert(*cloud_)) );
    stored.push_back( std::make_pair(get_ros_type(found_result), messageStore.insert(found_result)) );
    stored.push_back( std::make_pair(get_ros_type(object_id_ros_msg), messageStore.insert(object_id_ros_msg)) );

    StringPairList spl;
    for(auto & pair : stored) {
        spl.pairs.push_back(mongodb_store::makePair(pair.first, pair.second));
    }

    // and add some descriptive information
    mongo::BSONObjBuilder metaBuilder;
    metaBuilder.append("description", "checkObjectPresence result");
    metaBuilder.append("result_time", mongo::Date_t(ros::Time::now().toSec() * 1000));

    // and store
    messageStore.insert(spl, metaBuilder.obj());*/


    feedback_.status = "Succeeded";
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    as_->setSucceeded(result_);
    as_->publishFeedback(feedback_);

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "CheckObjectPresenceAction");

  CheckObjectPresenceAction object_presence(ros::this_node::getName());
  ros::spin();

  return 0;
}
