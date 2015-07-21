#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <singleview_object_recognizer/CheckObjectPresenceAction.h>
#include <pcl/console/parse.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_object_presence");

  std::string object_id = "frucht_molke.pcd";
  pcl::console::parse_argument (argc, argv, "-object_id", object_id);
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<singleview_object_recognizer::CheckObjectPresenceAction> ac("/CheckObjectPresenceAction", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  ROS_INFO("Looking for object %s \n", object_id.c_str());

  // send a goal to the action
  singleview_object_recognizer::CheckObjectPresenceGoal goal;
  goal.object_id = object_id;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
    singleview_object_recognizer::CheckObjectPresenceResultConstPtr res = ac.getResult();
    std::cout << "The object was found:" << res->found << std::endl;
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
