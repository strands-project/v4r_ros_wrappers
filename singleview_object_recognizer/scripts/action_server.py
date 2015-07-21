#!/usr/bin/python
import rospy

import actionlib

from singleview_object_recognizer.msg import *
from sensor_msgs.msg import PointCloud2,  JointState
import math
from recognition_srv_definitions.srv import * 
from mongodb_store.message_store import MessageStoreProxy
from robblog.msg import RobblogEntry
from robblog import utils as rb_utils
from std_msgs.msg import String

from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState
from geometry_msgs.msg import PoseWithCovarianceStamped

from world_state.observation import MessageStoreObject, Observation, TransformationStore
from world_state.identification import ObjectIdentification
from world_state.state import World, Object

import cv
import cv2
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError

from mongodb_store.message_store import MessageStoreProxy

class ActionServer(object):
  # create messages that are used to publish feedback/result
  _feedback = CheckObjectPresenceFeedback()
  _result   = CheckObjectPresenceResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, CheckObjectPresenceAction,
                                            execute_cb=self.execute_cb, auto_start = False)
    self._ptu_pub =  rospy.Publisher("/ptu/cmd", JointState)
    self._ptu_angles =  JointState()
    self._ptu_angles.name = ['pan', 'tilt']
    self._ptu_angles.velocity = [0.6, 0.6]
    rospy.loginfo("Waiting for service '/recognition_service/mp_recognition'")
    rospy.wait_for_service('/recognition_service/mp_recognition')

    self.id_service = rospy.ServiceProxy("/recognition_service/mp_recognition",
                                         recognize)
    self.blog_msg_store = MessageStoreProxy(collection='robblog')
    self._as.start()
    rospy.loginfo("Action server up: %s"%self._action_name)
    
  
  def command_ptu(self, pan, tilt):
    self._ptu_angles.position = [pan / 180.0 * math.pi, tilt / 180.0 *  math.pi]
    self._ptu_pub.publish(self._ptu_angles)
    rospy.sleep(5)
    
  def send_feedback(self, txt):
    self._feedback.status = txt
    self._as.publish_feedback(self._feedback)
    rospy.loginfo(txt)
    
  def execute_cb(self, goal):
    # move the ptu
    self.send_feedback("Moving PTU to %f,  %f"%(goal.ptu_pan, goal.ptu_tilt))
    self.command_ptu(goal.ptu_pan, goal.ptu_tilt)
    
    observation =  Observation.make_observation(topics=[("/amcl_pose", PoseWithCovarianceStamped),
                                                          ("/head_xtion/rgb/image_color", Image), 
                                                          ("/head_xtion/rgb/camera_info", CameraInfo), 
                                                          ("/head_xtion/depth_registered/points", PointCloud2),
                                                          ("/head_xtion/depth/camera_info", CameraInfo),
                                                          ("/ptu/state", JointState)])

    self.send_feedback("Getting pointcloud")
    try:
#      cloud = rospy.wait_for_message("/head_xtion/depth_registered/points", PointCloud2, 5)
      cloud = observation.get_message('/head_xtion/depth_registered/points')
    except:
      self.send_feedback("Failed to get a pointcloud")
      self._result.found = 0
      self._as.set_succeeded(self._result)
      return

    
    self.send_feedback("Returning PTU home")
    self.command_ptu(0, 0)
      
    object_searched = goal.object_id
    rospy.loginfo("Looking for object with id: %s"%object_searched)

    self.send_feedback("Calling object identification service.")
    try:
      result = self.id_service(cloud)
    except:
      self._result.found = 0
      self._as.set_succeeded(self._result)
      return

    rospy.loginfo("Number of objects found: %s"%len(result.ids))
    found = False
    world = World()
    objects=""
    for i in result.ids:
      rospy.loginfo(" - found %s"%i.data)
      objects+="* %s\n"%i.data[:-4]
      
      # TODO: Should project all other objects into observation and "kill" if not visible
      new_object =  world.create_object()
                    
      # TODO: store object pose etc
      
      # TODO: the classification should include class info

      classification = {}
      identification = {i.data[:-4]:1}
      
      new_object.add_identification("ObjectInstanceRecognition",
                                    ObjectIdentification(classification, identification))
      

      if i.data == object_searched:
        found = True
        break
      
    if found:
      rospy.loginfo("Found %s"%object_searched)
      self._result.found = 1
      self._as.set_succeeded(self._result)
    else:
      rospy.loginfo("Did not find %s"%object_searched)
      self._result.found = 0
      self._as.set_succeeded(self._result)

    try:
      waypoint=rospy.wait_for_message("/current_node",String,5)
    except:
      waypoint=String("-")
    title = 'Fire Extinguisher Check'
    body = 'I checked for `%s` at %s and it was '%(object_searched[:-4],waypoint.data)
    if found:
      body += '*present*'
    else:
      body += '*not in place*'
    body += '.\n\nHere is an image:\n\n'

    msg_store_blog = MessageStoreProxy(collection='robblog')
    img = observation.get_message('/head_xtion/rgb/image_color')
    img_id = msg_store_blog.insert(img)
    body += '![Image of the location](ObjectID(%s))' % img_id
    if len(objects)>0:
      body+='\n\nI found:\n\n'
      body+=objects
          
    e = rb_utils.create_timed_entry(title=title, body=body)
    self.blog_msg_store.insert(e)


      
if __name__ == '__main__':
  rospy.init_node('CheckObjectPresenceAction')
  ActionServer(rospy.get_name())
  rospy.spin()
