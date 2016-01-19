# Overview
In this tutorial you will learn how to recognise and track trained 3D object models. You will first model objects by rotating them in front of an RGB-D camera. These objects can then be recognised by a recognition component via a ROS service call. You will also learn how much you can (or can't) trust recogniser responses.

This tutorial requires installation of a specific vision library, which is part of the STRANDS repositories.

# The V4R (Vision for Robotics) library

**This is now released as Ubuntu packages as well, so there should be no need to install this from source as detailed below. Just install with `sudo apt-get install ros-indigo-v4r-ros-wrappers` and you should be ready to go! Jump straight to [[Tutorial.md#object-modelling]].**


This library is part of the STRANDS repositories 'https://github.com/strands-project/v4r'. The library itself is independent of ROS, so it is built outside ROS catkin. There are wrappers for ROS 'https://github.com/strands-project/v4r_ros_wrappers', which can then be placed inside the normal catkin workspace.

## Dependencies
* openni drivers: `sudo apt-get install libopenni-dev libopenni-sensor-primesense0 libopenni2-dev`
* Qt 4 (http://qt-project.org/): `sudo apt-get install libqt4-dev`
* Boost (http://www.boost.org/): comes with ubuntu
* Point Cloud Library 1.7.x (http://pointclouds.org/): comes with ROS
* Eigen3 (http://eigen.tuxfamily.org/): `sudo apt-get install libeigen3-dev`
* OpenCV 2.x (http://opencv.org/): comes with ROS
* Ceres Solver 1.9.0 (http://ceres-solver.org/): `sudo apt-get install libceres-dev`
* OpenGL GLSL mathematics (libglm-dev): `sudo apt-get install libglm-dev`
* GLEW - The OpenGL Extension Wrangler Library (libglew-dev): `sudo apt-get install libglew-dev`
* libraries for sparse matrices computations (libsuitesparse-dev): `sudo apt-get install libsuitesparse-dev`
* Google Logging Library `sudo apt-get install libgoogle-glog-dev`
* X-related libraries `sudo apt-get install libx11-dev libxinerama-dev libxi-dev libxrandr-de libassimp-dev`

## Installation
Clone 'https://github.com/strands-project/v4r' somewhere on your computer, e.g. `~/somewhere/v4r/') and build using cmake:
```
cd ~/somewhere
git clone 'https://github.com/strands-project/v4r'
cd v4r
mkdir build
cd build
cmake ..
make
sudo make install (optional)
```
Now you can install the STRANDS ros wrappers. Clone v4r_ros_wrappers into your catkin workspace:
```
cd my_catkin_ws/src
git clone https://github.com/strands-project/v4r_ros_wrappers.git
cd ..
```
Then call catkin_make once. If you chose not to install the V4R library (optional point above) you will get an error regarding V4RModules.cmake. This is easy to fix by locating the build/install directory of V4R and setting it appropriately within cmake:
```
catkin_make -DV4R_DIR=/path/to/your/v4r
```
It should compile fine now. Eventually you have to source the workspace with 
```
source devel/setup.bash
```

# Object modelling
First you have to model all the objects you want to recognise later. You use an offline tool for this, the RTM Toolbox.

* Place the object on a flat surface on a newspaper or something similar. This allows you to rotate the object without touching it. The texture on the newspaper also helps view registration. The pictures below were taken with the object on a turn table, which is the most convenient way of rotating the object. You can also model objects without a turntable by putting the object on the floor for instance and moving your camera. In this case just skip the step definining the ROI.
* Start the modelling tool: ~/somewhere/v4r/bin/RTMT
* Press "Camera Start": You should now see the camera image  
  ![](images/object-modeling-01.png)
* Press "ROI Set" + click on the flat surface next to the object  
  ![](images/object-modeling-02.png)
* Press Tracker Start: you now see the tracking quality bar top left  
  ![](images/object-modeling-03.png)
* Rotate 360 degrees, the program will generate a number of keyframes.  
  **IMPORTANT:** Do not touch the object itself while moving it.  
  ![](images/object-modeling-04.png)
  ![](images/object-modeling-05.png)
* Press "Tracker Stop"
* Press "Camera Stop"
* Press "Pose Optimize" (optional): bundle adjustment of all cameras (be patient!)
* Press "Object Segment": The object should already be segmented correctly thanks to the ROI set previously  
  ![](images/object-modeling-06.png)
  You can check segmentation (< > buttons). Highlighted areas are segments that will be clustered to the object model. If some highlighted areas do not belong to the object (e.g. supporting plane), you can click on these wrong segmentations to undo. You can also click on dark areas to add these segments to the object model. These areas should automatically be projected into the other frames.
* Press "Object ...Ok"  
  ![](images/object-modeling-07.png)
  ![](images/object-modeling-08.png)
* Press "Store for Recognizer": saves the point clouds in a format for object recognition. You will be asked for an model name.  
  By default the program will store models in various subfolders of the folder "./data", which will be created if not present. This can be changed in the configuration options (see below).
* Press "Store for Tracker": save a different model suitable for tracking
* If the 3D point cloud visualization is activated +/- can be used to increase/ decrease the size of dots

This is a convenient way to model objects with the STRANDS robots. Put the objects on something elevated (a trash can in this case) to bring it within a good distance to the robot's head camera.

![](images/object-modeling-09.jpg)

## Configuration options:
* Set data folder and model name:  
  (File -> Preferences -> Settings -> Path and model name)
* Configure number of keyfames to be selected using a camera rotation and a camera translation threshold:  
  (File -> Preferences -> Settings -> Min. delta angle, Min. delta camera distance)

## Trouble shooting
* If you press any of the buttons in the wrong order, just restart. Recovery is futile.
* If you do not get an image, there is a problem with the OpenNI device driver.
Check the file `/etc/openni/GlobalDefaults.ini`, set `UsbInterface=2` (i.e. BULK).
* If the plane supporting the object is not removed completely, try to increase the inlier distance for dominant plane segmentation in File -> Preferences -> Postprocessing.

# Object recognition
With the models created above you can now call the object recognition service within the STRANDS system.

Start the object recogniser on the side PC with the head camera attached. To do this, you must SSH into the side PC *without* X forwarding then run:
```
export DISPLAY=:0
rosrun singleview_object_recognizer recognition_service -m /path/to/your/model/data
```
If you want to get a description about other parameters, just add -h to the command. The ones you might want to change are:
* `-z 2.5` (will neglect all points further away than 2.5m - this will ensure the noise level of the measured points is not too high. Note that RGB-D data from the Asus or Kinect gets worse with distance)
* `--knn_sift 3` (this will increase the number of generated hypotheses)
* `--do_shot true` (this will enable SHOT feature extraction - necessary for objects without visual texture information)
* `-c 5` (increase speed at the cost of possibly missing objects if they are e.g. half occluded.)

The recogniser offers a service `/recognition_service/sv_recognition`, defined in `v4r_ros_wrappers/recognition_srv_definitions/srv`:
```
sensor_msgs/PointCloud2 cloud  # input scene point cloud
float32[] transform            # optional transform to world coordinate system
std_msgs/String scene_name     # optional name for multiview recognition
std_msgs/String view_name      # optional name for multiview recognition
---
std_msgs/String[] ids                 # name of the recognised object model
geometry_msgs/Transform[] transforms  # 6D object pose
float32[] confidence                  # ratio of visible points
geometry_msgs/Point32[] centroid      # centroid of the cluster
object_perception_msgs/BBox[] bbox    # bounding box of the cluster
sensor_msgs/PointCloud2[] models_cloud  # point cloud of the model transformed into camera coordinates
```
For you, all you have to provide is a point cloud. The recogniser will return arrays of ids (the name you gave during modelling), transforms (the 6D object poses), as well as confidences, bounding boxes and the segmented point clouds corresponding to the recognised portions of the scene.

There is a test ROS component for you as an example:
```
rosrun singleview_object_recognizer test_single_view_recognition_from_file _topic:=/camera/depth_registered/points
```
where you have to set the topic to the respective RGB-D source of your robot, e.g. the head_xtion.

The recogniser publishes visualisation information.

* `/recognition_service/sv_recogniced_object_instances_img`: displays the original image with overlaid bounding boxes of recognised objects
* `/recognition_service/sv_recogniced_object_instances`: the model point clouds of the recognised objects, in the camera frame.
The following picture shows an example where R2-D2 is detected in a shelf, with the debug picture and recognised model displayed in rviz.
![](images/object-recognition-01.png)

## Recognition performance
The modelling tool provides full 3D object models from all the views you provided, which in principle allow the recogniser to recogise the object in any condition dfrom any view.
Practically, however, recognition performance depends on several factors:
* Distance: Performance can quite rapidly decline with distance. First, because the object features on which the recogniser depends become too small (no way that it could detect an object just a few pixels large). Second, because the depth data, on which a pose verification step in the recogniser depends, becomes more and more noisy (and close to useless beyond 3 m or so)
* Lighting conditions: In principle the object features are lighting invariant. Practically, different characteristics of the camera which was used for modelling and the camera used for recognition can affect the appearance of the object features.
* Motion blur: The robot might be moving while it taske a picture. Motion blur will deteriorate object feature extraction.
* Occlusions: Objects might not be entirely visible. The recogniser does not need all object features, so it can handle a certain amount of occlusion. How much, depends on the object and how many features it is has.
* Object specifics: Some objects are harder to detect than others, e.g. because they have few features, are small, have a somewhat shiny surface, or are non-rigid.

Before using the recogniser in any object search scenario it is therefore important to gather some statistics about the recognition performance. The recogniser's confidence value can be a useful measure. But don't trust it too much -it is not an actual probability.

Useful aspects to learn are:
* How fast the recognition rate (in how many cases is the object found) drops with distance.
* How false positive rate and confidence measure are related.

## Trouble shooting
* If you get an error like this
```
terminate called after throwing an instance of 'flann::FLANNException'
  what():  Saved index does not contain the dataset and no dataset was provided.
[recognition_service-2] process has died [pid 17534, exit code -6, cmd /home/mz/work/STRANDS/code/catkin_ws/devel/lib/singleview_object_recognizer/recognition_service __name:=recognition_service __log:=/home/mz/.ros/log/61fb16b8-4afc-11e5-801d-503f56004d09/recognition_service-2.log].
log file: /home/mz/.ros/log/61fb16b8-4afc-11e5-801d-503f56004d09/recognition_service-2*.log
```
locate the file `sift_flann.idx`, probably right in your catkin workspace or in `~/.ros`, and remove it.
Do the same if you get some `vector::_M_range_check` error. In case you enable SHOT, please also remove `shot_omp_flann.idx`. 

* Please also make sure that your camera uses VGA resolution for both RGB and depth. You can check the values for `color_mode` and `depth_mode` in `rosrun rqt_reconfigure rqt_reconfigure` (camera -> driver).

* Also ensure you turned on depth\_registration (i.e. start the camera with `roslaunch openni2_launch openni2.launch depth_registration:=true`)

# Object tracker
If you stored you object model for tracking, you can track single objects in real-time using the object-tracker module. You can start the service by 
```
rosrun object_tracker object_tracker_service -m /path/to/your/model/data/object_name/tracking_model.ao
```

The tracker will start as soon as you call the service 
```
rosservice call /object_tracker/start_recording
```

This will publish topics for the 3D object pose and the confidence of this estimate
```
rostopic echo /object_tracker/object_pose
rostopic echo /object_tracker/object_tracker_confidence
```

To visualize the pose, you can use RVIZ and check out the image topic `/object_tracker/debug_images`.

To stop the tracker call
```
rosservice call /object_tracker/stop_recording
rosservice call /object_tracker/cleanup
```

# References

When referencing this work, pleace cite:

1. J. Prankl, A. Aldoma Buchaca, A. Svejda, M. Vincze, RGB-D Object Modelling for Object Recognition and Tracking. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2015.

2. Thomas Fäulhammer, Aitor Aldoma, Michael Zillich and Markus Vincze
Temporal Integration of Feature Correspondences For Enhanced Recognition in Cluttered And Dynamic Environments
IEEE International Conference on Robotics and Automation (ICRA), Seattle, WA, USA, 2015.

3. Thomas Fäulhammer, Michael Zillich and Markus Vincze
Multi-View Hypotheses Transfer for Enhanced Object Recognition in Clutter,
IAPR International Conference on Machine Vision Applications (MVA), Tokyo, Japan, 2015.

4. A. Aldoma Buchaca, F. Tombari, J. Prankl, A. Richtsfeld, L. di Stefano, M. Vincze, Multimodal Cue Integration through Hypotheses Verification for RGB-D Object Recognition and 6DOF Pose Estimation. IEEE International Conference on Robotics and Automation (ICRA), 2013.

5. J. Prankl, T. Mörwald, M. Zillich, M. Vincze, Probabilistic Cue Integration for Real-time Object Pose Tracking. Proc. International Conference on Computer Vision Systems (ICVS). 2013.

For further information check out [this site](http://www.acin.tuwien.ac.at/forschung/v4r/software-tools/rtm).
