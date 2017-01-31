<a id="top"/> 
# object_gestalt_segmentation

The current segmentation is based on the work of Ekaterina Popatova and Andreas Richtsfeld. Segmented objects are discarded if they are too tall or too far away from the robot. A service is provided that returns the segmented object in the current scene. A more sophisticated solution based on attention cues will be provided in the near future.  

##Technical Maintainer: [markus](https://github.com/edith-langer (Edith Langer, TU Wien) - langer@acin.tuwien.ac.at

##Contents

1. <a href="#1--installation-requirements">Installation Requirements</a>
2. <a href="#2--execution">Execution</a>
3. <a href="#3--software-architecture">Software architecture</a>


## 1. Installation Requirements: <a id="1--installation-requirements"/> 

####ROS packages
The ROS packages dependencies can be installed with the command:
```
rosdep install --from-path object_gestalt_segmentation -i -y
```
## 2. Execution: <a id="2--execution"/> 
```
roslaunch object_gestalt_segmentation startup.launch
```

<a href="#top">top</a>
