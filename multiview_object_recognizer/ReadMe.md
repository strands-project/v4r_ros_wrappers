##usage:
`rosrun multiview_object_recognizer multiview_object_recognizer_node -m /path/to/your/models/ [--optional_parameter p]`

##params (see extended help output with -h):
*	models\_dir [in] - Directory containing the object models (REQUIRED)
*	do\_sift [in] - if true, does SIFT feature matching
*	do\_shot [in] - if true, does SHOT feature matching
*	chop\_z [in] - cut of distance in meters with respect to the camera coordinate system
* compute\_mst [in] - if true, does point cloud registration by SIFT background matching (given scene\_to\_scene == true), by using given pose (if use\_robot\_pose == true) and by common object hypotheses (if hyp\_to\_hyp == true) from all the possible connection a Mimimum Spanning Tree is computed. If false, it only uses the given pose for each point cloud. The given pose can be extracted from the .pcd's header fields sensor\_origin and sensor\_orientation or from the extracted camera pose e.g. from the camera\_tracker package
*	scene\_to\_scene [in] - if true, estimates relative camera transform between two viewpoints using SIFT background matching
* use\_go3d [in] - if true, verifies against a reconstructed scene from multiple viewpoints. Otherwise only against the current viewpoint.
*	cg\_size\_thresh [in] - minimum number of feature corrrespondences neccessary to generate an object hypothesis (low number increases recall and compuation time, high number increases precision)
*	hv\_color\_sigma_l [in] - maximum allowed standard deviation of the luminance color (L channel in LAB color space) for points for generated object hypotheses and scene (low number increases precision, high number increases recall)
*	hv\_color\_sigma\_ab [in] - maximum allowed standard deviation of the luminance color (AB channels in LAB color space) for points for generated object hypotheses and scene (low number increases precision, high number increases recall)
*	max\_vertices\_in\_graph [in] - maximum number of view points taken into account by the multi-view recognizer
*	transfer\_feature\_matches [in] - defines the method used for transferring information from other viewpoints (1 = keypoint correspondences (Faeulhammer ea., ICRA2015 paper); 0 = full hypotheses only (Faeulhammer ea., MVA2015 paper))
*	distance\_keypoints\_get\_discarded [in] - defines the squared distance [m²] for keypoints to be considered the same. This avoids redundant keypoints when transformed from other view points (only used for extension\_mode=0)

further description of parameters in the example object\_recognizer\_multiview in V4R/samples

##Test:
`rosrun multiview_object_recognizer test_multiview_object_recognizer_node [_optional_parameter:=p]`

##Test params (NOTE THAT THESE ARE ROS PARAMETERS):
*  input\_method[in] (default: 0) - 0=camera input; 1 = input from disk
*  topic[in] (default: /camera/depth\_registered/points) - camera topic being used when input\_method=0
*  directory[in] - directory being used to read test .pcd files when input\_method=1
  
Object models and test scenes can be obtained from https://repo.acin.tuwien.ac.at/tmp/permanent/dataset_index.php
To model your own objects have a look at *http://www.acin.tuwien.ac.at/forschung/v4r/software-tools/rtm/*

##References:
* [*Thomas Fäulhammer, Aitor Aldoma, Michael Zillich, Markus Vincze*, *Temporal Integration of Feature Correspondences For Enhanced Recognition in Cluttered And Dynamic Environments, IEEE Int. Conf. on Robotics and Automation (ICRA), 2015*]

* [*Thomas Fäulhammer, Michael Zillich, Markus Vincze*, *Multi-View Hypotheses Transfer for Enhanced Object Recognition in Clutter, IAPR Conference on Machine Vision Applications (MVA), 2015*]
