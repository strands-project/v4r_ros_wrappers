##usage:
`rosrun singleview_object_recognizer recognition_service -m /path/to/your/models/ [--optional_parameter p]`

##params (see extended help output with -h)::
*	models\_dir [in] - Directory containing 3D models (saved as *.pcd files)
*	chop\_z [in] - cut of distance in meters with respect to the camera
	
##Test:
`rosrun singleview_object_recognizer test_single_view_recognition [_optional_parameter:=p]` 

##Test params (NOTE THAT THESE ARE ROS PARAMETERS):
*  input\_method[in] (default: 0) - 0=camera input; 1 = input from disk
*  topic[in] (default: /camera/depth\_registered/points) - camera topic being used when input\_method=0
*  directory[in] - directory being used to read test .pcd files when input\_method=1
   

Object models and test scenes can be obtained from https://repo.acin.tuwien.ac.at/tmp/permanent/dataset_index.php
To model your own objects have a look at *http://www.acin.tuwien.ac.at/forschung/v4r/software-tools/rtm/*
