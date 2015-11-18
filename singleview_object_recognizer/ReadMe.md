##usage:
`rosrun singleview_object_recognizer recognition_service  _models_dir:=/your/path/to/models/ _training_dir:=/your/path/to/training_data _chop_z:=2`

##params:
*	models\_dir [in] - Directory containing 3D models (saved as *.pcd files)
*	training\_dir [in] - directory containing training views with segmentation mask and camera pose information of the models from different perspectives (stored in seperate folders named model\_name.pcd)
*	chop\_z [in] - cut of distance in meters with respect to the camera
	
	further parameters are specified in V4R examples
	
##Test:
`rosrun singleview_object_recognizer test_single_view_recognition` 

params:
*  input\_method[in] (default: 0) - 0=camera input; 1 = input from disk
*  topic[in] (default: /camera/depth\_registered/points) - camera topic being used when input\_method=0
*  directory[in] - directory being used to read test .pcd files when input\_method=1
   

models and test scenes can be obtained from https://repo.acin.tuwien.ac.at/tmp/permanent/dataset_index.php
