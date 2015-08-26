##usage:

rosrun multiview_object_recognizer multiview_object_recognizer_node _extension_mode:=0 _models_dir:=/media/Data/datasets/TUW/models/ _visualize:=true _training_dir_sift:=/media/Data/datasets/TUW/sift_trained/ _training_dir_shot:=/media/Data/datasets/TUW/shot_trained/ _recognition_structure_dir:=/media/Data/datasets/TUW/training_data _scene_to_scene:=true _do_sift:=true _opt_type:=0 _do_shot:=false _do_ourcvfh:=false _chop_z:=2.0 _cg_size_thresh:=3 _cg_size:=0.015 _cg_ransac_threshold:=0.015 _cg_dist_for_clutter_factor:=0 _cg_max_taken:=2 _cg_max_time_for_cliques_computation:=100 _cg_dot_distance:=0.2 _hv_resolution:=0.005 _hv_inlier_threshold:=0.015 _hv_radius_clutter:=0.03 _hv_regularizer:=3 _hv_clutter_regularizer:=5 _hv_occlusion_threshold:=0.01 _hv_optimizer_type:=0 _hv_color_sigma_l:=0.6 _hv_color_sigma_ab:=0.5 _hv_use_supervoxels:=true _hv_detect_clutter:=true _hv_ignore_color:=false _icp_iterations:=10 _icp_type:=1 _icp_voxel_size:=0.005 _max_vertices_in_graph:=5 _distance_keypoints_get_discarded:=0.000025 _use_robot_pose:=false


##params:
*	models\_dir [in] - Directory containing 3D models
*	recognizer\_structure\_sift [in] - directory for training models from different perspectives
*	training\_dir\_sift [out] - directory where sift descriptors will be stored
*	training\_dir\_shot [out] - directory where shot descriptors will be stored
*	do\_sift [in] - if true, does SIFT matching
*	do\_shot [in] - if true, does SHOT matching
*	do\_ourcvfh [in] - if true, does OURCVFH matching (experimental! don't use with extension\_mode = 0)
*	scene\_to\_scene [in] - if true, estimates relative camera transform between two viewpoints using SIFT background matching
*	use\_robot_pose [in] - if true, uses given transformation for camera poses
*	chop\_z [in] - cut of distance in meters
*	cg\_size\_thresh [in] - minimum number of feature corrrespondences neccessary to generate an object hypothesis (low number increases recall, high number increases precision)
*	hv\_color\_sigma_l [in] - maximum allowed standard deviation of the luminance color (L channel in LAB color space) for points for generated object hypotheses and scene (low number increases precision, high number increases recall)
*	hv\_color\_sigma\_ab [in] - maximum allowed standard deviation of the luminance color (AB channels in LAB color space) for points for generated object hypotheses and scene (low number increases precision, high number increases recall)
*	max\_vertices\_in\_graph [in] - maximum number of view points taken into account by the multi-view recognizer
*	extension\_mode [in] - defines the method used for transferring information from other viewpoints (0 = keypoint correspondences (Faeulhammer, ICRA2015 paper); 1 = full hypotheses only (Faeulhammer, MVA2015 paper))
*	distance\_keypoints\_get\_discarded [in] - defines the squared distance [m²] for keypoints to be considered the same. This avoids redundant keypoints when transformed from other view points (only used for extension\_mode=0)
