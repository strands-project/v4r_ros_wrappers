usage:

rosrun singleview_object_recognizer recognition_service  _models_dir:=/media/Data/datasets/TUW/models/ _training_dir_sift:=/media/Data/datasets/TUW/sift_trained/ _training_dir_shot:=/media/Data/datasets/TUW/shot_trained/ _recognizer_structure_sift:=/media/Data/TUW/training_data _chop_z:=1.5


params:
	models_dir [in] - Directory containing 3D models
	recognizer_structure_sift [in] - directory for training models from different perspectives
	training_dir_sift [out] - directory where sift descriptors will be stored
	training_dir_shot [out] - directory where shot descriptors will be stored
	chop_z [in] - cut of distance in meters
