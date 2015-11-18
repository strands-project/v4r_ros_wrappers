This classifier relies on object shape and is based on the approach of [1].

It uses CAD data for training, such as https://repo.acin.tuwien.ac.at/tmp/permanent/Cat200_ModelDatabase.zip (taken from https://repo.acin.tuwien.ac.at/tmp/permanent/3d-net.org).
In order to speed up training, please select a subset of the training data you want to use and put it in a folder `Cat200_ModelDatabase__small`.

##Usage:
The classifier can then be started by:

* `roslaunch openni_launch openni.launch depth_registration:=true`

* run segmentation to get some point cloud clusters (if you want to launch both segmentation and classification at the same time, please refer to the launch file in segment\_and\_classify and add the classification parameters desrcibed below)

* `roslaunch object_classifier classifier.launch models_dir:=your_dataset_dir/Cat200_ModelDatabase__small/ topic:=/camera/depth_registered/points training_dir:=your_training_dir`

##params:
* models_dir - training directory with the CAD models of the classes, 
* training_dir - directory containing the trained data (if they exist - otherwise they will be re-trained)

##Test:
To test, you can use the test node in segment\_and\_classify.

##References:
* [1] W. Wohlkinger and M. Vincze*, *Ensemble of Shape Functions for 3D Object Classification*
