^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package multiview_object_recognizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2016-01-29)
------------------

0.1.3 (2016-01-28)
------------------

0.1.2 (2016-01-28)
------------------

0.1.1 (2016-01-28)
------------------

0.1.0 (2016-01-27)
------------------
* added possibility to write intermediate images to a file
* Merge branch 'master' into dynamic_object_learning
  Conflicts:
  multiview_object_recognizer/CMakeLists.txt
  singleview_object_recognizer/CMakeLists.txt
* updated cmake for new v4r cmake system
* tmp
* Contributors: Thomas Fäulhammer

0.0.12 (2016-01-27)
-------------------
* add c++11 definition
* use new parameter parsing option inside V4R library
* add OpenCV libs to multi-view cmake file
* Contributors: Thomas Fäulhammer

0.0.11 (2016-01-11)
-------------------
* updated readme and launch files
* adapt
* adapt_to_new_v4r_interfaces!
* use pcl_conversion from ROS package
  create camera tracker header
* Contributors: Thomas Fäulhammer

0.0.10 (2015-11-25)
-------------------
* using packaged version of pcl_conversions.h
* Contributors: Thomas Fäulhammer

0.0.9 (2015-11-24)
------------------

0.0.8 (2015-11-24)
------------------

0.0.7 (2015-11-23)
------------------
* updated ReadMes
  removed unused files
  updated some launch files, created segement_and_classify package
* updated namespace
* fixed paramter input of doubles
* using v4r_config.h to check for SIFTGPU
* tmp commit
* fix headers and some warnings
* change namespace according to v4r
* Contributors: Thomas Fäulhammer

0.0.6 (2015-10-15)
------------------
* fixed namespace and include issues to fit V4R Version 1.0.11
* Contributors: Thomas Fäulhammer

0.0.5 (2015-09-07)
------------------

0.0.4 (2015-08-29)
------------------
* fixed string formatting bug
* Contributors: Marc Hanheide

0.0.3 (2015-08-29)
------------------

0.0.1 (2015-08-29)
------------------
* added V4R ass dependency where needed and changed maintainers to STRANDS people
* Update ReadMe.md
* Fixed wrong recognition_structure_dir parameter name
* removed deprecated strands_v4r dependency
* fixed camera tracker, single- and multi-view recognition with new V4R
  added some ReadMe
  still need to check classification
* added cam tracker
* tmp commit
* fixed some linking and namespace issues
* tmp
  Conflicts:
  dynamic_object_learning/CMakeLists.txt
  dynamic_object_learning/package.xml
* deleted tmp commits
* .
* initial commit
* Contributors: Marc Hanheide, Michael Zillich, Thomas Faeulhammer, Thomas Fäulhammer
