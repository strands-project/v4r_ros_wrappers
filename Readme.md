This repository contains ROS wrappers for the V4R library.

# The nature of this library

IMPORTANT: This repository is a ROS wrapper only and shall therefore only include ROS interfaces (service calls, test nodes, visualizations, etc.) to objects / functions defined in the [V4R library](https://github.com/strands-project/v4r) (e.g. by deriving from classes defined in the V4R library). Please do not commit any essential code into this repository - this should go directly into V4R library (so that people can use it also without ROS). This also means that external dependencies should already be resolved by the V4R library.

As a convention, please create seperate folders for service/message definitions (e.g. do not put your `*.cpp/*.hpp` files together with `*.msg/*.srv`).

# Installation

## From Ubuntu Packages

simply install `sudo apt-get install ros-indigo-v4r-ros-wrappers` after enabling the [STRANDS repositories](https://github.com/strands-project-releases/strands-releases/wiki#using-the-strands-repository). 

## From Source

Make sure you install [V4R](https://github.com/strands-project/v4r) first.

Clone v4r_ros_wrappers into your catkin workspace:
```
cd my_catkin_ws/src
git clone https://github.com/strands-project/v4r_ros_wrappers.git
cd ..
```

Then call `catkin_make` once. You might get an error regarding V4RModules.cmake. This is easy to fix:
```
cd my_catkin_ws/build
ccmake ../src
```
Locate the option `V4R_DIR` and set it, according to where you build V4R library, e.g.:
```
V4R_DIR   /home/somewhere/v4r/build
```
Then call catkin again, and all should now compile fine.

# Tutorial

A [tutorial](https://github.com/strands-project/lamor15/wiki/Tutorial-materials-3) has been given at the LAMoR summer school.

# Troubleshooting

## OpenGL not working for SiftGPU

A GPU (best: NVIDIA) is required for many components. For this to work, the user running the software needs to be allowed to access the X server. The easiest (but *very* insecure in a not secured network) way to achieve this is to allow access for all users via `DISPLAY=:0 sudo xhost +`. 

