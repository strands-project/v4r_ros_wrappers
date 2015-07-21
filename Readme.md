This repository contains ROS wrappers for the V4R library.

IMPORTANT: This repository is a ROS wrapper only and shall therefore only include ROS interfaces (service calls, test nodes, visualizations, etc.) to objects / functions defined in the V4R library (e.g. by deriving from classes defined in the V4R library). Please do not commit any essential code into this repository - this should go directly into V4R library (so that people can use it also without ROS). This also means that external dependencies should already be resolved by the V4R library.

As a convention, please create seperate folders for service/message definitions (e.g. do not put your *.cpp/*.hpp files together with *.msg/*.srv).
