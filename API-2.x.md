<!--ts-->
<!--te-->

# Introduction

Each component of the dVRK described in the [software architecture](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Software-Architecture) provides a set of functionalities, i.e. commands for a [cisstMultiTask](https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-tutorial) component or topic/service for a ROS node (see [`dvrk_ros`](https://github.com/jhu-dvrk/dvrk-ros)`/dvrk_robot`).

In general, we try to expose most C++ commands and events as ROS topics or services.  Starting with the dVRK release 2.0, we are using the [CRTK naming conventions](https://github.com/collaborative-robotics/documentation/wiki/Robot-API) as much as possible.  There are also some commands bery specific to the dVRK not covered by CRTK.  These can be found in [`dvrk_console.cpp`](https://github.com/jhu-dvrk/dvrk-ros/blob/master/dvrk_robot/src/dvrk_console.cpp).  


WORK IN PROGRESS 