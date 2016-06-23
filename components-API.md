# Introduction

Each component of the dVRK described in the [software architecture](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Software-Architecture) provides a set of functionalities, i.e. commands for a [cisstMultiTask](https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-tutorial) component or topic for a ROS node (see `[dvrk_ros](https://github.com/jhu-dvrk/dvrk-ros)/dvrk_robot`).

In general, we try to expose most C++ commands and events as ROS topics.  The latest mapping can be found in [dvrk_add_topics_functions](https://github.com/jhu-dvrk/dvrk-ros/blob/master/dvrk_robot/src/dvrk_add_topics_functions.cpp).

## Console

The following commands are available for the console component (see class `mtsIntuitiveResearchKitConsole`):
* `