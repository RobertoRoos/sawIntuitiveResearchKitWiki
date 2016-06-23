# Introduction

Each component of the dVRK described in the [software architecture](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Software-Architecture) provides a set of functionalities, i.e. commands for a [cisstMultiTask](https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-tutorial) component or topic for a ROS node (see [`dvrk_ros`](https://github.com/jhu-dvrk/dvrk-ros)`/dvrk_robot`).

In general, we try to expose most C++ commands and events as ROS topics.  The latest mapping can be found in [`dvrk_add_topics_functions`](https://github.com/jhu-dvrk/dvrk-ros/blob/master/dvrk_robot/src/dvrk_add_topics_functions.cpp).

## Console

### Commands

The following commands are available for the console component (see class `mtsIntuitiveResearchKitConsole`):
* **v 1.4+**: `PowerOff`.  This void command will send a power off request to all arms.  There is no feedback confirming that any or all arms where successfully powered off.<br>
  ROS subscriber: `/dvrk/console/power_off`: `std_msgs::Empty`
* **v 1.4+**`Home`.  This void command will turn off power, turn it back on and then perform homing.  The first homing will calibrate the encoders based on potentiometers and joint limits (last joint on MTMs).  Later calls will skip the encoder calibration.  There is no feedback confirming that all arms are successfully homed.<br>
  ROS subscriber: `/dvrk/console/home`: `std_msgs::Empty`
* **v 1.4+**`TeleopEnable`:  The write command sends an enable/disable request to all teleoperation components (either for PSMs or ECM) known to the console.  There is no feedback confirming that any or all teleoperation components were properly enabled or disabled.<br>
  ROS subscriber: `/dvrk/console/teleop/enable`: `std_msgs::Bool`

### Feedback