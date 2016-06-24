# Introduction

Each component of the dVRK described in the [software architecture](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Software-Architecture) provides a set of functionalities, i.e. commands for a [cisstMultiTask](https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-tutorial) component or topic for a ROS node (see [`dvrk_ros`](https://github.com/jhu-dvrk/dvrk-ros)`/dvrk_robot`).

In general, we try to expose most C++ commands and events as ROS topics.  The latest mapping can be found in [`dvrk_add_topics_functions`](https://github.com/jhu-dvrk/dvrk-ros/blob/master/dvrk_robot/src/dvrk_add_topics_functions.cpp).

# Console

The following commands and feedback are available for the console component (see class `mtsIntuitiveResearchKitConsole`):

## Commands

* **v 1.4+**: `PowerOff`.  This _void_ command will send a power off request to all arms.  There is no feedback confirming that any or all arms where successfully powered off.<br>
  ROS subscriber: `/dvrk/console/power_off`: `std_msgs::Empty`
* **v 1.4+**: `Home`.  This _void_ command will turn off power, turn it back on and then perform homing.  The first homing will calibrate the encoders based on potentiometers and joint limits (last joint on MTMs).  Later calls will skip the encoder calibration.  There is no feedback confirming that all arms are successfully homed.<br>
  ROS subscriber: `/dvrk/console/home`: `std_msgs::Empty`
* **v 1.4+**: `TeleopEnable`.  This _write_ command sends an enable/disable request to all teleoperation components (either for PSMs or ECM) known to the console.  There is no feedback confirming that any or all teleoperation components were properly enabled or disabled.<br>
  ROS subscriber: `/dvrk/console/teleop/enable`: `std_msgs::Bool`
* **v 1.4+**: `SetScale`.  This _write_ command sends a request to set the teleoperation scale to all teleoperation components (either for PSMs or ECM) known to the console.<br>
  ROS subscriber: `/dvrk/console/teleop/set_scale`: `std_msgs::Float32`

## Feedback

* **v 1.4+**: `Scale`.  This _write_ event indicates that a request to change the scale has been sent to all teleoperation components (either for PSMs or ECM) known to the console.  This event is not triggered if the scale of a single teleoperation component is changed, i.e. one can change the `MTMR-PSM1` scale only.  In the case, the `MTMR-PSM1` component will trigger it's own event but the console is not changing the overall scale.<br>
  ROS publisher: `/dvrk/console/teleop/scale`: `std_msgs::Float32`

**--- Work in progress ---**