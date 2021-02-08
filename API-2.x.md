<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](https://github.com/thlorenz/doctoc)*

- [Introduction](#introduction)
- [Console](#console)
  - [Commands](#commands)
  - [Feedback](#feedback)
- [Foot pedals](#foot-pedals)
  - [Feedback](#feedback-1)
- [All arms](#all-arms)
  - [Commands (read)](#commands-read)
  - [Commands (write)](#commands-write)
- [MTMs](#mtms)
  - [Commands (read)](#commands-read-1)
  - [Commands (write)](#commands-write-1)
- [Coordinate systems](#coordinate-systems)
  - [Individual arms](#individual-arms)
  - [Setup Joints](#setup-joints)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# Introduction

Each component of the dVRK described in the [software architecture](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Software-Architecture) provides a set of functionalities, i.e. commands for a [cisstMultiTask](https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-tutorial) component or topic for a ROS node (see [`dvrk_ros`](https://github.com/jhu-dvrk/dvrk-ros)`/dvrk_robot`).

In general, we try to expose most C++ commands and events as ROS topics.  The latest mapping can be found in [`dvrk_add_topics_functions`](https://github.com/jhu-dvrk/dvrk-ros/blob/master/dvrk_robot/src/dvrk_add_topics_functions.cpp).  From version 1.3 to 1.4, many ROS messages have changed as we are trying to use messages with a header as much as possible.  This allows to have a timestamp, name and sequence number for most messages.

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

# Foot pedals

## Feedback

The following feedback is available for the foot pedals:

* **v 1.3+**: `Clutch`.  This _write_ event indicates that the "clutch" pedal has been pressed.  The C++ payload is `prmEventButton`.  The name of the event is defined in the `sawRobotIO1394` XML configuration file.<br>
  ROS publisher: `/dvrk/footpedals/clutch`: **v 1.3** `std_msgs::Bool`, **V 1.4+** `sensor_msgs::Joy`.

* **v 1.3+**: `Coag`.  This _write_ event indicates that the "coag" pedal has been pressed.  On some systems this pedal is labeled "mono".  The C++ payload is `prmEventButton`.  The name of the event is defined in the `sawRobotIO1394` XML configuration file.<br>
  ROS publisher: `/dvrk/footpedals/coag`: **v 1.3** `std_msgs::Bool`, **V 1.4+** `sensor_msgs::Joy`.

* **v 1.3+**: `Camera`.  This _write_ event indicates that the "camera" pedal has been pressed.  The C++ payload is `prmEventButton`.  The name of the event is defined in the `sawRobotIO1394` XML configuration file.<br>
  ROS publisher: `/dvrk/footpedals/camera`: **v 1.3** `std_msgs::Bool`, **V 1.4+** `sensor_msgs::Joy`.

* **v 1.3+**: `Cam+`.  This _write_ event indicates that the camera focus "+" pedal has been pressed.  The C++ payload is `prmEventButton`.  The name of the event is defined in the `sawRobotIO1394` XML configuration file.<br>
  ROS publisher: `/dvrk/footpedals/camera_plus`: **v 1.3** `std_msgs::Bool`, **V 1.4+** `sensor_msgs::Joy`.

* **v 1.3+**: `Cam-`.  This _write_ event indicates that the camera focus "-" pedal has been pressed.  The C++ payload is `prmEventButton`.  The name of the event is defined in the `sawRobotIO1394` XML configuration file.<br>
  ROS publisher: `/dvrk/footpedals/camera_minus`: **v 1.3** `std_msgs::Bool`, **V 1.4+** `sensor_msgs::Joy`.

# All arms

The following commands and feedback are available for all arm components (see class `mtsIntuitiveResearchKitArm`):

## Commands (read)

* **v 1.3**: `GetPositionJoint`.  This _read_ command returns the current joint positions based on the encoders and the actuator to joint coupling matrix.  On a PSM, when no tool is present, the joint positions are the same as encoder positions.<br>
  ROS publisher: `/dvrk/<arm_name>/position_joint_current`: `sensor_msgs::JointState`.  Deprecated in **v 1.4**, see `state_joint_current`.

* **v 1.3**: `GetPositionJointDesired`.  This _read_ command returns the last requested joint positions used by the PID controller.<br>
  ROS publisher: `/dvrk/<arm_name>/position_joint_desired`: `sensor_msgs::JointState`.  Deprecated in **v 1.4**, see `state_joint_current`.

* **v 1.3+**: `GetStateJoint`.  This _read_ command returns the current joint positions and velocities based on the encoders and the actuator to joint coupling matrix.  The effort is based on the current feedback provided by the FPGAQLA controllers and the coupling matrix from actuator to joint.  On a PSM, when no tool is present, the positions, velocities and efforts are the same as encoder positions.<br>
  ROS publisher: `/dvrk/<arm_name>/state_joint_current`: `sensor_msgs::JointState`.  The names of joints are defined in the `sawControllersPID` XML file.

* **v 1.3+**: `GetStateJointDesired`.  This _read_ command returns the last requested joint positions used by the PID controller.  Velocities are undefined.   Efforts are based on the output of the PID controller, i.e. requested effort.<br>
  ROS publisher: `/dvrk/<arm_name>/state_joint_desired`: `sensor_msgs::JointState`.  The names of joints are defined in the `sawControllersPID` XML file.

* **v 1.3+**: `GetPositionCartesian`.  This _read_ command returns the current cartesian position based on encoders.  If the arm has a base frame defined, the reported position includes the base frame (e.g. PSM wrt ECM or PSM with SUJ).<br>
  ROS publisher: `/dvrk/<arm_name>/position_cartesian_current`: **v 1.3** `geometry_msgs::Pose`, **v 1.4+** `geometry_msgs::PoseStamped` 

* **v 1.3+**: `GetPositionCartesianDesired`.  This _read_ command returns the desired cartesian position based on the PID desired joint positions.  If the arm has a base frame defined, the reported position includes the base frame (e.g. PSM wrt ECM or PSM with SUJ).<br>
  ROS publisher: `/dvrk/<arm_name>/position_cartesian_desired`: **v 1.3** `geometry_msgs::Pose`, **v 1.4+** `geometry_msgs::PoseStamped` 

* **v 1.3+**: `GetPositionCartesianLocal`.  This _read_ command returns the current cartesian position based on encoders.  The reported position is based on the arm kinematic chain only, it doesn't include any base frame.<br>
  ROS publisher: `/dvrk/<arm_name>/position_cartesian_local_current`: **v 1.3** `geometry_msgs::Pose`, **v 1.4+** `geometry_msgs::PoseStamped` 

* **v 1.3+**: `GetPositionCartesianLocalDesired`.  This _read_ command returns the desired cartesian position based on the PID desired joint positions.  The reported position is based on the arm kinematic chain only, it doesn't include any base frame.<br>
  ROS publisher: `/dvrk/<arm_name>/position_cartesian_local_desired`: **v 1.3** `geometry_msgs::Pose`, **v 1.4+** `geometry_msgs::PoseStamped` 

* **v 1.4+**: `GetVelocityCartesian`.  This _read_ command returns the current cartesian velocity based on the encoders and the body jacobian.<br>
  ROS publisher: `/dvrk/<arm_name>/twist_body_current`: `geometry_msgs::TwistStamped` 

* **v 1.4+**: `GetWrenchBody`.  This _read_ command returns the current body wrench based on the motor current (electric current) feedback and the body jacobian.<br>
  ROS publisher: `/dvrk/<arm_name>/wrench_body_current`: `geometry_msgs::WrenchStamped` 

* **v 1.4+**: `GetJacobianBody`.  This _read_ command returns the current body jacobian.<br>
  ROS publisher: `/dvrk/<arm_name>/jacobian_body`: `std_msgs::Float64MultiArray` 

* **v 1.4+**: `GetJacobianSpatial`.  This _read_ command returns the current spatial jacobian.<br>
  ROS publisher: `/dvrk/<arm_name>/jacobian_body`: `std_msgs::Float64MultiArray` 

## Commands (write)

* **v 1.4+**: `SetBaseFrame`.  This _write_ command sets a base frame used for all forward and inverse kinematic computations.  This transformation is prepended to the kinematic chain, i.e. `base_frame * base_offset * DH_chain * tooltip_offset`.   See transformation section.<br>
  ROS subscriber: `/dvrk/<arm_name>/set_base_grame`: `geometry_msgs::Pose`.

* **v 1.3+**: `SetRobotControlState`.  This _write_ command sets the desired state for the arm.  The parameter is a string, see https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/components/code/mtsIntuitiveResearchKitArmTypes.cdg.  If the desired state is not possible based on the current state, an error event will be raised (ROS error message).<br>
  ROS subscriber: `/dvrk/<arm_name>/set_robot_state`: `std_msgs::String`.

* **v 1.3+**: `SetPositionJoint`.  This _write_ command sets the desired joint position.  The controller will send the desired joint position directly to the PID component, no trajectory will be generated.  The caller has to make sure the desired position is reasonable (within joint limits and PID tracking error).  The arm has to be in `DVRK_POSITION_JOINT` mode, see `SetRobotControlState` to change mode.<br>
  ROS subscriber: `/dvrk/<arm_name>/set_position_joint`: `sensor_msgs::JointState`.

* **v 1.3+**: `SetPositionGoalJoint`.  This _write_ command sets the desired joint goal position.  The controller will generate a trajectory in joint space that will be sent to the PID component.  The generated trajectory is of type LSPB and always assumes a zero velocity at both start and end positions.  **v 1.5+** Reflexxes RML II is used to generate the trajectories so initial velocity is based on current velocity, final velocity is still assumed to be zero.  When the trajectory is completed, a `GoalReached` event is raised (ROS: `goal_reached`).  The user should make sure the goal has been reached before sending another goal.  The arm has to be in `DVRK_POSITION_GOAL_JOINT` mode, see `SetRobotControlState` to change mode.<br>
  ROS subscriber: `/dvrk/<arm_name>/set_position_goal_joint`: `sensor_msgs::JointState`.

* **v 1.3+**: `SetPositionCartesian`.  This _write_ command sets the desired cartesian position.  The controller will compute the inverse kinematic and send the desired joint position directly to the PID component, no trajectory will be generated.  The arm has to be in `DVRK_POSITION_CARTESIAN` mode, see `SetRobotControlState` to change mode.<br>
  ROS subscriber: `/dvrk/<arm_name>/set_position_cartesian`: `geometry_msgs::Pose`.

* **v 1.3+**: `SetPositionGoalCartesian`.  This _write_ command sets the desired cartesian goal position.  The controller will compute the inverse kinematics and generate a trajectory in joint space that will be sent to the PID component.  The generated trajectory is of type LSPB and always assumes a zero velocity at both start and end positions (see also `SetPositionGoalJoint`).    **v 1.5+** Reflexxes RML II is used to generate the trajectories so initial velocity is based on current velocity, final velocity is still assumed to be zero.  The arm has to be in `DVRK_POSITION_GOAL_CARTESIAN` mode, see `SetRobotControlState` to change mode.<br>
  ROS subscriber: `/dvrk/<arm_name>/set_position_goal_cartesian`: `geometry_msgs::Pose`.

* **v 1.4+**: `SetWrenchBody`.  This _write_ command sets the desired wrench based on the body jacobian (tooltip frame).  The controller will compute joint torques using the jacobian and send these to the PID component.  The arm has to be in `DVRK_EFFORT_CARTESIAN` mode, see `SetRobotControlState` to change mode.<br>
  ROS subscriber: `/dvrk/<arm_name>/set_wrench_body`: `geometry_msgs::Wrench`.

* **v 1.4+**: `SetWrenchSpatial`.  This _write_ command sets the desired wrench based on the spatial jacobian (base frame).  The controller will compute joint torques using the jacobian and send these to the PID component.  The arm has to be in `DVRK_EFFORT_CARTESIAN` mode, see `SetRobotControlState` to change mode.<br>
  ROS subscriber: `/dvrk/<arm_name>/set_wrench_spatial`: `geometry_msgs::Wrench`.

* **v 1.4+**: `SetWrenchBodyOrientationAbsolute`.  This _write_ command sets a flag used to determine if the body wrenches (both set and get) should be defined using the tooltip orientation or the base frame orientation.  If set to true, all wrenches are based on the base frame orientation, i.e. constant.<br>
  ROS subscriber: `/dvrk/<arm_name>/set_wrench_body_orientation_absolute`: `std_msgs::Bool`.

* **v 1.4+**: `SetGravityCompensation`.  This _write_ command sets a flag used to determine if the gravity compensation should be added when the arm is in `DVRK_EFFORT_CARTESIAN` mode.<br>
  ROS subscriber: `/dvrk/<arm_name>/set_gravity_compensation`: `std_msgs::Bool`.

# MTMs

The following commands and feedback are available for MTMs only (see class `mtsIntuitiveResearchKitMTM`):

## Commands (read)

* **v 1.3+**: `GetGripperPosition`.  This _read_ command returns the current gripper position based on the Hall Effect sensors.<br>
  ROS publisher: `/dvrk/<mtm_name>/gripper_position_current`: `std_msgs::Float32`.

## Commands (write)

* **v 1.4+**: `LockOrientation`.  This _write_ command sets the desired orientation while in effort mode.  The caller has to make sure the desired orientation is reasonable (within joint limits and PID tracking error).  The arm has to be in `DVRK_EFFORT_CARTESIAN` mode, see `SetRobotControlState` to change mode.<br>
  ROS subscriber: `/dvrk/<mtm_name>/lock_orientation`: `geometry_msgs::Quaternion`.

* **v 1.4+**: `UnlockOrientation`.  This _void_ command unlocks the MTM orientation while in cartesian effort mode.  The arm has to be in `DVRK_EFFORT_CARTESIAN` mode, see `SetRobotControlState` to change mode.<br>
  ROS subscriber: `/dvrk/<mtm_name>/unlock_orientation`: `std_msgs::Empty`.

**--- Work in progress ---**

# Coordinate systems

## Individual arms

For each individual arm, the cartesian positions are based on:
* DH parameters: defined in JSON file, used to compute the forward kinematics using joint positions
* Kinematic chain offsets (optional):
  * Defined in JSON file
  * `base-offset`: constant offset prepended to the kinematic chain (all arms)
  * `tooltip-offset`: constant offset appended to the kinematic chain (PSM and ECM only, not MTM)
* Base Frame:
  * Set at runtime by command or ROS topic
  * Prepended to the kinematic chain, including `base-offset`
  * Defined in the console JSON configuration per arm using `base-frame`

For example, the MTM kinematic (DH parameters) start from the link 0, i.e. close to the attachment point (see `mtm.json`) and has Z pointing up, X to the left and Y towards the user.  But the ISI convention expects that X points to the left when viewed from the stereo display, Y should point up and Z away from the user.  Furthermore the ISI convention places the origin in the middle of the stereo display (i.e. between the operator's eyes).  In practice MTMs are always mounted rigidly to a frame so we need to apply a constant rotation and translation to match the ISI convention.  The two masters are also mounted apart from each other so there is also a positive X translation for MTML and a negative X translation for MTMR.  The change of reference frame should be defined using the `base-frame` in the console configuration file (see [jhu-dVRK/share](/jhu-dvrk/sawIntuitiveResearchKit/blob/master/share/jhu-dVRK/console-MTMR-PSM1-MTML-PSM2-Teleop.json)).

For a dynamic change of reference frame one should use the `SetBaseFrame` command (ROS topic `set_base_frame`).  For example, this is used to make sure the PSMs are always defined with respect to the ECM tooltip (camera frame) if the camera can move.
* When used with the setup joints, the console class propagates the different base frames to make sure the PSMs are defined with respect to the camera frame (see [jhu-daVinci/share](/jhu-dvrk/sawIntuitiveResearchKit/blob/master/share/jhu-daVinci/console-SUJ-ECM-MTMR-PSM1-MTML-PSM2-Teleop.json)).
* If you don't have access to the setup joints but have a way to register the PSMs to the camera, you can use the `SetBaseFrame` command on the PSMs.
* Finally, if the camera is fixed, you can use `base-frame` in your console configuration (similar to MTMs, see [jhu-dVRK/share](/jhu-dvrk/sawIntuitiveResearchKit/blob/master/share/jhu-dVRK/console-MTMR-PSM1-MTML-PSM2-Teleop.json)).  Note that the `base-frame` depends on where your PSMs are mounted with respect to your fixed camera.

There are two possible positions to query using commands (see ROS topics above):
* `GetPositionCartesian`: `BaseFrame * base-offset * DH * tooltip-offset`
* `GetPositionCartesianLocal`: `base-offset * DH * tooltip-offset`

## Setup Joints

When setup joint are present, the following positions are reported:
* ECM frames:
  * ECM-SUJ-Local:
    * Defined with respect to patient cart origin
    * Uses `base-offset * DH * tooltip-offset` from `suj.json`
    * Defines ECM-RCM
  * ECM-SUJ:
    * Same as ECM-SUJ-Local
  * ECM-Local:
    * Defined with respect to the ECM RCM
    * Uses `DH * tooltip-offset` from `ecm-*.json`
    * `tooltip-offset` depends on type of scope (looking straight, up or down)
  * ECM:
    * **Camera frame defined with respect to patient cart**
    * Uses `ECM-SUJ * ECM-Local` (`ECM-SUJ` is the base frame)
* PSM frames
  * PSM-SUJ-Local:
    * See ECM-SUJ-Local
  * PSM-SUJ:
    * Defined with respect to camera frame
    * Uses `inverse(ECM) * PSM-SUJ-Local` (`inverse(ECM)` is the base frame)
  * PSM-Local:
    * See ECM-Local
    * `DH`, `coupling`, `tooltip-offset`... depend on type of tool
  * PSM:
    * **PSM tooltip frame defined with respect to camera frame**
    * Uses `PSM-SUJ * PSM-Local` (`PSM-SUJ` is the base frame)