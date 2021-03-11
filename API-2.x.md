<!--ts-->
   * [Introduction](#introduction)
   * [Arms](#arms)
      * [All](#all)
         * [Operating state](#operating-state)
         * [Motion queries](#motion-queries)
         * [Motion commands](#motion-commands)
         * [Configuration](#configuration)
      * [ECM](#ecm)
      * [MTM](#mtm)
      * [PSM](#psm)
      * [SUJ](#suj)
   * [Tele-operation](#tele-operation)
      * [PSM Tele-operation](#psm-tele-operation)
      * [ECM Tele-operation](#ecm-tele-operation)
   * [Console](#console)
      * [General](#general)
      * [Tele-operation](#tele-operation-1)
      * [Foot pedals](#foot-pedals)

<!-- Added by: anton, at: 2021-02-10T11:36-05:00 -->

<!--te-->

# Introduction

Each component of the dVRK described in the [software architecture](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Software-Architecture) provides a set of functionalities, i.e. commands/events for a [cisstMultiTask](https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-tutorial) component or topic/service for a ROS node (see [`dvrk_ros`](https://github.com/jhu-dvrk/dvrk-ros)`/dvrk_robot`).

In general, we try to expose most C++ commands and events as ROS topics or services under the same name.  Starting with the dVRK release 2.0, we are using the [CRTK naming conventions](https://github.com/collaborative-robotics/documentation/wiki/Robot-API).  There are also some commands very specific to the dVRK not covered by CRTK.  These can be found in [`dvrk_console.cpp`](https://github.com/jhu-dvrk/dvrk-ros/blob/master/dvrk_robot/src/dvrk_console.cpp).  

To determine the payloads on ROS, use `rostopic info`.

# Arms

## All

### Operating state

* `state_command`
  * *cisst*: write command
  * *ROS*: subscriber
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-status)
* `operating_state`
  * *cisst*: write event and read command
  * *ROS*: publisher
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-status)
* `desired_state`
  * *cisst*: write event
  * *ROS*: publisher
  * dVRK specific.  Uses the CRTK state whenever possible
* `error`
  * *cisst*: write event
  * *ROS*: publisher
  * dVRK specific.  Error messages, can be used for custom GUI.  For ROS, these messages are also sent as errors for ROS log.
* `warning`
  * *cisst*: write event
  * *ROS*: publisher
  * dVRK specific.  Warning messages, can be used for custom GUI.  For ROS, these messages are also sent as warnings for ROS log.
* `status`
  * *cisst*: write event
  * *ROS*: publisher
  * dVRK specific.  Status messages, can be used for custom GUI.  For ROS, these messages are also sent as status messages for ROS log.
* `goal_reached`:
  * *cisst*: write event
  * *ROS*: publisher
  * dVRK specific.  Boolean that indicates if the last `move_` command was completed successfully or not.  It is possible to used the CRTK `operating_state` fields `is_busy` and `state` instead.  This is provided for backward compatibility with dVRK 1.x applications.

### Motion queries

* `measured_cp`
  * *cisst*: read command
  * *ROS*: publisher
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion)
* `measured_cv`
  * *cisst*: read command
  * *ROS*: publisher
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion)
* `measured_js`
  * *cisst*: read command
  * *ROS*: publisher
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion)
* `setpoint_cp`
  * *cisst*: read command
  * *ROS*: publisher
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion)
* `setpoint_js`
  * *cisst*: read command
  * *ROS*: publisher
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion)
* `local/measured_cp`
  * *cisst*: read command
  * *ROS*: publisher
  * dVRK specific.  Measured cartesian position relative to the first frame of the kinematic chain.  This doesn't include any base frame.  For the PSMs and ECM, the first frame of the kinematic chain is centered on the RCM point.  For the MTMs, the first frame of the kinematic chain is centered near the mounting point.   The non "local" `measured_cp` includes the base frame.  For example, MTM cartesian positions are defined with respect to the stereo display and the PSM cartesian positions are defined with respect to the endoscope (i.e. ECM tip).  See also [Coordinate Systems](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Coordinate-Systems).
* `local/setpoint_cp`
  * *cisst*: read command
  * *ROS*: publisher
  * dVRK specific.  Cartesian set point relative to the first frame of the kinematic chain.  See notes for `local/measured_cp`.
* `body/jacobian`
  * *cisst*: read command
  * *ROS*: publisher
  * dVRK specific.  Body jacobian, i.e. relative to end effector.
* `body/measured_cf`
  * *cisst*: read command
  * *ROS*: publisher
  * dVRK specific.  Estimated forces on the end effector.  These are computed using the current feedback on the actuators.  From there, the joint efforts are estimated using the actuator to joint coupling matrix.  Finally, the cartesian effort is computed using the jacobian.  This is not a very rough cartesian force emulation as the computations don't gravity compensation nor any other dynamic model or the arm. 
* `spatial/jacobian`
  * *cisst*: read command
  * *ROS*: publisher
  * dVRK specific.  Body jacobian, i.e. relative to the base frame (first frame in kinematic chain).
* `spatial/measured_cf`

### Motion commands

* `servo_cp`
  * *cisst*: write command
  * *ROS*: subscriber
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion)
* `servo_jf`
  * *cisst*: write command
  * *ROS*: subscriber
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion)
* `servo_jp`
  * *cisst*: write command
  * *ROS*: subscriber
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion)
* `servo_jr`
  * *cisst*: write command
  * *ROS*: subscriber
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion)
* `spatial/servo_cf`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `body/servo_cf`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `set_cartesian_impedance_gains`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `move_cp`
  * *cisst*: write command
  * *ROS*: subscriber
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion)
* `move_jp`
  * *cisst*: write command
  * *ROS*: subscriber
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion)
* `move_jr`
  * *cisst*: write command
  * *ROS*: subscriber
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion)

### Configuration

* `use_gravity_compensation`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `body/set_cf_orientation_absolute`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `trajectory_j/ratio`
* `trajectory_j/ratio_a`
* `trajectory_j/ratio_v`
* `trajectory_j/set_ratio`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `trajectory_j/set_ratio_a`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `trajectory_j/set_ratio_v`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.

## ECM

* `manip_clutch`
* `endoscope_type`
* `set_endoscope_type`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.

## MTM

* `gripper/measured_js`
* `gripper/closed`
* `gripper/pinch`
* `orientation_locked`
* `lock_orientation`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `unlock_orientation`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.

## PSM

* `jaw/measured_js`
* `jaw/setpoint_js`
* `jaw/servo_jf`
* `jaw/servo_jp`
* `jaw/move_jp`
* `tool_type`
* `tool_type_request`
* `set_tool_type`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `set_adapter_present`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `set_tool_present`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `io/adapter`
* `io/tool`
* `io/manip_clutch`
* `io/suj_clutch`

## SUJ

# Tele-operation

## PSM Tele-operation

* `current_state`
* `desired_state`
* `state_command`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `following`
* `scale`
* `set_scale`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `align_mtm`
* `alignment_offset`
* `set_align_mtm`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `rotation_locked`
* `lock_rotation`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `translation_locked`
* `lock_translation`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `set_registration_rotation` (obsolete)

## ECM Tele-operation

* `current_state`
* `desired_state`
* `state_command`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `following`
* `scale`
* `set_scale`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.

# Console

## General

* `console/power_off`
* `console/power_on`
* `console/home`
* `console/camera`
* `console/clutch`
* `console/operator_present`
* `console/emulate_camera`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `console/emulate_clutch`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `console/emulate_operator_present`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `console/volume`
* `console/set_volume`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `console/string_to_speech`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `console/beep`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.

## Tele-operation

* `console/teleop/enabled`
* `console/teleop/enable`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `console/teleop/scale`
* `console/teleop/set_scale`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `console/teleop/teleop_psm_selected`
* `console/teleop/teleop_psm_unselected`
* `console/teleop/cycle_teleop_psm_by_mtm`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.
* `console/teleop/select_teleop_psm`
  * *cisst*: write command
  * *ROS*: subscriber
  * dVRK specific.

## Foot pedals

* `footpedals/clutch`
* `footpedals/camera`
* `footpedals/cam_minus`
* `footpedals/cam_plus`
* `footpedals/bicoag`
* `footpedals/coag`
