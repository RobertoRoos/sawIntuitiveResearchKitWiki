<!--ts-->
<!--te-->

# Introduction

Each component of the dVRK described in the [software architecture](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Software-Architecture) provides a set of functionalities, i.e. commands for a [cisstMultiTask](https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-tutorial) component or topic/service for a ROS node (see [`dvrk_ros`](https://github.com/jhu-dvrk/dvrk-ros)`/dvrk_robot`).

In general, we try to expose most C++ commands and events as ROS topics or services.  Starting with the dVRK release 2.0, we are using the [CRTK naming conventions](https://github.com/collaborative-robotics/documentation/wiki/Robot-API).  There are also some commands very specific to the dVRK not covered by CRTK.  These can be found in [`dvrk_console.cpp`](https://github.com/jhu-dvrk/dvrk-ros/blob/master/dvrk_robot/src/dvrk_console.cpp).  

# Arms

## All

### Operating state

* `state_command`
* `operating_state`
* `desired_state`
* `error`
* `warning`
* `status`
* `goal_reached`

### Motion queries

* `measured_cp`
* `measured_cv`
* `measured_js`
* `setpoint_cp`
* `setpoint_js`
* `local/measured_cp`
* `local/setpoint_cp`
* `body/jacobian`
* `body/measured_cf`
* `spatial/jacobian`
* `spatial/measured_cf`

### Motion commands

* `servo_cp`
* `servo_jf`
* `servo_jp`
* `servo_jr`
* `spatial/servo_cf`
* `body/servo_cf`
* `set_cartesian_impedance_gains`
* `move_cp`
* `move_jp`
* `move_jr`

### Configuration

* `use_gravity_compensation`
* `body/set_cf_orientation_absolute`
* `trajectory_j/ratio`
* `trajectory_j/ratio_a`
* `trajectory_j/ratio_v`
* `trajectory_j/set_ratio`
* `trajectory_j/set_ratio_a`
* `trajectory_j/set_ratio_v`

## ECM

* `manip_clutch`
* `endoscope_type`
* `set_endoscope_type`

## MTM

* `gripper/measured_js`
* `gripper/closed`
* `gripper/pinch`
* `orientation_locked`
* `lock_orientation`
* `unlock_orientation`

## PSM

## SUJ

* `jaw/measured_js`
* `jaw/setpoint_js`
* `jaw/servo_jf`
* `jaw/servo_jp`
* `jaw/move_jp`
* `tool_type`
* `tool_type_request`
* `set_tool_type`
* `set_adapter_present`
* `set_tool_present`
* `io/adapter`
* `io/tool`
* `io/manip_clutch`
* `io/suj_clutch`

# Tele-operation

## PSM Tele-operation

* `current_state`
* `desired_state`
* `state_command`
* `following`
* `scale`
* `set_scale`
* `align_mtm`
* `alignment_offset`
* `set_align_mtm`
* `rotation_locked`
* `lock_rotation`
* `translation_locked`
* `lock_translation`
* `set_registration_rotation` (obsolete)

## ECM Tele-operation

* `current_state`
* `desired_state`
* `state_command`
* `following`
* `scale`
* `set_scale`

# Console

## General

* `console/power_off`
* `console/power_on`
* `console/home`
* `console/camera`
* `console/clutch`
* `console/operator_present`
* `console/emulate_camera`
* `console/emulate_clutch`
* `console/emulate_operator_present`
* `console/volume`
* `console/set_volume`
* `console/string_to_speech`
* `console/beep`

## Tele-operation

* `console/teleop/enable`
* `console/teleop/scale`
* `console/teleop/set_scale`
* `console/teleop/teleop_psm_selected`
* `console/teleop/teleop_psm_unselected`
* `console/teleop/cycle_teleop_psm_by_mtm`
* `console/teleop/select_teleop_psm`

## Foot pedals

* `footpedals/clutch`
* `footpedals/camera`
* `footpedals/cam_minus`
* `footpedals/cam_plus`
* `footpedals/bicoag`
* `footpedals/coag`
