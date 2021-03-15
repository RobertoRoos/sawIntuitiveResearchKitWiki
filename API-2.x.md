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

If you are migrating your *cisstMultiTask* or ROS code from the dVRK 1.7, you can find some porting information in the [`crtk-port`](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/devel/crtk-port).

# Arms

## All

C++ class is `mtsIntuitiveResearchKitArm`.

### Operating state

* `state_command`
  * *cisst*: write command `std::string`
  * *ROS*: subscriber `crtk_msgs/StringStamped`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-status)
* `operating_state`
  * *cisst*: write event and read command `prmOperatingState`
  * *ROS*: publisher `crtk_msgs/operating_state`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-status)
* `desired_state`
  * *cisst*: write event `std::string`
  * *ROS*: publisher `std_msgs/String`
  * dVRK specific.  Uses the CRTK state whenever possible
* `error`
  * *cisst*: write event `std::string`
  * *ROS*: publisher `std_msgs/String`
  * dVRK specific.  Error messages, can be used for custom GUI.  For ROS, these messages are also sent as errors for ROS log.
* `warning`
  * *cisst*: write event `std::string`
  * *ROS*: publisher `std_msgs/String`
  * dVRK specific.  Warning messages, can be used for custom GUI.  For ROS, these messages are also sent as warnings for ROS log.
* `status`
  * *cisst*: write event `std::string`
  * *ROS*: publisher `std_msgs/String`
  * dVRK specific.  Status messages, can be used for custom GUI.  For ROS, these messages are also sent as status messages for ROS log.
* `goal_reached`
  * *cisst*: write event `bool`
  * *ROS*: publisher `std_msgs/Bool`
  * dVRK specific.  Boolean that indicates if the last `move_{jc}{pr}` command was completed successfully or not.  It is possible to use the CRTK `operating_state` fields `is_busy` and `state` instead.  This is provided for backward compatibility with dVRK 1.x applications.

### Motion queries

* `measured_cp`
  * *cisst*: read command `prmPositionCartesianGet`
  * *ROS*: publisher `geometry_msgs/TransformStamped`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  Measured cartesian position.
* `measured_cv`
  * *cisst*: read command `prmVelocityCartesianGet`
  * *ROS*: publisher `geometry_msgs/TwistStamped`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  Measured cartesian velocity.
* `measured_js`
  * *cisst*: read command `prmStateJoint`
  * *ROS*: publisher `sensor_msgs/JointState`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  Measured joint state (position, velocity, effort).
* `setpoint_cp`
  * *cisst*: read command `prmPositionCartesianGet`
  * *ROS*: publisher `geometry_msgs/TransformStamped`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  Cartesian position setpoint.
* `setpoint_js`
  * *cisst*: read command `prmStateJoint`
  * *ROS*: publisher `sensor_msgs/JointState`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  Joint setpoint (either position or effort).
* `local/measured_cp`
  * *cisst*: read command `prmPositionCartesianGet`
  * *ROS*: publisher `geometry_msgs/TransformStamped`
  * dVRK specific.  Measured cartesian position relative to the first frame of the kinematic chain.  This doesn't include any base frame.  For the PSMs and ECM, the first frame of the kinematic chain is centered on the RCM point.  For the MTMs, the first frame of the kinematic chain is centered near the mounting point.   The non "local" `measured_cp` includes the base frame.  For example, MTM cartesian positions are defined with respect to the stereo display and the PSM cartesian positions are defined with respect to the endoscope (i.e. ECM tip).  See also [Coordinate Systems](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Coordinate-Systems).
* `local/setpoint_cp`
  * *cisst*: read command `prmPositionCartesianGet`
  * *ROS*: publisher `geometry_msgs/TransformStamped`
  * dVRK specific.  Cartesian setpoint relative to the first frame of the kinematic chain.  See notes for `local/measured_cp`.
* `body/jacobian`
  * *cisst*: read command `vctDoubleMat`
  * *ROS*: publisher `std_msgs/Float64MultiArray`
  * dVRK specific.  Body jacobian, i.e. relative to end effector.  See [cisstRobot](https://github.com/jhu-cisst/cisst/wiki/cisstRobot-robManipulator).
* `body/measured_cf`
  * *cisst*: read command `prmForceCartesianGet`
  * *ROS*: publisher `geometry_msgs/WrenchStamped`
  * dVRK specific.  Estimated forces on the end effector.  These are computed using the current feedback on the actuators.  From there, the joint efforts are estimated using the actuator to joint coupling matrix.  See also `body/set_cf_orientation_absolute` and [cisstRobot](https://github.com/jhu-cisst/cisst/wiki/cisstRobot-robManipulator).  Finally, the cartesian effort is computed using the jacobian.  This is a rough cartesian force emulation as the computations don't take into account gravity compensation nor any other dynamic model or the arm. 
* `spatial/jacobian`
  * *cisst*: read command `vctDoubleMat`
  * *ROS*: publisher `std_msgs/Float64MultiArray`
  * dVRK specific.  Spatial jacobian, i.e. relative to the base frame (first frame in kinematic chain).  See [cisstRobot](https://github.com/jhu-cisst/cisst/wiki/cisstRobot-robManipulator).
* `spatial/measured_cf`
  * *cisst*: read command `prmForceCartesianGet`
  * *ROS*: publisher `geometry_msgs/WrenchStamped`
  * dVRK specific.  See `body/measured_cf`.

### Motion commands

* `servo_cp`
  * *cisst*: write command `prmPositionCartesianSet`
  * *ROS*: subscriber `geometry_msgs/TransformStamped`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  Set cartesian position goal for low-level controller (PID).  Use with caution, goals should be reachable withing a single clock tick (< 1 ms).  Use `move_cp` for large motions.
* `servo_jf`
  * *cisst*: write command `prmForceTorqueJointSet`
  * *ROS*: subscriber `sensor_msgs/JointState`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  Set joint effort goal for low-level controller (direct current control).  Use with caution since the only safety mechanism built-in is the cap on maximum motor current.
* `servo_jp`
  * *cisst*: write command `prmPositionJointSet`
  * *ROS*: subscriber `sensor_msgs/JointState`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  Set joint position goal for low-level controller (PID).  Use with caution, goals should be reachable withing a single clock tick (< 1 ms).  Use `move_jp` or `move_jr` for large motions.
* `servo_jr`
  * *cisst*: write command `prmPositionJointSet`
  * *ROS*: subscriber `sensor_msgs/JointState`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion). Set joint relative position goal for low-level controller (PID).  The goal is defined as an increment that will be added to the current `setpoint_jp`.  See also notes for `servo_jp`.
* `spatial/servo_cf`
  * *cisst*: write command `prmForceCartesianSet`
  * *ROS*: subscriber `geometry_msgs/WrenchStamped`
  * dVRK specific.  Set cartesian effort goal for low-level controller using `spatial/jacobian` (direct current control).  Use with caution since the only safety mechanism built-in is the cap on maximum motor current.  For most application, use `body/servo_cf`.  Gravity compensation will be added based on last call to `use_gravity_compensation` (for MTMs and ECM).  See [cisstRobot](https://github.com/jhu-cisst/cisst/wiki/cisstRobot-robManipulator).
* `body/servo_cf`
  * *cisst*: write command `prmForceCartesianSet`
  * *ROS*: subscriber `geometry_msgs/WrenchStamped`
  * dVRK specific.  Set cartesian effort goal for low-level controller using `body/jacobian` (direct current control).  Use with caution since the only safety mechanism built-in is the cap on maximum motor current.  Useful for haptic on MTM.  By default direction of force is defined by the orientation of the end effector.  To use the absolute orientation, toggle on/off using `body/set_cf_orientation_absolute`.  Gravity compensation will be added based on last call to `use_gravity_compensation` (for MTMs and ECM).  See [cisstRobot](https://github.com/jhu-cisst/cisst/wiki/cisstRobot-robManipulator).  
* `set_cartesian_impedance_gains`
  * *cisst*: write command `prmCartesianImpedanceGains`
  * *ROS*: subscriber `cisst_msgs/prmCartesianImpedanceGains`
  * dVRK specific.  Apply wrench based on difference between measured and goal cartesian positions as well as twist (cartesian velocity).  The cartesian space is divided in 12 cases: negative and positive (**2**) error in position and orientation (**x 2**) along axes X, Y and Z (**x 3 = 12**).  The payload for this command includes 3 parameters for each case: a linear gain, a damping gain and an offset.  This command can be used to define a simple haptic virtual fixture (plane, line, point, box corner...).  Internally the dVRK code uses the class `osaCartesianImpedanceController` from the package [*sawControllers*](https://github.com/jhu-saw/sawControllers).
* `move_cp`
  * *cisst*: write command `prmPositionCartesianSet`
  * *ROS*: subscriber `geometry_msgs/TransformStamped`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  Set cartesian trajectory goal.  The current implementation converts the cartesian goal into a joint trajectory goal and then execute the trajectory in joint space.  Therefore the current controller doesn't generate straight lines in cartesian space. See `move_jp`.
* `move_jp`
  * *cisst*: write command `prmPositionJointSet`
  * *ROS*: subscriber `sensor_msgs/JointState`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  Set joint trajectory goal.
    * Goal can be changed before the trajectory is completed.  The trajectory generator will use the current joint velocities and accelerations to smooth the trajectory.
    * User can check if the trajectory is completed using `is_busy` from the `operating_state` 
    * The current implementation assumes the final velocity is zero.  Sending a fast succession of goals can generate a stop-and-go motion
    * Internally, the dVRK code uses the class `robReflexxes` from [*cisstRobot*](https://github.com/jhu-cisst/cisst/tree/master/cisstRobot) (wrapper for [Reflexxes Type II Library](www.reflexxes.com))
* `move_jr`
  * *cisst*: write command `prmPositionJointSet`
  * *ROS*: subscriber `sensor_msgs/JointState`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  Set relative joint trajectory goal.  See `move_jp`.

### Configuration

* `use_gravity_compensation`
  * *cisst*: write command `bool`
  * *ROS*: subscriber `std_msgs/Bool`
  * dVRK specific.  Turn on or off gravity compensation.  As of dVRK 1.7, gravity compensation is well supported for [MTMs](/jhu-dvrk/dvrk-gravity-compensation/blob/master/README.md).  ECM gravity compensation has been introduced in dVRK 2.0 but is roughly tuned.  It is used to help the low level controller (PID) and when the arm is in manual mode ("clutched"). 
* `body/set_cf_orientation_absolute`
  * *cisst*: write command `bool`
  * *ROS*: subscriber `std_msgs/Bool`
  * dVRK specific.  When using `body/servo_cf`, reference frame to apply the wrench is the end effector frame.  This makes sense for the position but can be confusing for the orientation.  For example, using the MTM, applying a contant force in Z direction feels like holding a rocket in your hand, the direction of the force will change as the user rotates the gripper.  To feel a force in a constant direction, independently of the hand's orientation, use `set_cf_orientation_absolute`.
* `trajectory_j/ratio`
  * *cisst*: event write `double`
  * *ROS*: publisher `std_msgs/Float64`
  * dVRK specific.  Ratio applied to both maximum velocity and acceleration used for joint trajectory generation.  If a user overrides the ratio using either the velocity or acceleration specific ratio, this value in undefined.  See `trajectory_j/set_ratio`.
* `trajectory_j/ratio_a`
  * *cisst*: event write `double`
  * *ROS*: publisher `std_msgs/Float64`
  * dVRK specific.  Ratio applied to maximum acceleration used for joint trajectory generation.  See `trajectory_j/set_ratio_a`.
* `trajectory_j/ratio_v`
  * *cisst*: event write `double`
  * *ROS*: publisher `std_msgs/Float64`
  * dVRK specific.  Ratio applied to maximum velocity used for joint trajectory generation.  See `trajectory_j/set_ratio_v`.
* `trajectory_j/set_ratio`
  * *cisst*: write command `double`
  * *ROS*: subscriber `std_msgs/Float64`
  * dVRK specific.  Set ratio applied to both maximum velocity and acceleration used for joint trajectory generation.  Ratio must be in range **]0, 1]**.  Default ratio is 1.  This is the recommended way to slow down arm trajectories.
* `trajectory_j/set_ratio_a`
  * *cisst*: write command `double`
  * *ROS*: subscriber `std_msgs/Float64`
  * dVRK specific.  Set ratio applied to maximum acceleration used for joint trajectory generation.  This is provided for backward compatibility and fine tunning but the recommend approach is to use `trajectory_j/set_ratio`.
* `trajectory_j/set_ratio_v`
  * *cisst*: write command `double`
  * *ROS*: subscriber `std_msgs/Float64`
  * dVRK specific.  Set ratio applied to maximum velocity used for joint trajectory generation.  This is provided for backward compatibility and fine tunning but the recommend approach is to use `trajectory_j/set_ratio`.

## ECM

C++ class is `mtsIntuitiveResearchKitArmECM`.

* `manip_clutch`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs::Joy`
  * dVRK specific.  Indicates if the clutch button on the ECM (located on top of the translation/insertion stage) is pressed or not.
* `endoscope_type`
  * *cisst*: event write `std::string`
  * *ROS*: publisher `std_msgs::String`
  * dVRK specific.
* `set_endoscope_type`
  * *cisst*: write command `std::string`
  * *ROS*: subscriber `std_msgs::String`
  * dVRK specific.

## MTM

C++ class is `mtsIntuitiveResearchKitArmMTM`.

* `gripper/measured_js`
  * *cisst*: read command `prmStateJoint`
  * *ROS*: publisher `sensor_msgs/JointState`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  `measured_js` for the MTM gripper.  The only field available is the position of the gripper.  These is no measurement available for velocity or effort.  
* `gripper/closed`:
  * *cisst*: event write `bool`
  * *ROS*: publisher `std_msgs/Bool`
  * dVRK specific.
* `gripper/pinch`
  * *cisst*: event void
  * *ROS*: publisher `std_msgs/Empty`
  * dVRK specific.  Provided for backward compatibility.  Same as `gripper/closed` is `true`. 
* `orientation_locked`
  * *cisst*: event write `bool`
  * *ROS*: publisher `std_msgs/Bool`
  * dVRK specific.
* `lock_orientation`
  * *cisst*: write command `vctMatRot3`
  * *ROS*: subscriber `geometry_msgs/Quaternion`
  * dVRK specific.
* `unlock_orientation`
  * *cisst*: void command
  * *ROS*: subscriber `std_msgs/Empty`
  * dVRK specific.

## PSM

C++ class is `mtsIntuitiveResearchKitArmPSM`.

* `jaw/measured_js`
  * *cisst*: read command `prmStateJoint`
  * *ROS*: publisher `sensor_msgs/JointState`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  `measured_js` for the PSM jaws.  Position, velocity and effort are provided.  Effort is based on the current feedback and can be affected by multiple factors so it is not an exact torque applied on the jaws.
* `jaw/setpoint_js`
  * *cisst*: read command `prmStateJoint`
  * *ROS*: publisher `sensor_msgs/JointState`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  `setpoint_js` for the PSM jaws.  
* `jaw/servo_jf`
  * *cisst*: write command `prmForceTorqueJointSet`
  * *ROS*: subscriber `sensor_msgs/JointState`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  `servo_jf` for the PSM jaws.
* `jaw/servo_jp`
  * *cisst*: write command `prmPositionJointSet`
  * *ROS*: subscriber `sensor_msgs/JointState`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  `servo_jp` for the PSM jaws.
* `jaw/move_jp`
  * *cisst*: write command `prmPositionJointSet`
  * *ROS*: subscriber `sensor_msgs/JointState`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  `move_jp` for the PSM jaws.
* `tool_type`
  * *cisst*: event write `std::string`
  * *ROS*: publisher `std_msgs/String`
  * dVRK specific.
* `tool_type_request`
  * *cisst*: event void
  * *ROS*: publisher `std_msgs/Empty`
  * dVRK specific.
* `set_tool_type`
  * *cisst*: write command `std::string`
  * *ROS*: subscriber `std_msgs/String`
  * dVRK specific.
* `set_adapter_present`
  * *cisst*: write command `bool`
  * *ROS*: subscriber `std_msgs/Bool`
  * dVRK specific.
* `set_tool_present`
  * *cisst*: write command `bool`
  * *ROS*: subscriber `std_msgs/Bool`
  * dVRK specific.
* `io/adapter`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.
* `io/tool`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.
* `io/manip_clutch`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.
* `io/suj_clutch`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.

## SUJ

# Tele-operation

## PSM Tele-operation

C++ class is `mtsTeleOperationPSM`.  Tele-operation topics for ROS are published under the "namespace" `MTMx_PSMx` (e.g. `MTML_PSM1`, `MTMR_PSM3`...).

* `current_state`
  * *cisst*: event write `std::string`
  * *ROS*: publisher `std_msgs/String`
  * dVRK specific.
* `desired_state`
  * *cisst*: event write `std::string`
  * *ROS*: publisher `std_msgs/String`
  * dVRK specific.
* `state_command`  Gravity compensation will be added based on last call to `use_gravity_compensation` (for MTMs and ECM).
  * *cisst*: write command `std::string`
  * *ROS*: subscriber `crtk_msgs/StringStamped`
  * dVRK specific.
* `following`
  * *cisst*: event write `bool`
  * *ROS*: publisher `std_msgs/Bool`
  * dVRK specific.
* `scale`
  * *cisst*: event write `double`
  * *ROS*: publisher `std_msgs/Float64`
  * dVRK specific.
* `set_scale`
  * *cisst*: write command `double`
  * *ROS*: subscriber `std_msgs/Float64`
  * dVRK specific.
* `align_mtm`
  * *cisst*: event write `bool` 
  * *ROS*: publisher `std_msgs/Bool`
  * dVRK specific.
* `alignment_offset`
  * *cisst*: read command `vctMatRot3`
  * *ROS*: publisher `geometry_msgs/QuaternionStamped`
  * dVRK specific.
* `set_align_mtm`
  * *cisst*: write command `bool`
  * *ROS*: subscriber `std_msgs/Bool`
  * dVRK specific.
* `rotation_locked`
  * *cisst*: event write `bool`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.
* `lock_rotation`
  * *cisst*: write command `bool`
  * *ROS*: subscriber `std_msgs/Bool`
  * dVRK specific.
* `translation_locked`
  * *cisst*: event write `bool`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.
* `lock_translation`
  * *cisst*: write command `bool`
  * *ROS*: subscriber `std_msgs/Bool`
  * dVRK specific.
* `set_registration_rotation` (obsolete)

## ECM Tele-operation

C++ class is `mtsTeleOperationECM`.

* `current_state`
  * *cisst*: event write `std::string`
  * *ROS*: publisher `std_msgs::String`
  * dVRK specific.
* `desired_state`
  * *cisst*: event write `std::string`
  * *ROS*: publisher `std_msgs::String`
  * dVRK specific.
* `state_command`
  * *cisst*: write command `std::string`
  * *ROS*: subscriber `std_msgs::String`
  * dVRK specific.
* `following`
  * *cisst*: event write `bool`
  * *ROS*: publisher `std_msgs::Bool`
  * dVRK specific.
* `scale`
  * *cisst*: event write `double`
  * *ROS*: publisher `std_msgs::Float64`
  * dVRK specific.
* `set_scale`
  * *cisst*: write command `double`
  * *ROS*: subscriber `std_msgs::Float64`
  * dVRK specific. Applies to translation only.

# Console

C++ class is `mtsIntuitiveResearchKitConsole`.

## General

* `console/power_off`
  * *cisst*: void command
  * *ROS*: subscriber `std_msgs/Empty`
  * dVRK specific.
* `console/power_on`
  * *cisst*: void command
  * *ROS*: subscriber `std_msgs/Empty`
  * dVRK specific.
* `console/home`
  * *cisst*: void command
  * *ROS*: subscriber `std_msgs/Empty`
  * dVRK specific.
* `console/camera`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.
* `console/clutch`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.
* `console/operator_present`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.
* `console/emulate_camera`
  * *cisst*: write command `prmEventButton`
  * *ROS*: subscriber `sensor_msgs/Joy`
  * dVRK specific.
* `console/emulate_clutch`
  * *cisst*: write command `prmEventButton`
  * *ROS*: subscriber `sensor_msgs/Joy`
  * dVRK specific.
* `console/emulate_operator_present`
  * *cisst*: write command `prmEventButton`
  * *ROS*: subscriber `sensor_msgs/Joy`
  * dVRK specific.
* `console/volume`
  * *cisst*: event write `double`
  * *ROS*: publisher `std_msgs/Float64`
  * dVRK specific.
* `console/set_volume`
  * *cisst*: write command `double`
  * *ROS*: subscriber `std_msgs/Float64`
  * dVRK specific.
* `console/string_to_speech`
  * *cisst*: write command `std::string`
  * *ROS*: subscriber `std_msgs/String`
  * dVRK specific.
* `console/beep`
  * *cisst*: write command `vctDoubleVec`
  * *ROS*: subscriber `std_msgs/Float64MultiArray`
  * dVRK specific.

## Tele-operation

* `console/teleop/enabled`
  * *cisst*: event write `bool`
  * *ROS*: publisher `std_msgs/Bool`
  * dVRK specific.
* `console/teleop/enable`
  * *cisst*: write command `bool`
  * *ROS*: subscriber `std_msgs/Bool`
  * dVRK specific.
* `console/teleop/scale`
  * *cisst*: event write `double`
  * *ROS*: publisher `std_msgs::Float64`
  * dVRK specific.
* `console/teleop/set_scale`
  * *cisst*: write command `double`
  * *ROS*: subscriber `std_msgs/Float64`
  * dVRK specific.
* `console/teleop/teleop_psm_selected`
  * *cisst*: event write `prmKeyValue`
  * *ROS*: publisher `diagnostic_msgs/KeyValue`
  * dVRK specific.
* `console/teleop/teleop_psm_unselected`
  * *cisst*: event write `prmKeyValue`
  * *ROS*: publisher `diagnostic_msgs/KeyValue`
  * dVRK specific.
* `console/teleop/cycle_teleop_psm_by_mtm`
  * *cisst*: write command `std::string`
  * *ROS*: subscriber `std_msgs/String`
  * dVRK specific.
* `console/teleop/select_teleop_psm`
  * *cisst*: write command `prmKeyValue`
  * *ROS*: subscriber `diagnostic_msgs/KeyValue`
  * dVRK specific.

## Foot pedals

* `footpedals/clutch`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.
* `footpedals/camera`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.
* `footpedals/cam_minus`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.
* `footpedals/cam_plus`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.
* `footpedals/bicoag`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.
* `footpedals/coag`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.

# Miscellaneous

## ISI focus controller

Using original focus controller from Intuitive Surgical.

* `endoscope_focus/locked`
  * *cisst*: event write `bool`
  * *ROS*: publisher `std_msgs/Bool`
  * dVRK specific.
* `endoscope_focus/focusing_in`
  * *cisst*: event write `bool`
  * *ROS*: publisher `std_msgs/Bool`
  * dVRK specific
* `endoscope_focus/focusing_out`
  * *cisst*: event write `bool`
  * *ROS*: publisher `std_msgs/Bool`
  * dVRK specific
* `endoscope_focus/lock`
  * *cisst*: command write `bool`
  * *ROS*: subscriber `std_msgs/Bool`
  * dVRK specific.
* `endoscope_focus/focus_in`
  * *cisst*: command write `bool`
  * *ROS*: subscriber `std_msgs/Bool`
  * dVRK specific
* `endoscope_focus/focus_out`
  * *cisst*: command write `bool`
  * *ROS*: subscriber `std_msgs/Bool`
  * dVRK specific

## dVRK focus controller

Using custom hardware and QLA/FPGA to control focus.