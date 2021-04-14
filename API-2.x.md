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
   * [Miscellaneous](#miscellaneous)
      * [ISI focus controller](#isi-focus-controller)
      * [dVRK focus controller](#dvrk-focus-controller)

<!-- Added by: anton, at: 2021-03-16T17:51-04:00 -->

<!--te-->

# Introduction

Each component of the dVRK described in the [software architecture](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Software-Architecture) provides a set of functionalities, i.e. commands/events for a [cisstMultiTask](https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-tutorial) component or topic/service for a ROS node (see [`dvrk_ros`](https://github.com/jhu-dvrk/dvrk-ros)`/dvrk_robot`).

In general, we try to expose most C++ commands and events as ROS topics or services under the same name.  Starting with the dVRK release 2.0, we are using the [CRTK naming convention](https://github.com/collaborative-robotics/documentation/wiki/Robot-API).  There are also some commands very specific to the dVRK not covered by CRTK.  ROS bridge for the dVRK specific commands can be found in [`dvrk_console.cpp`](https://github.com/jhu-dvrk/dvrk-ros/blob/master/dvrk_robot/src/dvrk_console.cpp).  For the CRTK commands, the [cisst to ROS CRTK bridge](https://github.com/jhu-cisst/cisst-ros) is used.

If you are migrating your *cisstMultiTask* or ROS code from the dVRK 1.7, you can find some porting information in the [`crtk-port`](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/devel/crtk-port).

# Arms

## All

C++ class is `mtsIntuitiveResearchKitArm`.  Arm names are typically all upper case and follow the da Vinci naming convention: MTML and MTMR for MTM left and right, PSM1, PSM2 and PSM3 for PSMs and finally ECM.  ROS topics are organized in a namespace using the arm's name (e.g. `MTMR/setpoint_cp`). 

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
* `query_cp`:
  * *cisst*: qualified read command
    * `vctDoubleVec`
    * `vctFrm4x4`
  * *ROS*: service `cisst_msgs/QueryForwardKinematics`
    * `sensor_msgs/JointState jp`
    * `geometry_msgs/PoseStamped cp`
  * dVRK specific.  Compute forward kinematic based on joint values provided.  The length of the vector of joint positions determines which frame should be computed along the kinematic chain.  For ROS, the field `jp.position` is used to store joint positions.  This method prepends the base frame for the arm to the result.
* `local/query_cp`:
  * *cisst*: qualified read command
    * `vctDoubleVec`
    * `vctFrm4x4`
  * *ROS*: service `cisst_msgs/QueryForwardKinematics`
    * `sensor_msgs/JointState jp`
    * `geometry_msgs/PoseStamped cp`
  * dVRK specific.  Compute forward kinematic based on joint values provided.  The length of the vector of joint positions determines which frame should be computed along the kinematic chain.  For ROS, the field `jp.position` is used to store joint positions.

### Motion commands

* `servo_cp`
  * *cisst*: write command `prmPositionCartesianSet`
  * *ROS*: subscriber `geometry_msgs/TransformStamped`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  Set cartesian position goal for low-level controller (PID).  **Use with caution**, goals should be reachable within a single clock tick (< 1 ms).  Use `move_cp` for large motions.
* `servo_jf`
  * *cisst*: write command `prmForceTorqueJointSet`
  * *ROS*: subscriber `sensor_msgs/JointState`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  Set joint effort goal for low-level controller (direct current control).  **Use with caution**, the only safety mechanism built in is the cap on maximum motor current.
* `servo_jp`
  * *cisst*: write command `prmPositionJointSet`
  * *ROS*: subscriber `sensor_msgs/JointState`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  Set joint position goal for low-level controller (PID).  **Use with caution**, goals should be reachable within a single clock tick (< 1 ms).  Use `move_jp` or `move_jr` for large motions.
* `servo_jr`
  * *cisst*: write command `prmPositionJointSet`
  * *ROS*: subscriber `sensor_msgs/JointState`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion). Set joint relative position goal for low-level controller (PID).  The goal is defined as an increment that will be added to the current `setpoint_jp`.  See also notes for `servo_jp`.
* `spatial/servo_cf`
  * *cisst*: write command `prmForceCartesianSet`
  * *ROS*: subscriber `geometry_msgs/WrenchStamped`
  * dVRK specific.  Set cartesian effort goal for low-level controller using `spatial/jacobian` (direct current control).  **Use with caution**, the only safety mechanism built-in is the cap on maximum motor current.  For most application, use `body/servo_cf`.  Gravity compensation will be added based on last call to `use_gravity_compensation` (for MTMs and ECM).  See [cisstRobot](https://github.com/jhu-cisst/cisst/wiki/cisstRobot-robManipulator).
* `body/servo_cf`
  * *cisst*: write command `prmForceCartesianSet`
  * *ROS*: subscriber `geometry_msgs/WrenchStamped`
  * dVRK specific.  Set cartesian effort goal for low-level controller using `body/jacobian` (direct current control).  **Use with caution**, the only safety mechanism built in is the cap on maximum motor current.  Useful for haptic on MTM.  By default direction of force is defined by the orientation of the end effector.  To use the absolute orientation, toggle on/off using `body/set_cf_orientation_absolute`.  Gravity compensation will be added based on last call to `use_gravity_compensation` (for MTMs and ECM).  See [cisstRobot](https://github.com/jhu-cisst/cisst/wiki/cisstRobot-robManipulator).  
* `set_cartesian_impedance_gains`
  * *cisst*: write command `prmCartesianImpedanceGains`
  * *ROS*: subscriber `cisst_msgs/prmCartesianImpedanceGains`
  * dVRK specific.  Apply wrench based on difference between measured and goal cartesian positions as well as twist (cartesian velocity).  The cartesian space is divided in 12 cases: negative and positive (**2**) error in position and orientation (**x 2**) along axes X, Y and Z (**x 3 = 12**).  The payload for this command includes 3 parameters for each case: a linear gain, a damping gain and an offset.  This command can be used to define a simple haptic virtual fixture (plane, line, point, box corner...).  **Use with caution**, specially if the frame used to compute the cartesian impedance is far from the current arm position as this could lead to strong forces applied to the arm.  Internally the dVRK code uses the class `osaCartesianImpedanceController` from the package [*sawControllers*](https://github.com/jhu-saw/sawControllers).
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
  * dVRK specific.  Set ratio applied to maximum acceleration used for joint trajectory generation.  This is provided for backward compatibility and fine tuning but the recommended approach is to use `trajectory_j/set_ratio`.
* `trajectory_j/set_ratio_v`
  * *cisst*: write command `double`
  * *ROS*: subscriber `std_msgs/Float64`
  * dVRK specific.  Set ratio applied to maximum velocity used for joint trajectory generation.  This is provided for backward compatibility and fine tuning but the recommended approach is to use `trajectory_j/set_ratio`.

## ECM

C++ class is `mtsIntuitiveResearchKitArmECM`.

* `manip_clutch`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs::Joy`
  * dVRK specific.  Indicate if the clutch button on the ECM (located on top of the translation/insertion stage) is pressed or not.
* `endoscope_type`
  * *cisst*: event write `std::string`
  * *ROS*: publisher `std_msgs::String`
  * dVRK specific.  Indicate which endoscope is currently in use.  Note that the endoscope type is not detected automatically so this setting depends on the user.  It can be modified using the GUI or programmatically.
* `set_endoscope_type`
  * *cisst*: write command `std::string`
  * *ROS*: subscriber `std_msgs::String`
  * dVRK specific.  Set the type of endoscope mounted on the ECM.  The endoscope type is used for two things.  Up/down/straight is used to compute the tool tip transformation for the forward kinematic.  HD/SD is used for gravity compensation, the HD camera head happens to be a bit heavier than the SD one (see  [video pipeline](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Video-Pipeline)).  Possible values are defined in file `components/code/mtsIntuitiveResearchKitEndoscopeTypes.cdg` ([cisstDataGenerator](https://github.com/jhu-cisst/cisst/wiki/cisstCommon-Data-Generator)): `NONE`, `SD_STRAIGHT`, `SD_UP`, `SD_DOWN`, `HD_STRAIGHT`, `HD_UP`, `HD_DOWN`.

## MTM

C++ class is `mtsIntuitiveResearchKitArmMTM`.

* `gripper/measured_js`
  * *cisst*: read command `prmStateJoint`
  * *ROS*: publisher `sensor_msgs/JointState`
  * [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API-motion).  `measured_js` for the MTM gripper.  The only field available is the position of the gripper.  These is no measurement available for velocity or effort.  
* `gripper/closed`:
  * *cisst*: event write `bool`
  * *ROS*: publisher `std_msgs/Bool`
  * dVRK specific.  Indicate if the gripper is closed or not based on a hard coded threshold (0.0).  This is provided for convenience and backward compatibility but users can instead use `gripper/measured_js`, `position[0]` with their own threshold and logic to determine if the gripper is closed or not.
* `gripper/pinch`
  * *cisst*: event void
  * *ROS*: publisher `std_msgs/Empty`
  * dVRK specific.  Provided for backward compatibility.  Same as `gripper/closed` is `true`. 
* `orientation_locked`
  * *cisst*: event write `bool`
  * *ROS*: publisher `std_msgs/Bool`
  * dVRK specific.  Indicate if the orientation is locked or not.  See `lock_orientation`.
* `lock_orientation`
  * *cisst*: write command `vctMatRot3`
  * *ROS*: subscriber `geometry_msgs/Quaternion`
  * dVRK specific.  Send an orientation goal for the orientation of the MTM with respect to its base frame.  A joint trajectory is used to reach the orientation goal.  Once the MTM has reached the desired orientation, it will maintain said orientation even when the arm moves.  This command has no effect if the MTM is not controlled in effort mode (i.e. `servo_cf`).  The best example of usage is to lock the MTM orientation (~wrist) when in clutch mode.  The operator can move around freely but the absolute orientation remains constant so the MTM is still aligned to the PSM when the user restart the tele-operation.
* `unlock_orientation`
  * *cisst*: void command
  * *ROS*: subscriber `std_msgs/Empty`
  * dVRK specific.  Free the orientation.  This is used when the operator ends the MTM to PSM clutch.

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
  * dVRK specific.  Indicate which tool is currently in use.  Note that the tool type can be determined in different ways depending on your hardware and configuration files.  See [Tool Detection](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Tool-Detection).
* `tool_type_request`
  * *cisst*: event void
  * *ROS*: publisher `std_msgs/Empty`
  * dVRK specific.  When using `MANUAL` [tool detection](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Tool-Detection), event that indicates that a new tool has been detected and the software needs to know which type of tool it is.  The tool type can also be set using the dropdown menu on the GUI PSM widget.
* `set_tool_type`
  * *cisst*: write command `std::string`
  * *ROS*: subscriber `std_msgs/String`
  * dVRK specific.  When using `MANUAL` [tool detection](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Tool-Detection), set the tool type.  Possible values are defined in file `components/code/mtsIntuitiveResearchKitToolTypes.cdg` ([cisstDataGenerator](https://github.com/jhu-cisst/cisst/wiki/cisstCommon-Data-Generator)).  A tool description file with a filename matching the tool name needs to be provided as well.  Description files can be found in `share/tool` for many common da Vinci tools.  The tool type can also be set using the dropdown menu on the GUI PSM widget.
* `set_adapter_present`
  * *cisst*: write command `bool`
  * *ROS*: subscriber `std_msgs/Bool`
  * dVRK specific.  Tell the console that the sterile adapter is present without any actual hardware detection of the adapter.  This can be used to force engaging a non-dVRK modified sterile adapter (see [hardware modification](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Hardware)).  **Use with caution**, this can lead to undesired motions if a tool is also inserted.  The vast majority of users should **not**, **ever** use this command.
* `set_tool_present`
  * *cisst*: write command `bool`
  * *ROS*: subscriber `std_msgs/Bool`
  * dVRK specific.   Tell the controller that a tool is present without any actual hardware detection of the tool.  This can be used to force engaging a tool without a Dallas chip (see [tool detection](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Tool-Detection)).  **Use with caution**, this can lead to undesired motions if the wrong tool is inserted.  The vast majority of users should **not**, **ever** use this command.
* `io/adapter`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.  Indicate if the sterile adapter is present or not.
* `io/tool`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.  Indicate if a tool is present or not.
* `io/manip_clutch`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.  Indicate if the manipulator clutch button is pressed or not.  This is the white button located on top of the translation stage on the PSM.  This button is used to release the PID on the arm and move it manually.
* `io/suj_clutch`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.  Indicate if the manipulator SUJ (Set Up Joints) clutch button is pressed or not.  This is the white button located on the side of the "horizontal" link of the PSM.  This button is used to release the brakes on the arm's SUJ if you happen to have the [dVRK SUJ controller](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Controller-Boxes#da-vinci-classic-setup-joint-controller).

## SUJ

# Tele-operation

## PSM Tele-operation

C++ class is `mtsTeleOperationPSM`.  Tele-operation components names are typically all upper case use the name of the MTM and PSM (e.g. for the cisst component: `MTMR-PSM1`).  Topics for ROS are published under the namespace `MTMx_PSMx` (e.g. `MTML_PSM1`, `MTMR_PSM3`...).  Note that the `-` is replaced by `_` as ROS doesn't support the minus character in namespaces and topics.

* `current_state`
  * *cisst*: event write `std::string`
  * *ROS*: publisher `std_msgs/String`
  * dVRK specific.  Current state.  Possible values are defined in `components/code/mtsTeleOperationPSM.cpp`: `DISABLED`, `SETTING_ARMS_STATE`, `ALIGNING_MTM` and `ENABLED`.
* `desired_state`
  * *cisst*: event write `std::string`
  * *ROS*: publisher `std_msgs/String`
  * dVRK specific.  Desired state.  Possible values are defined in `components/code/mtsTeleOperationPSM.cpp`: `DISABLED`, `ALIGNING_MTM` and `ENABLED`.
* `state_command`  Gravity compensation will be added based on last call to `use_gravity_compensation` (for MTMs and ECM).
  * *cisst*: write command `std::string`
  * *ROS*: subscriber `crtk_msgs/StringStamped`
  * dVRK specific.  Send command to change desired state.  Possible values are: `enable`, `disable` and `align_mtm`
* `following`
  * *cisst*: event write `bool`
  * *ROS*: publisher `std_msgs/Bool`
  * dVRK specific.  Indicate if the PSM is following the MTM.  This can only happen if the tele-operation is `ENABLED`, the user has engaged the MTM and the tele-operation is not clutched.  This can can be used to detect when the tele-operation component is actually sending commands to the PSM (using a combination of `servo_cp` and `jaw/servo_jp`).
* `scale`
  * *cisst*: event write `double`
  * *ROS*: publisher `std_msgs/Float64`
  * dVRK specific.  Indicate what is the current scaling factor between the MTM and PSM translations.
* `set_scale`
  * *cisst*: write command `double`
  * *ROS*: subscriber `std_msgs/Float64`
  * dVRK specific.  Set the scaling factor between the MTM and PSM translations.  This command changes the scale for this tele-operation component only.  **Use with caution**, it might be confusing for a user if both hands are not using the same scale.  User should most likely use the `console/teleop/set_scale` command instead.  This setting can also be changed using the GUI.
* `align_mtm`
  * *cisst*: event write `bool` 
  * *ROS*: publisher `std_msgs/Bool`
  * dVRK specific.  Indicates if the tele-operation component will attempt to align the MTM orientation (with respect to the stereo display) to the orientation of the PSM end effector (with respect to the camera).  See `set_align_mtm`.
* `alignment_offset`
  * *cisst*: read command `vctMatRot3`
  * *ROS*: publisher `geometry_msgs/QuaternionStamped`
  * dVRK specific.  Difference between the MTM orientation and PSM orientation.  When `align_mtm` is set, the difference is capped by the maximum threshold allowed to engage (i.e. start following mode).  The default threshold is defined in `components/include/sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h` and is set to 5 degrees.  When `align_mtm` is set to `false`, this allows to track the difference of orientation between MTM and PSM when the operator starts tele-operating.
* `set_align_mtm`
  * *cisst*: write command `bool`
  * *ROS*: subscriber `std_msgs/Bool`
  * dVRK specific.  Set wether the tele-operation component requires the MTM orientation to match the PSM orientation to start the following mode.  When set, the tele-operation component will attempt to orient the MTM to match the PSM orientation.   For alternate MTMs without motorized wrist, the operator will have to manually re-orient the MTM to match the PSM orientation.  Also when set, in clutch mode, the component will lock the MTM orientation and leave the position (x, y, z) free so the operator can re-position their hands while preserving the orientation.  By default `align_mtm` is set to `true` and it mimics the behavior of the clinical da Vinci systems.  Setting `align_mtm` to false allows relative orientation between the MTM and the PSM.  This can be useful for alternate MTMs with a smaller SO3 space (e.g. ForceDimension haptic systems or Phanton Omni).  This setting can also be changed using the GUI.
* `rotation_locked`
  * *cisst*: event write `bool`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.  Indicate if the rotation is locked.  See `lock_rotation`.
* `lock_rotation`
  * *cisst*: write command `bool`
  * *ROS*: subscriber `std_msgs/Bool`
  * dVRK specific.  Lock the orientation.  On the PSM side, the tele-operation component will only send translation commands and will not change the orientation of the tool tip.  On the MTM side, the component will lock the wrist (similar to clutch in following mode when `align-mtm` is set).  This setting can also be changed using the GUI.
* `translation_locked`
  * *cisst*: event write `bool`
  * *ROS*: publisher `sensor_msgs/Joy`Triggers power off sequence for the whole system.
  * dVRK specific.  Indicates if the translation is locked.  See `lock_translation`.
* `lock_translation`
  * *cisst*: write command `bool`
  * *ROS*: subscriber `std_msgs/Bool`
  * dVRK specific.  Lock the orientation.  On the PSM side, the tele-operation component will only send rotation commands and will not change the position of the tool tip.  There is no effect on the MTM side.  This setting can also be changed using the GUI.
* `set_registration_rotation` (obsolete): 

## ECM Tele-operation

C++ class is `mtsTeleOperationECM`.

* `current_state`
  * *cisst*: event write `std::string`
  * *ROS*: publisher `std_msgs::String`
  * dVRK specific.  See similar command for PSM tele-operation.
* `desired_state`
  * *cisst*: event write `std::string`
  * *ROS*: publisher `std_msgs::String`
  * dVRK specific.  See similar command for PSM tele-operation.
* `state_command`
  * *cisst*: write command `std::string`
  * *ROS*: subscriber `std_msgs::String`
  * dVRK specific.  See similar command for PSM tele-operation.
* `following`
  * *cisst*: event write `bool`
  * *ROS*: publisher `std_msgs::Bool`
  * dVRK specific.  See similar command for PSM tele-operation.
* `scale`
  * *cisst*: event write `double`
  * *ROS*: publisher `std_msgs::Float64`
  * dVRK specific.  See similar command for PSM tele-operation.
* `set_scale`
  * *cisst*: write command `double`
  * *ROS*: subscriber `std_msgs::Float64`
  * dVRK specific.  See similar command for PSM tele-operation.

# Console

C++ class is `mtsIntuitiveResearchKitConsole`.

## General

* `console/power_off`
  * *cisst*: void command
  * *ROS*: subscriber `std_msgs/Empty`
  * dVRK specific.  Trigger power off sequence for the whole system.
* `console/power_on`
  * *cisst*: void command
  * *ROS*: subscriber `std_msgs/Empty`
  * dVRK specific.  Trigger power on sequence for the whole system.  If some arms are in `FAULT` state, this method will first `disable` them. 
* `console/home`
  * *cisst*: void command
  * *ROS*: subscriber `std_msgs/Empty`
  * dVRK specific.  Triggers homing procedure for the whole system, including powering if the system is not yet powered.
* `console/camera`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.  Indicate if the console assumes it should tele-operate the ECM if all conditions are met: operator is present and tele-operation is enabled.  When in camera mode, the console disables the PSM tele-operation components (if any) and enables the ECM tele-operation component (if present).
* `console/clutch`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.  Indicate if the console assumes it should clutch the active tele-operation components.  The console component simple passes the clutch state to the tele-operation components who then have to handle the clutch.
* `console/operator_present`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.  Indicate if the console assumes the operator is present (either through a dead man switch/foot pedal, head sensor or emulated).
* `console/emulate_camera`
  * *cisst*: write command `prmEventButton`
  * *ROS*: subscriber `sensor_msgs/Joy`
  * dVRK specific.  Emulate the camera pedal press.
* `console/emulate_clutch`
  * *cisst*: write command `prmEventButton`
  * *ROS*: subscriber `sensor_msgs/Joy`
  * dVRK specific.  Emulate the clutch pedal press.
* `console/emulate_operator_present`
  * *cisst*: write command `prmEventButton`
  * *ROS*: subscriber `sensor_msgs/Joy`
  * dVRK specific.  Emulate the operator presence sensor.
* `console/volume`
  * *cisst*: event write `double`
  * *ROS*: publisher `std_msgs/Float64`
  * dVRK specific.  Indicates the current volume for beeps generated by the console.
* `console/set_volume`
  * *cisst*: write command `double`
  * *ROS*: subscriber `std_msgs/Float64`
  * dVRK specific.  Set the 
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
  * dVRK specific.  Indicates if the tele-operation is enabled at the console level. 
* `console/teleop/enable`
  * *cisst*: write command `bool`
  * *ROS*: subscriber `std_msgs/Bool`
  * dVRK specific.  Enable or disable the tele-operation at the console level.  When tele-operation is enabled for the console, the console will manage with tele-operation components should be enabled using the following logic:
    * Tele-operation components must be declared in the console JSON configuration file: `MTMR-PSM1`, `MTML-PSM2`, `MTMR-PSM3`, `MTML-MTMR-ECM`...
    * For PSM tele-operation, the pair (e.g. `MTMR-PSM1`) also needs to be selected (there should be one pair per MTM selected by default when the console starts).
    * Finally the console determines which type of tele-operation component should be enabled based on the *camera* input:
      * If the *camera* input is off (i.e. *camera* foot pedal not pressed), the console will enable the PSM tele-operation components that have been selected.
      * If the *camera* inout is on, the console will enable the ECM tele-operation component (if declared in the console configuration file)
    * When a tele-operation is enabled, it will perform its own logic before getting into `following` mode...
* `console/teleop/scale`
  * *cisst*: event write `double`
  * *ROS*: publisher `std_msgs::Float64`
  * dVRK specific.  Last scale sent to all tele-operation components.  If a scale is set directly for a specific tele-operation component (i.e. not using the console tele-operation scale), said component can potentially use a different scale from the others.
* `console/teleop/set_scale`
  * *cisst*: write command `double`
  * *ROS*: subscriber `std_msgs/Float64`
  * dVRK specific.  Set the scale for all the tele-operation components declared in the console configuration file.
* `console/teleop/teleop_psm_selected`
  * *cisst*: event write `prmKeyValue`
  * *ROS*: publisher `diagnostic_msgs/KeyValue`
  * dVRK specific.  Indicates which pairs of MTM-PSMs are currently selected (PSM tele-operation components).
* `console/teleop/teleop_psm_unselected`
  * *cisst*: event write `prmKeyValue`
  * *ROS*: publisher `diagnostic_msgs/KeyValue`
  * dVRK specific. Indicates which pairs of MTM-PSMs are currently unselected (PSM tele-operation components).
* `console/teleop/cycle_teleop_psm_by_mtm`
  * *cisst*: write command `std::string`
  * *ROS*: subscriber `std_msgs/String`
  * dVRK specific.  Cycle PSM tele-operation for a given MTM.  For example, if the console has the pairs `MTML-PSM2` and `MTML-PSM3` and `MTML-PSM2` is currently selected, using `cycle_teleop_by_mtm(MTML)` will unselect `MTML-PSM2` and select `MTML-PSM3`.  There is a special case hard-coded in the console code to mimic the behavior of a clinical da Vinci system.  A quick-tap on the clutch pedal will trigger a `cycle_teleop_psm_by_mtm` for the MTM that has been used for two PSM tele-operations declared in the console configuration file.
* `console/teleop/select_teleop_psm`
  * *cisst*: write command `prmKeyValue`
  * *ROS*: subscriber `diagnostic_msgs/KeyValue`
  * dVRK specific.  Selected a specific MTM-PSM tele-operation.  If the MTM is currently associated to a selected pair, said pair will first be unselected.

## Foot pedals

* `footpedals/clutch`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.  Indicates if the physical pedal *clutch* is released (`0`), pressed (`1`) or a quick tap happened (`2`).
* `footpedals/camera`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.  Indicates if the physical pedal *camera* is released (`0`), pressed (`1`) or a quick tap happened (`2`).
* `footpedals/cam_minus`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.  Indicates if the physical pedal *camera -* is released (`0`), pressed (`1`) or a quick tap happened (`2`).
* `footpedals/cam_plus`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.  Indicates if the physical pedal *camera +* is released (`0`), pressed (`1`) or a quick tap happened (`2`).
* `footpedals/bicoag`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.  Indicates if the physical pedal *bicoag* is released (`0`), pressed (`1`) or a quick tap happened (`2`).
* `footpedals/coag`
  * *cisst*: event write `prmEventButton`
  * *ROS*: publisher `sensor_msgs/Joy`
  * dVRK specific.  Indicates if the physical pedal *coag* is released (`0`), pressed (`1`) or a quick tap happened (`2`).

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