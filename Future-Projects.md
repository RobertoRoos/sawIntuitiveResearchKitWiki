<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](https://github.com/thlorenz/doctoc)*

- [Control](#control)
  - [MTMs grippers](#mtms-grippers)
  - [Encoder/potentiometer redundancy](#encoderpotentiometer-redundancy)
  - [PID component cleanup](#pid-component-cleanup)
  - [Velocity control](#velocity-control)
  - [Torque limits specific to tool](#torque-limits-specific-to-tool)
  - [Gravity compensation](#gravity-compensation)
  - [Better use of MTM redundancies](#better-use-of-mtm-redundancies)
  - [Better PSM teleoperation](#better-psm-teleoperation)
  - [ECM teleoperation](#ecm-teleoperation)
  - [Trajectory generation](#trajectory-generation)
  - [Support for ROS MoveIt!](#support-for-ros-moveit)
- [Video](#video)
  - [RViz console](#rviz-console)
  - [Calibration/registration](#calibrationregistration)
  - [Stereo endoscope focus](#stereo-endoscope-focus)
  - [Motorized 3D camera](#motorized-3d-camera)
- [Applications](#applications)
  - [Potentiometer and DH calibration](#potentiometer-and-dh-calibration)
  - [Dynamic simulation](#dynamic-simulation)
    - [Gazebo](#gazebo)
    - [VRep](#vrep)
- [Documentation](#documentation)
  - [Documented code examples](#documented-code-examples)
  - [Repository of CAD files](#repository-of-cad-files)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

For all projects that impact the dVRK software stack (i.e. sawIntuitiveResearchKit or dark-ros), proper tickets/issues should be created on github.

# Control

## MTMs grippers
The current calibration process uses the fully open angle and lightly closed angles.  We could also use the fully closed angle and maybe use a non linear mapping between the master grippers and tool's jaws.<br>
Use both Hall Effect sensors, we need to use one of the digital outs to toggle the mux.  See [#23](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/issues/23)<br>
After 1.7, mapping is a bit different.  The teleop code now gets the max angle for the PSM jaws and MTM grippers.  It then find the scale from 0 to max between PSM and MTM.   This allows to open fully and close as needed.  Negative angles (applying more torque) use the positive scale.  There is still room for improvement here.<br>
**[Improvement, core C++]**

## Encoder/potentiometer redundancy
Includes better filtering, maybe including some kind of delay to take into account the fact that the potentiometers are slow.  Filtering on the PC side tends to introduce more delay so maybe add filtering on the FPGA.   The current (simplistic) implementation is based on total of successive failures.  In release 1.6 we've introduced a new test based on distance and latency with parameters set per joint and per arm in XML configuration file.  This is not ideal but should be sufficient for most users.<br>
Figure out how to use potentiometer on master roll (last active joint).  See [#25](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/issues/25).<br>
**[New feature, core C++]**

## PID component cleanup
~~The current PID implementation needs to be cleaned up.  All joint commands should use iterators instead of for loops and cisstVector methods on vector of joints.<br>
Fix velocity when in simulated mode.~~<br>
Fix jumps/clamp when outside joint limit: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/issues/63.<br>
Maybe moving to JSON format could help but this might be an unnecessary incompatibility.<br>
**[Improvement, core C++]**

## Velocity control
Add special mode to PID to servo torque based on current velocity or any better approach.  In arm class, add code to convert cartesian velocities (likely in body frame) to joint velocities and use PID interface to control joint velocities.  Add ROS topics for this new feature.<br>
**[New feature, core C++]**

## Gravity compensation
~~Develop an algorithm to automatically identify the center of mass and mass.  We assume inertia matrices are not needed at that point.  This can be calibration stage with an initial data collection using the ROS bridges, maybe directly from Matlab (that would require all groups to have the Robotics Systems Toolbox) or Numpy/Scipy, parameter identification offline, C++ code to adjust the joint torques at runtime.  See [#3](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/issues/3).<br>
We would need this form the MTMs as well as the ECM so the clutch mode would be weightless.<br>
Some work has been done at WPI and there's a student working over summer '16 at JHU.~~  This has been provided by CUHK group and released with 1.7.<br>
**[New feature, core C++ + ROS + Matlab/scipy]**

## Better use of MTM redundancies
Positioning the wrist in effort mode, see [#2](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/issues/2)<br>
Taking advantage of the symmetry of the master gripper and maximize the joint space overlap with PSM, see [#56](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/issues/56).<br>
After 1.7, code was introduced to optimize the 4th joint of MTM (platform) when in free moving mode.  This is still work in progress for IK.<br>
**[New feature, core C++]**

## Better PSM teleoperation
Take joint PSM limits into account and provide force feedback on MTMs.<br>
Evaluate a different control, we currently use absolute positions since the follow mode started.  Maybe using relative motions since last command (incremental positions and/or velocities) would reduce the likelihood of jumps in the cartesian space.<br> 
**[Improvement, core C++]**

## ECM teleoperation
~~Implement ECM two hands teleoperation.<br>
An empty component was introduced in 1.4 but the actual teleoperation is not implemented (missing both ECM motion and MTM force feedback).~~  This was implemented in 1.6.<br>
**[New feature, core C++]**

## Trajectory generation
~~Current implementation uses robLSPB which always assumes an initial velocity 0.  We can't interrupt existing trajectory so it only works with blocking commands (wait for goal reached event).  Using Reflexxes RMLII we could have an implementation with trajectories picking up where the last one was.   There's some work done in cisstRobot and the dVRK repository (in branches feature-RML but it's incomplete, untested and the build is tricky (should use CMake external project to be portable).~~  As a side note, we should add support for trajectories in cartesian space, maybe interpolation position/Euler angles.<br>
**[Improvement, core C++]**

## Support for ROS MoveIt!
~~We could provide an interface accepting ROS Trajectory messages (http://docs.ros.org/jade/api/trajectory_msgs/html/msg/JointTrajectory.html), we already have the `joint_state` so that should be enough for joint space.  I'm not totally sure what would be needed for cartesian trajectories but I assume a better integration with TF is required.  The joint space trajectories shouldn’t be too hard to code on the C++ side, take a list of points (PT and/or PVT) and use the existing robQuintic code to implement a trajectory following mode.  There will be a need to spec a cisstParameterTypes message and add the conversion method from ROS to cisst.  An alternative solution would be to use a separate ROS node that would rely on the ros controller "FollowJointTrajectory", the communication with the dVRK can be done by re-implementing a hardware (HW) abstraction using ROS topics.~~  This is partially implemented in CRTK branches.<br>
**[New feature, core C++, ROS]**

# Video

## RViz console
Create a virtual console with:
* Live or simulated video
* Maybe pro-view equivalent
* Icons for dVRK status: tool need re-align, clutch pressed, ...
* Custom 3D widgets, maybe text viewers, drop tags, ...
**[New feature, ROS/RViz, maybe C++ RViz module]**

## Calibration/registration
For dVRK users, tools to:
* Manually re-align MTMs to PSMs, i.e. user visually aligns the MTM and based on current position estimate the rotation matrix between the PSM base coordinate system and the camera frame.
* Calibrate stereo camera, some have used ROS tools based on OpenCV, others have used the latest Matlab toolkits.  It would be nice to have howto and maybe comparative results between both methods. 
* Using simple grid held by PSMs, register PSM to camera?
* Tooltracking?
**[New feature, ROS/Matlab, documentation]**

## Stereo endoscope focus
~~Add support in console to use foot pedals CAM+/- to control the focus.  See [#55](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/issues/55) and [#50](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/issues/50)~~.  This has been implemented in 1.6.<br>
See impact of focus change on camera calibration<br>
Maybe track and approximate camera focus to provide a better camera calibration estimate<br>
In March 2020, we discovered that the focus unit has a motor with encoder and two joint limits.  We could bypass the whole ISI focus controller and use the dVRK controller instead.  One would need to develop an adapter board and all the code to control this "joint" using the existing `sawRobotIO` and `sawControllers/mtsPID` components.

## Motorized 3D camera
For dVRK users (i.e. users who don't have a full da Vinci), alternative arm with stereo head.<br>
Maybe a simple pan/tilt camera with ROS interface controllable from MTMs?  In any case, we should include a mechanism to report the position of the PSMs with respect to the camera.  In the case of a pan/tilt, the orientation is really what we are after.

# Applications

## Potentiometer and DH calibration
We have a procedure to calibrate the scales but we're missing many offsets, the only one handled for now are the last 4 on PSMs.  Can we find a way to calibrate the pots on MTMs, ECM and first 3 on PSMs?<br>
Develop a procedure to collect 3D positions both based on a tracking system (likely optical) and based on encoders and then identify the ideal DH parameters.  JHU had a high school student working on similar project during summer '16.<br>
**[New feature, ROS, ...]**

## Dynamic simulation
Two different goals:
* Offline simulation
* Real-time simulation, research skill simulator?

### Gazebo
Gazebo reviews are mixed, some reported issues with stability and poor documentation but widely used in the ROS community, including for the DARPA project and at JHU with WAM arms.<br>
There seems to a fair amount of work done at WPI and some at JHU.  There's a JHU summer '16 undergrad project ongoing.<br>
**[New feature, Gazebo/RViz/ROS]**

### VRep
Some JHU users have experience with VRep.  They've been happy with user support, stable API.  Not as popular as Gazebo.
**[New feature, VRep/ROS]**

# Documentation

## Test protocol
* Must compile code on a clean install, latest two LTS Ubuntu (e.g. 16.04 and 14.04)
* Test compilation with and without catkin/ROS
* Include examples with generic/derived arms in compilation
* Run Python/Matlab test scripts to make sure ROS topics have been updated on all ends
* Add test scripts checking basic transitions between joint/cartesian, add small random noise to PID measured positions in simulated mode to make sure we can differentiate measured vs commanded position
 
## Documented code examples
* Class derived from an existing arm
* ~~Class derived from a teleoperation component (PSM or ECM)~~
* ~~Python and Matlab examples, maybe more than get/set position?~~

## Repository of CAD files

E.g. custom cannula from Children's DC or PSM calibration potentiometer calibration plate (JHU or IC).