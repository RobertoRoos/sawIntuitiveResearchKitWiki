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
  - [Better use of redundancies](#better-use-of-redundancies)
  - [Better PSM teleoperation](#better-psm-teleoperation)
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
**[Improvement, core C++]**

## Encoder/potentiometer redundancy
Includes better filtering, maybe including some kind of delay to take into account the fact that the potentiometers are slow.  Filtering on the PC side tends to introduce more delay so maybe add filtering on the FPGA.   The current (simplistic) implementation is based on total of successive failures.<br>
Figure out how to use potentiometer on master roll (last active joint).  See [#25](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/issues/25).<br>
**[New feature, core C++]**

## PID component cleanup
The current PID implementation needs to be cleaned up.  All joint commands should use iterators instead of for loops and cisstVector methods on vector of joints.<br>
Fix velocity when in simulated mode.<br>
Maybe moving to JSON format could help but this might be an unnecessary incompatibility.<br>
**[Improvement, core C++]**

## Velocity control
Add special mode to PID to servo torque based on current velocity or any better approach.  In arm class, add code to convert cartesian velocities (likely in body frame) to joint velocities and use PID interface to control joint velocities.  Add ROS topics for this new feature.<br>
**[New feature, core C++]**

## Torque limits specific to tool
Add torque limits per joint in PID configurable by tool.  The JSON file for the tool is loaded by the PSM class so we need to add the proper interfaces to the PID component.<br>
**[New feature, core C++]**

## Gravity compensation
Develop an algorithm to automatically identify the center of mass and mass.  We assume inertia matrices are not needed at that point.  This can be calibration stage with an initial data collection using the ROS bridges, maybe directly from Matlab (that would require all groups to have the Robotics Systems Toolbox) or Numpy/Scipy, parameter identification offline, C++ code to adjust the joint torques at runtime.<br>
Some work has been done at WPI and there's a student working over summer '16 at JHU<br>
**[New feature, core C++ + ROS + Matlab/scipy]**

## Better use of redundancies
Positioning the wrist in effort mode<br>
Taking advantage of the symmetry of the master gripper and maximize the joint space overlap with PSM, see [#56](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/issues/56) and [#50](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/issues/50)<br>
**[New feature, core C++]**

## Better PSM teleoperation
Take joint PSM limits into account and provide force feedback on MTMs.<br>
Evaluate a different control, we currently use absolute positions since the follow mode started.  Maybe using relative motions since last command (incremental positions and/or velocities) would reduce the likelihood of jumps in the cartesian space.<br> 
**[Improvement, core C++]**

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
* Manually re-align MTMs to PSMs, i.e. user visually aligns the MTM and based on current position estimate the rotation matrix used for the teleoperation components.
* Calibrate stereo camera, some have used ROS tools based on OpenCV, others have used the latest Matlab toolkits.  It would be nice to have howto and maybe comparative results between both methods. 
* Using simple grid held by PSMs, register PSM to camera?
* Tooltracking?
**[New feature, ROS/Matlab, documentation]**

## Stereo endoscope focus
Add support in console to use foot pedals CAM+/- to control the focus.  See [#55](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/issues/55)<br>
See impact of focus change on camera calibration<br>
Maybe track and approximate camera focus to provide a better camera calibration estimate<br>
Autofocus?

## Motorized 3D camera
For dVRK users (i.e. users who don't have a full daVinci), alternative arm with stereo head.<br>
Maybe a simple pan tilt camera with ROS interface controllable from MTMs?

# Applications

## Potentiometer and DH calibration
We have a procedure to calibrate the scales but we're missing many offsets, the only one handled for now are the last 4 on PSMs.  Can we find a way to calibrate the pots on MTMs, ECM and first 3 on PSMs?<br?
Develop a procedure to collect 3D positions both based on a tracking system (likely optical) and based on encoders and then identify the ideal DH parameters.  JHU has a high school student working on similar project during summer '16.<br>
**[New feature, ROS, ...]**

## Dynamic simulation
Two different goals:
* Offline simulation
* Realtime simulation, research skill simulator?

### Gazebo
Gazebo reviews are mixed, some reported issues with stability and poor documentation but widely used in the ROS community, including for the DARPA project and at JHU with WAM arms.<br>
There seems to a fair amount of work done at WPI and some at JHU.  There's a JHU summer '16 undergrad project ongoing.<br>
**[New feature, Gazebo/ROS]**

### VRep
Some JHU users have experience with VRep.  They've been happy with user support, stable API.  Not as popular as Gazebo.
**[New feature, Gazebo/ROS]**

# Documentation

## Documented code examples
* Class derived from an existing arm
* Class derived from a teleoperation component (PSM or ECM)
* Python and Matlab examples, maybe more than get/set position?

## Repository of CAD files

E.g. custom cannula from Children's DC