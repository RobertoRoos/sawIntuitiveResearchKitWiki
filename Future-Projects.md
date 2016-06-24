For all projects that impact the dVRK software stack (i.e. sawIntuitiveResearchKit or dark-ros), proper tickets/issues should be created on github.

# Control

## Encoder/potentiometer redundancy

Includes better filtering, maybe including some kind of delay to take into account the fact that the potentiometers are slow.  Filtering on the PC side tends to introduce more delay so maybe add filtering on the FPGA.   The current (simplistic) implementation is based on total of successive failures.<br>
Figure out how to use potentiometer on master roll (last active joint).<br>
**[Improvement, core C++]**

## PID component cleanup

The current PID implementation needs to be cleaned up.  All joint commands should use iterators instead of for loops and cisstVector methods on vector of joints.<br>
Fix velocity when in simulated mode.<br>
Maybe moving to JSON format could help but this might be an unnecessary incompatibility.<br>
**[Improvement, core C++]**

## Velocity control

Add special mode to PID to servo torque based on current velocity or any better approach.  In arm class, add code to convert cartesian velocities (likely in body frame) to joint velocities and use PID interface to control joint velocities.  Add ROS topics for this new feature. 
**[Improvement, core C++]**

## Torque limits specific to tool

Add torque limits per joint in PID configurable by tool

## Gravity compensation



[edit]1.3 PSM cleanup

This includes:
Configuration files to load different tools<s> Only file created for LND, should be easy to create others
PID gains per configuration <s>joint limits.
Update DH based on tools
Re-write adapter/tool engage procedure
[edit]1.4 Tele-operation states

The current tele-operation components doesn't check things as simple as the master orientation when engaging.
Create base class for tele-operation to manage states
Implement current position based teleop using new base class
Add new tele using rates for positions and force feedback for orientation





## 

## Better use of redundancies
* Positioning the wrist in effort mode
* Taking advantage of the symmetry of the master gripper and maximize the joint space overlap with PSM

# Video

2.3 Read both Hall Effect sensors

This should be fairly simple, maybe goes with firmware to toggle mux without adding too much FireWire
[edit]2.4 Master gravity compensation

WPI has done some work, not sure they take into account parallel links
WPI code seems to be Matlab only
Calibration tool using Matlab, runtime in cisst/SAW C++?
[edit]2.5 Force feedback on masters

Add force feedback on master when PSM/ECM reaches joint limits
Others?
[edit]2.6 Calibration tools

There are a few parameters users should be able to re-calibrate:
Gripper (exists) but probably needs improvement
Potentiometers, find slopes from encoder (should be simple), find zeros for homing. Should generate new XML config file, maybe merge with .cal file?
[edit]2.7 Simulink

Provide examples and documentation, add to github/saw-dvrk or other organization on github
[edit]3 Video

[edit]3.1 RViz console

Create a virtual console with:
Live or simulated video
Maybe pro-view equivalent
Icons for dVRK status
[edit]3.2 Hand eye registration

For dVRK users, tools to:
calibrate stereo camera
using simple grid held by PSMs, register PSM to camera?
[edit]3.3 Motorized 3D camera

For dVRK users, alternative arm with stereo head.
[edit]4 Simulation


# Applications

4.2 Dynamic simulation

Two different goals:
Offline simulation
Realtime simulation, can emulate training simulator from ISI?
[edit]4.2.1 Gazebo
There seems to a fair amount of work done at WPI.
JHU would like to test and maybe use this. If we do use it we can support it and push this to the saw-dvrk github organization.
Reports from JHU users re. known bugs without fixes
[edit]4.2.2 VRep
Some JHU users have experience with VRep
Happy with user support, stable API
