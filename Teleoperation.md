# Introduction

The teleoperation components provided along the dVRK are provided as examples of implementation.  As much as we would like to have an implementation as good as the teleoperation provided by Intuitive Surgical on their clinical systems, this is not yet the case.  The current implementation simply computes the cartesian displacement of the MTM since teleoperation started and attempts to replicate the same motion (with scaling and optional orientation change) on the PSM side.   There is no force feedback on the MTM to reflect errors in position on the PSM side (e.g. joint limits, obstacles...).  Furthermore, the current MTM controller doesn't take advantage of the extra degree of freedom to position the MTM arm away from the operator's hand (see issues #25 and #56).

This being said, the two current components (PSM and ECM teleoperation) can be used for simple tasks and as a basis for derived classes implementing user specific teleoperation logic (e.g. applying force feedback or haptic on MTMs).  Using the current implementations as base classes allows the programmers to focus on the teleoperation itself while all the state transitions can be inherited from the base class.

## PSM teleoperation

### Code

* [dVRK constants](/jhu-dvrk/sawIntuitiveResearchKit/blob/master/components/include/sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h) (some related to PSM teleoperation)
* [Header file](/jhu-dvrk/sawIntuitiveResearchKit/blob/master/components/include/sawIntuitiveResearchKit/mtsTeleOperationPSM.h)
* [Code file](/jhu-dvrk/sawIntuitiveResearchKit/blob/master/components/code/mtsTeleOperationPSM.cpp)

### Behavior

The current implementation primarily attempts to replicate the teleoperation transitions implemented on the Intuitive clinical systems.  One of the main design decision is to maintain the consistency between the orientation of the PSM with respect to camera with the MTM with respect to the display.  Since the MTM orientation is motorized, whenever the operation is not in "follow mode" (i.e. the PSM replicates the MTM motion), the MTM orientation should follow the orientation of the PSM. So:
* When the teleoperation pair is enabled, the MTM will try to move to the same orientation as the PSM
* If the operator applies too much force on the MTM and mis-align the MTM, the teleoperation will not start (the error threshold can be found in `mtsIntuitiveResearchKit.h`).  A message is displayed in the Qt console indicating that the user should release their grip.
* To start the follow mode, the current implementation also makes sure the MTM jaw angle matches the PSM gripper angle.  A message is displayed in the Qt console indicating that the user should pinch and/or release the master gripper.
* When the operator uses the clutch to reposition their hands, the orientation on the MTM is locked so it will stay aligned to the PSM.
* If there is a camera and the camera can move, since the PSM position is defined with respect to the camera, the MTM orientation will change.

### API

In general, see [configuration parameters](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Configuration-File-Formats#teleoperation-components).

* State:
  * The different states for the teleoperation components are defined in: [`mtsTeleOperationPSM.cpp`](/jhu-dvrk/sawIntuitiveResearchKit/blob/master/components/code/mtsTeleOperationPSM.cpp).
  * ROS topics:
* Scale:
  * The scale can be set per teleoperation pair or for all teleoperation components simultaneously via the console.   For most applications it probably makes sense to control the scale via the console.
  * ROS topics:
* Orientation:
  * The registration orientation matrix was initially provided so users could provide the rotation between the PSM reference frame and the MTM reference frame.  This rotation is applied to all MTM motions before being sent to the PSM.  It is not the recommended way to register the MTM and PSMs anymore.  Instead, we recommend to use the reference/base frame for all arms so their coordinate systems are properly aligned.  See [console configuration](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Configuration-File-Formats#base-frame)
  * ROS topics:
* Ignoring jaws:
  * In some cases, it might be useful to not control the jaws (e.g. custom tools, tools with attached sensor over the jaws...).   When the user chooses to ignore the jaws, the teleoperation component will not try to match the jaws and gripper angles before engaging the follow mode.
  * Configuration parameter: `ignore-jaws`
* Lock translation:
  * This setting allows to lock the translation.  In this mode, the translation part of the MTM motion is ignored, the orientation is sent to the PSM.  Furthermore the MTM position is locked (first 3 degrees of freedom).
  * ROS topics:
* Lock orientation:
* Align MTM:

### Selecting teleoperation pairs

On a clinical da Vinci system, there are usually 3 PSMs and only 2 MTMs.  So the user has to select which PSMs should be teleoperated at a given time (selected) and which one should be left alone (unselected).  The clinical system uses a menu (or buttons) on the console to set the configuration (e.g. MTMR will drive PSM1 and PSM3 while MTML will drive PSM2).  Then the operator can use a foot pedal to toggle the PSMs on the MTM configured to drive 2 PSMs.  For the da Vinci Classic and S, the operator had to do a "quick tap" on the clutch pedal.

In practice, for the dVRK, you can swap using the “Clutch” foot pedal if there are two PSMs controlled by a single MTM.  This is done using a “quick tap”, i.e. about 1/10 of a second tap on the clutch. This is similar to the Intuitive implementation on the clinical first two generations of daVincis.

To do a full swap, say from MTMR/PSM1 & MTML/PSM2 to MTMR/PSM2 & MTML/PSM1 you will need to use ROS topics.
The ROS topics are defined in https://github.com/jhu-dvrk/dvrk-ros/blob/master/dvrk_robot/src/dvrk_add_topics_functions.cpp#L41

We use the `diagnostic_msgs::KeyValue` which is a pair of strings to represent the MTM/PSM pair.

For example, if a user with two MTMs (MTML and MTMR) and two PSMs (PSM1 and PSM2) wants to swap the two PSMs while the application is running, they first have to declare all 4 possible combinations in the console JSON configuration file.  Then:
* In your case, you would start with:<br>
  Status: MTMR/PSM1 - MTML/PSM2
* Then use the topic `/dvrk/console/teleop/select_teleop_psm` with `MTMR/“”` (use empty string to free the MTMR)<br>
  Status:  MTMR/“" - MTML/PSM2
* Then assign PSM2 to MTMR: `/dvrk/console/teleop/select_teleop_psm MTMR/PSM2`<br>
  Status: MTMR/PSM2 - MTML/“”
* Finally assign PSM1 to MTML: `/dvrk/console/teleop/select_teleop_psm MTML/PSM1`<br>
  Status: MTMR/PSM2 - MTML/PSM1

While you’re changing the selected pairs, you should make sure your requests are valid and listen to the ROS topics:
* `/dvrk/console/teleop/teleop_psm_selected`
* `/dvrk/console/teleop/teleop_psm_unselected`

## ECM teleoperation

### Behavior

### API