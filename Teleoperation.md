# Introduction

The teleoperation components provided along the dVRK are provided as examples of implementations.  As much as we would like to have an implementation as good as the teleoperation provided by Intuitive Surgical on their clinical systems, this is not yet the case.  The current implementation simply computes the cartesian displacement of the MTM since teleoperation started and attempts to replicate the same motion (with scaling and optional orientation change) on the PSM side.   There is no feedback on the MTM to reflect error in position on the PSM side (e.g. joint limits, obstacles...).  Furthermore, the current MTM controller doesn't take advantage of the extra degree of freedom to position the arm away from the operator's hand (see issues #25 and #56).

This being said, the two current components (PSM and ECM teleoperation) can be used for simple tasks and as a basis for derived classes implementing user specific teleoperation logic (e.g. applying force feedback or haptic on MTMs).  Using the current implementations as base classes allows the programmers to focus on the teleoperation itself while all the state transitions can be inherited from the base class.

## PSM teleoperation

### Code

* [dVRK constants](/jhu-dvrk/sawIntuitiveResearchKit/blob/master/components/include/sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h) (some related to PSM teleoperation)
* [Header file](/jhu-dvrk/sawIntuitiveResearchKit/blob/master/components/include/sawIntuitiveResearchKit/mtsTeleOperationPSM.h)
* [Code file](/jhu-dvrk/sawIntuitiveResearchKit/blob/master/components/code/mtsTeleOperationPSM.cpp)

### Behavior

The current implementation primarily attempts to replicate the teleoperation transitions implemented on the Intuitive controllers.  One of the main design decision is to maintain the consistency between the orientation of the PSM with respect to camera with the MTM with respect to the display.  Since the MTM orientation is motorized, whenever the operation is not in "follow mode" (i.e. the PSM replicates the MTM motion), the MTM orientation should follow the orientation of the PSM. So:
* When the teleoperation pair is enabled, the MTM will try to move to the same orientation as the PSM
* If the operator applies too much force on the MTM and mis-align the MTM, the teleoperation will not start (the error threshold can be found in `mtsIntuitiveResearchKit.h`).
* To start the follow mode, the current implementation also makes sure the MTM jaw angle matches the PSM gripper angle.
* When the operator uses the clutch to reposition their hands, the orientation on the MTM is locked so it will stay aligned to the PSM.
* If there is a camera and the camera can move, since the PSM position with respect to the camera will change, the MTM must track the PSM orientation and re-align itself.

### API

* Scale:
* Orientation:
* Ignoring jaws:
* Lock translation:
* Lock orientation:
* Align MTM:

### Selecting teleoperation pairs

On a clinical da Vinci system, there are usually 3 PSMs and only 2 MTMs.  So the user has to select which PSMs should be teleoperation at a given time and which one should be unselected.  The clinical system uses a menu (or buttons) on the console to set the configuration (e.g. MTMR will drive PSM1 and PSM3 while MTML will drive PSM2).   Then the operator can use a foot pedal to toggle the PSMs on the MTM configured to drive 2 PSMs.  For the da Vinci Classic and S, the operator had to do a "quick tap" on the clutch pedal.

In practice, you can swap using the “Clutch” foot pedal if there are two PSMs controlled by a single MTM.  This is done using a “quick tap”, i.e. about 1/10 of a second tap on the clutch. This is similar to the Intuitive implementation on the clinical first two generations of daVincis.

To do a full swap MTMR/PSM1 & MTML/PSM2 to MTMR/PSM2 & MTML/PSM1 you will need to use ROS topics.
The ROS topics are defined in https://github.com/jhu-dvrk/dvrk-ros/blob/master/dvrk_robot/src/dvrk_add_topics_functions.cpp#L41

We use the diagnostic_msgs::KeyValue which is a pair of strings to represent the MTM/PSM pair.

In your case, you would start with:
status: MTMR/PSM1 - MTML/PSM2

Then use the topic /dvrk/teleop/select_teleop_psm with MTMR/“” (use empty string to free the MTMR)
status:  MTMR/“" - MTML/PSM2

Then assign PSM2 to MTMR: /dvrk/teleop/select_teleop_psm MTMR/PSM2
status: MTMR/PSM2 - MTML/“”

Finally assign PSM1 to MTML: /dvrk/teleop/select_teleop_psm MTML/PSM1
status: MTMR/PSM2 - MTML/PSM1

While you’re changing the selected pairs, you should make sure your requests are valid and listen to the ROS topics:
/dvrk//teleop/teleop_psm_selected
/dvrk//teleop/teleop_psm_unselected

## ECM teleoperation

### Behavior

### API