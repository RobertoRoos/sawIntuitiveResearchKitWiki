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

### Selecting teleoperation pairs

## ECM teleoperation

### Behavior

### API