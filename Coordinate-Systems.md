<!--ts-->
<!--te-->

# Introduction

The original application for the dVRK/da Vinci is RAMIS (Robotically Assisted Minimally Invasive Surgery).  For this application, it makes sense to use the same convention for the system coordinates on both the surgeon's and patient's sides.  The ISI (Intuitive Surgical Inc) convention expects that X points to the left when viewed from the stereo display, Y should point up and Z away from the user.  Using the same convention greatly simplifies the implementation of teleoperation tasks.  From there it is important to always define the arm motions with respect to the task:
* MTMs with respect to the stereo display
* PSMs with respect to the camera coordinate systems

To make sure coordinate systems are properly aligned, one needs to first find the transformation between the arm base frame and the task frame (display or camera) using either a kinematic chain (SUJ) or a registration method.  Once this transformation is found, it has to be used as *base frame* for the arm.

# Individual arms

For each individual arm, the cartesian positions are based on:
* DH parameters: defined in JSON file, used to compute the forward kinematics using joint positions
* Kinematic chain offsets (optional):
  * Defined in JSON file
    * `base-offset`: constant offset prepended to the kinematic chain (all arms)
    * `tooltip-offset`: constant offset appended to the kinematic chain (PSM and ECM only, not MTM)
  * Most users should never deal with these, they are predefined for given PSM tools and ECM endoscopes
* Base Frame (`base-frame`):
  * Prepended to the whole kinematic chain, including `base-offset`
  * Can be:
     * defined in the console JSON configuration per arm using `base-frame`
     * set at runtime by command or ROS topic `set_base_frame`

For example, the MTM kinematic (DH parameters) start from the link 0, i.e. close to the attachment point/shoulder (see `mtm.json`) and has Z pointing up, X to the left and Y towards the user.  But the ISI convention expects that X points to the left when viewed from the stereo display, Y should point up and Z away from the user.  Furthermore the ISI convention places the origin in the middle of the stereo display (i.e. between the operator's eyes).  In practice MTMs are always mounted rigidly to a frame so we need to apply a constant rotation and translation to match the ISI convention.  The two masters are also mounted apart from each other so there is also a positive X translation for MTML and a negative X translation for MTMR.  The change of reference frame should be defined using the `base-frame` in the console configuration file (see [jhu-dVRK/share](/jhu-dvrk/sawIntuitiveResearchKit/blob/master/share/jhu-dVRK/console-MTMR-PSM1-MTML-PSM2-Teleop.json)).

For a dynamic change of reference frame one should use the `set_base_frame` command.  For example, this is used to make sure the PSMs are always defined with respect to the ECM tooltip (camera frame) if the camera can move.
* When used with the setup joints, the console class propagates the different base frames to make sure the PSMs are defined with respect to the camera frame (see [jhu-daVinci/share](/jhu-dvrk/sawIntuitiveResearchKit/blob/master/share/jhu-daVinci/console-SUJ-ECM-MTMR-PSM1-MTML-PSM2-Teleop.json)).  The transformation tree is described in section below.
* If you don't have access to the setup joints but have a way to register the PSMs to the camera, you can use the `set_base_frame` command on the PSMs.
* Finally, if the camera is fixed, you can use `base-frame` in your console configuration (similar to MTMs, see [jhu-dVRK/share](/jhu-dvrk/sawIntuitiveResearchKit/blob/master/share/jhu-dVRK/console-MTMR-PSM1-MTML-PSM2-Teleop.json)).  Note that the `base-frame` depends on where your PSMs are mounted with respect to your fixed camera.

There are two possible cartesian positions to query using commands:
* `measured_cp`: `base_frame * base-offset * DH * tooltip-offset`
* `local/measured_cp`: `base-offset * DH * tooltip-offset`


# Setup Joints

When setup joint are present, the following positions are reported:
* ECM frames:
  * ECM-SUJ-Local:
    * Defined with respect to patient cart origin
    * Uses `base-offset * DH * tooltip-offset` from `suj.json`
    * Defines ECM-RCM
  * ECM-SUJ:
    * Same as ECM-SUJ-Local
  * ECM-Local:
    * Defined with respect to the ECM RCM
    * Uses `DH * tooltip-offset` from `ecm-*.json`
    * `tooltip-offset` depends on type of scope (looking straight, up or down)
  * ECM:
    * **Camera frame defined with respect to patient cart**
    * Uses `ECM-SUJ * ECM-Local` (`ECM-SUJ` is the base frame)
* PSM frames
  * PSM-SUJ-Local:
    * See ECM-SUJ-Local
  * PSM-SUJ:
    * Defined with respect to camera frame
    * Uses `inverse(ECM) * PSM-SUJ-Local` (`inverse(ECM)` is the base frame)
  * PSM-Local:
    * See ECM-Local
    * `DH`, `coupling`, `tooltip-offset`... depend on type of tool
  * PSM:
    * **PSM tooltip frame defined with respect to camera frame**
    * Uses `PSM-SUJ * PSM-Local` (`PSM-SUJ` is the base frame)