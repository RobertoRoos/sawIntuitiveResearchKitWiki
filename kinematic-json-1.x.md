<!--ts-->
   * [Introduction](#introduction)
   * [PSMs](#psms)
      * [DH parameters](#dh-parameters)
      * [Tooltip offset](#tooltip-offset)
      * [Coupling matrices](#coupling-matrices)
      * [Tool engage parameters](#tool-engage-parameters)
      * [Tool joint limits](#tool-joint-limits)
      * [Base offset](#base-offset)
      * [Going to zero position when homing](#going-to-zero-position-when-homing)
   * [MTMs](#mtms)
      * [DH parameters](#dh-parameters-1)
      * [Base offset](#base-offset-1)
   * [ECM](#ecm)
      * [DH parameters](#dh-parameters-2)
      * [Tooltip offset](#tooltip-offset-1)
      * [Base offset](#base-offset-2)
      * [Going to zero position when homing](#going-to-zero-position-when-homing-1)

<!-- Added by: anton, at: 2021-04-05T15:29-04:00 -->

<!--te-->

# Introduction

Configuration file format for dVRK arms, version 1.x.  See [dVRK wiki](https://github.com/jhu-dVRK/sawIntuitiveResearchKit/wiki).

You can find examples of configuration files in the "shared" directory:
https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/master/share

**Pay close attention to units as we used different ones in different sections!**

# PSMs

Parsing is performed by the `Configure` method in `mtsIntuitiveResearchKitPSM`: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/code/mtsIntuitiveResearchKitPSM.cpp

You will need a different PSM configuration file per tool since the DH parameters vary as well as actuator/joint coupling and joint limits.  The first 4 joints should be the same across all PSM configurations files.

You need to provide the DH for the whole PSM (7 joints) even if you have designed a custom tool which uses less actuators.  The same applies for the coupling matrices (7x7).  If you don't use the last actuators, set the diagonal elements to 1.

## DH parameters

```json
"DH": {
  "links": [
    {
      "convention": "modified",
      "alpha":  1.5708, "A":  0.0000, "theta":  0.0000, "D":  0.0000,
      "type": "revolute",
      "mode": "active",
      "offset":  1.5708
    },
    ...
  ]
}
```

The DH is a list of links, loaded in the order they're found in the configuration file.  Note that values given as `1.5708` are automatically detected by the `robManipulator` parser as fractions of Pi and replaced by a more accurate value.  The current implementation doesn't load any mass/inertia information as these are not used for the PSMs (yet).

**DH uses SI units!**

## Tooltip offset

```json
  // rotation to match ISI convention (for read-only research API on commercial da Vinci)
  "tooltip-offset" : [[ 0.0, -1.0,  0.0,  0.0],
                      [ 0.0,  0.0,  1.0,  0.0],
                      [-1.0,  0.0,  0.0,  0.0],
                      [ 0.0,  0.0,  0.0,  1.0]]
```

Usually a rotation matrix to match ISI orientation and translation offset with respect to center of gripper.  If you want to define your forward and inverse kinematics using a different tooltip (e.g. using tip of gripper when closed), you can add a translation offset.

**Tooltip matrix uses SI units!**

## Coupling matrices

```json
  // values from the dVRK user guide, see tool appendix C
  "coupling" : {
    "ActuatorToJointPosition" : [[ 1.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000],
                                 [ 0.0000,  1.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000],
                                 [ 0.0000,  0.0000,  1.0000,  0.0000,  0.0000,  0.0000,  0.0000],
                                 [ 0.0000,  0.0000,  0.0000, -1.5632,  0.0000,  0.0000,  0.0000],
                                 [ 0.0000,  0.0000,  0.0000,  0.0000,  1.0186,  0.0000,  0.0000],
                                 [ 0.0000,  0.0000,  0.0000,  0.0000, -0.8306,  0.6089,  0.6089],
                                 [ 0.0000,  0.0000,  0.0000,  0.0000,  0.0000, -1.2177,  1.2177]],
    "JointToActuatorPosition" : [[...]],
    "ActuatorToJointEffort" : [[...]],
    "JointToActuatorEffort" : [[...]]
  }
```

For revision **1.4** and lower, you need to provide 4 matrices, two for position coupling and 2 for effort coupling.  The actuator to joint should be the inverse of joint to actuator in both cases.  If you're not using some of the actuators/joints, make sure the product of the coupling matrix by its inverse is still close to identity.

For revision **1.5** and higher, you only need to provide the actuator to joint position coupling matrix.

## Tool engage parameters

```json
  // angles used to engage the tool, in degrees or millimeters
  // make sure these are within the range of motion IN the cannula
  "tool-engage-position" : {
    "lower" : [0.0, 0.0, 0.0, -270.0, -15.0,  15.0, 0.0],
    "upper" : [0.0, 0.0, 0.0,  270.0,  15.0, -15.0, 0.0]
  }
```

This is a very important section.  During the engage tool procedure, the actuators will rotate to engage the mating disks between the sterile adapter and the tool.  Once the disks are engaged, the tool will start moving but there is no easy way to detect it so the actuator will keep moving.  If the tool is inside the cannula during the engage procedure, this will result to collisions and might damage your tool or even worse the PSM itself.

These values are the upper/lower joint positions used during the engage procedure (joint values, not actuator!).  Since the first 3 joints don't need to be engaged, the current C++ implementation will ignore whatever values you provide.  The last 4 values are used.  Note that for a needle driver the last joint (jaw opening) is not used (remains closed) but, because of the coupling, all last 4 actuators will still move.

If you're developing your own tool, you should start with a very small range.

**These parameters are in millimeters and degrees** to be more human readable.  These are converted internally in SI units.

## Tool joint limits

```json
  // values from dVRK user guide, converted to degrees or millimeters and floored (closest degree).
  // see PSM calibration for first 3 joints
  // see tool appendix for last 4 joints
  // for last joint, manual says [0, 30] but we need -10 to allow stronger torque, 80 to open wide
  "tool-joint-limit" : {
    "lower" : [ -91.0, -53.0,   0.0, -260.0, -80.0, -80.0, -10.0],
    "upper" : [  91.0,  53.0, 240.0,  260.0,  80.0,  80.0,  80.0]
  }
```

All parameters are provided by ISI.  You might need to play with the last joint lower limit to increase the power applied to the tool's jaws.

**These parameters are in millimeters and degrees** to be more human readable.  These are converted internally in SI units.

## Base offset

**This applies to version 1.4.0 and 1.5.0.  For version 1.6, see `"base-frame"` in `"arm"`**

One can specify a base offset, i.e. a fixed transformation added at the base of the kinematic chain.  This is an optional field, it can be used for systems without setup joints to make sure the PSMs cartesian positions are defined with respect to the camera.   For a moving camera, we recommend using the `SetBaseFrame` C++ command instead (see also ROS topic `set_base_frame`).

```json
  "base-offset" : [[ 1.0, 0.0, 0.0, 0.0],
                   [ 0.0, 1.0, 0.0, 0.0],
                   [ 0.0, 0.0, 1.0, 0.0],
                   [ 0.0, 0.0, 0.0, 1.0]]
```

**Base offset matrix uses SI units!**

## Going to zero position when homing

Since **version 1.4** the arm won't go to zero position during the homing procedure.  One can override this using the `homing-zero-position` boolean flag.  This field is optional, the default for PSMs and ECM is now to stay in place when homing.  MTMs always go to zero position when homed.

```json
  "homing-zero-position": 1 // or 0 for false
```

# MTMs

**This applies to version 1.4.0 and later**, older versions use `.rob` files.

Parsing is performed by the `Configure` method in `mtsIntuitiveResearchKitMTM`: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/code/mtsIntuitiveResearchKitMTM.cpp

We provide 3 files, `mtm.json`, `mtml-deprecated.json` and `mtmr-deprecated.json`.  These share the same DH parameters, the only difference is the base offset matrix.  For a single MTM, we assume no base offset, the origin is at the base of the kinematic chain.  For MTML and MTMR, the base offset re-orients the coordinate system to match the stereo display (angled at 30 degrees) and is centered on the eye piece.  This takes into account the distance between the two MTMs.  In **version 1.6** and above, it is recommended to use the `"arm"` `"base-frame"` to set the base frame of the arm.

## DH parameters

See PSMs section first.

```json
"DH": {
  "links": [
    {
      "convention": "standard",
      "alpha":  1.5708, "A":  0.0000, "theta":  0.0000, "D":  0.0000,
      "type": "revolute",
      "mode": "active",
      "offset":  -1.5708,
      "mass":    0.0000,
      "cx":   0.0000, "cy":   0.0000, "cz":   0.0000,
      "Ixx":  0.0001, "Iyy":  0.0001, "Izz":  0.0001,
      "x1":   1.0000, "x2":   0.0000, "x3":   0.0000,
      "y1":   0.0000, "y2":   1.0000, "y3":   0.0000,
      "z1":   0.0000, "z2":   0.0000, "z3":   1.0000
    },
    ...
  ]
}
```

The main difference between the PSM's DH and MTM's is that we need the mass, center of mass and inertia matrix.  We provide a rough estimate of these values but we will ultimately need a way to identify these parameters for each arm.

**DH uses SI units!**

## Base offset

```json
  // dummy example, it would take a 4x4 matrix
  "base-offset" : [[  1.0,  0.0,  0.0,  0.0],
                   [  0.0,  1.0,  0.0,  0.0],
                   [  0.0,  0.0,  1.0,  0.0],
                   [  0.0,  0.0,  0.0,  1.0]]
```

**Base offset matrix uses SI units!**   This base offset is included in the "local" kinematic chain and should not be specific to each user's setup.  For a transformation that represents where the arm is mounted, e.g. where is the MTM with respect to the display, use the "base-frame" field in the console JSON configuration file (introduced in **version 1.6***).

# ECM

**This applies to version 1.4.0 and later**, older versions use `.rob` files.

Parsing is performed by the `Configure` method in `mtsIntuitiveResearchKitECM`: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/code/mtsIntuitiveResearchKitECM.cpp

The `tooltip-offset` is used to change the camera coordinate system, it follows the ISI conventions and includes the scope angle.  For a straight endoscope, one should use `ecm-straight.json`.

## DH parameters

See PSM DH section.  For the ECM we only need 4 joints.  The current implementation doesn't load any mass/inertia information as these are not used for the ECM (but these would be nice when the ECM is manually operated so the user wouldn't have to lift the whole arm).

**DH uses SI units!**

## Tooltip offset

```json
  // rotation to match ISI convention (for read-only research API on commercial da Vinci) for a straight endoscope
  "tooltip-offset" : [[ 0.0, -1.0,  0.0,  0.0],
                      [-1.0,  0.0,  0.0,  0.0],
                      [ 0.0,  0.0, -1.0,  0.0],
                      [ 0.0,  0.0,  0.0,  1.0]]
```

Usually a rotation matrix to match ISI convention and the scope angle (straight, up, down).  Following the ISI convention, the PSMs motion in camera view should be x to the left, y down and z towards the users.

**Tooltip matrix uses SI units!**

## Base offset

One can specify a base offset, i.e. a fixed transformation added at the base of the kinematic chain.  This is an optional field, see PSM Base offset section.

## Going to zero position when homing

See PSMs.
