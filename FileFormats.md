<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](https://github.com/thlorenz/doctoc)*

- [Kinematics (JSON)](#kinematics-json)
- [Console (JSON)](#console-json)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

You can find examples of configuration files in the "shared" directory:
https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/master/share

**Pay close attention to units as we used different ones in different sections!**

# Kinematics (JSON)

## PSMs

Parsing is performed by the `Configure` method in `mtsIntuitiveResearchKitPSM`: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/code/mtsIntuitiveResearchKitPSM.cpp

You will need a different PSM configuration file per tool since the DH parameters vary as well as actuator/joint coupling and joint limits.  The first 4 joints should be the same across all PSM configurations files.

You need to provide the DH for the whole PSM (7 joints) even if you have designed a custom tool which uses less actuators.  The same applies for the coupling matrices (7x7).  If you don't use the last actuators, set the diagonal elements to 1.

### DH parameters

```js
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

### Tooltip offset

```js
  // rotation to match ISI convention (for read-only research API on commercial da Vinci)
  "tooltip-offset" : [[ 0.0, -1.0,  0.0,  0.0],
                      [ 0.0,  0.0,  1.0,  0.0],
                      [-1.0,  0.0,  0.0,  0.0],
                      [ 0.0,  0.0,  0.0,  1.0]]
```
 
Usually a rotation matrix to match ISI orientation and translation offset with respect to center of gripper.  If you want to define your forward and inverse kinematics using a different tooltip (e.g. using tip of gripper when closed), you can add a translation offset.

**Tooltip matrix uses SI units!**

### Coupling matrices

```js
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

You need to provide 4 matrices, two for position coupling and 2 for effort coupling.  The actuator to joint should be the inverse of joint to actuator in both cases.  If you're not using some of the actuators/joints, make sure the product of the coupling matrix by its inverse is still close to identity.

### Tool engage parameters

```js
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

### Tool joint limits

```js
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

## MTMs

** This applies to version 1.4.0 and later**

Parsing is performed by the `Configure` method in `mtsIntuitiveResearchKitMTM`: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/code/mtsIntuitiveResearchKitMTM.cpp

We provide 3 files, `mtm.json`, `mtml.json` and `mtmr.json`.  These share the same DH parameters, the only difference is the base offset matrix.  For a single MTM, we assume no base offset, the origin is at the base of the kinematic chain.  For MTML and MTMR, the base offset re-orients the coordinate system to match the stereo display (angled at 30 degrees) and is centered on the eye piece.  This takes into account the distance between the two MTMs.
 
### DH parameters

See PSMs section first.

```js
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

### Base offset

```js
  // transformation to match ISI convention (for read-only research
  // API on commercial da Vinci), uses stereo display as origin
  "base-offset" : [[ -1.0,  0.0,          0.0,          0.180],
                   [  0.0, -0.866025404, -0.5,         -0.400],
                   [  0.0, -0.5,          0.866025404, -0.325],
                   [  0.0,  0.0,          0.0,          1.0]]
```

**Tooltip matrix uses SI units!**

# Console (JSON)

## IO section

```js
{
    "io": {
        "period": 0.0005, // in seconds
        "port": 0 // default is 0
    }
```

## Arms

```js
    "arms":
    [
        {
            "name": "SUJ",
            "type": "SUJ",
            "io": "sawRobotIO1394-SUJ.xml",
            "kinematic": "suj-ECM-1-2-3.json",
            "base-frame": {
                "component": "ECM",
                "interface": "Robot"
            }
        }
        ,
        {
            "name": "PSM1",
            "type": "PSM",
            "io": "sawRobotIO1394-PSM1-49695.xml",
            "pid": "../sawControllersPID-PSM.xml",
            "kinematic": "../psm-large-needle-driver.json",
            "base-frame": {
                "component": "SUJ",
                "interface": "PSM1"
            }
        }
        ,
        {
            "name": "ECM",
            "type": "ECM",
            "io": "sawRobotIO1394-ECM-29738.xml",
            "pid": "../sawControllersPID-ECM.xml",
            "kinematic": "../dvecm.rob",
            "base-frame": {
                "component": "SUJ",
                "interface": "ECM"
            }
        }
        ,
        {
            "name": "MTMR",
            "type": "MTM",
            "io": "sawRobotIO1394-MTMR-22403-foot-pedal.xml",
            "pid": "../sawControllersPID-MTMR.xml",
            "kinematic": "../mtmr.json"
        }
    ]
```

## Teleoperation components

```js
    "operator-present": {
        "component": "io",
        "interface": "COAG"
    }
```

```js
    "psm-teleops":
    [
        {
            "master": "MTMR",
            "slave": "PSM1",
            "period": 0.005, // in seconds
            "rotation" : [[ 0.0000,  1.0000,  0.0000],
                          [ 1.0000,  0.0000,  0.0000],
                          [ 0.0000,  0.0000, -1.0000]]
        }
    ]
    ,
    "ecm-teleop":
    {
        "master-left": "MTML",
        "master-right": "MTMR",
        "slave": "ECM",
        "rotation" : [[ 1.0000,  0.0000,  0.0000],
                      [ 0.0000,  1.0000,  0.0000],
                      [ 0.0000,  0.0000,  1.0000]]
    }
```