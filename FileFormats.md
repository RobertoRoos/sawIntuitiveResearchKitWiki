<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](https://github.com/thlorenz/doctoc)*

- [Kinematics (JSON)](#kinematics-json)
- [Console (JSON)](#console-json)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# Kinematics (JSON)

All kinematics files contains the DH parameters.

## PSMs

You will need a different PSM configuration file per tool since the DH parameters vary as well as actuator/joint coupling and joint limits.  The first 4 joints should be the same across all PSM configurations files.

You need to provide the DH for the whole PSM (7 joints) even if you have designed a custom tool which uses less actuators.  The same applies for the coupling matrices (7x7).  If you don't use the last actuators, set the diagonal elements to 1.

## DH parameters

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

### Tooltip offset

```js
  // rotation to match ISI convention (for read-only research API on commercial da Vinci)
  "tooltip-offset" : [[ 0.0, -1.0,  0.0,  0.0],
                      [ 0.0,  0.0,  1.0,  0.0],
                      [-1.0,  0.0,  0.0,  0.0],
                      [ 0.0,  0.0,  0.0,  1.0]]
```
 
Usually a rotation matrix to match ISI orientation and translation offset with respect to center of gripper.  If you want to define your forward and inverse kinematics using a different tooltip (e.g. using tip of gripper when closed), you can add a translation offset.

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

You need to provide 4 matrices, two for position coupling and 2 for effort coupling.  The actuator to joint should be the inverse of joint to actuator in both cases.  If you're not using some of the actuators/joints, make sure the product of the coupling matrix by it's inverse is still close to identity.

### Tool engage parameters

```js
  // angles used to engage the tool, in degrees or millimeters
  // make sure these are within the range of motion IN the cannula
  "tool-engage-position" : {
    "lower" : [0.0, 0.0, 0.0, -270.0, -15.0,  15.0, 0.0],
    "upper" : [0.0, 0.0, 0.0,  270.0,  15.0, -15.0, 0.0]
  }
```


# Console (JSON)

```js
{
    "io": {
        "period": 0.0005, // in seconds
        "port": 0 // default is 0
    }
    ,
    "operator-present": {
        "component": "io",
        "interface": "COAG"
    }
    ,
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
            "name": "PSM2",
            "type": "PSM",
            "io": "sawRobotIO1394-PSM2-32204.xml",
            "pid": "../sawControllersPID-PSM.xml",
            "kinematic": "../psm-large-needle-driver.json",
            "base-frame": {
                "component": "SUJ",
                "interface": "PSM2"
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
        ,
        {
            "name": "MTML",
            "type": "MTM",
            "io": "sawRobotIO1394-MTML-34863.xml",
            "pid": "../sawControllersPID-MTML.xml",
            "kinematic": "../mtml.json"
        }
    ]
    ,
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
        ,
        {
            "master": "MTML",
            "slave": "PSM2",
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
}
```