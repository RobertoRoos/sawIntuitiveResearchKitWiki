<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](https://github.com/thlorenz/doctoc)*

- [Kinematics (JSON)](#kinematics-json)
  - [PSMs](#psms)
    - [DH parameters](#dh-parameters)
    - [Tooltip offset](#tooltip-offset)
    - [Coupling matrices](#coupling-matrices)
    - [Tool engage parameters](#tool-engage-parameters)
    - [Tool joint limits](#tool-joint-limits)
  - [MTMs](#mtms)
    - [DH parameters](#dh-parameters-1)
    - [Base offset](#base-offset)
- [Console (JSON)](#console-json)
  - [IO section](#io-section)
  - [Arms](#arms)
  - [Teleoperation components](#teleoperation-components)

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

### Base offset

One can specify a base offset, i.e. a fixed transformation added at the base of the kinematic chain.  This is an optional field, it can be used for systems without setup joints to make sure the PSMs cartesian positions are defined with respect to the camera.   For a moving camera, we recommend using the `SetBaseFrame` C++ command instead (see also ROS topic `set_base_frame`).
  
```js
  "base-offset" : [[ 1.0, 0.0, 0.0, 0.0],
                   [ 0.0, 1.0, 0.0, 0.0],
                   [ 0.0, 0.0, 1.0, 0.0],
                   [ 0.0, 0.0, 0.0, 1.0]]
```

**Base offset matrix uses SI units!**

## MTMs

**This applies to version 1.4.0 and later**, older versions still use `.rob` files.

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

**Base offset matrix uses SI units!**

## ECM

Parsing is performed by the `Configure` method in `mtsIntuitiveResearchKitECM`: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/code/mtsIntuitiveResearchKitECM.cpp

The `tooltip-offset` is used to change the camera coordinate system, it follows the ISI conventions and includes the scope angle.  For a straight endoscope, one should use `ecm-straight.json`. 

### DH parameters

See PSM DH section.  For the ECM we only need 4 joints.  The current implementation doesn't load any mass/inertia information as these are not used for the ECM (but these would be nice when the ECM is manually operated so the user wouldn't have to lift the whole arm).

**DH uses SI units!**

### Tooltip offset

```js
  // rotation to match ISI convention (for read-only research API on commercial da Vinci) for a straight endoscope
  "tooltip-offset" : [[ 0.0, -1.0,  0.0,  0.0],
                      [-1.0,  0.0,  0.0,  0.0],
                      [ 0.0,  0.0, -1.0,  0.0],
                      [ 0.0,  0.0,  0.0,  1.0]]
```
 
Usually a rotation matrix to match ISI convention and the scope angle (straight, up, down).  Following the ISI convention, the PSMs motion in camera view should be x to the left, y down and z towards the users.

**Tooltip matrix uses SI units!**

### Base offset

One can specify a base offset, i.e. a fixed transformation added at the base of the kinematic chain.  This is an optional field, see PSM Base offset section.

# Console (JSON)

Parsing is performed by the `Configure` method in `mtsIntuitiveResearchKitConsole`: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/code/mtsIntuitiveResearchKitConsole.cpp

The `Configure` method calls:
* `ConfigureArmJSON` to configure each arm
* `ConfigurePSMTeleopJSON` to configure the PSM teleoperation components
* `ConfigureECMTeleopJSON` to configure the ECM teleoperation components (from 1.4.0) 

All files referenced within the `console-<your_setup>.json` file can be defined by an absolute path (not recommended) or a path relative to the console configuration file itself.  We recommend to keep all the files shared across systems in the provided `sawIntuitiveResearchKit/share` directory.  Then create a sub-directory for your system (e.g. `jhu-dVRK`) for all your files, e.g. `sawRobotIO1394` XML files and console JSON files.   In your console configuration files, you can refer to the shared configuration files (PID, kinematics) without any path.  By default, the console looks for files in the following directories:
 * Current directory, i.e. directory in which the application was started
 * Directory of the console file itself, this is how your IO files are usually found
 * Shared source directory for the dVRK, this is how the default PID and kinematics files are found

Once you've created and populated your system specific directory, you can send us a github pull request or a zip file and we'll merge it with the master branch.

## IO section

This section allows to tweak the IO component, i.e. the component that interfaces with the controllers over FireWire/Ethernet and performs all the read/writes.  One can change the refresh rate (in this example, 1/2 millisecond) and the FireWire port (default is 0).  It is recommended to not include this section and use the default values.

In most cases, the PID components run in the same thread as the IO, so changing the IO period also changes the PID period.  See [software architecture](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Software-Architecture).

```js
  "io": {
    "period": 0.0003, // in seconds
    "port": 0 // default is 0
  }
```

## Arms

List of arms to be configured in your system.  Each arm needs a unique name, type and configuration file for kinematics.  The arm can be a physical one (`ECM`, `PSM`, `MTM` or `SUJ`) or a simulated one (`ECM`, `PSM` or `MTM`).  We don't support simulated SUJs yet.

It the arm is simulated, one need to add `"simulation": "KINEMATIC"` and the `"io"` field is ignore (see https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/share/console-PSM1_KIN_SIMULATED.json).

For a real arm, you need to provide the file name for the IO, i.e. `sawRobotIO1394-<your_arm>.xml`.
  
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
      "pid": "sawControllersPID-PSM.xml",
      "kinematic": "psm-large-needle-driver.json",
      "base-frame": {
        "component": "SUJ",
        "interface": "PSM1"
      }
    }
  ]
```

The `base-frame` field is used only in combination with the SUJs.

## Teleoperation components

The `operator-present` is optional, i.e. the default behavior is to use the "io" component and the "COAG" button as a dead man switch.  You can use this field to select a different foot pedal.

```js
  "operator-present": {
    "component": "io",
    "interface": "COAG"
  }
```

You can then define multiple tele-operation components for different pairs of MTM/PSMs.  Please note that we still don't support multiple tools for a single arm like a real daVinci would so make sure each MTM and PSM is used only once.

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

Please note that the `"period"` is optional and you should probably set it only if you have specific needs.  The `"rotation"` is used to re-align the master coordinate systems between the MTM(s) and PSM/ECM.  You might have a different matrix if you're not using the setup joints.

Please note that `"ecm-teleop"` requires an ECM.  This feature requires version 1.4.0 and above.