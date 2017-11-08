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
    - [Base offset](#base-offset)
    - [Going to zero position when homing](#going-to-zero-position-when-homing)
  - [MTMs](#mtms)
    - [DH parameters](#dh-parameters-1)
    - [Base offset](#base-offset-1)
  - [ECM](#ecm)
    - [DH parameters](#dh-parameters-2)
    - [Tooltip offset](#tooltip-offset-1)
    - [Base offset](#base-offset-2)
    - [Going to zero position when homing](#going-to-zero-position-when-homing-1)
- [Console (JSON)](#console-json)
  - [IO section](#io-section)
  - [Arms](#arms)
  - [Teleoperation components](#teleoperation-components)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

You can find examples of configuration files in the "shared" directory:
https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/master/share

**Pay close attention to units as we used different ones in different sections!**

# Robot IO (XML)

These files are used to configure the IOs for each arm identified by its serial number.  They include the board IDs, digital inputs/outputs to use, conversion factors for the encoders, potentiometers, commanded current and current feedback as well as some safety parameters and coupling matrices (for MTMs).

## Automatic generation

The normal procedure to create your sawRobotIO1394 XML configuration files is:
* Obtain the calibration files from ISI (`.cal` files).  There will be one file per arm identified by its serial number (see [XML configuration](/jhu-dvrk/sawIntuitiveResearchKit/wiki/XMLConfig)).
* Generate the XML configuration files from the `.cal` files using the provided Matlab application (see [XML configuration](/jhu-dvrk/sawIntuitiveResearchKit/wiki/XMLConfig)).
* Calibrate each arm.  The different calibration procedures will update the XML configuration file for each arm (see [Calibration](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Calibration))

## Porting between releases

Some new dVRK release might require to port your RobotIO XML configuration files.  The safest approach is to re-generate and re-calibrate the configuration files for each release.  In some cases, it is possible to update the XML configuration files manually.

### Version 3 (dVRK 1.5)

Version 3 for the sawRobotIO1394 XML configuration files was introduced in dVRK release 1.5.  One can manually port from version 2 to 3 but since we introduced a new and improved current calibration procedure in release 1.5, it is strongly recommended to regenerate your XML configuration files and perform the normal calibration procedures.

If you are willing to update the files manually, you will need to change:
* `Version` should be set to `3`
* All `DigitalIn` in MTM configuration files should be removed.
* All unused `DigitalIn` in PSM and ECM configuration files should be removed.
* In MTM configuration files, remove all coupling matrices except `ActuatorToJointPosition`
* In PSM and ECM configuration files, remove the `Coupling` section altogether
* In all files, you need to add the potentiometers tolerance.  The defaults depend on the arm type, check existing configuration files in `share/jhu-dVRK`.  For example, an MTM should have:
    ```xml
 <Potentiometers Position="Joints">
         <Tolerance Axis="0" Distance="5.00" Latency="0.01" Unit="deg"/>
         <Tolerance Axis="1" Distance="5.00" Latency="0.01" Unit="deg"/>
         <Tolerance Axis="2" Distance="5.00" Latency="0.01" Unit="deg"/>
         <Tolerance Axis="3" Distance="5.00" Latency="0.01" Unit="deg"/>
         <Tolerance Axis="4" Distance="5.00" Latency="0.01" Unit="deg"/>
         <Tolerance Axis="5" Distance="5.00" Latency="0.01" Unit="deg"/>
         <Tolerance Axis="6" Distance="1000.00" Latency="0.01" Unit="deg"/>
         <Tolerance Axis="7" Distance="1000.00" Latency="0.01" Unit="deg"/>
      </Potentiometers>
```


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

For revision **1.4** and lower, you need to provide 4 matrices, two for position coupling and 2 for effort coupling.  The actuator to joint should be the inverse of joint to actuator in both cases.  If you're not using some of the actuators/joints, make sure the product of the coupling matrix by its inverse is still close to identity.

For revision **1.5** and higher, you only need to provide the actuator to joint position coupling matrix.

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

**This applies to version 1.4.0 and later**

One can specify a base offset, i.e. a fixed transformation added at the base of the kinematic chain.  This is an optional field, it can be used for systems without setup joints to make sure the PSMs cartesian positions are defined with respect to the camera.   For a moving camera, we recommend using the `SetBaseFrame` C++ command instead (see also ROS topic `set_base_frame`).
  
```js
  "base-offset" : [[ 1.0, 0.0, 0.0, 0.0],
                   [ 0.0, 1.0, 0.0, 0.0],
                   [ 0.0, 0.0, 1.0, 0.0],
                   [ 0.0, 0.0, 0.0, 1.0]]
```

**Base offset matrix uses SI units!**

### Going to zero position when homing

Since **version 1.4** the arm won't go to zero position during the homing procedure.  One can override this using the `homing-zero-position` boolean flag.  This field is optional, the default for PSMs and ECM is now to stay in place when homing.  MTMs always go to zero position when homed.
 
```js
  "homing-zero-position": 1 // or 0 for false
```

## MTMs

**This applies to version 1.4.0 and later**, older versions use `.rob` files.

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

**This applies to version 1.4.0 and later**, older versions use `.rob` files.

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

### Going to zero position when homing

See PSMs.

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

In **version 1.5** and higher, you can specify the watchdog time-out using:
```js
  "io": {
    ...
    "watchdog-timeout": 0.03, // in seconds
    ...
  }
```
The watchdog time-out is set on the FPGA-QLA controllers.  If the controllers don't receive any message for a period exceeding the watchdog time-out, they will automatically turn off the power on all motors.   This is used if the physical communication is lost (unplugged wire) or if the application has crashed or is not sending commands fast enough.  The maximum value for the watchdog time-out is 300 ms.  Setting the time-out to zero turns off the watchdog and is not recommended.   This field is optional.

Also in **version 1.5**, IOs configuration for the foot pedals can (and should) be separated from the arm configuration.   In **version 1.4** and lower, users had to maintain two sawRobotIO1394 XML configuration files for each MTM, one with the foot pedals IO configuration and one without.  In **version 1.4** the arms section is used to defined the arms only while the new option `"io": { "footpedals": }` allows user to re-use a predefined configuration for the foot pedals IO configuration.  For example, if the foot pedals are connected to an MTMR controller, you will need:
```js
  "io": {
    ...
    "footpedals": "sawRobotIO1394-MTMR-foot-pedals.xml"
    ...
  }
```
 
In **version 1.4** and higher, you can also specify the FireWire protocol used to communicate between the PC and the controllers:
```js
  "io": {
    "firewire-protocol": "sequential-read-broadcast-write" // recommended for rev up to 1.5
  }
```
The following protocols are supported:
* `sequential-read-write`: the PC reads data from each board (2 FPGA/QLA per controller), performs its computations (conversions, safety checks, PID, ...) and then writes sequentially to each board (N reads, N writes).  This is the only protocol supported on older firmware (3 and below).
* `sequential-read-broadcast-write`: the PC reads sequentially but performs a single write for all the boards.  The write message is an array of values sent to the boards, each board access the data it needs by index (N reads, 1 write).  This is the default protocol for the dVRK controllers with firmware 4 and above.
* **experimental** `broadcast-read-write`: the PC sends a single query/synchronization to all boards, read values are aggregated in single packet over the bus and there's a single write (1 query, 1 read, 1 write).  This is the fastest protocol available but some FireWire cards seem to have trouble synchronizing the read packets.  You will have to test it on your hardware to see if it supports this protocol or not.

## Arms

List of arms to be configured in your system.  Each arm needs a unique name, type and configuration file for kinematics.  The arm can be a physical one (`ECM`, `PSM`, `MTM` or `SUJ`) or a simulated one (`ECM`, `PSM` or `MTM`).  We don't support simulated SUJs yet.

If the arm is simulated, one need to add `"simulation": "KINEMATIC"` and the `"io"` field is ignored (see https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/share/console-PSM1_KIN_SIMULATED.json).

For a real arm, you need to provide the file name for the IO, i.e. `sawRobotIO1394-<your_arm>.xml`.

Finally you can override the default periodicity of the arm class.  For example adding `"period": 0.001` will set the periodicity to 0.001 (in seconds).
  
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

Please note that `"ecm-teleop"` requires an ECM.  This feature requires version 1.5.0 and above (i.e. it's not implemented yet).