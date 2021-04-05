<!--ts-->
   * [Introduction](#introduction)
   * [Console (JSON)](#console-json)
      * [IO section](#io-section)
         * [Period and port](#period-and-port)
         * [Watchdog time-out](#watchdog-time-out)
         * [Foot pedals](#foot-pedals)
         * [FireWire protocol](#firewire-protocol)
      * [Other devices](#other-devices)
         * [Head sensor](#head-sensor)
         * [Endoscope focus](#endoscope-focus)
      * [Arms](#arms)
         * [Name](#name)
         * [Type](#type)
         * [Period](#period)
         * [IO](#io)
         * [PID](#pid)
         * [Simulation](#simulation)
         * [Base frame](#base-frame)
         * [Component and interface](#component-and-interface)
      * [Teleoperation components](#teleoperation-components)

<!-- Added by: anton, at: 2021-04-05T15:19-04:00 -->

<!--te-->

# Introduction

Configuration file format for the dVRK console, version 1.x.  See [dVRK wiki](https://github.com/jhu-dVRK/sawIntuitiveResearchKit/wiki).  This format is used by the application `sawIntuitiveResearchKitQtConsoleJSON` and the ROS application `dvrk_robot dvrk_console_json`.

Parsing is performed by the `Configure` method in `mtsIntuitiveResearchKitConsole`: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/code/mtsIntuitiveResearchKitConsole.cpp

The `Configure` method calls:
* `ConfigureArmJSON` to configure each arm
* `ConfigurePSMTeleopJSON` to configure the PSM teleoperation components
* `ConfigureECMTeleopJSON` to configure the ECM teleoperation components (from 1.4.0)

All files referenced within the `console-<your_setup>.json` file can be defined by an absolute path (not recommended) or a path relative to the console configuration file itself.  We recommend to keep all the files shared across systems in the provided `sawIntuitiveResearchKit/share` directory.  Then create a sub-directory for your system (e.g. `jhu-dVRK`) for all your files, e.g. `sawRobotIO1394` XML files and console JSON files.   In your console configuration files, you can refer to the shared configuration files (PID, kinematics) without any path.  By default, the console looks for files in the following directories:
 * Current directory, i.e. directory in which the application was started
 * Directory of the console file itself, this is how your IO files are usually found
 * Shared source directory for the dVRK, this is how the default PID and kinematics files are found
 * Shared source directory '/io'.  This folder is used to store share IO files without any system specific settings (foot pedals, camera focus, head sensor...)

Once you've created and populated your system specific directory, you can send us a github pull request or a zip.  We'll merge it with the `master` branch and sync the `devel` branch.

## IO section

This section allows to tweak the IO component, i.e. the component that interfaces with the controllers over FireWire/Ethernet and performs all the read/writes.

### Period and port

One can change the refresh rate (in this example, 1/2 millisecond) and the FireWire port (default is 0).  It is recommended to not include this section and use the default values.

In most cases, the PID components run in the same thread as the IO, so changing the IO period also changes the PID period.  See [software architecture](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Software-Architecture).

```json
  "io": {
    "period": 0.0003, // in seconds
    "port": 0 // default is 0
  }
```

### Watchdog time-out

In **version 1.5** and higher, you can specify the watchdog time-out using:
```json
  "io": {
    ...
    "watchdog-timeout": 0.03, // in seconds
    ...
  }
```
The watchdog time-out is set on the FPGA-QLA controllers.  If the controllers don't receive any message for a period exceeding the watchdog time-out, they will automatically turn off the power on all motors.   This is used if the physical communication is lost (unplugged wire) or if the application has crashed or is not sending commands fast enough.  The maximum value for the watchdog time-out is 300 ms.  Setting the time-out to zero turns off the watchdog and is not recommended.   This field is optional and it is recommended to not override the default.

### Foot pedals

Also in **version 1.5**, IOs configuration for the foot pedals can (and should) be separated from the arm configuration.   In **version 1.4** and lower, users had to maintain two sawRobotIO1394 XML configuration files for each MTM, one with the foot pedals IO configuration and one without.  In **version 1.4** the arms section is used to defined the arms only while the new option `"io": { "footpedals": }` allows user to re-use a predefined configuration for the foot pedals IO configuration.  For example, if the foot pedals are connected to an MTMR controller, you will need:
```json
    "io": {
        "footpedals": "sawRobotIO1394-MTMR-foot-pedals.xml"
    }
```

### FireWire protocol

In **version 1.4** and higher, you can also specify the FireWire protocol used to communicate between the PC and the controllers:
```json
    "io": {
       "firewire-protocol": "sequential-read-broadcast-write" // recommended for rev up to 1.5
    }
```
The following protocols are supported:
* `sequential-read-write`: the PC reads data from each board (2 FPGA/QLA per controller), performs its computations (conversions, safety checks, PID, ...) and then writes sequentially to each board (N reads, N writes).  This is the only protocol supported on older firmware (3 and below).
* `sequential-read-broadcast-write`: the PC reads sequentially but performs a single write for all the boards.  The write message is an array of values sent to the boards, each board access the data it needs by index (N reads, 1 write).  This is the default protocol for the dVRK controllers with firmware 4 and above.
* **experimental** `broadcast-read-write`: the PC sends a single query/synchronization to all boards, read values are aggregated in single packet over the bus and there's a single write (1 query, 1 read, 1 write).  This is the fastest protocol available but some FireWire cards seem to have trouble synchronizing the read packets.  You will have to test it on your hardware to see if it supports this protocol or not.

## Other devices

This section is used to configure some of the external devices found on a full da Vinci system.  These devices need to be connected to the dVRK controllers using custom cables.

### Head sensor

Introduced in **version 1.6**.  Instructions to connect to the da Vinci head sensor can be found [here](/jhu-dvrk/sawIntuitiveResearchKit/wiki/HeadSensor).  Once you have the cable, the console configuration file needs the following:

```json
    "operator-present": {
        "io": "sawRobotIO1394-MTMR-dv-head-sensor.xml"
    }
```

### Endoscope focus

Introduced in **version 1.6**.  Instructions to connect to the da Vinci endoscope focus controller can be found [here](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Full-da-Vinci).  Once you have the cable, the console configuration file needs the following:

```json
    "endoscope-focus": {
        "io": "sawRobotIO1394-MTML-dv-endoscope-focus.xml"
    }
```

## Arms

List of arms to be configured in your system.  Each arm needs a unique name, type and configuration file for kinematics.

### Name

The name of the arm.  In most cases this will be one of `MTML`, `MTMR`, `PSM1`, `PSM2`, `PSM3`, `ECM` or `SUJ`.  The name will be used to create all the interfaces for the component connections, Qt GUI and ROS topics.  For a dVRK arm, this will create a cisst/SAW component with the arm name.  For some other devices, e.g. `sawForceDimensionSDK`, the arm name will match a provided interface name (see Component and interface below).

### Type

The arm can be a dVRK arm: `ECM`, `PSM`, `MTM` or `SUJ`.  These are the predefined classes in `sawIntuitiveResearchKit` and the most commonly used.  For these base dVRK arms, the console class will create all the required components (IO, Qt Widgets and ROS topics).

One can also declare a derived arm using the types `ECM_DERIVED`, `PSM_DERIVED` or `MTM_DERIVED`.   For all derived types, the console will assume that the user has already created and added the arm to the component manager (see https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-concepts).  To create and add the component, one can either write their own `main.cpp` or use dynamic loading (see example of `component-manager` below).  For all derived classes, the console will automatically add the IO connections as well as the Qt GUI widgets and ROS topics.

Finally, one can use `ECM_GENERIC`, `PSM_GENERIC` or `MTM_GENERIC`.  In this case the console will not create any IO component, Qt widget nor ROS topics.  The only requirement is that this generic arm has all the required commands and events in its provided interfaces.  For example, a device supported by the ForceDimension SDK can be used as a generic master arm with the following configuration:
```json
    "component-manager": {
        "components":
        [
            {
                "shared-library": "sawForceDimensionSDK",
                "class-name": "mtsForceDimension",
                "constructor-arg": {
                    "Name": "ForceDimensionSDK"
                },
                "configure-parameter": "sawForceDimensionSDK-MTMR.json"
            }
        ]
    }
```
The above configuration tells the console class to:
  * Load a shared library names `sawForceDimensionSDK` (cisst/SAW wrapper for the Force Dimension SDK)
  * Create an object of type `mtsForceDimension` under the name `ForceDimensionSDK`
  * The configuration file contains the name of the interface to be created, i.e. `MTMR`

Then in the `arms` section, we can create a generic MTM using:
```json
    "arms":
    [
        {
            "name": "MTMR",  // created previously using custom components
            "type": "MTM_GENERIC",
            "component": "ForceDimensionSDK", // name of component
            "interface": "MTMR" // name of interface
        }
    ]
```

### Period

You can override the default periodicity of the arm class for all the classes .  For example adding `"period": 0.001` will set the periodicity to 0.001 (in seconds).  This works only for the dVRK and derived arm types.

### IO

This is required for all arms connected to the hardware using the component from sawRobotIO1394, i.e. all the dVRK arms and derived (unless you're using the simulation mode).  These files are specific to each arm since the contain the calibration information, i.e. `sawRobotIO1394-<your_arm>.xml`.

### PID

This is required for all the dVRK and derived arms, either in simulation mode or not.  It is strongly recommended to use the configuration files provided along the source code.  Since the console class has a search path that includes the directory where the default PID configurations files are, there is no need to copy the files from the source directory.

### Simulation

If the arm is simulated, one need to add `"simulation": "KINEMATIC"`.  In this case, `"io"` field is ignored (see https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/share/console-PSM1_KIN_SIMULATED.json).  The only simulation currently supported is "kinematic", i.e. the PID will set the measured positions based on the commanded positions.   This allows to test the kinematic but doesn't include any dynamic nor simulation of interactions with the work like Gazebo or VREP would.

### Base frame

There are two possible syntaxes to define the base frame for the arm, one for a static configuration and one for a moving base frame.  The syntax for a static configuration has been introduced in **version 1.6**.  It requires two fields:
  * `"reference-frame"`: name of the reference frame, i.e. string used for display as well as ROS tf broadcast
  * `"transform"`: a 4x4 transformation matrix
```json
    {
        "name": "MTMR",
        "type": "MTM",
        "io": "sawRobotIO1394-MTMR-28247.xml",
        "pid": "sawControllersPID-MTMR.xml",
        "kinematic": "mtm.json",
        "base-frame": {
            "reference-frame": "HRSV",
            "transform": [[ -1.0,  0.0,          0.0,         -0.180],
                          [  0.0,  0.866025404,  0.5,          0.400],
                          [  0.0,  0.5,         -0.866025404,  0.475],
                          [  0.0,  0.0,          0.0,          1.0]]
        }
    }
```
In the example above, we define the base frame of the MTMR using the ISI convention, i.e. the origin is just in the middle of the eye piece of the stereo display (`"HSRV"`), the Z axis is aligned with the display depth (hence the 30 degrees rotation) and shifted to the right (translation by -0.180 in X).

For a moving frame, the `base-frame` is retrieved in runtime using a cisst/SAW component and interface.  Therefore it requires two fields:
  * `"component"`: name of the component providing the base frame
  * `"interface"`: name of the provided interface with the command to retrieve the base frame
```json
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
```
In the above example, the PSM1 base frame is provided at runtime by the `SUJ` component.  This component has one provided interface per SUJ arm, appropriately named after the arm it carries (in this case `PSM1`).

### Component and interface

These two fields are only required for "generic" arms so the console knows which component/interface to connect to.  For example:
```json
    "arms":
    [
        {
            "name": "MTMR",  // created previously using custom components
            "type": "MTM_GENERIC",
            "component": "ForceDimensionSDK",
            "interface": "MTMR"
        }
   ]
```

## Teleoperation components

The `operator-present` is optional, i.e. the default behavior is to use the "io" component and the "COAG" button as a dead man switch.  You can use this field to select a different foot pedal.

```json
  "operator-present": {
    "component": "io",
    "interface": "COAG"
  }
```

You can then define multiple tele-operation components for different pairs of MTM/PSMs.  Please note that we still don't support multiple tools for a single arm like a real daVinci would so make sure each MTM and PSM is used only once.

```json
    "psm-teleops":
    [
        {
            "master": "MTMR",
            "slave": "PSM1",
            "period": 0.005 // in seconds
        }
    ]
    ,
    "ecm-teleop":
    {
        "master-left": "MTML",
        "master-right": "MTMR",
        "slave": "ECM"
    }
```

Please note that the `"period"` is optional and you should probably set it only if you have specific needs.  The `"rotation"` is used to re-align the master coordinate systems between the MTM(s) and PSM/ECM.  You might have a different matrix if you're not using the setup joints.

Please note that `"ecm-teleop"` requires an ECM.  This feature is available in version 1.6.0 and above.

For release **1.6** and above, there is a new scope for tele-operation components called "configure-parameter".   In this scope, one can define "rotation" using the same format as release **1.5** and below as well as "scale" (a floating point value, ideally greater than 0.0 and lesser than 1.0).  For PSM tele operation, one can also specify "ignore-jaw" which can be set to `true` or `false` if your master arm doesn't have a gripper or your slave arm doesn't have jaws.  For example:
```json
  "psm-teleops":
    [
        {
            "mtm": "MTMR",
            "psm": "PSM1",
            "configure-parameter": {
                "ignore-jaw": true
            }
        }
    ]
```
