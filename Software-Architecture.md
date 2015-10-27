<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

- [1. Overview](#1-overview)
- [2. Low level](#2-low-level)
  - [2.1. FPGA/QLA boards](#21-fpgaqla-boards)
  - [2.2. AmpIO library](#22-ampio-library)
- [3. C++](#3-c)
  - [3.1. Threads](#31-threads)
  - [3.2. Robot IOs](#32-robot-ios)
  - [3.3. PID controller](#33-pid-controller)
  - [3.4. Arm classes](#34-arm-classes)
  - [3.5. Tele-operation](#35-tele-operation)
  - [3.6. Console](#36-console)
- [4. Qt widgets](#4-qt-widgets)
  - [4.1. Robot IOs](#41-robot-ios)
  - [4.2. PID controller](#42-pid-controller)
  - [4.3. Arm classes](#43-arm-classes)
  - [4.4. Tele-operation](#44-tele-operation)
  - [4.5. Console](#45-console)
- [5. ROS](#5-ros)
  - [5.1. Topics](#51-topics)
  - [5.2. Python](#52-python)
  - [5.3. Matlab](#53-matlab)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# 1. Overview

The dVRK hardware and software stack is composed of:
* Firmware on FPGA/QLA interfacing IO with FireWire
* Lightweight C library on PC side to interface to FPGA via FireWire
* C++ components using the cisst/SAW libraries to implement IOs, controllers (PID, tele-operation), console, GUI, bridges to ROS, ...
* ROS wrapper around dVRK topics

# 2. Low level

## 2.1. FPGA/QLA boards
The embedded firmware performs:
  * Collect data from digital inputs data (limit/home switches)
  * Control digital outputs (ON/OFF/PWM)
  * Compute encoder positions and velocities, including detecting overflow and preload
  * Perform basic safety checks on motor current (consistency between requested and measured)
  * Implement subset of FireWire protocol to communicate with a computer
  * Maintain a watchdog to make sure the PC is still connected and communicating with the controller
  * See:
    * http://jhu-cisst.github.io/mechatronics
    * https://github.com/jhu-cisst/mechatronics-firmware

## 2.2. AmpIO library
C low level library:
  * Runs on the PC sides on top of Linux/libraw1394
  * Pack/unpack data to/from FPGA, i.e. convert bits to usable numbers (integers)
  * Handles multiple FPGA and treat them as single controller (
  * Simple text based programs to test hardware (`qladisp`, `qlatest`, `qlacloserelays`, ...)
  * See:
    * https://github.com/jhu-cisst/mechatronics-software/wiki
    * https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Hardware

# 3. C++

All C++ components are based on the cisst/SAW libraries, more specifically the cisstMultiTask framework:
* cisst libraries: https://github.com/jhu-cisst/cisst/wiki
* cisstMultiTask: [tutorial](https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-tutorial) and [concepts](https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-concepts)

The overall architecture is described in this picture:
![Overview](/jhu-dvrk/sawIntuitiveResearchKit/wiki/dVRK-component-thread-view.png)

The core components are colored in blue.

## 3.1. Threads

In general, each component owns a thread and can run at its own frequency.  There are a few notable exceptions:
 * There is a single IO component with multiple interfaces (one per arm connected).  This is required to avoid simultaneous accesses to the FireWire port (FireWire read/write are thread safe but processes can hang for a couple seconds).
 * The PID components could run in separate threads but this would introduce a fair amount of latency since the thread safe communication mechanisms in cisstMultiTask are based on queues.   Assuming a 1 millisecond period for both IO and PID, the PID would read some cached data (position and velocity) from the IO (between 0+ and 1 millisecond old) and then request a new effort.  This request being queued will be acted on between 0+ and 1 millisecond later.  Overall, the time between read and write could be as high as 2 milliseconds.  Instead, we used the cisstMultiTask ExecIn/ExecOut feature (see [cisstMultiTask concepts](https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-concepts) which allows to attach a component to another.  Effectively, the parent thread now runs the child's computation whenever needed.  In pseudo code:

   ```c++
  IO::Run(void) {
     ReadAllData(void);
     SaveReadDataInStateTable(void); // state tables are used to cache data
     ExecOut(); // trigger PID.Run() for all PID components attached to this IO
     ProcessQueuedCommands(); // dequeue all commands, including those from PID
  }
   ```

 * Qt manages its own thread(s)
 * The ROS bridges (cisst-ros) can be configured based on the user's needs (see below)

## 3.2. Robot IOs

The class mtsRobotIO1394 is part of the [sawRobotIO1394](https://github.com/jhu-saw/sawRobotIO1394) library.  It provides:
* conversion to from integer from SI units
* actuator/joint coupling for positions and efforts
* extra safety checks (consistency between potentiometers and encoders, compare required and measured current)
* configuration using XML, see sawRobotIO*.xml files in sawIntuitiveResearchKit [shared directory](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/master/share/jhu-daVinci)
* support for motors and brakes (e.g. dVRK ECM)

## 3.3. PID controller

The class mtsPID is part of the [sawControllers](https://github.com/jhu-saw/sawControllers) library.  It provides:
* a simple PID controller
* clamp requested position within joint limits
* check for PID tracking errors
* configuration using XML, see sawControllersPID*.xml files in sawIntuitiveResearchKit [shared directory](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/master/share)
* cisstMultiTask interfaces

## 3.4. Arm classes

## 3.5. Tele-operation

## 3.6. Console

# 4. Qt widgets

## 4.1. Robot IOs

## 4.2. PID controller

## 4.3. Arm classes

## 4.4. Tele-operation

## 4.5. Console

# 5. ROS

## 5.1. Topics

## 5.2. Python

## 5.3. Matlab