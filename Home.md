# sawIntuitiveResearchKit Documentation

# **Firmware**
Firmware version 3.0 is now required, please upgrade.
https://github.com/jhu-cisst/mechatronics-firmware/wiki/FPGA-Program 

# 1. Introduction

![Controller with Patient Side Manipulators (PSMs)](/jhu-dvrk/sawIntuitiveResearchKit/wiki/ControllerWithPSM.jpg)

The sawIntuitiveResearchKit folder provides several example applications for controlling the Research Kit for da Vinci System using the [mechatronics:wiki IEEE-1394 (Firewire) controller]. The picture to the right shows two Controllers connected to two da Vinci Patient Side Manipulators (PSMs) at Worcester Polytechnic Institute (WPI).

The software applications use some or all of the following SAW components:
* mtsRobotIO1394 (sawRobotIO1394 folder) - interface to IEEE-1394 (Firewire) controller boards
* mtsPID (sawControllers folder) - PID controller used for MTM and PSM robots
* mtsTeleoperation (sawControllers folder) - Teleoperation component
* mtsTextToSpeech (sawTextToSpeech folder) - Text to speech component (used for warning and error messages)

The applications contain graphical user interfaces written in Qt, and make use of the following SAW Qt components:
* mtsRobotIO1394QtWidget (sawRobotIO1394 folder)
* mtsPIDQtWidget (sawControllers folder)
* mtsTeleoperationQtWidget (sawControllers folder)

The components are cross-platform, except for mtsRobotIO1394, which relies on a low-level IEEE-1394 interface library (`libraw1394`) that is primarily available on Linux. Thus, the build instructions focus on Linux. For setting up the Firewire interface on Linux, see [this page](/jhu-cisst/mechatronics-software/wiki/Development-Environment).

A ROS interface is available via mtsROSBridge (sawROS folder).

Related sites:
* Google group https://groups.google.com/d/forum/research-kit-for-davinci and research-kit-for-davinci@googlegroups.com
* Intuitive Surgical hardware wiki http://research.intusurg.com/dvrk
* Johns Hopkins University Mechatronics http://jhu-cisst.github.io/mechatronics
* Video introduction on YouTube http://www.youtube.com/watch?v=0n_9CgNSx6Y
* [Google map](https://mapsengine.google.com/map/embed?mid=z14AfgTT1a9w.ktOc3SMAsVF4) with current and future sites 

## 2. Updates

* *May 2013*: Initial Public Release

## 3. Tutorials

### 3.1. Getting Started

* [First Steps](/jhu-dvrk/sawIntuitiveResearchKit/wiki/FirstSteps)
* [Setting up development environment](/jhu-cisst/mechatronics-software/wiki/Development-Environment)
* [Software build instructions](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Build)
* [Generating XML configuration files](/jhu-dvrk/sawIntuitiveResearchKit/wiki/XMLConfig)
* [Hardware setup and testing](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Hardware)
* [Calibrating and updating XML configuration files](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Calibration)
* [Running provided examples](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Examples)
* [E-STOP](/jhu-dvrk/sawIntuitiveResearchKit/wiki/ESTOP)

### 3.2. Advanced 

* [Build with ROS](/jhu-dvrk/dvrk-ros)
* [Simulate MTM/PSM with ROS](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Simulation)

## 4. Miscellaneous

* [Frequently Asked Questions](/jhu-dvrk/sawIntuitiveResearchKit/wiki/FAQ)
* [Head Sensor](/jhu-dvrk/sawIntuitiveResearchKit/wiki/HeadSensor)
* [JHU DVRK Hardware Status](JHU-DVRK-Hardware-Status)