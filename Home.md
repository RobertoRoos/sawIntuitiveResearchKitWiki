<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

- [**Firmware**](#firmware)
- [Introduction](#introduction)
- [Updates](#updates)
- [Documentation](#documentation)
  - [Getting Started](#getting-started)
  - [Advanced](#advanced)
  - [Controller](#controller)
  - [Publications](#publications)
- [Miscellaneous](#miscellaneous)
- [Acknowledgments](#acknowledgments)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# **Firmware**
Firmware version 3.0 or 4.0 is now required, please upgrade.  Version 4.0 adds support for FireWire broadcasts, which enables faster I/O performance.
https://github.com/jhu-cisst/mechatronics-firmware/wiki/FPGA-Program 

# Introduction

![Controller with Patient Side Manipulators (PSMs)](/jhu-dvrk/sawIntuitiveResearchKit/wiki/ControllerWithPSM.jpg)

The sawIntuitiveResearchKit folder provides several example applications for controlling the Research Kit for the da Vinci System using the [IEEE-1394 (FireWire) controller](http://jhu-cisst.github.io/mechatronics/). The picture above shows two Controllers connected to two da Vinci Patient Side Manipulators (PSMs) at Worcester Polytechnic Institute (WPI).

The software applications use some or all of the following SAW components (and Qt widgets):
* [mtsRobotIO1394](https://github.com/jhu-saw/sawRobotIO1394) - interface to IEEE-1394 (FireWire) controller boards
* [mtsPID](https://github.com/jhu-saw/sawControllers) - PID controller used for MTM and PSM robots
* [mtsTeleoperation] (https://github.com/jhu-saw/sawControllers) - Teleoperation component
* [mtsTextToSpeech] (https://github.com/jhu-saw/sawTextToSpeech) - Text to speech component (for warning and error messages)

The components are cross-platform, except for mtsRobotIO1394, which relies on a low-level IEEE-1394 interface library (`libraw1394`) that is primarily available on Linux. Thus, the build instructions focus on Linux. For setting up the FireWire interface on Linux, see [this page](/jhu-cisst/mechatronics-software/wiki/Development-Environment).

A ROS interface is available via [mtsROSBridge](https://github.com/jhu-cisst/cisst-ros) base class and [dVRK programs and files](https://github.com/jhu-dvrk/dvrk-ros).

Related sites:
* Google group https://groups.google.com/d/forum/research-kit-for-davinci and research-kit-for-davinci@googlegroups.com
* Intuitive Surgical hardware wiki http://research.intusurg.com/dvrk
* *cisst* libraries http://github.com/jhu-cisst/cisst/wiki
* Johns Hopkins University Mechatronics http://jhu-cisst.github.io/mechatronics
* Video introduction on YouTube http://www.youtube.com/watch?v=0n_9CgNSx6Y
* [Google map](https://mapsengine.google.com/map/embed?mid=z14AfgTT1a9w.ktOc3SMAsVF4) with current and future sites 
* List of all JHU LCSR Software http://jhu-lcsr.github.io/software/

# Updates

* *January 2016*: Version 1.3.0 released, see latest change logs:
  * [sawIntuitiveResearchKit](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/CHANGELOG.md)
  * [dvrk-ros](https://github.com/jhu-dvrk/dvrk-ros/blob/master/CHANGELOG.md)
  * [cisst](https://github.com/jhu-cisst/cisst/blob/master/CHANGELOG.md)
  * [cisst-ros](https://github.com/jhu-cisst/cisst-ros/blob/master/CHANGELOG.md)
  * [sawRobotIO1394](https://github.com/jhu-saw/sawRobotIO1394/blob/master/CHANGELOG.md)
  * [sawControllers](https://github.com/jhu-saw/sawControllers/blob/master/CHANGELOG.md)
* *October 2015*: Version 1.2.0 released
* *April 2015*: Version 1.1.0 released
* *April 2014*: Moved to GitHub
* *May 2013*: Initial Public Release

# Publications

* Kazanzides P, Chen Z, Deguet A, Fischer GS, Taylor RH, DiMaio SP, [An Open-Source Research Kit for the da Vinci Surgical System, International Conference on Robotics and Automation](/jhu-dvrk/sawIntuitiveResearchKit/wiki/kazanzides-chen-etal-icra-2014.pdf) - ICRA 2014, May 2014.

* Chen Z, Deguet A, Taylor RH, DiMaio SP, Fischer GS, Kazanzides P, [An Open-Source Hardware and Software Platform for Telesurgical Robotics Research](/jhu-dvrk/sawIntuitiveResearchKit/wiki/chen-deguet-etal-miccai-2013.pdf), The MIDAS Journal - Systems and Architectures for Computer Assisted Interventions 2013, June 2013.

# Acknowledgments

The cisst software has been developed with the support of the National Science Foundation, EEC 9731748, EEC 0646678, and MRI 0722943.