<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

- [Introduction](#introduction)
- [Credit / Citation](#credit--citation)
- [Links](#links)
  - [General](#general)
  - [Software](#software)
- [Updates](#updates)
  - [Firmware](#firmware)
  - [Software](#software-1)
- [Acknowledgments](#acknowledgments)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# Introduction

The da Vinci Research Kit is an “open-source mechatronics” system, consisting of electronics, firmware, and software that is being used to control research systems based on the first-generation da Vinci system.

![Controller with Patient Side Manipulators (PSMs)](/jhu-dvrk/sawIntuitiveResearchKit/wiki/ControllerWithPSM.jpg)

The sawIntuitiveResearchKit folder provides several example applications for controlling the Research Kit for the da Vinci System using the [IEEE-1394 (FireWire) controller](http://jhu-cisst.github.io/mechatronics/). The picture above shows two Controllers connected to two da Vinci Patient Side Manipulators (PSMs) at Worcester Polytechnic Institute (WPI).

# Credit / Citation

If you use the dVRK in your research, please cite the following paper:

  P. Kazanzides, Z. Chen, A. Deguet, G. S. Fischer, R. H. Taylor, and S. P. DiMaio, “[An open-source research kit for the da Vinci(R) surgical system](/jhu-dvrk/sawIntuitiveResearchKit/wiki/kazanzides-chen-etal-icra-2014.pdf),” in IEEE Intl. Conf. on Robotics and Automation (ICRA), 2014, pp. 6434–6439. [BibTeX](/jhu-dvrk/sawIntuitiveResearchKit/wiki/kazanzides-chen-etal-icra-2014)

For posters and videos, please include the [dVRK logo](https://github.com/jhu-dvrk/dvrk-logo) if possible.

# Links

## General

* [Groups](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Timeline), [Videos](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Videos) and [publications](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Publications)
* Google group https://groups.google.com/d/forum/research-kit-for-davinci and research-kit-for-davinci@googlegroups.com
* Intuitive Surgical hardware wiki http://research.intusurg.com/dvrk
* *cisst* libraries http://github.com/jhu-cisst/cisst/wiki
* Johns Hopkins University Mechatronics http://jhu-cisst.github.io/mechatronics
* List of all JHU LCSR Software http://jhu-lcsr.github.io/software/

## Software

The software applications use some or all of the following SAW components (and Qt widgets):
* [mtsRobotIO1394](https://github.com/jhu-saw/sawRobotIO1394) - interface to IEEE-1394 (FireWire) controller boards
* [mtsPID](https://github.com/jhu-saw/sawControllers) - PID controller used for MTM and PSM robots
* [mtsTeleoperation](https://github.com/jhu-saw/sawControllers) - Teleoperation component
* [mtsTextToSpeech](https://github.com/jhu-saw/sawTextToSpeech) - Text to speech component (for warning and error messages)

The components are cross-platform, except for mtsRobotIO1394, which relies on a low-level IEEE-1394 interface library (`libraw1394`) that is primarily available on Linux. Thus, the build instructions focus on Linux. For setting up the FireWire interface on Linux, see [this page](/jhu-cisst/mechatronics-software/wiki/Development-Environment).

A ROS interface is available via [mtsROSBridge](https://github.com/jhu-cisst/cisst-ros) base class and [dVRK programs and files](https://github.com/jhu-dvrk/dvrk-ros).

# Updates

## Firmware

Firmware version 3.0, 4.0, or 5.0 is now required, please upgrade.  Version 4.0 adds support for FireWire broadcasts, which enables faster I/O performance.
https://github.com/jhu-cisst/mechatronics-firmware/wiki/FPGA-Program 

## Software

* *August 2016*: Version 1.4.0 released, see latest change logs:
  * [sawIntuitiveResearchKit](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/CHANGELOG.md)
  * [dvrk-ros](https://github.com/jhu-dvrk/dvrk-ros/blob/master/CHANGELOG.md)
  * [cisst-ros](https://github.com/jhu-cisst/cisst-ros/blob/master/CHANGELOG.md)
  * [sawRobotIO1394](https://github.com/jhu-saw/sawRobotIO1394/blob/master/CHANGELOG.md)
* *January 2016*: Version 1.3.0 released
* *October 2015*: Version 1.2.0 released
* *April 2015*: Version 1.1.0 released
* *April 2014*: Moved to GitHub
* *May 2013*: Initial Public Release

# Acknowledgments

The cisst software has been developed with the support of the National Science Foundation, EEC 9731748, EEC 0646678, and MRI 0722943.

The da Vinci Research Kit is supported by the National Science Foundation, via the National Robotics Initiative (NRI), as part of the collaborative research project "Software Framework for Research in Semi-Autonomous Teleoperation" between The Johns Hopkins University (IIS 1637789), Worcester Polytechnic Institute (IIS 1637759), and the University of Washington (IIS 1637444).