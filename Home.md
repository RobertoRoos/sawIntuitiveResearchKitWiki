<!--ts-->
   * [Introduction](#introduction)
   * [Credit / Citation](#credit--citation)
   * [Links](#links)
      * [Community](#community)
      * [Software](#software)
   * [Updates](#updates)
      * [Firmware](#firmware)
      * [Software](#software-1)
   * [Acknowledgments](#acknowledgments)

<!-- Added by: anton, at:  -->

<!--te-->

# Introduction

The da Vinci Research Kit (**dVRK**) is an “open-source mechatronics” system, consisting of electronics, firmware, and software that is being used to control research systems based on the now retired first-generation da Vinci system from Intuitive Surgical Inc.

![Controller with Patient Side Manipulators (PSMs)](/jhu-dvrk/sawIntuitiveResearchKit/wiki/ControllerWithPSM.jpg)

The sawIntuitiveResearchKit folder provides several example applications for controlling the Research Kit for the da Vinci System using the [IEEE-1394 (FireWire) controller](http://jhu-cisst.github.io/mechatronics/). The picture above shows two Controllers connected to two da Vinci Patient Side Manipulators (PSMs) at Worcester Polytechnic Institute (WPI).

# Credit / Citation

If you use the dVRK in your research, please cite the following paper:

  P. Kazanzides, Z. Chen, A. Deguet, G. S. Fischer, R. H. Taylor, and S. P. DiMaio, “[An open-source research kit for the da Vinci(R) surgical system](/jhu-dvrk/sawIntuitiveResearchKit/wiki/kazanzides-chen-etal-icra-2014.pdf),” in IEEE Intl. Conf. on Robotics and Automation (ICRA), 2014, pp. 6434–6439. [BibTeX](/jhu-dvrk/sawIntuitiveResearchKit/wiki/kazanzides-chen-etal-icra-2014)

For posters and videos, please include the [dVRK logo](https://github.com/jhu-dvrk/dvrk-logo) if possible.

# Links

## Community

* [News](/jhu-dvrk/sawIntuitiveResearchKit/wiki/News) from the dVRK community
* [Groups](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Timeline) and deployment timeline
* [Videos](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Videos) of the dVRK in action
* [Publications](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Publications)
* Resources:
  * Google group https://groups.google.com/d/forum/research-kit-for-davinci and research-kit-for-davinci@googlegroups.com (**for dVRK users only**; use the Google group web page to request membership, don't forget to mention your group/university so the group admin can identify you).
  * Intuitive Surgical hardware wiki http://research.intusurg.com/dvrk
  * Intuitive Foundation http://www.intuitive-foundation.org/dvrk/ (includes online applications to get retired da Vinci components from Intuitive Surgical)
  * *cisst* libraries http://github.com/jhu-cisst/cisst/wiki
  * Johns Hopkins University Mechatronics http://jhu-cisst.github.io/mechatronics
  * List of all JHU LCSR Software http://jhu-lcsr.github.io/software/

## Software

The software applications use some or all of the following SAW components (and Qt widgets):
* [mtsRobotIO1394](https://github.com/jhu-saw/sawRobotIO1394) - interface to IEEE-1394 (FireWire) controller boards
* [mtsPID](https://github.com/jhu-saw/sawControllers) - PID controller used for MTM and PSM robots
* [mtsTeleoperation](https://github.com/jhu-saw/sawIntuitiveResearchKit) - Teleoperation components
* [mtsTextToSpeech](https://github.com/jhu-saw/sawTextToSpeech) - Text to speech component (for warning and error messages)

The components are cross-platform, except for mtsRobotIO1394, which relies on a low-level IEEE-1394 interface library (`libraw1394`) that is primarily available on Linux. Thus, the build instructions focus on Linux. For setting up the FireWire interface on Linux, see [this page](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Development-Environment).

A ROS interface is available via [mtsROSBridge](https://github.com/jhu-cisst/cisst-ros) base class and [dVRK ROS programs and files](https://github.com/jhu-dvrk/dvrk-ros).

# Updates

## Firmware

Firmware version 4.0, 5.0 or 6.0 is now required (dVRK 1.6), please upgrade your firmware to Rev6.  Version 6.0 adds support for FPGA based velocity estimation. See
https://github.com/jhu-cisst/mechatronics-firmware/wiki/FPGA-Program for step-by-step instructions to upgrade your firmware.

## Software

* *July 2019*: Version 1.7.1 released:
  * Fixed error in MTM gravity compensation code
* *April 2019*: Version 1.7.0 released:
  * Using c++ 14 features: **Ubuntu 16.04 or higher required**
  * [sawIntuitiveResearchKit](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/CHANGELOG.md) (main dVRK code)
  * [dvrk-ros](https://github.com/jhu-dvrk/dvrk-ros/blob/master/CHANGELOG.md) (ROS specific dVRK code)
  * [cisst](https://github.com/jhu-cisst/cisst/blob/master/CHANGELOG.md) (core libraries)
  * [cisst-ros](https://github.com/jhu-cisst/cisst-ros/blob/master/CHANGELOG.md) (ROS bridge for cisst libraries)
  * [sawRobotIO1394](https://github.com/jhu-saw/sawRobotIO1394/blob/master/CHANGELOG.md) (FireWire IO component)
  * [sawControllers](https://github.com/jhu-saw/sawControllers/blob/master/CHANGELOG.md) (PID component)
* *May 2018*: Version 1.6.0 released
* *November 2017*: Version 1.5.0 released
* *August 2016*: Version 1.4.0 released
* *January 2016*: Version 1.3.0 released
* *October 2015*: Version 1.2.0 released
* *April 2015*: Version 1.1.0 released
* *April 2014*: Moved to GitHub
* *May 2013*: Initial Public Release

# Acknowledgments

The cisst software has been developed with the support of the National Science Foundation, EEC 9731748, EEC 0646678, and MRI 0722943.

The da Vinci Research Kit is supported by the National Science Foundation, via the National Robotics Initiative (NRI), as part of the collaborative research project "Software Framework for Research in Semi-Autonomous Teleoperation" between The Johns Hopkins University (IIS 1637789), Worcester Polytechnic Institute (IIS 1637759), and the University of Washington (IIS 1637444).
