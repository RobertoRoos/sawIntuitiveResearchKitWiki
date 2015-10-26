<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

- [1 Overview](#1-overview)
- [2 Low level](#2-low-level)
  - [2.1 FPGA/QLA boards](#21-fpgaqla-boards)
  - [2.2 AmpIO library](#22-ampio-library)
- [3 C++](#3-c)
  - [3.1 IO level](#31-io-level)
  - [3.2 PID controller](#32-pid-controller)
  - [3.3 Arm classes](#33-arm-classes)
  - [3.4 Tele-operation](#34-tele-operation)
  - [3.5 Console](#35-console)
- [4 Qt](#4-qt)
- [5 ROS](#5-ros)
  - [5.1 Topics](#51-topics)
  - [5.2 Python](#52-python)
  - [5.3 Matlab](#53-matlab)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# 1 Overview

The dVRK hardware and software stack is composed of:
* Firmware on FPGA/QLA interfacing IO with FireWire
* Lightweight C library on PC side to interface to FPGA via FireWire
* C++ components using the cisst/SAW libraries to implement IOs, controllers (PID, tele-operation), console, GUI, bridges to ROS, ...
* ROS wrapper around dVRK topics

# 2 Low level

## 2.1 FPGA/QLA boards
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

## 2.2 AmpIO library
C low level library:
  * Runs on the PC sides on top of Linux/libraw1394
  * Pack/unpack data to/from FPGA, i.e. convert bits to usable numbers (integers)
  * Handles multiple FPGA and treat them as single controller (
  * Simple text based programs to test hardware (`qladisp`, `qlatest`, `qlacloserelays`, ...)
  * See:
    * https://github.com/jhu-cisst/mechatronics-software/wiki
    * https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Hardware

# 3 C++

All C++ components are based on the cisst/SAW libraries, more specifically the cisstMultiTask framework:
* cisst libraries: https://github.com/jhu-cisst/cisst/wiki
* cisstMultiTask:
  * Tutorial: https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-tutorial
  * Concepts: https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-concepts

## 3.1 IO level

## 3.2 PID controller

## 3.3 Arm classes

## 3.4 Tele-operation

## 3.5 Console

# 4 Qt

# 5 ROS

## 5.1 Topics

## 5.2 Python

## 5.3 Matlab