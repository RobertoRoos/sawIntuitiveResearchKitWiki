<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

- [Overview](#overview)
- [Low level](#low-level)
- [C++](#c)
  - [IO level](#io-level)
  - [PID controller](#pid-controller)
  - [Arm classes](#arm-classes)
  - [Tele-operation](#tele-operation)
  - [Console](#console)
- [Qt](#qt)
- [ROS](#ros)
  - [Topics](#topics)
  - [Python](#python)
  - [Matlab](#matlab)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# Overview

The dVRK hardware and software stack is composed of:
* Firmware on FPGA/QLA interfacing IO with FireWire
* Lightweight C library on PC side to interface to FPGA via FireWire
* C++ components using the cisst/SAW libraries to implement IOs, controllers (PID, tele-operation), console, GUI, bridges to ROS, ...
* ROS wrapper around dVRK topics

# Low level

* FPGA/QLA boards: the embedded firmware performs:
  * Collect data from digital inputs data (limit/home switches)
  * Control digital outputs (ON/OFF/PWM)
  * Compute encoder positions and velocities, including detecting overflow and preload
  * Perform basic safety checks on motor current (consistency between requested and measured)
  * Implement subset of FireWire protocol to communicate with a computer
  * Maintain a watchdog to make sure the PC is still connected and communicating with the controller
  * See:
    * http://jhu-cisst.github.io/mechatronics
    * https://github.com/jhu-cisst/mechatronics-firmware
* C low level library, AmpIO library:
  * Runs on the PC sides on top of Linux/libraw1394
  * Pack/unpack data to/from FPGA, i.e. convert bits to usable numbers (integers)
  * Handles multiple FPGA and treat them as single controller (
  * Simple text based programs to test hardware (`qladisp`, `qlatest`, `qlacloserelays`, ...)
  * See:
    * https://github.com/jhu-cisst/mechatronics-software/wiki
    * https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Hardware

# C++

All C++ components are based on the cisst/SAW libraries, more specifically the cisstMultiTask framework:
* cisst libraries: https://github.com/jhu-cisst/cisst/wiki
* cisstMultiTask:
  * Tutorial: https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-tutorial
  * Concepts: https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-concepts

## IO level

## PID controller

## Arm classes

## Tele-operation

## Console

# Qt

# ROS

## Topics

## Python

## Matlab