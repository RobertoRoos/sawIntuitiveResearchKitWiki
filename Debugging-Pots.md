<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](https://github.com/thlorenz/doctoc)*

- [Introduction](#introduction)
- [Connections](#connections)
- [Visual checks](#visual-checks)
- [Configuration files](#configuration-files)
- [Using ROS bags to collect data](#using-ros-bags-to-collect-data)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# Introduction

The dVRK uses the analog potentiometers on the robotic arms to:
  * Home the arms.  The encoders are relative encoders so we use the potentiometers to find and pre-load the zero value.  See [potentiometer calibration](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Calibration#4-potentiometers)
  * Perform safety checks.  Once the arms are powered, the dVRK software continuously monitors the joint (or actuator based on each type of arm) positions reported by both the encoders and the potentiometers.

For the safety checks, we use two different parameters per potentiometer.  Consistency is based on:
  * Distance, i.e. error tolerated in position between both (`|pots - encoders|`) readings
  * Latency, i.e. amount of time during which the distance is continuously above a given threshold
The issue is to find the "best" distance and latency for each system.

# Connections

If you're running into error messages regarding potentiometer and encoder inconsistencies, the first thing to check is the physical connections.  Potentiometers feedback is an analog signal and is sensitive to bad connections/grounding.  Check:
  * The large Cannon ITT connector at the back of the dVRK controller (see [Cannon ITT](https://www.ittcannon.com/products/dl-zif-connector/).  Make sure the connector is clean and the screw on the back is tighten (a quarter turn).
  * The two Micro DB68 SCSI cables between the dMIB and the QLA boards (see [FAQ](/jhu-dvrk/sawIntuitiveResearchKit/wiki/FAQ) for acronym definitions).

# Visual checks

# Configuration files

# Using ROS bags to collect data