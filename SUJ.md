<!--ts-->
   * [Introduction](#introduction)
   * [Setup joints](#setup-joints)

<!-- Added by: adeguet1, at: 2019-08-06T12:14-04:00 -->

<!--te-->

# Introduction

Acronyms used in this page are defined in [Frequently Asked Questions](/jhu-dvrk/sawIntuitiveResearchKit/wiki/FAQ).

This page describes how to use the Setup Joints (SUJs) with the dVRK.  This is useful only for the groups with a [full da Vinci](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Full-da-Vinci.md)

The Setup Joints use potentiometers to read the joint positions.   Each joint has two potentiometers and each arm has up to 6 joints.  So we have a total of 48 potentiometers, each provides a voltage that needs to be converted to an angle (revolute joints) or distance (prismatic joints).  The dVRK controller has 4 analog to digital inputs so it reads the potentiometer values sequentially using a multiplexer.  Cycling through the potentiometers takes time so you need to make sure you let the readings stabilize before doing anything.

The brakes are controlled per arm, not per joint.  The dVRK controller uses the linear amps dedicated to motor control to release the brakes.  Finally, some patient carts come with a third PSM (aka PSM3).  The PSM3 SUJ is mounted under the ECM SUJ and doesn't have any counter weights.  It can be lifted using a separate motor controlled by a PWM unit included on the dSIB.

To use the SUJ with the dVRK controller and software, the main challenge is to calibrate the potentiometers.  To do so, we use some custom labels that can be attached to each joint of the SUJs.

# Setup joints

![SUJ arm widget](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-gui-arm-suj.png)

**TBD**  Hardware is not yet available for setup joints.

# Labels

## Physical dimension

Joints diameters:
* 0: Translation, where's the origin?
* 1: Rotation, vertical axis, diameter 147.5 mm, circumference 463.4 mm (all -90/+90)  
* 2: Rotation, vertical axis, diameter 133.5 mm, circumference 419.4 mm (PSM1 -135/0, PSM2 0/+135, PSM3/ECM -90/+90) 
* 3: Rotation, vertical axis, diameter 107.5 mm, circumference 337.7 mm (all -90/+90)
* 4: Rotation, horizontal axis, diameter 93.3 mm, circumference 293.1 mm (PSMs only -135/+135)
* 5: Rotation, "vertical" axis, diameter 89.4 mm, circumference 280.9 mm (PSMs only -135/+135)

## Labels

You can download a document to print on letter paper: [dVRK labels for SUJ](/jhu-dvrk/sawIntuitiveResearchKit/wiki/suj-labels.svg).  The `.svg` file can be opened, modified and printed using `inkscape` on Linux.  You can also find a [pdf version](/jhu-dvrk/sawIntuitiveResearchKit/wiki/suj-labels.pdf) if you don't have `inkscape`.  Please, if you update the `.svg` file, make sure you also update the `.pdf`.

## Label placement

### PSM1 and PSM2

![SUJ PSM1 Joint 1](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-ECM-joint-1.jpg)

![SUJ PSM1 Joint 2](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-ECM-joint-2.jpg)
### PSM3

### ECM


# Calibration

## PSM1 and PSM2


## PSM3

## ECM
