<!--ts-->
   * [Introduction](#introduction)
   * [Setup joints](#setup-joints)
   * [Labels](#labels)
      * [Physical dimension](#physical-dimension)
      * [Labels](#labels-1)
      * [Label Placement](#label-placement)
         * [Important Notes](#important-notes)
         * [Directions for Placing the Labels](#directions-for-placing-the-labels)
            * [PSMs](#psms)
            * [ECM](#ecm)
         * [PSM1 and PSM2](#psm1-and-psm2)
         * [PSM3](#psm3)
         * [ECM](#ecm-1)
   * [Calibration](#calibration)
      * [PSM1 and PSM2](#psm1-and-psm2-1)
      * [PSM3](#psm3-1)
      * [ECM](#ecm-2)

<!-- Added by: anton, at:  -->

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

## Label Placement

### Important Notes
Every joint moves in the positive direction when turned counter clockwise and in the negative direction when turned clockwise. This is why the labels for joint 4 on the three PSMs are the only labels with negative values on the left and negative values on the right.

The label for joint 3 of the ECM is made so that 180 is the value that is aligned with the divot in the joint because the 0 position for the ECM is where the ECM is doubled back, facing the main column.

Make sure to put the labels on the link before the joint.

PSM1 and PSM2 have identical SUJs except for their second joints that have the same geometry but have different bounds of rotation (PSM1 moves from -135 to 0 and PSM 2 moves from 0 to 135). This is why the directions for their label placement is the same.

The numbering of the joints are as follows:
* Joint 0: Translation
* Joint 1:

### Directions for Placing the Labels

| ECM  | PSM1 | PSM2  | PSM3 |
| ------------- | ------------- | ------------- | ------------- |
| [ECM Labels PDF](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/ECM_Labels.pdf)  | [PSM1 Labels PDF](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/PSM_1_Labels.pdf)  | [PSM2 Labels PDF](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/PSM_2_Labels.pdf)  | [PSM3 Labels PDF](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/PSM_3_Labels.pdf)  |
| [ECM Labels SVG](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/ECM_Labels.svg)  | [PSM1 Labels SVG](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/PSM_1_Labels.svg)  | [PSM2 Labels SVG](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/PSM_2_Labels.svg)  | [PSM3 Labels SVG](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/PSM_3_Labels.svg)  |

#### PSMs
1. Open the PSM 1 and PSM 2 Labels files at the top of this section and print out the document on either letter size paper or glossy white/ clear polyester sheets (glossy white sheets are preferred).
2. Check that the box at the bottom of the document measures 20 cm.
3. Cut out the label for the joint you are marking.
4. Align the zero degree measurement with the indented line already on the joint as shown in the images below.
5. Either tape on the paper or stick on the sheets in that location (if you are using the glossy sheets, reinforce them by taping them down).

#### ECM
1. Open the ECM Labels file at the top of this section and print out the document on either letter size paper or gloss white/ clear polyester sheets.
2. Check that the box at the bottom of the document measures 20 cm.
3. Cut out the label for the joint you are marking.
4. Align the zero degree measurement with the indented line already on joints 1 and 2 as shown in the images below.
5. Align the 180 degree measurement with the indented line already on joint 3 as shown in the image below.
6. Either tape on the paper or stick on the sheets in that location.

### PSM1 and PSM2

<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM1-joint-1.jpg" width="200">

<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM1-joint-1.jpg" width="500">

![SUJ PSM1 Joint 2](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM1-joint-2.jpg)

![SUJ PSM1 Joint 3](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM1-joint-3.jpg)

![SUJ PSM1 Joint 4](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM1-joint-4.jpg)

![SUJ PSM1 Joint 5](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM1-joint-5.jpg)


### PSM3

![SUJ PSM3 Joint 1](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM3-joint-1.jpg)

![SUJ PSM3 Joint 2](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM3-joint-2.jpg)

![SUJ PSM3 Joint 3](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM3-joint-3.jpg)

![SUJ PSM3 Joint 4](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM3-joint-4.jpg)

![SUJ PSM3 Joint 5](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM3-joint-5.jpg)

### ECM

![SUJ ECM Joint 1](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-ECM-joint-1.jpg)

![SUJ ECM Joint 2](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-ECM-joint-2.jpg)

![SUJ ECM Joint 3](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-ECM-joint-3.jpg)

# Calibration

## PSM1 and PSM2
![PSM Translation Bottom](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/translation-laser-bottom.jpg)
<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/translation-laser-bottom.jpg" width="500">

![PSM Translation Top](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/translation-laser-top.jpg)

## PSM3
![Validation RCM-ECM](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/validation-RCM-ECM.jpg)
![Validation RCM-RCM](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/validation-RCM-RCM.jpg)

## ECM
![ECM Translation Bottom](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/translation-ECM-bottom.jpg)
![ECM Translation Bottom](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/translation-ECM-top.jpg)
