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

## Purpose

The SUJs are passive arms with electric brakes.  They are used to position the patient side arms remote centers of motion (RCM) on the patient.  Once the system is setup for a surgery, the SUJs are not supposed to be moved until the end of the surgery.  Depending on the daVinci, there are between 3 and 4 SUJs mounted on the patient cart.  All systems have an ECM SUJ mounted on the front of the support column.  They also have 2 SUJs, one for the PSM1 (mounted on the right) and one for the PSM2 (mounted on the left).  For systems with a third PSM, the PSM3 is mounted on the front of the center column, under the ECM SUJ.

## Original hardware

The Setup Joints use potentiometers to read the joint positions.   Each joint has two potentiometers and each arm has up to 6 joints.  So we have a total of 48 potentiometers, each provides a voltage that needs to be converted to an angle (revolute joints) or distance (prismatic joints).

The brakes are controlled per arm, not per joint.  Finally, some patient carts come with a third PSM (aka PSM3).  The PSM3 SUJ is mounted under the ECM SUJ and doesn't have any counter weights.  It can be lifted using a separate motor. 

## dVRK usage

### dVRK SUJ controller

**This controller is in final phase of testing and close to production (as of December 2019)**

The controller support all the features available on the daVinci patient cart, i.e.:
* Read joint positions. The dVRK QLA has 4 analog to digital inputs so it reads the potentiometer values sequentially using a multiplexer.
* Release brakes.  The dVRK controller uses the linear amps of the QLA dedicated to motor control to release the brakes. 
* Lift PSM3.  The dVRK FPGA generates a PWM signal sent to the PWM power unit included on the dSIB.

### dVRK software

The dVRK software stack provides all the functions and parameters needed to compute the forward kinematic of the SUJ arms based on the joint values.  Furthermore, it maintains the will maintain the transformation tree between all the kinematics chain so that the PSM1 tip will be defined wrt the ECM tip (including active ECM motion).

Physical buttons are also used by the software to release the SUJ brakes and the SUJ PSM3 lift.  The following "buttons" are supported:
* Physical button on SUJ arms (black handle).   The only SUJ arms with this button are the SUJ PSM1 and SUJ PSM2.
* Physical button on the actual arm (i.e. ECM or PSM).  On the ECM, black handle on top of parallel link.  On the PSMs, white button on the side of the parallel link.  **Note:** the software can only handle these buttons if the actual arm controller (i.e. ECM, PSM1, PSM2 and/or PSM3) is used by the dVRK software.
* Software "clutch" button for all SUJ.  The button is found in the Qt Widget for the SUJ arm.  You have to press continuously on the button to release the brake.  This logic prevents users from accidentally keep the brakes released. 
* Physical lift toggle button on PSM3.  This button is usually attached on the parallel link of the PSM3 using a strap.
* Software "lift" buttons for SUJ PSM3.

### dVRK simulation mode

All dVRK arm can be used in simulation mode, this includes the SUJs.  There are two main applications for this simulation with the SUJs:
* Simulate the whole patient cart.  At that point all PSMs and the ECM are also simulated.
* Simulate only the SUJs.  This can be used if you don't have access to the dVRK SUJ controller.  With the simulation mode, you have to find and manually enter the SUJs joint values.  Once this is done, the software can compute the forward kinematic and provide all the base frames you need for the PSM and ECM teleoperation.
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

The label for joint 3 of the ECM is made so that 180 is the value that is aligned with the divot in the joint because the 0 position for the ECM is where the ECM is facing the support column.

Make sure to put the labels on the link before the joint.

PSM1 and PSM2 have identical SUJs except for their second joints that have the same geometry but have different bounds of rotation (PSM1 moves from -135 to 0 and PSM 2 moves from 0 to 135). This is why the directions for their label placement is the same.

The numbering of the joints are best understood if the arm is fully extended. Joint 0 is the translational movement. The rest of the joints are numbered increasingly as you move out from the main column. So joint 1 is the closest joint to the column and joint is the second closest joint. Joint 4 is the joint that rotates the carriage in the horizontal plane.

### Directions for Placing the Labels

| ECM  | PSM1 | PSM2  | PSM3 |
| ------------- | ------------- | ------------- | ------------- |
| [ECM Labels PDF](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/ECM_Labels.pdf)  | [PSM1 Labels PDF](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/PSM_1_Labels.pdf)  | [PSM2 Labels PDF](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/PSM_2_Labels.pdf)  | [PSM3 Labels PDF](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/PSM_3_Labels.pdf)  |
| [ECM Labels SVG](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/ECM_Labels.svg)  | [PSM1 Labels SVG](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/PSM_1_Labels.svg)  | [PSM2 Labels SVG](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/PSM_2_Labels.svg)  | [PSM3 Labels SVG](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/PSM_3_Labels.svg)  |

#### PSMs
1. Open the PSM1 and PSM2 Labels files at the top of this section and print out the document on either letter size paper or glossy white/ clear polyester sheets (glossy white sheets are preferred).
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

<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM1-joint-1.jpg" width="350"> 
<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM1-joint-2.jpg" width="350">
<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM1-joint-3.jpg" width="350">
<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM1-joint-4.jpg" width="350">
<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM1-joint-5.jpg" width="350">

### PSM3

<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM3-joint-1.jpg" width="350">
<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM3-joint-2.jpg" width="350">
<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM3-joint-3.jpg" width="350">
<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM3-joint-4.jpg" width="350">
<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-PSM3-joint-5.jpg" width="350">

### ECM

<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-ECM-joint-1.jpg" width="350">
<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-ECM-joint-2.jpg" width="350">
<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/SUJ-ECM-joint-3.jpg" width="350">

# Calibration

## PSM1 and PSM2

The dVRK graphical table has two rows and six columns. The first column is for joint 0, second for joint 1, third for joint 2, and so on.

### Joint 0
To calibrate joint 0 it is necessary to have a laser measuring tool. The procedure for calibration is as follows:
1. Lower the PSM to its lowest point.
2. Place the measuring tool underneath joint 1 as shown in the picture below.
3. Find and record the height of the PSM.
4. Enter 0 in the graphical table in the first column of the first row.
5. Raise the PSM to its highest point.
6. Find and record the height of the PSM.
7. Subtract the final height from the initial height.
8. Enter the difference in height in the second column of the first row. 

### Joints 1-5
The procedure for calibration is as follows:
1. Turn the joint to an extreme (either its highest or lowest degree measurement on the label).
2. Enter that angle into the graphical table in the first row and in the column that corresponds with the joint.
3. Turn the joint to the other extreme.
4. Enter that angle into the graphical table in the second row and in the column that corresponds with the joint.
5. Repeat until all 5 joints have values in the top and bottom row.

## PSM3

### Joint 0
To calibrate joint 0 it is necessary to have a laser measuring tool. The procedure for calibration is as follows:
1. Lower the PSM to its lowest point.
2. Place the measuring tool underneath joint 1 as shown in the picture below of PSM1 (all 3 PSMs have similar flat pieces under their joints).
3. Find and record the height of the PSM.
4. Enter 0 in the graphical table in the first column of the first row.
5. Raise the PSM to its highest point.
6. Find and record the height of the PSM.
7. Subtract the final height from the initial height.
8. Enter the difference in height in the second column of the first row.
9. Anton, we need to add something here about adjusting the offset.

### Joints 1-5
The procedure for calibration is as follows:
1. Turn the joint to an extreme (either its highest or lowest degree measurement on the label).
2. Enter that angle into the graphical table in the first row and in the column that corresponds with the joint.
3. Turn the joint to the other extreme.
4. Enter that angle into the graphical table in the second row and in the column that corresponds with the joint.
5. Repeat until all 5 joints have values in the top and bottom row.

## ECM
**The ECM can only be calibrated after calibrating PSM1 or PSM2**

### Joint 0
The procedure for calibration is as follows:
1. Lower the ECM to its lowest point.
2. Lower PSM1 to the point at which the bottom of PSM1's second joint and the ECM's second joint are level as seen below.
3. Record the reported height for PSM1 from the graphical as the initial height of the ECM.
4. Enter 0 in the graphical table in the first column of the first row.
5. Raise the ECM to its highest point.
6. Raise PSM1 to the point at which the bottom of PSM1's second joint and the ECM's second joint are level as seen below.
7. Subtract the final height from the initial height.
8. Enter the difference in height in the second column of the first row.

### Joints 1-3
The procedure for calibration is as follows:
1. Turn the joint to an extreme (either its highest or lowest degree measurement on the label).
2. Enter that angle into the graphical table in the first row and in the column that corresponds with the joint.
3. Turn the joint to the other extreme.
4. Enter that angle into the graphical table in the second row and in the column that corresponds with the joint.
5. Repeat until all 3 joints have values in the top and bottom row.
6. Fill the last two columns in with random numbers as place holders.

**When the table is full of values hit "Manual Calibration" and the correct offsets and scales will print in the terminal.**

### Calibration Check
When the calibration is complete, place all the RCMs together and check that their reported values on the graphical are within a 20 mm cube of each other. (This is show in the pictures labeled PSM3)

<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/translation-laser-bottom.jpg" width="350">
<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/translation-laser-top.jpg" width="350">

## PSM3

<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/validation-RCM-ECM.jpg" width="350">
<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/validation-RCM-RCM.jpg" width="350">

## ECM

<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/translation-ECM-bottom.jpg" width="350">
<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/translation-ECM-top.jpg" width="350">

