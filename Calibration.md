<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

- [1. Calibration](#1-calibration)
- [2. Current offsets](#2-current-offsets)
  - [2.1. Introduction](#21-introduction)
  - [2.2. Procedure](#22-procedure)
- [3. Gripper on MTMs](#3-gripper-on-mtms)
  - [3.1. Introduction](#31-introduction)
  - [3.2. Procedure](#32-procedure)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# 1. Calibration

The following sections assumes that you performed every step in: 
* [Generating XML configuration files](/jhu-dvrk/sawIntuitiveResearchKit/wiki/XMLConfig)
* [Hardware setup and testing](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Hardware)

The calibration steps are required to fine tune the XML configuration file generated from the `.cal` file provided by ISI.  It also requires a fully functional controller, i.e. the arm must be connected to the controller and the controller must be connected to the PC.  You must also make sure that you can power on/off the actuator amplifiers as described in the [wiki:/sawIntuitiveResearchKitTutorial/Hardware Hardware setup and testing] page.

# 2. Current offsets

## 2.1. Introduction

The FPGA/QLA controllers specify the current between an upper and a lower bound represented by an integer between 0 and 65,535.  The actual values requested by the users are specified in amperes.  The conversion factors (scales and offsets per actuator) are specified in the XML config file.   The default offset is 65,535 divided by two, i.e. 32,768.

  ```xml
  <Drive>
    <AmpsToBits Offset="32768" Scale="5242.88"/>
    ...
  </Drive>
  ```

Unfortunately the requested current is not perfect and there is an offset caused by the overall system, i.e. the controller and the robot itself.  The goal of the calibration procedure is to:
* request a null current on all actuators knowing that it won't be totally null
* measure the current feedback
* update the requested current offsets based on the difference between requested and measured current
* save the new offsets in the XML configuration file

## 2.2. Procedure

The program to calibrate the requested current is called `sawRobotIO1394CurrentCalibration`.  To use it you will need an existing XML configuration file.  The command line options are:

  ```bash
  sawRobotIO1394CurrentCalibration:
   -c <value>, --config <value> : configuration file (required)
   -p <value>, --port <value> : firewire port number(s) (optional)
  ```

For most users, the default firewire port (0) should work so you should be able to run the program using something like (but with your combination of MTML/MTMR and serial numbers):

  ```bash
  sawRobotIO1394CurrentCalibration -c sawRobotIO1394-MTML-12345.xml
  ````

The program takes only a few seconds to run and the expected output is something like:

  ```
  Configuration file: sawRobotIO1394-MTML-22723.xml
  Port: 0
  Make sure:
   - your computer is connected to the firewire controller.
   - the arm corresponding to the configuration file "sawRobotIO1394-MTML-22723.xml" is connected to the controller.
   - the E-Stop is closed, i.e. will let the controller power on.
   - you have no other device connected to the firewire chain.
   - you have no other program trying to communicate with the controller.

  Press any key to get started.
  Loading config file ...
  Creating robot ...
  Creating port ...
  FirewirePort: number of ports = 1
  FirewirePort: successfully initialized port 0
  ScanNodes: base node id = ffc0
  ScanNodes: building node map for 9 nodes:
    Node 0, BoardId = 6, Firmware Version = 3
    Node 1, BoardId = 7, Firmware Version = 3
    Node 2, BoardId = 8, Firmware Version = 3
    Node 3, BoardId = 9, Firmware Version = 3
    Node 4, BoardId = 0, Firmware Version = 3
    Node 5, BoardId = 1, Firmware Version = 3
    Node 6, BoardId = 3, Firmware Version = 3
    Node 7, BoardId = 2, Firmware Version = 3

  Ready to power?  Press any key to start.
  Enabling power ...
  Status: power seems fine.
  Starting calibration ...
  Status: average current feedback in mA:      8.35252     -33.1456      11.4888     -17.2173      10.3679     -16.9912      26.5735      8.29041
  Status: standard deviation in mA:            1.12377      1.11879      1.23513      1.13384      1.21539      1.18651      1.11418      1.17129
  Status: kept 2458 samples out of 50000
  Status: new average in mA:                   8.39934     -33.1659      11.6538     -17.1952      10.3920     -16.9736      26.6081      8.33897

  Do you want to update the config file with these values? [Y/y]
  Status: current offsets in XML configuration file:      32768.0      32768.0      32768.0      32768.0      32768.0      32768.0      32768.0      32768.0
  Status: new current offsets:                            32812.0      32941.9      32706.9      32858.2      32822.5      32857.0      32907.5      32724.3

  Do you want to save these values? [S/s]
  Status: new configuration file is "sawRobotIO1394-MTML-22723.xml-new"
  ```

Steps:
* Before you start the calibration, you might want to make a copy of your existing XML configuration file.
* If the program fails and displays endless `WriteAllBoards: handle for port 0 is NULL`, hit ctrl-c to stop it.   Then test with `qladisp` to make sure your firewire connection is good.
* If the program fails to power the controllers, make sure you can power the controllers using the utility `qladisp`.
* If you are calibrating an MTM, please keep in mind that the last actuator (8) is not powered so you can ignore the last column.
* When prompted to save the new offsets, press [y] if the values for `new average in mA` make sense.  These values shouldn't exceed more than a few tens of mA.  If you have significantly higher values, DO NOT PROCEED and report your results to the research kit google group.
* You will get one more chance to review the new offsets and compare to the existing values in the original XML configuration file.  You can now save by pressing [s].
* A new XML configuration file should have been generated in the same directory as the original configuration file.  We strongly suggest you review the differences.  You can for example use the graphical utility `meld <original-file> <original-file>-new`.
* Finally, it is recommended test the new offsets by re-running the calibration utility.  Call `sawRobotIO1394CurrentCalibration` on the '''new''' configuration file.  At the point, the offsets in mA should be close to 0 (few tenths of mA) and the offsets in the XML file should be close to 0:

  ```
  Status: new average in mA:                  0.100924   -0.0292067    -0.230016   -0.0497301     0.135562   -0.0820793     0.131824      8.34862
  Status: current offsets in XML configuration file:      32811.0      32941.0      32709.0      32859.0      32823.0      32858.0      32907.0      32634.0
  Status: new current offsets:                            32811.5      32941.2      32710.2      32859.3      32823.7      32858.4      32907.7      32590.2
  ```
* Once the calibration is finished, make sure you copy your new XML configuration file to your configuration directory and rename it if needed.

# 3. Gripper on MTMs

## 3.1. Introduction

The grippers on the MTM use a Hall Effect sensor to measure the opening.  The analog values are converted to an approximative angle using a linear equation.  The goal of the calibration procedure is to define the scale and offset used for the conversion.  The da Vinci MTM gripper used two small springs, one weak outer spring to keep the gripper opened and a stronger inner spring (but shorter one) used to create some resistance when the gripper is almost closed.  From the user point of view, we have 3 different repeatable positions:
* gripper fully opened, this happens when the gripper is left alone.  The position is defined by the mechanical limit.
* gripper closed but not tight.  When the user closes the gripper with very little force, he or she should feel the moment where the stronger spring.
* gripper closed and tight.  This position is defined by the mechanical limit when maximum force is applied to close the gripper.

Ideally the slave tool should close when the master gripper is close but not tight.   Closing the master gripper more should:
* apply more force on the slave side to reduce the risk of slippage
* give a feeling of force feedback on the master side using the strong inner spring

## 3.2. Procedure

For the calibration procedure, we use the fully opened position and the closed-but-not-tight position.  The user should practice with the gripper (the robot doesn't have to be turned on) before starting the calibration to learn how strong the two springs are and repeatedly find the closed-but-not-tight position.  

The gripper calibration program used the same parameters as the current calibration program.  A typical run looks like:

   ```
  > sawIntuitiveResearchGripperCalibration -c sawRobotIO1394-MTML-22723.xml 
  Configuration file: sawRobotIO1394-MTML-22723.xml
  Port: 0
  Make sure:
   - your computer is connected to the firewire controller.
   - the MTM arm corresponding to the configuration file "sawRobotIO1394-MTML-22723.xml" is connected to the controller.
   - the E-Stop is opened, this program doesn't require powered actuators.
   - you have no other device connected to the firewire chain.
   - you have no other program trying to communicate with the controller.

  Press any key to start.
  Loading config file ...
  Creating robot ...
  Creating port ...
  FirewirePort: number of ports = 1
  FirewirePort: successfully initialized port 0
  ScanNodes: base node id = ffc0
  ScanNodes: building node map for 9 nodes:
    Node 0, BoardId = 6, Firmware Version = 3
    Node 1, BoardId = 7, Firmware Version = 3
    Node 2, BoardId = 8, Firmware Version = 3
    Node 3, BoardId = 9, Firmware Version = 3
    Node 4, BoardId = 0, Firmware Version = 3
    Node 5, BoardId = 1, Firmware Version = 3
    Node 6, BoardId = 3, Firmware Version = 3
    Node 7, BoardId = 2, Firmware Version = 3

  Press any key to start collecting data.
  Fully open and close the gripper up to the second spring on the MTM multiple times.
  NOTE: It is very important to not close the gripper all the way, stop when you feel some resitance from the second spring.
  + indicates a new maximum, - indicates a new minimum.
  Press any key to stop collecting data.
  +-+++++----++++++++++-----------------------------------------------------------------------------------------------------------
  --------------------------------------------------------------------------------------------------------------------------------
  ----------------------------------------------------------------------------------------------------
  Status: found range [64.5983, 89.7076] degrees using 12210 samples.

  Do you want to update the config file with these values? [Y/y]
  Enter the new desired max for the gripper, 60 (degrees) is recommended to match the maximum tool opening.
  60
  Status: offset and scale in XML configuration file: 263.576 -64.5157
  Status: new offset and scale:                       475.469 -154.164

  Do you want to save these values? [S/s]
  Status: new config file is "sawRobotIO1394-MTML-22723.xml-new"
  ```

Notes:
* Once the data collection is started, you will see some '-' and '+' signs appear.  A new '+' sign appears when we find a new upper value and a '-' appears when a new lower value is found.
* Make sure you don't close the gripper past the closed-but-not-tight position.   If you do, stop the data collection, do not save the results and restart the program.
* As for the current calibration, don't forget to rename the '-new' file to replace the original configuration file.
* To verify your calibration file, re-run the program and it should display a found range from 0 to the maximum value you provided (suggested is 60).
* Since the closed-but-not-tight position depends on the user, it is not perfectly repeatable.  You will need some trials and errors to fine tune the calibration.