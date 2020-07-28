<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

- [1. Calibration](#1-calibration)
- [2. Current offsets](#2-current-offsets)
  - [2.0 Prerequisite](#20-prerequisite)
  - [2.1. Introduction](#21-introduction)
  - [2.2. Procedure](#22-procedure)
- [3. Gripper on MTMs](#3-gripper-on-mtms)
  - [3.1. Introduction](#31-introduction)
  - [3.2. Procedure](#32-procedure)
- [4. Potentiometers](#4-potentiometers)
  - [4.1. Introduction](#41-introduction)
  - [4.2. Requirements](#42-requirements)
  - [4.3. Calibrating scales](#43-calibrating-scales)
  - [4.3. Calibrating offsets](#43-calibrating-offsets)
  - [4.4. Errors during the potentiometer calibration](#44-errors-during-the-potentiometer-calibration)


<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# 1. Calibration

The following sections assumes that you performed every step in: 
* [Generating XML configuration files](/jhu-dvrk/sawIntuitiveResearchKit200 Avenue de la Vieille Tour

33405 Talence/wiki/XMLConfig)
* [Hardware setup and testing](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Hardware)

The calibration steps are required to fine tune the XML configuration file generated from the `.cal` file provided by ISI.  It also requires a fully functional controller, i.e. the arm must be connected to the controller and the controller must be connected to the PC.  You must also make sure that you can power on/off the actuator amplifiers as described in the [hardware setup and testing](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Hardware#3-motor-power) page.

# 2. Current offsets

## 2.0 Prerequisite
Please read the [ESTOP debugging](/jhu-dvrk/sawIntuitiveResearchKit/wiki/ESTOP#8-debugging) section before you start.   
When you are calibrating motor current, it might be simpler to have a single controller on the ESTOP chain.   If you prefer to keep all your controllers on the ESTOP chain, make sure all the controllers are also connected to the FireWire chain and you will have to use the `qlacloserelays` utility program to enable power.

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
   -b, --brakes : calibrate current feedback on brakes instead of actuators (optional)
  ```
The `--brakes` option is used to calibrate the brakes on the ECM arm only.  **For the ECM, the procedure needs to be executed twice, once for the brakes (with `-b`) and once for the actuators (without `-b`)**.

For most users, the default firewire port (0) should work so you should be able to run the program using something like (but with your combination of MTML/MTMR and serial numbers):

  ```bash
  sawRobotIO1394CurrentCalibration -c sawRobotIO1394-MTML-00000.xml
  ````

The program takes only a few seconds to run and the expected output is something like:

  ```
  Configuration file: sawRobotIO1394-MTML-00000.xml
  Port: 0
  Make sure:
   - your computer is connected to the firewire controller.
   - the arm corresponding to the configuration file "sawRobotIO1394-MTML-00000.xml" is connected to the controller.
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
  Status: new configuration file is "sawRobotIO1394-MTML-00000.xml-new"
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
  > sawIntuitiveResearchKitGripperCalibration -c sawRobotIO1394-MTML-00000.xml 
  Configuration file: sawRobotIO1394-MTML-00000.xml
  Port: 0
  Make sure:
   - your computer is connected to the firewire controller.
   - the MTM arm corresponding to the configuration file "sawRobotIO1394-MTML-00000.xml" is connected to the controller.
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
  Status: new config file is "sawRobotIO1394-MTML-00000.xml-new"
  ```

Notes:
* Once the data collection is started, you will see some '-' and '+' signs appear.  A new '+' sign appears when we find a new upper value and a '-' appears when a new lower value is found.
* Make sure you don't close the gripper past the closed-but-not-tight position.   If you do, stop the data collection, do not save the results and restart the program.
* As for the current calibration, don't forget to rename the '-new' file to replace the original configuration file.
* To verify your calibration file, re-run the program and it should display a found range from 0 to the maximum value you provided (suggested is 60).
* Since the closed-but-not-tight position depends on the user, it is not perfectly repeatable.  You will need some trials and errors to fine tune the calibration.


# 4. Potentiometers

## 4.1. Introduction

The potentiometers on the dVRK are used for:
* Homing, i.e. they provide an absolute reference to define the zero position
* Safety, i.e. by reading both encoders and potentiometers continuously one can detect discrepencies

The potentiometer values are read as voltages and converted to SI positions (radians for revolute joints and meters for prismatic joints).  The conversion is a linear function based on an offset and a scale, i.e. `position = offset + scale * voltage`.  Intuitive Surgical performed an initial calibration for all arms and can provide these values in a `.cal` file.  Using these `.cal` file and the dVRK config generator, we get the `sawRobotIO1394-00000.xml` files used for the dVRK.  See [config generator](/jhu-dvrk/sawIntuitiveResearchKit/wiki/XMLConfig).

The problem is that these values are partially based on the electronics used during the calibration.  As such, they are a bit off.   We developed two different strategies to calibrate the scales and offsets.
 * For the scales, the simplest solution is to rely on the encoders.  We generate a large motion on each actuator and collect both the encoder and potentiometer values. 
 * For the offsets, it is a bit more challenging since we need to identify a zero position based on mechanical properties.

## 4.2. Requirements

**For the ECM**, make sure the brakes are properly calibrated.   This requires to calibrate both the controller current (see above) and the power to release the brakes, see [Full da Vinci system](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Full-da-Vinci).

As for the other calibration steps, you need to have all the configuration files generated, the C++ code compiled and the current calibration performed.  Furthermore, the current implementation requires the ROS bridges and Python.  Make sure you compiled your dVRK software stack using `catkin build`.  See [build with ROS](/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild).

For the offsets, we need a physical mechanism to maintain the arm in zero position (or any known position).  We currently have a fairly easy solution for the last 4 joints of the PSM.  The four metal bars/gears are in zero positions when aligned.  We tried different methods and got similar results so you should use whatever is the most convenient for you:
 * Calibration template made of plexiglass plate with holes for the pins on the 4 wheels.
  ![Plexiglass plate to calibrate PSM pots](/jhu-dvrk/sawIntuitiveResearchKit/wiki/psm-pot-calib-plate-in-place.jpg)
 * Two vertical bars pushing on the sides using Lego pieces.  One can probably use a rubber band to pull the two vertical bars against the gears.
  ![Lego bars to calibrate PSM pots](/jhu-dvrk/sawIntuitiveResearchKit/wiki/psm-pot-calib-lego-in-place.jpg)

* A design for laser cutting can be download: [DWG](https://github.com/hamlyn-centre/dVRK/blob/master/calibration_template.DWG), [Solidwork Part](https://github.com/hamlyn-centre/dVRK/blob/master/calibration_template.SLDPRT)

## 4.3. Calibrating scales

These instructions are for all arms, PSMs, MTMs and ECM.  For the calibration, one needs to start the `dvrk_console_json` application for the arm to be calibrated.  Since we also need the low level data (potentiometer values), we have to provide the `-i` option.  For example, to calibrate a PSM2, command line options for `dvrk_console_json` should look like:
```sh
# In directory ~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share
# <my-config-dir> is the directory with your sawRobotIO1394-PSM2-00000.xml configuration files 
rosrun dvrk_robot dvrk_console_json -j <my-config-dir>/console-PSM2.json -i ros-io-PSM2.json
```
The file `console-PSM2.json` is specific to each system since it points to your `sawRobotIO1394-PSM2-00000.xml` file.  On the other hand, the file `ros-io-PSM2.json` can be found in the `sawIntuitiveResearchKit/share` directory since it isn't system specific.

Once `dvrk_console_json` is started, make sure you can home the arm.  If you have multiple controllers connected to the same safety chain/e-stop, you can use the utility `qlacloserelays` to close all safety relays.

In a separate shell, start the calibration script using the following command line for dVRK 1.x:
```sh
# In directory ~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/<my-config-dir>
rosrun dvrk_robot dvrk_calibrate_potentiometers.py scales PSM2 sawRobotIO1394-PSM2-00000.xml
```
For dVRK 2.x:
```sh
# In directory ~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/<my-config-dir>
rosrun dvrk_robot dvrk_calibrate_potentiometers.py -t scales -a PSM2 -c sawRobotIO1394-PSM2-00000.xml
```
Make sure you use the same `sawRobotIO1394-XXX-00000.xml` for the calibration script and the console application!  The file name can be found in the console-PSM2.json file you're using.

The calibration script will query the arm serial number from the XML file and will display it.  The console application will do the same and display the serial number in the IO Qt widget.  This ensures that both applications are using an XML file specific to the arm you are trying to calibrate.  But, if you happen to use different copies of the configuration file for your arm, the current system has no way to detect it.  So, make sure you are using the same file for both applications (console and calibration script).

You will have to acknowledge a few prompt messages:
```
Calibrating scales using encoders as reference
Values will be saved in:  pot_calib_scales_sawRobotIO1394-PSM2-00000.csv
To start with some initial values, you first need to "home" the robot.  When homed, press [enter]
Since you are calibrating a PSM, make sure there is no tool inserted.  Please remove tool or calibration plate if any and press [enter]
The robot will make LARGE MOVEMENTS, please hit [enter] to continue once it is safe to proceed
```

**IMPORTANT NOTE:** For the scale calibration, we try to use a wide range of positions so the arm will pretty much go from joint limits to joint limits.  Make sure there are no obstacles in the way!

The result should look like:
```
index | old scale  | new scale  | correction
 0    | -44.329108 | -43.493731 |  1.019207
 1    | -29.309363 | -28.708860 |  1.020917
 2    |  60.074692 |  59.488202 |  1.009859
 3    | -78.384293 | -78.608156 |  0.997152
 4    | -77.862774 | -78.044577 |  0.997671
 5    | -78.279990 | -78.374442 |  0.998795
 6    | -79.427331 | -79.140566 |  1.003623
```

In this case you can see corrections as high as 2% on the third joint (index 2).  Press `y[enter]` to save the results in a new XML file.  You can review the changes with `meld` or your preferred diff tool.  If the changes make sense, replace your default XML configuration file with the new one:
```sh
# In directory ~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/<my-config-dir>
mv sawRobotIO1394-PSM2-00000.xml-new sawRobotIO1394-PSM2-00000.xml
```

Then stop the console application, make sure you restart it with the updated XML file and re-run the calibration script.  The results should improve:
```
index | old scale  | new scale  | correction
 0    | -43.493731 | -43.490507 |  1.000074
 1    | -28.708860 | -28.694983 |  1.000484
 2    |  59.488202 |  59.479411 |  1.000148
 3    | -78.608156 | -78.605950 |  1.000028
 4    | -78.044577 | -78.041157 |  1.000044
 5    | -78.374442 | -78.373988 |  1.000006
 6    | -79.140566 | -79.138265 |  1.000029
```
There is usually no point to save the results of the second pass.

## 4.3. Calibrating offsets

These instructions are for all arms but we only know how to properly hold the joints at their zero position for the last 4 joints of the **PSMs**.  If you need to calibrate offsets on different arms (MTM, ECM), you will need to figure out a way to constraint the arm to its zero position (mechanical zero).

For the scales calibration, you first need to start the console application and power the arm.  If the arm can power with the existing potentiometer offsets, home the arm.  You can then either keep the arm powered and use the motors to position it close to it's mechanical zero.   For the ECM and PSM, when the arm is maintained in position using its motors, you can use the "clutch" button to release the PID controller and position the arm manually.  For all arms, you can also use the ROS topics to send `move` goals or use the Qt GUI (dVRK 2.0 and above).  Once the arm is close to its mechanical zero position, you can use the script below.

In a separate shell, start the calibration script using the following command line for dVRK 1.x:
```sh
# In directory ~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/<my-config-dir>
rosrun dvrk_robot dvrk_calibrate_potentiometers.py offsets PSM2 sawRobotIO1394-PSM2-00000.xml
```
For dVRK 2.x:
```sh
# In directory ~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/<my-config-dir>
rosrun dvrk_robot dvrk_calibrate_potentiometers.py -t offsets -a PSM2 -c sawRobotIO1394-PSM2-00000.xml
```

Follow the instructions and place the calibration template (either Lego bars or plexiglass plate) when prompted to.  The result should look like:
```
index | old offset  | new offset  | correction
 0    |   99.441352 |   99.441352 |  0.000000 
 1    |   68.032665 |   68.032665 |  0.000000 
 2    |  -14.153006 |  -14.153006 |  0.000000 
 3    |  176.339392 |  177.817309 | -1.477917 
 4    |  176.606849 |  176.959943 | -0.353094 
 5    |  174.920864 |  175.741625 | -0.820761 
 6    |  179.924389 |  179.851204 |  0.073185 
```
For the MTMs or ECM, the script will save all joint offsets.   For the PSMs, since we know there is an easy way to calibrate the last 4 joint offsets, the script will prompt you to figure out if you should save all the joints or only the last 4.   If you are using the Lego bars or template describe above, **DO NOT** save all, just save the last 4.

Then stop the console application, make sure you restart it with the updated XML file and re-run the calibration script.  The results should improve:
```
index | old offset  | new offset  | correction
 0    |   99.441352 |   99.441352 |  0.000000 
 1    |   68.032665 |   68.032665 |  0.000000 
 2    |  -14.153006 |  -14.153006 |  0.000000 
 3    |  177.817309 |  177.817577 | -0.000269 
 4    |  176.959943 |  176.986576 | -0.026634 
 5    |  175.741625 |  175.801207 | -0.059582 
 6    |  179.851204 |  179.858797 | -0.007594 
```
Similar to the scales, there is usually no point to save the results of the second pass for the offsets.

## 4.4. Errors during the potentiometer calibration

There is a potential egg and chicken issue.   While trying to calibrate the potentiometers, the safety checks using the potentiometers are still active.   So if the original calibration (from `.cal` files) is way off, there will likely be some safety checks triggered while the robot arms are moving.

To avoid this, you can disable the pot/encoder safety checks but you MUST first visually check that the potentiometers and potentiometers work fine (see https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Debugging-Potentiometer-Issues#visual-checks), you can disable the pots/encoder safety checks manually during the calibration process.   To do so, “Home” the arm and then switch to the IO Qt Widget, check the “Direct control” box and approve.  Then uncheck “Use pot/encoder check” box.   At that point, you can start the scale calibration using the Python script.
