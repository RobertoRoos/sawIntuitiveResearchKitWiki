<!--ts-->
   * [Robot IO (XML)](#robot-io-xml)
   * [Automatic generation](#automatic-generation)
   * [Porting between releases](#porting-between-releases)
      * [Version 3 (dVRK 1.5)](#version-3-dvrk-15)
      * [Version 4 (dVRK 2.0)](#version-4-dvrk-20)

<!-- Added by: anton, at: 2021-04-05T15:25-04:00 -->

<!--te-->

# Robot IO (XML)

These files are used to configure the IOs for each arm identified by its serial number.  They include the board IDs, digital inputs/outputs to use, conversion factors for the encoders, potentiometers, commanded current and current feedback as well as some safety parameters and coupling matrices (for MTMs).  For details of implementation, see code from [sawRobotIO1394](https://github.com/jhu-saw/sawRobotIO1394).

You can find examples of dVRK specific configuration files in the "shared" directory:
https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/master/share


# Automatic generation

The normal procedure to create your sawRobotIO1394 XML configuration files is:
* Obtain the calibration files from ISI (`.cal` files).  There will be one file per arm identified by its serial number (see [XML configuration](/jhu-dvrk/sawIntuitiveResearchKit/wiki/XMLConfig)).
* Generate the XML configuration files from the `.cal` files using the provided Matlab application (see [XML configuration](/jhu-dvrk/sawIntuitiveResearchKit/wiki/XMLConfig)).
* Calibrate each arm.  The different calibration procedures will update the XML configuration file for each arm (see [Calibration](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Calibration))

# Porting between releases

Some new dVRK release might require to port your RobotIO XML configuration files.  The safest approach is to re-generate and re-calibrate the configuration files for each release.  In some cases, it is possible to update the XML configuration files manually.

## Version 3 (dVRK 1.5)

Version 3 for the sawRobotIO1394 XML configuration files was introduced in dVRK release 1.5.  One can manually port from version 2 to 3 but since we introduced a new and improved current calibration procedure in release 1.5, it is strongly recommended to regenerate your XML configuration files and perform the normal calibration procedures.

If you are willing to update the files manually, you will need to change:
* `Version` should be set to `3`.
* All `DigitalIn` in MTM configuration files should be removed.
* All unused `DigitalIn` in PSM and ECM configuration files should be removed.
* In MTM configuration files, remove all coupling matrices except `ActuatorToJointPosition`.
* In PSM and ECM configuration files, remove the `Coupling` section altogether.
* In all files, you need to add the potentiometers tolerance.  The defaults depend on the arm type, check existing configuration files in `share/jhu-dVRK`.  For example, an MTM should have:
   ```XML
      <Potentiometers Position="Joints">
         <Tolerance Axis="0" Distance="5.00" Latency="0.01" Unit="deg"/>
         <Tolerance Axis="1" Distance="5.00" Latency="0.01" Unit="deg"/>
         <Tolerance Axis="2" Distance="5.00" Latency="0.01" Unit="deg"/>
         <Tolerance Axis="3" Distance="5.00" Latency="0.01" Unit="deg"/>
         <Tolerance Axis="4" Distance="5.00" Latency="0.01" Unit="deg"/>
         <Tolerance Axis="5" Distance="5.00" Latency="0.01" Unit="deg"/>
         <Tolerance Axis="6" Distance="1000.00" Latency="0.01" Unit="deg"/>
         <Tolerance Axis="7" Distance="1000.00" Latency="0.01" Unit="deg"/>
      </Potentiometers>
   ```

## Version 4 (dVRK 2.0)

The safest solution is to restart from the calibrations files.  But, if you have some configurations files that are hard to re-calibrate, you should be able to update your configuration files by hand.  The main changes are:
* `Version` should be set to `4`.
* `BitsToPosSI` and `VoltsToPosSI` now require a `Unit` to be specified.  The implicit units were `deg` and `mm` so if you don't change the `Scale` or `Offset`, make sure you add `Unit="deg" (for `Type="Revolute"`) or `Unit="mm"` (for `Type="Prismatic"`).
* For MTMs, the axis "7", i.e. the 8th and last joint has been removed.  It was used for the Hall Effect sensor in the gripper.  We now use a separate file.  This allows to not enable power on the 8th axis.  So you need to update `NumOfActuator` and `NumOfJoints` to `7`, remove all the settings for the last joint, remove the `Potentiometers/Tolerance`for `Axis="7"` and finally remove the last column and row of `ActuatorToJointPosition` (resize matrix from 8x8 to 7x7).
* For MTMs, you will need a new configuration file for the gripper.  The file's name must match the arm's name, for example: `sawRobotIO1394-MTMR-gripper-28247.xml`.  In the file name, replace the serial number with your serial number.  To create the file, you can either use the XML configuration generator or start from an existing file in `share/jhu-dVRK`.  If you start from an existing file, make sure you update the `Name`, `SN` and `BoardID`.  In any case, you should run the [gripper calibration tool](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Calibration#3-gripper-on-mtms) afterwards.
* For PSMs, assuming that your hardware can access the tool [Dallas chip](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Tool-Detection), you will need to add `<DallasChip BoardID="7" Name="PSM1-Dallas" />`.   The `BoardID` should match the board ID of the second board in the PSM controller, i.e. 7 for a PSM1, 9 for a PSM2 and 11 for a PSM3.  Make sure the `Name` also reflect the PSM name/number.
