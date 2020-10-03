<!--ts-->
   * [Introduction](#introduction)
   * [Hardware](#hardware)
      * [Prerequisites](#prerequisites)
      * [dMIB rev F or newer](#dmib-rev-f-or-newer)
      * [Older dMIB modification](#older-dmib-modification)
   * [Software](#software)
      * [Testing](#testing)
      * [dVRK console configuration](#dvrk-console-configuration)

<!-- Added by: anton, at:  -->

<!--te-->

# Introduction

The da Vinci instruments can be automatically identified when inserted in the sterile adapter using the `add-only` chip embedded in the tool (aka Dallas chip).  This feature was not supported in early versions of the dVRK both from a software and hardware perspective.

To retrieve the instrument tool type, the dVRK hardware can use two different approaches:
* dMIB Dallas driver chip.  This chip communicates with the tool using a 1-wire interface.  The FPGA then communicates with the dMIB Dallas chip.  This requires a recent dMIB (See requirements below).
* FPGA 1-wire interface.  In this scenario, the FPGA/QLA emulates a Dallas driver chip and communicates directly with the tool's chip.  This method can be used on slightly modified older dMIBs as well as recent dMIBs.

# Hardware

## Prerequisites

Reading the instrument info requires:
* a dMIB rev F or newer, or a modified dMIB rev A-E (see below)
* a QLA version 1.4 or newer
* FPGA firmware 7 or newer
* dVRK software 2.0 or higher

The dMIB and QLA versions are printed on the silkscreen on both boards. You can also check the version of QLA by querying the serial number.  To locate the dMIB and QLA, see [Controller Boxes](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Controller-Boxes).

If your dVRK controller is shipped later than `??`, you do not need to modify. You should have a v1.4+ QLA and rev F+ dMIB. The instrument info is supported out of the box.

Otherwise, if you have a QLA version 1.4 or newer (shipped later than `??`), your controller is compatible with the modification in the next section.

We do not have a plan to support QLA earlier than version 1.4.

## dMIB rev F or newer

For recent dMIBs, one can either the dMIB Dallas driver or the 1-wire FPGA approach.  To select which one is be used, you need to place a jumper on the dMIB.  The jumper (J42) is located between the two SCSI-68 cables on the internal face of the dMIB (i.e. you don't need to remove the dMIB from the controller to access the jumper).

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dmib-tool-jumper-empty.jpg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dmib-tool-jumper-empty.jpg" width="250"></a>

To configure the dMIB to use the 1-wire FPGA based approach, you need to jump the first 2 pins (see picture).

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dmib-tool-jumper-FPGA.jpg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dmib-tool-jumper-FPGA.jpg" width="250"></a>

To use the dMIB Dallas driver, you will need to jump the pins 2 and 3.

**Note:** As of September 2020, the dMIB Dallas driver approach is not yet supported by the firmware.

## Older dMIB modification

:warning: | Do not do this if you have a recently built controller (with dMIB rev F or newer)
:---: | :---

This dMIB hardware modification connects a wire between the QLA and the add-only memory inside the instruments so the dVRK software can read the type of the installed instrument.

You need the PSM dVRK controllers, screwdrivers/nut drivers/hex wrenches, a piece of small insulated wire or magnet wire, and a soldering iron.

**Step 1.** Unplug power. Unplug cables from the dMIB/QLA so you can work on the back side of the 156-pin ITT Cannon connector (that mates with the robot arm connector) or take the dMIB out. Please make sure to label the cables as you unplug them.

**Step 2.** (optional) Remove dMIB from the PSM dVRK controller box. This step may be optional if you have small dexterous fingers and good soldering skills (or use the EndoWrist soldering iron).

**Step 3.** See the figure below. Solder a jumper wire between the `R1 pin` in the 156-pin connector and the resistor `R69` pad that is closest to the SCSI connector. *Some dMIB have misaligned silkscreen for the 156-pin connector, like the rev. D in the figure.* Do not remove the resistor R69. If you did so and cannot solder the original part back in, you can jump an approximately 1 kOhm resistor between the `R69` pad you connected the jumper wires to and the `T6 pin` of the 156-pin connector.

![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dmib-tool-info-mod.jpg)

**Step 4.** Reconnect the cables between QLA and dMIB. Connect the PSM and test the functionality. Reassemble the controller box.

# Software

## Testing

You can test the hardware and firmware configuration using the command line tool `instrument` provided with the low level software (along `qladisp` in `AmpIO`).  The `instrument` program will dump the memory from the instrument in a text file.  Make sure you have an instrument properly seated in the sterile adapter before launching the program.
```
instrument [-pP] <board num>
```

## dVRK console configuration

This requires the dVRK stack rev 2.0 or higher.  In your PSM configuration file, you can set the tool detection to be manual, automatic or fixed:
```json
{
    "kinematic": "kinematic/psm.json",
    "tool-detection": "AUTOMATIC"
    // "tool-detection": "MANUAL"
    // "tool-detection": "FIXED",
    // "tool": "LARGE_NEEDLE_DRIVER_400006"
}
```
The different options for `tool-detection` are:
* `AUTOMATIC`: this will rely on the Dallas chip query
* `MANUAL`: when a tool is inserted, the user or application has to specify which tool to use.  This can be done using the Arm GUI with a drop down menu or using a programmatic interface (e.g. ROS topic).
* `FIXED`: fixed type of tool, i.e. there is only one type of tool used.  The configuration file must then define the tool type using `tool`.  Tools definitions can be found in `share/tool`.