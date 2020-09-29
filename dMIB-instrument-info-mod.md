This dMIB hardware modification connects a wire between the QLA and the add-only memory inside the instruments so the dVRK software can read the type of the installed instrument.

# Prerequisites

Reading the instrument info requires 

* a dMIB rev F or newer, and 
* a QLA version 1.4 or newer, and
* FPGA firmware 7 or newer.

The version is printed on the silkscreen on both boards. You can also check the version of QLA by querying the serial number.

If your dVRK controller is shipped later than `??`, you do not need to modify. You should have a v1.4+ QLA and rev F+ dMIB. The instrument info is supported out of the box.

Otherwise, if you have a QLA version 1.4 or newer (shipped later than `??`), your controller is compatible with the modification in the next section.

We do not have a plan to support QLA earlier than version 1.4.

# Modification

You need the PSM dVRK controllers, screwdrivers/nut drivers/hex wrenches, a piece of small insulated wire or magnet wire, and a soldering iron.

**Step 1.** Unplug power. Unplug cables from the dMIB/QLA so you can work on the back side of the 156-pin ITT Cannon connector (that mates with the robot arm connector) or take the dMIB out. Please make sure to label the cables as you unplug them.

**Step 2.** (optional) Remove dMIB from the PSM dVRK controller box. This step may be optional if you have small dexterous fingers and good soldering skills (or use the EndoWrist soldering iron). 

**Step 3.** See the figure below. Solder a jumper wire between the `R1 pin` in the 156-pin connector and the resistor `R69` pad that is closest to the SCSI connector. *Some dMIB have misaligned silkscreen for the 156-pin connector, like the rev. D in the figure.* Do not remove the resistor R69. If you did so and cannot solder the original part back in, you can jump an approximately 1 kOhm resistor between the `R69` pad you connected the jumper wires to and the `T6 pin` of the 156-pin connector.

![](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/dmib-tool-info-mod.jpg)

**Step 4.** Reconnect the cables between QLA and dMIB. Connect the PSM and test the functionality. Reassemble the controller box.

# Testing

Use the `instrument` program to dump the memory from the instrument.

```
instrument [-pP] <board num>
```