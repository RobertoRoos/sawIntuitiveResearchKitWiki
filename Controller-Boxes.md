# Controller Boxes (MTM, PSM, ECM)

Each da Vinci arm (MTM, PSM, ECM) is controlled by a single box, shown below. A similar controller box is used for the Setup Joint controller.

## Exterior Connectors
* One AC power connector, with on/off switch
* One 156-pin connector (for the MTM, PSM, or ECM)
* Two FireWire connectors
* Two Ethernet connectors (later versions)
* One or two 4 or 5-pin safety chain connectors (depending on version); see [ESTOP page](/jhu-dvrk/sawIntuitiveResearchKit/ESTOP)

## Internal Components 

Internally, each controller box contains two FPGA/QLA board sets, one dMIB (da Vinci Manipulator Interface Board), LED boards, power supplies and relays.

### Custom Boards (PCBs)
* The FPGA and QLA designs are open source and available via this [GitHub page](http://jhu-cisst.github.io/mechatronics/).
* The dMIB is provided by Intuitive Surgical. Schematics/BOM are available here:
  * Schematics: Rev B, Rev D, Rev E, Rev F
  * Bill of Materials (BOM): Rev B, Rev D, Rev E, Rev F

### Power Supplies
* All boxes contain a 12V logic power supply that provides power to the FPGA boards and the safety chain.
* Each box also contains one or more motor power supplies that are connected to the QLAs:
  * MTM: one 24V power supply connected to QLA #1 and one 12V power supply connected to QLA #2
  * PSM: one 24V power supply connected to both QLAs
  * ECM: one 36V power supply connected to both QLAs