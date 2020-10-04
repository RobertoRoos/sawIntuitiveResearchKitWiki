# Controller Boxes (MTM, PSM, ECM)

Each da Vinci arm (MTM, PSM, ECM) is controlled by a single box, shown below. A similar controller box is used for the Setup Joint controller.

## Exterior Connectors
* One AC power connector, with on/off switch
* One 156-pin connector (for the MTM, PSM, or ECM)
* Two FireWire connectors
* Two Ethernet connectors (later versions)
* One or two 4 or 5-pin safety chain connectors (depending on version); see [ESTOP page](/jhu-dvrk/sawIntuitiveResearchKit/wiki/ESTOP)
* One DB15 footpedal connector; see [dMIB I/O page](/jhu-dvrk/sawIntuitiveResearchKit/wiki/dMIB-IOs)
* Seven HD15 expansion connectors and one HD26 expansion connector; see [dMIB I/O page](/jhu-dvrk/sawIntuitiveResearchKit/wiki/dMIB-IOs)

## Internal Components 

Internally, each controller box contains two FPGA/QLA board sets, one dMIB (da Vinci Manipulator Interface Board), LED boards, power supplies and relays.

  ![Controller layout](/jhu-dvrk/sawIntuitiveResearchKit/wiki/controller-layout.jpg)

### Custom Boards (PCBs)
* [Component versions](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Board-Versions).
* The FPGA and QLA designs are open source and available via this [GitHub page](http://jhu-cisst.github.io/mechatronics/).
* The dMIB is provided by Intuitive Surgical. The designs, including schematics and BOM, are available via this [GitHub page](https://github.com/jhu-dvrk/dvrk-pcb-dMIB).

### Power Supplies
* All boxes contain a 12V (50W) logic power supply that provides power to the FPGA boards and the safety chain.
* Each box also contains one or more motor power supplies that are connected to the QLAs:
  * MTM: one 24V (75W) power supply connected to QLA #1 and one 12V (50W) power supply connected to QLA #2
  * PSM: one 24V (225W) power supply connected to both QLAs
  * ECM: one 36V (225W) power supply connected to both QLAs
* Replacement power supplies
  * 12V Logic Power Supply (For All) & 12V Motor Power Supply (For MTM): https://www.digikey.com/product-detail/en/cui-inc/VGS-50-12/102-1935-ND/2045666
  * 24V Motor Power Supply (For MTM): https://www.digikey.com/product-detail/en/cui-inc/VGS-75-24/102-1943-ND/2045674
  * 24V Motor Power Supply (For PSM): https://www.astrodynetdi.com/ecatalog/power-supplies/PMK225S-24U
  * 36V Motor Power Supply (For ECM): https://www.astrodynetdi.com/ecatalog/power-supplies/PMK225S-36U