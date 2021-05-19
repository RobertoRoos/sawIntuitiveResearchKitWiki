<!--ts-->
   * [da Vinci Classic Active Arm Controllers (MTM, PSM, ECM)](#da-vinci-classic-active-arm-controllers-mtm-psm-ecm)
      * [Exterior Connectors](#exterior-connectors)
      * [Internal Components](#internal-components)
         * [Custom Boards (PCBs)](#custom-boards-pcbs)
         * [Power Supplies](#power-supplies)
      * [Hardware modifications](#hardware-modifications)
   * [da Vinci Classic Setup Joint Controller](#da-vinci-classic-setup-joint-controller)
      * [Exterior Connectors](#exterior-connectors-1)
      * [Internal Components](#internal-components-1)
      * [Installation](#installation)
   * [LEDs](#leds)
      * [Power](#power)
      * [Cards A and B](#cards-a-and-b)

<!-- Added by: anton, at:  -->

<!--te-->

For all acronyms see [FAQ](/jhu-dvrk/sawIntuitiveResearchKit/wiki/FAQ).

# da Vinci Classic Active Arm Controllers (MTM, PSM, ECM)

Each da Vinci arm (MTM, PSM, ECM) is controlled by a single box, shown below. A similar controller box is used for the Setup Joint controller (see below).  The controllers are built around 2 QLA/FPGA stacks.  They are designed to interface with the da Vinci Classic (first generation) active arms, both on the patient and surgeon's side.  They provide inputs for the potentiometers and encoders as well as miscellaneous digital IOs (foot pedals, buttons...).  For motor control they use linear amplifiers with current feedback.  All controllers come with FireWire interfaces so they can be daisy chained and communicate with a computer.  Later controllers came with an Ethernet adapter (supported with firmware 7+ and software 2+).  See also [controller versions](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Board-Versions). 

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
* [Component versions](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Board-Versions) by build/date.
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

## Hardware modifications

* dMIB:
  * [ECM switch for SUJ](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Full-da-Vinci-dMIB-pre-2015)
  * [PSM Dallas chip for tool detection](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Tool-Detection)
* QLAs:
  * [Heat sink and fan](/jhu-dvrk/sawIntuitiveResearchKit/wiki/QLA-Heat-Sink)

# da Vinci Classic Setup Joint Controller

The dVRK SUJ controller supports all the features available on the daVinci patient cart, i.e.:
* Read joint positions. The dVRK QLA has 4 analog to digital inputs so it reads the potentiometer values sequentially using a multiplexer.
* Release brakes.  The dVRK controller uses the linear amps of the QLA dedicated to motor control to release the brakes. 
* Lift PSM3.  The dVRK FPGA generates a PWM signal sent to the PWM power unit included on the dSIB.

For more details regarding the software features and configuration, see the [SUJ page](/jhu-dvrk/sawIntuitiveResearchKit/wiki/SUJ).

## Exterior Connectors

* One AC power connector, with on/off switch
* 4 156-pin connectors (one per SUJ arm)
* Two FireWire connectors
* One Ethernet connector
* Two 5-pin safety chain connectors; see [ESTOP page](/jhu-dvrk/sawIntuitiveResearchKit/wiki/ESTOP)

 ![SUJ controller exterior](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/suj-controller-exterior.png)

## Internal Components

Internally, each controller box contains one FPGA/QLA board set, one dSIB (da Vinci SUJ Interface Board), LED boards and 2 power supplies:
* 12V (50W) logic power supply that provides power to the FPGA board
* Brake power supply connected to the QLA (48V)

  ![SUJ controller interior](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/suj-controller-interior.png)

## Installation

The dVRK SUJ controller should be mounted on the back of the vertical column of the patient cart.   The enclosure comes with 4 holes that should match existing screw holes on the column.  You should:
* Remove the existing plate
* Leave the 4 thin grey cables untouched (these are for the potentiometers measuring the up/down translations for the SUJ) 
* Use spacers between the column and the dVRK SUJ controller to prevent crushing the cables
* Connect the SUJ connectors.  There should be enough space below the controller to run the SUJ-PSM1 and SUJ-ECM cables under it.

  <a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/suj-controller-mount.jpgg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/suj/suj-controller-mount.jpg" width="350"></a>

# LEDs

The dVRK controllers have LEDs grouped in different sections, Power and Card(s).

## Power

The LEDs are:
* Logic: power used for the FPGA board, i.e. on board computing/logic
* Rail A/B: power supplies used for motor control (see details above).  MTM controllers have two different motor power supplies so the Rail A and B LEDs are meaningful, PSM, ECM and SUJ controllers use a single motor power supply so the only meaningfull LED is Rail A.  Rail A/B, i.e. motor power, can be turned on/off using a PC and need the [safety chain](/jhu-dvrk/sawIntuitiveResearchKit/wiki/ESTOP) to be closed.

In general, the LEDs for power follow the following convention:
* Flashing Red - no power V<1V
* Solid Red - voltage present but too low, below VS_min
* Solid Green - voltage present, between VS_min & VS_max
* Solid Orange (Red & Green) - voltage present but too high, above VS_max

## Cards A and B

* The LEDs A/B (or C/D for the second card) are used to show that the firmware is fully loaded.  When the firmware is fully loaded both LEDs will go back and forth between red and green
* The LED MV is for Motor Voltage.  It should turn to green when motor power is requested
* The 8 LEDs (4 for Card A and 4 for card B) labelled either "Fault" (older controllers) or "Axis" (recent controllers) turn red when an axis is powered 
