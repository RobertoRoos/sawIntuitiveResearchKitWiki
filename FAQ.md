## What does ISI, Classic, S, Si, PSM, MTM, ECM, SUJ, HSRV, ... mean?

Some ISI acronyms:
 * ISI: Intuitive Surgical Inc.
 * Classic: refers to first generation da Vinci.  The research kit is based on the Classic.
 * S: second generation da Vinci.  New slaves, master arms are similar to first generation.
 * Si: third generation da Vinci. New masters, dual console, new stereo display, slave arms are similar to second generation.
 * PSM: Patient Side Manipulator, 2 to 3 on a full da Vinci system, 2 included in Research Kit: PSM1 and PSM2 - Mechanically identical. 
 * MTM: Master Tool Manipulator, 2 on a full da Vinci system (4 with dual console on SI system): 2 included in Research Kit: MTML and MTMR - Not mechanically identical, last joints are different for left and right arms.
 * ECM: Endoscopic Camera Manipulator, 1 on real da Vinci system.  Not included in Research Kit
 * SUJ: Setup Joints, 3 to 4 on a  da Vinci system.  Not included in Research Kit
 * HSRV: High Resolution Stereo Viewer.  At least 3 versions exist, CRT 640x480 (Standard and S), CRT 1024x720 (S HD), LCD (Si).  The Research Kit comes with the CRT 640x480.
 * Tray, foot pedal tray: foot pedals including clutch, camera, camera focus, bi and mono (or coag).  One is included in Research Kit.

Please read the Research Kit user manual: http://research.intusurg.com/dvrkwiki/index.php?title=DVRK:Docs:Main (ISI private wiki)


## What does QLA-FPGA, qladisp, ... mean?

Some JHU acronyms:
 * JHU: Johns Hopkins University.
 * QLA: quad linear amps, JHU designed board with power for up to 4 axis.
 * FPGA: logic board designed by JHU, mounted on top of the QLA.  Provides 2 firewire connector to daisy chain and connect to PC.
 * QLA-FPGA: board set including a QLA an FPGA board.
 * Firmware: embedded software running on the FPGA logic board.
 * dMIB: da Vinci Manipulator Interface Board, board designed to interface between the ISI manipulators using an ITT Cannon plus a foot pedal connector and the QLA-FPGA connectors (SCSI and RS cables)
 * dSIB: da Vinci Setup joints Interface Board.  This doesn't exist yet, we are working on it! 
 * Controller:
  * Enclosure
  * Two QLA-FPGA to control up to 8 axis
  * dMIB: mounted on the back
  * Power supplies: 12V for logic + motor power:
    * 24 V for all actuators on PSM
    * 24 V for first 3 actuators and 12 V for last 4 actuators on MTM
  * Safety relays
 * qladisp: text based application used to test up to 2 QLA-FPGA boards.  See [wiki:/sawIntuitiveResearchKitTutorial/Hardware#a2.2Testingwithqladisp]

See JHU Mechatronics for more details: http://jhu-cisst.github.io/mechatronics regarding the QLA-FPGA (JHU public page).


## What PC configuration do you suggest?

You will need a PC running Linux:
 * 4 to 8 cores
 * 4 GB RAM
 * A dedicated firewire controller for each chain of controllers - you can hook 4 controllers in a single chain so 1 firewire controller is fine for most users.
  * we've heard good thing re. SIIG !FireWire adapter NN-E20012-S2 (uses a TI chipset)
 * Ubuntu 12.04 LTS.  Our code is portable and should run well on other systems but we only tested 12.04 LTS.


## The PSMs aren't moving

All manipulators should be completely back drivable when not powered.  If your PSMs are stiff and you can't move all the joints by hand, make sure you have removed all the brackets and zip ties used to protect the arm during transportation.   Please read the unpacking guide: http://research.intusurg.com/dvrkwiki/index.php?title=DVRK:Docs:Main (ISI private wiki).


## All signals freeze for a couple seconds

 * Make sure you don't have two programs trying to access the firewire controllers simultaneously.   The low-level API provided by JHU performs a check but you might have found a way to defeat it.
 * Make sure you don't have any other firewire devices on the same firewire controller.  For example, don't connect an external firewire hard drive or camera on the same firewire chain.   Please note that PC that comes with a built-in firewire controller might provide multiple external connectors that are managed by a single firewire chip.
 * Make sure your firewire "chain" is good from your computer to the last FPGA-QLA board set.  See Firewire below.
 

## Some intermittent signals

 * Make sure you don't have any loose ITT Cannon cable from your manipulator to the controller.
 * Make sure the SCSI and RS cables between the dMIB and QLA boards inside the enclosure are not loose.
 

## Power Issue
 
ESTOP


## Firewire Connection

### Symptom
 * Can't even connect to controllers
 * See some of the QLA-FPGA but not all
 * `qladisp` seems to be working but the display freezes for a couple of seconds

### Firewire Cables

Cheap cables tend to have more quality issues.  We found a working combination for our controllers (at JHU), see http://research.intusurg.com/dvrkwiki/index.php?title=DVRK:GroupPages:JHU#Mechatronics (ISI private wiki).  The best way to test your setup is to start with a single cable and single QLA-FPGA and then add the extra boards one by one to the daisy chain.   This requires to open the controller enclosures.


## I plugged my PC to the stereo display but I get no image?

You need a special cable and the correct settings on your computer.  The cable adapter is not provided with the Research Kit and is not standard so you will have to make it.  See http://research.intusurg.com/dvrkwiki/index.php?title=DVRK:Topics:StereoViewer (ISI private wiki).