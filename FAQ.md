<!--ts-->
   * [What does ISI, Classic, S, Si, Xi, PSM, MTM, ECM, SUJ, HRSV, ... mean?](#what-does-isi-classic-s-si-xi-psm-mtm-ecm-suj-hrsv--mean)
   * [Generations of da Vinci systems](#generations-of-da-vinci-systems)
   * [What does QLA-FPGA, qladisp, ... mean?](#what-does-qla-fpga-qladisp--mean)
   * [What PC configuration do you suggest?](#what-pc-configuration-do-you-suggest)
   * [The PSMs aren't moving](#the-psms-arent-moving)
   * [Where are qladisp and qlacloserelays, everything else has been compiled](#where-are-qladisp-and-qlacloserelays-everything-else-has-been-compiled)
   * [All signals freeze for a couple seconds](#all-signals-freeze-for-a-couple-seconds)
   * [Some intermittent signals](#some-intermittent-signals)
   * [Power issues](#power-issues)
   * [Firewire connection](#firewire-connection)
      * [Symptom](#symptom)
      * [Firewire cables](#firewire-cables)
   * [I plugged my PC to the stereo display but I get no image?](#i-plugged-my-pc-to-the-stereo-display-but-i-get-no-image)

<!-- Added by: anton, at:  -->

<!--te-->

# What does ISI, Classic, S, Si, Xi, PSM, MTM, ECM, SUJ, HRSV, ... mean?

Some ISI acronyms:
 * **ISI**: Intuitive Surgical Inc.
 * **Classic**: refers to first generation da Vinci.  The research kit is based on the Classic.  Also know as Standard.
 * **S**: second generation da Vinci.  New PSMs/ECM, MTMs arms are similar to first generation.
 * **Si**: third generation da Vinci. New MTMs, dual console, new stereo display, PSMs are similar to second generation.
 * **Xi**: fourth generation da Vinci.  New setup joints, new arm to replace both PSMs and ECM (now known as USMs) can be used to hold tools or camera.  Surgeon console and stereo display similar to Si.
 * **X**: Similar to Si but with Xi patient side arms.
 * **PSM**: Patient Side Manipulator, 2 to 3 on a full da Vinci system, 2 included in Research Kit: PSM1 and PSM2 - Mechanically identical.
 * **MTM**: Master Tool Manipulator, 2 on a full da Vinci system (4 with dual console on Si/Xi system): 2 included in Research Kit: MTML and MTMR - Not mechanically identical, last joints are different for left and right arms.
 * **ECM**: Endoscopic Camera Manipulator, 1 on real da Vinci system.
 * **SUJ**: Setup Joints, 3 to 4 on a  da Vinci system.  Not included in Research Kit yet.
 * **HRSV**: High Resolution Stereo Viewer.  At least 3 versions exist, CRT 640x480 (Standard and S), CRT 1024x768 (S HD), LCD (Si/Xi).  The Research Kit comes with the CRT 640x480.
 * **CCU**: Camera Control Unit.   The two boxes in the vision cart that are connected to the endoscope cameras.  These usually have either an NTSC or SDI output for frame grabbers.
 * Tray, foot pedal tray: foot pedals including clutch, camera, camera focus, bi and mono (or coag).  One is included in Research Kit.

Please read the Research Kit user manual: http://research.intusurg.com/dvrkwiki/index.php?title=DVRK:Docs:Main (ISI private wiki)

# Generations of da Vinci systems

This is a high level description of the different generations of da Vinci systems.

| Model | Year | Surgeon's console | PSM/ECM/USM | Setup Joints | Endoscope |
|-------|------|-------------------|----------|--------------|-----------|
| Classic | 2000 | Rev 1 | Rev 1 (PSM/ECM)| Rev 1 | Rev 1 with SD |
| S       | 2006 | Rev 1 | Rev 2 (PSM/ECM)| Rev 2 | Rev 1 with SD or HD |
| Si      | 2009 | Rev 2 | Rev 2 (PSM/ECM)| Rev 2 | Rev 2 HD|
| X       | 2017 | Rev 2 | Rev 3 (USM)| Rev 2 | Rev 3 |
| Xi      | 2014 | Rev 2 | Rev 3 (USM)| Rev 3 | Rev 3 |

# What does QLA-FPGA, qladisp, ... mean?

Some JHU acronyms:
 * **JHU**: Johns Hopkins University.
 * **QLA**: quad linear amplifier, JHU designed board with power for up to 4 axis.
 * **FPGA**: logic board designed by JHU, mounted on top of the QLA.  Provides 2 firewire connector to daisy chain and connect to PC. Version 2+ also includes an Ethernet connector.
 * **QLA-FPGA**: board set including a QLA and FPGA board.
 * **Firmware**: embedded software running on the FPGA logic board.
 * **dMIB**: da Vinci Manipulator Interface Board, board designed to interface between the ISI manipulators using an ITT Cannon plus a foot pedal connector and the QLA-FPGA connectors (SCSI and RS cables)
 * **dSIB**: da Vinci Setup joints Interface Board.
 * **Controller**:
   * Enclosure
   * Two QLA-FPGA to control up to 8 axes
   * dMIB: mounted on the back
   * Power supplies: 12V for logic + motor power:
      * 24 V for all actuators on PSM
      * 24 V for first 3 actuators and 12 V for last 4 actuators on MTM
      * 36 V for ECM, mostly to be able to release the brakes
      * 48 V for SUJ, for brakes and PWM units to lift/lower the PSM3 SUJ
   * Safety relays
 * `qladisp`: text based application used to test up to 2 QLA-FPGA boards.  See [testing hardware with `qladisp`](/jhu-dvrk/sawIntuitiveResearchKit/wiki//Hardware#22-testing-with-qladisp)
 * `qlacloserelays`: text based application used to close safety relays for all the controllers connected through FireWire, very useful if you're not using all the controllers connected through the FireWire and e-stop chain and don't want to re-configure all your wiring.

See JHU Mechatronics for more details: http://jhu-cisst.github.io/mechatronics regarding the QLA-FPGA (JHU public page).


# What PC configuration do you suggest?

You will need a PC running Linux:
 * 8 cores recommended (Intel i7, i9 or equivalent Xeon)
 * 8 GB RAM minimum, 16 recommended
 * FireWire
   * A dedicated firewire controller for each chain of controllers - you can hook 8 controllers (16 FPGA/QLA boards) in a single chain so 1 firewire controller is fine for most users.
   * FireWire adapters:
     * [SYBA Low Profile PCI-Express Firewire](https://www.amazon.com/gp/product/B002S53IG8/).  This card comes with a regular and a low profile plate so it can also be used in low profile and full size desktop computers 
     * Early dVRK users have tested different cards and the SIIG FireWire adapter NN-E20012-S2 works well (uses a TI chipset)
   * See also the following document: http://support.presonus.com/hc/en-us/article_attachments/203654243/Compatible_Hardware_List_7-12.pdf.  Any card from the compatible list should work.
   * If possible, choose a PCIe card to get better performance.  Try to avoid adapters with a PCI chipset and a PCI to PCIe bridge.
   * See also https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/ControllerConnection
 * Graphic adapters
   * If you plan to send images to the stereo display, you will need two extra VGA outputs for the standard CRTs or two DVI outputs for the flat panels ([see ISI private Wiki](http://research.intusurg.com/dvrkwiki/index.php?title=DVRK:Topics:StereoViewerLCD))
   * In general Nvidia cards work fine on Linux.  If you have multiple cards, try to match them (use same model for all cards) but we strongly recommend using a single card with enough outputs to drive all the monitors you need.
 * Software
   * Ubuntu 16.04 or 18.04 (64 bits of course)
   * ROS Kinetic on Ubuntu 16.04, Melodic on Ubuntu 18.04
   * As far as we know, recent versions of the dVRK software (1.3 to 1.6) will likely compile with older Ubuntu/ROS so you might not have to upgrade your OS.  For release 1.7, you will need a recent C++ compiler (Ubuntu 14.04 doesn't work as-is, 16.04 and 18.04 are fine).

# The PSMs aren't moving

All manipulators should be completely back drivable when not powered.  If your PSMs are stiff and you can't move all the joints by hand, make sure you have removed all the brackets and zip ties used to protect the arm during transportation.   Please read the unpacking guide: [ISI private wiki](http://research.intusurg.com/dvrkwiki/index.php?title=DVRK:Docs:Main).

# Where are `qladisp` and `qlacloserelays`, everything else has been compiled

You're probably missing the curses development libraries.  Install them, re-run CMake or just `catkin build` if you're a ROS user.   To install on Ubuntu:
```sh
  sudo apt-get install libncurses5-dev
```

# All signals freeze for a couple seconds

 * Make sure you don't have two programs trying to access the firewire controllers simultaneously.   The low-level API provided by JHU performs a check but you might have found a way to defeat it.
 * Make sure you don't have any other firewire devices on the same firewire controller.  For example, don't connect an external firewire hard drive or camera on the same firewire chain.   Please note that PC that comes with a built-in firewire controller might provide multiple external connectors that are managed by a single firewire chip.
 * Make sure your firewire "chain" is good from your computer to the last FPGA-QLA board set.  See Firewire below.

# Some intermittent signals

 * Make sure you don't have any loose ITT Cannon cables from your manipulator to the controller.
 * Make sure the SCSI and RS cables between the dMIB and QLA boards inside the enclosure are not loose.
 * See also [controller boxes description](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Controller-Boxes) and [debugging potentiometer issues](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Debugging-Potentiometer-Issues)

# Power issues

Most power issues are related to the emergency stop: [E-Stop](/jhu-dvrk/sawIntuitiveResearchKit/wiki/ESTOP)

# Firewire connection

## Symptom
 * Can't even connect to controllers
 * See some of the QLA-FPGA but not all
 * `qladisp` seems to be working but the display freezes for a couple of seconds

## Firewire cables

Cheap cables tend to have more quality issues.  We strongly recommend to buy your FireWire cables from Unibrain.

For a more detailed description of the cables used for our controllers (at JHU), see [ISI private wiki](http://research.intusurg.com/dvrkwiki/index.php?title=DVRK:GroupPages:JHU#Mechatronics).  The best way to test your setup is to start with a single cable and single QLA-FPGA and then add the extra boards one by one to the daisy chain.   This requires to open the controller enclosures.

# I plugged my PC to the stereo display but I get no image?

You need a special cable and the correct settings on your computer.  The cable adapter is not provided with the Research Kit and is not standard so you will have to make it.  See [ISI private wiki](http://research.intusurg.com/dvrkwiki/index.php?title=DVRK:Topics:StereoViewer).
