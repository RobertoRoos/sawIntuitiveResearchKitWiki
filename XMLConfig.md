<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

- [XML configuration file](#xml-configuration-file)
  - [1. How to generate XML configuration file](#1-how-to-generate-xml-configuration-file)
  - [2. Config Generator](#2-config-generator)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# XML configuration file

The software requires XML files with configuration parameters for the various manipulators.
Sample files with nominal values are in the [share sub-directory](/jhu-dvrk/sawIntuitiveResearchKit/tree/master/share).

For a specific da Vinci manipulator, Intuitive Surgical provides manipulator-specific calibration data in a `cal` file.  The sample files in the repository can be used to get started but the calibrations values are quite different between systems so it is important to generate a configuration file per arm using the `cal` file provided by Intuitive Surgical.  We provide a MATLAB-based GUI to read the `cal` file and generate the corresponding XML file.

## 1. How to generate XML configuration file

1. Choose the calibration file provided by Intuitive Surgical, mXXXX.cal or pXXXX.cal. The naming convention is that the first letter indicates device type with 'm' for master tool manipulator and 'p' for patient-side manipulator (slave), followed by the device serial number. The serial number can be found on the mechanical arm itself, on a silver label with multiple bar codes.   Look for the 'TRK ID'.

![TRK ID on MTL](/jhu-dvrk/sawIntuitiveResearchKit/wiki/trk-id-mtm.jpg)
![TRK ID on PSM](/jhu-dvrk/sawIntuitiveResearchKit/wiki/trk-id-psm.jpg)

1. Choose device type under 
 * MTML: master left
 * MTMR: master right
 * PSM1: slave 1
 * PSM2: slave 2

1. Choose board ID: the board ID is the rotary switch value (4-bit from 0 to F), which should be unique among daisy-chained controller boards. The board ID should be set based on the following convention.  Not all Research Kit come with the ECM and the PSM3 but the board Id should be reserved nevertheless:

  |            | MTML | MTMR | ECM | PSM1 | PSM2 | PSM3   |
  |------------|------|------|-----|------|------|--------|
  | Board ID 1 | 0    | 2    | 4   | 6    | 8    | 10 (A) |
  | Board ID 2 | 1    | 3    | 5   | 7    | 9    | 11 (B) |

 * To make sure the board IDs are set properly, you will have to open the controller enclosures.  The board ID can be changed by turning the rotary switch with a flat head screw driver.

![Board ID selector](/jhu-dvrk/sawIntuitiveResearchKit/wiki/board-id-selector.jpg)

 * Looking from the front of the enclosure, the first board is on the left and the second is on the right.  While you have the enclosure open, you should check that the QLA-FPGA sets are properly connected to the dMIB on the back of the controller: first board connected to bottom connectors, second board to top connectors.

![Controller layout](/jhu-dvrk/sawIntuitiveResearchKit/wiki/controller-layout.jpg)

 * If your configuration is unusual, you can overwrite the default IDs and specify the board IDs manually using the drop down menu to choose the board ID.  Make sure these two board IDs are different, otherwise the configuration file will not be generated. Also, if you plan to daisy chain multiple controllers, make sure you don't have two boards with the same ID on a single firewire port. 

1. Choose digital input settings.  For all PSMs, the defaults shouldn't be modified unless you are using some digital inputs for a specific application.  For the MTMs, you need to select 'Default Footpedal' for the controller box you plan to connect the foot pedal to.   If you plan to daisy chain multiple controllers, make sure the digital input names are unique.  As a direct consequence, make sure only one of the MTMs is configured with the digital inputs for the foot pedal.

1. Choose drive direction.  You should hit the 'Default' button for all arms.  This multipliers (1 or -1) allow to change the direction of each axis.  This flag is used to make sure that the software uses the axis directions as defined in the Intuitive Research Kit User Manual.  It is important check the direction of each axis for each arm to make sure it corresponds to the documentation using the `sawIntuitiveResearchKitQtPID` or any other applications that allows you to monitor the potentiometers and encoder directions as well as torque directions.  If you find that the default directions don't match your hardware, please let us know. 

1. Generate: click the generate button to generate XML configuration file. 

![XML config generator screenshot](/jhu-dvrk/sawIntuitiveResearchKit/wiki/configGUI_tutorial.png)


## 2. Config Generator

This configuration generator consists of three files: 
* configGUI.m: GUI logic 
* configGUI.fig: GUI design file
* configGenerator.m: xml file generator with real computation

If you want to know how values in the XML file are computed, please check [configGenerator.m](/jhu-dvrk/sawIntuitiveResearchKit/tree/master/share) for more information.