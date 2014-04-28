# XML configuration file

The software requires XML files with configuration parameters for the various manipulators.
Sample files with nominal values are in the [source:trunk/saw/applications/sawIntuitiveResearchKit/share share sub-directory].

For a specific da Vinci manipulator, Intuitive Surgical provides manipulator-specific calibration data in a `cal` file.  The sample files in the repository can be used to get started but the calibrations values are quite different between systems so it is important to generate a configuration file per arm using the `cal` file provided by Intuitive Surgical.  We provide a MATLAB-based GUI to read the `cal` file and generate the corresponding XML file.

## 1. How to generate XML configuration file

1. Choose the calibration file provided by Intuitive Surgical, mXXXX.cal or pXXXX.cal. The naming convention is that the first letter indicates device type with 'm' for master tool manipulator and 'p' for patient-side manipulator (slave), followed by the device serial number. The serial number can be found on the mechanical arm itself, on a silver label with multiple bar codes.   Look for the 'TRK ID'.

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

[[Image(wiki:sawIntuitiveResearchKitTutorial/XMLConfig:board-id-selector.jpg, 400px)]]

 * Looking from the front of the enclosure, the first board is on the left and the second is on the right.  While you have the enclosure open, you should check that the QLA-FPGA sets are properly connected to the dMIB on the back of the controller: first board connected to bottom connectors, second board to top connectors.

[[Image(wiki:sawIntuitiveResearchKitTutorial/XMLConfig:controller-layout.jpg, 400px)]]

 * If your configuration is unusual, you can overwrite the default IDs and specify the board IDs manually using the drop down menu to choose the board ID.  Make sure these two board IDs are different, otherwise the configuration file will not be generated. Also, if you plan to daisy chain multiple controllers, make sure you don't have two boards with the same ID on a single firewire port. 

1. Choose digital input settings.  For all PSMs, the defaults shouldn't be modified unless you are using some digital inputs for a specific application.  For the MTMs, you need to select 'Default Footpedal' for the controller box you plan to connect the foot pedal to.   If you plan to daisy chain multiple controllers, make sure the digital input names are unique.  As a direct consequence, make sure only one of the MTMs is configured with the digital inputs for the foot pedal.

1. Choose drive direction.  You should hit the 'Default' button for all arms.  This multipliers (1 or -1) allow to change the direction of each axis.  This flag is used to make sure that the software uses the axis directions as defined in the Intuitive Research Kit User Manual.  It is important check the direction of each axis for each arm to make sure it corresponds to the documentation using the `sawIntuitiveResearchKitQtPID` or any other applications that allows you to monitor the potentiometers and encoder directions as well as torque directions.  If you find that the default directions don't match your hardware, please let us know. 

1. Generate: click the generate button to generate XML configuration file. 

[[Image(wiki:sawIntuitiveResearchKitTutorial:configGUI_tutorial.png, 1000px)]]


## 2. Config Generator

This configuration generator consists of three files: 
* configGUI.m: GUI logic 
* configGUI.fig: GUI design file
* configGenerator.m: xml file generator with real computation

If you want to know how values in the XML file are computed, please check [source:trunk/saw/applications/sawIntuitiveResearchKit/share configGenerator.m] for more information.