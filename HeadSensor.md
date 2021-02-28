<!--ts-->
   * [Introduction](#introduction)
   * [dVRK Head Sensor](#dvrk-head-sensor)
      * [Hardware](#hardware)
      * [Wiring](#wiring)
      * [Physical setup](#physical-setup)
      * [Software](#software)
   * [daVinci Head Sensor](#davinci-head-sensor)
      * [Wiring](#wiring-1)
      * [Testing with qladisp](#testing-with-qladisp)
      * [Software configuration](#software-configuration)

<!-- Added by: anton, at: 2021-01-28T16:10-05:00 -->

<!--te-->

# Introduction

The real da Vinci system uses a head sensor to detect if the operator is present.  Without the operator, the system will not enable the tele-operation.   For the dVRK, we have used a foot pedal as a dead man switch to detect if the operator is present (usually the "COAG" foot pedal).   This is a reasonable solution for brief experiments but it's not very convenient.   In this page we describe how to either create a "dVRK" head sensor from cheap parts or hack the existing da Vinci head sensor.  The later option requires a full da Vinci system and it's important to note that you will need to keep the plastic back covers off the surgeon's console to switch back and forth between the ISI controllers and dVRK controllers.

# dVRK Head Sensor

## Hardware
 * 1 Digital Distance Sensor 10cm
   * http://www.pololu.com/product/1134
 * 20 Molex pin connectors (Digikey Part No. WM2510-ND)
   * NOTE: we only need 6 pin connectors
 * 12-feet S-Video Cable
    * just get one that is long enough
    * any 3 wire cable would work, we just happened to have an old S-Video cable handy
 * 1 male DB 15 connector (for the connection to the dVRK controller)
 * 1 3-pin right angle connector (for the connection on the sensor side)

## Wiring

| Sensor | Cable | Controller (J18, aka DOF 4) |
|--------|-------|-----------------------------|
| VIN    | Red   | Pin 8 (VCC-CON-A 5V)        |
| GND    | White | Pin 6 (GND)                 |
| OUT    |Yellow | Pin 7 (HOME4)               |

Notes:
* J18 is a 15-pin connector labelled DOF 4 on the back of the dVRK controller
* Please connect head sensor and foot pedal on same controller box

## Physical setup

* Setup option 1: base

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/dvrk-head-sensor-base.jpg)

* Setup option 2: side

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/dvrk-head-sensor-side.jpg)

* Setup option 3: full surgeon's console.  If you have a full da Vinci, you can pull the forehead foam pad and sticj the wire underneath.  Alternatively you can make a custom cable and use the original da Vinci head sensor (see below).

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/dVRK-head-sensor-full-system.jpg)

* Connector for controller box, connect to DOF 4 on controller with the foot pedals (to use default config files)

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/dvrk-head-sensor-connector.jpg)
  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/dvrk-head-sensor-controller.jpg)

## Software
 There is no specific configuration to perform at that point provided that you connect the head sensor on the same controller as the foot pedals.
 * For **rev 1.4** and below, rerun MATLAB XML config generator to make sure the digital input is renamed "HEAD"
 * For **rev 1.5** and above, HEAD is already included in the IO foot pedal XML files
 * Update your JSON config file to set the presence sensor or point to the IO foot pedal configuration file (rev 1.5)
    ```json
    "console-inputs": {
        "operator-present": {
            "component": "io",  // hard coded in source code, file mtsIntuitiveResearchKitConsole.cpp
            "interface": "Head" // name of the button you want to use, defined in sawRobotIO1394 configuration file
        }
    }
    ```

# daVinci Head Sensor

## Wiring

The head sensor is located under the master console's cover.   It has four strobing LEDs on one side and four light sensors on the other side.  It is mounted on the head's sides, behind the 4 round holes forming a diamond shape on each side.

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/daVinci-head-sensor.jpg)

Under the cover, there's a long cable going to the ISI controller at the base of the master console.  There's also a short cable going between the LEDs on one side and the sensors on the other side.  The sensors are hidden behind a metal plate to make sure only the lights from the LEDs can be detected.  It is recommended to leave these alone!

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/daVinci-head-sensor-sensors.jpg)

We found that the easiest solution to connect to the head sensor is to locate the DB 25 cable that connects both the head sensor and the speakers under the surgeon's console.   That connector is located on the back of the console, on the left side, just behind the arms.   You will need to take the side cover off to find it:

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/daVinci-head-sensor-plug.jpg)

You will then need to make a new cable to connect the da Vinci head sensor to the dVRK controllers.  It will be a DB 25 female on the head sensor's end and a high density HD 15 (aka DB 15) male on the controller's end.  The HD 15 male is designed to be connected to the `DOF 1` connector on the back of the dVRK controller (we provide examples of configuration files for the head sensor connected to `DOF 1`).  The wiring pin out is provided in the following formats
 * [PDF](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/daVinci-head-sensor-DB-25-to-DB-15.pdf)
 * [ods](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/daVinci-head-sensor-DB-25-to-DB-15.ods)

Once you have build your custom cable, you can connect it to the da Vinci head sensor:

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/daVinci-head-sensor-cable.jpg)

## Testing with `qladisp`

The HD-15 connector can be plugged on one of the "DOF" connectors on the back of the dVRK controller.   For the following section, we assume the head sensor is connected to "DOF 1" on a PSM3 controller.   This means that it will be interfaced using the IOs for a the first axis on the first board on the PSM3 controller, i.e. board ID is 10.   To test the head sensor, start `qladisp 10`.  You can test your head sensor on any controller, just replace the `10` by the first board ID in the controller you're using.

Then, one can turn on/off the LEDs using the key '0' to toggle.   The value of `DigOut` in `qladisp` should toggle between `0xF` (off) and `0xE` (on).   Then turned on, motion between the LEDs and the sensors should be displayed in the `Home`, `PosLim` and `NegLim` fields.  When the light is blocked, the values should go up by one (e.g. `0xC` to `0xD` or `0xE` to `0xF`):
  * Sensor 1: `Home`, Bit Id 0
  * Sensor 2: `PosLim`, Bit Id 4
  * Sensor 3: `NegLim`, Bit Id 8

## Software configuration

Assuming that you're connecting your head sensor to the MTMR controller, always on the `DOF 1` connector, you just need to add the following line in your console JSON configuration file:
```json
   "operator-present": {
        "io": "sawRobotIO1394-MTMR-dv-head-sensor.xml"
   }
```

**Important note:** The board numbering on the software side start with index 0 (e.g. in XML IO configuration file) but the labels on the back of the controller start at index 1.  Keep this in mind if you plan to connect the head sensor to another "DOF" and create your own XML configuration files.