<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

- [Introduction](#introduction)
- [dVRK Head Sensor](#dvrk-head-sensor)
  - [Hardware](#hardware)
  - [Wiring](#wiring)
  - [Physical setup](#physical-setup)
  - [Software](#software)
- [daVinci Head Sensor](#davinci-head-sensor)
  - [Wiring](#wiring-1)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# Introduction

The real da Vinci system uses a head sensor to detect if the operator is present.  Without the operator, the system will not enable the tele-operation.   For the dVRK, we have used a foot pedal as a dead man switch to detect if the operator is present (usually the "COAG" foot pedal).   This is a reasonable solution for brief experiments but it's not very convenient.   In this page we describe how to either create a "dVRK" head sensor from cheap parts or hack the existing da Vinci head sensor.  The later option requires a full da Vinci system and it's important to note that it's not trivial to switch back and forth between the ISI controllers and dVRK controllers since we're disconnecting the ISI controller underneath the head cover.

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

| Sensor | Cable | Controller (J18)     |
|--------|-------|----------------------|
| VIN    | Red   | Pin 8 (VCC-CON-A 5V) |
| GND    | White | Pin 6 (GND)          |
| OUT    |Yellow | Pin 7 (HOME4)        |

Notes:
* J18 is a 15-pin connector labelled DOF 4 on the back of the dVRK controller
* Please connect head sensor and foot pedal on same controller box

## Physical setup

* Setup option 1: base

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/dvrk-head-sensor-base.jpg)

* Setup option 2: side

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/dvrk-head-sensor-side.jpg)  

* Connector for controller box, connect to DOF 4 on controller with the foot pedals (to use default config files)

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/dvrk-head-sensor-connector.jpg)
  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/dvrk-head-sensor-controller.jpg)

## Software
 * For rev 1.4 and below, rerun MATLAB XML config generator to make sure the digital input is renamed "HEAD"
 * For rev 1.5 and above, HEAD is already included in the IO foot pedal XML files
 * Update JSON config file to set the presence sensor or point to the IO foot pedal configuration file (rev 1.5)

# daVinci Head Sensor

## Wiring

The head sensor is located under the master console's cover.   It has four strobing LEDs on one side and four light sensors on the other side.

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/daVinci-head-sensor.jpg)

Under the cover, there's a single connector going to the controller and a cable going between the LEDs on one side and the sensors on the other side.

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/daVinci-head-sensor-cable.jpg)

The sensors are hidden behind a metal plate.

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/daVinci-head-sensor-sensors.jpg)
