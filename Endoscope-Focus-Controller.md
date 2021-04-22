<!--ts-->
   * [Introduction](#introduction)
   * [da Vinci focus controller](#da-vinci-focus-controller)
   * [dVRK focus controller](#dvrk-focus-controller)

<!-- Added by: adeguet1, at: 2019-08-06T12:19-04:00 -->

<!--te-->

# Introduction

Acronyms used in this page are defined in [Frequently Asked Questions](/jhu-dvrk/sawIntuitiveResearchKit/wiki/FAQ).

This page describes how to use the endoscopic camera focus with the dVRK.  This is useful only for the groups with an Intuitive Surgical camera (see also [Video Pipeline](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Video-Pipeline)).

We offer two different ways to control the camera focus with the dVRK:
* **da Vinci focus controller**: Use the original Intuitive Surgical da Vinci focus controller using the dVRK arm controller to trigger the +/- digital inputs from the foot pedals.  This is the simplest approach and it replicates the features of the clinical system. 
* **dVRK focus controller**: bypass the original controller and use the dVRK arm controller to control the motor in the camera stereo head.  This requires a more complex adapter but gives access to more information (e.g. encoder position of the focus stage). 

# daVinci focus controller

The goal of this section is to describe how to control the camera focus using the foot pedals through the dVRK controllers/software.

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/focus/camera-focus-front.jpg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/focus/camera-focus-front.jpg" width="350"></a>

* da Vinci focus controller
  * Back of endoscope focus controller, male DSUB 15 pins
  * Focus +, short pin 1 with pin 9, power comes from pin 1 (floating high, 5V)
  * Focus -, short pin 4 with pin 9, power comes from pin 4 (floating high, 5V)

* dVRK controller
  * Using digital outs on first QLA FPGA
  * Digital out 3 connected to DB 15 on `DOF 1`:
     * Pin 14: on: 0V, off: 5V.   We should use this pin for focus + control, off by default
     * Pin 10 is grounded
  * Digital out 2 connected to DB 15 on `DOF 2`:
     * Pin 14: on: 0V, off: 5V.   We should use this pin for focus - control, off by default 
     * Pin 10 is grounded

* Cable wiring
  * The cable is "Y" shaped, on the dVRK controller side you will need two male DB 15 connectors (3 rows of 5 pins) which will plug in the connector labeled `DOF 1` and `DOF 2` on the back of the dVRK controller.  On the ISI focus controller side, you need a single male DB 15 connector (1 row of 8 pins, 1 row of 7 pins).
  * You need 3 wires in your cable:
    * Focus + signal: dMIB `DOF 1` pin 14 <-> Focus controller pin 4
    * Focus - signal: dMIB `DOF 2` pin 14 <-> Focus controller pin 1
    * Ground: dMIB `DOF 1` pin 10 and/or `DOF 2` pin 10 <-> Focus controller pin 9 (dMIB share ground between `DOF 1` and `DOF2`)

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/focus/dvrk-focus-control-cable.jpg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/focus/dvrk-focus-control-cable.jpg" width="350"></a>
<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/focus/camera-focus-back.jpg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/focus/camera-focus-back.jpg" width="350"></a>

Once you've build your cable, you can modify your console JSON configuration file and add:
```json
    "endoscope-focus": {
        "io": "sawRobotIO1394-MTML-dv-endoscope-focus.xml"
    }
```
The example above assumes that:
 * You're using the dVRK software **rev 1.6** or above
 * You have connected to the camera focus cable to the MTML controller.  If you're connecting the endoscope focus unit to another controller and you can't find the corresponding configuration file in `sawIntuitiveResearchKit/share/io`, feel free to create one and contribute it back to the community
 * You have the da Vinci foot pedal connected with the Camera +/- toggle pedal properly working.  You can check in the Qt graphical user interface, under the IO/Buttons tab.

At that point, you should be able to control the camera focus using the foot pedals.  When pressing the +/- pedal you should:
  * See the focus change
  * See the "Focus In"/"Focus Out" LED turn on/off on the vision cart
  * Hear the motor on the camera head

# dVRK focus controller

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/focus/dvrk-focus-controller-adapter.png"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/focus/dvrk-focus-controller-adapter.png" width="350"></a>
