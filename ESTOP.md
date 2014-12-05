<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

- [1. General Info](#1-general-info)
- [2. E-STOP Setup](#2-e-stop-setup)
  - [2.1. Serial Connection (WPI)](#21-serial-connection-wpi)
  - [2.2. Configurable Serial Connection (JHU)](#22-configurable-serial-connection-jhu)
  - [2.3. Parallel Connection (Not Recommended)](#23-parallel-connection-not-recommended)
- [3. Debugging](#3-debugging)
  - [3.1. Test single FPGA-QLA board set (bypassing relays on QLA boards)](#31-test-single-fpga-qla-board-set-bypassing-relays-on-qla-boards)
  - [3.2. Test single controller box with QLA relays in the loop](#32-test-single-controller-box-with-qla-relays-in-the-loop)
  - [3.3. Test full system](#33-test-full-system)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# 1. General Info

Each controller box contains 3 relays, as shown in Figure 1: 
* 1 relay on each of the two QLA boards 
* 1 relay to control the motor power supply 

**IMPORTANT**: Later versions of the controller box have a 5-pin safety connector, rather than the 4-pin connector shown here.  The pinout is the same, except that pin #2 is GND, which shifts the signals from pins 2-4 on the old (4-pin) connector to pins 3-5 on the new (5-pin) connector.

To enable the motor power supply, the 12V power needs to be connected to RELAY 3. Relay and E-Stop are two main components in the system's safety module. 
* The E-STOP provides a mechanism for the operator to shut down the system when the software or FPGA firmware fails. 
* Relays controlled by the FPGA are serially chained, which will shut down the motor power if the FPGA power system goes down. 

![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/estop_relay.jpg) 

Figure 1: Controller box relay and power supply


# 2. E-STOP Setup

## 2.1. Serial Connection (WPI)
When you receive your controllers, this should be your default setup. Each e-stop cable is made to connect two controller boxes (e.g., 1 MTM and 1 PSM). The cable has to be redone to support fewer or more controllers.

![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/daVinci-Estop_2Controllers-1.png)

## 2.2. Configurable Serial Connection (JHU)

The single safety connector (on each box) is brought out to two 4-pin safety connectors, as shown in the diagram below. There are three different types of cables:
* E-Stop Cable: connects the e-stop button to pins 1 and 2; there should only be one of these cables in a single setup
* Extension Cable: connects the safety circuit between two boxes; there can be any number of these cables
* Termination Plug: connects pins 2 and 3; there should only be one of these plugs in a single setup

This design is intended to enable quick reconfiguration of the safety circuit. For example, a complete DVRK setup (4 daisy-chained controller boxes) would have 1 E-Stop Cable, 3 Extension Cables, and 1 Termination Plug. To split this into two separate systems (e.g., MTMR+PSM1 and MTML+PSM2), each system would use 1 E-Stop Cable, 1 Extension Cable, and 1 Termination Plug.

Note that the GND connection is not needed for the e-stop functionality, but it is included because it is a good way to make sure that all controller boxes share a common GND. Since the 4-pin safety connector does not include a GND pin, this GND connection could be obtained by attaching to a screw on the enclosure.  On the 5-pin safety connector, the GND is available on pin 2.

We have implemented this at JHU using mini-DIN connectors.  The female connectors are Digikey P/N CP-2140-ND (free hanging) or CP-2540-ND (panel mount). The male connector is Digikey P/N CP-2040-ND (free hanging). For the extension cable, it is possible to use an S-Video cable IF the pin carrying the 12V is cut (pin 1 in the drawing below). Otherwise, you would also connect the 12V supplies together, as in the Parallel Connection below, though at least the current would not flow through the safety relays.

![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/estop_proposed.png)

**Implementation Guide**

This guide documents the steps to implement this configurable serial connection using S-Video (Mini-DIN-4) connector and S-Video cables.

**List of Materials:** (To make E-STOP wiring for N boxes)
 * 1 E-STOP 
 * 2 male connector (Digikey P/N CP-2040-ND)
   * 1 for E-STOP
   * 1 for termination plug
 * 2 x N female connectors (Digikey P/N CP-2140-ND)
 * N-1 S-Video cables 
 * (2 + 2 x N) x 4 Molex pin connectors (Digikey Part No. WM2510-ND)

Example: a 4-box system: 
 * 1 E-STOP
 * 2 male connector (Digikey P/N CP-2040-ND)
 * 8 female connectors (Digikey P/N CP-2140-ND) 
 * 3 S-Video cables
 * 40 Molex pin connectors (Digikey Part No. WM2510-ND)
NOTE: we do recommend you order 2 more male and 2 more female connectors in case you broke one and their unit price is around $1.50 in small quantity. 

**WARNING:** the pinout for male and female connectors are different! 

* Step 1: E-STOP. Solder the two pins of E-STOP to pin 1 and pin 2 of a male connector, respectively. 
* Step 2: Termination Plug.  Short pin 2 and pin 3 of a male connector. 
* Step 3: Female Connector. Solder pins according to the schematic. Wire colors: 
 * Pin 1 - RED  12 V 
 * Pin 2 - YELLOW  S
 * Pin 3 - GREEN  EN
 * Pin 4 - BLACK GND

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/estop_svideo_0.jpg)
  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/estop_svideo_1.jpg)
  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/estop_svideo_2.jpg)
  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/estop_svideo_3.jpg)
  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/estop_svideo_5.jpg)

 * Step 4: S-Video Cable. Modification on the S-Video cable is required to prevent connecting 12 V power supply. Bend pin 1 at both ends of the S-Video cable, as shown in the following figure. 

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/estop_svideo_cable.jpg)

Reference: S-Video Pinout:

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/estop_svideo_male.png)

Male connector

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/estop_svideo_female.png)

Female connector


## 2.3. Parallel Connection (Not Recommended)

The advantage is that the connection is parallel, so it can be used for any number of controller boxes. For example, the figure below shows an e-stop cable connected to two controller boxes. But, you can also use it to control just one controller box (i.e., it still works if the second connector is not used).  But, there is a **major drawback** to this wiring strategy.
The drawback is that the 12V power supplies in the boxes will all be connected together when the QLA safety relays are closed.  It is generally not a good practice to connect power supplies, since the power supply with the highest output voltage will source all the current, until the high load current causes its output voltage to drop below the output of one of the other power supplies (note that the load current will be high because the 12V supply also provides the logic power for the FPGA boards).  Even worse, the current that flows from one box to the others all flows through the contacts of the QLA safety relay. If this current is too high, it will permanently weld the relay contacts, destroying the relay (which is not easy to replace). This has actually happened at JHU, destroying the relays on 9 QLA boards.  At JHU, the problem was exacerbated by the fact that one of the "12V" power supplies in the controller box was actually a 24V supply, so when the safety relays were closed, this power supply provided current to all controllers (thereby powering all FPGAs). But, this problem could potentially happen in systems that only have 12V supplies, since their output voltages will not be exactly equal. It is easy to test whether the relay contacts have been fused by checking continuity (it should be normally-open, so if there is continuity when the system is powered off, it has been fused).

![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/estop_current.jpg)

# 3. Debugging

If you have trouble powering on the motors, please continue reading this section. 

## 3.1. Test single FPGA-QLA board set (bypassing relays on QLA boards)

As step 1, we want to confirm that the FPGA board, QLA board and power supplies all work. We do this by bypassing the internal relays in the box but keeping the E-STOP in the chain as shown in the next figure.
Connect the modified connector to the controller box you want to debug and run the ''qladisp'' program: 

```bash
 # assume we are testing MTML box 
 $ qladisp 0 1  

 # Press 'p' to turn on power
 #   - 'p' first turns on the QLA relays (this step does NOT matter, since those relays are bypassed)
 #   - then turns on board and amplifier power
 # The mv-good and all amplifiers should be turned on at this time 
 #   - if not, check the power system physical connections
```

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/estop_bypass_one.jpg)


## 3.2. Test single controller box with QLA relays in the loop

After confirming that the power system is working, we start to add relays inside one controller box to the chain. Modify the connector as indicated in the following figure. 

```bash
 # assume we are testing MTML box
 # relays are serial chained, so connect to 0 and 1 at the same time  
 $ qladisp 0 1

 # Press 'p' to turn on power
 #   - 'p' first turns on the QLA relays
 #   - then turns on board and amplifier power
 # The mv-good and all amplifiers should be turned on at this time
 #   - if not, check the DEBUG section
```

![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/estop_one.jpg)

**DEBUG**  

I'm sorry you are reading this section, but we need to figure it out. You will need a multimeter to debug. 

The very first step is to check if the relay on the QLA board is working. As shown in the following figure, there are two test points (T1, T2). NOTE, the relay might look different depending on your hardware revision. The connection between the two points is designed to be open when the relay is turned OFF and shorted when the relay is ON. 

Assume we test board 0 first:
1. Do a continuity test between T1 and T2. It should be open; if not, contact us. (No power)
1. Turn on relay now 
 * `$ qladisp 0` 
 * Press 'p' to turn on relay 
 * You should also hear a click sound from the relay
1. Do a continuity test between T1 and T2. Now they should be shorted; if not, contact us. 
REPEAT the same process for board 1. 

![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/estop_relay_debug.png)

Now, you have two working relays. Please check the wire connection, make sure:
1. they are serially connected
2. the connection to the E-STOP terminal is correct. 

The next step is to test them together. The middle two pins (P1 and P2) of the E-STOP terminal are connected to the two relays. If the system is working, they should be open while the relays are OFF and shorted while the relays are ON. 

Assume we are testing MTML board 0 and 1:
1. Do a continuity test between P1 and P2. Should be open. 
1. Turn on relay now 
 * `$ qladisp 0 1`
 * Press 'p' to turn on relay 
 * You should also hear a click sound from the relay
1. Do a continuity test between P1 and P2. Now they should be shorted; if not, check the wire connection. 

Finally, do the test in section 3.2. You should be good to go. 


## 3.3. Test full system

At this point, we are sure that the relay and power system work properly. What's left is to make your connector based on your setup. The next figure shows a setup for two controller boxes (from WPI). 

NOTE: when you receive your controller box, you might also have received 4 setup diagrams including 
* 2 controller boxes
* 4 controller boxes
* 2 controller boxes bypassing internal relays
* 4 controller boxes bypassing internal relays 

![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/daVinci-Estop_2Controllers-1.png) ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/daVinci-Estop_4Controllers-1.png)
