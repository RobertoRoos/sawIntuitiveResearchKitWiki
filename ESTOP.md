<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# 1. General Info

Each controller box contains one or two connectors to support a "safety chain". The basic concept is that the safety chain must be unbroken
(i.e., electrically closed) for the motor power supplies in all of the connected controller boxes to be enabled.
If any device on the safety chain breaks electrical continuity, then all motor power supplies in all connected controller boxes are disabled.
Devices in the safety chain include manually operated switches, such as the e-stop button, and computer-controlled relays, which enable the
system to disable motor power.

The safety chain is connected via one or two connectors on the back of the controller box (depending on the controller box version).
After some iterations, the final design consists of two connectors: a 4-pin connector and a 5-pin connector.
The purpose of this page is to explain how to correctly connect the safety chain, based on this final design.

**As of June 2018, we have developed retrofit kits to convert all existing controller boxes to the new standard. Designs, documentation and images of the retrofit kits can be found in this [GitHub project](https://github.com/jhu-dvrk/dvrk-estop-retrofit) and assembled retrofit kits (i.e., a small PCB with connectors) is being made available to the community at no cost.**

# 2. Overview of Safety Chain

Each controller box contains 3 relays, as shown in Figure 1: 
* 1 relay on each of the two QLA boards 
* 1 relay to control the motor power supply 

To enable the motor power supply, the 12V power needs to be connected to RELAY 3.

![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/estop_relay.jpg) 

Figure 1: Controller box relay and power supply

Figure 1 shows a 4-pin safety connector on the controller box. There have, however, been several variations of safety connectors in the different
generations of controller boxes, starting with one 4-pin connector, then one 5-pin connector, then two 5-pin connectors, and finally one 4-pin and
one 5-pin connector.
In all systems, the connector pinouts are the same. The 5-pin connector is the same as the 4-pin connector except that pin #2 is GND, which shifts
the signals from pins 2-4 on the old (4-pin) connector to pins 3-5 on the new (5-pin) connector.

The advantage of the final design, of one 4-pin and one 5-pin connector is that it enables both a monolithic (hard-wired) e-stop chain, as
implemented at WPI, and a modular (reconfigurable) e-stop chain that was developed at JHU and is currently used on most systems.

# 3. Modular E-stop Chain (recommended)

The modular e-stop chain, also called reconfigurable e-stop chain, consists of three different types of cables:

1. E-Stop Cable: connects the e-stop to the 5-pin connector on one controller box
2. Extension (Daisy-Chain) Cable: connects the 4-pin connector on one box to the 5-pin connector on another box
3. Termination Plug: placed on a 4-pin connector on one of the controller boxes

These are shown in the following image (from the dvrk-estop-retrofit project):

![](https://github.com/jhu-dvrk/dvrk-estop-retrofit/blob/master/docs/dvrk-estop.png)

This design is intended to enable quick reconfiguration of the safety circuit. For example, a complete DVRK setup (4 daisy-chained controller boxes) would have 1 E-Stop Cable, 3 Extension Cables, and 1 Termination Plug. To split this into two separate systems (e.g., MTMR+PSM1 and MTML+PSM2), each system would use 1 E-Stop Cable, 1 Extension Cable, and 1 Termination Plug.

# 4. Monolithic E-stop Chain (not recommended)

The monolithic e-stop chain, previously called the serial e-stop chain, is built to connect to a specific number of controllers. For example,
the figure below shows the connection to two controller boxes (with 4-pin connectors). If the controller box has a 5-pin connector, it
would be better to use that, so that the GND can also be connected. Note that whether using the 4-pin or 5-pin connectors,
the disadvantage of this setup is that the cable would have to be redone to support fewer or more controllers.

![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/daVinci-Estop_2Controllers-1.png)

# 5. A quick note about grounding

It is good practice to connect the GND (ground) on all components of a system. The original controller box, with a 4-pin safety connector, did not include a specific GND connection between controller boxes and therefore relied on the likelihood that the GND would be shared via the AC wiring (e.g., if all controller boxes are plugged in to the same AC circuit).

To correct this deficiency, later versions of the controller box include at least one 5-pin safety connector, where the extra pin is GND. This provides a way to guarantee that GND will be shared between all controller boxes in a system (where the "system" is defined by which boxes are connected via the safety chain).

Since the 4-pin safety connector does not include a GND pin, this GND connection could be obtained by attaching via a screw on the enclosure. See the notes on
the [dvrk-estop-retrofit project](https://github.com/jhu-dvrk/dvrk-estop-retrofit). On the 5-pin safety connector, the GND is available on pin 2.

# 6. Misc.

[Prior (obsolete) e-stop information](/jhu-dvrk/sawIntuitiveResearchKit/wiki/ESTOP-archive).

# 7. Debugging

If you have trouble powering on the motors, please continue reading this section.

**Note: this section was created assuming a 4-pin safety connector and should be updated.**

## 3.1. Test single FPGA-QLA board set (bypassing relays on QLA boards)

As step 1, we want to confirm that the FPGA board, QLA board and power supplies all work. We do this by bypassing the internal relays in the box but keeping the E-STOP in the chain as shown in the next figure. ** For the new safety chain, it should be sufficient to connect the E-Stop Cable and Termination Plug to the
controller box.**
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
