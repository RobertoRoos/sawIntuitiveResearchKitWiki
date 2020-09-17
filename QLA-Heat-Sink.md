<!--ts-->
   * [Introduction](#introduction)
   * [Power source for the fans](#power-source-for-the-fans)
   * [Mounting screws](#mounting-screws)
   * [Recommended solution](#recommended-solution)
   * [Custom built June 2016](#custom-built-june-2016)
      * [Parts](#parts)
      * [Manufacturing](#manufacturing)
      * [Results](#results) 
<!-- Added by: anton, at:  -->
<!--te-->

# Introduction

The dVRK controllers use a JHU designed Quad Linear Amps board ([acronyms defined in FAQ](/jhu-dvrk/sawIntuitiveResearchKit/wiki/FAQ)).  The linear amplifiers are all mounted against a large passive heat sink (square aluminum bar).  This solution provides some heat dissipation but this is not enough for the dVRK arms.

For the ECM and the SUJs, **passive cooling is definitely not enough**.  Make sure you have upgraded your controllers with an extra heat sink and fan.  The ECM arm is not as light as the PSMs and requires a fair bit more power.  The second joint itself can draw close to 0.5 A continuously.

The MTMs and PSMs can run for a little while (half hour or so) without any issues but for prolonged usage, we strongly recommend to upgrade your controllers with a an extra heat sink and fan.

# Power source for the fans

The QLA boards have multiple 2 pins 12 V connectors that are powered when the motor power is turned on.  We can use these to power a fan for active cooling.

# Mounting screws

The screws used to mount the heat sink depend on the QLA serial number. To the best of our knowledge, although the older heat sink drawing indicates metric (M3) threads, we believe most were tapped for Imperial 4-40. The exception is build #3, with QLA S/N 4265-xx.
 * QLA S/N 4265-xx: Metric M3, 8 mm long
 * All other QLA: Imperial 4-40, 3/8" long

# Recommended solution

We found that a single Pentium Socket 370 heat sink and fan performs very well.  There is very little manufacturing required so users can likely upgrade their controllers by themselves.   We uses the StarTech FANP1003LD (https://www.amazon.com/dp/B000IXS6DW) with success but other heatsinks for Socket 370 might work as well.

![Socket 370 fan](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/qla-heat-sink-socket-370/qla-heat-sink-fan.jpg)

If you decide or need to get a different socket 370 heat sink and fan assembly, make sure there's a "valley" wide enough in the middle to drill a hole and accommodate a screw head.  You will need to drill a single hole in the middle of the heat sink.

![Heat sink with hole](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/qla-heat-sink-socket-370/qla-heat-sink-hole.jpg)

Once you've drilled the heat sink and located the screw to attach it, apply some thermal paste between the new heat sink and the QLA heat sink:

![Heat sink mounted](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/qla-heat-sink-socket-370/qla-heat-sink-mounted.jpg)

The plug that comes with the StarTech fans is **not compatible with the dVRK QLA power plugs**!  You must reconfigure it to match the QLA wiring.  The pins in the white housing (fan plug) can be removed, you need to gently pull on the wire while pushing on the little metal tab on the side of the plug.  Once you've pulled all the pins, you can cut the yellow wire and re-insert the other two pins (black and red) to match the QLA.  The QLA power plugs are labeled with "12V" and "GND".  You must reconfigure the fan's plug so the black wire goes to "GND" and the red wire goes to "12V".  Make sure your wiring matches the photos below, i.e. the plug is not symmetrical.

![Connector](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/qla-heat-sink-socket-370/qla-heat-sink-connector-a.jpg)

![Connector](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/qla-heat-sink-socket-370/qla-heat-sink-connector-b.jpg)

You can then screw the fan back on top of the heat sink and plug it.  There are three 12V connectors on the QLA, all equivalent.  Note that one of them might already be used for the whole controller case fan.  The fan's plug still has room for 3 pins while the QLA plugs only have two pins so there are two physical ways to plug the fans.   Make sure you plug it so the two wires (black and red) and aligned with the QLA plug.

![12V connectors on a QLA](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/qla-heat-sink-socket-370/qla-heat-sink-board-power.jpg)

![All set](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/qla-heat-sink-socket-370/qla-heat-sink-mounted.jpg)

# Custom built June 2016 

This section is provided for your information only, this is mostly historical.  We don't recommend this approach anymore, the socket 370 heat sink and fan are much easier to implement.

## Parts

Parts to build 4 heat sinks with fans,
 * Heat Sink: Digikey P/N ATS2192-ND
   http://www.digikey.com/short/3p1qw4, $19.20 (need 2, each is cut in half)
 * Fan: Digikey P/N 259-1578-ND
   http://www.digikey.com/short/3p183v, $6.04 each (need 4)
 * Connector: Digikey P/N A31017-ND
   Qty 10, $0.168 each

## Manufacturing

Each heat sink was first cut in half to match the length of the existing heat sink.  We then drilled 3 holes, matching the spacing on the existing heat sink.  We also drilled 4 holes, diameter 2.5 mm to mount the 40 mm fan.  To mount the fan, we used 3 mm screws, 20 mm long.  The length matches the cumulative thickness of the fan and heatsink:

![Mounting holes](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/qla-heat-sink-2006/qla-heat-sink-01.jpg)

To provide enough clearance for all the screws, we had to mill the heat sink fins:

![Milled fins](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/qla-heat-sink-2006/qla-heat-sink-02.jpg)

![Heat sink mounted](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/qla-heat-sink-2006/qla-heat-sink-03.jpg)

With the fan:

![Fan mounted](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/qla-heat-sink-2006/qla-heat-sink-04.jpg)

## Results

This was tested on the SUJ controller, dSIB rev 2.0.

| Fan | Time | Temperature |
|-----|------|-------------|
| off | 0:00 | 29 C (room) |
| off | 0:15 | 44 C        |
| off | 0:30 | 50 C        |
| on  | 0:40 | 46 C        |
| on  | 0:45 | 37 C        |
| on  | 0:50 | 34 C        |

Without the fan the temperature seems stable around 50 C.  With the fan, the temperature goes down to 34 C.
