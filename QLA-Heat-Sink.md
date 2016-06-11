# Introduction

The dVRK controllers use a JHU designed Quad Linear Amps board ([acronyms defined in FAQ](/jhu-dvrk/sawIntuitiveResearchKit/wiki/FAQ)).  The linear amplifiers are all mounted against a large heat sink (square aluminum barre).  This solution provides enough heat dissipation for both the MTMs and PSMs since these arms tend to require very little power (for most tasks, less than 0.5 A per board).  For the ECM and the SUJs, passive cooling is not sufficient.  The ECM arm is not as light as the PSMs and require a fair bit more power.  The second joint itself can draw close to 0.5 A continuously.

The QLA boards have multiple 2 pins 12 V connectors that are powered when the motor power is turned on.  We can use these to power a fan for active cooling.

# JHU custom built June 2016 

## Parts

Parts to build 4 heat sinks with fans,
 * Heat Sink: Digikey P/N ATS2192-ND
   http://www.digikey.com/short/3p1qw4, $19.20 (need 2, each is cut in half)
 * Fan: Digikey P/N 259-1578-ND
   http://www.digikey.com/short/3p183v, $6.04 each (need 4)
 * Connector: Digikey P/N A31017-ND
   Qty 10, $0.168 each

## Manufacturing

Each heat sink was first cut in half to match the length of the existing heat sink.  We then drilled 3 holes, matching the spacing on the existing heat sink.  We also drilled 4 holes, diameter 2.5 mm to mount the 40 mm fan.  To mount the fan, we used 3 mm screws, 20 mm long.  The length matches the cumulated thickness of the fan and heatsink:
![Mounting holes](/jhu-dvrk/sawIntuitiveResearchKit/wiki/qla-heat-sink-01.jpg)

To provide enough clearance for all the screws, we had to mill the heat sink fins:
![Milled fins](/jhu-dvrk/sawIntuitiveResearchKit/wiki/qla-heat-sink-02.jpg)

The screws used to mount the heat sink depend on the QLA revision:
 * Rev 1.0, delivered mm/yyyy: imperial 4-40, 3/8 long
 * Rev 2.0, delivered mm/yyyy: ??
 * Rev 3.0, delivered mm/yyyy: ??
 * Rev 4.0, delivered mm/yyyy: imperial 4-40, 3/8 long

To provide enough clearance for all the screws, we had to mill the heat sink fins:
![Heat sink mounted](/jhu-dvrk/sawIntuitiveResearchKit/wiki/qla-heat-sink-03.jpg)

With the fan:
![Fan mounted](/jhu-dvrk/sawIntuitiveResearchKit/wiki/qla-heat-sink-04.jpg)

## Results

| Fan | Time | Temperature |
|-----|------|-------------|
| on  | 0:00 | 26 C        |