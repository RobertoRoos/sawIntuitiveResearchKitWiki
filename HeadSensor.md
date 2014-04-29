# 1. General Info
The page documents how to make a head sensor for da Vinci Research Kit.  The goal is to detect the presence of the operator at the master console using a proximity sensor mounted on the stereo display.

# 2. Hardware

 * 1 Digital Distance Sensor 10cm
   * http://www.pololu.com/product/1134
 * 20 Molex pin connectors (Digikey Part No. WM2510-ND) 
   * NOTE: we only need 6 pin connectors
 * 12-feet S-Video Cable 
    * just get one that is long enough
    * any 3 wire cable would work, we just happened to have an old S-Video cable handy 
 * 1 3-pin straight connector (for the connection to the dVRK controller)
 * 1 3-pin right angle connector (for the connection on the sensor side)

# 3. Wiring

| Senor | Cable | Controller (J18)     |
| VIN   | Red   | Pin 8 (VCC-CON-A 5V) |
| GND   | White | Pin 6 (GND)          |
| OUT   |Yellow | Pin 7 (HOME4)        |

Notes:
* J18 is a 15-pin connector labelled DOF 4 on the back of the dVRK controller
* Please connect head sensor and foot pedal on same controller box

# 4. Setup

[[Image(wiki:sawIntuitiveResearchKitTutorial/HeadSensor:dvrk-head-sensor-base.jpg, 300px)]]
Setup option 1: base

[[Image(wiki:sawIntuitiveResearchKitTutorial/HeadSensor:dvrk-head-sensor-side.jpg, 300px)]]  
Setup option 2: side

[[Image(wiki:sawIntuitiveResearchKitTutorial/HeadSensor:dvrk-head-sensor-controller.jpg, 300px)]]
Connection to controller box

#. 5 Software
 * Rerun MATLAB XML config generator to make sure the digital input is renamed "HEAD"
 * Update JSON config file to set the presence sensor