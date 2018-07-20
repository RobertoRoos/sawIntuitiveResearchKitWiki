## Controllers

In the table below:
* **Board**: which FPGA/QLA board set
* **I/O**:  whether Input (I) or Output (O), followed by bit number
* **QLA**:  signal name on QLA schematics
* **MTM**:  use of this I/O by MTM controller
* **PSM**:  use of this I/O by PSM controller
* **ECM**:  use of this I/O by ECM controller
* **SUJ/dSIB**: use of this I/O for setup joint controller
* **dMIB pin**: which connector (DOF1-DOF8 or FP) and pin number (after :). Note that DOF1-DOF7 are HD15 connectors, DOF8 is a HD26 connector, and FP (footpedal) is a DB15 connector.

| Board | I/O  |  QLA    | MTM | PSM | ECM | SUJ/dSIB | dMIB pin |
|-------|------|---------|-----|-----|-----|----------|----------|
| 1     | I:0  | HOME1   | _daVinci Head_ | SUJ Clutch | Manip Clutch | Clutch 1 | DOF1:7 |
| 1     | I:1  | HOME2   | | | | Clutch 2 | DOF2:7 |
| 1     | I:2  | HOME3   | | Manip clutch | SUJ Clutch | Clutch 3 | DOF3:7 |
| 1     | I:3  | HOME4   | _dVRK head_ | | | Clutch 4 | DOF4:7 |
| 1     | I:4  | POSLIM1 | _daVinci Head_ | | | MotorUp | DOF1:3 |
| 1     | I:5  | POSLIM2 | | | | MotorDown | DOF2:3 |
| 1     | I:6  | POSLIM3 | | | | | DOF3:3 |
| 1     | I:7  | POSLIM4 | | | | | DOF4:3 |
| 1     | I:8  | NEGLIM1 | _daVinci Head_ | | | | DOF1:5 |
| 1     | I:9  | NEGLIM2 | | | | | DOF2:5 |
| 1     | I:10 | NEGLIM3 | | | | | DOF3:5 |
| 1     | I:11 | NEGLIM4 | | | | | DOF4:5 |
| 1     | O:0  | DOUT1   | _Endoscope_<br>Or _daVinci Head_ | | | NoMuxReset | DOF1:14 |
| 1     | O:1  | DOUT2   | _Endoscope_ | | | MuxIncrement | DOF2:14 |
| 1     | O:2  | DOUT3   | | | | ControlPWM | DOF3:14 |
| 1     | O:3  | DOUT4   | | | | DisablePWM | DOF4:14 |
| 2     | I:0  | HOME1   | _Foot pedal_ | | | | DOF5:7, FP:5/7 |
| 2     | I:1  | HOME2   | _Foot pedal_ | | | | DOF6:7, FP:12/13 |
| 2     | I:2  | HOME3   | _Foot pedal_ | | | | DOF7:7, FP:10/11 |
| 2     | I:3  | HOME4   | _Foot pedal_ | | | | FP:1/2 |
| 2     | I:4  | POSLIM1 | _Foot pedal_ | | | | DOF5:3, FP:6/8 |
| 2     | I:5  | POSLIM2 | _Foot pedal_ | | | | DOF6:3, FP:4/3 |
| 2     | I:6  | POSLIM3 | | | | | DOF7:3 |
| 2     | I:7  | POSLIM4 | | Tool | | | |
| 2     | I:8  | NEGLIM1 | | | | | DOF5:5 |
| 2     | I:9  | NEGLIM2 | | | | | DOF6:5 |
| 2     | I:10 | NEGLIM3 | | Adaptor | | | DOF7:5 |
| 2     | I:11 | NEGLIM4 | | | | | |
| 2     | O:0  | DOUT1   | | | | | DOF5:14 |
| 2     | O:1  | DOUT2   | | | | | DOF6:14 |
| 2     | O:2  | DOUT3   | | | | | DOF7:14 |
| 2     | O:3  | DOUT4   | | | | | DOF8:16 |

## External devices

### Foot pedals

| Board | I/O  | Function |
|-------|------|----------|
| 2     | I:0  | Clutch |
| 2     | I:1  | Cam- |
| 2     | I:2  | Cam+ |
| 2     | I:3  | Coag |
| 2     | I:4  | Camera |
| 2     | I:5  | BiCoag |

### da Vinci head sensor

| Board | I/O  | Function |
|-------|------|----------|
| 1     | I:0  | HeadSensor1 |
| 1     | I:4  | HeadSensor2 |
| 1     | I:8  | HeadSensor3 |
| 1     | I:12 | HeadSensor4 |
| 1     | O:0  | HeadSensorTurnOff |

### dVRK head sensor

| Board | I/O  | Function |
|-------|------|----------|
| 1     | I:3  | Head |

### Endoscope focus controller

| Board | I/O  | Function |
|-------|------|----------|
| 1     | O:0  | EndoscopeFocusIn |
| 1     | O:1  | EndoscopeFocusOut |
