## Controllers

| Board | I/O  | MTM | PSM | ECM | SUJ/dSIB | dMIB pin |
|-------|------|-----|-----|-----|----|----------|
| 1 | I:0  | _daVinci Head_ | SUJ Clutch | Manip Clutch | Clutch 1 | DOF1:7 |
| 1 | I:1  | | | | Clutch 2 | |
| 1 | I:2  | | Manip clutch | SUJ Clutch | Clutch 3 | |
| 1 | I:3  | _dVRK head_ | | | Clutch 4 | DOF4:7 |
| 1 | I:4  | _daVinci Head_ | | | MotorUp | | | DOF1:5 |
| 1 | I:5  | | | | MotorDown | |
| 1 | I:6  | | | | |
| 1 | I:7  | | | | |
| 1 | I:8  | _daVinci Head_ | | | | DOF1:3 |
| 1 | I:9  | | | | |
| 1 | I:10 | | | | |
| 1 | I:11 | | | | |
| 1 | O:0  | _Endoscope_<br>Or _daVinci Head_ | | | NoMuxReset | DOF1:14 |
| 1 | O:1  | _Endoscope_ | | | MuxIncrement | DOF2:14 |
| 1 | O:2  | | | | ControlPWM | DOF3:14 |
| 1 | O:3  | | | | DisablePWM | DOF4:14 |
| 2 | I:0  | _Foot pedal_ | | | | FP:5/7 |
| 2 | I:1  | _Foot pedal_ | | | | FP:12/13 |
| 2 | I:2  | _Foot pedal_ | | | | FP:10/11 |
| 2 | I:3  | _Foot pedal_ | | | | FP: 1/2|
| 2 | I:4  | _Foot pedal_ | | | | FP: 6/8|
| 2 | I:5  | _Foot pedal_ | | | | FP: 4/3|
| 2 | I:6  | | | | |
| 2 | I:7  | | Tool | | | |
| 2 | I:8  | | | | |
| 2 | I:9  | | | | |
| 2 | I:10 | | Adaptor | | | |
| 2 | I:11 | | | | |
| 1 | O:0  | | | | | DOF5:14 |
| 1 | O:1  | | | | | DOF6:14 |
| 1 | O:2  | | | | | DOF7:14 |
| 1 | O:3  | | | | | DOF8:14 |

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
| 1     | I:12  | HeadSensor4 |
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