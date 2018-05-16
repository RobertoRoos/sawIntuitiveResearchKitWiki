## Codes
* SEMP: SUJ, ECM, MTM, PSM.  Result SELR123
* Q/R: Qt, ROS

## Checks
| test | 1.5 | 1.6 |
|------|-----|-----|
|Compile w/o ROS                |?|Y|
|Compile 14.04 + Jade           |Y|?|
|Compile 16.04 + Kinetic        |Y|Y|
|Compile 16.04 + Lunar          |Y|?|
|Compile 17.04 + Lunar          | |Y|
|Compile 18.04 + Melodic        | |Y|
|Firmware mix 4/5               |Y|Y|
|Firmware 6                     |N|Y|
|All firewire protocols full dV |Not full broadcast|Not full broadcast|
|Matlab ConfigGenerator (EMP)   |EMP|EMP|
|Current calibration (EMP)      |SEMP|SEMP|
|Pot calibration (EMP)          |EMP|EMP|
|MTM gripper calibration (M)    |M|M|
|Footpedals event(QR)           |QR|QR|
|Trigger watchdog (Q)           |QR|Q|
|Qt PID application             |EMP|
|Home PSM, coupling<br>no adapter, no tool |Y|Y|
|Home PSM, coupling<br>adapter, no tool    |Y|Y|
|Home PSM, coupling<br>adapter, tool       |Y|Y|
|5mm tool<br>Teleop, Python, Matlab        |Y|
|Teleop PSM over UDP socket<br>`console-MTMR-PSM1SocketClient-Teleop.json`<br>`console-PSM1SocketServer.json`|Y|
|Teleop PSM simulated<br>console-MTMR-PSM1_KIN_SIMULATED-Teleop.json|Y|
|ROS Python<br>`rosrun dvrk_robot dvrk_mtm_test.py X`<br>`rosrun dvrk_robot dvrk_arm_test.py X`| <br>R<br>R|
|ROS Matlab `dvrk_matlab/test`<br>`test_arm_api('X')`<br>`test_arm_move('X')`| <br>L<br>L |