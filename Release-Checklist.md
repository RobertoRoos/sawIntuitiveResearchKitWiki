## Codes
* SEMP: SUJ, ECM, MTM, PSM.  Result SELR123
* Q/R: Qt, ROS

## Checks
| test | 1.5 |
|------|-----|
|Compile w/o ROS                |?|
|Compile 14.04 + Jade           |Y|
|Compile 16.04 + Kinetic        |Y|
|Compile 16.04 + Lunar          |Y|
|Firmware mix 4/5               |Y|
|Firmware 6                     |N|
|All firewire protocols full dV |Not full broadcast|
|Matlab ConfigGenerator (EMP)   |EMP|
|Footpedals event(QR)           |QR|
|Trigger watchdog (Q)           |QR|
|Current calibration (EMP)      |SEMP|
|Pot calibration (EMP)          |EMP|
|MTM gripper calibration (M)    |M|
|Qt PID application             |EMP|
|Home PSM, coupling<br>no adapter, no tool |Y|
|Home PSM, coupling<br>adapter, no tool    |Y|
|Home PSM, coupling<br>adapter, tool       |Y|
|5mm tool<br>Teleop, Python, Matlab        |Y|
|Teleop PSM over UDP socket<br>`console-MTMR-PSM1SocketClient-Teleop.json`<br>`console-PSM1SocketServer.json`|Y|
|Teleop PSM simulated<br>console-MTMR-PSM1_KIN_SIMULATED-Teleop.json|Y|
|ROS Python<br>`rosrun dvrk_robot dvrk_mtm_test.py X`<br>`rosrun dvrk_robot dvrk_arm_test.py X`| <br>R<br>R|
|ROS Matlab `dvrk_matlab/test`<br>`test_arm_api('X')`<br>`test_arm_move('X')`| <br>L<br>L |