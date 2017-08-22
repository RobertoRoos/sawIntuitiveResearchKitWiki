## Codes
* SEMP: SUJ, ECM, MTM, PSM.  Result SELR123
* Q/R: Qt, ROS

## Checks
| test | 1.5 |
|------|-----|
|Compile w/o ROS                | |
|Compile 14.04 + Jade           | |
|Compile 16.04 + Kinetic        | |
|Compile 16.04 + Lunar          | |
|Firmware mix 4/5               | |
|Firmware 6                     | |
|All firewire protocols full dV | |
|Matlab ConfigGenerator (EMP)   | |
|Footpedals event(QR)           | |
|Trigger watchdog (Q)           | |
|Current calibration (EMP)      | |
|Pot calibration (EMP)          | |
|MTM gripper calibration (M)    | |
|Qt PID application             | |
|Home PSM, coupling<br>no adapter, no tool | |
|Home PSM, coupling<br>adapter, no tool    | |
|Home PSM, coupling<br>adapter, tool       | |
|5mm tool<br>Teleop, Python, Matlab        | |
|Teleop PSM over TCP<br>PUT SCRIPT NAME    | |
|Teleop PSM simulated<br>PUT SCRIPT NAME   | |
|ROS Python<br>rosrun dvrk_robot dvrk_mtm_test.py X<br>rosrun dvrk_robot dvrk_arm_test.py X| <br>R<br>R|
|ROS Matlab `dvrk_matlab/test`<br>test_arm_api('X')<br>test_arm_move('X') | <br>L<br>L |