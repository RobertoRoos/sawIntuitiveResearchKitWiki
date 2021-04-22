
Testing IOs and scale:
```sh 
sawRobotIO1394QtConsole -c jhu-dVRK/sawRobotIO1394-PSM1-focus-controller-28007.xml
```

To figure out encoder ratio, we used a broken camera head and physically measured the displacement.  Number are provided in spreadsheet.

Testing and tuning PID:
```sh
sawIntuitiveResearchKitQtPID -i jhu-dVRK/sawRobotIO1394-PSM1-focus-controller-28007.xml -p pid/sawControllersPID-dVRK-focus-controller.xml -n Focus
```

For the PID gains, we use a high D gain to make sure the focus doesn't change too fast if the setpoint is accidentally set too far.  We also need a small PID tracking error and make sure most commands use `move_jp` or `move_jr` but no `servo_jp` nor `servo_jf`.  `servo_jp` might be used internally.

