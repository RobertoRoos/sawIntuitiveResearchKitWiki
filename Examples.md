<!--ts-->
   * [How to run the examples](#how-to-run-the-examples)
   * [Python Test Programs](#python-test-programs)
   * [sawRobotIO1394QtConsole](#sawrobotio1394qtconsole)
      * [Usage](#usage)
      * [Widgets](#widgets)
      * [Testing the digital inputs](#testing-the-digital-inputs)
      * [Testing sensors](#testing-sensors)
      * [Testing motor current](#testing-motor-current)
   * [sawIntuitiveResearchKitQtPID](#sawintuitiveresearchkitqtpid)
      * [Usage](#usage-1)
      * [Widgets](#widgets-1)
   * [sawIntuitiveResearchKitQtConsoleJSON (<strong>and</strong> ROS dvrk_robot dvrk_console_json)](#sawintuitiveresearchkitqtconsolejson-and-ros-dvrk_robot-dvrk_console_json)
      * [Usage](#usage-2)
      * [Widgets](#widgets-2)
         * [Instructions](#instructions)

<!-- Added by: anton, at: 2021-07-13T09:35-04:00 -->

<!--te-->

# How to run the examples

The following sections assumes that you performed every step in:
* [Controller connectivity](/jhu-dVRK/sawIntuitiveResearchKit/wiki/ControllerConnection)
* [Software build instructions](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Build)
* [Generating XML configuration files](/jhu-dvrk/sawIntuitiveResearchKit/wiki/XMLConfig)
* [Hardware setup and testing](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Hardware)
* [Calibrating and updating XML configuration files](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Calibration

The examples are listed in order of complexity.  The last example is actually the core application used to run the dVRK software (aka the "dVRK console").  It is nevertheless recommended to follow the instructions for the **sawRobotIO1394QtConsole** example to test the hardware upon reception.  You can skip the **sawIntuitiveResearchKitQtPID**.

There are three ways to run the software.  The first one is the most common (_and recommended!_), the later two are script based:

1. Running an executable based C++ code that creates and connects the components:
  * `sawIntuitiveResearchKitQtPID` -- requires several command-line parameters
  * `sawIntuitiveResearchKitQtConsoleJSON` -- requires a console.json configuration file
  * `rosrun dvrk_robot dvrk_console_json` -- same as sawIntuitiveResearchKitQtConsoleJSON but with ROS topics

1. Using the `cisstComponentManager` to process a [script file](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/share/sawIntuitiveResearchKitQtPID.cisst) that creates and connects the components (requires shared libraries):
  * `cisstComponentManager -e sawIntuitiveResearchKitQtPID.cisst`

1. Using Python to process a [script file](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/share/sawIntuitiveResearchKitQtPID.py) that creates and connects the components (requires shared libraries and Python wrapping):
  * `python -i sawIntuitiveResearchKitQtPID.py`

The latter two options require the software to be built using `shared` libraries (see CMake options, set by default if you build the dVRK software using ROS catkin build). The last option also requires that cisst be built with Python support, which adds a dependency on SWIG, Python, and numpy. Note that the script files and the XML files are in the `sawIntuitiveResearchKit/share` sub-directory.

# Python Test Programs

Assuming that you're using ROS, you can use `dvrk_arm_test.py` to let an arm go through a little predetermined dance, testing all the degrees of freedom.  First start the dVRK console using `rosrun dvrk_robot dvrk_console_json` and then
run the test script with:
```bash
rosrun dvrk_python dvrk_arm_test.py -a ECM
```

```bash
dvrk_arm_test.py
 -a <value>, --arm <value> : name of the arm to test ('ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3')
 -i <value>, --interval <value> : expected interval in seconds between messages sent by the device (default: 0.01)
```

# sawRobotIO1394QtConsole

## Usage

Use `sawRobotIO1394QtConsole` to test your configuration files and connections.  The command line parameters can be found using `sawRobotIO1394QtConsole` without options:
```bash
sawRobotIO1394QtConsole:
 -c <value>, --config <value> : configuration file (required)
 -p <value>, --port <value> : firewire port number(s) (optional)
 -n <value>, --name <value> : robot name (optional)
```

The only required parameter is the `sawRobotIO1394` XML configuration file generated from the `.cal` file provided by Intuitive Surgical Inc and updated using the different calibration procedures in [Calibrating and updating XML configuration files](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Calibration).  Assuming that you have set your path correctly and you are in the directory that contains your configuration file, you can start the program using something like:
```
sawRobotIO1394QtConsole -c sawRobotIO1394-PSM1-12345.xml
```

## Widgets

This program relies on two Qt widgets, one for all the digital inputs and one for the IO per axis.

![Digital IO GUI](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-gui-buttons.png)

The "buttons" widget shows all the digital inputs as defined in the XML configuration files.  For each button, the widget provides:
* The digital input name which also corresponds to the IO component interface name.
* The state of the button, i.e. either `pressed` or `released`
* The total number of events received so far
* Time of last event received

![Robot IO GUI](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-gui-io.png)

The top sections of the IO widget are:
* Power.  The power on the QLA boards is controlled at two levels, per board and per actuator.  You can either `Power all` to power both the boards and actuators or use `Power boards` to power the boards only.  You can then enable or disable power per actuator using the check boxes `Actuator power`.
* Watchdog.  The watchdog is used to stop all power on the controllers if not message has been received for a while.   This can happen if a program crashes or the firewire connection is lost.  The default value is 300 ms.   A value of 0 disables the watchdog and is not recommended.
* Encoders.  For the da Vinci research system, use the button `Bias from potentiometers`.  This will reset the zero position of the encoders based on the current potentiometer value.  The calibration file provided by Intuitive Surgical Inc. should define the potentiometer offsets properly, i.e. the zero position should correspond to the arm's home position.
* Current.  The checkbox allows you to turn on or off direct current control, i.e. allows you to set the desired current from the IO widget using either the text boxes or sliders.
* Timing.  This is a standard cisst widget used to report the timing performance of the underlying component.

## Testing the digital inputs

1. Foot pedals.  Please note that the second foot pedal from the right is not wired.  All other foot pedals should be connected to a dVRK controllers (since rev. 1.5, there's a separate configuration files for each controller dissociated from the arm configuration file).  Test the foot pedal using the XML file for that specific controller (i.e. MTML or MTMR).  The foot pedals to test are, from left to right:
   * `Clutch`
   * `Camera`
   * `CAM+` and `CAM-`, the long center pedal can be toggle on both ends.
   * Not wired on older systems, `Bi` or `BiCoag` for recent dVRK controllers.
   * `Coag` or `Mono`.  Please note that the labeling might differ.
1. PSM buttons.  There are 4 digital inputs configured per PSM arm:
   * `SUJClutch`.  White button on side of top horizontal link.
   * `ManipClutch`.  White button on top of vertical link (translation for tool insertion)
   * `Adapter`.  Triggered when the sterile adapter is engaged or removed.  Make sure you have shorted the two first pins as described in [the hardware page](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Hardware)
   * `Tool`.  Triggered when the tool is inserted or removed.   Please note that the tool can NOT be inserted until the sterile adapter's gears are matched with the arm's gears.   DO NOT force the tool in the sterile adapter!

## Testing sensors

***NOTE:*** you should have to the power off using the e-stop.

1. Check the current feedback values, these should be close to zero, +/- 50 mA.   If you get a current feedback much higher (e.g. 2A, 6A), this might indicate a hardware issue.   We test all the FPGA/QLA boards before shipping but better be safe than sorry.
1. Hit the button "Encoders - bias from potentiometers".  At that point, move each joint one by one and check that encoder AND potentiometer positions are consistent, i.e. same sign and approximatively the same values.
1. Check the encoders direction.   Again, using the ISI user manual, verify for each joint that the direction is correct, i.e. angles increase with motion in the positive direction.
1. Check the home value, i.e. move the MTM/PSM by hand and find the zero position (as displayed in the GUI).   The pose should match with the drawings in the ISI user manual (if you have the proper ".cal" files).

## Testing motor current

***NOTE:*** you should turn the power on using the e-stop.

***IMPORTANT:*** as soon as you have verified that the current direction is correct for a given joint, turn the requested current back to zero.

Once everything has been checked all the sensors, you can power the controllers.  When you hit "Power/ Enable all", all the power indicators on the Qt widget should turn green.   Please keep an eye on the current feedback and a hand on the e-stop button.  There is a couple more things to check:

1. Check the box "Current - Direct control".  This allows you to use the IO Widget to set the required currents.
1. You should see all current feedback values hovering close to zero.  If not, you might have to redo the current calibration as described in [Calibrating and updating XML configuration files](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Calibration).
1. Joint by joint, apply a very small current (in GUI request current), something in the order of 100 mA.   You should feel a torque/force on the arm AND the current feedback should match the requested current value.  Make sure each joint moves in the correct direction, i.e. if you apply a positive current the joint position increases and if you apply a negative current the joint position decreases.

# sawIntuitiveResearchKitQtPID

## Usage

**For most users, skip this section**.  We provide default configuration files for the PID parameters that should be perfectly fine.  Furthermore, we might need to update some of the parameters to reflect future PID implementations so maintaining your own configuration files for the PID would force you to merge new versions manually.

Use `sawIntuitiveResearchKitQtPID` to test and tune your PID parameters.  The command line parameters can be found using `sawIntuitiveResearchKitQtPID` without options:
```
sawIntuitiveResearchKitQtPID:
 -i <value>, --io <value> : configuration file for robot IO (see sawRobotIO1394) (required)
 -p <value>, --pid <value> : configuration file for PID controller (see sawControllers, mtsPID) (required)
 -f <value>, --firewire <value> : firewire port number(s) (optional)
 -a <value>, --arm name <value> : arm name (i.e. PSM1, PSM2, MTML or MTMR) as defined in the sawRobotIO1394 file (required)
```

Assuming that you have set your path correctly and you are in the directory that contains your configuration file, you can start the program using something like:
```
 sawIntuitiveResearchKitQtPID -i sawRobotIO1394-MTML-12345.xml -p sawControllersPID-MTM.xml -a MTML
```

## Widgets

This program uses the IO, Buttons and PID widgets.

![PID GUI](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-gui-pid.png)

The PID Widget doesn't provide any direct access to all the IO features.   Before you enable the PID, make sure you use the IO widget to:
* Power the boards.
* Bias the encoders based on potentiometers.   Important note re. the PSM joint 7:  place the last joint near the middle before you set the encoders based on potentiometers!  Please note that the full tele-operation examples performs this step during homing. 

Once you have configure the IO, you can enable the PID.
* `Maintain position` is the safest option.  This triggers a read of the current position and uses it as target position for the controller.  Ideally, you should hold the arm close to it's home position and then click the `Maintain position` button.
* `Zero position` will set the desired position to 0 for each joint and will attempt to move the arm as fast as possible.  Be careful when you do so, specially for the MTM last joint as the homing using potentiometer values is likely incorrect and can force that joint to hit the mechanical limits pretty hard.

The `Index` spin box can be used to change which axis is used for the plotting area.  The green line is the desired position, the red line is the current position.

Also to note, the PID gains you are setting will not be saved so take a screenshot and edit your XML PID configuration file by hand.

# sawIntuitiveResearchKitQtConsoleJSON (**and** ROS dvrk_robot dvrk_console_json)

## Usage

Use `sawIntuitiveResearchKitQtConsoleJSON` to test the whole system with a position based tele-operation controller.  The command line parameters can be found using `sawIntuitiveResearchKitQtConsoleJSON` without options:
```
sawIntuitiveResearchKitQtTeleOperationJSON:
 -j <value>, --json-config <value> : json configuration file (required)
 -g <value>, --gcmip <value> : global component manager IP address (optional)
```

Assuming that you have set your path correctly and you are in the directory that contains your configuration file, you can start the program using something like:
```
 sawIntuitiveResearchKitQtConsoleJSON -j console-xyz.json
```

The file `console.json` contains a description of your system, i.e. how many arms and which configuration files to use for your arms as well as teleoperation components (see [file formats](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Configuration-File-Formats)).  Since the `console-xyz.json` files depend on your system, you will have to create your own so they can reference the correct arm serial numbers.  Many examples can be found in the `share` directory.  `sawIntuitiveResearchKitQtConsoleJSON` will parse the JSON console configuration file to determine which components should be created, including Qt Widgets.  This allows to handle many different configurations without writing any C++ code nor re-compile.  Note that a "sister" application is available for ROS in `dvrk_robot`, `dvrk_console_json`.  The ROS topics will also be automatically populated based on the content of the JSON configuration file.

## Widgets

The application introduces a few new widgets.   Please note that all the homing logic is implemented in the high level components so the IO and PID widgets should only be used for monitoring and debugging.   You do not need to use the IO widget to power the boards, reset the encoders or bias the current based on current feedback.   These steps are all triggered in sequence when you hit the `Home` button on the console widget.      

![Console messages GUI](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-gui-console-messages.png)

The console widget is a widget that aggregates all the error messages from the different arms and allows to home all arms in one click.

![Arm GUI](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-gui-arm.png)

The arm widget can be used to monitor messages specific to an arm, current 3D position (you can right click on the 3D view to change the orientation widget), joint state, estimated wrench (you can right click on the 3D view to change the wrench widget) and timing of the underlying component (homing, kinematics, ...).

![Tele operation PSM GUI](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-gui-teleop.png)
![Tele operation ECM GUI](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-gui-teleop-ecm.png)

The widgets for PSM and ECM tele-operation will show the current state of the corresponding components.

### Instructions

You will first need to power and home the whole system.  To do so, just check the `Home` box on the first tab.  We recommend to remove the tools before homing but this is not necessary.   If the system doesn't power on properly (check message in the first GUI tab), make sure your E-Stop button is released.

Once the system is powered and home, you can add the sterile adapter.  At that point:
* The sterile adapter gears should turn back and forth until they are engaged.  When engaged, the 4 gears should be recessed.
* In the `Buttons` tab, you should see the `Adapter` state as `pressed`

Once the adapter is engaged, you can insert the tool.  At that point:
* The tool gears should get engaged automatically.  To check that the tool is properly engaged, wiggle to tool tip and you should feel the motor's resistance in all directions.
* In the `Buttons` tab, you should see the `Tool` state as `pressed`

To start the tele-operation, you will first need to push the tool tip past the cannula.  Since the PID controller is already running, you will need to press the `Tool clutch` button (this is the physical white button on top of the PSM, not a Qt GUI Button!).  While pressing the tool clutch button, move the tool tip down (away from the RCM).   You can now used the `Start` button in the `Tele operation` box to enable the tele-op controller.  Check the log messages in the first tab to make sure everything went as planned.

Once the tele-operation controller is running, press the `coag` (or `mono`) foot pedal continuously with your right foot to indicate the operator's presence.  The foot pedal acts as a dead-man switch and replace the optical system used in the real daVinci system to detected the presence of the operator's head.  If you've added a different sensor to detect the operator you obviously don't need to use the `coag` pedal (see [Head Sensor](/jhu-dvrk/sawIntuitiveResearchKit/wiki/HeadSensor)).  You can then tele-operate and use the clutch to re-position the master arms.

When the operator is present, the tele-op component will try to orient the MTM so it matches the orientation of the PSM tooltip.   If you don't let go of the MTM orientation, you might get an error message similar to:
```
14:29:30 Warning #5: MTML-PSM2: unable to align MTM, current angle error is 48.5626
```
Similarly, the gripper/jaw angles have to match before tele-op can start.   You will have to pinch/release the gripper on the MTM until it matches the gripper angle on the PSM.  Until you do so, the following message will be periodically sent to the console:
```
14:29:36 Warning #6: MTML-PSM2: unable to match gripper/jaw angle, pinch and release the gripper
```
