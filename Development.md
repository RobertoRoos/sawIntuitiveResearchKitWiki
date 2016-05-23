<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](https://github.com/thlorenz/doctoc)*

- [Compatible development branches](#compatible-development-branches)
- [Changes](#changes)
  - [`devel`](#devel)
- [Todo](#todo)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

## Compatible development branches

For each component, `git checkout <branch_name>`:

| Components               | Branches      | Branches     |
| ------------------------ | ------------- |------------- |
| cisst                    | devel         |         |
| cisst-ros                | devel         |         |
| sawKeyboard              | devel         |         |
| sawTextToSpeech          | devel         |         |
| sawRobotIO1394           | devel         |         |
| sawControllers           | devel         |         |
| sawConstraintControllers | devel         |         |
| sawIntuitiveResearchKit  | devel         |         |
| dvrk-ros                 | devel         |         |

Note: sawRobotIO1394 in the devel branch has a different git submodule so you need to go in the component source directory and update the submodule:
```sh
cd ~/catkin_ws/src/cisst-saw/sawRobotIO1394
git submodule init
git submodule update
```

## Changes

### `devel`

* Main API changes are in dvrk_python and dvrk_matlab, converging towards final API (see readme under dvrk_python)
* Code in the components directory has been re-organized to better separate the component's code from examples and applications.  It is overall cleaner and works with the latest catkin build tools.
* By default, the ROS topics have changed.  We now try to use "stamped" data types as much as possible and we've removed the joint positions/velocities topics since the state joint covers them all.  If you need the old topics, `dvrk_console_json` has the option `-c v1_3_0` to support older topics (`--compatibility`).  All topics with headers now have a reasonable timestamp, sequence number and id (string).
* New teleoperation components, one for PSM and one for ECM, now part of sawIntuitiveResearchKit.  Slight differences in ROS topics to control teleop (set state instead of enable).
* Console now handles most events and turn on/off teleoperation components (new `mtsStateMachine` introduced in these components).  Need new topics to control console.
* Better lock/positioning of MTMs
* Cleaner Qt layout

## Todo

Pending issues:
* #55 and #50, use separate IO XML files for external devices (foot pedal, camera focus)
* #54, save content of `.cal` file in generated XML file (for debug, retrieval, ...)
* #35, last issue is to control multiple slaves with one master (not likely in this release)

Solved in next release:
* #38, ROS timestamps
* #36, better states in tele-operation components

Besides pending issues, the following should be addressed before next release:
* Fix cisst warning when optional interfaces/functions are not connected
* Make sure console handles SUJ and PSM/ECM clutch
* Implement motion of ECM in teleopECM
* Check that console/teleops can be controlled and configured (base frame, rotations) using ROS topics
* Add psm-prograsp.json from Google group
* Add ROS topics backward compatibility to all dvrk_robot applications
* Fix CMake/Catkin for OpenIGT and ATIFT (not really dVRK but would help)
* Compute wrench at tooltip based on joint torques and publish as ROS topic
* Add dvrk-openigt to cisst-saw
* Add scale in Qt Console for all teleops
* Deprecate ROS packages `dvrk_kinematics`, `dvrk_teleop`, `rqt_dvrk` by prefixing package name with `deprecated_` and move to new directory `deprecated`.
* Fix teleop PSM (and likely ECM) to handle case where clutch is pressed while teleop is enabled
* In console, if no teleop ECM, do not disable teleop PSM on CAM foot pedal

Maybe for later releases:
* Add ROS TF support, specially for SUJ
* Remove gravity and clutch mode from MTM, now replaced by effort_mode + gravity on/off + lock orientation on/off
* Better handling of redundant joints in MTMs
* Use new state machine in arm classes
* Add alert when SUJ are moved without brakes released
* Use Reflexes/RML for trajectory generation