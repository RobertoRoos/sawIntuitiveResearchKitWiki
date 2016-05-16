<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](https://github.com/thlorenz/doctoc)*

- [Compatible development branches](#compatible-development-branches)
- [Changes](#changes)
  - [`devel`](#devel)
  - [`feature-ecm`](#feature-ecm)
- [Todo](#todo)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

## Compatible development branches

For each component, `git checkout <branch_name>`:

| Components               | Branches      | Branches     |
| ------------------------ | ------------- |------------- |
| cisst                    | devel         | devel        |
| cisst-ros                | devel         | devel        |
| sawKeyboard              | devel         | devel        |
| sawTextToSpeech          | devel         | devel        |
| sawRobotIO1394           | devel         | devel        |
| sawControllers           | devel         | devel        |
| sawConstraintControllers | devel         | devel        |
| sawIntuitiveResearchKit  | devel         | feature-ecm  |
| dvrk-ros                 | devel         | feature-ecm  |

Note: sawRobotIO1394 in the devel branch has a different git submodule so you need to go in the component source directory and update the submodule:
```sh
cd ~/catkin_ws/src/cisst-saw/sawRobotIO1394
git submodule init
git submodule update
```

## Changes

### `devel`

* Mostly small bug fixes
* Main API changes are in dvrk_python and dvrk_matlab, converging towards final API 
* Code in the components directory has been re-organized to better separate the component's code from examples and applications.  It is overall cleaner and works with the latest catkin build tools.
* By default, the ROS topics have changed.  We now try to use "stamped" data types as much as possible and we've removed the joint positions/velocities topics since the state joint covers them all.  If you need the old topics, `dvrk_console_json` has the option `-c v1_3_0` to support older topics (`--compatibility`).  All topics with headers now have a reasonable timestamp, sequence number and id (string).

### `feature-ecm`

* This branch will likely be merged before next release
* New teleoperation components, one for PSM and one for ECM, now part of sawIntuitiveResearchKit
* Console now handles most events and turn on/off teleoperation components (new `mtsStateTable` introduced in these components)
* Better lock/positioning of MTMs
* Tighter Qt layout

## Todo

Besides pending issues, the following should be addressed before next release:
* Fix cisst warning when optional interfaces/functions are not connected
* Make sure console handles SUJ and PSM/ECM clutch
* Make sure PID and arm classes don't re-timestamp joint data
* Implement motion of ECM in teleopECM
* Make sure console can be used with teleopECM but no teleopPSM
* Check that console/teleops can be controlled and configured (base frame, rotations) using ROS topics
* Add psm-prograsp.json from Google group
* Add ROS topics backward compatibility to all dvrk_robot applications
* PID class should have state joint data initialized before it's started (i.e. joint names list is empty in rostopic)
* Fix CMake/Catkin for OpenIGT and ATIFT (not really dVRK but would help)
* Compute wrench at tooltip based on joint torques and publish as ROS topic
* Add dvrk-openigt to cisst-saw
* Cleanup cisst CMake to avoid WARNING and use STATUS instead (reduce catkin build messages)

Maybe for later releases:
* Add ROS TF support, specially for SUJ
* Remove gravity and clutch mode from MTM, now replaced by effort_mode + gravity on/off + lock orientation on/off
* Better handling of redundant joints in MTMs
* Use new state machine in arm classes
* Add alert with SUJ are moved without brakes released
* Use Reflexes/RML for trajectory generation