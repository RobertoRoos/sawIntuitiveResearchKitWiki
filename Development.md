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

When switching from `devel` to `master` branch, it is recommended to do a full `catkin clean` since the file structure and CMake external projects might be different.

If you don't want to checkout out those repositories one by one, just checkout all the repositories within `cisst-saw`. Note that, don't forget to checkout `dvrk-ros`, which is not located in `cisst-saw`.

If it's your first time, follow this

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/jhu-cisst/cisst-saw --recursive
cd ~/catkin_ws/src/cisst-saw
git submodule foreach git checkout devel
git submodule foreach git submodule init
git submodule foreach git submodule update
cd ~/catkin_ws/src
git clone https://github.com/jhu-dvrk/dvrk-ros
cd ~/catkin_ws/src/dvrk-ros
git checkout devel
cd ~/catkin_ws/src
git clone https://github.com/jhu-dvrk/dvrk-gravity-compensation
cd ~/catkin_ws/src/dvrk-gravity-compensation
git checkout devel
```

Afterwards, follow this

```bash
cd ~/catkin_ws/src/cisst-saw
git submodule foreach git checkout devel
git submodule foreach git pull origin devel
git submodule foreach git submodule init
git submodule foreach git submodule update
cd ~/catkin_ws/src/dvrk-ros
git checkout devel
git pull origin devel
cd ~/catkin_ws/src/dvrk-gravity-compensation
git checkout devel
git pull origin devel
```

## Changes

### `devel`

## Roadmap for 1.8

Definitively:
* ~~ECM gravity compensation~~
* ~~Teleop with clutch/relative orientation~~
* ~~#35, last issue is to control multiple slaves with one master~~
* ~~Automatic tool type discovery, dynamically change tool type~~
* ~~Temperature checks with warning and errors~~
* ~~Check IO/PID periodicity and stops if overloaded~~
* ~~Closed kinematics for ECM with unit tests~~
* ~~Closed kinematics for MTM with unit tests~~
* Better handling of redundant joints in MTMs #2 and #56
* ~~TeleopPSM scale gripper/jaw based on joint limits~~
* ~~Add joint goal from Qt Arm widget~~
* ~~Fix cisst-ros to expose ROS node and pass arguments to node (e.g. namespace)~~
* ~~Update SUJ doc + example with simulated SUJ~~
* Update Teleop doc
* Fix ctrl+c hangs with ROS/cisstMultiTask/Qt

Maybe:
* #121, add cap on maximum difference between PID goals
* Trajectory in cartesian space
* Fix relative joint move, maybe relative cartesian move
* UDP support?
* IK with joint limits for PSMs
* #54, save content of `.cal` file in generated XML file (for debug, retrieval, ...)
* Add dvrk-openigt to cisst-saw
* Add optional for ROS bridge topics
* Add alert when SUJ are moved without brakes released
* Add test function on user Id and permission on `/dev/fw*`?

## Roadmap for 2.0

* CRTK compliance for C++ command/method and ROS topics
* MoveIt! (via CRTK)
