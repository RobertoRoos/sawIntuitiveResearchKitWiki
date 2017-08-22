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
```

Afterwards, follow this

```bash
cd ~/catkin_ws/src/cisst-saw
git submodule foreach git checkout devel
git submodule foreach git pull
git submodule foreach git submodule init
git submodule foreach git submodule update
cd ~/catkin_ws/src/dvrk-ros
git checkout devel
git pull
```

## Changes

### `devel`

## Todo for 1.5

Pending issues:
* #55 and #50, use separate IO XML files for external devices (foot pedal, camera focus) **[Done]**
* #54, save content of `.cal` file in generated XML file (for debug, retrieval, ...)
* #35, last issue is to control multiple slaves with one master (not likely in this release)

Besides pending issues, the following should be addressed before next release:
* Use Reflexes/RML for trajectory generation **[Done]**
* Fix cisst warning when optional interfaces/functions are not connected
* Add optional for ROS bridge topics
* Make sure console handles SUJ and PSM/ECM clutch
* Add warning when not compiled in release mode, cisstConfig.h should have build type
* Test code to re-position MTM in tele-op mode
* Add event on teleop components when in follow mode
* Implement motion of ECM in teleopECM
* Add dvrk-openigt to cisst-saw
* Remove gravity and clutch mode from MTM, now replaced by effort_mode + gravity on/off + lock orientation on/off
* Use new state machine in arm classes

Maybe for later releases:
* Add ROS TF support, specially for SUJ
* Better handling of redundant joints in MTMs
* Add alert when SUJ are moved without brakes released
* Add test function on user Id and permission on `/dev/fw*`?