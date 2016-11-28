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

## Changes

### `devel`

## Todo for 1.5

Pending issues:
* #55 and #50, use separate IO XML files for external devices (foot pedal, camera focus)
* #54, save content of `.cal` file in generated XML file (for debug, retrieval, ...)
* #35, last issue is to control multiple slaves with one master (not likely in this release)

Besides pending issues, the following should be addressed before next release:
* Use Reflexes/RML for trajectory generation **[Done]**
* Fix cisst warning when optional interfaces/functions are not connected
* Make sure console handles SUJ and PSM/ECM clutch
* Implement motion of ECM in teleopECM
* Add dvrk-openigt to cisst-saw
* Remove gravity and clutch mode from MTM, now replaced by effort_mode + gravity on/off + lock orientation on/off
* Use new state machine in arm classes

Maybe for later releases:
* Add ROS TF support, specially for SUJ
* Better handling of redundant joints in MTMs
* Add alert when SUJ are moved without brakes released
* Add test function on user Id and permission on `/dev/fw*`?