<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](https://github.com/thlorenz/doctoc)*

- [Compatible development branches](#compatible-development-branches)
- [Changes](#changes)
  - [`devel`](#devel)
- [Todo](#todo)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

## Compatible development branches

Starting with version 2.0, you should use the `.rosintall` file in the *dvrk-ros* repository to pull all the dependencies required for the `master` or `devel` branches. 

## Changes

### `devel`

## Roadmap for 2.1

* Small projects
  * Update Teleop doc
  * #121, add cap on maximum difference between PID goals
  * IK with joint limits for PSMs
  * #54, save content of `.cal` file in generated XML file (for debug, retrieval, ...)
  * Add alert when SUJ are moved without brakes released
  * Autonomous ECM motion
  * Documentation and scripts for stereo camera calibration

* Bigger projects
  * Trajectory in cartesian space
  * Controller class:
    * Separate control PSM from PSM jaws
    * Add different controllers: velocity, interpolate...
  * 3D User Interface, ideally with RViz + endoscopic stereo stream
  * Better teleop PSM:
    * Use symmetry of MTM to set orientation and optimize roll
    * Force feedback on MTM based on error in orientation between MTM and PSM
  * MoveIt! (via CRTK)
  * Binary packages with repository on dvrk.lcsr.jhu.edu
  * ROS 2 (would likely require binary packages)
