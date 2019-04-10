# DEPRECATED
For simulation needs, please try the following two options:
* [Kinematics Simulation](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Kinematics-Simulation)
* [V-REP Simulator from Universitá di Napoli Federico II](https://doi.org/10.1109/BIOROB.2018.8487187)

=======================
<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

- [intuitive_research_kit_ros Documentation (deprecated)](#intuitive_research_kit_ros-documentation-deprecated)
  - [1. About](#1-about)
  - [2. Dependency](#2-dependency)
  - [3. Download and Build](#3-download-and-build)
- [to ros workspace](#to-ros-workspace)
- [clone repository](#clone-repository)
- [add to ros workspace](#add-to-ros-workspace)
- [resource](#resource)
- [build irk_kinematics](#build-irk_kinematics)
- [build irk_teleop](#build-irk_teleop)
- [One MTM + PSM teleoperation](#one-mtm--psm-teleoperation)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# intuitive_research_kit_ros Documentation (deprecated)
Please refer to https://github.com/jhu-dvrk/dvrk-ros

## 1. About
A ROS based simulation stack has been developed to give Robot visualization and simulation.

## 2. Dependency
This package depends on CISST (cisstRobot) and sawROS, please check [wiki:sawROSTutorial sawROSTutorial] for build instructions.

## 3. Download and Build

1. Clone the repository into a directory under ROS_PACKAGE_PATH (alternatively you can use ROS workspace).  The code is under https://git.lcsr.jhu.edu/dvrk/intuitive_research_kit_ros

  ```bash
# to ros workspace
roscd
# clone repository
git clone git@git.lcsr.jhu.edu:dvrk/intuitive_research_kit_ros.git
# add to ros workspace
rosws set intuitive_research_kit_ros
# resource
source setup.sh
```

1. Change to branch ''dev_groovy''

  ```bash
git checkout dev_groovy
```

1. Build packages 

  ```bash
# build irk_kinematics 
rosmake irk_kinematics
# build irk_teleop
rosmake irk_teleop
```

NOTE: You might need to setup CISST path during build process in CMake.


## 4. Run MTM / PSM simulation

1. Run visualization 

  ```bash
# One MTM
roslaunch irk_model mtm_right_rviz.launch 

# One PSM
roslaunch irk_model psm_rviz.launch 

# One MTM + One PSM 
```

1. Run tele-operation example

  ```bash
# One MTM + PSM teleoperation
roslaunch irk_teleop test_teleop.launch 
```

3. Screenshot

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/teleop.png)

## 3. Packages details

* *irk_model*: all CAD models, URDF (robot description) files are in the irk_model package. 

* *irk_model*: this package includes logic robot witch connects to joint interface and provides a Cartesian interface.

* *irk_teleop*: the irk_teleop package has teleoperation control logic and a GUI as input device, it also displays MTM and PSM end effector position. 

Control diagram: 
  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/block_diagram.png)

## 6. Others

### 6.1. Related pages
TODO_HOW_TO_CONNECT_TO_REAL_ROBOT

### 6.2. TODO
* Give two MTM / PSM example
* Add Gazebo simulation (Physics)

### 6.3. Questions
Please email me if you have questions or are interested to do some simulation work.
Email: zihan.chen.jhu@gmail.com