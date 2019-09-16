<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

- [Introduction](#introduction)
- [Packages](#packages)
  - [cisstNetlib, cisst, SAW components and cisst-ros bridge](#cisstnetlib-cisst-saw-components-and-cisst-ros-bridge)
  - [dvrk-ros](#dvrk-ros)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# Introduction

These instructions are provided for those who want to build the whole cisst/SAW stack using catkin tools only.  This is an alternative to using `catkin_make`.
# Packages

## cisstNetlib, cisst, SAW components and cisst-ros bridge

You will first need to build cisst and its dependencies.   Follow the instructions provided for [cisst/SAW catkin build](https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros) and then come back to this page for the dVRK/ROS specific packages.  Once this is done, move on to the next section.

## dvrk-ros

These packages are not part of the cisst-saw repositories since they contain many CAD files that are of no use for most cisst-saw users.   You first need to download the cisst-saw libraries and components (see instructions above) and then do:

```bash
cd ~/catkin_ws/src
git clone https://github.com/jhu-dvrk/dvrk-ros
git clone https://github.com/jhu-dvrk/dvrk-gravity-compensation
catkin build
```