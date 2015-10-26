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
You will first need to build cisst and its dependencies, see [cisst/SAW catkin instructions](https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros)

## dvrk-ros

This package is not part of the cisst-saw yet as it contains many CAD files that are of no use for most cisst-saw users and we can't really justify 30MB of data.  Once we removed all the sldprt files, we can add the dvrk-ros as a submodule in cisst-saw and save that extra step:

```bash
cd ~/catkin_ws/src
git clone https://github.com/jhu-dvrk/dvrk-ros
catkin build
```