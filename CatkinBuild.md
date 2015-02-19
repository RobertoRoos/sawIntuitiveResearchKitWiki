# Introduction

These instructions are provided for those who want to build the whole cisst/SAW stack using catkin tools only.  This is an alternative to using `catkin_make`.
# Packages

## cisstNetlib, cisst, SAW components and cisst-ros bridge

See [cisst/SAW catkin instructions](https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros)

## dvrk-ros

This package is not part of the cisst-saw yet as it contains many CAD files that are of no use for most cisst-saw users and we can't really justify 30MB of data.  Once we removed all the sldprt files, we can add the dvrk-ros as a submodule in cisst-saw and save that extra step:

```bash
cd ~/catkin_ws/src
git clone https://github.com/jhu-dvrk/dvrk-ros
catkin build
```