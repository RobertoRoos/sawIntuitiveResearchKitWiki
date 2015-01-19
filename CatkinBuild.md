# Introduction

These instructions are provided for those who want to build the whole cisst/SAW stack using catkin tools only.  This is an alternative to using `catkin_make`.

## Links

* [catkin tools](http://catkin-tools.readthedocs.org/en/latest/index.html)

## Initializing your catkin workspace to work with catkin tools

Assuming you have already installed ROS and you're using Ubuntu, you can install the catkin build tools using:
```bash
sudo apt-get install python-catkin-tools
```

To make sure your path includes the ROS directories:
```bash
source /opt/ros/indigo/setup.sh
```

Then you should be able to create your catkin workspace.  Please note that existing workspaces used in combination with `catkin_make` are not compatible with the catkin build tools.

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

# Packages

## cisstNetlib, cisst, SAW components and cisst-ros bridge

### Get the source

```bash
cd ~/catkin_ws/src
git clone https://github.com/jhu-cisst/cisst-saw --recursive
```

As of January 2015, the master branch doesn't have the catkin/ROS build support.   You need to use the devel branch:
```bash
cd ~/catkin_ws/src/cisst-saw
git checkout devel
git submodule init
git submodule update
```

### Configure, build and install

We recommend to compile the Fortran version.  In case you don't have a Fortran compiler, you will need to install it:
```bash
sudo apt-get install gfortran
```

```bash
cd ~/catkin_ws
catkin build
```

## dvrk-ros

This package is not part of the cisst-saw yet as it contains many CAD files that are of no use for most cisst-saw users and we can't really justify 30MB of data.  Once we removed all the sldprt files, we can add the dvrk-ros as a submodule in cisst-saw and save that extra step:

```bash
cd ~/catkin_ws/src
git clone https://github.com/jhu-dvrk/dvrk-ros
catkin build
```