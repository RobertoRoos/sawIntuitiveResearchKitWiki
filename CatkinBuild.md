<!--ts-->
   * [Introduction](#introduction)
   * [catkin build and rosinstall](#catkin-build-and-rosinstall)
      * [Debian packages](#debian-packages)
      * [Catkin workspace, clone and build](#catkin-workspace-clone-and-build)
      * [Environment variables](#environment-variables)
   * [catkin build and git submodules](#catkin-build-and-git-submodules)
      * [cisstNetlib, cisst, SAW components and cisst-ros bridge](#cisstnetlib-cisst-saw-components-and-cisst-ros-bridge)
      * [dvrk-ros](#dvrk-ros)
   * [cmake; make and git submodules](#cmake-make-and-git-submodules)

<!-- Added by: anton, at:  -->

<!--te-->

# Introduction

We suggest 3 different ways to retrieve all the dVRK required source repositories and compile them.  The first two methods are for Linux/ROS users only and rely on the catkin build tools.   Pleas note that **we do not support `catkin_make`** .

# `catkin build` and `rosinstall`

The `rosinstall` configuration file is provided in dVRK 2.x and higher but can also be used with older versions (see https://github.com/jhu-dvrk/dvrk-ros).  The `rosinstall` file defines all the github repositories that need to be cloned in your workspace as well as which branches.
 
## Debian packages

This section assumes you already have ROS installed (see https://www.ros.org).  You will need to install a few more packages for the dVRK software:
* **Ubuntu 16.04** with ROS Kinetic: `sudo apt-get install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig flite sox espeak cmake-curses-gui cmake-qt-gui libopencv-dev git subversion gfortran libcppunit-dev qt5-default python-wstool python-catkin-tools`
* **Ubuntu 18.04** with ROS Melodic: `sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev python-wstool python-catkin-tools`
* **Ubuntu 20.04** with ROS Noetic:`sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev python3-wstool python3-catkin-tools python3-osrf-pycommon`

## Catkin workspace, clone and build

If you're using ROS Melodic and the devel branch, you can just copy/paste the following block of commands in a terminal.   For other configurations, make sure your replace `melodic` by `noetic` or whatever version of ROS you're using.  For the `master` branch, replace `devel` by `master`.

```sh
source /opt/ros/melodic/setup.bash # or use whatever version of ROS is installed!
mkdir ~/catkin_ws                  # create the catkin workspace
cd ~/catkin_ws                     # go in the workspace
wstool init src                    # we're going to use wstool to pull all the code from github
catkin init                        # create files for catkin build tool
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release # all code should be compiled in release mode
cd src                             # go in source directory to pull code
wstool merge https://raw.githubusercontent.com/jhu-dvrk/dvrk-ros/devel/dvrk_ros.rosinstall # or replace devel by master
wstool up                          # now wstool knows which repositories to pull, let's get the code
catkin build --summary             # ... and finally compile everything
```

## Environment variables

This is recommended if you're going to use a single catkin workspace (as most users do): https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#setting-up-your-environment-variables-for-ros

# `catkin build` and git submodules

This approach is a bit more complicated and will add some extra repositories for SAW components not needed for the dVRK software.  Compilation time will be slightly longer.  Most repositories in cisst/SAW will be cloned using git sub-modules.

## cisstNetlib, cisst, SAW components and cisst-ros bridge

You will first need to build cisst and its dependencies.   Follow the instructions provided for [cisst/SAW catkin build](https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros) and then come back to this page for the dVRK/ROS specific packages.

## dvrk-ros

These packages are not part of the cisst-saw repositories so you have to clone them manually.   You first need to download the cisst-saw libraries and components (see instructions above) and then do:

```bash
cd ~/catkin_ws/src
git clone https://github.com/jhu-dvrk/dvrk-ros
git clone https://github.com/jhu-dvrk/dvrk-gravity-compensation
git clone https://github.com/collaborative-robotics/crtk_msgs crtk/crtk_msgs
git clone https://github.com/collaborative-robotics/crtk_python_client crtk/crtk_python_client
git clone https://github.com/collaborative-robotics/crtk_matlab_client crtk/crtk_matlab_client
catkin build --summary
```

# `cmake; make` and `git submodules`

This is **not recommended** for most users but can be useful if you don't have ROS or want to compile some of the dVRK code on Windows.  See: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Build
