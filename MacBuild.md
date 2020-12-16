<!--ts-->
   * [Introduction](#introduction)
   * [Requirements](#requirements)
   * [Build](#build)
      * [Get the code](#get-the-code)
      * [cisstNetlib](#cisstnetlib)
      * [cisst/SAW](#cisstsaw)
   * [Run the code](#run-the-code)

<!-- Added by: anton, at:  -->

<!--te-->

# Introduction

This is totally experimental and not that useful.  When building for Mac OS, one can use the dVRK stack with simulated arms (in kinematic mode) and potentially with the real arms with the ethernet interface (Local Link).  The ethernet interface requires firmware version 7 or higher AND controllers with a physical a network interface.  Even with the network interface, the Mac OS build is laking the current features:
 * ROS.  The code will run but you will need to use something like the sawsSocketStreamer or sawOpenIGTKLink to communicate between programs instead of ROS
 * Potentiometer calibration and gravity compensation identification (calibration programs are using ROS)
 * `catkin build`

# Requirements
 * Xcode
 * CMake, see www.cmake.org
 * A Fortran compiler.  This one works: http://hpc.sourceforge.net/#fortran
 * Qt.  You will need to create an account and download the free version

# Build

The current process is not automated.  It might be possible to install the programs `wstool` and `catkin` since both are Python based but I didn't try.  The build is performed in steps and each step requires to configure (CMake), build and install.  I prefer "Unix Makefiles" but one could probably use Xcode projects.

## Get the code

```sh
mkdir ~/dVRK
cd ~/dVRK
clone https://github.com/jhu-cisst/cisst-saw.git --recursive
```
```sh
cd ~/dVRK/cisst-saw
git submodule foreach git checkout devel
git submodule foreach git pull origin devel
git submodule foreach git submodule init
git submodule foreach git submodule update
```

## cisstNetlib

When using CMake, pick the generator "Unix Makefiles" and select "Specify native compilers".  In the next window, specify the "Fortran" compiler only.  It you've installed the compiler mentioned above, it should be `/usr/local/bin/gfortran`.  For CMake, use the source directory `/Users/you/dVRK/cisst-saw/cisstNetlib`.  Please note that `you` in `/Users/you` should be replaced by your login name.

In CMake, change:
 * `CMAKE_BUILD_TYPE` to `Release`
 * `CMAKE_INSTALL_PREFIX` to `/Users/you/dVRK/install`

Then in CMake, configure and generate.   In the build tree, build using `make` and then install with `make install`.

## cisst/SAW

In CMake, use the generator "Unix Makefiles" and the source tree should be `/Users/you/dVRK/cisst-saw`.

In CMake, change:
 * `CMAKE_BUILD_TYPE` to `Release`
 * `CMAKE_INSTALL_PREFIX` to `/Users/you/dVRK/install`
 * `CISSTNETLIB_DIR` to `/Users/you/dVRK/install`
 * `CISSTNETLIB_USE_LOCAL_INSTALL` checked
 * `Cisstnetlib_DIR` to `/Users/you/dVRK/install/cmake`
 * `CISST_HAS_QT5` checked - you will also need to set all the Qt paths, e.g. `Qt5Core_DIR` set to `/Users/you/Qt/5.15.2/clang_64/lib/cmake/Qt5Core`
 * `CISST_USE_EXTERNAL` checked
 * `CISST_HAS_JSON` checked
 * `CISST_USE_SI_UNITS` checked
 * `CISST_BUILD_SHARED_LIB` checked
 * `CISST_cisstRobot` checked
 * `SAW_sawControllers` checked
 * `SAW_sawIntuitiveResearchKit` checked
 * `SAW_sawRobotIO1394` checked

You should be able to configure and generate in CMake.  In your build tree for cisst-saw, `make -j`.  No need to build.

# Run the code

In your build tree for cisst-saw, you will need to set some environment variables:
```sh
source cisst/cisstvars.sh
export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:/Users/you/dVRK/build/cisst-saw/cisst/cisstReflexxesTypeII/lib
```
The last command is to find the shared libraries for Reflexxes.  The path with depend on where you built cisst-saw.

Then go in your source tree:
```sh
cd ~/dVRK/cisst-saw/sawIntuitiveResearchKit/share/console
sawIntuitiveResearchKitQtConsoleJSON -j console-full-system-simulated.json
```

# Screenshots


