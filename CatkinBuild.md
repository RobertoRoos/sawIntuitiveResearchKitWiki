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

To setup your environment to use your newly created workspace:
```bash                      
source ~/catkin_ws/devel/setup.sh
```

# Packages

## cisstNetlib

### Get the source

```bash
cd ~/catkin_ws/src
git clone https://github.com/jhu-cisst/cisstNetlib
```

### Configure, build and install

We recommend to compile the Fortran version.  In case you don't have a Fortran compiler, you will need to install it:
```bash
sudo apt-get install gfortran
```

```bash
cd ~/catkin_ws
catkin build cisst_netlib --cmake-args -DCISSTNETLIB_LANGUAGE=Fortran -DCISSTNETLIB_ARCHITECTURE=x86_64 -DCMAKE_BUILD_TYPE=Release
cd ~/catkin_ws/build/cisst_netlib
make install
```

## cisst

### Get the source

```bash
cd ~/catkin_ws/src
git clone https://github.com/jhu-cisst/cisst
```

The default cisst branch is `master`.  If you need to use checkout a different branch:
```bash
cd ~/catkin/src/cisst
git checkout devel # or whatever branch you need
```

### Configure and build

```bash
cd ~/catkin_ws
catkin build cisst --cmake-args -DCISST_HAS_CISSTNETLIB=ON -DCISST_cisstRobot=ON -DCMAKE_BUILD_TYPE=Release
```