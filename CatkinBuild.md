# Introduction

These instructions are provided for those who want to build the whole cisst/SAW stack using catkin tools only.  This is an alternative to using `catkin_make`.

## Links

* [catkin tools](http://catkin-tools.readthedocs.org/en/latest/index.html)

## Initializing your catkin workspace to work with catkin tools

Assuming you have already installed ROS and you're using Ubuntu, you can install the catkin build tools using:
```bash
sudo apt-get install python-catkin-tools
```

Then you should create your catking workspace.  Please note that existing workspace used in combination with `catkin_make` are not compatible with the catkin build tools.

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init                       
```

# Packages

## catkin profiles

If you need to do any debugging, you will likely have to compile your code using debug options.  Once your code has been debugged, you should probably compile a version with the proper release options.  Fortunately, catkin tools can handle multiple profiles.

Create two basic profiles:
```bash
catkin profile create debug
catkin profile create release
```

To get started with the `debug` profile:
```bash
catkin profile set debug
```

To check which profiles are available and which one is active:
```bash
catkin profile list
```

## cisstNetlib

### Get the sources

### Configure and build