<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

- [Building the software](#building-the-software)
  - [1. Dependencies](#1-dependencies)
  - [2. cisst/saw](#2-cisstsaw)
    - [2.1. Automatic build](#21-automatic-build)
    - [2.2. Manual build](#22-manual-build)
  - [3. Compile in Release mode.](#3-compile-in-release-mode)
  - [4. Setting up some environment variables](#4-setting-up-some-environment-variables)
  - [5. Using Qt Creator](#5-using-qt-creator)
    - [5.1. Step by step instructions](#51-step-by-step-instructions)
    - [5.2. Settings](#52-settings)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# Building the software

These instructions are specific to Ubuntu Linux.  The low level software is Linux specific (i.e. it won't run on other OSs) but should run on any Linux distribution.  High level components (all but `sawRobotIO1394`) can be compiled on Linux, Windows, MacOS and used in separate processes using cisstMultiTask with ICE.

## 1. Dependencies

* Linux for IEEE-1394 (Firewire) - Ubuntu 12.04 preferably, Ubuntu 14.04 should work  
* git
* CMake for build and configuration
* C++ compiler, either gcc or clang
* libraw1394: IEEE-1394 API
* libxml2: for parsing XML config files
* Qt: GUI, version 4 on Ubuntu 12.04, 5 on Ubuntu 14.04
* libncurses5-dev: curses based test GUI for 1394
* flite: for some experimental text to speech

```bash
sudo apt-get install libxml2-dev libraw1394-dev libncurses5-dev qtcreator
sudo apt-get install flite cmake-curses-gui cmake-qt-gui libopencv-dev git subversion
```

## 2. cisst/saw

The main repositories for the dVRK project are:
* http://github.com/jhu-cisst/cisst-saw.git - equivalent of the cisst SVN repository - you need to checkout this first.  It uses submodules to clone all cisst libraries and SAW components
* http://github.com/jhu-cisst/cisst.git  - cisst libraries, this repository is included as a submodule of cisst-saw.git
* http://github.com/jhu-saw/....  - submodules for sawRobotIO1394, sawKeyboard, sawControllers, ....  All included as submodules of cisst-saw.git
* http://github.com/jhu-dvrk/sawIntuitiveResearchKit  - SAW components specific to the dVRK.  All included as submodules of cisst-saw.git

For ROS, you will need these extra repositories
* http://github.com/jhu-cisst/cisst-ros.git - SAW component and conversion functions used to create bridges between the cisst/SAW components and ROS
* http://github.com/jhu-dvrk/dvrk-ros.git - ROS bridge, data files, ... specific to the dVRK/ROS integration	

We provide two ways to build the software:
* Automatically download and build using the bash file [autocisst.sh](https://raw.githubusercontent.com/jhu-dvrk/sawIntuitiveResearchKit/master/share/autocisst.sh)
* Download, configure and build step by step manually 

Essentially, the `autocisst.sh` file performs all the steps described in the manual build section.

Please note that both of these ways will automatically set the code in release mode.

### 2.1. Automatic build

To use autocisst.sh to automatically download and build the cisst library and sawIntuitiveResearchKit, please create a directory for the library, download the script file and run the script.

  ```bash
  # create folder: here we use ~/dev/cisst 
  mkdir -p ~/dev/cisst
  cd ~/dev/cisst
  # download autocisst.sh
  wget https://raw.githubusercontent.com/jhu-dvrk/sawIntuitiveResearchKit/master/share/autocisst.sh
  # run script 
  bash ./autocisst.sh
  ```

Now under the `~/dev/cisst directory`, there are three folders: `cisst-saw`, `cisstNetlib-Linux` and `build`. If you have `sawIntuitiveResearchKitQtPID` under the `dev/cisst/build/cisst/bin directory`, the build is successful. 

### 2.2. Manual build

These instructions allow you to go step by step if you are running into issues with the automatic build.
 
1. Create folder, source, build

  ```bash 
  # create folder: here we use ~/dev/cisst 
  mkdir -p ~/dev/cisst
  cd ~/dev/cisst
  # create source & build directories
  mkdir build
  ```
2. Check out code

  ```bash
  git clone https://github.com/jhu-cisst/cisst-saw.git --recursive
  ```

3. CMake and build

  ```bash
  # go to build dir 
  cd build
  # set cmake settings
  ccmake ../cisst-saw
  ```

4. CMake settings
  * Type `[c]` to configure and check error
  * Press `[e]` to exit messages view
  * Press `[t]` to show advanced settings
  * Set `CISST_HAS_CISSTNETLIB` to `ON`
  * Set `CISST_HAS_JSON` to `ON`
  * Set `CISST_USE_SI_UNITS` to `ON` (for master branch released April 28th 2015)
  * Type `[c]` to configure and then type `[e]` to exit help
  * Set `CISSTNETLIB_DOWNLOAD_NOW` to `ON`
  * Type `[c]` to configure and then type `[e]` to exit help
  * Set `CISSTNETLIB_DOWNLOAD_ARCHITECT` to `i686` for 32-bit system or `x86_64` for 64-bit system (you can figure out your system architecture using the command line `uname -i`)
  * Set `CISST_cisstRobot` to `ON`
  * Set `CISST_cisstStereoVision` to `OFF` unless you want to use cisst for your video pipeline
  * Set `SAW_sawConstraintController` to `ON`
  * Set `SAW_sawControllers` to `ON`
  * Set `SAW_sawIntuitiveResearchKit` to `ON`
  * Set `SAW_sawRobotIO1394` to `ON`
  * Set `SAW_sawTextToSpeech` to `ON`
  * Type `[c]` to configure and then type `[e]` to exit help
  * Type `[c]` to configure and then type `[e]` to exit help (you have to do it twice!)
  * Set `CMAKE_BUILD_TYPE` to `Release`
  * Type `[g]` to generate

Once you're done with the CMake configuration, you can now compile the code using the command line:

  ```bash
  make -j4 -l
  ```
Please Note that the number 4 corresponds to the number of parallel compilations, replace it by whatever number of cores you have to compile faster.

## 3. Compile in Release mode.

Compiling in Release mode can greatly improve performances.  To make sure your build is configured to use the Release mode (telling the compiler to optimize the code), go to your build directory and start CMake using:

  ```bash
  cmake -gui .
  ```

Alternatively, you can also use the CMake text based program using:

  ```bash
  ccmake .
  ```
Once CMake is started, look for the variable `CMAKE_BUILD_TYPE` and make sure it is set to `Release`.  By default, the variable is empty.


## 4. Setting up some environment variables
If you intend to do some C ++ development or coding this is a nicer environment(IDE).

cisst/saw uses a few environment variables, standard ones such as `PATH` (see http://www.linfo.org/path_env_var.html) and `LD_LIBRARY_PATH` (see http://tldp.org/HOWTO/Program-Library-HOWTO/shared-libraries.html).  To simplify the user's life, we provide scripts to set these environment variables based on individual setups.  To set your environment variables with `bash`, go in your build tree and type:

  ```bash
  . cisst/cisstvars.sh
  ```

Notes:

* The environment variables are set per shell, i.e. if you open a new terminal, you need to "source" the `cisstvars.sh` script again.

* If you want to set the cisst variables once and for all, you can modify your `.bashrc` or `.profile` configuration files.

* More info can be found on the [cisst wiki](/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake)

* If you still have some issues with your firewire permissions and need to use `sudo` to start any dVRK program, the `cisstvars.sh` won't work since `sudo` starts a new shell that won't inherit the current shell's variables.   So, fix the firewire permissions first (see https://github.com/jhu-cisst/mechatronics-software/wiki/Development-Environment)

## 5. Using Qt Creator

Qt Creator is a nice IDE (Integrated Development Environment) that works well with CMake.   There are many pages describing the process:
* http://qt-project.org/doc/qtcreator-2.8/creator-project-cmake.html (skip the Windows specifics)
* http://www.youtube.com/watch?v=I208-iVEpwk (youtube video)

### 5.1. Step by step instructions
For the research kit, the basic steps are:
* First setup your build tree manually.  Starting with an empty build tree from QtCreator won't work as you need to set some CMake options first.
* Once you have a working source and build tree (i.e. you've followed instructions above and can compile), start QtCreator and:
 * In menu `File`, `Open File or Project`, go to source directory to locate the top `cisst-saw/CMakeLists.txt`
  * Replace the build directory suggested by QtCreator by your own (e.g. `~/dev/cisst/build`)
  * Hit `Run CMake` (or depending on your software `Configure Project)

### 5.2. Settings

To compile a bit faster, you can change the  `Build Settings` and add the command line option `-j4` or whatever number of core you have.

![Build settings screenshot](/jhu-dvrk/sawIntuitiveResearchKit/wiki/qt-creator-build-settings.png)


To run and debug programs from QtCreator, go to `Run Settings` or in some cases it should be under desktop and `Run`:
* Select an executable target (i.e. program)
* Add the command line options
* Change the working directory to whatever location you have used to store your configurations file

![Run settings screenshot](/jhu-dvrk/sawIntuitiveResearchKit/wiki/qt-creator-run-settings.png)