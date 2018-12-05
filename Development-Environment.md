<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](https://github.com/thlorenz/doctoc)*

- [Install Ubuntu](#install-ubuntu)
- [Install libraw1394 on Ubuntu](#install-libraw1394-on-ubuntu)
- [Set permission for 1394 device](#set-permission-for-1394-device)
  - [Convenient solution](#convenient-solution)
  - [Safer solution](#safer-solution)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

Development Environment Setup 
The current software is written in C/C++ and uses the libraw1394 library under Linux. See http://www.dennedy.org/libraw1394/ for libraw1394 documentation. 

# FireWire Adapter
The dVRK controllers use FireWire as the fieldbus, and you will need a FireWire adapter. Due to the fact that FireWire is a sophisticated protocol, some chipset implementation is not fully functional and has various issues such as dropping packets and supporting a limited number of FireWire nodes. We **STRONGLY** recommend adapters with chipsets from Texas Instrument, such as the PEX1394A2V 2-Port PCI Express FireWire Card from StarTech. 

Reference: 

[1] PreSonus: Approved FireWire chipsets for FireStudio™-series interfaces and StudioLive™ mixers. 

[2] StarTech FireWire card: [Amazon Link](https://www.amazon.com/Port-1394a-Express-FireWire-Card/dp/B00IAABHFE/ref=sr_1_9?ie=UTF8&qid=1543974002&sr=8-9&keywords=pcie+firewire)

# Install Ubuntu

(October 30, 2017) We recommend Ubuntu 16.04 LTS, which is a long-term support version.  For ROS, we recommend Kinetic.

#  Install libraw1394 on Ubuntu

This has to be done only once by a user with sudo privileges.  You can use Synaptic to install packages or the command line.
 * **Synaptic:** Search for raw1394 and install the libraw1394-dev package.
 * **Terminal:** On the command line, install it with:
   ```sh
   sudo apt-get install libraw1394-dev
   ```

# Set permission for 1394 device 
If you installed linux recently, you are likely to have Kernel 3.x with the new Juju firewire driver stack, which means the firewire device would be /dev/fw* instead of the old /dev/raw1394*. In order to run the control software without root permission, please do the following steps:
 * Create `/etc/udev/rules.d` folder if it's not there
 * Add rules for `/dev/fw*` devices (or `/dev/raw1394*` for Ubuntu versions prior to 10.04)
 * Optionally create group **fpgaqla**
 * Optionally add the current user to Group **fpgaqla**
 * Reload udev rules

**Note**: pick one of the two solutions described below!

## Convenient solution

If you or your institution doesn't care about who can access the FireWire devices on your system, you can grant anyone to have read and write permissions on all FireWire devices.  This is simpler to manage but not as safe as the solution described in the next section.

The following script should be run only once per computer:
```sh
   sudo mkdir /etc/udev/rules.d # create a directory if needed
   cd
   echo 'KERNEL=="fw*", GROUP="fpgaqla", MODE="0666"' > ~/80-firewire-all.rules # create the rule
   sudo mv ~/80-firewire-all.rules /etc/udev/rules.d/80-firewire-all.rules  # move the rule in the proper directory
   sudo addgroup fpgaqla          # create the group with read-write access to /dev/fw*
   sudo udevadm control --reload-rules # apply new rules
```

## Safer solution

If you or your institution cares about who can access the FireWire devices on your computer you can create a dedicated unix group to control who can access the FireWire devices.

The following script should be run only once per computer and performs the steps described above:
```sh
   sudo mkdir /etc/udev/rules.d # create a directory if needed
   cd
   echo 'KERNEL=="fw*", GROUP="fpgaqla", MODE="0660"' > ~/80-firewire-fpgaqla.rules # create the rule
   sudo mv ~/80-firewire-fpgaqla.rules /etc/udev/rules.d/80-firewire-fpgaqla.rules  # move the rule in the proper directory
   sudo addgroup fpgaqla          # create the group with read-write access to /dev/fw*
   sudo udevadm control --reload-rules # apply new rules
   sudo adduser `whoami` fpgaqla  # add current user to the group
```

For all additional users, you will need to add the new user to the group.   To find the user id, one can either use the command `id` or do an `ls /home`.   Once the user id is known, someone with sudo privileges should do:
```sh
   sudo adduser put_the_new_user_id_here fpgaqla
```
Once a user has been added to the `fpgaqla` group, they need to logout/login so the group membership can take effect.   To check if the group membership is correct, the user can use the shell command `id`.