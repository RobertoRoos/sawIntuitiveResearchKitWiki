<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](https://github.com/thlorenz/doctoc)*

- [Install Ubuntu](#install-ubuntu)
- [Install libraw1394 on Ubuntu](#install-libraw1394-on-ubuntu)
- [Set permission for 1394 device](#set-permission-for-1394-device)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

Development Environment Setup 
The current software is written in C/C++ and uses the libraw1394 library under Linux. See http://www.dennedy.org/libraw1394/ for libraw1394 documentation. 

# Install Ubuntu

(October 30, 2017) We recommend Ubuntu 16.04 LTS, which is a long term support version.  For ROS, we recommend Kinetic.

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
 * Create Group **fpgaqla**
 * Optionally add the current user to Group **fpgaqla**
 * Restart computer

The following script should be run only once per computer and performs the steps described above:
```sh
   sudo mkdir /etc/udev/rules.d # create a directory if needed
   cd
   echo 'KERNEL=="fw*", GROUP="fpgaqla", MODE="0660"' > ~/80-firewire-fpgaqla.rules # create the rule
   sudo mv ~/80-firewire-fpgaqla.rules /etc/udev/rules.d/80-firewire-fpgaqla.rules  # move the rule in the proper directory
   sudo addgroup fpgaqla          # create the group with read-write access to /dev/fw*
   sudo adduser `whoami` fpgaqla  # add current user to the group
```

For all additional users, you will need to add the new user to the group.   To find the user id, one can either use the command `id` or do an `ls /home`.   Once the user id is known, someone with sudo privileges should do:
```sh
   sudo adduser put_the_new_user_id_here fpgaqla
```
Once a user has been added to the `fpgaqla` group, they need to logout/login so the group membership can take effect.   To check if the group membership is correct, the user can use the shell command `id`.