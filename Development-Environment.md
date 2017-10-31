Development Environment Setup 
The current software is written in C/C++ and uses the libraw1394 library under Linux. See http://www.dennedy.org/libraw1394/ for libraw1394 documentation. 

# Install Ubuntu

(October 30, 2017) We recommend Ubuntu 16.04 LTS, which is a long term support version.  For ROS, we recommend Kinetic.

#  Install libraw1394 on Ubuntu 
You can use Synaptic to install packages or the command line.
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
 * Add (current) user to Group **fpgaqla**
 * Restart computer

```sh
   sudo mkdir /etc/udev/rules.d
   cd
   echo 'KERNEL=="fw*", GROUP="fpgaqla", MODE="0660"' > ~/80-firewire-fpgaqla.rules
   sudo mv ~/80-firewire-fpgaqla.rules /etc/udev/rules.d/80-firewire-fpgaqla.rules 
   sudo addgroup fpgaqla  
   sudo adduser `whoami` fpgaqla
```
