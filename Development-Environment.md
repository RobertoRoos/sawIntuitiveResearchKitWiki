<!--ts-->
   * [FireWire Adapter](#firewire-adapter)
   * [Install Ubuntu](#install-ubuntu)
   * [Install libraw1394 on Ubuntu](#install-libraw1394-on-ubuntu)
   * [Set permission for 1394 device](#set-permission-for-1394-device)
      * [Convenient solution](#convenient-solution)
      * [Safer solution](#safer-solution)

<!-- Added by: anton, at: 2021-01-28T16:01-05:00 -->

<!--te-->

Development Environment Setup 
The current software is written in C/C++ and uses the libraw1394 library under Linux. See http://www.dennedy.org/libraw1394/ for libraw1394 documentation. 

# FireWire Adapter
The dVRK controllers use FireWire as the fieldbus, and you will need a FireWire adapter. Due to the fact that FireWire is a sophisticated protocol, some chipset implementations are not fully functional and have various issues such as dropping packets and supporting a limited number of FireWire nodes. We **STRONGLY** recommend adapters with chipsets from Texas Instruments (see links below).

To get the chipset model of your FireWire card: 
```sh
# use lshw to get all hardware info and look for the firewire section. 
$ lshw 
-------------
Example FireWire Section.
*-firewire
                description: FireWire (IEEE 1394)
                product: TSB12LV23 IEEE-1394 Controller
                vendor: Texas Instruments
                physical id: 0
                bus info: pci@0000:03:00.0
                version: 00
                width: 32 bits
                clock: 33MHz
                capabilities: ohci bus_master cap_list
                configuration: driver=firewire_ohci latency=32 maxlatency=4 mingnt=3
                resources: irq:21 memory:fe404000-fe4047ff memory:fe400000-fe403fff
```

Reference: 

[1] PreSonus: Approved FireWire chipsets for FireStudio™-series interfaces and StudioLive™ mixers. 

[2] StarTech FireWire card (old, hard to find): [Amazon link](https://www.amazon.com/Port-1394a-Express-FireWire-Card/dp/B00IAABHFE/ref=sr_1_9?ie=UTF8&qid=1543974002&sr=8-9&keywords=pcie+firewire).

[3] Syba FireWire card: [Amazon link](https://www.amazon.com/gp/product/B002S53IG8/).  This card comes with regular and low profile plate so it can also be used in low profile desktop computers. 

# Install Ubuntu

We recommend Ubuntu 18.04 LTS but one can also use 16.04 or 20.04 with the latest dVRK (2020).  For ROS, we recommend Kinetic on 16.04, Melodic on 18.04 and Noetic on 20.04.

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

# Testing connectivity

There are a few ways to test that your controllers are properly connected.  You can start with the higher level command line application provided along the dVRK software, `qladisp`.  Just type `qladisp` in a terminal (without options) and the output should show the list of boards found with their board Id and firmware version.  For example:
```sh
Trying to detect boards on port:
ParseOptions: no option provided, using default fw:0
FirewirePort::Init: number of ports = 1
  Port 0: /dev/fw12, 14 nodes
FirewirePort::Init: successfully initialized port 0
Using libraw1394 version 2.1.2
FirewirePort::Init: successfully disabled cycle start packet
FirewirePort::InitNodes: base node id = ffc0
BasePort::ScanNodes: building node map for d nodes:
  Node 0, BoardId = c, Firmware Version = 7
  Node 1, BoardId = a, Firmware Version = 7
  Node 2, BoardId = b, Firmware Version = 7
  Node 3, BoardId = 5, Firmware Version = 7
  Node 4, BoardId = 4, Firmware Version = 7
  Node 5, BoardId = 8, Firmware Version = 7
  Node 6, BoardId = 9, Firmware Version = 7
  Node 7, BoardId = 6, Firmware Version = 7
  Node 8, BoardId = 7, Firmware Version = 7
  Node 9, BoardId = 3, Firmware Version = 7
  Node a, BoardId = 2, Firmware Version = 7
  Node b, BoardId = 1, Firmware Version = 7
  Node c, BoardId = 0, Firmware Version = 7
```
This is the output for a full system.  For most systems, you should see two boards per controller/arm.

If this doesn't work, check that all FireWire devices have been found and created with the correct files permissions using `ls -al /dev/fw*`.   The output should look like:
```sh
crw-rw-rw- 1 root fpgaqla 243,  0 Feb 12 09:31 /dev/fw0
crw-rw-rw- 1 root fpgaqla 243,  1 Mar  2 11:45 /dev/fw1
crw-rw-rw- 1 root fpgaqla 243, 10 Mar  2 11:45 /dev/fw10
crw-rw-rw- 1 root fpgaqla 243, 11 Mar  2 11:45 /dev/fw11
crw-rw-rw- 1 root fpgaqla 243, 12 Mar  2 11:45 /dev/fw12
crw-rw-rw- 1 root fpgaqla 243, 13 Mar  2 11:45 /dev/fw13
crw-rw-rw- 1 root fpgaqla 243,  2 Mar  2 11:45 /dev/fw2
crw-rw-rw- 1 root fpgaqla 243,  3 Mar  2 11:45 /dev/fw3
crw-rw-rw- 1 root fpgaqla 243,  4 Mar  2 11:45 /dev/fw4
crw-rw-rw- 1 root fpgaqla 243,  5 Mar  2 11:45 /dev/fw5
crw-rw-rw- 1 root fpgaqla 243,  6 Mar  2 11:45 /dev/fw6
crw-rw-rw- 1 root fpgaqla 243,  7 Mar  2 11:45 /dev/fw7
crw-rw-rw- 1 root fpgaqla 243,  8 Mar  2 11:45 /dev/fw8
crw-rw-rw- 1 root fpgaqla 243,  9 Mar  2 11:45 /dev/fw9
```

You should have two `fw` devices created for each controllers (except 1 for SUJ controller).  Note that `fw0` is the FireWire adapter on the PC itself.  If you have multiple FireWire cards on your PC, the first nodes will correspond to the cards on the PC (e.g. for 2 cards, `fw0` and `fw1`).

**Very important note:** The `fw` devices should be numbered contiguously, i.e. there shouldn't be any gap between the numbers.  It there are some gaps, the FireWire bus initialization likely failed.   You can force a bus reset by unplugging and re-plugging the FireWire cable on your PC.

Lastly, you can also monitor the kernel messages using the command `dmesg -w`.  Start the command in a separate terminal and leave it alone while plugging/unplugging the FireWire cables.  You should see messages re. the creation of FireWire devices:
```sh
[2413623.229296] firewire_core 0000:09:04.0: created device fw8: GUID fa610e3f00000007, S400
[2413623.229365] firewire_core 0000:09:04.0: created device fw11: GUID fa610e2f00000007, S400
[2413623.229382] firewire_core 0000:09:04.0: created device fw9: GUID fa610e7e00000007, S400
[2413623.229398] firewire_core 0000:09:04.0: created device fw6: GUID fa610e8f00000007, S400
```

The example above shows the output for firmware 7+.  With older firmware versions, you will get some warnings/errors you can ignore.  For firmware 7, the output is quite useful:
  * **fa610e**3f00000007: **fa610e** is the vendor Id, i.e. JHU/dVRK
  * fa610e**3**f00000007: **3** is the board Id
  * fa610e3**f**00000007: **f** is the FPGA board type, i.e. **f** for FireWire only, **e** for boards with Ethernet adapter 
  * fa610e3f0000000**7**: **7** is the firmware version 