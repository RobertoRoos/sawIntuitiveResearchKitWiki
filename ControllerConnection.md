<!--ts-->
   * [Introduction](#introduction)
   * [FireWire](#firewire)
      * [FireWire Adapter](#firewire-adapter)
      * [Install Ubuntu](#install-ubuntu)
      * [Set permissions for FireWire devices](#set-permission-for-firewire-devices)
         * [Convenient solution](#convenient-solution)
         * [Safer solution](#safer-solution)
      * [Testing connectivity](#testing-connectivity)
         * [qladisp](#qladisp)
         * [ls -l /dev/fw*](#ls--l-devfw)
         * [dmesg -w](#dmesg--w)
         * [udevadm](#udevadm)
   * [Ethernet UDP](#ethernet-udp)
      * [Adapter and configuration](#adapter-and-configuration)
      * [Testing connectivity](#testing-connectivity-1)
         * [qladisp](#qladisp-1)
         * [ping](#ping)

<!-- Added by: anton, at: 2021-03-16T18:17-04:00 -->

<!--te-->

# Introduction

The current software is written in C/C++ and uses the libraw1394 library under Linux (FireWire is also known as 1394). See http://www.dennedy.org/libraw1394/ for libraw1394 documentation.  FireWire is the preferred way to communicate with the dVRK controllers.

Starting with dVRK Software Version 2.0, Ethernet UDP is also supported, provided that there is at least one FPGA V2.x board (with Ethernet jack).  This approach is not as heavily tested as FireWire so make sure your computer also has a FireWire adapter.

# FireWire

**Important Notes:**
* There is no such thing as a FireWire to USB adapter so you will need a computer with a FireWire adapter
* FireWire is not supported by virtualization software (VMWare Fusion, Parallel...) so you will need to use a native Linux OS (we strongly recommend Ubuntu LTS)

## FireWire Adapter

The dVRK controllers use FireWire as the fieldbus, and you will need a FireWire adapter. Due to the fact that FireWire is a sophisticated protocol, some chipset implementations are not fully functional and have various issues such as dropping packets and supporting a limited number of FireWire nodes. We **STRONGLY** recommend adapters with chipsets from Texas Instruments (see PC configuration in [FAQ](/jhu-dvrk/sawIntuitiveResearchKit/wiki/FAQ)).

To get the chipset model of your FireWire card: 
```sh
lshw
```
Example FireWire Section:
```
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

## Install Ubuntu

We recommend Ubuntu 18.04 LTS but one can also use 16.04 or 20.04 with the latest dVRK (2020).  For ROS, we recommend Kinetic on 16.04, Melodic on 18.04 and Noetic on 20.04.

You can then install *libraw1394*.  This has to be done only once by a user with sudo privileges
```sh
 sudo apt-get install libraw1394-dev
```

## Set permissions for FireWire devices

If you installed linux recently, you are likely to have Kernel 3.x with the new Juju firewire driver stack, which means the firewire device would be /dev/fw* instead of the old /dev/raw1394*. In order to run the control software without root permission, please do the following steps:
 * Create `/etc/udev/rules.d` folder if it's not there
 * Add rules for `/dev/fw*` devices (or `/dev/raw1394*` for Ubuntu versions prior to 10.04)
 * Optionally create group **fpgaqla**
 * Optionally add the current user to Group **fpgaqla**
 * Reload udev rules

**Note**: pick one of the two solutions described below!

### Convenient solution

If you or your institution doesn't care about who can access the FireWire devices on your system, you can grant anyone to have read and write permissions on all FireWire devices.  This is simpler to manage and should satisfy the requirements of most if not all dVRK users.

The following script should be run only once per computer:
```sh
   sudo mkdir /etc/udev/rules.d # create a directory if needed
   cd
   echo 'KERNEL=="fw*", GROUP="fpgaqla", MODE="0666"' > ~/80-firewire-all.rules # create the rule
   sudo mv ~/80-firewire-all.rules /etc/udev/rules.d/80-firewire-all.rules  # move the rule in the proper directory
   sudo addgroup fpgaqla          # create the group with read-write access to /dev/fw*
   sudo udevadm control --reload-rules # apply new rules
```

### Safer solution

If you or your institution really, really cares about who can access the FireWire devices on your computer you can create a dedicated unix group to control who can access the FireWire devices.

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

## Testing connectivity

### `qladisp`

**Note:** `qladisp` is part of the dVRK software, so you will to build the software first.  See [Build with ROS](/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild).

There are a few ways to test that your controllers are properly connected.  You can start with the command line application provided with the dVRK software `qladisp`.  Just type `qladisp` in a terminal (without options) and the output should show the list of boards found with their board Id and firmware version.  For example:
```sh
Trying to detect boards on port:
ParseOptions: no option provided, using default fw:0
FirewirePort::Init: number of ports = 1
  Port 0: /dev/fw12, 14 nodes
FirewirePort::Init: successfully initialized port 0
Using libraw1394 version 2.1.2
FirewirePort::Init: successfully disabled cycle start packet
FirewirePort::InitNodes: base node id = ffc0
BasePort::ScanNodes: building node map for 13 nodes:
  Node 0, BoardId = c, Firmware Version = 7
  Node 1, BoardId = a, Firmware Version = 7
  Node 2, BoardId = b, Firmware Version = 7
  Node 3, BoardId = 5, Firmware Version = 7
  ...
```
This is the output for a full system.  For most systems, you should see two boards per controller/arm.

### `ls -l /dev/fw*`

If `qladisp` doesn't work, check that all FireWire devices have been found and created with the correct files permissions using `ls -al /dev/fw*`.   The output should look like:
```sh
crw-rw-rw- 1 root fpgaqla 243,  0 Feb 12 09:31 /dev/fw0
crw-rw-rw- 1 root fpgaqla 243,  1 Mar  2 11:45 /dev/fw1
crw-rw-rw- 1 root fpgaqla 243,  2 Mar  2 11:45 /dev/fw2
crw-rw-rw- 1 root fpgaqla 243,  3 Mar  2 11:45 /dev/fw3
crw-rw-rw- 1 root fpgaqla 243,  4 Mar  2 11:45 /dev/fw4
...
```

You should have two `fw` devices created for each controllers (except 1 for SUJ controller).  Note that `fw0` is the FireWire adapter on the PC itself.  If you have multiple FireWire cards on your PC, the first nodes will correspond to the cards on the PC (e.g. for 2 cards, `fw0` and `fw1`).

**Very important note:** The `fw` devices should be numbered contiguously, i.e. there shouldn't be any gap between the numbers.  It there are some gaps, the FireWire bus initialization likely failed.  This can happen when FireWire cables are unplugged and re-plugged too fast for the kernel, make sure you wait a few seconds between steps.  If this happens, you can force a bus reset by unplugging, waiting 5 seconds and re-plugging the FireWire cable on your PC.

### `dmesg -w`

You can also monitor the kernel messages using the command `dmesg -w`.  Start the command in a separate terminal and leave it alone while plugging/unplugging the FireWire cables.  You should see messages re. the creation of FireWire devices:
```sh
[2413623.229296] firewire_core 0000:09:04.0: created device fw8: GUID fa610e3f00000007, S400
[2413623.229365] firewire_core 0000:09:04.0: created device fw11: GUID fa610e2f00000007, S400
...
```

The example above shows the output for firmware 7+.  With older firmware versions, you will get some warnings/errors you can ignore:
```
[455344.152159] firewire_core 0000:04:00.0: skipped unsupported ROM entry 879e7ffe at fffff00005c0
[455344.152178] firewire_core 0000:04:00.0: skipped unsupported ROM entry 99df7fe9 at fffff00005c0
...
```

For firmware 7, the output is quite useful:
  * **fa610e**3f00000007: **fa610e** is the vendor Id, i.e. [JHU LCSR](https://www.lcsr.jhu.edu)
  * fa610e**3**f00000007: **3** is the board Id
  * fa610e3**f**00000007: **f** is the FPGA board type, i.e. **f** for FireWire only, **e** for boards with Ethernet adapter (see [controller versions](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Board-Versions))
  * fa610e3f0000000**7**: **7** is the firmware version

### `udevadm`

Lastly, once the controllers are properly connected you can check all the attributes using:
```sh
udevadm info --name=/dev/fw1 --attribute-walk  | less
```

The output will include the info provided by `dmesg` and more:
```
 looking at device '/devices/pci0000:00/0000:00:1c.4/0000:03:00.0/0000:04:00.0/fw1':
    KERNEL=="fw1"
    SUBSYSTEM=="firewire"
    DRIVER==""
    ATTR{guid}=="0xfa610e6f00000007"
    ATTR{is_local}=="0"
    ATTR{model}=="0x000001"
    ATTR{model_name}=="FPGA1/QLA"
    ATTR{units}==""
    ATTR{vendor}=="0xfa610e"
    ATTR{vendor_name}=="JHU LCSR"
```
The above indicates that `fw1` has FPGA V1.x (no Ethernet). For FPGA V2.x (Ethernet), the `model` will be 2 and the `model_name` will be `"FPGA2/QLA"`.

# Ethernet UDP

## Adapter and configuration

You will need a network adapter dedicated to the communication with the dVRK controller (e.g. a PCIe Ethernet adapter), it can't be plugged in a router or hub and used to access Internet.  Therefore we recommend to install 2 network adapters on your computer, one for the LAN/WAN and one for the dVRK.  The dVRK dedicated network adapter will be directly connected to one of the dVRK controllers on the FireWire chain.  Please avoid having dVRK controllers connected to 2 different PCs through Ethernet.

We recommend a built-in network adapter (e.g. a PCIe Ethernet adapter).  We don't have any specific recommendation for the chipset, just make sure it is supported by Linux.  USB3/USB-C network adapters might work too but we don't have any extensive experience with these.

We also recommend a native OS (as opposed to a VM).  If you succeed at running the dVRK software in a VM, let us know.

Finally, you will need to configure the dVRK dedicated network adapter to use " Link-Local Only"

### Ubuntu

Start the application `sudo nm-connection-editor` (this should work on Ubuntu 16.04, 18.04 and 20.04).  Select the Ethernet adapter you want to configure:
* In tab "Ethernet", change MTU to 3000.  The default is 1500 and is not enough if you have a full daVinci (2 MTMS, 3 PSMs, ECM and SUJ).
* In tab "IPv4 Settings", change "method" to "Link-Local Only"

### MacOS

Running the dVRK on MacOS is experimental and not that useful.  This being said, there is no network configuration required on MacOS.  Somehow the OS figures out the adpater should be Link-Local by itself. 

### Windows

## Testing connectivity

### `qladisp`

`-pudp`

### `ping`

