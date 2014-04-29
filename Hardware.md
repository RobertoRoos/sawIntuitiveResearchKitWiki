# Hardware setup and testing

## 1. Unboxing

### 1.1. Arms

Please read the unboxing instructions provided by ISI at http://research.intusurg.com/dvrkwiki/index.php?title=DVRK:Docs:Main

### 1.2. Sterile adapter

***IMPORTANT:*** The PSMs have the ability to detect when a sterile adapter or tool is installed using digital inputs.   For this to work, you will need to short a couple of pins on your sterile adapters (left two pins on the photo).

Make sure you place the wire to short the two pins as deep as possible and keep the extremities of the pins as clean as possible.   If the tip of the pins are covered with soldering, you might have issues with the electric contact when inserting the adapter.

![Modified sterile adapter](/jhu-dvrk/sawIntuitiveResearchKit/wiki/modified-sterile-adapter.jpg)

### 1.2. Controllers

1. There are two types of controllers, MTM (Master Manipulators) and PSM (Patient Side Manipulator).  It is important not to mix them as the Master controllers use a 24V power supply for the first 4 actuators and a 12V power supply for the last 3 actuators while the Slave controllers use a 24V power supply for all 7 actuators.  You can verify which one is which by opening the controllers.

1. We strongly recommend to label the controllers with the name of the arm you plan to control along with the board IDs.   The board IDs should follow the convention described in the [wiki:/sawIntuitiveResearchKitTutorial/XMLConfig XML configuration page].   Each controller contains two sets of boards.    Each set is composed of a lower board (aka QLA for quad linear amps) and the upper board (FPGA + firewire connections).   The upper board (FPGA) has a rotary dial to set the board ID.   Using a small screw driver you can change the Id for each board. For example, the controller dedicated to the MTM Right should have a label with:

  ```
  MTMR (board IDs 2 - 3) 
```

1. When you will first unbox the controllers, check the internal connections.  If you find a cable with a loose end or partially unplugged you'll have to plug it back.

1. We recommend plugging all your controllers to a single power strip with a switch.  This will ensure that all controllers can be turned on or off using a single button.


## 2. Firewire

***NOTE:*** the following steps should be performed without connecting the arms to the controllers.

### 2.1. FPGA Power

For the Rev 1 controllers, there is a minor issue with the controllers and power provided via the Firewire cable.  If there are too many controllers on the daisy chain, there won't be enough power the FPGA boards properly and they might get stuck in an unstable state.  This is due to the fact that the Rev 1.1 FPGA boards (inside the Rev 1 controller box) can obtain power from the 6-pin Firewire cable or from its internal power supply -- specifically, the boards will use whichever voltage is higher. If the Firewire cable is connected to a box that is switched off, the FPGA boards will draw power from the PC via the Firewire cable.  This problem does not occur with the Rev 2 controllers, which contain Rev 1.2 FPGA boards, because those boards have a jumper (J10) to enable/disable power from  the 6-pin Firewire connector. By default, this jumper is not installed, so the FPGA boards cannot obtain power from the Firewire cable.
* For the Rev 1 controllers, if you are using a 6 pin firewire A cable, we strongly recommend to unplug the firewire cable from the PC, power on the controllers and then re-plug the firewire cable on the PC.  This way, the controllers start using their own power supplies.
* If you are using a 4 pin firewire A cable (this is often the case on PC laptops), you won't have any issue since these cables don't provide power over firewire.
* This is also not an issue with the Rev 2 controllers, unless you install jumper J10 on the FPGA boards.

### 2.2. Testing with `qladisp`

To test the firewire connections, the simplest solution is to use the command line tool `qladisp`.   Assuming that you have built [wiki:sawIntuitiveResearchKitTutorial/Build the software], `qladisp` can be found in the cisst build tree, subdirectory `build/bin`.  The program requires one or more board IDs.  The user can optionally specify a port number:
```
qladisp b1 [b2] [-pP]
```
For example, to display basic informations related to the board with Id 3 on firewire port 0, launch:
```
qladisp 3
```
You can monitor up to two boards.  Here is a screen shot for `qladisp` using boards 0 and 1:

![Using qladisp for 2 boards](/jhu-dvrk/sawIntuitiveResearchKit/wiki/qladisp-2-boards.png)


### 2.3. Testing the connections

Verify that the Firewire cables you use are good, i.e. of good quality and not too long.  We found that external cables longer that 4 meters often cause problems.  To quickly check if your Firewire chain is healthy, start the `qladisp` utility and make sure:
* all boards are listed.  You can verify the board IDs and firmware versions as well.
* the display doesn't freeze intermittently.  The `dt` value displayed on the first row is the maximum time in milliseconds between packets since the application started.  The value should remain low (i.e. a few milliseconds).  It will increase by about 80 milliseconds when you try powering the controllers.   Values above 1 second usually indicate issues with the firewire chain.

You should proceed methodically and add components one by one:
1. Test with a single controller.  Use a single firewire cable between the PC and the controller and run `qladisp` for each board in the controller (each controller box contains two boards).  Test both firewire port on the back of the controller.  Test each controller individually.
2. Test with a second controller.  Again, test each board on each controller connected.
3. If you are running into issues (missing board or freezing display), try different cables.

### 2.4. Testing the digital inputs using `qladisp`

***NOTE:*** you can do this without enabling power in `qladisp` and E-Stop in off position but you will need the arms and foot pedal plugged. 

***NOTE:*** when you connect the arm to the controller, make sure you tighten the screw on the connector (1/4 turn) otherwise you might have a loose connection missing signals.

***NOTE:*** the arguments passed to qladisp (e.g. 2 3) are board ID. The board ID is the rotary switch value (4-bit from 0 to F), which should be unique among daisy-chained controller boards. The board ID should be automatically set based on the following convention.  Not all Research Kit come with the ECM and the PSM3 but the board Id should be reserved nevertheless:

|            | MTML | MTMR | ECM | PSM1 | PSM2 | PSM3   |
|------------|------|------|-----|------|------|--------|
| Board ID 1 | 0    | 2    | 4   | 6    | 8    | 10 (A) |
| Board ID 2 | 1    | 3    | 5   | 7    | 9    | 11 (B) |

The simplest way to test these inputs is to use qladisp and make sure all digital inputs are wired.  `qladisp` has been designed for generic applications where digital inputs can be used for limit and home switches.   Since the da Vinci doesn't have limit and switch limits the digital inputs are used for buttons and pedals.  In the `qladisp`, you still need to look for `NegLim`, `PosLim` and `Home`.   Since `qladisp` allows to display up to two boards, we'll refer to `NegLim1`, `NegLim2`, ...  

On the master side, all inputs are related to the foot pedal so only one of the controllers will display any information:
using if the foot pedal is connected to MTMR controller:
```
qladisp 2 3
```
If the foot pedal is connected to MTML controller:
```
qladisp 0 1
```

You should have the following default values (no pedal pressed)
```
  NegLim1: 0xF  PosLim1: 0xF  Home1: 0xF  NegLim2: 0x7  PosLim2: 0xF  Home2: 0xF
```

Pressing foot pedal:
* Clutch -> Home2: 0xE
* Camera -> PosLim2: 0xE
* Cam+ -> Home2: 0xB
* Cam- -> Home2: 0xD
* Coag -> Home2: 0x7 (note COAG is sometimes labeled "Mono", first pedal on the right)

For the PSM:
```
qladisp 6 7
```

You should have the following default values (no button pressed, no sterile adapter, no tool, ...)
```
NegLim1: 0xF  PosLim1: 0xF  Home1: 0xD  NegLim2: 0x7  PosLim2: 0xF  Home2: 0xF
```

Pressing buttons and inserting sterile adapter or tool:
* SUJ clutch button (white button horizontal bar on the side) -> Home1: 0xE
* Tool clutch button (white button on top) -> Home1: 0x9
* Sterile adapter (modifed) -> NegLim2: 0x3
* Tool -> PosLim2: 0x7 (tool adapter should still be in so NegLim2 should still be 0x3)

## 3. Motor Power

***NOTE:*** you should perform these tests with the arms unplugged.

***NOTE*** To test individual board, you need to bypass relay, FOLLOW instruction on the [wiki:sawIntuitiveResearchKitTutorial/ESTOP#a3Debugging ESTOP] page.

Motor power can be disabled by hardware interlocks (relays, e-stop) or by software, so it is necessary to first inspect the hardware setup. Specifically, the e-stop cable harness is usually designed to daisy-chain the safety circuit between multiple controllers. Thus, if you wish to individually test controllers, you may need to temporarily modify your e-stop cable harness. See [wiki:sawIntuitiveResearchKitTutorial/ESTOP this page] for further instructions.

Once the e-stop cable harness is suitably modified, the simplest solution is to use the QLA utility `qladisp` ([https://github.com/jhu-cisst/mechatronics-software/wiki/Example-Programs Mechatronics Examples]) for each board on your hardware and try to enable/disable power(press `p` key) with and without your wired E-stop engaged.

* When the power is ON you should have:
 * `MV` LED should be green (MV stands for motor voltage)
 * `Fault x` LEDs should turn red (4 at a time)

## 4. Testing your XML configuration files

Once you have checked the overall hardware, you can [wiki:/sawIntuitiveResearchKitTutorial/Calibration calibrate and update XML configuration files].  To test the configuration files, please use the example `sawRobotIO1394QtConsole` as described in the [wiki:/sawIntuitiveResearchKitTutorial/Examples examples page].