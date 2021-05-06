<!--ts-->
   * [Introduction](#introduction)
   * [Hardware not yet supported](#hardware-not-yet-supported)
   * [Supported hardware](#supported-hardware)

<!-- Added by: adeguet1, at: 2019-08-06T12:27-04:00 -->

<!--te-->

# Introduction

Acronyms used in this page are defined in [Frequently Asked Questions](/jhu-dvrk/sawIntuitiveResearchKit/wiki/FAQ).

There are two different groups of users for the dVRK mechatronics and software, those with a da Vinci Research Kit provided by Intuitive Surgical (2 MTMs, 2PSMs, stereo display and foot pedals) and those with a full decommissioned da Vinci system.  This page addresses some issues specific to the later group.

# Hardware not yet supported

It is important to notice that there are a few hardware features from the full da Vinci system that are not currently supported by the dVRK mechatronics and software:
* Video pipeline and messages in stereo display
* Audio pipeline
* Control panel and switches on the master console arm rest (on/off, scaling, arm pairing, ...)
* Height adjustment for the stereo display
* ...

# Supported hardware

On the other hand, the dVRK mechatronics and software support the camera manipulator (ECM) and will soon support the setup joints:
* For the ECM, we use a FPGA-QLA based PSM controller with a special configuration (XML file) to support the brakes.  See the [ECM page](/jhu-dvrk/sawIntuitiveResearchKit/wiki/ECM) for more details.
* For the setup joints, we use a single FPGA-QLA board connected to a board designed by Intuitive Surgical (dSIB) which can interface with all 4 setup joints.  This board has been designed and is being tested at JHU.  See the [SUJ page](/jhu-dvrk/sawIntuitiveResearchKit/wiki/SUJ) for more details.

You can also access the head sensor and control the endoscope focus using custom cables.  For more details regarding the cables (that you will need to assemble), see:
* Head sensor: [head sensor options](/jhu-dvrk/sawIntuitiveResearchKit/wiki/HeadSensor#davinci-head-sensor) (see daVinci Head Sensor section)
* Camera focus: [endoscope focus page](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Endoscope-Focus-Controller)

You will be able to switch back and forth between the mechatronics from ISI and the dVRK controllers but this requires to unplug and replug a few cables.  The user has to connect the arms, setup joints and foot pedal directly to the dVRK controllers:
* MTM cables can be found on the back of the master console, grey plastic covers need to be removed.  These cables are a bit short so you will have to keep the MTM controllers really close to the surgeon's console.
  ![Masters connectors](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/full/master-console-arm-cables.jpg)
* Foot pedal cable can be found on the front of the master console, under the stereo display and the cover needs to be removed.  Unplug the bottom part and connect it to the dVRK controller.  This cable is rather short so one might consider investing in an extension cable (e.g. https://smile.amazon.com/Monoprice-6ft-DB15-Molded-Cable/dp/B002LWJ7TA).
  ![Foot pedals connector](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/full/master-console-pedals-cable.jpg)
* :warning: On the patient side, both setup joints and arms need to be unplugged from the back of the cart to be connected to the dVRK controllers. **The long cables normally used to connect the patient cart to the master console are not used**.  The cables coming from the arms shouldn't be connected to the back to the cart, they must be connected directly to the dVRK controllers!   Since the cables coming from the arms are not very long, you will need to place all the PSM and ECM controllers really close behind the patient cart.
  ![Rack mounted PSM/ECM controllers](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/full/patient-cart-rack.jpg)
  ![Unplugged patient cart](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/full/patient-cart-arm-suj-plugs.jpg)
