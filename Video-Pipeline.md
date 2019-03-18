<!--ts-->
   * [Introduction](#introduction)
   * [Sources](#sources)
      * [ISI SD endoscope](#isi-sd-endoscope)
         * [Endoscope](#endoscope)
         * [CCU front](#ccu-front)
         * [CCU back](#ccu-back)
      * [ISI HD endoscope](#isi-hd-endoscope)
         * [Endoscope](#endoscope-1)
         * [CCU front](#ccu-front-1)
         * [CCU back](#ccu-back-1)
      * [Custom cameras](#custom-cameras)
   * [Displays](#displays)
      * [ISI SD HRSV](#isi-sd-hrsv)
      * [ISI HD HRSV](#isi-hd-hrsv)
      * [Hacked ISI HRSV](#hacked-isi-hrsv)
   * [Hardware based pipeline](#hardware-based-pipeline)
   * [Software based pipeline](#software-based-pipeline)

<!-- Added by: anton, at: 2019-03-14T16:12-04:00 -->

<!--te-->

# Introduction

The dVRK community is fairly heterogeneous, each site has different needs and hardware available.  The goal of this page is to list the different options and solutions used on different sites.  If you are a member of the dVRK community, feel free to contribute to this document.

# Sources

## ISI SD endoscope

This is the most common endoscope found on Classic/Standard full da Vinci system.  The camera heads come with a motorized mechanism for focus.  To process the raw video signals, there are two CCUs (Camera Control Units), usually bolted to the video tower.  One of the CCU model used is the Panasonic GP US742 (at least in the US).   The video outputs on the back of the CCUs are all analog and the simplest option is to use the [S-Video](https://www.lifewire.com/s-video-definition-1082148) connection.  On US system, the signal is NTSC.

### Endoscope

![SD endoscope with Panasonic Cameras](/jhu-dvrk/sawIntuitiveResearchKit/wiki/video-panasonic-sd-endoscope.jpg)

### CCU front

CCUs are in the middle, the endoscope focus controller is on top.  A light source is below.
![SD Panasonic CCUs front](/jhu-dvrk/sawIntuitiveResearchKit/wiki/video-panasonic-sd-ccu-front.jpg)

### CCU back

![SD Panasonic CCUs back](/jhu-dvrk/sawIntuitiveResearchKit/wiki/video-panasonic-sd-ccu-back.jpg)

## ISI HD endoscope

Intuitive started to release an HD endoscope along the da Vinci S (see [FAQ](/jhu-dvrk/sawIntuitiveResearchKit/wiki/FAQ) for different generations of systems).  The endoscope and mount is actually the same as the Standard SD endoscope but the camera heads are different.  They still use a motorized mechanism for focus but the sensor now support full HD (1080x1920).  To process the raw signal from the camera, there are also two CCUs.  Intuitive used different CCUs over time.  Some systems come with the Ikegami CCUs.  The video output is SDI.

### Endoscope

![HD endoscope with Ikegami Cameras](/jhu-dvrk/sawIntuitiveResearchKit/wiki/video-ikegami-hd-endoscope.jpg)

### CCU front

![HD Ikegami CCUs front](/jhu-dvrk/sawIntuitiveResearchKit/wiki/video-ikegami-hd-ccu-front.jpg)

### CCU back

![HD Ikegami CCUs back](/jhu-dvrk/sawIntuitiveResearchKit/wiki/video-ikegami-hd-ccu-back.jpg)

## Custom cameras

Contributions from the dVRK community can go here.
 
# Displays

At the other end of the video pipeline, we have the stereo viewer.  It can be either the HRSV provided by Intuitive or your own display (3D or not).

## ISI SD HRSV

The da Vinci Classic came with two SD CRT (Cathodic Ray Tube) monitors, their resolution is 640x480.  The connectors on the back of the monitor is not totally standard and each group will have to build a custom cable to provide a VGA interface.   One can find instructions on the [ISI Research Wiki](https://research.intusurg.com/index.php/DVRK:Topics:StereoViewer) **[Password protected]**.
 
## ISI HD HRSV

The da Vinci S consoles came with two higher resolution CRTs (1024x768).  These monitors have a standard VGA input and you don't need any special adapter.

## Hacked ISI HRSV

Some groups have replaced the CRTs in the HRSV provided by ISI with a pair of LCD panels.  The main issue is to find the proper size of monitor, i.e. roughly 14" with 4/3 ratio and ideally a fairly high resolution.

  * The group at JHU found some replacement parts for old laptops and LCD controllers that work fine (see [ISI Research Wiki](https://research.intusurg.com/index.php/DVRK:Topics:StereoViewerLCD) **[Password protected]**).  Unfortunately, some of these parts are getting harder to find online.

  * The group at CUHK used a pair of DELL monitors ([1505FP](https://www.amazon.com/Dell-1505FP-15-IN-LCD-MONITOR/dp/B0026JQ85Y)) as replacement.  The resolution is 1024x768.

![HRSV with DELL 15" monitors](/jhu-dvrk/sawIntuitiveResearchKit/wiki/video-hrsv-dell-15.jpg)

# Hardware based pipeline

The hardware based pipeline is the simplest solution and likely the cheapest.  It is based on off-the-shelf converters that can be placed between the CCUs and the monitors.  The main drawback is that one can not modify nor process the videos coming from the CCUs before displaying on the HRSV monitors.

![Hardware based video pipeline](/jhu-dvrk/sawIntuitiveResearchKit/wiki/video-pipeline-hardware.png)

You will need some standard cables as well as a pair of video converters (one for the left video and one for the right video):
 * S-Video to VGA, one could use: --- find suggestion from community
 * SDI to VGA, one could use: --- find suggestion from community

# Software based pipeline 

If you need to process the video inputs (e.g. camera calibration, filtering, 3D reconstruction...) or modify the videos being displayed (camera distortion compensation, overlays, messages...) you will need to put a PC in the middle of the video pipeline.   In this case, the main 3 "components" on the PC are:
 * Frame grabbers
 * Software
 * Video output (most likely a graphic card with 2 extra outputs)

![Software based video pipeline](/jhu-dvrk/sawIntuitiveResearchKit/wiki/video-pipeline-software.png)

For the frame grabbers and software, you can find some documentation on the [dVRK ROS GitHub repository](https://github.com/jhu-dvrk/dvrk-ros/blob/master/dvrk_robot/video.md).