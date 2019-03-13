# Introduction

The dVRK community is fairly heterogeneous, each site has different needs and hardware available.  The goal of this page is to list the different options and solutions used on different sites.  If you are a member of the dVRK community, feel free to contribute to this document.

# Sources

## ISI SD endoscope

This is the most common endoscope found on Classic/Standard full da Vinci system.  The camera heads come with a motorized mechanism for focus.  To process the raw video signals, there are two CCUs (Camera Control Units), usually bolted to the video tower.  One of the CCU model used is the Panasonic GP US742 (at least in the US).   The video outputs on the back of the CCUs are all analog and the simplest options is to use the s-video connection.  On US system, the signal is NTSC.

## ISI HD endoscope

Intuitive started to release an HD endoscope along the da Vinci S (see [FAQ](jhu-dvrk/sawIntuitiveResearchKit/wiki/FAQ) for different generations of systems).  The endoscope and mount is actually the same as the Standard SD endoscope but the camera heads are different.  They still use a motorized mechanism for focus but the sensor now support full HD (1080x1920).  To process the raw signal from the camera, there are also two CCUs.  Intuitive used different CCUs over time.  Some systems come with the Ikegami CCUs.  The video output is SDI.

## Custom cameras
 
# Displays

At the other end of the video pipeline, we have the stereo viewer.  It can be either the HRSV provided by Intuitive or your own display (3D or not).

## ISI SD HRSV

The da Vinci Classic came with two SD CRT (Cathodic Ray Tube) monitors, their resolution is 640x480.  The connectors on the back of the monitor is not totally standard and each group will have to build a custom cable to provide a VGA interface.   One can find instructions on the [ISI Research Wiki](https://research.intusurg.com/index.php/DVRK:Topics:StereoViewer) [Password protected].
 
## ISI HD HRSV

Some of the da Vinci S consoles came with two higher resolution CRTs (1024x768).  These monitors have a standard VGA input.

## Hacked ISI HRSV

Some groups have replaced the CRTs provided by ISI with a pair of LCD panels.  The issue is to find the proper size of monitor, i.e. roughly 14" with 4/3 ratio and ideally a fairly high resolution.

  * The group at JHU found some replacement part for older laptop and LCD controllers that work fine (see [ISI Research Wiki](https://research.intusurg.com/index.php/DVRK:Topics:StereoViewerLCD) [Password protected].  Unfortunately, some of the parts used are getting harder to find.

  * Other options?

# Hardware based pipeline

The hardware based pipeline is the simplest solution and likely the cheapest.  It is based on off-the-shelf converters that can be placed between the CCUs and the monitor.  The main drawback is that one can not modify nor use the video coming from the CCUs before displaying on the HRSV monitors.

![Hardware based video pipeline](/jhu-dvrk/sawIntuitiveResearchKit/wiki/video-pipeline-hardware.png)

The main problem is to find a proper converter (two for left and right videos):
 * S-Video to VGA, one could use: --- find suggestion from community
 * SDI to VGA, one could use: --- find suggestion from community

# Software base pipeline 

![Software based video pipeline](/jhu-dvrk/sawIntuitiveResearchKit/wiki/video-pipeline-software.png)
