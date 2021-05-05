The board versions and description of changes for each build can be found in the [FPGA](https://github.com/jhu-cisst/FPGA1394#release-notes) and [QLA](https://github.com/jhu-cisst/QLA#release-notes) release notes.  Layout is described in the [controller](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Controller-Boxes)'s page.

Some hardware [modifications](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Controller-Boxes#hardware-modifications) might be required as we're adding new features and discovering issues with older builds.

A list of sites for each build can be found in the [timeline](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Timeline).

| Build | Year | FPGA | S/N  | QLA  | S/N  | dMIB | 
| ----- | ---- | ---- | ---- | ---  | ---  | ---- |
| 0     | 2012 | 1.0  | 3116 | 1.1  | 3174 | x    |
| 1     | 2012 | 1.1  | 3792 | 1.2  | 3791 | x    |
| 2     | 2013 | 1.2  | 3985 | 1.3  | 3984 | x    |
| 3     | 2014 | 1.2  | 4266 | 1.3  | 4265 | x    |
| 4     | 2015 | 1.3  | 4652 | 1.3  | 4651 | x    |
| 5     | 2016 | 2.1  | 4981 | 1.4  | 4980 | x    |
| 6     | 2017 | 2.1  | 5496 | 1.4a | 5495 | x    |
| 7     | 2019 | 2.1  | 6007 | 1.4b | 6006 | x    |
| 8     | 2020 | 2.1  | 6547 | 1.4b | 6557 | F    |

The FPGA and QLA serial number (S/N) have the form `BBBB-xx` or `BBBB-xxx`, where `BBBB` denotes the build number (see table above) and `xx` or `xxx` is the board number, starting with `01` or `001`. For example, the first FPGA and QLA built have S/N `3116-01` and `3174-01`, respectively. The S/N is physically labeled on each board (white sticker) and can be programmed into the board EEPROM, which is accessible via software. Since Build 4, the S/N has been programmed during manufacturing. For older builds, the S/N can be programmed using the `pgm1394` utility.

All controllers come with FireWire adapters.  Ethernet adapters were introduced on FPGA 2+, i.e. in build #5 (2016).
