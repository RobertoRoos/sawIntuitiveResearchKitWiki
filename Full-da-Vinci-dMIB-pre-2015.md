# Introduction

These instructions are only needed if you need to support the SUJ and your dVRK controllers are pre 2015.   See [the full da Vinci page](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Full-da-Vinci).

# Hack

The dMIB is located on the back on the controller enclosure:

![dMIB in enclosure](/jhu-dvrk/sawIntuitiveResearchKit/wiki/dmib-in-enclosure.jpg).

You should be able to modify the dMIB without removing it from the enclosure.  Unscrew the cables connected to the dMIB and set it up vertically resting on its back panel.  The modifications required are:
* Wire (short) pin K3 to R3
* Wire pin N3 to back of spare digital input on side of board (see photo).

Please note that some dMIB have the letter labels (A, B, C, E, ...) off by one.  So make sure you rely on the photos to identify the proper pins.
* dMIB modified at JHU ![Hacked board at JHU](/jhu-dvrk/sawIntuitiveResearchKit/wiki/dmib-ecm-jhu.jpg)
* dMIB modified at ISI ![Hacked board at ISI](/jhu-dvrk/sawIntuitiveResearchKit/wiki/dmib-ecm-isi.jpg)