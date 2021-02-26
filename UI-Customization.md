# UI Customization

## GUI

Starting with the dVRK 2.0, we added support for a "pseudo" dark mode and Qt styles.  If you're using a Qt based window manager you will likely not use these features (e.g. KDE).  For the default Ubuntu window managers, these extra options allow some user customization of the dVRK GUIs.

To activate the dark mode, add the option `-D` when starting the dVRK console application.  This applies to the plain application `sawIntuitiveResearchKitQtConsoleJSON` as well as the ROS node `dvrk_robot dvrk_console_json`.

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-style-dark.png"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-style-dark.png" width="600"></a>

To change the Qt style, use the option `-S`.  To figure out which Qt styles are available, use a dummy style that doesn't exist: `-S unicorn` (let's hope no one will ever create a Qt style named "unicorn").  The application will fail to launch but it will display a list of available styles.

To install some extra styles:
* Ubuntu 18.04: `sudo apt install qt5-style-plugins kde-style-oxygen-qt5 kde-style-qtcurve-qt5`
* Ubuntu 20.04: `sudo apt install qt5-style*`

Since we use Qt for all GUIs, these options should work on all OSs but we've only tested them on Linux.

* Oxygen style on Ubuntu 18.04<br>
  <a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-style-oxygen.png"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-style-oxygen.png" width="600"></a>
* QtCurve style on Ubuntu 18.04<br>
  <a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-style-qt-curve.png"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-style-qt-curve.png" width="600"></a>
*  ukui-dark style on Ubuntu 20.04<br>
  <a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-style-ukui-dark.png"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-style-ukui-dark.png" width="600"></a>
## Audio

Starting with the dVRK 1.6 we added some audio feedback for some console events (operator present, pedal pressed...).  The dVRK applications rely on the [*sawTextToSpeech* component](https://github.com/jhu-saw/sawTextToSpeech) for both text-to-speech and beeps.  These commands are exposed by the ROS node under the topics:
* `/console/string_to_speech`: expects a plain string
* `/console/beep`: expects a `std_msgs/Float64MultiArray` with 3 values: duration (in seconds), frequency and volume (0 to 1).   For example `data: [0.5, 3000.0, 1.0]`

See the [*sawTextToSpeech* page](https://github.com/jhu-saw/sawTextToSpeech) for requirements.