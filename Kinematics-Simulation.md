## About
This page documents how to run kinematics simulation of da Vinci Reseach Kit (dVRK).

## Build 
Please follow the [catkin build](/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild) page to download and build the software stack. 

```sh
# Simplified version 
# NOTE: 
#   1) use your own ROS workspace path
#   2) select the appropriate releases if necessary
cd /path/to/your/ros/workspace && cd src
git clone https://github.com/jhu-dvrk/dvrk-ros
git clone https://github.com/jhu-cisst/cisst-saw --recursive
catkin build
```

## Run the simulation 

Please see [dvrk_robot](/jhu-dvrk/dvrk-ros/tree/master/dvrk_robot) page for more details. 

```sh
# NOTE
#  1) please use your own ROS workspace path
#  2) please use the right console json file (see sawIntuitiveResearchKit/share foler)

# MTML
roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=MTML \
config:=/path/to/ros/workspace/src/cisst-saw/sawIntuitiveResearchKit/share/console-MTML_KIN_SIMULATED.json

# PSM1 
roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=PSM1 \
config:=/path/to/ros/workspace/src/cisst-saw/sawIntuitiveResearchKit/share/console-PSM1_KIN_SIMULATED.json

# ECM
roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=ECM \
config:=/path/to/ros/workspace/src/cisst-saw/sawIntuitiveResearchKit/share/console-ECM_KIN_SIMULATED.json

# -------------------------------
# 1 real arm and 1 simulated arm
# -------------------------------
roslaunch dvrk_robot dvrk_master_slave_rviz.launch master:=MTML slave:=PSM1 \
config:=/path/to/ros/workspace/src/cisst-saw/sawIntuitiveResearchKit/share/jhu-dVRK/console-MTMR-PSM1_KIN_SIMULATED-Teleop.json

./jhu-dVRK/console-MTMR-PSM1_KIN_SIMULATED-Teleop.json
```

## Launch File & Configuration File
The dVRK kinematics simulation uses a simulated low-level hardware class and reuses rest of the code including the main executable **dvrk_console_josn**, which can run a real or a simulated arm based on a configuration file in json format. 

Here is a snippet from the launch file starting **dvrk_console_json**.
```xml
  <node name="dvrk_$(arg arm)_node"
        pkg="dvrk_robot"
        type="dvrk_console_json"
        args="-j $(arg config)"
        output="screen"/>
```

The example json configuration files are located in [sawIntuitiveResearchKit/share](/jhu-dvrk/sawIntuitiveResearchKit/) folder. Below is the _console-MTML_KIN_SIMULATED.json_ file. The ```simulation``` line indicates that this is a simulated MTM arm. For more details about the json file format, please refer to [Config File Formats](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Configuration-File-Formats) page.

```json
{
    "arms":
    [
        {
            "name": "MTML",
            "type": "MTM",
            "simulation": "KINEMATIC",
            "pid": "sawControllersPID-MTML.xml",
            "kinematic": "mtm.json"
        }
    ]
}
```

## Software API 
For most users, please use the ROS interface. The [dvrk_python](https://github.com/jhu-dvrk/dvrk-ros/tree/master/dvrk_python) or [dvrk_matlab](https://github.com/jhu-dvrk/dvrk-ros/tree/master/dvrk_matlab) package would be a good starting place. Please refer to [API](Components-APIs) page for more details. 

```python
# Example python code
import dvrk
m = dvrk.mtm('MTML')
m.get_current_joint_position()
m.move_joint_one(0.2, 0)
```

## Questions
Please use the dVRK Google Group if you have any questions. (See [First Steps](/jhu-dvrk/sawIntuitiveResearchKit/wiki/FirstSteps))

