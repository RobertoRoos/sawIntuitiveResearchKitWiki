<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](https://github.com/thlorenz/doctoc)*

- [Kinematics (JSON)](#kinematics-json)
- [Console (JSON)](#console-json)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# Kinematics (JSON)


# Console (JSON)

```json
{
    "io": {
        "period": 0.0005, // in seconds
        "port": 0 // default is 0
    }
    ,
    "operator-present": {
        "component": "io",
        "interface": "COAG"
    }
    ,
    "arms":
    [
        {
            "name": "SUJ",
            "type": "SUJ",
            "io": "sawRobotIO1394-SUJ.xml",
            "kinematic": "suj-ECM-1-2-3.json",
            "base-frame": {
                "component": "ECM",
                "interface": "Robot"
            }
        }
        ,
        {
            "name": "PSM1",
            "type": "PSM",
            "io": "sawRobotIO1394-PSM1-49695.xml",
            "pid": "../sawControllersPID-PSM.xml",
            "kinematic": "../psm-large-needle-driver.json",
            "base-frame": {
                "component": "SUJ",
                "interface": "PSM1"
            }
        }
        ,
        {
            "name": "PSM2",
            "type": "PSM",
            "io": "sawRobotIO1394-PSM2-32204.xml",
            "pid": "../sawControllersPID-PSM.xml",
            "kinematic": "../psm-large-needle-driver.json",
            "base-frame": {
                "component": "SUJ",
                "interface": "PSM2"
            }
        }
        ,
        {
            "name": "ECM",
            "type": "ECM",
            "io": "sawRobotIO1394-ECM-29738.xml",
            "pid": "../sawControllersPID-ECM.xml",
            "kinematic": "../dvecm.rob",
            "base-frame": {
                "component": "SUJ",
                "interface": "ECM"
            }
        }
        ,
        {
            "name": "MTMR",
            "type": "MTM",
            "io": "sawRobotIO1394-MTMR-22403-foot-pedal.xml",
            "pid": "../sawControllersPID-MTMR.xml",
            "kinematic": "../mtmr.json"
        }
        ,
        {
            "name": "MTML",
            "type": "MTM",
            "io": "sawRobotIO1394-MTML-34863.xml",
            "pid": "../sawControllersPID-MTML.xml",
            "kinematic": "../mtml.json"
        }
    ]
    ,
    "psm-teleops":
    [
        {
            "master": "MTMR",
            "slave": "PSM1",
            "period": 0.005, // in seconds
            "rotation" : [[ 0.0000,  1.0000,  0.0000],
                          [ 1.0000,  0.0000,  0.0000],
                          [ 0.0000,  0.0000, -1.0000]]
        }
        ,
        {
            "master": "MTML",
            "slave": "PSM2",
            "period": 0.005, // in seconds
            "rotation" : [[ 0.0000,  1.0000,  0.0000],
                          [ 1.0000,  0.0000,  0.0000],
                          [ 0.0000,  0.0000, -1.0000]]
        }
    ]
    ,
    "ecm-teleop":
    {
        "master-left": "MTML",
        "master-right": "MTMR",
        "slave": "ECM",
        "rotation" : [[ 1.0000,  0.0000,  0.0000],
                      [ 0.0000,  1.0000,  0.0000],
                      [ 0.0000,  0.0000,  1.0000]]
    }
}
```
