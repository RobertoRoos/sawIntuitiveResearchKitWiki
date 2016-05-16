## Compatible development branches

For each component, `git checkout <branch_name>`:

| Components               | Branches      | Branches     |
| ------------------------ | ------------- |------------- |
| cisst                    | devel         | devel        |
| cisst-ros                | devel         | devel        |
| sawKeyboard              | devel         | devel        |
| sawTextToSpeech          | devel         | devel        |
| sawRobotIO1394           | devel         | devel        |
| sawControllers           | devel         | devel        |
| sawConstraintControllers | devel         | devel        |
| sawIntuitiveResearchKit  | devel         | feature-ecm  |
| dvrk-ros                 | devel         | feature-ecm  |

Note: sawRobotIO1394 in the devel branch has a different git submodule so you need to go in the component source directory and update the submodule:
```sh
cd ~/catkin_ws/src/cisst-saw/sawRobotIO1394
git submodule init
git submodule update
```

## Changes

### `devel`

* Code in the components directory has been re-organized to better separate the component's code from examples and applications.  It's overall cleaner and works with the latest catkin build tools.
* 
