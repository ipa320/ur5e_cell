# Robot workcell description package
This package provides the workcell description of the ur5e robot, Shunk egp50 gripper, and the robot cell from ipa_326. 

````
vcs import src --skip-existing --input src/ur5e_cell/dependencies_rolling.repos
````

Build & source your workspace 
````
colcon build --symlink-install
source install/setup.bash
````
Launch the workcell
````
ros2 launch ur5e_cell_description view_ur_cell.launch.py

````

**Tip:**  To check xacro syntax during development
````
check_urdf <(xacro workcell.urdf.xacro)
````
# Real Robot Bringup

```
vcs import src --skip-existing --input src/ur5e_cell/dependencies_rolling.repos
```
This will pull necessary drivers and moveit_config packages into the workspace

Build & source your workspace 
````
colcon build --symlink-install
source install/setup.bash
````
Launch your robot driver and moveit
```
ros2 launch ur5e_cell_bringup ur5e_cell_bringup.launch.py 
```
-----------------------------------------------------------

### Issues
- [ ] Need to fix the measurement values in the urdf files from the real world measurements @alb
- [ ] Shunk EGP50 urdf is not perfect and needs to be exported directly from solidworks as urdf. @alb

## Reference
- [ROS wiki urdf](http://wiki.ros.org/urdf)