# Robot workcell description package
This package provides the workcell description of the ur5e robot, Shunk egp50 gripper, and the robot cell from ipa_326. 

**Note** This package has dependencies with `ur_description` and `realsense2_description` which is not yet released for `ros-humble`. 
````
vcs import src --skip-existing --input src/ur5e_workcell_bringup/ur5e_workcell.repos
````

Build & source your workspace 
````
colcon build --symlink-install
source install/setup.bash
````
Launch the workcell
````
ros2 launch workcell_description workcell_bringup.launch.py

````

![](https://articulatedrobotics.xyz/media/assets/posts/ready-for-ros/urdf-rsp.png  )

**Tip:**  To check xacro syntax during development
````
check_urdf <(xacro workcell.urdf.xacro)
````
# Simulation in Gazebo
Convert the xacro files to urdf

```
ros2 run xacro xacro workcell.urdf.xacro > workcell.urdf
```
Convert the urdf files to world file for Gazebo
```
gz sdf -p workcell.urdf > workcell.world
```


# Real Robot Bringup

```
vcs import src --skip-existing --input src/ur5e_workcell_bringup/ur5e_workcell.repos
```
This will pull necessary drivers and moveit_config packages into the workspace

Build & source your workspace 
````
colcon build --symlink-install
source install/setup.bash
````
Launch the robot driver
```
ros2 launch workcell_description ur5e.launch.py 

```
Launch the moveit_package
```
ros2 launch ur_moveit_config ur_moveit.launch.py 
```
------------------------------------------------------------

### Tasks 
- [x] Make urdf of the workcell
- [x] Modify the shunk egp50 urdf file
- [x] Create moveit_config package
- [ ] Add Gazebo plugins to the xacro
- [x] Create launch files to bringup sim
- [x] Create launch files to bringup sim

### Issues
- [ ] Create the STL of robotcell with the safety glasses installed. This is necessary to define the collision with the glasses. @alb
- [ ] Need to fix the measurement values in the urdf files from the real world measurements @rar/alb
- [ ] Shunk EGP50 urdf is not perfect and needs to be exported directly from solidworks as urdf. @alb

## Reference
- [ROS wiki urdf](http://wiki.ros.org/urdf)
- [ArticulatedRobotics URDF](https://articulatedrobotics.xyz/ready-for-ros-7-urdf/)
- [ArticulatedRobotics Gazebo](https://articulatedrobotics.xyz/ready-for-ros-8-gazebo/)
