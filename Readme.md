# Robot workcell description package
This package provides the workcell description of the ur5e robot, Shunk egp50 gripper, intel realsense D435 camera and the custom workcell. 

**Note** This package has dependencies with `ur_description` and `realsense2_description` which is not yet released for `ros-humble`. 
````
git clone -b ros2 https://github.com/IntelRealSense/realsense-ros.git
````
````
git clone -b ros2 https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
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
# Real Robot bringup

```
vcs import src --skip-existing --input ur5e_workcell_bringup/ur5e.repos 
```
This will pull necessary drivers and moveit_config packages into the workspace

Build & source your workspace 
````
colcon build --symlink-install
source install/setup.bash
````
------------------------------------------------------------

### Tasks 
- [x] Make urdf of the workcell
- [x] Modify the shunk egp50 urdf file
- [] Create moveit_config package
- [ ] Add Gazebo plugins to the xacro
- [x] Create launch files to bringup sim
- [ ] Create launch files to bringup sim

### Issues
- [ ] Create the STL of robotcell with the safety glasses installed. This is necessary to define the collision with the glasses. @alb
- [ ] Need to fix the measurement values in the urdf files from the real world measurements @rar/alb
- [ ] Shunk EGP50 urdf is not perfect and needs to be exported directly from solidworks as urdf. @alb

## Reference
- [ROS wiki urdf](http://wiki.ros.org/urdf)
- [ArticulatedRobotics URDF](https://articulatedrobotics.xyz/ready-for-ros-7-urdf/)
- [ArticulatedRobotics Gazebo](https://articulatedrobotics.xyz/ready-for-ros-8-gazebo/)