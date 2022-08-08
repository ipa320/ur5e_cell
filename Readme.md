# Robot workcell description
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

### Tasks 
- [x] Make urdf of the workcell
- [x] Modify the shunk egp50 urdf file
- [ ] Add Gazebo plugins to the xacro
- [x] Create launch files to bringup sim
- [ ] Create launch files to bringup sim

### Issues
- [ ] Need to fix the measurement values in the urdf files from the real world measurements
- [ ] Shunk EGP50 urdf is not perfect and needs to be exported directly from solidworks as urdf.
