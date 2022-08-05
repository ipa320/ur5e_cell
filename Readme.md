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



To visualize the robot cell in rviz
```
ros2 run robot_state_publisher robot_state_publisher ~/ws/ / /urdf/workcell.urdf.xacro
````

````
rviz2
````
Alternatively
````
ros2 launch workcell_description workcell_bringup.py

````
**Tip:**  To check xacro syntax
````
check_urdf <(xacro workcell.urdf.xacro)
````