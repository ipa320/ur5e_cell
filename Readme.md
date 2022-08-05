# Robot workcell description
This package provides the workcell description of the ur5e robot, Shunk egp50 gripper, intel realsense D435 camera and the custom workcell. 

- To visualize the robot cell in rviz
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
- Note:  To check xacro syntax
````
check_urdf <(xacro workcell.urdf.xacro)
````

**under development**