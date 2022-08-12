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

# Moveit Config package
Using the robot description package now it is possible to create the MoveIt configuration files necessary to control the robot using MoveIt2. Currently the MoveIt Setup Assistant 2(MSA2) is not very reliable we can make use of the MoveIt Setup Assistant 1(MSA1). But the latest release of MSA1 is available for `ros-noetic` distro which has the LTS of ROS1 and only available in `Ubuntu 20`. Our current tutorial is developed on `ros-humble` which is available only on `Ubuntu 22`. This can be solved easily using the docker and it is possible to use the MSA1 from the docker using the [gui_docker](docker-gui.md). 

- **Step 1**: Convert the `final_description_file.urdf.xacro` to  `final_description_file.urdf`. Please note that if you make any modifications to your `workcell.urdf.xacro` file during your development then make sure to convert the latest version of your `final_description_file.urdf.xacro`
```
ros2 run xacro xacro -o workcell.urdf workcell.urdf.xacro 
``` 
- **Step 2**: Use the docker to create the `ur5e_workcell_moveit_config`
- **Step 3**: create a `workcell_description` package in the docker workspace
```
cd /root/ros1_ws/src
catkin_create_pkg workcell_description urdf xacro  
```
Now create a `urdf` and `meshes` directory inside the `workcell_description`
```
mkdir urdf
mkdir meshes
```
Copy the `workcell.urdf` that we have exported in Step 1 to the `ùrdf` directory and files in the meshes to the meshes directory. Repeat this step for all the dependencies as well.
Now build and source the workspace
````
source /opt/ros/noetic/setup.bash 
catkin build
source devel/setup.bash 
````
- **Step 4**: Start the MoveIt Setup Assistant
```
roslaunch moveit_setup_assistant setup_assistant.launch
```
- [Step 5](https://industrial-training-master.readthedocs.io/en/foxy/_source/session3/ros2/3-Build-a-MoveIt-Package.html)

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