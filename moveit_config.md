
# Moveit Config package
Using the robot description package now it is possible to create the MoveIt configuration files necessary to control the robot using MoveIt2. Currently the MoveIt Setup Assistant 2(MSA2) is not very reliable we can make use of the MoveIt Setup Assistant 1(MSA1). But the latest release of MSA1 is available for `ros-noetic` distro which has the LTS of ROS1 and only available in `Ubuntu 20`. Our current tutorial is developed on `ros-humble` which is available only on `Ubuntu 22`. This can be solved easily using the docker and it is possible to use the MSA1 from the docker using the [gui_docker](docker-gui.md). 

- **Step 1**: Convert the `final_description_file.urdf.xacro` to  `final_description_file.urdf`. Please note that if you make any modifications to your `workcell.urdf.xacro` file during your development then make sure to convert the latest version of your `final_description_file.urdf.xacro`
```
ros2 run xacro xacro -o workcell.urdf workcell.urdf.xacro 
``` 
- **Step 2**: Use the docker to create the `ur5e_workcell_moveit_config`
- **Step 3**: create a `ur5e_cell_description` package in the docker workspace
```
cd /root/ros1_ws/src
catkin_create_pkg ur5e_cell_description urdf xacro  
```
Now create a `urdf` and `meshes` directory inside the `ur5e_cell_description`
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
