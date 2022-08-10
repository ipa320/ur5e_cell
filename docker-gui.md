## Developing code in docker without copy  

### You can develop your code in local filesystem and compile code in docker. 

``` 
docker run --rm -it -v ~/ipa/ros_i/ros2_ws:/root/ros2_ws --name ros_i ros:humble-ros-base /bin/bash 
``` 

- `~/ipa/ros_i/ros2_ws` is your local folder 

- `/root/ros2_ws` is the folder in your docker container 

- If you don't want to remove container after exist the container, remove "--rm" 

## Exiting the container and restart it 

``` 
docker start -i ros_i 
``` 

## Using gui in docker 

Download "gui-docker" script 

``` 
wget https://raw.githubusercontent.com/ros-industrial/ros2_i_training/main/gui-docker 
 
```
```

sudo chmod +x gui-docker 

``` 

- Start docker 

``` 

./gui-docker -it -v ~/ipa/ros_i/ros2_ws:/root/ros2_ws --name ros_i ros:humble-ros-base /bin/bash 

``` 
- Exiting the container and restart it 

``` 
./gui-docker -c ros_i 
``` 

**Debug**  

https://github.com/ros-industrial/ros2_i_training/tree/main/workshop 


## Save your container into a image 

e.g. if you don't want to install all tools, you can save the container 

``` 

docker ps 

CONTAINER ID   IMAGE                 COMMAND                  CREATED             STATUS          PORTS     NAMES 

a1a39380d932   ros:humble-ros-base   "/ros_entrypoint.sh …"   46 minutes ago      Up 46 minutes             test 

 

docker commit a1a39380d932 ros-i/humble-ros-tools 

``` 

- Then you can use image: 
```
ros-i/humble-ros-tools:latest 
```

