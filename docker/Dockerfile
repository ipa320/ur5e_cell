FROM ros:humble

RUN apt-get update && apt-get install -y \
    && apt install python3-vcstool \
    && apt-get install -y git 

ENV ros2_ws=/root/ur_ws
RUN mkdir -p $ros2_ws/src
WORKDIR $ros2_ws/src

RUN git clone https://github.com/ipa-rar/ur5e_cell.git 

WORKDIR $ros2_ws
RUN vcs import src --skip-existing --input src/ur5e_cell/dependencies_rolling.repos \
    && source /opt/ros/humble/setup.bash \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && echo "Dependecies are installed in the workspace"

RUN colcon build \
    && rm -rf /var/lib/apt/lists/* \
    && echo "Building packages completed"


COPY ./entrypoint-server.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"] 
CMD ["bash"]