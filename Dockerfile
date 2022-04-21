FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"] 

RUN apt update && \
    apt install -y \
    python3-catkin-tools \
    python3-serial \
    ros-noetic-tf \
    socket

COPY . /nucleus_ws/src/driver
RUN source /opt/ros/noetic/setup.bash && cd /nucleus_ws && catkin build

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]