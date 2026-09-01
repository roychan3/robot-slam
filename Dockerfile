FROM ros:noetic-ros-base-focal

# GAZEBO_RESOURCE_PATH is deliberately not set here: the entrypoint sources
# Gazebo's own setup.sh, which reassigns it, so a value set at this layer would
# be silently discarded. The entrypoint sets it after that instead.
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    TURTLEBOT3_MODEL=waffle_pi \
    GAZEBO_MODEL_DATABASE_URI=

# ROS Noetic is the ROS 1 release that provides slam_gmapping. Gazebo's ROS
# plugins provide the differential drive, lidar, and RGB-D point-cloud sensors.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-noetic-gazebo-ros \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-gmapping \
        ros-noetic-octomap-server \
        ros-noetic-pcl-ros \
        ros-noetic-robot-state-publisher \
        ros-noetic-rviz \
        ros-noetic-teleop-twist-keyboard \
        ros-noetic-turtlebot3-description \
        ros-noetic-xacro \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Classic needs an X rendering context for camera/depth sensors even
# when gzclient is disabled. Xvfb keeps that requirement inside the container.
# x11-utils supplies xdpyinfo, which the entrypoint uses to wait for that
# display before starting x11vnc.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        novnc \
        openbox \
        websockify \
        x11-utils \
        x11vnc \
        xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY catkin_ws /app/catkin_ws
COPY docker/entrypoint.sh /ros_entrypoint.sh

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -C /app/catkin_ws" \
    && chmod +x /ros_entrypoint.sh /app/catkin_ws/src/robot_slam/scripts/*.py \
    && useradd --create-home --uid 10001 appuser \
    && mkdir -p /data /home/appuser/.gazebo \
    && chown -R appuser:appuser /app /data /home/appuser/.gazebo

USER appuser
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["roslaunch", "robot_slam", "robot_slam.launch", "gui:=false"]
