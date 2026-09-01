FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    GZ_VERSION=harmonic \
    TURTLEBOT3_MODEL=waffle_pi

# Jazzy is paired with the Gazebo Harmonic LTS release. ros_gz provides the
# simulator integration and transport bridge; SLAM Toolbox replaces the ROS 1
# gmapping package used by the old Gazebo Classic image.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        ros-jazzy-octomap-server \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-ros-gz \
        ros-jazzy-rviz2 \
        ros-jazzy-sensor-msgs-py \
        ros-jazzy-slam-toolbox \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Sim needs an X rendering context for camera/depth sensors even when
# its GUI is disabled. Xvfb keeps that requirement inside the container.
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

# Serve noVNC below a release-specific path. Browsers otherwise reuse cached
# JavaScript from the former Focal/noVNC 1.0 image with Noble's noVNC 1.3 HTML,
# which prevents the client from completing its connection.
RUN mkdir -p /usr/share/novnc/jazzy-harmonic \
    && cp -a \
        /usr/share/novnc/app \
        /usr/share/novnc/core \
        /usr/share/novnc/vendor \
        /usr/share/novnc/vnc.html \
        /usr/share/novnc/vnc_lite.html \
        /usr/share/novnc/jazzy-harmonic/

WORKDIR /app
COPY ros2_ws /app/ros2_ws
COPY docker/entrypoint.sh /ros_entrypoint.sh

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && cd /app/ros2_ws && colcon build --merge-install" \
    && chmod +x /ros_entrypoint.sh /app/ros2_ws/src/robot_slam/scripts/*.py \
    && useradd --create-home --uid 10001 appuser \
    && mkdir -p /data /home/appuser/.gz \
    && chown -R appuser:appuser /app /data /home/appuser/.gz

USER appuser
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "robot_slam", "robot_slam.launch.py", "gui:=false"]
