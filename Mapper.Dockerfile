ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ARG CARLA_ROS_BRIDGE_REPO=https://github.com/valentinrusche/ros-bridge.git
ARG STARS_MESSAGES_REPO=https://github.com/valentinrusche/stars-ros-messages.git
ARG ENABLED_FLAG_DEV_MODE=0
ARG UID
ARG GID

WORKDIR /app

SHELL ["/bin/bash", "-c"]

RUN apt-get update -y && apt-get -y dist-upgrade && \
    apt-get install --no-install-recommends --fix-missing -y \
    && rm -rf /var/lib/apt/lists/*

# create a non root user group with GID 1000
RUN groupadd -g ${GID} developer

# create a non root user with UID 1000
RUN useradd -u ${UID} -g developer --create-home --groups sudo --shell /bin/bash developer && \
    mkdir -p /etc/sudoers.d && \
    echo "developer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer
ENV HOME /home/developer

COPY entrypoint.sh /

# allow access for all users to source the nessecary files. u+x won't do it..
RUN chmod a+x /entrypoint.sh

RUN chown -R developer: /app/

# build Carla message types as well as the stars message types
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && git clone --recurse-submodules ${CARLA_ROS_BRIDGE_REPO} \
    && colcon build --merge-install --packages-select carla_msgs carla_waypoint_types carla_actor_state_types

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && git clone ${STARS_MESSAGES_REPO} \
    && colcon build --merge-install --packages-select stars_msgs

# Copy carla_stars_bridge to the container
COPY CarlaStarsRosMapper/ros2_ws /app/ros2_ws

# build the bridge ros2 program.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --merge-install --packages-select carla_stars_ros_mapper

RUN if [ "$ENABLED_FLAG_DEV_MODE" != "1" ]; then \
      rm -rf /app/ros2_ws && rm -rf /app/build; \
    fi


# Return to the non-root developer user
USER developer

ENTRYPOINT [ "/entrypoint.sh"]
