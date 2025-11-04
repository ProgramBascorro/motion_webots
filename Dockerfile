# syntax=docker/dockerfile:1

ARG WEBOTS_IMAGE=cyberbotics/webots:R2023b-ubuntu22.04
FROM ${WEBOTS_IMAGE} AS webots

FROM osrf/ros:humble-desktop

ARG ROS_DISTRO=humble
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

ARG SKIP_APT=false

# Base tooling and build prerequisites
RUN if [ "${SKIP_APT}" != "true" ]; then \
        apt-get update && \
        apt-get install -y --no-install-recommends \
            build-essential \
            python3-colcon-common-extensions \
            python3-rosdep \
            python3-vcstool \
            git \
            curl \
            gnupg \
            lsb-release \
            wget \
            vim \
            nano \
            # X11 and GUI support for Webots
            x11-apps \
            libxcb-xinerama0 \
            libxkbcommon-x11-0 \
            libxcb-cursor0 \
            libglu1-mesa \
            libgl1-mesa-glx \
            libgl1-mesa-dri \
            libglib2.0-0 \
            # Audio support
            pulseaudio \
            # Additional Webots dependencies
            libxcomposite1 \
            libxdamage1 \
            libxrandr2 \
            libxtst6 \
            libnss3 \
            libatk1.0-0 \
            libatk-bridge2.0-0 \
            libcups2 \
            libdrm2 \
            libgbm1 \
            libasound2 \
        && rm -rf /var/lib/apt/lists/*; \
    else \
        echo "Skipping apt dependency installation"; \
    fi

# Copy Webots runtime from upstream image to avoid external download during the build
COPY --from=webots /usr/local/webots /usr/local/webots

WORKDIR /ws
COPY . /ws

ARG SKIP_ROSDEP=false
ARG SKIP_COLCON=false

# Initialize rosdep if not already initialized
RUN if [ "${SKIP_ROSDEP}" != "true" ]; then \
        rosdep update || rosdep init && rosdep update; \
    fi

# Install missing dependencies that rosdep can't resolve
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-vision-opencv \
    libopencv-dev \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-usb-cam \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher \
    liborocos-kdl-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Create COLCON_IGNORE files for ROS1 packages
RUN touch src/ROBOTIS-Utility/ros_madplay_player/COLCON_IGNORE && \
    touch src/ROBOTIS-Utility/ros_mpg321_player/COLCON_IGNORE

# Resolve system dependencies (best effort)
RUN if [ "${SKIP_ROSDEP}" != "true" ]; then \
        source /opt/ros/${ROS_DISTRO}/setup.bash && \
        rosdep install --rosdistro ${ROS_DISTRO} --from-paths src --ignore-src -y \
            --skip-keys="catkin roscpp opencv opencv4 Eigen3 eigen3 cmake_modules orocos_kdl uvc_camera map_server" || \
        echo "Some rosdep dependencies failed, continuing..."; \
    else \
        echo "Skipping rosdep dependency resolution"; \
    fi

# Build the workspace with error handling
RUN if [ "${SKIP_COLCON}" != "true" ]; then \
        source /opt/ros/${ROS_DISTRO}/setup.bash && \
        colcon build --symlink-install \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
            --packages-skip ros_madplay_player ros_mpg321_player \
            --continue-on-error && \
        echo "Build completed (some packages may have failed)" || \
        (echo "Build failed, but image will still be created for manual building" && exit 0); \
    else \
        echo "Skipping colcon build"; \
    fi

# Environment setup
ENV WEBOTS_HOME=/usr/local/webots
ENV PATH=${WEBOTS_HOME}:${PATH}
ENV LD_LIBRARY_PATH=${WEBOTS_HOME}/lib/controller:${WEBOTS_HOME}/lib
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=0

# Create a setup script that sources ROS and the workspace
RUN echo '#!/bin/bash\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
if [ -f /ws/install/setup.bash ]; then\n\
    source /ws/install/setup.bash\n\
fi\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
