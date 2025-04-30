# Use the official ROS 2 Jazzy base image
FROM ros:jazzy-ros-base

# Set shell for consistent command execution
SHELL ["/bin/bash", "-c"]

# Set non-interactive frontend for apt commands
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Basic setup and locale configuration
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata \
    locales \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install essential ROS tools, build tools, and system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-venv \
    python3-lark \
    python3-yaml \
    python3-scipy \
    python3-opencv \
    xterm \
    # Add ROS dependencies available via apt
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-msgs \
    # *** REMOVED ros-jazzy-robotis-controller-msgs ***
    # *** REMOVED ros-jazzy-op3-walking-module-msgs ***
    # *** REMOVED ros-jazzy-op3-action-module-msgs ***
    ros-jazzy-webots-ros2-driver \
    ros-jazzy-teleop-twist-keyboard \
    # Add any other missing *available* 'ros-jazzy-...' packages
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Set up the ROS 2 workspace directory
WORKDIR /ros2_ws

# Copy the entire source directory into the image
COPY ./src /ros2_ws/src

# Install dependencies listed in package.xml files using rosdep
# This will install system deps for the source packages (like build-essential for C++)
# but won't find the missing ROS message debs (which is fine, we build them next)
RUN source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO

# Create and activate a Python virtual environment INSIDE the container
RUN python3 -m venv /opt/ros2_venv
ENV PATH="/opt/ros2_venv/bin:$PATH"

# Install necessary Python packages into the venv using pip
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir "numpy<2" && \
    python3 -m pip install --no-cache-dir ultralytics

# Build the ENTIRE ROS 2 workspace from source
# This will now build robotis_controller_msgs, op3_walking_module_msgs, etc.
# because their source code was copied in the COPY step.
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --event-handlers console_direct+

# Setup entrypoint script to source environments correctly
COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Default command (can be overridden)
CMD ["bash"]