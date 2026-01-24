# ---- stage 1: download Webots tarball (keeps final image cleaner)
FROM ubuntu:22.04 AS webots_downloader
ARG DEBIAN_FRONTEND=noninteractive
ARG WEBOTS_VERSION=R2025a
ARG WEBOTS_PACKAGE_PREFIX=

RUN apt-get update && apt-get install -y --no-install-recommends \
    wget bzip2 ca-certificates \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN wget -q https://github.com/cyberbotics/webots/releases/download/${WEBOTS_VERSION}/webots-${WEBOTS_VERSION}-x86-64${WEBOTS_PACKAGE_PREFIX}.tar.bz2 \
 && tar xjf webots-*.tar.bz2 \
 && rm webots-*.tar.bz2

# ---- stage 2: ROS Humble base + Webots runtime deps + your workspace (no build)
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# 1) System deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git \
    libeigen3-dev libyaml-cpp-dev libboost-all-dev libopencv-dev \
    python3-colcon-common-extensions python3-rosdep \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-eigen \
    ros-humble-webots-ros2-driver \
    ros-humble-interactive-markers \
    ros-humble-rosbridge-server \
    ros-humble-web-video-server \
    ros-humble-usb-cam \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-angles \
    ros-humble-octomap-ros \
    ros-humble-octomap-server \
    ros-humble-octomap-msgs \
    ros-humble-joy \
    ros-humble-foxglove-bridge \
    ros-humble-rqt-image-view \
    python3-scipy \
    libncurses-dev \
    qtbase5-dev qttools5-dev \
    xvfb wget locales \
    curl gnupg \
    tmux \
 && rm -rf /var/lib/apt/lists/*

# ---- Charm repo (gum) ----
RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://repo.charm.sh/apt/gpg.key | gpg --dearmor -o /etc/apt/keyrings/charm.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/charm.gpg] https://repo.charm.sh/apt/ * *" > /etc/apt/sources.list.d/charm.list && \
    apt-get update && apt-get install -y --no-install-recommends gum && \
    rm -rf /var/lib/apt/lists/*

# ---- pnpm ----
# Installs pnpm to /root/.local/share/pnpm and sets up PATH for future shells.
RUN curl -fsSL https://get.pnpm.io/install.sh | sh -
ENV PNPM_HOME=/root/.local/share/pnpm
ENV PATH=${PNPM_HOME}:${PATH}

# Use Node LTS via pnpm (downloads Node into PNPM_HOME)
RUN ${PNPM_HOME}/pnpm env use --global lts

# 2) Install Webots runtime deps using Cyberbotics script
RUN apt-get update && apt-get install -y --no-install-recommends wget \
 && wget -q https://raw.githubusercontent.com/cyberbotics/webots/master/scripts/install/linux_runtime_dependencies.sh \
 && chmod +x linux_runtime_dependencies.sh \
 && ./linux_runtime_dependencies.sh \
 && rm -f linux_runtime_dependencies.sh \
 && rm -rf /var/lib/apt/lists/*

# 3) Install Webots itself
WORKDIR /usr/local
COPY --from=webots_downloader /tmp/webots /usr/local/webots

# Webots env
ENV WEBOTS_HOME=/usr/local/webots
ENV PATH=/usr/local/webots:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/webots/lib:/usr/local/webots/lib/controller
ENV QTWEBENGINE_DISABLE_SANDBOX=1
ENV USER=root

# Locale (optional but nice)
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# 4) rosdep init/update (optional; safe)
RUN rosdep init 2>/dev/null || true && rosdep update

# 5) Workspace + helper scripts (NO rosdep install, NO colcon build)
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

# Copy workspace source
COPY src/ /ros2_ws/src/

# Copy helper script(s)
COPY ./script.sh /ros2_ws/script.sh
COPY ./scripts/ /ros2_ws/scripts/

# Make scripts executable (best-effort)
RUN chmod +x /ros2_ws/script.sh 2>/dev/null || true && \
    find /ros2_ws/scripts -type f -name "*.sh" -exec chmod +x {} \; 2>/dev/null || true

# Make interactive shells auto-source ROS + pnpm
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "export PNPM_HOME=/root/.local/share/pnpm" >> /etc/bash.bashrc && \
    echo "export PATH=\$PNPM_HOME:\$PATH" >> /etc/bash.bashrc

# 6) Entrypoint
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
