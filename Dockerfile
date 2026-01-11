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

# ---- stage 2: ROS Humble base + Webots runtime deps + your workspace
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# 1) Build + ROS dev deps (yours)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git \
    libeigen3-dev libyaml-cpp-dev libboost-all-dev libopencv-dev \
    python3-colcon-common-extensions python3-rosdep \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-eigen \
    ros-humble-webots-ros2-driver \
    ros-humble-interactive-markers \
    qtbase5-dev qttools5-dev \
    xvfb wget locales \
 && rm -rf /var/lib/apt/lists/*


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
ENV LD_LIBRARY_PATH=/usr/local/webots/lib:/usr/local/webots/lib/controller:${LD_LIBRARY_PATH}
ENV QTWEBENGINE_DISABLE_SANDBOX=1
ENV USER=root

# Locale (optional but nice)
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# 4) rosdep init/update (safe)
RUN rosdep init || true && rosdep update

# 5) Copy your workspace + install deps + build
WORKDIR /ros2_ws
COPY src/ src/

RUN bash -lc "source /opt/ros/humble/setup.bash"

# && \
#     rosdep install --from-paths src --ignore-src -r -y && \
#     colcon build --symlink-install"

# 6) Entrypoint
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
