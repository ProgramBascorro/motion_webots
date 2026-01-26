# ---- stage 1: download Webots tarball (optional)
FROM ubuntu:22.04 AS webots_downloader
ARG DEBIAN_FRONTEND=noninteractive
ARG WEBOTS_VERSION=R2025a
ARG WEBOTS_PACKAGE_PREFIX=
ARG WITH_WEBOTS=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    wget bzip2 ca-certificates \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp

RUN if [ "$WITH_WEBOTS" = "1" ]; then \
      wget -q "https://github.com/cyberbotics/webots/releases/download/${WEBOTS_VERSION}/webots-${WEBOTS_VERSION}-x86-64${WEBOTS_PACKAGE_PREFIX}.tar.bz2" && \
      tar xjf webots-*.tar.bz2 && \
      rm webots-*.tar.bz2 ; \
    else \
      echo "WITH_WEBOTS=0 -> skipping Webots download"; \
      mkdir -p /tmp/webots; \
    fi


# ---- stage 2: ROS Humble base + (optional) Webots + your workspace
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ARG WITH_WEBOTS=1

# Tool versions (pin for reproducibility)
ARG NODE_MAJOR=20
ARG PNPM_VERSION=9.15.4
ARG GUM_VERSION=0.14.5

# 1) System deps (+ Node repo deps)
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
    xvfb locales \
    curl ca-certificates gnupg \
    tmux \
    xz-utils \
 && rm -rf /var/lib/apt/lists/*

# ---- Install Node.js (Debian/Ubuntu-friendly) ----
# Using NodeSource repo (stable for Ubuntu 22.04).
RUN set -eux; \
  mkdir -p /etc/apt/keyrings; \
  curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg; \
  echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_${NODE_MAJOR}.x nodistro main" > /etc/apt/sources.list.d/nodesource.list; \
  apt-get update; \
  apt-get install -y --no-install-recommends nodejs; \
  rm -rf /var/lib/apt/lists/*

# ---- pnpm via Corepack (reproducible; no SHELL=bash hack) ----
ENV PNPM_HOME=/root/.local/share/pnpm
ENV PATH=${PNPM_HOME}:${PATH}

RUN set -eux; \
  corepack enable; \
  corepack prepare "pnpm@${PNPM_VERSION}" --activate; \
  pnpm --version

# ---- gum via GitHub release binary (avoids apt https/keyring issues) ----
RUN set -eux; \
  arch="$(dpkg --print-architecture)"; \
  case "$arch" in \
    amd64) gum_arch="x86_64" ;; \
    arm64) gum_arch="arm64" ;; \
    *) echo "Unsupported arch: $arch" >&2; exit 1 ;; \
  esac; \
  curl -fsSL -o /tmp/gum.tar.gz \
    "https://github.com/charmbracelet/gum/releases/download/v${GUM_VERSION}/gum_${GUM_VERSION}_Linux_${gum_arch}.tar.gz"; \
  tar -xzf /tmp/gum.tar.gz -C /tmp; \
  install -m 0755 /tmp/gum_${GUM_VERSION}_Linux_${gum_arch}/gum /usr/local/bin/gum; \
  rm -rf /tmp/gum.tar.gz /tmp/gum_${GUM_VERSION}_Linux_${gum_arch}; \
  gum --version

# Locale
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# rosdep
RUN rosdep init 2>/dev/null || true && rosdep update

# 2) Install Webots runtime deps (optional)
RUN if [ "$WITH_WEBOTS" = "1" ]; then \
      apt-get update && apt-get install -y --no-install-recommends wget && \
      wget -q https://raw.githubusercontent.com/cyberbotics/webots/master/scripts/install/linux_runtime_dependencies.sh && \
      chmod +x linux_runtime_dependencies.sh && \
      ./linux_runtime_dependencies.sh && \
      rm -f linux_runtime_dependencies.sh && \
      rm -rf /var/lib/apt/lists/* ; \
    else \
      echo "WITH_WEBOTS=0 -> skipping Webots runtime dependencies"; \
    fi

# 3) Install Webots itself (optional)
WORKDIR /tmp
COPY --from=webots_downloader /tmp/webots /tmp/webots

RUN if [ "$WITH_WEBOTS" = "1" ]; then \
      mkdir -p /usr/local && mv /tmp/webots /usr/local/webots ; \
    else \
      echo "WITH_WEBOTS=0 -> removing staged Webots files" && rm -rf /tmp/webots ; \
    fi

# Webots env (OK if you always build WITH_WEBOTS=1; otherwise these point to non-existent path)
ENV WEBOTS_HOME=/usr/local/webots
ENV PATH=/usr/local/webots:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/webots/lib:/usr/local/webots/lib/controller
ENV QTWEBENGINE_DISABLE_SANDBOX=1
ENV USER=root

# Workspace
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src
COPY src/ /ros2_ws/src/

COPY ./script.sh /ros2_ws/script.sh
COPY ./scripts/ /ros2_ws/scripts/
RUN chmod +x /ros2_ws/script.sh 2>/dev/null || true && \
    find /ros2_ws/scripts -type f -name "*.sh" -exec chmod +x {} \; 2>/dev/null || true

# Global bash rc convenience
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "export PNPM_HOME=/root/.local/share/pnpm" >> /etc/bash.bashrc && \
    echo "export PATH=\$PNPM_HOME:\$PATH" >> /etc/bash.bashrc

COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
