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
ARG OPENCV_VERSION=4.8.1
ARG OPENCV_PREFIX=/opt/opencv-${OPENCV_VERSION}
ARG OPENVINO_VERSION=2025.4.0
ARG OPENVINO_FULL_VERSION=2025.4.0.20398.8fdad55727d

# Tool versions (pin for reproducibility)
# NOTE: We avoid NodeSource apt repo to prevent signature/proxy issues during docker build.
ARG NODE_VERSION=20.11.1
ARG PNPM_VERSION=9.15.4
ARG GUM_VERSION=0.14.5
ARG TTYD_VERSION=1.7.7

# (Optional hardening) Prefer HTTPS for Ubuntu + ROS repos to reduce MITM/captive portal issues.
# Safe even if the files don't exist.
RUN set -eux; \
    sed -i 's|http://archive.ubuntu.com/ubuntu|https://archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list || true; \
    sed -i 's|http://security.ubuntu.com/ubuntu|https://security.ubuntu.com/ubuntu|g' /etc/apt/sources.list || true; \
    if [ -f /etc/apt/sources.list.d/ros2.list ]; then \
      sed -i 's|http://packages.ros.org/ros2/ubuntu|https://packages.ros.org/ros2/ubuntu|g' /etc/apt/sources.list.d/ros2.list; \
    fi

# 1) System deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git \
    libeigen3-dev libyaml-cpp-dev libboost-all-dev libopencv-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libgtk-3-dev \
    python3-colcon-common-extensions python3-rosdep \
    python3-pip \
    libgl1 libgl1-mesa-dri libglfw3 libglew2.2 libosmesa6 \
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
    ubuntu-keyring \
 && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir mujoco

# ---- OpenCV (newer than Ubuntu repo) for ONNX DNN compatibility ----
RUN set -eux; \
  mkdir -p /tmp/opencv; \
  cd /tmp/opencv; \
  curl -fsSL -o opencv.tar.gz "https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.tar.gz"; \
  tar -xzf opencv.tar.gz --strip-components=1; \
  cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${OPENCV_PREFIX}" \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_opencv_python3=OFF \
    -DBUILD_opencv_java=OFF \
    -DBUILD_opencv_apps=OFF \
    -DWITH_CUDA=OFF \
    -DWITH_OPENCL=OFF \
    -DWITH_IPP=OFF \
    -DWITH_TBB=OFF \
    -DWITH_FFMPEG=ON \
    -DWITH_GSTREAMER=OFF \
    -DWITH_GTK=ON \
    -DWITH_V4L=ON \
    -DBUILD_LIST=core,imgproc,imgcodecs,videoio,highgui,objdetect,video,dnn; \
  cmake --build build --parallel "$(nproc)"; \
  cmake --install build; \
  rm -rf /tmp/opencv

# ---- OpenVINO Runtime (archive install, CPU-ready in Docker) ----
RUN set -eux; \
  arch="$(dpkg --print-architecture)"; \
  case "$arch" in \
    amd64) \
      ov_archive="openvino_toolkit_ubuntu22_${OPENVINO_FULL_VERSION}_x86_64.tgz"; \
      ov_dir="openvino_toolkit_ubuntu22_${OPENVINO_FULL_VERSION}_x86_64" ;; \
    arm64) \
      ov_archive="openvino_toolkit_ubuntu20_${OPENVINO_FULL_VERSION}_arm64.tgz"; \
      ov_dir="openvino_toolkit_ubuntu20_${OPENVINO_FULL_VERSION}_arm64" ;; \
    *) echo "Unsupported arch for OpenVINO: $arch" >&2; exit 1 ;; \
  esac; \
  mkdir -p /opt/intel; \
  curl -fsSL -o /tmp/openvino.tgz \
    "https://storage.openvinotoolkit.org/repositories/openvino/packages/2025.4/linux/${ov_archive}"; \
  tar -xf /tmp/openvino.tgz -C /tmp; \
  mv "/tmp/${ov_dir}" "/opt/intel/openvino_${OPENVINO_VERSION}"; \
  /opt/intel/openvino_${OPENVINO_VERSION}/install_dependencies/install_openvino_dependencies.sh -y; \
  rm -rf /tmp/openvino.tgz "/tmp/${ov_dir}"

# ---- Install Node.js from official tarball (no apt repos) ----
RUN set -eux; \
  arch="$(dpkg --print-architecture)"; \
  case "$arch" in \
    amd64) node_arch="x64" ;; \
    arm64) node_arch="arm64" ;; \
    *) echo "Unsupported arch: $arch" >&2; exit 1 ;; \
  esac; \
  curl -fsSL -o /tmp/node.tar.xz \
    "https://nodejs.org/dist/v${NODE_VERSION}/node-v${NODE_VERSION}-linux-${node_arch}.tar.xz"; \
  tar -xJf /tmp/node.tar.xz -C /usr/local --strip-components=1; \
  rm -f /tmp/node.tar.xz; \
  node -v; npm -v

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

# ---- ttyd (web terminal) via GitHub release binary ----
RUN set -eux; \
  arch="$(dpkg --print-architecture)"; \
  case "$arch" in \
    amd64) ttyd_arch="x86_64" ;; \
    arm64) ttyd_arch="aarch64" ;; \
    *) echo "Unsupported arch: $arch" >&2; exit 1 ;; \
  esac; \
  curl -fsSL -o /tmp/ttyd \
    "https://github.com/tsl0922/ttyd/releases/download/${TTYD_VERSION}/ttyd.${ttyd_arch}"; \
  install -m 0755 /tmp/ttyd /usr/local/bin/ttyd; \
  rm -f /tmp/ttyd; \
  ttyd --version

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

# Webots env
ENV WEBOTS_HOME=/usr/local/webots
ENV PATH=/usr/local/webots:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/webots/lib:/usr/local/webots/lib/controller
ENV QTWEBENGINE_DISABLE_SANDBOX=1
ENV USER=root
ENV OPENVINO_ROOT=/opt/intel/openvino_${OPENVINO_VERSION}
ENV OpenVINO_DIR=${OPENVINO_ROOT}/runtime/cmake
ENV OP3_OPENCV_PREFIX=${OPENCV_PREFIX}
ENV OpenCV_DIR=${OPENCV_PREFIX}/lib/cmake/opencv4
ENV CMAKE_PREFIX_PATH=${OpenVINO_DIR}:${CMAKE_PREFIX_PATH}
ENV LD_LIBRARY_PATH=${OPENVINO_ROOT}/runtime/lib/intel64:${OPENVINO_ROOT}/runtime/3rdparty/tbb/lib:${OPENCV_PREFIX}/lib:${LD_LIBRARY_PATH}

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
