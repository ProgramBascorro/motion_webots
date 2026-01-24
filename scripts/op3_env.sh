#!/usr/bin/env bash

# Local OpenCV override for YOLO (built in third_party/opencv-4.8.1/install).
if [ -n "${BASH_VERSION-}" ]; then
  _op3_env_source="${BASH_SOURCE[0]}"
elif [ -n "${ZSH_VERSION-}" ]; then
  _op3_env_source="${(%):-%N}"
else
  _op3_env_source="$0"
fi
SCRIPT_DIR="$(cd -- "$(dirname -- "$_op3_env_source")" && pwd)"
DEFAULT_OPENCV_PREFIX="$SCRIPT_DIR/../third_party/opencv-4.8.1/install"
OP3_OPENCV_PREFIX="${OP3_OPENCV_PREFIX:-$DEFAULT_OPENCV_PREFIX}"

if [ -d "$OP3_OPENCV_PREFIX" ]; then
  export OpenCV_DIR="$OP3_OPENCV_PREFIX/lib/cmake/opencv4"
  export LD_LIBRARY_PATH="$OP3_OPENCV_PREFIX/lib:${LD_LIBRARY_PATH:-}"
  export PKG_CONFIG_PATH="$OP3_OPENCV_PREFIX/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
  export CMAKE_PREFIX_PATH="$OP3_OPENCV_PREFIX:${CMAKE_PREFIX_PATH:-}"
  export PATH="$OP3_OPENCV_PREFIX/bin:${PATH:-}"
fi
