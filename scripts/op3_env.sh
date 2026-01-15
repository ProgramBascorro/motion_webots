#!/usr/bin/env bash

# Local OpenCV override for YOLO (built in third_party/opencv-4.8.1/install).
OP3_OPENCV_PREFIX="${OP3_OPENCV_PREFIX:-/home/farhan/Projects/Surgical_lokalisasi_bismillah/third_party/opencv-4.8.1/install}"

export OpenCV_DIR="$OP3_OPENCV_PREFIX/lib/cmake/opencv4"
export LD_LIBRARY_PATH="$OP3_OPENCV_PREFIX/lib:${LD_LIBRARY_PATH:-}"
export PKG_CONFIG_PATH="$OP3_OPENCV_PREFIX/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
export CMAKE_PREFIX_PATH="$OP3_OPENCV_PREFIX:${CMAKE_PREFIX_PATH:-}"
export PATH="$OP3_OPENCV_PREFIX/bin:${PATH:-}"
