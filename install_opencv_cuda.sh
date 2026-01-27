#!/bin/bash
# Install OpenCV with CUDA support for RTX 3050 (Compute Capability 8.6)

set -e

echo "Installing OpenCV with CUDA support for RTX 3050..."
echo "This will take 30-60 minutes. Press Ctrl+C to cancel."
sleep 3

# Install dependencies
sudo apt update
sudo apt install -y build-essential cmake git pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-dev libopenexr-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev

# Create build directory
mkdir -p ~/opencv_cuda_build
cd ~/opencv_cuda_build

# Download OpenCV
echo "Downloading OpenCV 4.10.0..."
if [ ! -d "opencv" ]; then
    git clone --depth 1 --branch 4.10.0 https://github.com/opencv/opencv.git
fi
if [ ! -d "opencv_contrib" ]; then
    git clone --depth 1 --branch 4.10.0 https://github.com/opencv/opencv_contrib.git
fi

# Build
cd opencv
mkdir -p build
cd build

echo "Configuring CMake..."
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
    -D WITH_CUDA=ON \
    -D CUDA_ARCH_BIN=8.6 \
    -D WITH_CUDNN=ON \
    -D OPENCV_DNN_CUDA=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D WITH_CUBLAS=1 \
    -D WITH_TBB=ON \
    -D WITH_V4L=ON \
    -D WITH_OPENGL=ON \
    -D BUILD_opencv_python3=ON \
    -D PYTHON3_EXECUTABLE=$(which python3) \
    -D PYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
    -D PYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
    ..

echo "Building OpenCV (this takes a while)..."
make -j$(nproc)

echo "Installing OpenCV..."
sudo make install
sudo ldconfig

echo "Done! OpenCV with CUDA is now installed."
echo "You can now set use_gpu: true in yolo.yaml"
