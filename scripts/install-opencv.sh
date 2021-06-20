#!/usr/bin/env sh
sudo apt update
sudo apt upgrade -y

sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev pkg-config -y

sudo apt-get install libtbb2 libtbb-dev -y

sudo add-apt-repository 'deb http://security.ubuntu.com/ubuntu xenial-security main'
sudo apt update

sudo apt-get install libjpeg-dev libpng-dev libtiff5-dev libjasper-dev libdc1394-22-dev libeigen3-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev sphinx-common libtbb-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libopenexr-dev libgstreamer-plugins-base1.0-dev libavutil-dev libavfilter-dev libavresample-dev -y


sudo -s

cd /opt/

wget -O opencv.zip https://github.com/opencv/opencv/archive/4.5.2.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/refs/tags/4.5.2.zip

unzip opencv.zip
unzip opencv_contrib.zip


mkdir -p build && cd build


cmake -D BUILD_TIFF=ON -D WITH_CUDA=OFF -D ENABLE_AVX=OFF -D WITH_OPENGL=ON -D WITH_OPENCL=OFF -D WITH_IPP=OFF -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=OFF -D WITH_V4L=OFF -D WITH_VTK=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules /opt/opencv/ -D OPENCV_GENERATE_PKGCONFIG=ON 

make -j7

sudo make install

sudo ldconfig

pkg-config --cflags opencv4
