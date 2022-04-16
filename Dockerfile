FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu18.04

LABEL author="Marcelo Garcia"

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update --fix-missing && apt-get upgrade -y && apt-get install -y \
    build-essential \
    git \
    git-core \
    git-lfs \ 
    pkg-config \
    wget \
    vim \
    unzip \
    zip \
    zlib1g-dev \
    libpcl-dev \
    libgtk2.0-dev\ 
    pkg-config\
    libavcodec-dev\
    libavformat-dev\
    libswscale-dev\
    gdb

# Install cmake 
WORKDIR /opt
RUN mkdir cmake/
RUN wget https://github.com/Kitware/CMake/releases/download/v3.23.1/cmake-3.23.1-linux-x86_64.sh -q -O cmake/cmake-install.sh
WORKDIR /opt/cmake
RUN chmod +x cmake-install.sh
RUN bash cmake-install.sh --skip-license
RUN rm cmake-install.sh
ENV PATH="/opt/cmake/bin/:${PATH}"

# Install opencv 4.5.5
WORKDIR /opt
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.5.5.zip
RUN wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.5.5.zip
RUN unzip opencv.zip
RUN unzip opencv_contrib.zip
RUN cd opencv-4.5.5 && mkdir build
WORKDIR /opt/opencv-4.5.5/build
RUN cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.5.5/modules -DOPENCV_GENERATE_PKGCONFIG=YES
RUN make
RUN make install


WORKDIR /project

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

