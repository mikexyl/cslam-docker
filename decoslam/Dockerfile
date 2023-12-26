FROM cuda-ros-base:noetic

ENV ROS_DISTRO=noetic

ENV DEBIAN_FRONTEND=noninteractive

# install OpenCV
RUN apt-get update && \
    apt-get install -y libopencv-dev && \
    rm -rf /var/lib/apt/lists/*

# Install Ceres (version 2.0 or higher)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libyaml-cpp-dev \
    libceres-dev && \
    rm -rf /var/lib/apt/lists/*

# Install libzmq
RUN apt update && \
    # install git libtool
    apt install -y git libtool build-essential pkg-config autoconf automake uuid-dev g++ gcc && \
    rm -rf /var/lib/apt/lists/* && \
    git clone https://github.com/zeromq/libzmq.git /libzmp && \
    cd /libzmp && \
    ./autogen.sh && \
    ./configure && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# Install cppzmq
RUN git clone https://github.com/zeromq/cppzmq.git /cppzmq && \
    cd /cppzmq && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

# Install opengv
RUN git clone https://github.com/laurentkneip/opengv && \
    cd opengv && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

RUN git clone https://github.com/VIS4ROB-lab/decoSLAM.git

RUN /bin/bash -c 'cd decoSLAM && \
    mkdir build && \
    cd build && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cmake .. && \
    make -j$(nproc)'

COPY ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD [ "bash" ]