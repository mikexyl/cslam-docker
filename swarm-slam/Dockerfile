FROM cuda-ros-base:foxy

RUN apt update && apt install -y --no-install-recommends \
    sudo git python3-vcstool wget \
    && rm -rf /var/lib/apt/lists/*

# Add PPA
RUN add-apt-repository ppa:borglab/gtsam-release-4.1
# Install:
RUN apt-get update && apt-get install -y --no-install-recommends libgtsam-dev libgtsam-unstable-dev && \
    rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y --no-install-recommends \
    python3-pip python3-dev python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install python3-rosdep python3-colcon-common-extensions \
    python3-vcstool build-essential -y && \
    rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y --no-install-recommends \
    ros-foxy-rtabmap-ros ros-foxy-perception-pcl && \ 
    rm -rf /var/lib/apt/lists/*

# Install dependencies
RUN apt update && apt install -y \
    curl \
    git \
    build-essential \
    clang \
    && rm -rf /var/lib/apt/lists/*

# Install Rust and Cargo
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Clone and build zenoh-plugin-dds
RUN git clone https://github.com/eclipse-zenoh/zenoh-plugin-dds.git

RUN cd zenoh-plugin-dds && \
    cargo install --locked --path zenoh-bridge-dds

RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
    apt update && apt install -y --no-install-recommends \
    apt-transport-https && \
    rm -rf /var/lib/apt/lists/*

RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    tee /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && apt install -y --no-install-recommends \
    librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg && \
    rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y --no-install-recommends \
    librealsense2-udev-rules && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros2_ws/src && \
    cd /ros2_ws && \
    wget https://raw.githubusercontent.com/MISTLab/Swarm-SLAM/main/cslam.repos && \
    vcs import src < cslam.repos && \
    wget https://raw.githubusercontent.com/MISTLab/Swarm-SLAM/main/requirements.txt && \
    sed -i '/open3d/d' requirements.txt && \
    # upgrade pip3
    pip3 install --upgrade pip && \
    pip3 install -r requirements.txt # "The requirements file is in the main Swarm-SLAM repo.You don't need torch if you are only using lidar.c"

RUN pip3 install open3d

WORKDIR /ros2_ws/
RUN source /opt/ros/foxy/setup.bash && \
    colcon build 

CMD [ "/usr/bin/bash" ]
