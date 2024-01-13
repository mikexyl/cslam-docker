FROM cuda-ros-base:iron

RUN apt update && apt install -y --no-install-recommends \
    sudo git python3-vcstool wget software-properties-common\
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
    ros-${ROS_DISTRO}-rtabmap-ros ros-${ROS_DISTRO}-perception-pcl && \ 
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

RUN mkdir -p /ros2_ws/src && \
    cd /ros2_ws && \
    wget https://raw.githubusercontent.com/MISTLab/Swarm-SLAM/main/cslam.repos && \
    vcs import src < cslam.repos

RUN pip3 install --upgrade pip && \
    pip3 install --ignore-installed blinker && \
    pip3 install --user open3d networkx numba numpy Pillow scikit_learn scipy sortedcontainers torch torchvision distinctipy numpy-quaternion

# apply patch to cslam
# RUN cd /ros2_ws/src/cslam
    # git apply /ros_log_str.patch && \
    # git apply /rclpy_param_types.patch

WORKDIR /ros2_ws/

# use bash
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build

# install runtime dependencies
RUN apt update && apt install -y --no-install-recommends \
    ros-$ROS_DISTRO-compressed-image-transport ros-$ROS_DISTRO-compressed-depth-image-transport && \ 
    rm -rf /var/lib/apt/lists/*

# download pretrained models
RUN mkdir -p /root/.cache/torch/hub/checkpoints/ && \
    wget https://download.pytorch.org/models/resnet18-f37072fd.pth -q -O /root/.cache/torch/hub/checkpoints/resnet18-f37072fd.pth && \
    wget https://download.pytorch.org/models/resnet50-19c8e357.pth -q -O /root/.cache/torch/hub/checkpoints/resnet50-19c8e357.pth && \
    wget https://download.pytorch.org/models/resnet101-5d3b4d8f.pth -q -O /root/.cache/torch/hub/checkpoints/resnet101-5d3b4d8f.pth && \
    wget https://download.pytorch.org/models/resnet152-b121ed2d.pth -q -O /root/.cache/torch/hub/checkpoints/resnet152-b121ed2d.pth

RUN git clone https://github.com/lajoiepy/cslam_visualization.git src/cslam_visualization \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --packages-select cslam_visualization

COPY swarm-slam/launch /ros2_ws/src/cslam_experiments/launch

COPY swarm-slam/resnet18_64.pth /ros2_ws/src/cslam/models/

COPY swarm-slam/cslam_vis_ros_parameter.patch /cslam_vis_ros_parameter.patch
COPY swarm-slam/rclpy_param_types.patch /rclpy_param_types.patch

RUN cd /ros2_ws/src/cslam_visualization && \
    git apply /cslam_vis_ros_parameter.patch && \
    cd /ros2_ws/src/cslam && \
    git apply /rclpy_param_types.patch

# build again to install launch files
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build

COPY ./ros_entrypoint.sh /
ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD [ "/usr/bin/bash" ]
