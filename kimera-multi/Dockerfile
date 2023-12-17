FROM cuda-ros-base:noetic

ENV ROS_DISTRO=noetic

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y openssh-client && \
    rm -rf /var/lib/apt/lists/* 

ARG SSH_PRIVATE_KEY_BASE64
RUN mkdir /root/.ssh && \
    echo "$SSH_PRIVATE_KEY_BASE64" | base64 -d > /root/.ssh/id_ed25519 && \
    chmod 600 /root/.ssh/id_ed25519 && \
    ssh-keyscan github.com > /root/.ssh/known_hosts

RUN apt update && apt install -y --no-install-recommends \
    sudo git python3-vcstool wget \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p catkin_ws/src && \
    cd catkin_ws/src/ && \
    git clone git@github.com:MIT-SPARK/Kimera-Multi.git kimera_multi && \
    # For full install
    vcs import . --input kimera_multi/kimera_multi.repos --recursive

RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils && \
    apt-get install -y cmake && \
    apt-get install -y libboost-all-dev && \
    apt-get install -y \
        build-essential unzip pkg-config \
        libjpeg-dev libpng-dev libtiff-dev \
        libvtk6-dev \
        libgtk-3-dev \
        libparmetis-dev \
        libatlas-base-dev \
    gfortran && \
    apt-get install -y libtbb-dev && \
    apt-get install -y libgflags-dev libgoogle-glog-dev && \
    apt-get install -y python3-wstool python3-catkin-tools ros-$ROS_DISTRO-cmake-modules protobuf-compiler autoconf && \
    rm -rf /var/lib/apt/lists/*

RUN apt update && \
    apt install -y ros-$ROS_DISTRO-ompl && \
    apt-get install -y build-essential cmake-gui libsuitesparse-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws
RUN catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin config -a --cmake-args -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DOPENGV_BUILD_WITH_MARCH_NATIVE=OFF && \
    catkin build --continue -s -j12 opencv3_catkin
RUN catkin build --continue -s -j12 gtsam

RUN apt update && \
    apt install -y libprotobuf-dev && \
    rm -rf /var/lib/apt/lists/* && \
    catkin build --continue -s -j12 voxblox
RUN catkin build --continue -s -j12 kimera_vio

RUN apt update && apt install -y \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-geometry \
    ros-$ROS_DISTRO-tf-conversions \
    ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-eigen-conversions \
    ocl-icd-libopencl1 \
    nvidia-opencl-dev \
    clinfo \
    opencl-headers \
    ros-$ROS_DISTRO-pcl-ros && \
    rm -rf /var/lib/apt/lists/*

RUN cd / && \ 
    git clone --recursive https://github.com/KhronosGroup/OpenCL-CLHPP && \
    git clone https://github.com/KhronosGroup/OpenCL-ICD-Loader && \
    git clone https://github.com/KhronosGroup/OpenCL-Headers && \
    rm -rf build/pose_graph_tools

RUN catkin build --continue -s -j12 rviz_map_plugin
RUN catkin build --continue -s -j12 pose_graph_tools

RUN apt update && apt install -y ros-$ROS_DISTRO-depth-image-proc && \
    rm -rf /var/lib/apt/lists/*

RUN catkin build --continue -s -j12  
