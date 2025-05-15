# Use NVIDIA CUDA base image
FROM nvidia/cuda:12.1.1-cudnn8-devel-ubuntu22.04

ARG ROS_DISTRO=humble
ENV ROS_DISTRO=${ROS_DISTRO}
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# noninteractive APT, enable GPU + graphics capabilities
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,video,graphics,display

# Set environment variables
ENV PATH=/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

# Set up timezone and locale in one layer
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    tzdata \
    locales \
    gnupg2 \
    curl \
    ca-certificates && \
    echo 'Etc/UTC' > /etc/timezone && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Verify and update NVIDIA repositories if needed
RUN rm -f /usr/share/keyrings/cuda-archive-keyring.gpg && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub | gpg --dearmor --batch -o /usr/share/keyrings/cuda-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/cuda-archive-keyring.gpg] https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/ /" > /etc/apt/sources.list.d/cuda.list && \
    rm -f /etc/apt/sources.list.d/cuda-ubuntu2204-x86_64.list

# Update CUDA and cuDNN to ensure specific versions
RUN apt-get update && \
    apt-get install -y --allow-change-held-packages \
    cuda-toolkit-12-1 \
    libcudnn8 \
    libcudnn8-dev && \
    rm -rf /var/lib/apt/lists/*

# Add apt repositories
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release software-properties-common apt-transport-https ca-certificates && \
    add-apt-repository -y universe && \
    # Add ROS2 repository
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install SO dependencies
# Install SO dependencies in smaller batches to avoid timeouts
RUN apt-get update -qq && \
    apt-get install -y \
    build-essential \
    sudo \
    cmake \
    git \
    libgtk2.0-dev \
    libgtk-3-dev \
    pkg-config \
    iputils-ping \
    wget \
    python3-pip \
    python3-dev \
    libtool \
    git-all \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update -qq && \
    apt-get install -y \
    software-properties-common \
    bash-completion \
    curl \
    tmux \
    zsh \
    nano \
    xvfb \
    ansible \
    qt5ct \
    rsync \
    && rm -rf /var/lib/apt/lists/*

# install navpilot dependencies
#GTSAM librari
RUN add-apt-repository ppa:borglab/gtsam-release-4.1

RUN apt-get update -qq && \
    apt-get install -y \
    libeigen3-dev \
    libpcl-dev \
    libpcap-dev \
    can-utils \
    libqt5serialport5-dev \
    libpugixml-dev \
    geographiclib-tools \
    libgeographic-dev \
    libgtsam-unstable-dev \
    libgtsam-dev \
    && rm -rf /var/lib/apt/lists/*

# Install PyTorch, TorchVision, and Torchaudio
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# Install required Python packages
RUN pip3 install --no-cache-dir \
    numpy==1.26.4 \
    matplotlib==3.5.1 \
    tensorboardX \
    open3d \
    tqdm \
    pillow \
    opencv-python \
    trimesh 

# Install ROS2
RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-desktop-full && \
    rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
RUN apt-get update && \
    apt-get install -y \
        python3-colcon-common-extensions \
        ros-${ROS_DISTRO}-color-util \
        ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-foxglove-bridge \
        ros-${ROS_DISTRO}-image-common \
        ros-${ROS_DISTRO}-image-transport-plugins \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-pcl-msgs \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-perception-pcl \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-tracetools-launch \
        ros-${ROS_DISTRO}-velodyne-msgs \
        ros-${ROS_DISTRO}-vision-msgs \
        ros-${ROS_DISTRO}-vision-opencv \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-cv-bridge && \
        rm -rf /var/lib/apt/lists/*

# Clean up
RUN apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Set up zsh with Oh My Zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended && \
    chsh -s $(which zsh)

RUN echo 'export TERM=xterm-256color' >> ~/.zshrc && \
    echo 'alias ll="ls -alF"' >> ~/.zshrc && \
    echo 'alias la="ls -A"' >> ~/.zshrc && \
    echo 'alias l="ls -CF"' >> ~/.zshrc && \
    echo 'export ZSH_THEME="robbyrussell"' >> ~/.zshrc && \
    echo 'PROMPT="%F{yellow}%*%f %F{green}%~%f %F{blue}➜%f "' >> ~/.zshrc

# Source ROS setup scripts
SHELL ["/bin/bash", "-c"]
#let this space
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash"   >> ~/.bashrc \
 && echo "source /opt/ros/${ROS_DISTRO}/setup.zsh"    >> ~/.zshrc \
 && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc \
 && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.zshrc

# Set up tmux configuration
RUN echo 'set -g default-terminal "screen-256color"' >> ~/.tmux.conf && \
    echo 'set -g mouse on' >> ~/.tmux.conf

# set up rviz2 dark theme
WORKDIR /workspace
RUN git clone https://github.com/armando-genis/rviz_dark_custom_theme.git

WORKDIR /workspace/rviz_dark_custom_theme
RUN sed -i 's|qt5ct_conf_file:.*|qt5ct_conf_file: "/workspace/rviz_dark_custom_theme/qt5ct.conf"|' main.yml && \
    sed -i 's|me_qss_file:.*|me_qss_file: "/workspace/rviz_dark_custom_theme/rviz.qss"|' main.yml

RUN ansible-playbook -i localhost, main.yml
ENV QT_QPA_PLATFORMTHEME=qt5ct

WORKDIR /workspace/rviz_dark_custom_theme/icons
RUN cp map_point.svg /opt/ros/humble/share/rviz_default_plugins/icons/classes/PublishPoint.svg && \
    cp robot.png /opt/ros/humble/share/rviz_default_plugins/icons/classes/SetInitialPose.png && \
    cp 2dgoal.png /opt/ros/humble/share/rviz_default_plugins/icons/classes/SetGoal.png

# Set up workspace
WORKDIR /workspace
ENV PATH="/root/.local/bin:${PATH}"

