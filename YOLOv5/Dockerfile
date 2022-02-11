FROM nvidia/cuda:11.3.0-runtime-ubuntu20.04

ENV TZ=Asia/Tokyo
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt -y clean && \
    rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

RUN apt update && \
    apt install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && \
    apt install -y  -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" keyboard-configuration && \
    apt install -y ros-foxy-desktop && \
    apt install -y python3-colcon-common-extensions && \
    apt install -y git && \
    apt install -y xterm && \
    apt install -y wget && \
    apt install -y ros-foxy-v4l2-camera && \
    apt -y clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt update && \
    apt install -y python3-pip && \
    apt install -y python3-opencv && \
    apt -y clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt update && \
    pip3 install -U pip && \
    apt -y clean && \
    rm -rf /var/lib/apt/lists/*

# install pytorch using pip
RUN pip3 install torch==1.9.0+cu111 torchvision==0.10.0+cu111 torchaudio==0.9.0 -f https://download.pytorch.org/whl/torch_stable.html
RUN pip3 install opencv-python matplotlib numpy Pillow requests PyYAML requests scipy tqdm tensorboard pandas seaborn

# RUN ln -s /usr/local/cuda/lib64/stubs/libcuda.so /usr/local/cuda/lib64/stubs/libcuda.so.1
# RUN LD_LIBRARY_PATH=/usr/local/cuda/lib64/stubs/:$LD_LIBRARY_PATH

CMD ["bash"]
WORKDIR /ros2_ws
WORKDIR /target-dir

# ================================
# docker build . -t challenge-yolov5
# docker run --rm -it --runtime nvidia --device /dev/video0:/dev/video0:mwr -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --mount type=bind,source=$(pwd)/target-dir,target=/target-dir challenge-yolov5 /bin/bash