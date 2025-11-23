FROM ubuntu:22.04

# --- ROS 2 Humbleのための前準備 ---
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# --- ROS 2 Humbleのリポジトリ登録 ---
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# --- ROS 2 Humbleのインストール ---
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    iputils-ping \
    ros-humble-rclpy \
    ros-humble-rmw-cyclonedds-cpp \  
    && rm -rf /var/lib/apt/lists/*

# rosdep 初期化
RUN rosdep init || true
RUN rosdep update || true

# --- 作業ディレクトリ ---
WORKDIR /app

# --- Python環境 ---
RUN apt-get update && apt-get install -y \
    sudo python3-pip python3-setuptools python3-venv \
    open-jtalk open-jtalk-mecab-naist-jdic alsa-utils pulseaudio-utils \
    curl ffmpeg unzip libatomic1  git \
    && rm -rf /var/lib/apt/lists/*

# pip パッケージ
RUN pip install --no-cache-dir requests python-dotenv vosk pyserial gpiod

ARG USER_NAME
ARG USER_ID
ARG USER_GID
ARG GPIO_GID
RUN mkdir -p /home/${USER_NAME} && chown ${USER_ID}:${USER_GID} /home/${USER_NAME}
RUN mkdir -p /home/${USER_NAME}/.config/pulse && chown -R ${USER_ID}:${USER_GID} /home/${USER_NAME}/.config
RUN echo "${USER_NAME}:x:${USER_ID}:${USER_GID}::/home/${USER_NAME}:/bin/bash" >> /etc/passwd

# グループ作成（存在しなければ作成）
RUN groupadd -f -g ${USER_GID} ${USER_NAME} \
    && groupadd -f -g ${GPIO_GID} gpio

# ユーザー作成（存在しなければ作成） + sudo 権限付与
RUN apt-get update && apt-get install -y sudo \
    && id -u ${USER_NAME} &>/dev/null || useradd -m -u ${USER_ID} -g ${USER_GID} -s /bin/bash ${USER_NAME} \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && passwd -d ${USER_NAME} \
    && usermod -U ${USER_NAME} \
    && rm -rf /var/lib/apt/lists/*

# ホームディレクトリ権限
RUN chown -R ${USER_NAME}:${USER_NAME} /home/${USER_NAME}

# --- デフォルトユーザー切り替え ---
USER ${USER_ID}:${USER_GID}

# --- ROS 2 環境をユーザーの bashrc に追加 ---
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USER_NAME}/.bashrc

# --- 作業ディレクトリにコードコピー ---
COPY . /app
COPY voice/hts-voice /usr/share/hts-voice

# --- ROS 2 環境変数 ---
ENV ROS_PYTHON_VERSION=3
ENV PATH=/opt/ros/humble/bin:$PATH
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH
ENV COLCON_PREFIX_PATH=/opt/ros/humble:$COLCON_PREFIX_PATH