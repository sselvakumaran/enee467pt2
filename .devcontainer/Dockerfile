FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Additional setup
RUN apt-get update \
 && apt-get install -y -qq --no-install-recommends \
  wget \
  git \
  libxext-dev \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libxext6 \
  libx11-6

ENV QT_X11_NO_MITSHM 1

# Add a non-root user
ARG USERNAME=467-terp
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV ROS_DISTRO=humble

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

ENV DEBIAN_FRONTEND=
