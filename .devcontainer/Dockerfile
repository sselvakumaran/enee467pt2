FROM ros:humble as base

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
# Extended stage for Nvidia GPU support (Requires NVIDIA Container Toolkit)
FROM base as nvidia

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# Extended stage for AMD GPU support (Requires AMD ROCm software)
FROM base as amd

# Setup AMD ROCm
ARG ROCM_VERSION=5.3
ARG AMDGPU_VERSION=5.3

ARG APT_PREF
RUN echo "$APT_PREF" > /etc/apt/preferences.d/rocm-pin-600
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils curl libnuma-dev \
  && curl -sL https://repo.radeon.com/rocm/rocm.gpg.key | apt-key add - \
  && printf "deb [arch=amd64] https://repo.radeon.com/rocm/apt/$ROCM_VERSION/ jammy main" | tee /etc/apt/sources.list.d/rocm.list \
  && printf "deb [arch=amd64] https://repo.radeon.com/amdgpu/$AMDGPU_VERSION/ubuntu jammy main" | tee /etc/apt/sources.list.d/amdgpu.list \
  && apt-get update && apt-get install -y --no-install-recommends \
  kmod \
  file \
  python3-pip \
  # rocm-cmake \
  # rocm-device-libs \
  # rocm-utils \
  rocm-dev \
  rocm-libs && \
  apt-get clean && \
  rm -rf /var/lib/apt/lists/*

RUN  groupadd -g 109 render

ENV DEBIAN_FRONTEND=
