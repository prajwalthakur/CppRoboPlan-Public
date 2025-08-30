FROM  nvidia/cuda:12.6.0-cudnn-devel-ubuntu22.04

RUN apt-get update && apt-get upgrade -y

# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Kolkata

# Pre-seed tzdata so it never asks
RUN echo "tzdata tzdata/Areas select Asia"       | debconf-set-selections \
 && echo "tzdata tzdata/Zones/Asia select Kolkata" | debconf-set-selections

RUN apt-get install --no-install-recommends -y \
    software-properties-common \
    vim \
    python3-pip\
    tmux \
    git \
    wget \
    curl \
    doxygen \
    graphviz \
    libgtest-dev

# Install git + CA bundle
RUN apt-get update && apt-get install -y --no-install-recommends \
      git ca-certificates openssl \
 && update-ca-certificates \
 && git config --system http.sslCAInfo /etc/ssl/certs/ca-certificates.crt \
 && rm -rf /var/lib/apt/lists/*

# (optional) sanity checks
RUN test -s /etc/ssl/certs/ca-certificates.crt && \
    git --version && \
    ls -l /etc/ssl/certs/ca-certificates.crt




# Added updated mesa drivers for integration with cpu - https://github.com/ros2/rviz/issues/948#issuecomment-1428979499

RUN add-apt-repository ppa:kisak/kisak-mesa && \
    apt-get update && apt-get upgrade -y &&\
    apt-get install libxcb-cursor0 -y && \
    apt-get install ffmpeg python3-opengl -y

RUN apt-get update && apt install -y cmake && \
    apt-get install --no-install-recommends ca-certificates gnupg lsb-release wget

# ------------------------------------------------------------------------------
# 2) Drake binary repo for Meshcat C++ (drake-dev)
# ------------------------------------------------------------------------------

RUN wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - \
| tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null
RUN echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" \
| tee /etc/apt/sources.list.d/drake.list >/dev/null
RUN apt-get update -y
RUN apt-get install -y --no-install-recommends drake-dev 


# ------------------------------------------------------------------------------
# 3) robotpkg for Pinocchio (python 3.10 build on 22.04)
# ------------------------------------------------------------------------------
    RUN install -d -m 0755 /etc/apt/keyrings \
    && curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
       -o /etc/apt/keyrings/robotpkg.asc
   
   RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] \
        http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
        | tee /etc/apt/sources.list.d/robotpkg.list
   
   RUN apt-get update \
    && apt-get install -y --no-install-recommends \
         robotpkg-py310-pinocchio robotpkg-py310-example-robot-data \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------------------
# 4) Environment for runtime / build
# ------------------------------------------------------------------------------
ENV PATH=/opt/openrobots/bin:/opt/drake/bin:$PATH \
    LD_LIBRARY_PATH=/opt/openrobots/lib:/opt/drake/lib:$LD_LIBRARY_PATH \
    PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH \
    PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:/opt/drake/lib/python3.10/site-packages:$PYTHONPATH \
    CMAKE_PREFIX_PATH=/opt/openrobots:/opt/drake:$CMAKE_PREFIX_PATH

# Copy workspace files
ENV WORKSPACE_PATH=/root/workspace
COPY workspace/ $WORKSPACE_PATH/src/


# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Final cleanup
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Set default shell to bash
CMD ["/bin/bash"]
