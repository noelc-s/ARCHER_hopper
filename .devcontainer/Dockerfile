FROM nvidia/cuda:12.1.0-devel-ubuntu22.04
LABEL authors="ivan"

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=user
ARG GROUPID
ARG USERID

## Manage user
# Create user
RUN groupadd --gid $GROUPID $USER_NAME

RUN useradd --uid $USERID --gid $GROUPID -ms /bin/bash $USER_NAME
RUN echo 'docker:docker' | chpasswd
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USER_NAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER_NAME \
    && chmod 0440 /etc/sudoers.d/$USER_NAME

# Add user group to docker user
RUN usermod -aG sudo $USER_NAME
RUN usermod -aG root $USER_NAME

# ## Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime

## Install some packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      apt-transport-https \
      apt-utils \
      ca-certificates \
      curl \
      gnupg2 \
      lsb-release \
      software-properties-common \
      wget

# ## Install dependencies
RUN apt-get update && \
    apt-get install -y \
    cmake \
    gcc-12 \
    g++-12 \
    git \
    libboost-all-dev \
    make \
    ninja-build \
    zsh

RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 12 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 12

## Switch to user
USER $USER_NAME
ENV HOME /home/$USER_NAME

##  Copy bash_aliases
ADD --chown=$USER_NAME:$USER_NAME setup/setup.bash $HOME/
ADD --chown=$USER_NAME:$USER_NAME setup/.bashrc $HOME/
ADD --chown=$USER_NAME:$USER_NAME setup/BezierTubes $HOME/libs/BezierTubes
ADD --chown=$USER_NAME:$USER_NAME setup/cddlib $HOME/libs/cddlib

WORKDIR $HOME/libs 

RUN sudo apt-get update && \
sudo apt-get install -y libglfw3-dev liburdfdom-dev python3-numpy libeigen3-dev libxinerama-dev libxcursor-dev\
                        libxi-dev texlive-latex-base

# RUN git clone --branch 3.4.1 https://gitlab.com/libeigen/eigen.git && \
#     cd eigen && \
#     mkdir build && \
#     cd build && \
#     cmake .. && \
#     sudo cmake --build . --target install

RUN git clone --branch 0.8.0 https://github.com/jbeder/yaml-cpp.git && \
    cd yaml-cpp && \
    mkdir build && \
    cd build && \
    cmake .. -G Ninja -DYAML_BUILD_SHARED_LIBS=on && \
    sudo ninja install

RUN git clone --recurse-submodules --branch v0.6.3 https://github.com/osqp/osqp.git && \
    cd osqp && \
    mkdir build && \
    cd build && \
    cmake .. -G Ninja && \
    sudo ninja install 

RUN git clone --recurse-submodules --branch v0.8.1 https://github.com/robotology/osqp-eigen.git && \
    cd osqp-eigen && \
    mkdir build && \
    cd build && \
    cmake .. -G Ninja && \
    sudo ninja install     

#Pinnochio Dependencies 

RUN git clone --recursive -b v3.1.1 https://github.com/stack-of-tasks/eigenpy.git && \
    cd eigenpy && \
    mkdir build && \
    cd build && \
    cmake .. -G Ninja -DPYTHON_EXECUTABLE=$(which python3) -DCMAKE_BUILD_TYPE=Release && \
    sudo ninja install 

RUN git clone --recursive --branch pinocchio3-preview https://github.com/stack-of-tasks/pinocchio && \
    cd pinocchio && \
    mkdir build && \
    cd build && \
    cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && \
    sudo ninja install
RUN git clone --recursive --branch 2.3.7 https://github.com/deepmind/mujoco.git && \
    cd mujoco && \
    mkdir build && \
    cd build && \
    cmake .. && \
    cmake --build . && \
    sudo cmake --install . 
    
RUN git clone --recursive https://github.com/artivis/manif.git && \
    cd manif && \
    mkdir build && \
    cd build && \
    cmake .. -G Ninja && \
    sudo ninja install

RUN cd cddlib && \
    mkdir build && \
    cd build && \
    cmake .. -G Ninja && \
    sudo ninja install

RUN git clone --recursive https://github.com/vsamy/eigen-cddlib.git && \
    cd eigen-cddlib && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -GNinja && \
    sudo ninja install

RUN cd BezierTubes && \
    mkdir build && \
    cd build && \
    cmake .. -G Ninja && \
    ninja

RUN sudo apt-get install python3-pip && \
    sudo pip install networkx
WORKDIR $HOME
ENTRYPOINT ["/home/docker/setup.bash"]
CMD ["bash"]
