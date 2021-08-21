ARG base_image="nvidia/opengl"
ARG base_tag="base-ubuntu20.04"
ARG branch="development"
ARG robocomp_version="dsr-components"

FROM ${base_image}:${base_tag} AS dependencies-version
# Disable Prompt During Packages Installation
ARG DEBIAN_FRONTEND=noninteractive
# Update Ubuntu Software repository
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
      cmake \
      curl \
      freeglut3-dev \
      g++ \
      gcc-multilib \
      git \
      git-annex \
      libboost-dev \
      libboost-system-dev \
      libboost-thread-dev \
      libgsl-dev \
      libopenscenegraph-dev \
      libpyside2-dev \
      libqt5opengl5-dev \
      libqt5xmlpatterns5-dev \
      libzeroc-icestorm3.7 \
      make \
      python3 \
      python3-argcomplete \
      python3-distutils \
      python3-pip \
      python3-prompt-toolkit \
      python3-pyparsing \
      python3-setuptools \
      python3-termcolor \
      sudo \
      zeroc-ice-all-dev \
      zeroc-icebox \
  && rm -rf /var/lib/apt/lists/*

# Some incompatibility from pyside2 (5.15) and default qt installation (5.14)
RUN sudo pip3 install rich typer
# pyside2-tools have removed pyside2-uic
RUN echo "uic -g python \$@" >> /usr/bin/pyside2-uic && chmod a+x  /usr/bin/pyside2-uic
RUN export uid=1000 gid=1000 && \
    mkdir -p /home/robolab && \
    echo "robolab:x:${uid}:${gid}:Robolab,,,:/home/robolab:/bin/bash" >> /etc/passwd && \
    echo "robolab:x:${uid}:" >> /etc/group && \
    echo "robolab:robolab"|chpasswd && \
    echo "robolab ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/robolab-user && \
    chown ${uid}:${gid} -R /home/robolab

FROM dependencies-version AS base-version
ARG branch
USER robolab
WORKDIR /home/robolab/
# Install Robocomp
RUN echo ${branch}
ADD https://api.github.com/repos/robocomp/robocomp/git/refs/heads/${branch} version.json
RUN git clone -b ${branch} https://github.com/robocomp/robocomp.git

ENV PATH=$PATH:/opt/robocomp/bin/:'/home/robolab/.local/bin'
RUN export PATH=$PATH:/opt/robocomp/bin/:'/home/robolab/.local/bin'
RUN sudo pip install /home/robolab/robocomp/tools/cli/robocompdsl/
RUN pip install /home/robolab/robocomp/tools/cli/rcconfig/
RUN rcconfig
RUN cd robocomp && mkdir build && cd build && cmake .. && make -j$(nproc) && sudo make install
RUN sudo sh -c "echo '/opt/robocomp/lib/' >> /etc/ld.so.conf"
RUN sudo ldconfig
WORKDIR robocomp
ENV ROBOCOMP=/opt/robocomp/
RUN git annex get \
  files/innermodel/simpleworld.xml \
  files/osgModels/textures/wood.jpg \
  files/innermodel/tar36h11-*.png \
  files/osgModels/textures/Metal.jpg \
  files/osgModels/robex/robex.ive
RUN cp -r -L files /home/robolab/files 2>/dev/null || :


FROM base-version AS fcl-version
USER root
RUN apt update && apt-get install --no-install-recommends --yes \
  libfcl-dev
USER robolab
RUN cd /home/robolab/robocomp && mkdir -p build && cd build && cmake -DFCL_SUPPORT=True .. && make -j$(nproc)
USER root
RUN cd /home/robolab/robocomp/build && make install
USER robolab


FROM fcl-version AS dsr-version
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
  g++-10 \
  gcc-10 \
  libeigen3-dev \
  python3-dev \
  python3-pybind11 \
  libqglviewer-dev-qt5 \
  libasio-dev \
  libtinyxml2-dev  \
  libqt5svg5-dev \
  && rm -rf /var/lib/apt/lists/*
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 1
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 1
RUN python3 -m pip install pyrep

# ICE fix
COPY Connection.h /usr/include/Ice/Connection.h

### Third party dependencies
RUN curl -S https://raw.githubusercontent.com/robocomp/robocomp/development/libs/dsr/dependencies.sh | bash -x
RUN rm -r /usr/local/include/cppitertools
RUN git clone https://github.com/ryanhaining/cppitertools /usr/local/include/cppitertools


#Compilation of DSR libs in Robocomp
USER robolab
ENV PATH $PATH:/opt/robocomp/bin/
RUN export PATH=$PATH:/opt/robocomp/bin/
RUN cd ~/robocomp/ && git pull && mkdir -p build && cd build && cmake -DFCL_SUPPORT=True -DDSR=True  .. && make -j$(nproc)
USER root
RUN cd /home/robolab/robocomp/build && make install
RUN ldconfig


FROM dsr-version AS dsr-components-version
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
  libopencv-dev \
  python3-zeroc-ice  \
  psmisc \
  && rm -rf /var/lib/apt/lists/*

#Compilation of DSR components
USER robolab
WORKDIR /home/robolab/robocomp/components/
RUN git clone --progress https://github.com/robocomp/dsr-graph.git
WORKDIR /home/robolab/robocomp/components/dsr-graph/components/
USER robolab
ENV ROBOCOMP=/home/robolab/robocomp
RUN cd idserver && cmake . && make -j10
RUN cd pioneer_dsr && cmake . && make -j10
RUN cd path_follower && cmake . && make -j10
RUN cd path_planner_astar && cmake . && make -j10
RUN cd elastic_band && cmake . && make -j10


FROM ${robocomp_version}-version AS final
ARG robocomp_version
RUN echo "Built ${robocomp_version}"