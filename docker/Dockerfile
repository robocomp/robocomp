ARG base_image="nvidia/opengl"
ARG base_tag="base-ubuntu20.04"
ARG branch="development"
ARG robocomp_version="dsr-components"

FROM ${base_image}:${base_tag} AS dependencies-version
# Disable Prompt During Packages Installation
ENV DEBIAN_FRONTEND=noninteractive
ARG branch
RUN echo "branch: ${branch}"
# DEFAULT LD_LIBRARY_PATH vaiables from the nvidia/opengl image confuses pyside2 about the qt5 Library paths
ENV LD_LIBRARY_PATH="/lib:/usr/lib"
RUN export LD_LIBRARY_PATH="/lib:/usr/lib"
RUN export LC_ALL=C.UTF-8
RUN export LANG=C.UTF-8
RUN apt-get update && apt-get install -y --no-install-recommends  \
      ca-certificates \
      curl \
      sudo \
    && rm -rf /var/lib/apt/lists/* \
RUN echo "branch: ${branch}"
RUN cd ~ && export branch=$branch \
    && curl -sL https://raw.githubusercontent.com/robocomp/robocomp/$branch/tools/install/resources/robocomp_prerequisites_install.sh
RUN export uid=1000 gid=1000 && \
    mkdir -p /home/robolab && \
    echo "robolab:x:${uid}:${gid}:Robolab,,,:/home/robolab:/bin/bash" >> /etc/passwd && \
    echo "robolab:x:${uid}:" >> /etc/group && \
    echo "robolab:robolab"|chpasswd && \
    echo "robolab ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/robolab-user && \
    chown ${uid}:${gid} -R /home/robolab
USER robolab
WORKDIR /home/robolab/


FROM dependencies-version AS base-version
ARG branch
# Using robocomp installation script
ADD cmake_binary_download.sh /home/robolab/
RUN export LC_ALL=C.UTF-8
RUN export LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8
RUN bash -x cmake_binary_download.sh
RUN echo "branch: ${branch}"
RUN cd ~ && export branch=$branch \
    && curl -sL https://raw.githubusercontent.com/robocomp/robocomp/$branch/tools/install/robocomp_install.sh | bash -x -s $branch
ENV PATH=$PATH:/opt/robocomp/bin/:'/home/robolab/.local/bin'
ENV ROBOCOMP=/home/robolab/robocomp
RUN export ROBOCOMP=/home/robolab/robocomp
RUN export PATH=$PATH:/opt/robocomp/bin/:'/home/robolab/.local/bin'
#RUN sudo pip3 install /home/robolab/robocomp/tools/cli/
#RUN cd robocomp && rcconfig init && mkdir build && cd build && cmake .. && make -j$(nproc) && sudo make install
#RUN sudo sh -c "echo '/opt/robocomp/lib/' >> /etc/ld.so.conf"
#RUN sudo ldconfig
WORKDIR robocomp
RUN git annex get \
  files/innermodel/simpleworld.xml \
  files/osgModels/textures/wood.jpg \
  files/innermodel/tar36h11-*.png \
  files/osgModels/textures/Metal.jpg \
  files/osgModels/robex/robex.ive
RUN cp -r -L files /home/robolab/files 2>/dev/null || :


FROM base-version AS fcl-version
RUN export LC_ALL=C.UTF-8
RUN export LANG=C.UTF-8
USER root
RUN apt update && apt-get install --no-install-recommends --yes \
  libfcl-dev
USER robolab
RUN cd /home/robolab/robocomp && rcconfig init && mkdir -p build && cd build && cmake -DFCL_SUPPORT=True .. && make -j$(nproc)
USER root
RUN cd /home/robolab/robocomp/build && make install
USER robolab


FROM fcl-version AS dsr-version
USER root
RUN add-apt-repository ppa:ubuntu-toolchain-r/test && \
    apt-get update && apt-get install -y --no-install-recommends \
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
RUN (curl -O https://raw.githubusercontent.com/robocomp/robocomp/development/classes/threadpool/threadpool.h && mkdir -p /usr/include/threadpool && mv threadpool.h /usr/include/threadpool )
RUN curl -S https://raw.githubusercontent.com/robocomp/cortex/development/installation.sh | bash -x
RUN rm -r /usr/local/include/cppitertools
RUN git clone https://github.com/ryanhaining/cppitertools /usr/local/include/cppitertools
USER robolab


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
RUN cd idserver && cmake . && make -j10
RUN cd pioneer_dsr && cmake . && make -j10
RUN cd path_follower && cmake . && make -j10
RUN cd path_planner_astar && cmake . && make -j10
RUN cd elastic_band && cmake . && make -j10


FROM ${robocomp_version}-version AS final
ARG robocomp_version
RUN echo "Built ${robocomp_version}"
