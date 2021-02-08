FROM nvidia/opengl:base-ubuntu20.04
# LABEL about the custom image
LABEL maintainer="emartinena@unex.es"
LABEL version="0.1"
LABEL description="This is custom Docker Image for Robocomp in Ubuntu Focal for production"
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
  libgsl23 \
  libopenscenegraph-dev \
  libqt5opengl5-dev \
  libzeroc-ice3.7 \
  libzeroc-icestorm3.7 \
  python3-prompt-toolkit \
  python3-termcolor \
  sudo \
  zeroc-icebox \
  && rm -rf /var/lib/apt/lists/*
RUN export uid=1000 gid=1000 && \
    mkdir -p /home/robolab && \
    echo "robolab:x:${uid}:${gid}:Robolab,,,:/home/robolab:/bin/bash" >> /etc/passwd && \
    echo "robolab:x:${uid}:" >> /etc/group && \
    echo "robolab:robolab"|chpasswd && \
    echo "robolab ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/robolab-user && \
    chown ${uid}:${gid} -R /home/robolab 
COPY --from=robocomp/robocomp:focal_dev /opt/robocomp /opt/robocomp/
COPY --from=robocomp/robocomp:focal_dev /etc/ld.so.conf /etc/ld.so.conf
COPY --from=robocomp/robocomp:focal_dev /etc/profile.d/robocomp.sh /etc/profile.d/robocomp.sh
COPY --from=robocomp/robocomp:focal_dev /etc/bash_completion.d/python-argcomplete /etc/bash_completion.d/python-argcomplete
COPY --from=robocomp/robocomp:focal_dev /home/robolab/files/ /home/robocomp/robocomp/files/
RUN ldconfig
ENV PATH=$PATH:/opt/robocomp/bin
USER robolab

WORKDIR /home/robocomp/robocomp/files/innermodel
# ENTRYPOINT rcis simpleworld.xml
# docker build -t robocomp/robocomp:focal .
# docker run -v /tmp/.X11-unix:/tmp/.X11-unix --env="DISPLAY=$DISPLAY" --runtime=nvidia -it robocomp:focal

