FROM nvidia/opengl:base-ubuntu20.04
# LABEL about the custom image
LABEL maintainer="emartinena@unex.es"
LABEL version="0.1"
LABEL description="This is custom Docker Image for Robocomp in Ubuntu Focal for production"
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libgsl23 \
    libgssapi-krb5-2 \
    libopenscenegraph-dev \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xinerama0 \
    libxcb-xkb1 \
    libxkbcommon-x11-0 \
    libzeroc-ice3.7 \
    libzeroc-icestorm3.7 \
    python3 \
    python3-argcomplete \
    python3-pip \
    python3-prompt-toolkit \
    python3-pyparsing \
    python3-termcolor \
    sudo \
    zeroc-icebox \
  && rm -rf /var/lib/apt/lists/*
RUN pip3 install rich pyside2
RUN echo "uic -g python \$@" >> /usr/bin/pyside2-uic && chmod a+x  /usr/bin/pyside2-uic
RUN export uid=1000 gid=1000 && \
    mkdir -p /home/robolab && \
    echo "robolab:x:${uid}:${gid}:Robolab,,,:/home/robolab:/bin/bash" >> /etc/passwd && \
    echo "robolab:x:${uid}:" >> /etc/group && \
    echo "robolab:robolab"|chpasswd && \
    echo "robolab ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/robolab-user && \
    chown robolab:robolab -R /home/robolab
RUN ln -s /home/robolab /home/robocomp
COPY --from=robocomp/robocomp:focal_dev /opt/robocomp /opt/robocomp/
COPY --from=robocomp/robocomp:focal_dev /home/robolab/files/ /home/robolab/robocomp/files/
RUN chown robolab:robolab -R /home/robocomp
RUN chown robolab:robolab -R /home/robolab/
RUN ldconfig
RUN activate-global-python-argcomplete3 --dest /etc/profile.d/
ENV PATH=$PATH:/opt/robocomp/bin
USER robolab
WORKDIR /home/robolab/
RUN export QT_QPA_PLATFORM_PLUGIN_PATH="/usr/local/lib/python3.8/dist-packages/PySide2/Qt/plugins/platforms/"
ENV QT_QPA_PLATFORM_PLUGIN_PATH="/usr/local/lib/python3.8/dist-packages/PySide2/Qt/plugins/platforms/"
ENV PATH $PATH:/opt/robocomp/bin/
# ENTRYPOINT rcis simpleworld.xml
# docker build -t robocomp/robocomp:focal .
# docker run -v /tmp/.X11-unix:/tmp/.X11-unix --env="DISPLAY=$DISPLAY" --runtime=nvidia -it robocomp:focal

