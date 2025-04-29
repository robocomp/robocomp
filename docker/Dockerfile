FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm
ENV TZ=Europe/Madrid

# Config tzdata
RUN ln -fs /usr/share/zoneinfo/${TZ} /etc/localtime && \
    apt-get update && \
    apt-get install -y --no-install-recommends tzdata && \
    echo ${TZ} > /etc/timezone && \
    dpkg-reconfigure --frontend noninteractive tzdata && \
    rm -rf /var/lib/apt/lists/*

# Create non-root user
RUN useradd -m robouser && \
    apt-get update && \
    apt-get install -y sudo git && \
    echo "robouser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to non-root user
USER robouser
WORKDIR /home/robouser

# Define las variables de entorno necesarias para RoboComp
ENV USER="robouser"
ENV PATH="$PATH:/home/$USER/.local/bin"
ENV ROBOCOMP="/home/robouser/robocomp"

# Clone and install
RUN git clone https://github.com/robocomp/robocomp.git && \
    cd robocomp &&  \
    bash robocomp_install.sh -y

RUN sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

CMD ["tail", "-f", "/dev/null"]
