ARG base_image=ubuntu
ARG base_tag=22.04
FROM ${base_image}:${base_tag}

ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm
ENV TZ=Europe/Madrid

# Config tzdata
RUN ln -fs /usr/share/zoneinfo/${TZ} /etc/localtime && \
    apt-get update && \
    apt-get install -y --no-install-recommends tzdata && \
    echo ${TZ} > /etc/timezone && \
    dpkg-reconfigure --frontend noninteractive tzdata

# Create non-root user
RUN useradd -m robolab && \
    apt-get update && \
    apt-get install -y sudo git wget && \
    echo "robolab ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to non-root user
USER robolab
WORKDIR /home/robolab

ENV USER="robolab"
ENV PATH="$PATH:/home/$USER/.local/bin"
ENV ROBOCOMP="/home/robolab/robocomp"

# Clone and install
ARG robocomp_version=base
ARG branch=development
RUN wget https://raw.githubusercontent.com/robocomp/robocomp/${branch}/robocomp_install.sh && \
    bash robocomp_install.sh -y -j 2 --version ${robocomp_version}

# Clean
USER root
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*

USER robolab
CMD ["tail", "-f", "/dev/null"]
