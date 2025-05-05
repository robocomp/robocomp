#!/bin/bash
set -euo pipefail

# Default values
AUTO_CONFIRM=false
INSTALL_CORTEX=0
CORES=$(grep -c ^processor /proc/cpuinfo)
JOBS=$(( CORES * 3 / 4 ))
[ "$JOBS" -lt 1 ] && JOBS=1  # Minimum 1 job

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -y|--yes)
            AUTO_CONFIRM=true
            shift
            ;;
        -j|--jobs)
            if [[ "$2" =~ ^[0-9]+$ ]]; then
                JOBS="$2"
                shift 2
            else
                echo "Error: -j requires a numeric argument" >&2
                exit 1
            fi
            ;;
        --version)
            if [[ "$2" =~ ^[a-z]+$ ]]; then
                if [ "$2" = "base" ]; then
                    INSTALL_CORTEX=-1
                elif [ "$2" = "dsr" ]; then
                    INSTALL_CORTEX=1
                else
                    echo "Error: version supported base or dsr" >&2
                    exit 1
                fi
                shift 2
            else
                echo "Error: --version requires a lowercase string argument [base/dsr]" >&2
                exit 1
            fi
            ;;
        *)
            echo "Unknown parameter: $1" >&2
            exit 1
            ;;
    esac
done

# Apply settings
export MAKEFLAGS="-j$JOBS"
echo "Using $JOBS parallel jobs for compilation"

clear


# Improved color definitions
NC='\033[0m'        # No Color
WHITE='\033[1;37m'
RED='\033[1;31m'
GREEN='\033[1;32m'
YELLOW='\033[1;33m'
BLUE='\033[1;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'

# Status message functions
status_msg() {
    echo -e "${BLUE}[*]${NC} $1"
}

success_msg() {
    echo -e "${GREEN}[✓]${NC} $1"
}

error_msg() {
    echo -e "${RED}[✗]${NC} $1"
}

warning_msg() {
    echo -e "${YELLOW}[!]${NC} $1"
}

# Display banner
echo -e "${WHITE}
▄▄▄▄▄▄              ▄▄                                                        
██▀▀▀▀██            ██                                                        
██    ██   ▄████▄   ██▄███▄    ▄████▄    ▄█████▄   ▄████▄   ████▄██▄  ██▄███▄ 
███████   ██▀  ▀██  ██▀  ▀██  ██▀  ▀██  ██▀    ▀  ██▀  ▀██  ██ ██ ██  ██▀  ▀██
██  ▀██▄  ██    ██  ██    ██  ██    ██  ██        ██    ██  ██ ██ ██  ██    ██
██    ██  ▀██▄▄██▀  ███▄▄██▀  ▀██▄▄██▀  ▀██▄▄▄▄█  ▀██▄▄██▀  ██ ██ ██  ███▄▄██▀
▀▀    ▀▀▀   ▀▀▀▀    ▀▀ ▀▀▀      ▀▀▀▀      ▀▀▀▀▀     ▀▀▀▀    ▀▀ ▀▀ ▀▀  ██ ▀▀▀  
                                                                      ██      
${NC}"

echo -e "${BLUE}Welcome to RoboComp Installation Script!${NC}\n"                     
echo -e "${WHITE}If you encounter any issues during installation, please contact us at robocomp.team@gmail.com${NC}\n"
sleep 2

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    error_msg "Do not run this script as root/sudo. You'll be prompted for privileges when needed."
    exit 1
fi

# System update and dependencies installation
status_msg "Updating system and installing dependencies..."
sudo apt-get update
sudo apt-get install -y \
    python3 \
    python3-pip \
    cmake \
    git \
    wget \
    libopenscenegraph-dev \
    libgsl-dev \
    qt6-base-dev \
    qt6-declarative-dev \
    qt6-scxml-dev \
    libqt6statemachineqml6 \
    libqt6statemachine6 \
    libbz2-dev \
    libssl-dev \
    zeroc-icebox \
    zeroc-ice-all-dev \
    libzeroc-icestorm3.7 \
    libeigen3-dev \
    meld

pip3 install vcstool PySide6 zeroc-ice toml rich

# Install libQGLViewer
status_msg "Installing libQGLViewer..."
mkdir -p ~/software
if [ ! -d ~/software/libQGLViewer ]; then
    git clone https://github.com/GillesDebunne/libQGLViewer.git ~/software/libQGLViewer
    cd ~/software/libQGLViewer && qmake6 *.pro && sudo make install -j$JOBS && sudo ldconfig && cd -
    if [ $? -ne 0 ]; then
        echo "Error: Compilation failed for libQGLViewer"
        exit 1
    fi
else
    warning_msg "libQGLViewer already exists in ~/software, skipping installation."
fi

# Install tomlplusplus
status_msg "Installing tomlplusplus..."
if [ ! -d ~/software/tomlplusplus ]; then
    git clone https://github.com/marzer/tomlplusplus.git ~/software/tomlplusplus
    cd ~/software/tomlplusplus && cmake -B build && sudo make install -C build -j$JOBS && cd -
    if [ $? -ne 0 ]; then
        echo "Error: Compilation failed for tomlplusplus"
        exit 1
    fi
else
    warning_msg "tomlplusplus already exists in ~/software, skipping installation."
fi

# Set RoboComp installation path
status_msg "Setting RoboComp installation directory..."
DEFAULT_PATH="$HOME/robocomp"
if $AUTO_CONFIRM; then
    status_msg "Set default installation directory, $DEFAULT_PATH"
    path_robocomp=$DEFAULT_PATH
else
    read -p "Where do you want to install RoboComp? [default: $DEFAULT_PATH]: " path_robocomp
    path_robocomp=${path_robocomp:-$DEFAULT_PATH}
fi

# Add environment variables to bashrc
status_msg "Adding environment variables to bashrc..."
echo "export ROBOCOMP=$path_robocomp" >> ~/.bashrc
echo "export PATH=\$PATH:$HOME/.local/bin" >> ~/.bashrc
echo "alias rcnode='bash $path_robocomp/tools/rcnode/rcnode.sh&'" >> ~/.bashrc
echo "alias cbuild='cmake -B build && make -C build -j$(nproc)'" >> ~/.bashrc
source ~/.bashrc

# Download RoboComp
status_msg "Downloading RoboComp..."
wget https://raw.githubusercontent.com/robocomp/robocomp/development/robocomp.repos -O /tmp/robocomp.repos

mkdir -p "$path_robocomp"
vcs import "$path_robocomp" < /tmp/robocomp.repos --recursive

# Create symbolic links
cd "$path_robocomp"
ln -sf core/cmake cmake
ln -sf core/classes classes
mkdir -p components

# Install CLI tools
pushd . > /dev/null
cd "$path_robocomp/tools/cli/" && pip install . && popd > /dev/null

# Create Eigen symbolic link
sudo ln -sf /usr/include/eigen3/Eigen/ /usr/include/Eigen

# Cortex installation prompt
if [ "$INSTALL_CORTEX" -eq 0 ]; then
    if $AUTO_CONFIRM; then
        INSTALL_CORTEX=1
        status_msg "Auto-confirm enabled: Cortex will be installed"
    else 
        while true; do
            read -p "Do you want to install Cortex? [Y/n]: " yn
            case $yn in
                [Yy]* ) INSTALL_CORTEX=1; break;;
                [Nn]* ) INSTALL_CORTEX=-1; break;;
                * ) INSTALL_CORTEX=1; break;;
            esac
        done
    fi
fi

if [ "$INSTALL_CORTEX" -eq 1 ]; then
    status_msg "Installing Cortex dependencies..."
    sudo apt-get install -y \
        libasio-dev \
        libtinyxml2-dev \
        libopencv-dev \
        python3-dev \
        python3-pybind11 \


    # Install cppitertools
    status_msg "Installing cppitertools..."
    if [ ! -d /usr/local/include/cppitertools ]; then
        git clone https://github.com/ryanhaining/cppitertools ~/software/cppitertools
        cd ~/software/cppitertools && cmake -B build && sudo make install -C build -j$JOBS && cd -
        if [ $? -ne 0 ]; then
            echo "Error: Compilation failed for cppitertools"
            exit 1
        fi
    else
        warning_msg "Fast-CDR already exists in ~/software, skipping installation."
    fi

    # Install FastDDS dependencies
    export MAKEFLAGS=-j$(($(grep -c ^processor /proc/cpuinfo) - 0))

    status_msg "Installing Fast-CDR..."
    if [ ! -d ~/software/Fast-CDR ]; then
        git clone https://github.com/eProsima/Fast-CDR.git ~/software/Fast-CDR
        cd ~/software/Fast-CDR && cmake -B build && sudo make install -C build -j$JOBS && cd -
        if [ $? -ne 0 ]; then
            echo "Error: Compilation failed for Fast-CDR"
            exit 1
        fi
    else
        warning_msg "Fast-CDR already exists in ~/software, skipping installation."
    fi

    status_msg "Installing foonathan_memory_vendor..."
    if [ ! -d ~/software/foonathan_memory_vendor ]; then
        git clone https://github.com/eProsima/foonathan_memory_vendor.git ~/software/foonathan_memory_vendor
        cd ~/software/foonathan_memory_vendor && cmake -B build && sudo make install -C build -j$JOBS && cd -
        if [ $? -ne 0 ]; then
            echo "Error: Compilation failed for foonathan_memory_vendor"
            exit 1
        fi
    else
        warning_msg "foonathan_memory_vendor already exists in ~/software, skipping installation."
    fi

    status_msg "Installing Fast-DDS..."
    if [ ! -d ~/software/Fast-DDS ]; then
        git clone https://github.com/eProsima/Fast-DDS.git ~/software/Fast-DDS
        cd ~/software/Fast-DDS && cmake -B build && sudo make install -C build -j$JOBS && cd -
        if [ $? -ne 0 ]; then
            echo "Error: Compilation failed for Fast-DDS"
            exit 1
        fi
        sudo ldconfig
    else
        warning_msg "Fast-DDS already exists in ~/software, skipping installation."
    fi

    # Install Cortex
    status_msg "Installing Cortex..."
    sudo cp -r $path_robocomp/classes/threadpool /usr/include/
    cd "$path_robocomp/cortex" && cmake -B build -DDSR=TRUE && sudo make install -C build -j$JOBS && cd -
    if [ $? -ne 0 ]; then
        echo "Error: Compilation failed for Cortex"
        exit 1
    fi
    sudo ldconfig
else
    status_msg "Skipping Cortex installation as requested."
fi

success_msg "RoboComp installation completed successfully!"