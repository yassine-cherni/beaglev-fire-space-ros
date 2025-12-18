#!/bin/bash
# BeagleV-Fire Space-ROS Setup Script
# Run this to create all kas configuration files

set -e

echo "Creating directory structure..."
mkdir -p kas/yocto
mkdir -p kas/machine
mkdir -p kas/spaceros

echo "Creating kas/yocto/scarthgap.yml..."
cat > kas/yocto/scarthgap.yml << 'EOF'
header:
  version: 14

defaults:
  repos:
    branch: "scarthgap"

# https://downloads.yoctoproject.org/releases/yocto/yocto-5.0.13/RELEASENOTES
repos:
  openembedded-core:
    url: "https://github.com/openembedded/openembedded-core.git"
    commit: "7af6b75221d5703ba5bf43c7cd9f1e7a2e0ed20b"
    path: "layers/openembedded-core"
    layers:
      meta:

  bitbake:
    url: "https://github.com/openembedded/bitbake.git"
    branch: "2.8"
    commit: "1c9ec1ffde75809de34c10d3ec2b40d84d258cb4"
    path: "layers/bitbake"
    layers:
      bitbake: "disabled"

  meta-openembedded:
    url: "https://github.com/openembedded/meta-openembedded.git"
    commit: "15e18246dd0c0585cd1515a0be8ee5e2016d1329"
    path: "layers/meta-openembedded"
    layers:
      meta-filesystems:
      meta-multimedia:
      meta-networking:
      meta-oe:
      meta-perl:
      meta-python:

local_conf_header:
  standard: |
    CONF_VERSION = "2"
    PACKAGE_CLASSES ?= "package_rpm"
    SANITY_TESTED_DISTROS ?= " "
    
  parallel: |
    BB_NUMBER_THREADS ?= "${@oe.utils.cpu_count()}"
    PARALLEL_MAKE ?= "-j ${@oe.utils.cpu_count()}"
    
  downloads: |
    DL_DIR ?= "${TOPDIR}/../downloads"
    SSTATE_DIR ?= "${TOPDIR}/../sstate-cache"

  distro: |
    WARN_QA:remove = "license-exists"
EOF

echo "Creating kas/machine/beaglev-fire.yml..."
cat > kas/machine/beaglev-fire.yml << 'EOF'
header:
  version: 14

machine: "beaglev-fire"

repos:
  meta-mchp:
    url: "https://github.com/linux4microchip/meta-mchp.git"
    branch: "scarthgap"
    commit: "b04ce9003b12e1f681e1b9eb2e4582de22aebf4f"
    path: "layers/meta-mchp"
    layers:
      meta-mchp-common:
      meta-mchp-polarfire-soc/meta-mchp-polarfire-soc-bsp:
      meta-mchp-polarfire-soc/meta-mchp-polarfire-soc-community:

local_conf_header:
  machine: |
    # BeagleV-Fire specific configurations
    PREFERRED_PROVIDER_virtual/kernel = "linux-polarfire-soc"
    PREFERRED_VERSION_linux-polarfire-soc = "6.6%"
EOF

echo "Creating kas/spaceros/jazzy.yml..."
cat > kas/spaceros/jazzy.yml << 'EOF'
header:
  version: 14

repos:
  meta-ros:
    url: "https://github.com/ros/meta-ros.git"
    branch: "scarthgap"
    commit: "HEAD"
    path: "layers/meta-ros"
    layers:
      meta-ros-common:
      meta-ros2:
      meta-ros2-jazzy:

local_conf_header:
  ros2: |
    # ROS2 Jazzy Configuration
    ROS_DISTRO = "jazzy"
    ROS_OE_RELEASE_SERIES = "scarthgap"
    
  spaceros: |
    # Space-ROS specific packages
    IMAGE_INSTALL:append = " packagegroup-ros-world-jazzy"
EOF

echo "Creating kas/common.yml..."
cat > kas/common.yml << 'EOF'
header:
  version: 14

local_conf_header:
  common: |
    DISTRO_FEATURES += "usrmerge"
    IMAGE_FEATURES += "empty-root-password allow-empty-password allow-root-login post-install-logging"
    INHERIT += "rm_work"
    LICENSE_FLAGS_ACCEPTED += "commercial"
    USER_CLASSES ?= "buildstats"
EOF

echo "Creating kas/diskmon.yml..."
cat > kas/diskmon.yml << 'EOF'
header:
  version: 14

local_conf_header:
  diskmon: |
    BB_DISKMON_DIRS ?= "\
        STOPTASKS,${TMPDIR},1G,100K \
        STOPTASKS,${DL_DIR},1G,100K \
        STOPTASKS,${SSTATE_DIR},1G,100K \
        STOPTASKS,/tmp,100M,100K \
        HALT,${TMPDIR},100M,1K \
        HALT,${DL_DIR},100M,1K \
        HALT,${SSTATE_DIR},100M,1K \
        HALT,/tmp,10M,1K"
EOF

echo "Creating kas/limit-pressure.yml..."
cat > kas/limit-pressure.yml << 'EOF'
header:
  version: 14

local_conf_header:
  bb_limit_pressure: |
    BB_NICE_LEVEL = "11"
    BB_PRESSURE_MAX_CPU = "1000000"
    BB_PRESSURE_MAX_IO = "50000"
    BB_PRESSURE_MAX_MEMORY = "10000"
EOF

echo "Creating kas/systemd.yml..."
cat > kas/systemd.yml << 'EOF'
header:
  version: 14

local_conf_header:
  systemd: |
    DISTRO_FEATURES += " pam systemd "
    DISTRO_FEATURES_BACKFILL_CONSIDERED += "sysvinit"
    VIRTUAL-RUNTIME_init_manager = "systemd"
EOF

echo "Creating kas/beaglev-fire-space-ros.yml (main composition file)..."
cat > kas/beaglev-fire-space-ros.yml << 'EOF'
header:
  version: 14
  includes:
    - kas/yocto/scarthgap.yml
    - kas/machine/beaglev-fire.yml
    - kas/spaceros/jazzy.yml
    - kas/common.yml
    - kas/diskmon.yml
    - kas/systemd.yml

build_system: oe

target:
  - core-image-minimal

local_conf_header:
  project: |
    # BeagleV-Fire Space-ROS Project
    DISTRO = "ros2"
    
    # Additional image features
    IMAGE_FEATURES += "splash package-management ssh-server-dropbear"
    
    # Development tools
    IMAGE_INSTALL:append = " \
        htop \
        tmux \
        vim \
        git \
        python3 \
        python3-pip \
        i2c-tools \
        can-utils \
        usbutils \
        pciutils \
    "
EOF

echo ""
echo "✅ All kas configuration files created successfully!"
echo ""
echo "Directory structure:"
tree kas/ || find kas -type f

echo ""
echo "Next steps:"
echo "1. Run: ./setup.sh"
echo "2. Build: ./kas-container build kas/beaglev-fire-space-ros.yml"
echo "3. Or shell: ./kas-container shell kas/beaglev-fire-space-ros.yml"