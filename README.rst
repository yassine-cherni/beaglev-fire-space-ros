===============================================
BeagleV-Fire Space-ROS Yocto Build System
===============================================

.. warning::
   
   **⚠️ PROJECT STATUS: UNDER ACTIVE DEVELOPMENT ⚠️**
   
   This project is currently in **testing phase** and under active development.
   
   - Build configurations may change frequently
   - Some features may not work as expected
   - Documentation is being continuously updated
   - Use at your own risk in production environments
   
   **Last Updated:** December 2025
   
   **Tested Configurations:**
   
   ✓ Ubuntu 22.04 LTS
   
   ✓ Ubuntu 24.04 LTS
   
   ✗ Production deployment (not yet recommended)

Professional Yocto/OpenEmbedded build system for BeagleV-Fire with ROS2 Jazzy and Space-ROS support.

.. contents:: Table of Contents
   :depth: 3
   :local:

Overview
========

This repository provides a complete Yocto build configuration for the BeagleV-Fire single-board computer, featuring:

- **Yocto Scarthgap 5.0.13+** - Latest LTS release
- **ROS2 Jazzy** - Long Term Support (EOL May 2029)
- **Space-ROS** - NASA's robotics framework for space applications
- **Linux 6.6.x** - LTS kernel with PolarFire SoC support ( Migrating to 6.12.x )
- **Systemd** - Modern init system
- **Professional kas configuration** - Modular, maintainable structure

Target Hardware
===============

BeagleV-Fire Specifications
----------------------------

:SoC: Microchip PolarFire MPFS025T FPGA SoC
:CPU: 5-core RISC-V (1x E51 monitor + 4x U54 application cores @ 625MHz)
:Architecture: RV64IMAFDC (RV64GC)
:Memory: 2GB LPDDR4
:Storage: 16GB eMMC, 128MB SPI Flash, microSD slot
:FPGA: 23K Logic Elements, 68 Math Blocks, 4x 12.7 Gbps SERDES
:Connectivity: Gigabit Ethernet, USB-C, M.2 E-Key, SYZYGY
:Camera: 22-pin CSI connector
:Expansion: BeagleBone Cape compatibility

Prerequisites
=============

Host System Requirements
------------------------

**Operating System:**

- Ubuntu 20.04, 22.04, 24.04 LTS (tested and recommended)
- Fedora 40, 41
- Debian 12
- Other Linux distributions may work but are not officially supported

**Hardware Requirements:**

- **Disk Space:** 100GB+ free (150GB+ recommended)
- **RAM:** 16GB minimum (32GB recommended)
- **CPU:** Multi-core processor (more cores = faster build)
- **Internet:** Fast, stable connection (first build downloads ~50GB)

**Install Docker or Podman:**

.. code-block:: bash

   # Docker (recommended)
   sudo apt-get install -y docker.io
   sudo usermod -aG docker $USER
   
   # Log out and log back in for group changes to take effect
   
   # Verify Docker installation
   docker --version

First-Time Setup
================

Step 1: Clone the Repository
-----------------------------

.. code-block:: bash

   git clone https://github.com/yassine-cherni/beaglev-fire-space-ros.git
   cd beaglev-fire-space-ros

Step 2: Make Setup Scripts Executable
--------------------------------------

.. code-block:: bash

   chmod +x setup.sh

.. note::
   
   The ``chmod +x`` command makes scripts executable. You only need to do this once.

Step 3: Run Initial Setup
--------------------------

.. code-block:: bash

   ./setup.sh

**What this does:**

1. Downloads ``kas-container`` (version 5.1) if not present
2. Creates ``downloads/`` directory for persistent source caches
3. Creates ``sstate-cache/`` directory for persistent build artifacts

**Expected output:**

.. code-block:: text

   kas-container (version 5.1) downloaded and made executable.
   Persistent directories created: downloads and sstate-cache.
   Setup complete! Use ./kas-container shell kas/beaglev-fire-space-ros.yml to enter the build shell.

Step 4: Verify Configuration
-----------------------------

Test the kas configuration without building:

.. code-block:: bash

   ./kas-container shell kas/beaglev-fire-space-ros.yml
  
Building Images
===============

First Build (Standard Method)
------------------------------

.. warning::
   
   **First build takes 2-4 hours** depending on your system and internet speed.
   
   - Downloads ~50GB of source code
   - Compiles thousands of packages
   - Requires significant CPU and RAM
   - Do NOT interrupt the build process

Build the minimal image with ROS2 support:

.. code-block:: bash

   ./kas-container build kas/beaglev-fire-space-ros.yml

**Build progress indicators:**

- You'll see tasks being executed: ``do_fetch``, ``do_unpack``, ``do_compile``, etc.
- Progress format: ``Currently X running tasks (Y of Z)``
- **Do not worry about warnings** - only errors will stop the build

**Successful build output location:**

.. code-block:: text

   build/tmp/deploy/images/beaglev-fire/
   └── core-image-minimal-beaglev-fire.wic.gz
