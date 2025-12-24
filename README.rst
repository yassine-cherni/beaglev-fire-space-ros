===============================================
BeagleV-Fire Space-ROS Yocto Build System
===============================================

Professional Yocto/OpenEmbedded build system for BeagleV-Fire with ROS2 Jazzy and Space-ROS support.

.. warning::

   **Project Status: Under Active Development**

   This project is in active development. Build configurations and features may change.
   Community contributions are welcome.

   **Last Updated:** December 24, 2025

Overview
========

This repository provides a complete Yocto build configuration for the BeagleV-Fire RISC-V single-board computer with FPGA capabilities, featuring ROS2 Jazzy, Space-ROS framework, and hardware acceleration support.

**Key Features:**

- Yocto Scarthgap 5.0.14 LTS (EOL April 2028)
- ROS2 Jazzy LTS (EOL May 2029)
- Space-ROS NASA robotics framework
- FPGA acceleration capabilities
- Linux 6.12.x LTS kernel
- Modular KAS-based configuration

Quick Start
===========

.. code-block:: bash

   # Clone and setup
   git clone https://github.com/yassine-cherni/beaglev-fire-space-ros.git
   cd beaglev-fire-space-ros
   ./setup.sh

   # Build (2-4 hours first time)
   ./kas-container build kas/beaglev-fire-space-ros.yml

   # Output: build/tmp/deploy/images/beaglev-fire/core-image-minimal-beaglev-fire.wic.gz

Documentation
=============

Complete documentation is available in the ``docs/`` directory:

**Getting Started:**

- `Project Overview <docs/overview.rst>`_ - Detailed project information and roadmap
- `Hardware Specifications <docs/hardware.rst>`_ - BeagleV-Fire technical details
- `Installation Guide <docs/installation.rst>`_ - Prerequisites and setup
- `Building Images <docs/building.rst>`_ - Build process and workflow

**Development:**

- `Architecture <docs/architecture.rst>`_ - Repository structure and layers
- `Recipe Management <docs/recipes.rst>`_ - Creating and managing Yocto recipes with MASH
- `Customization <docs/customization.rst>`_ - Adding packages and features
- `FPGA Acceleration <docs/fpga.rst>`_ - Hardware acceleration (planned)

**Support:**

- `Troubleshooting <docs/troubleshooting.rst>`_ - Common issues and solutions
- `Contributing <docs/contributing.rst>`_ - Contribution guidelines

System Requirements
===================

**Host System:**

- Ubuntu 22.04/24.04 LTS (recommended)
- 150GB free disk space
- 16GB RAM minimum (32GB recommended)
- Docker or Podman installed

**Installation:** See `Installation Guide <docs/installation.rst>`_

License
=======

MIT License - See LICENSE file for details.

**Third-Party Components:** Yocto Project, ROS2, Space-ROS, Linux Kernel - See respective licenses.

Links
=====

- **Documentation:** `docs/ <docs/>`_
- **Issues:** https://github.com/yassine-cherni/beaglev-fire-space-ros/issues
- **Discussions:** https://github.com/yassine-cherni/beaglev-fire-space-ros/discussions

Acknowledgments
===============

- Rob Woolley - MASH tool creator + meta-ros Maintainer
- BeagleBoard.org Foundation
- Microchip Technology
- NASA/JPL Space-ROS team
- Open Source Robotics Foundation

---

Built for the Space-ROS and RISC-V communities

*Last Updated: December 24, 2025*