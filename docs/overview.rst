================
Project Overview
================

.. contents:: Table of Contents
   :depth: 2
   :local:

Introduction
============

The BeagleV-Fire Space-ROS project delivers a production-ready Yocto/OpenEmbedded build system for deploying NASA's Space-ROS framework on RISC-V hardware with FPGA acceleration capabilities.

This project bridges embedded Linux, robotics middleware, and programmable logic to enable next-generation space applications on open-source hardware.

Core Technologies
=================

Operating System Stack
----------------------

**Yocto Scarthgap 5.0.14**

- Long Term Support release (EOL April 2028)
- Security updates and bug fixes
- Production-grade embedded Linux
- Customizable root filesystem

**Linux Kernel 6.12.x**

- LTS kernel with extended support
- PolarFire SoC BSP integration
- RISC-V optimizations
- Real-time patches (planned)

**Systemd Init System**

- Modern service management
- Dependency-based boot ordering
- Journal-based logging
- Network configuration

Robotics Framework
------------------

**ROS2 Jazzy Jalisco**

- Long Term Support (EOL May 2029)
- Latest DDS middleware
- Real-time capabilities
- Hardware acceleration APIs

**Space-ROS**

- NASA's robotics framework for space
- Safety-critical design patterns
- Deterministic behavior
- Radiation-tolerant considerations

Hardware Platform
-----------------

**RISC-V ISA**

- Open instruction set architecture
- Growing ecosystem
- Custom extensions possible
- No licensing fees

**PolarFire FPGA SoC**

- Programmable logic + processors
- Hardware acceleration potential
- Low power consumption
- Radiation-tolerant variants available

Project Goals
=============

Primary Objectives
------------------

1. **Space-Grade Linux Distribution**
   
   Create a specialized Linux distribution optimized for space applications with real-time capabilities, safety-critical toolchain integration, and minimal footprint.

2. **Robotics Demonstration Platform**
   
   Provide working implementations of Canadarm2 robotic arm control and Mars Curiosity Rover navigation systems with complete simulation and hardware control stacks.

3. **FPGA Acceleration Framework**
   
   Enable hardware-accelerated computation for ROS2 nodes, computer vision processing, and sensor fusion using the PolarFire FPGA fabric.

4. **Reproducible Build System**
   
   Maintain a KAS-based configuration that ensures consistent, reproducible builds across different development environments.

Roadmap
=======

Phase 1: Foundation (Current)
------------------------------

**Status: In Progress**

- Base Yocto Scarthgap + ROS2 Jazzy integration - COMPLETE
- BeagleV-Fire BSP with PolarFire SoC support - COMPLETE
- Space-ROS demo applications integration - COMPLETE
- Custom recipe generation using MASH tool - IN PROGRESS
- RAUC OTA update system integration - IN PROGRESS

**Deliverables:**

- Bootable SD card image with ROS2
- Space-ROS demo packages included
- Development tools installed
- Documentation complete

Phase 2: Space Applications Distro
-----------------------------------

**Status: Planned**

- Dedicated Linux distro for space-certified applications
- Real-time kernel patches for deterministic behavior
- Safety-critical toolchain integration (ELISA compliance)
- Footprint optimization for embedded space systems
- Secure boot implementation
- Verified boot chain

**Deliverables:**

- Custom Space-ROS distro configuration
- RT-Linux kernel integration
- Security hardening documentation
- Certification readiness assessment

Phase 3: Robotics Integration
------------------------------

**Status: Planned**

- Canadarm2 full simulation and control stack
- Mars Curiosity Rover navigation and perception
- MoveIt2 motion planning with hardware acceleration
- Gazebo simulation with FPGA co-processing
- ROS2 Control hardware interfaces
- Sensor drivers for space-grade hardware

**Deliverables:**

- Working Canadarm2 demonstration
- Curiosity Rover navigation stack
- Hardware control interfaces
- Simulation environments

Phase 4: FPGA Acceleration
---------------------------

**Status: Research Phase**

- Hardware-accelerated computer vision (OpenCV on FPGA)
- DDS middleware offload to programmable logic
- Custom ROS2 compute nodes in FPGA fabric
- Real-time sensor fusion acceleration
- Power-optimized AI inference on PolarFire FPGA
- Custom IP cores for robotics workloads

**Deliverables:**

- FPGA reference designs
- Hardware acceleration examples
- Performance benchmarks
- Integration documentation

Target Applications
===================

Space Systems
-------------

**Satellite Operations**

- Attitude determination and control
- Payload management
- Communication protocols
- Power management

**Robotic Arms**

- Canadarm2-style manipulators
- Precision motion control
- Force/torque feedback
- Collision avoidance

**Planetary Rovers**

- Autonomous navigation
- Terrain mapping
- Sample collection
- Remote operation

Terrestrial Applications
-------------------------

**Research Platforms**

- University robotics labs
- Algorithm development
- Hardware-in-the-loop testing
- Education and training

**Industrial Automation**

- Robotic manipulation
- Quality inspection
- Warehouse automation
- Collaborative robots

**Autonomous Systems**

- Unmanned vehicles
- Agricultural robots
- Search and rescue
- Infrastructure inspection

Why This Matters
=================

Open Source Space Robotics
---------------------------

Traditional space robotics systems rely on proprietary, closed-source solutions with high costs and vendor lock-in. This project demonstrates:

- Open-source from silicon (RISC-V) to software (ROS2)
- Community-driven development
- Transparent, auditable systems
- Accessible to universities and small organizations

FPGA + RISC-V Convergence
--------------------------

The combination of RISC-V processors and FPGA fabric enables:

- Custom hardware acceleration without ASIC development
- Adaptable to mission-specific requirements
- Power efficiency through hardware offload
- Real-time deterministic behavior

Reproducible Embedded Linux
----------------------------

Yocto provides the foundation for:

- Bit-exact reproducible builds
- Complete supply chain transparency
- Security update management
- Long-term support lifecycle

Technical Specifications
========================

Software Versions
-----------------

===============================  ==================  =================
Component                        Version             EOL Date
===============================  ==================  =================
Yocto Project                    Scarthgap 5.0.14    April 2028
ROS2                             Jazzy Jalisco       May 2029
Linux Kernel                     6.12.x              December 2026
BitBake                          2.8                 April 2028
KAS                              5.1                 Active
===============================  ==================  =================

Repository Statistics
---------------------

==================  ============
Metric              Value
==================  ============
Custom Layers       3
Recipe Files        50+
Build Time          2-4 hours
Disk Usage          150GB
Download Size       50GB
Supported Boards    1 (growing)
==================  ============

Success Criteria
================

A successful Phase 1 delivery includes:

1. Bootable image for BeagleV-Fire
2. ROS2 Jazzy operational
3. Space-ROS demos functional
4. Documentation complete
5. Reproducible builds verified
6. CI/CD pipeline established

Future success will be measured by:

- Expanded hardware support
- FPGA acceleration benchmarks
- Community adoption
- Academic publications
- Real-world deployments

Related Projects
================

Upstream Dependencies
---------------------

- **Yocto Project**: https://www.yoctoproject.org/
- **ROS2**: https://docs.ros.org/
- **Space-ROS**: https://space.ros.org/
- **meta-ros**: https://github.com/ros/meta-ros
- **MASH**: https://github.com/robwoolley/mash

Similar Initiatives
-------------------

- **meta-spaceros**: NASA's Space-ROS layers
- **meta-ros**: Official ROS Yocto layers
- **ELISA Project**: Linux for safety-critical systems
- **RISC-V Robotics SIG**: RISC-V robotics working group

Community Resources
===================

Getting Help
------------

- GitHub Issues: Project-specific bugs and features
- GitHub Discussions: General questions and ideas
- ROS Discourse: ROS2 and Space-ROS topics
- Yocto Mailing List: Build system questions
- BeagleBoard Forum: Hardware-specific questions

Contributing
------------

See `Contributing Guide <contributing.rst>`_ for:

- Code contribution process
- Documentation standards
- Testing requirements
- Review guidelines

Stay Updated
------------

- Watch the GitHub repository
- Join project discussions
- Subscribe to release notifications
- Follow BeagleBoard.org announcements

Contact
=======

**Project Maintainer:** Yassine Cherni

**Project Repository:** https://github.com/yassine-cherni/beaglev-fire-space-ros

**License:** MIT License

---

Next: `Hardware Specifications <hardware.rst>`_