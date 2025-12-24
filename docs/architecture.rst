============
Architecture
============

.. contents:: Table of Contents
   :depth: 2
   :local:

Repository Structure
====================

Directory Layout
----------------

.. code-block:: text

   beaglev-fire-space-ros/
   ├── kas/                          # KAS configuration files
   │   ├── beaglev-fire-space-ros.yml  # Main build configuration
   │   ├── yocto/
   │   │   └── scarthgap.yml         # Yocto release definition
   │   ├── machine/
   │   │   └── beaglev-fire.yml      # Hardware BSP configuration
   │   ├── spaceros/
   │   │   └── jazzy.yml             # ROS2 + Space-ROS layers
   │   ├── common.yml                # Shared build settings
   │   ├── systemd.yml               # Init system config
   │   ├── rauc.yml                  # OTA updates (future)
   │   ├── diskmon.yml               # Build disk monitoring
   │   └── limit-pressure.yml        # CPU/IO pressure limits
   ├── layers/                       # Custom Yocto layers
   │   ├── meta-beaglev-fire-bsp/    # Board-specific patches
   │   ├── meta-spaceros-app/        # Space-ROS demo apps
   │   └── meta-spaceros-distro/     # Custom distro configuration
   ├── downloads/                    # Source code cache (gitignored)
   ├── sstate-cache/                 # Build cache (gitignored)
   ├── build/                        # Build output (gitignored)
   ├── docs/                         # Documentation
   ├── setup.sh                      # First-time setup script
   ├── kas-container                 # KAS container wrapper
   ├── README.rst                    # Project overview
   └── LICENSE                       # MIT License

Key Directories
---------------

**kas/**
  Configuration files defining the build. Each .yml file configures a specific aspect.

**layers/**
  Custom Yocto layers containing recipes, configurations, and patches specific to this project.

**downloads/**
  Cached source code tarballs. Persists across builds. Not in git.

**sstate-cache/**
  Cached build artifacts. Dramatically speeds up rebuilds. Not in git.

**build/**
  Generated during build. Contains all intermediate and final outputs. Not in git.

**docs/**
  Complete project documentation in reStructuredText format.

KAS Configuration System
=========================

What is KAS?
------------

KAS is a setup tool for BitBake-based projects. It:

- Manages layer dependencies
- Clones git repositories
- Configures builds declaratively
- Ensures reproducible environments
- Runs in containers for isolation

Configuration Files
-------------------

Main Configuration
^^^^^^^^^^^^^^^^^^

**kas/beaglev-fire-space-ros.yml**

Main entry point. Includes other configuration files and defines:

- Build targets (core-image-minimal)
- Custom repositories (local layers)
- Project-specific settings
- Package installation list

.. code-block:: yaml

   header:
     version: 20
     includes:
       - kas/yocto/scarthgap.yml
       - kas/machine/beaglev-fire.yml
       - kas/spaceros/jazzy.yml
       - kas/common.yml
       - kas/diskmon.yml
       - kas/systemd.yml
       - kas/rauc.yml
       - kas/limit-pressure.yml

   build_system: oe
   
   repos:
     meta-beaglev-fire-bsp:
       path: layers/meta-beaglev-fire-bsp
     meta-spaceros-app:
       path: layers/meta-spaceros-app
     meta-spaceros-distro:
       path: layers/meta-spaceros-distro

   target:
     - core-image-minimal

Yocto Release Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**kas/yocto/scarthgap.yml**

Defines Yocto Project version and core layers:

- openembedded-core (Yocto base)
- bitbake (build engine)
- meta-openembedded (additional packages)

Pinned to specific commits for reproducibility.

.. code-block:: yaml

   repos:
     openembedded-core:
       url: "https://github.com/openembedded/openembedded-core.git"
       commit: "2b3d2b671a149cbeea2bdc9ba42192da2015c3b7"
       path: "layers/openembedded-core"

Machine Configuration
^^^^^^^^^^^^^^^^^^^^^

**kas/machine/beaglev-fire.yml**

Hardware-specific settings:

- Machine name: beaglev-fire
- BSP layers (meta-mchp)
- PolarFire SoC support

.. code-block:: yaml

   machine: "beaglev-fire"
   repos:
     meta-mchp:
       url: "https://github.com/linux4microchip/meta-mchp.git"
       branch: "scarthgap"

ROS2 Configuration
^^^^^^^^^^^^^^^^^^

**kas/spaceros/jazzy.yml**

ROS2 and Space-ROS layers:

- meta-ros (official ROS layers)
- ROS2 Jazzy distribution
- Space-ROS packages

.. code-block:: yaml

   repos:
     meta-ros:
       url: "https://github.com/ros/meta-ros.git"
       branch: "scarthgap"
       layers:
         meta-ros-common:
         meta-ros2:
         meta-ros2-jazzy:

Common Settings
^^^^^^^^^^^^^^^

**kas/common.yml**

Shared configuration for all builds:

- DISTRO_FEATURES
- IMAGE_FEATURES
- Build optimizations (rm_work)
- License acceptance

.. code-block:: yaml

   local_conf_header:
     common: |
       DISTRO_FEATURES += "usrmerge"
       IMAGE_FEATURES += "empty-root-password"
       INHERIT += "rm_work"

Systemd Configuration
^^^^^^^^^^^^^^^^^^^^^

**kas/systemd.yml**

Enables systemd as init system:

.. code-block:: yaml

   local_conf_header:
     systemd: |
       DISTRO_FEATURES += "pam systemd"
       VIRTUAL-RUNTIME_init_manager = "systemd"

RAUC Configuration
^^^^^^^^^^^^^^^^^^

**kas/rauc.yml**

OTA update system configuration (planned):

.. code-block:: yaml

   repos:
     meta-rauc:
       url: "https://github.com/rauc/meta-rauc.git"
       branch: "scarthgap"

Resource Limits
^^^^^^^^^^^^^^^

**kas/diskmon.yml**

Disk space monitoring to prevent build failures:

.. code-block:: yaml

   BB_DISKMON_DIRS ?= "\
       STOPTASKS,${TMPDIR},1G,100K \
       HALT,${TMPDIR},100M,1K"

**kas/limit-pressure.yml**

System resource limits:

.. code-block:: yaml

   BB_NICE_LEVEL = "11"
   BB_PRESSURE_MAX_CPU = "1000000"
   BB_PRESSURE_MAX_IO = "50000"
   BB_PRESSURE_MAX_MEMORY = "10000"

Layer Architecture
==================

Yocto Layer Hierarchy
---------------------

.. code-block:: text

   Core Layers (from Yocto Project)
   ├── openembedded-core/meta
   └── meta-openembedded/
       ├── meta-oe
       ├── meta-python
       ├── meta-networking
       ├── meta-multimedia
       ├── meta-perl
       └── meta-filesystems

   BSP Layers (Hardware Support)
   └── meta-mchp/
       ├── meta-mchp-common
       └── meta-mchp-polarfire-soc/
           ├── meta-mchp-polarfire-soc-bsp
           └── meta-mchp-polarfire-soc-community

   ROS2 Layers
   └── meta-ros/
       ├── meta-ros-common
       ├── meta-ros2
       └── meta-ros2-jazzy

   Custom Layers (This Project)
   ├── meta-beaglev-fire-bsp
   ├── meta-spaceros-app
   └── meta-spaceros-distro

Layer Dependencies
------------------

.. code-block:: text

   meta-beaglev-fire-bsp
   └── depends on: core, meta-mchp

   meta-spaceros-app
   └── depends on: core, meta-ros2-jazzy

   meta-spaceros-distro
   └── depends on: core

Custom Layers
=============

meta-beaglev-fire-bsp
---------------------

**Purpose:** Board-specific configurations and patches

**Structure:**

.. code-block:: text

   meta-beaglev-fire-bsp/
   ├── conf/
   │   └── layer.conf            # Layer configuration
   ├── recipes-kernel/
   │   └── linux/                # Kernel patches (future)
   ├── recipes-bsp/
   │   └── u-boot/               # Bootloader patches (future)
   ├── wic/
   │   └── beaglev-fire-rauc.wks # Partition layout
   ├── COPYING.MIT
   └── README

**layer.conf:**

.. code-block:: bitbake

   BBPATH .= ":${LAYERDIR}"
   BBFILES += "${LAYERDIR}/recipes-*/*/*.bb"
   BBFILE_COLLECTIONS += "meta-beaglev-fire-bsp"
   BBFILE_PRIORITY_meta-beaglev-fire-bsp = "6"
   LAYERSERIES_COMPAT_meta-beaglev-fire-bsp = "scarthgap"

meta-spaceros-app
-----------------

**Purpose:** Space-ROS demonstration applications

**Structure:**

.. code-block:: text

   meta-spaceros-app/
   ├── conf/
   │   └── layer.conf
   ├── recipes-spaceros/
   │   ├── canadarm_demo/
   │   │   └── canadarm-demo_1.0.0.bb
   │   ├── canadarm_description/
   │   │   └── canadarm-description_1.0.0.bb
   │   ├── canadarm_gazebo/
   │   │   └── canadarm-gazebo_0.1.0.bb
   │   ├── canadarm_moveit_config/
   │   │   └── canadarm-moveit-config_0.3.0.bb
   │   ├── canadarm_wrench_publisher/
   │   │   └── canadarm-wrench-publisher_0.0.0.bb
   │   ├── curiosity_description/
   │   │   └── curiosity-description_1.0.0.bb
   │   ├── curiosity_gazebo/
   │   │   └── curiosity-gazebo_0.1.0.bb
   │   ├── curiosity_rover_demo/
   │   │   └── curiosity-rover-demo_0.0.1.bb
   │   ├── ros_trick_bridge/
   │   │   └── ros-trick-bridge_0.0.0.bb
   │   ├── trick_canadarm_moveit_config/
   │   │   └── trick-canadarm-moveit-config_0.3.0.bb
   │   └── trick_ros2_control/
   │       └── trick-ros2-control_0.0.0.bb
   ├── COPYING.MIT
   └── README

**Recipe Example:**

.. code-block:: bitbake

   DESCRIPTION = "Canadarm demonstration package"
   LICENSE = "Apache-2.0"
   
   inherit ros_distro_jazzy
   
   SRC_URI = "git://github.com/space-ros/canadarm.git;protocol=https;branch=main"
   SRCREV = "..."
   
   S = "${WORKDIR}/git"
   
   ROS_BUILD_TYPE = "ament_cmake"

meta-spaceros-distro
--------------------

**Purpose:** Custom distribution configuration

**Structure:**

.. code-block:: text

   meta-spaceros-distro/
   ├── conf/
   │   ├── layer.conf
   │   └── distro/
   │       └── spaceros.conf         # Custom distro definition (future)
   ├── COPYING.MIT
   └── README

**Future distro configuration:**

.. code-block:: bitbake

   require conf/distro/poky.conf
   
   DISTRO = "spaceros"
   DISTRO_NAME = "Space-ROS"
   DISTRO_VERSION = "jazzy"
   
   DISTRO_FEATURES:append = " systemd"
   ROS_DISTRO = "jazzy"

Build System Flow
=================

Build Process Steps
-------------------

1. **KAS Initialization**
   
   - Parse kas/beaglev-fire-space-ros.yml
   - Process included files
   - Clone all repositories
   - Check out specific commits

2. **BitBake Setup**
   
   - Generate build/conf/bblayers.conf
   - Generate build/conf/local.conf
   - Parse all layers
   - Build recipe index

3. **Dependency Resolution**
   
   - Read all recipes
   - Build dependency tree
   - Determine build order
   - Identify parallel tasks

4. **Task Execution**
   
   - Fetch sources
   - Unpack and patch
   - Configure packages
   - Compile sources
   - Install to staging
   - Create packages
   - Generate rootfs
   - Create bootable image

5. **Output Generation**
   
   - WIC image creation
   - Compression
   - Manifest generation
   - Deploy to output directory

BitBake Variables
-----------------

Key variables used in configuration:

**MACHINE**
  Hardware platform (beaglev-fire)

**DISTRO**
  Distribution name (spaceros)

**IMAGE_FEATURES**
  Features to include in image

**IMAGE_INSTALL**
  Packages to install in image

**TMPDIR**
  Build temporary directory (build/tmp)

**DL_DIR**
  Download cache directory (downloads/)

**SSTATE_DIR**
  Shared state cache (sstate-cache/)

**DEPLOY_DIR_IMAGE**
  Final image output location

Configuration Files
-------------------

Generated configuration:

**build/conf/bblayers.conf**

Lists all layers to use:

.. code-block:: bitbake

   BBLAYERS ?= " \
     /work/layers/openembedded-core/meta \
     /work/layers/meta-openembedded/meta-oe \
     /work/layers/meta-ros/meta-ros-common \
     /work/layers/meta-ros/meta-ros2 \
     /work/layers/meta-ros/meta-ros2-jazzy \
     /work/layers/meta-spaceros-app \
     ..."

**build/conf/local.conf**

Build-specific configuration from all KAS files.

Integration Points
==================

ROS2 Integration
----------------

**meta-ros Layers:**

- meta-ros-common: Common ROS infrastructure
- meta-ros2: ROS2 core packages
- meta-ros2-jazzy: Jazzy-specific packages

**Key Variables:**

.. code-block:: bitbake

   ROS_DISTRO = "jazzy"
   ROS_OE_RELEASE_SERIES = "scarthgap"

**Package Groups:**

- packagegroup-ros-world-jazzy: All available ROS2 packages
- packagegroup-ros-core-jazzy: Minimal ROS2 installation

Space-ROS Integration
----------------------

Custom packages in meta-spaceros-app:

- Canadarm2 robotic arm demos
- Mars Curiosity Rover navigation
- NASA TRICK simulation bridge

**Installation:**

.. code-block:: yaml

   IMAGE_INSTALL:append = " \
       canadarm-demo \
       curiosity-rover-demo \
       ros-trick-bridge"

Systemd Integration
-------------------

**Services:**

- ROS2 daemon
- Network management
- SSH server
- Custom application services

**Unit Files:**

Located in recipes, e.g.:

.. code-block:: text

   recipes-spaceros/canadarm-demo/files/canadarm-demo.service

RAUC Integration (Planned)
---------------------------

**Features:**

- A/B rootfs partitions
- Atomic updates
- Rollback capability
- Update verification

**Partition Scheme:**

.. code-block:: text

   /boot     - Bootloader and kernel
   /         - Root filesystem A
   (unused)  - Root filesystem B
   /data     - Persistent data

Data Flow
=========

Source to Binary
----------------

.. code-block:: text

   Source Repository (GitHub)
        |
        | git clone
        v
   downloads/ (tarball cache)
        |
        | unpack
        v
   build/tmp/work/.../package/version/
        |
        | configure, compile, install
        v
   build/tmp/work/.../package/version/image/
        |
        | package (IPK)
        v
   build/tmp/deploy/ipk/
        |
        | install to rootfs
        v
   build/tmp/work/core-image-minimal/rootfs/
        |
        | create filesystem
        v
   build/tmp/deploy/images/beaglev-fire/
        |
        | WIC image
        v
   core-image-minimal-beaglev-fire.wic.gz

Shared State Cache
------------------

.. code-block:: text

   Source Code
        |
        v
   Compile (first time)
        |
        | save to sstate-cache/
        v
   sstate-cache/XX/sstate:package:arch:version:hash.tgz
        |
        | reuse on next build
        v
   Compile (skipped) - Extract from cache

Development Workflow
====================

Typical Development Cycle
--------------------------

1. **Modify source code or recipe**

   .. code-block:: bash
   
      vim layers/meta-spaceros-app/recipes-spaceros/my-pkg/my-pkg_1.0.0.bb

2. **Clean package**

   .. code-block:: bash
   
      ./kas-container shell kas/beaglev-fire-space-ros.yml
      bitbake -c cleansstate my-pkg

3. **Rebuild package**

   .. code-block:: bash
   
      bitbake my-pkg

4. **Test in devshell**

   .. code-block:: bash
   
      bitbake -c devshell my-pkg
      # Manual testing

5. **Rebuild full image**

   .. code-block:: bash
   
      exit  # from devshell
      bitbake core-image-minimal

6. **Flash and test on hardware**

Adding New Packages
-------------------

See `Recipe Management Guide <recipes.rst>`_ for detailed instructions.

Quick workflow:

1. Generate recipe with MASH
2. Place in appropriate layer
3. Add to IMAGE_INSTALL in KAS config
4. Build and test

Customizing Images
------------------

See `Customization Guide <customization.rst>`_ for options.

Common customizations:

- Add/remove packages
- Change init system
- Modify kernel configuration
- Add custom services
- Adjust image features

Maintenance
===========

Updating Dependencies
---------------------

**Update layer commits:**

Edit kas/*.yml files:

.. code-block:: yaml

   repos:
     openembedded-core:
       commit: "new-commit-hash"

**Update all at once:**

.. code-block:: bash

   # Update to latest scarthgap
   cd layers/openembedded-core
   git fetch
   git log --oneline origin/scarthgap
   # Copy desired commit hash to kas/yocto/scarthgap.yml

Version Management
------------------

**Yocto Release:**

Currently Scarthgap 5.0.14. Update by changing commit in kas/yocto/scarthgap.yml.

**ROS2 Distribution:**

Jazzy (fixed). Changing requires different meta-ros2-* layer.

**Kernel Version:**

Managed by meta-mchp BSP. Update BSP commit for kernel updates.

Security Updates
----------------

**Process:**

1. Monitor security advisories
2. Update affected layer commits
3. Test build
4. Deploy updated images

**Sources:**

- Yocto Project security page
- CVE databases
- Vendor security bulletins

Best Practices
==============

Repository Management
---------------------

**Git Workflow:**

- Main branch: stable releases
- Development branch: active development
- Feature branches: new features
- Tag releases: v1.0.0, v1.1.0

**Commit Messages:**

.. code-block:: text

   layer-name: brief description
   
   Detailed explanation of changes.
   Why the change was made.
   
   Signed-off-by: Your Name <your.email@example.com>

Configuration Management
------------------------

**KAS Files:**

- Keep modular (separate concerns)
- Document changes
- Pin to specific commits
- Test before committing

**Layers:**

- Follow Yocto naming conventions
- Use appropriate BBFILE_PRIORITY
- Document dependencies
- Maintain layer.conf compatibility

Recipe Quality
--------------

**Recipe Standards:**

- Follow Yocto style guide
- Include all required fields
- Test on clean build
- Document patches
- Use appropriate licenses

**Testing:**

- Build from scratch
- Test on hardware
- Verify dependencies
- Check for warnings

Documentation
-------------

**Keep Updated:**

- README for quick start
- Detailed docs for reference
- Inline comments for complex parts
- Changelog for releases

**Format:**

- reStructuredText for consistency
- Clear headings and structure
- Code examples
- Cross-references

Related Resources
=================

Yocto Project
-------------

- Manual: https://docs.yoctoproject.org/
- Reference Manual: https://docs.yoctoproject.org/ref-manual/
- Dev Manual: https://docs.yoctoproject.org/dev-manual/

KAS
---

- Documentation: https://kas.readthedocs.io/
- GitHub: https://github.com/siemens/kas

ROS2
----

- ROS2 Docs: https://docs.ros.org/
- meta-ros: https://github.com/ros/meta-ros

BitBake
-------

- User Manual: https://docs.yoctoproject.org/bitbake/

---

Next: `Recipe Management <recipes.rst>`_