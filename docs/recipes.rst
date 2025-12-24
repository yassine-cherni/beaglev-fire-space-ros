==================
Recipe Management
==================

.. contents:: Table of Contents
   :depth: 2
   :local:

Introduction
============

This guide covers creating, managing, and maintaining Yocto recipes for ROS2 packages, with emphasis on using the MASH tool for automated recipe generation.

What are Recipes?
-----------------

BitBake recipes (.bb files) are:

- Instructions for building software packages
- Metadata about source code location
- Build dependencies and requirements
- Installation and packaging instructions

**Example recipe structure:**

.. code-block:: bitbake

   DESCRIPTION = "Package description"
   LICENSE = "Apache-2.0"
   LIC_FILES_CHKSUM = "file://LICENSE;md5=..."
   
   SRC_URI = "git://github.com/..."
   SRCREV = "commit-hash"
   
   inherit ros_distro_jazzy
   
   S = "${WORKDIR}/git"
   ROS_BUILD_TYPE = "ament_cmake"

MASH Tool Overview
==================

What is MASH?
-------------

**MASH (Meta Automatic Superflore Helper)** is a tool that automatically generates Yocto recipes from ROS2 package metadata.

**Project:** https://github.com/robwoolley/mash

**Benefits:**

- Automated recipe generation from package.xml
- Handles dependencies automatically
- Reduces manual work
- Maintains consistency
- Speeds up porting

**Created by:** Rob Woolley (Open Source Robotics Foundation)

How MASH Works
--------------

1. **Reads package.xml** - ROS2 package metadata
2. **Extracts information** - Name, version, dependencies
3. **Generates .bb recipe** - Complete BitBake recipe
4. **Includes dependencies** - Build and runtime deps
5. **Adds licensing** - License information

Installing MASH
===============

Prerequisites
-------------

**System Requirements:**

- Python 3.8+
- Git
- ROS2 workspace (optional, for testing)

Installation Steps
------------------

.. code-block:: bash

   # Clone MASH repository
   git clone https://github.com/robwoolley/mash.git
   cd mash

   # Install Python dependencies
   pip3 install --user -r requirements.txt

   # Make executable
   chmod +x mash.py

   # Test installation
   ./mash.py --help

Using MASH
==========

Basic Usage
-----------

**Generate recipe for a ROS2 package:**

.. code-block:: bash

   ./mash.py \
       --ros-distro jazzy \
       --package-name your_ros2_package

**Options:**

- ``--ros-distro``: ROS2 distribution (jazzy, humble, etc.)
- ``--package-name``: Name of ROS2 package
- ``--output-dir``: Where to save recipe (default: output/)

Example: Generate Canadarm Demo Recipe
---------------------------------------

.. code-block:: bash

   # Navigate to MASH directory
   cd ~/mash

   # Generate recipe
   ./mash.py \
       --ros-distro jazzy \
       --package-name canadarm_demo \
       --output-dir /tmp/recipes

   # Output:
   # /tmp/recipes/canadarm_demo/canadarm-demo_1.0.0.bb

**Generated recipe example:**

.. code-block:: bitbake

   DESCRIPTION = "Canadarm demonstration package"
   LICENSE = "Apache-2.0"
   LIC_FILES_CHKSUM = "file://LICENSE;md5=86d3f3a95c324c9479bd8986968f4327"

   inherit ros_distro_jazzy

   DEPENDS = " \
       ament-cmake \
       rclcpp \
       std-msgs \
   "

   SRC_URI = "git://github.com/space-ros/canadarm-demo.git;protocol=https;branch=main"
   SRCREV = "0123456789abcdef0123456789abcdef01234567"

   S = "${WORKDIR}/git"

   ROS_BUILD_TYPE = "ament_cmake"
   ROS_BUILD_DEPENDS = ""
   ROS_EXEC_DEPENDS = ""

Batch Generation
----------------

Generate recipes for multiple packages:

.. code-block:: bash

   # Create package list
   cat > packages.txt <<EOF
   canadarm_demo
   canadarm_description
   canadarm_gazebo
   canadarm_moveit_config
   canadarm_wrench_publisher
   curiosity_description
   curiosity_gazebo
   curiosity_rover_demo
   ros_trick_bridge
   trick_canadarm_moveit_config
   trick_ros2_control
   EOF

   # Generate all recipes
   while read pkg; do
       ./mash.py --ros-distro jazzy --package-name "$pkg"
   done < packages.txt

Customizing MASH Output
------------------------

**Override defaults:**

.. code-block:: bash

   # Specify custom source repository
   ./mash.py \
       --ros-distro jazzy \
       --package-name my_package \
       --src-uri "git://github.com/myorg/mypackage.git"

   # Specify branch
   ./mash.py \
       --ros-distro jazzy \
       --package-name my_package \
       --branch develop

Recipe Structure
================

Recipe Anatomy
--------------

**1. Header Information**

.. code-block:: bitbake

   DESCRIPTION = "Brief description of package"
   SECTION = "devel"
   LICENSE = "Apache-2.0"
   LIC_FILES_CHKSUM = "file://LICENSE;md5=..."

**2. Source Location**

.. code-block:: bitbake

   SRC_URI = "git://github.com/org/repo.git;protocol=https;branch=main"
   SRCREV = "commit-hash"
   S = "${WORKDIR}/git"

**3. Dependencies**

.. code-block:: bitbake

   DEPENDS = " \
       ament-cmake \
       rclcpp \
       std-msgs \
   "

**4. Build Configuration**

.. code-block:: bitbake

   inherit ros_distro_jazzy
   ROS_BUILD_TYPE = "ament_cmake"

**5. Additional Instructions** (optional)

.. code-block:: bitbake

   do_install:append() {
       # Custom installation steps
   }

ROS2-Specific Variables
-----------------------

**ROS_DISTRO**
  ROS2 distribution name (jazzy, humble, etc.)

**ROS_BUILD_TYPE**
  Build system type (ament_cmake, ament_python, cmake, etc.)

**ROS_BUILD_DEPENDS**
  Build-time ROS dependencies

**ROS_EXEC_DEPENDS**
  Runtime ROS dependencies

**ROS_BUILDTOOL_DEPENDS**
  Build tool dependencies (usually ament-cmake)

License Checksums
-----------------

Calculate license checksum:

.. code-block:: bash

   md5sum LICENSE

Update recipe:

.. code-block:: bitbake

   LIC_FILES_CHKSUM = "file://LICENSE;md5=86d3f3a95c324c9479bd8986968f4327"

Integrating Recipes
===================

Adding to Layer
---------------

**Step 1: Create directory structure**

.. code-block:: bash

   mkdir -p layers/meta-spaceros-app/recipes-spaceros/canadarm_demo

**Step 2: Copy recipe**

.. code-block:: bash

   cp /tmp/recipes/canadarm_demo/canadarm-demo_1.0.0.bb \
      layers/meta-spaceros-app/recipes-spaceros/canadarm_demo/

**Step 3: Verify structure**

.. code-block:: text

   layers/meta-spaceros-app/
   └── recipes-spaceros/
       └── canadarm_demo/
           └── canadarm-demo_1.0.0.bb

Adding to Image
---------------

**Edit kas/beaglev-fire-space-ros.yml:**

.. code-block:: yaml

   local_conf_header:
     project: |
       IMAGE_INSTALL:append = " \
           canadarm-demo \
       "

**Note:** Package name uses hyphens, not underscores.

Testing Recipe
--------------

**Build individual recipe:**

.. code-block:: bash

   ./kas-container shell kas/beaglev-fire-space-ros.yml
   bitbake canadarm-demo

**Common build tasks:**

.. code-block:: bash

   # Fetch sources
   bitbake -c fetch canadarm-demo

   # Clean and rebuild
   bitbake -c cleansstate canadarm-demo
   bitbake canadarm-demo

   # Development shell
   bitbake -c devshell canadarm-demo

Recipe Development
==================

Manual Recipe Creation
----------------------

When MASH doesn't work or for custom packages:

**1. Create recipe file**

.. code-block:: bash

   mkdir -p layers/meta-spaceros-app/recipes-spaceros/my-package
   vim layers/meta-spaceros-app/recipes-spaceros/my-package/my-package_1.0.0.bb

**2. Write minimal recipe**

.. code-block:: bitbake

   DESCRIPTION = "My custom package"
   LICENSE = "MIT"
   LIC_FILES_CHKSUM = "file://LICENSE;md5=..."

   inherit ros_distro_jazzy

   SRC_URI = "git://github.com/myorg/my-package.git;protocol=https;branch=main"
   SRCREV = "${AUTOREV}"  # Use during development

   S = "${WORKDIR}/git"

   ROS_BUILD_TYPE = "ament_cmake"

**3. Test build**

.. code-block:: bash

   bitbake my-package

**4. Pin SRCREV for production**

Replace ``${AUTOREV}`` with actual commit hash.

BBAppend Files
--------------

Modify existing recipes without editing them:

**Create .bbappend:**

.. code-block:: bash

   mkdir -p layers/meta-spaceros-app/recipes-spaceros/existing-package
   vim layers/meta-spaceros-app/recipes-spaceros/existing-package/existing-package_%.bbappend

**Example modifications:**

.. code-block:: bitbake

   # Add patch
   FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
   SRC_URI += "file://my-fix.patch"

   # Add dependency
   DEPENDS += "additional-package"

   # Append to installation
   do_install:append() {
       install -d ${D}${sysconfdir}/my-package
       install -m 0644 ${WORKDIR}/config.yaml ${D}${sysconfdir}/my-package/
   }

Patches
-------

**Create patch file:**

.. code-block:: bash

   # Make changes in devshell
   bitbake -c devshell package-name

   # Generate patch
   git diff > my-changes.patch

   # Or use quilt
   quilt new my-changes.patch
   quilt add file-to-modify
   # Make changes
   quilt refresh

**Add to recipe:**

.. code-block:: bitbake

   FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
   SRC_URI += "file://my-changes.patch"

Dependency Management
=====================

Types of Dependencies
---------------------

**DEPENDS**
  Build-time dependencies (headers, libraries)

**RDEPENDS**
  Runtime dependencies (executed packages)

**RRECOMMENDS**
  Recommended runtime packages (optional)

**RSUGGESTS**
  Suggested packages (informational)

**RPROVIDES**
  Virtual package provision

**RCONFLICTS**
  Conflicting packages

Declaring Dependencies
----------------------

**In recipe:**

.. code-block:: bitbake

   DEPENDS = " \
       ament-cmake \
       rclcpp \
       sensor-msgs \
   "

   RDEPENDS:${PN} = " \
       python3-numpy \
       ros-jazzy-rclpy \
   "

**Virtual dependencies:**

.. code-block:: bitbake

   DEPENDS = "virtual/kernel"
   RDEPENDS:${PN} = "virtual/mesa"

Resolving Dependencies
----------------------

**Find providers:**

.. code-block:: bash

   ./kas-container shell kas/beaglev-fire-space-ros.yml
   
   # Search for package
   bitbake-layers show-recipes | grep package-name
   
   # Show recipe info
   bitbake -e package-name | grep ^DEPENDS=

**Common issues:**

- Missing layer: Add to kas configuration
- Wrong package name: Check with show-recipes
- Circular dependency: Restructure packages

Advanced Topics
===============

Python Packages
---------------

**For ament_python packages:**

.. code-block:: bitbake

   DESCRIPTION = "Python ROS2 package"
   LICENSE = "Apache-2.0"

   inherit ros_distro_jazzy

   ROS_BUILD_TYPE = "ament_python"

   SRC_URI = "git://github.com/org/package.git;protocol=https;branch=main"
   SRCREV = "..."

   S = "${WORKDIR}/git"

   RDEPENDS:${PN} = " \
       python3-numpy \
       ros-jazzy-rclpy \
   "

Conditional Compilation
-----------------------

**Based on MACHINE:**

.. code-block:: bitbake

   EXTRA_OECMAKE:append:beaglev-fire = " -DENABLE_FPGA=ON"

**Based on DISTRO_FEATURES:**

.. code-block:: bitbake

   PACKAGECONFIG ??= "${@bb.utils.contains('DISTRO_FEATURES', 'x11', 'gui', '', d)}"

**Based on dependencies:**

.. code-block:: bitbake

   PACKAGECONFIG[cuda] = "-DUSE_CUDA=ON,-DUSE_CUDA=OFF,cuda"

Multiple Packages from One Recipe
----------------------------------

**Split package:**

.. code-block:: bitbake

   PACKAGES = "${PN} ${PN}-dev ${PN}-dbg"

   FILES:${PN} = "${bindir}/* ${libdir}/lib*.so.*"
   FILES:${PN}-dev = "${includedir} ${libdir}/lib*.so"
   FILES:${PN}-dbg = "${bindir}/.debug ${libdir}/.debug"

Native and Cross Recipes
-------------------------

**Native package (runs on build host):**

.. code-block:: bitbake

   inherit native

**Cross package (for target arch):**

Default behavior for most recipes.

Package Groups
--------------

**Create package group:**

.. code-block:: bash

   mkdir -p layers/meta-spaceros-app/recipes-core/packagegroups
   vim layers/meta-spaceros-app/recipes-core/packagegroups/packagegroup-spaceros-demos.bb

**Content:**

.. code-block:: bitbake

   DESCRIPTION = "Space-ROS demonstration packages"
   LICENSE = "MIT"

   inherit packagegroup

   RDEPENDS:${PN} = " \
       canadarm-demo \
       curiosity-rover-demo \
       ros-trick-bridge \
   "

**Use in image:**

.. code-block:: yaml

   IMAGE_INSTALL:append = " packagegroup-spaceros-demos"

Best Practices
==============

Recipe Standards
----------------

**Naming:**

- Use lowercase with hyphens: ``my-package_1.0.0.bb``
- Version in filename: ``package_1.2.3.bb``
- Use AUTOREV only during development

**Organization:**

- One recipe per directory
- Related files in ``files/`` subdirectory
- Patches with descriptive names

**Documentation:**

.. code-block:: bitbake

   # Description at top
   DESCRIPTION = "Detailed package description"
   HOMEPAGE = "https://github.com/org/package"
   BUGTRACKER = "https://github.com/org/package/issues"

Version Management
------------------

**Stable releases:**

.. code-block:: bitbake

   SRC_URI = "git://github.com/org/package.git;protocol=https;branch=main"
   SRCREV = "abc123def456..."  # Pin to specific commit

**Development:**

.. code-block:: bitbake

   SRCREV = "${AUTOREV}"  # Always use latest
   PV = "1.0+git${SRCPV}"  # Version with git info

**Update SRCREV:**

.. code-block:: bash

   # Get latest commit
   git ls-remote https://github.com/org/package.git main

   # Update in recipe
   SRCREV = "new-commit-hash"

Testing
-------

**Before committing:**

1. Clean build from scratch
2. Test on target hardware
3. Check for warnings
4. Verify dependencies
5. Test package installation
6. Review generated files

**Automated testing:**

.. code-block:: bash

   # Build test
   bitbake -c cleanall my-package
   bitbake my-package

   # Install test
   bitbake core-image-minimal

Troubleshooting
===============

Common Issues
-------------

**"No provider for X"**

Missing dependency or layer. Check:

.. code-block:: bash

   bitbake-layers show-recipes | grep package-name

**"Checksum mismatch"**

License file changed. Update:

.. code-block:: bash

   md5sum path/to/LICENSE

**"Fetch failed"**

Wrong URL or network issue:

.. code-block:: bash

   # Test manually
   git clone <SRC_URI>

**"Compile failed"**

Check log file:

.. code-block:: bash

   find build/tmp/work -name "log.do_compile" | grep package-name
   less /path/to/log.do_compile

Debug Mode
----------

**Verbose output:**

.. code-block:: bash

   bitbake -v my-package

**Keep temp files:**

.. code-block:: bash

   bitbake -c compile -f my-package
   # Temp files remain in build/tmp/work/.../my-package/temp/

**Manual debugging:**

.. code-block:: bash

   bitbake -c devshell my-package
   # Manually run compile commands

Recipe Tools
============

Useful Commands
---------------

**Recipe information:**

.. code-block:: bash

   # Show all variables
   bitbake -e package-name

   # Show specific variable
   bitbake -e package-name | grep ^DEPENDS=

   # Show recipe location
   bitbake-layers show-recipes package-name

**Dependency analysis:**

.. code-block:: bash

   # Dependency graph
   bitbake -g package-name

   # Reverse dependencies
   bitbake -g -u depexp

**Layer management:**

.. code-block:: bash

   # List layers
   bitbake-layers show-layers

   # Add layer
   bitbake-layers add-layer path/to/layer

   # Remove layer
   bitbake-layers remove-layer layer-name

Recipe Utilities
----------------

**recipetool:**

.. code-block:: bash

   # Create recipe from source
   recipetool create -o my-package_1.0.0.bb https://github.com/org/package

   # Append to recipe
   recipetool appendfile my-package /path/to/file

**devtool:**

.. code-block:: bash

   # Create workspace
   devtool create-workspace workspace

   # Add package
   devtool add my-package https://github.com/org/package

   # Edit and test
   devtool edit-recipe my-package
   devtool build my-package

   # Generate patches
   devtool finish my-package layers/meta-spaceros-app

Resources
=========

Documentation
-------------

- **MASH:** https://github.com/robwoolley/mash
- **Yocto Manual:** https://docs.yoctoproject.org/
- **BitBake Manual:** https://docs.yoctoproject.org/bitbake/
- **meta-ros:** https://github.com/ros/meta-ros

Community
---------

- **Yocto Mailing List:** yocto@lists.yoctoproject.org
- **ROS Discourse:** https://discourse.ros.org/
- **meta-ros Issues:** https://github.com/ros/meta-ros/issues

Examples
--------

- meta-ros recipes: https://github.com/ros/meta-ros/tree/master/meta-ros2-jazzy/generated-recipes
- Space-ROS packages: https://github.com/space-ros
- This project: layers/meta-spaceros-app/recipes-spaceros/

---

Next: `Customization Guide <customization.rst>`_