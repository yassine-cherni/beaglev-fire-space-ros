==============
Building Images
==============

.. contents:: Table of Contents
   :depth: 2
   :local:

Build Process Overview
======================

The build process transforms source code and configuration into a bootable Linux image with ROS2 and Space-ROS support.

Build Workflow
--------------

1. **Parse Configuration** - KAS reads YAML files
2. **Clone Layers** - Git repositories are fetched
3. **Parse Recipes** - BitBake reads all .bb files
4. **Resolve Dependencies** - Build order determined
5. **Fetch Sources** - Download source tarballs
6. **Extract & Patch** - Prepare source code
7. **Configure** - Run configuration scripts
8. **Compile** - Build all packages
9. **Install** - Stage binaries
10. **Package** - Create .ipk packages
11. **Generate Image** - Create root filesystem
12. **Create Bootable Image** - Generate .wic file

Time Expectations
-----------------

**First Build:**

- 2-4 hours on recommended hardware
- 4-8 hours on minimum hardware
- Downloads ~50GB of sources
- Creates ~150GB of build artifacts

**Incremental Builds:**

- 20-60 minutes for small changes
- Reuses cached compilation (sstate-cache)
- Only rebuilds changed packages

**Clean Rebuild:**

- 30-90 minutes with warm caches
- Much faster than first build
- Uses downloads/ and sstate-cache/

Starting a Build
================

Basic Build Command
-------------------

.. code-block:: bash

   ./kas-container build kas/beaglev-fire-space-ros.yml

This single command:

- Starts Docker container
- Sets up build environment
- Fetches all layers
- Builds the image
- Produces output in ``build/tmp/deploy/images/beaglev-fire/``

What Gets Built
---------------

**core-image-minimal** includes:

- Linux kernel 6.12.x
- Systemd init system
- Basic utilities (bash, coreutils)
- Network support (Ethernet, WiFi)
- ROS2 Jazzy complete workspace
- Space-ROS demonstration packages
- Development tools (vim, git, htop, tmux, python3)
- Canadarm2 simulation and control
- Mars Curiosity Rover navigation
- Trick simulation integration

Build Output
------------

Successful build creates:

.. code-block:: text

   build/tmp/deploy/images/beaglev-fire/
   ├── core-image-minimal-beaglev-fire.wic.gz
   ├── core-image-minimal-beaglev-fire.manifest
   ├── core-image-minimal-beaglev-fire.testdata.json
   ├── Image-beaglev-fire.bin
   ├── modules-beaglev-fire.tgz
   └── <additional files>

**Primary artifact:** ``core-image-minimal-beaglev-fire.wic.gz``

This is the complete bootable disk image.

Understanding Build Output
===========================

Console Messages
----------------

**Task Execution:**

.. code-block:: text

   NOTE: Executing Tasks
   NOTE: Running task 1234 of 5678
   NOTE: recipe package-name-version-r0: task do_fetch: Started
   NOTE: recipe package-name-version-r0: task do_fetch: Succeeded

**Progress Indicator:**

.. code-block:: text

   Currently 8 running tasks (1234 of 5678):
     0: package1-version-r0 do_compile (pid 12345)
     1: package2-version-r0 do_configure (pid 12346)
     ...

**Warnings (Safe to Ignore Usually):**

.. code-block:: text

   WARNING: package-name-version-r0 do_package_qa: QA Issue: ...

**Errors (Build Stops):**

.. code-block:: text

   ERROR: package-name-version-r0 do_compile: ...
   ERROR: Task failed: ...

Success Indicators
------------------

Build completes successfully when you see:

.. code-block:: text

   NOTE: Tasks Summary: Attempted 5678 tasks of which 0 didn't need to be rerun
   NOTE: Build completed successfully

Output location is reported:

.. code-block:: text

   NOTE: Writing buildhistory for image core-image-minimal
   NOTE: Writing image manifest: .../core-image-minimal-beaglev-fire.manifest

Monitoring Build Progress
==========================

Resource Usage
--------------

**Monitor in real-time:**

.. code-block:: bash

   # CPU and memory
   htop

   # Disk I/O
   iotop

   # Disk space
   watch -n 60 df -h

**Expected during build:**

- CPU: 100% on all cores
- Memory: 8-16GB used
- Disk I/O: High, especially during compilation
- Network: Spikes during source downloads

Estimated Completion
--------------------

Calculate based on progress:

.. code-block:: text

   Currently running: 1234 of 5678 tasks

**Rough estimate:**

- Percentage complete: (1234 / 5678) × 100 = 21.7%
- Time elapsed: 30 minutes
- Estimated total: 30 / 0.217 ≈ 138 minutes (2.3 hours)

Note: Later tasks often take longer (compilation).

Log Files
---------

Build logs are in:

.. code-block:: text

   build/tmp/work/<arch>/<package>/<version>/temp/log.*

**Useful logs:**

- ``log.do_compile`` - Compilation output
- ``log.do_configure`` - Configuration output
- ``log.task_order`` - Task execution order

**View live log:**

.. code-block:: bash

   ./kas-container shell kas/beaglev-fire-space-ros.yml
   tail -f build/tmp/work/riscv64-oe-linux/package-name/version/temp/log.do_compile

Build Variants
==============

Different Build Targets
-----------------------

**Minimal Image (Default):**

.. code-block:: bash

   ./kas-container build kas/beaglev-fire-space-ros.yml

**SDK for Cross-Compilation:**

.. code-block:: bash

   ./kas-container build kas/beaglev-fire-space-ros.yml:target=populate_sdk

**Package Feed:**

.. code-block:: bash

   ./kas-container build kas/beaglev-fire-space-ros.yml:target=package_index

Build Configurations
--------------------

**Debug Build:**

Add to ``kas/common.yml``:

.. code-block:: yaml

   local_conf_header:
     debug: |
       EXTRA_IMAGE_FEATURES += "debug-tweaks dbg-pkgs tools-debug"

**Development Build:**

.. code-block:: yaml

   local_conf_header:
     dev: |
       EXTRA_IMAGE_FEATURES += "dev-pkgs tools-sdk"

**Production Build:**

Remove debug features and minimize size.

Interactive Development
=======================

Using devshell
--------------

Open development shell for a package:

.. code-block:: bash

   ./kas-container shell kas/beaglev-fire-space-ros.yml
   bitbake -c devshell package-name

This opens a shell with:

- Package source code
- Build environment configured
- Toolchain ready
- Manual build/test possible

Building Individual Packages
-----------------------------

**Build specific package:**

.. code-block:: bash

   ./kas-container shell kas/beaglev-fire-space-ros.yml
   bitbake package-name

**Clean and rebuild:**

.. code-block:: bash

   bitbake -c cleansstate package-name
   bitbake package-name

**Common tasks:**

.. code-block:: bash

   bitbake -c fetch package-name      # Download sources
   bitbake -c unpack package-name     # Extract sources
   bitbake -c configure package-name  # Run configure
   bitbake -c compile package-name    # Compile
   bitbake -c install package-name    # Install to staging

Incremental Development
-----------------------

**Modify source code:**

.. code-block:: bash

   # Enter devshell
   bitbake -c devshell package-name

   # Edit files
   vim src/main.cpp

   # Compile manually
   make

   # Test changes

   # Exit devshell
   exit

   # Rebuild package properly
   bitbake -c compile -f package-name
   bitbake package-name

Cleaning Builds
===============

Clean Levels
------------

**Clean a single package:**

.. code-block:: bash

   bitbake -c clean package-name

Removes package work directory.

**Clean package state:**

.. code-block:: bash

   bitbake -c cleansstate package-name

Removes package from sstate-cache.

**Clean everything:**

.. code-block:: bash

   bitbake -c cleanall package-name

Removes package from downloads too.

**Clean entire build:**

.. code-block:: bash

   rm -rf build/

Preserves downloads/ and sstate-cache/.

Selective Cleaning
------------------

**When to use each:**

==================  ========================================
Command             When to Use
==================  ========================================
clean               Testing small code changes
cleansstate         Recipe changes, forcing rebuild
cleanall            Source download issues
rm -rf build/       Starting completely fresh
==================  ========================================

Handling Build Failures
========================

Common Failure Scenarios
------------------------

**Network Failure:**

.. code-block:: text

   ERROR: Fetcher failure for URL: 'https://...'

**Solution:** Retry, check network/proxy.

**Disk Space:**

.. code-block:: text

   ERROR: No space left on device

**Solution:** Free space, see `Installation Guide <installation.rst>`_.

**Dependency Missing:**

.. code-block:: text

   ERROR: Nothing PROVIDES 'package-name'

**Solution:** Check layer inclusion, recipe dependencies.

**Compilation Error:**

.. code-block:: text

   ERROR: package-name do_compile failed

**Solution:** Check log file, search for similar issues.

Debugging Build Failures
-------------------------

**Step 1: Identify failing task**

.. code-block:: text

   ERROR: package-name-version-r0 do_compile: ...

**Step 2: Find log file**

.. code-block:: bash

   find build/tmp/work -name "log.do_compile" | grep package-name

**Step 3: Read log**

.. code-block:: bash

   less /path/to/log.do_compile

**Step 4: Search for error**

Look for lines with "error:", "failed", "undefined reference".

**Step 5: Research**

- Search Yocto mailing list
- Check package's GitHub issues
- Search for error message

**Step 6: Apply fix**

- Patch recipe
- Update configuration
- Report upstream bug

Resuming Failed Builds
-----------------------

After fixing the issue:

.. code-block:: bash

   # If error was in specific package
   bitbake -c cleansstate failed-package
   bitbake failed-package

   # Resume full build
   ./kas-container build kas/beaglev-fire-space-ros.yml

BitBake will resume from where it failed.

Advanced Build Topics
=====================

Dependency Graphing
-------------------

Generate dependency graph:

.. code-block:: bash

   ./kas-container shell kas/beaglev-fire-space-ros.yml
   bitbake -g core-image-minimal
   
   # Creates:
   # pn-buildlist      - Build order
   # task-depends.dot  - Task dependencies
   # pn-depends.dot    - Package dependencies

View with graphviz:

.. code-block:: bash

   dot -Tpng pn-depends.dot -o depends.png

Shared State Cache
------------------

**Location:** ``sstate-cache/``

**Purpose:** Cache compiled artifacts for reuse.

**Benefits:**

- Drastically speeds up rebuilds
- Share between machines (network mount)
- Survive full build/ deletion

**Size management:**

.. code-block:: bash

   # Check size
   du -sh sstate-cache/

   # Clean old entries (automatic with rm_work)
   # Or manually:
   find sstate-cache/ -atime +30 -delete

Disk Monitoring
---------------

Configuration in ``kas/diskmon.yml`` prevents disk full:

.. code-block:: yaml

   BB_DISKMON_DIRS ?= "\
       STOPTASKS,${TMPDIR},1G,100K \
       HALT,${TMPDIR},100M,1K"

**Behavior:**

- STOPTASKS: Pause new tasks if < 1GB free
- HALT: Stop build if < 100MB free

Parallel Builds
---------------

Current configuration in ``kas/limit-pressure.yml``:

.. code-block:: yaml

   BB_NUMBER_THREADS = "auto"  # Parallel tasks
   PARALLEL_MAKE = "-j auto"   # Parallel compilation

**Tuning:**

Adjust based on your hardware. See `Installation Guide <installation.rst>`_.

Build Performance Tips
======================

Speeding Up Builds
------------------

**Use SSD:**

- Move build/, downloads/, sstate-cache/ to SSD
- 2-3x faster than HDD

**More RAM:**

- Enables more parallel tasks
- Reduces disk I/O (tmpfs possible)

**More CPU cores:**

- Linear speedup with more cores
- Diminishing returns after ~16 cores

**Fast network:**

- Fiber/Cable preferred
- Reduces download time significantly

**Enable rm_work:**

Already enabled in kas/common.yml:

.. code-block:: yaml

   INHERIT += "rm_work"

Deletes work directories after build.

ccache Integration
------------------

Speed up recompilations:

Add to ``kas/common.yml``:

.. code-block:: yaml

   local_conf_header:
     ccache: |
       INHERIT += "ccache"

tmpfs Build Directory
---------------------

For systems with lots of RAM (64GB+):

.. code-block:: bash

   # Create tmpfs
   sudo mkdir /mnt/yocto-tmp
   sudo mount -t tmpfs -o size=50G tmpfs /mnt/yocto-tmp
   sudo chown $USER:$USER /mnt/yocto-tmp

   # Symlink tmp directory
   ln -s /mnt/yocto-tmp build/tmp

Warning: Data lost on reboot.

Continuous Integration
======================

Automated Builds
----------------

Example GitHub Actions workflow:

.. code-block:: yaml

   name: Build BeagleV-Fire Image
   on: [push, pull_request]
   
   jobs:
     build:
       runs-on: ubuntu-22.04
       steps:
         - uses: actions/checkout@v3
         - name: Setup
           run: ./setup.sh
         - name: Build
           run: ./kas-container build kas/beaglev-fire-space-ros.yml
         - name: Upload artifacts
           uses: actions/upload-artifact@v3
           with:
             name: images
             path: build/tmp/deploy/images/

Caching Strategy
----------------

Speed up CI with caches:

.. code-block:: yaml

   - uses: actions/cache@v3
     with:
       path: |
         downloads
         sstate-cache
       key: yocto-cache-${{ hashFiles('kas/**/*.yml') }}

Build Verification
------------------

After build completes:

.. code-block:: bash

   # Check image was created
   test -f build/tmp/deploy/images/beaglev-fire/core-image-minimal-beaglev-fire.wic.gz

   # Check manifest
   test -f build/tmp/deploy/images/beaglev-fire/core-image-minimal-beaglev-fire.manifest

   # Verify ROS2 packages
   grep "ros-" build/tmp/deploy/images/beaglev-fire/core-image-minimal-beaglev-fire.manifest

Next Steps
==========

After successful build:

1. Flash image to hardware - See hardware documentation
2. Boot and test - Verify ROS2 functionality
3. Customize build - `Customization Guide <customization.rst>`_
4. Develop applications - `Architecture Guide <architecture.rst>`_

For build issues:

- Check `Troubleshooting Guide <troubleshooting.rst>`_
- Review BitBake logs
- Ask in project discussions

---

Next: `Architecture Guide <architecture.rst>`_