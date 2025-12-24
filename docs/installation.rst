==================
Installation Guide
==================

.. contents:: Table of Contents
   :depth: 2
   :local:

Prerequisites
=============

This guide covers setting up your development environment for building BeagleV-Fire Space-ROS images.

Host System Requirements
-------------------------

Operating System Support
^^^^^^^^^^^^^^^^^^^^^^^^

**Fully Tested:**

- Ubuntu 22.04 LTS (Jammy Jellyfish)
- Ubuntu 24.04 LTS (Noble Numbat)
- Fedora 40, 41

**Should Work:**

- Ubuntu 20.04 LTS
- Debian 12 (Bookworm)
- RHEL/CentOS/Rocky/Alma 8+
- openSUSE Leap 15.4+
- Arch Linux (rolling)

**Not Supported:**

- Windows (use WSL2 with Ubuntu)
- macOS (use Linux VM)

Hardware Requirements
^^^^^^^^^^^^^^^^^^^^^

**Minimum Configuration:**

==================  ============
Component           Requirement
==================  ============
Disk Space          100 GB free
RAM                 16 GB
CPU Cores           4
Network             Broadband
==================  ============

**Recommended Configuration:**

==================  ============
Component           Requirement
==================  ============
Disk Space          200 GB free
RAM                 32 GB
CPU Cores           8+
Storage Type        NVMe SSD
Network             Fiber/Cable
==================  ============

**Performance Impact:**

- More RAM: Enables parallel builds (faster)
- More CPU cores: Faster compilation
- SSD: Much faster than HDD
- Fast network: Reduces download time

Installing Docker
=================

Docker is required to run the KAS container build environment. This provides a consistent, isolated build environment regardless of your host system.

Why Docker?
-----------

- Consistent build environment
- No host system contamination
- Easy updates
- Works across distributions

Ubuntu Installation
-------------------

**Official Documentation:**

https://docs.docker.com/engine/install/ubuntu/

**Quick Install:**

.. code-block:: bash

   # Update package index
   sudo apt-get update

   # Install dependencies
   sudo apt-get install -y \
       ca-certificates \
       curl \
       gnupg \
       lsb-release

   # Add Docker's official GPG key
   sudo mkdir -p /etc/apt/keyrings
   curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
       sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

   # Set up repository
   echo \
     "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
     https://download.docker.com/linux/ubuntu \
     $(lsb_release -cs) stable" | \
     sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

   # Install Docker Engine
   sudo apt-get update
   sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

   # Add your user to docker group (avoid sudo)
   sudo usermod -aG docker $USER

   # IMPORTANT: Log out and back in for group changes

**Verify Installation:**

.. code-block:: bash

   # After logging back in
   docker --version
   docker run hello-world

Fedora Installation
-------------------

**Official Documentation:**

https://docs.docker.com/engine/install/fedora/

**Quick Install:**

.. code-block:: bash

   # Remove old versions
   sudo dnf remove docker \
       docker-client \
       docker-client-latest \
       docker-common \
       docker-latest \
       docker-latest-logrotate \
       docker-logrotate \
       docker-selinux \
       docker-engine-selinux \
       docker-engine

   # Install dnf-plugins-core
   sudo dnf -y install dnf-plugins-core

   # Add Docker repository
   sudo dnf config-manager \
       --add-repo \
       https://download.docker.com/linux/fedora/docker-ce.repo

   # Install Docker Engine
   sudo dnf install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

   # Start Docker
   sudo systemctl start docker
   sudo systemctl enable docker

   # Add user to docker group
   sudo usermod -aG docker $USER

   # Log out and back in

**Verify Installation:**

.. code-block:: bash

   docker --version
   docker run hello-world

Alternative: Podman
-------------------

Podman is a daemonless Docker alternative that works with KAS.

**Ubuntu:**

.. code-block:: bash

   sudo apt-get install -y podman

**Fedora:**

.. code-block:: bash

   sudo dnf install -y podman

**Usage:**

Podman commands are compatible with Docker. Simply alias:

.. code-block:: bash

   alias docker=podman

Installing Git
==============

Git is required to clone the repository.

**Ubuntu/Debian:**

.. code-block:: bash

   sudo apt-get install -y git

**Fedora:**

.. code-block:: bash

   sudo dnf install -y git

**Verify:**

.. code-block:: bash

   git --version

Disk Space Management
=====================

The Yocto build system requires substantial disk space. Here's what to expect:

Space Requirements
------------------

======================  ==============
Directory               Size
======================  ==============
downloads/              ~50 GB
sstate-cache/           ~30-40 GB
build/tmp/              ~60-80 GB
Total (after build)     ~150-170 GB
======================  ==============

**Growth over time:**

- Multiple builds: sstate-cache grows
- Different configurations: tmp/ grows
- Source updates: downloads/ grows

Choosing a Build Location
--------------------------

**Recommended:**

- Dedicated partition or disk
- Fast SSD for best performance
- Exclude from backups (optional)

**Check Available Space:**

.. code-block:: bash

   df -h /path/to/build/location

**Example Setup:**

.. code-block:: bash

   # Create dedicated build directory
   sudo mkdir /build
   sudo chown $USER:$USER /build
   cd /build
   git clone https://github.com/yassine-cherni/beaglev-fire-space-ros.git

Cleaning Up Space
-----------------

**Safe to delete:**

- ``build/tmp/`` - Rebuild will be slower
- ``build/`` - Complete rebuild required

**Keep for performance:**

- ``downloads/`` - Avoid re-downloading sources
- ``sstate-cache/`` - Speeds up rebuilds significantly

**Maintenance:**

.. code-block:: bash

   # Remove old build artifacts (automated in config)
   # rm_work is enabled in kas/common.yml

   # Manually clean specific package
   ./kas-container shell kas/beaglev-fire-space-ros.yml
   bitbake -c cleansstate package-name

Network Configuration
=====================

Proxy Settings
--------------

If behind a corporate proxy:

**Docker Proxy:**

Create ``/etc/systemd/system/docker.service.d/http-proxy.conf``:

.. code-block:: ini

   [Service]
   Environment="HTTP_PROXY=http://proxy.example.com:8080"
   Environment="HTTPS_PROXY=http://proxy.example.com:8080"
   Environment="NO_PROXY=localhost,127.0.0.1"

Reload and restart Docker:

.. code-block:: bash

   sudo systemctl daemon-reload
   sudo systemctl restart docker

**BitBake Proxy:**

Add to ``kas/common.yml``:

.. code-block:: yaml

   local_conf_header:
     proxy: |
       export HTTP_PROXY="http://proxy.example.com:8080"
       export HTTPS_PROXY="http://proxy.example.com:8080"
       export NO_PROXY="localhost,127.0.0.1"

Firewall Considerations
-----------------------

**Ports Used:**

- Outbound HTTPS (443): Source downloads
- Outbound HTTP (80): Some source repositories
- Outbound Git (9418): Git protocol (if used)

**Whitelist domains:**

- github.com
- githubusercontent.com
- yoctoproject.org
- kernel.org
- And various source repositories

Cloning the Repository
======================

Basic Clone
-----------

.. code-block:: bash

   git clone https://github.com/yassine-cherni/beaglev-fire-space-ros.git
   cd beaglev-fire-space-ros

Specific Branch
---------------

.. code-block:: bash

   git clone -b branch-name https://github.com/yassine-cherni/beaglev-fire-space-ros.git
   cd beaglev-fire-space-ros

Shallow Clone
-------------

For faster initial clone (not recommended for development):

.. code-block:: bash

   git clone --depth 1 https://github.com/yassine-cherni/beaglev-fire-space-ros.git
   cd beaglev-fire-space-ros

Running Setup Script
====================

The setup script initializes your build environment.

.. code-block:: bash

   # Make executable
   chmod +x setup.sh

   # Run setup
   ./setup.sh

What Setup Does
---------------

1. **Downloads KAS Container**
   
   - Version 5.1
   - Docker-based build orchestration
   - ~50 MB download

2. **Creates Directory Structure**
   
   - ``downloads/`` - Source code cache
   - ``sstate-cache/`` - Build artifacts cache
   - Persists across builds

3. **Validates Environment**
   
   - Checks Docker availability
   - Verifies permissions
   - Reports any issues

Expected Output
---------------

.. code-block:: text

   kas-container (version 5.1) downloaded and made executable.
   Persistent directories created: downloads and sstate-cache.
   Setup complete! Use ./kas-container shell kas/beaglev-fire-space-ros.yml to enter the build shell.

If you see this, setup was successful.

Troubleshooting Setup
---------------------

**"Docker not found"**

Install Docker and ensure it's running:

.. code-block:: bash

   sudo systemctl status docker
   docker ps

**"Permission denied"**

Add user to docker group and log out/in:

.. code-block:: bash

   sudo usermod -aG docker $USER
   # Log out and back in

**"Cannot download kas-container"**

Check network connectivity and proxy settings.

Verifying Installation
======================

Quick Test
----------

Enter the build environment without building:

.. code-block:: bash

   ./kas-container shell kas/beaglev-fire-space-ros.yml

You should see a bash prompt inside the container:

.. code-block:: text

   (kas) user@hostname:/work$

**Inside the container:**

.. code-block:: bash

   # Check BitBake
   bitbake --version

   # Check layers
   bitbake-layers show-layers

   # Exit
   exit

Configuration Validation
------------------------

Validate KAS configuration without building:

.. code-block:: bash

   ./kas-container shell kas/beaglev-fire-space-ros.yml --cmd "bitbake -e core-image-minimal | grep ^MACHINE="

Should output:

.. code-block:: text

   MACHINE="beaglev-fire"

System Tuning (Optional)
=========================

Parallel Builds
---------------

Configure based on your hardware:

**Edit ``kas/limit-pressure.yml``:**

.. code-block:: yaml

   local_conf_header:
     bb_limit_pressure: |
       BB_NUMBER_THREADS = "8"      # Number of parallel tasks
       PARALLEL_MAKE = "-j 8"       # Parallel compilation jobs

**Guidelines:**

- BB_NUMBER_THREADS: Number of CPU cores
- PARALLEL_MAKE: Number of CPU cores or cores + 2
- With 16GB RAM: Up to 8 threads
- With 32GB RAM: Up to 16 threads

Nice Level
----------

Prevent build from starving other processes:

.. code-block:: yaml

   BB_NICE_LEVEL = "11"  # Already set in limit-pressure.yml

Higher values (up to 19) make builds lower priority.

I/O Scheduling
--------------

For SSDs, consider:

.. code-block:: bash

   # Check current scheduler
   cat /sys/block/sda/queue/scheduler

   # For SSD, use none or mq-deadline
   echo none | sudo tee /sys/block/sda/queue/scheduler

Resource Limits
---------------

Current configuration (``kas/limit-pressure.yml``):

.. code-block:: yaml

   BB_PRESSURE_MAX_CPU = "1000000"      # CPU pressure limit
   BB_PRESSURE_MAX_IO = "50000"         # I/O pressure limit
   BB_PRESSURE_MAX_MEMORY = "10000"     # Memory pressure limit

These prevent build from overwhelming the system.

Next Steps
==========

Now that your environment is set up:

1. Proceed to `Building Images <building.rst>`_
2. Learn about `Architecture <architecture.rst>`_
3. Start with `Customization <customization.rst>`_

For issues during installation:

- Check `Troubleshooting Guide <troubleshooting.rst>`_
- Open an issue on GitHub
- Ask in project discussions

---

Next: `Building Images <building.rst>`_