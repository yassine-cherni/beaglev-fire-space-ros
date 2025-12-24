=======================
Hardware Specifications
=======================

.. contents:: Table of Contents
   :depth: 2
   :local:

BeagleV-Fire Overview
=====================

The BeagleV-Fire is an open-source single-board computer featuring a Microchip PolarFire FPGA SoC with RISC-V processor cores. It combines the flexibility of programmable logic with the power of a Linux-capable processor system.

Board Image and Layout
======================

Key Features
------------

- RISC-V processor subsystem with 5 cores
- FPGA fabric with 23K logic elements
- 2GB LPDDR4 memory
- Multiple connectivity options
- BeagleBone Cape compatibility
- Open-source hardware design

Core Specifications
===================

Processor Subsystem
-------------------

**SoC:** Microchip PolarFire MPFS025T FPGA SoC

**CPU Cores:**

- 1x E51 Monitor Core (RV64IMAC)
  
  - 625 MHz
  - 16KB I-Cache, 8KB D-Cache
  - Machine mode only
  - Boot and debug functions

- 4x U54 Application Cores (RV64GC)
  
  - 625 MHz each
  - 16KB I-Cache, 16KB D-Cache per core
  - Supervisor and user modes
  - Linux SMP capable
  - Hardware floating point (FD extensions)

**ISA Details:**

- RV64IMAFDC (RV64GC standard)
- I: Integer base
- M: Integer multiplication/division
- A: Atomic operations
- F: Single-precision floating point
- D: Double-precision floating point
- C: Compressed instructions

Memory System
-------------

**LPDDR4 RAM:**

- 2GB capacity
- LPDDR4-2400 (2400 MT/s)
- 32-bit data bus
- ECC support in FPGA controller

**Cache Hierarchy:**

- L1 I-Cache: 16KB per core
- L1 D-Cache: 8KB (E51) / 16KB (U54)
- L2 Cache: Shared, configurable in FPGA

**On-Chip Memory:**

- 128KB L2 LIM (Loosely Integrated Memory)
- Usable for critical code/data
- Low-latency access

Storage Options
---------------

**eMMC:**

- 16GB capacity
- eMMC 5.1 interface
- Default boot source
- Wear leveling support

**SPI Flash:**

- 128MB capacity
- Quad SPI interface
- FPGA bitstream storage
- Bootloader storage

**microSD:**

- microSD/SDHC/SDXC support
- UHS-I capable
- Alternative boot source
- User data storage

FPGA Fabric
===========

Logic Resources
---------------

**Logic Elements:** 23,000

- Combinational and sequential logic
- Distributed RAM and shift registers
- Fast carry chains

**Flip-Flops:** 220,000

- Edge-triggered storage elements
- Set/reset capability
- Clock enable

**LUTs (Look-Up Tables):** 23,000

- 4-input configuration
- Can function as distributed RAM
- 16 bits per LUT

**Math Blocks:** 68

- 18x18 multipliers
- Pipelined operation
- DSP-optimized

DSP Capabilities
----------------

**DSP Blocks:** 68 x 18x18 multipliers

- Dedicated multiply-accumulate (MAC)
- Saturating arithmetic
- Cascadable for larger operations
- Optimized for signal processing

**Applications:**

- Digital filters (FIR, IIR)
- FFT/IFFT operations
- Matrix multiplication
- AI inference acceleration

Memory in FPGA
--------------

**Block RAM:** Variable (design-dependent)

- True dual-port operation
- Configurable width and depth
- ECC support
- Low-latency access

**Typical Configurations:**

- 512 x 36 bits
- 1024 x 18 bits
- 2048 x 9 bits
- Custom sizes supported

High-Speed Interfaces
---------------------

**SERDES Channels:** 4

- 12.7 Gbps per lane
- PCIe Gen2 capable
- Custom protocols supported
- Low-jitter transceivers

**Use Cases:**

- High-speed communication
- Camera interfaces
- Data acquisition
- Custom interconnects

Connectivity
============

Networking
----------

**Gigabit Ethernet:**

- 10/100/1000 Mbps
- RJ45 connector
- Integrated MAC in FPGA
- PHY: TI DP83867IS
- IEEE 1588 PTP support

USB
---

**USB 3.0 Type-C:**

- SuperSpeed (5 Gbps)
- USB OTG capable
- Device and host modes
- Power delivery support
- Used for programming and data

Expansion Interfaces
--------------------

**M.2 E-Key Socket:**

- PCIe x1 interface
- USB 2.0 lanes
- Standard M.2 2230 cards
- WiFi/Bluetooth modules
- Custom PCIe devices

**SYZYGY Connector:**

- High-speed I/O standard
- FPGA GPIO connections
- Power management
- I2C, SPI, differential pairs
- Custom daughtercards

**BeagleBone Cape Headers:**

- 2x 46-pin headers
- 3.3V and 5V I/O
- GPIO, I2C, SPI, UART, PWM
- ADC channels
- Compatible with BeagleBone capes

Camera Interface
----------------

**MIPI CSI-2:**

- 22-pin connector
- 4-lane support
- Up to 2.5 Gbps per lane
- Raspberry Pi camera compatibility
- FPGA processing of image data

Debug and Programming
---------------------

**JTAG:**

- 10-pin Cortex Debug connector
- Microchip FlashPro5/6 compatible
- FPGA programming
- Processor debugging

**UART Console:**

- 3.3V TTL levels
- 115200 baud default
- Serial console access
- Boot messages

Power System
============

Power Input
-----------

**Primary:** USB Type-C

- 5V nominal
- 3A minimum recommended
- USB Power Delivery support

**Backup:** 5V barrel jack (optional)

- 2.1mm center positive
- 5V, 3A

Power Consumption
-----------------

**Typical:**

- Idle: 2-3W
- Active (CPU): 4-6W
- FPGA active: +1-3W (design-dependent)
- Peak: ~10W

**Power Domains:**

- Core voltage: Generated on-board
- I/O voltage: 3.3V, 1.8V, 1.2V
- FPGA fabric: Configurable

Environmental
-------------

**Operating Temperature:**

- Commercial: 0°C to 70°C
- Extended: -40°C to 85°C (with appropriate components)

**Storage Temperature:** -40°C to 85°C

Physical Dimensions
===================

**Board Size:**

- Length: 86.36 mm (3.4 inches)
- Width: 54.61 mm (2.15 inches)
- BeagleBone-compatible footprint

**Weight:** Approximately 60 grams

**Mounting:** 4x M3 mounting holes at corners

Performance Characteristics
===========================

Processing Power
----------------

**CPU Performance:**

- ~2500 DMIPS total (4x U54 cores)
- ~625 DMIPS per core at 625 MHz
- Hardware floating point
- Suitable for ROS2 middleware

**FPGA Performance:**

- Design-dependent
- 100+ MHz operation typical
- Parallel processing advantage
- Custom acceleration possible

Memory Bandwidth
----------------

**LPDDR4:**

- Theoretical: ~9.6 GB/s
- Practical: ~6-7 GB/s
- Shared between CPU and FPGA

**On-Chip:**

- L2 LIM: Very low latency
- Block RAM: Single-cycle access

I/O Bandwidth
-------------

**Ethernet:** 1 Gbps (125 MB/s)
**USB 3.0:** 5 Gbps (625 MB/s theoretical)
**SERDES:** 4x 12.7 Gbps (51 Gbps total)
**SD Card:** ~25 MB/s (UHS-I)

Why BeagleV-Fire for Space-ROS?
================================

RISC-V Advantages
-----------------

**Open ISA:**

- No licensing fees
- Full specification available
- Custom extensions possible
- Growing ecosystem

**Simplicity:**

- Clean, modular design
- Easy to verify
- Security features
- Fewer transistors than x86/ARM

FPGA Benefits
-------------

**Hardware Acceleration:**

- Offload compute-intensive tasks
- Real-time processing
- Parallel execution
- Low latency

**Flexibility:**

- Reconfigurable logic
- Mission-specific customization
- Post-deployment updates
- Fault mitigation

Space Applications
------------------

**Radiation Tolerance:**

- PolarFire RT variants available
- FPGA self-repair capabilities
- Redundancy implementation
- Error detection/correction

**Power Efficiency:**

- Low power consumption
- Fine-grained power management
- Sleep modes
- Dynamic reconfiguration

**Determinism:**

- Real-time guarantees
- Predictable behavior
- Hardware-timed operations
- No OS jitter in FPGA

Development Ecosystem
---------------------

**Software:**

- Full Linux support
- ROS2 compatibility
- Standard toolchains
- Active community

**Hardware:**

- Open schematics
- Cape ecosystem
- Educational resources
- Low cost ($150 USD)

Comparison with Alternatives
=============================

vs. Raspberry Pi
----------------

=============================  ==================  ===================
Feature                        BeagleV-Fire        Raspberry Pi 4B
=============================  ==================  ===================
Architecture                   RISC-V + FPGA       ARM Cortex-A72
CPU Cores                      5                   4
CPU Speed                      625 MHz             1.5 GHz
FPGA                           23K LEs             None
Memory                         2GB LPDDR4          1-8GB LPDDR4
Ethernet                       1 Gbps              1 Gbps
Hardware Acceleration          FPGA fabric         VideoCore VI GPU
Open ISA                       Yes                 No
Programmable Logic             Yes                 No
=============================  ==================  ===================

vs. Xilinx Zynq
---------------

=============================  ==================  ===================
Feature                        BeagleV-Fire        Zynq-7020
=============================  ==================  ===================
Architecture                   RISC-V + FPGA       ARM + FPGA
Openness                       Fully open          Proprietary CPU
CPU Cores                      5 RISC-V            2 ARM A9
CPU Speed                      625 MHz             866 MHz
FPGA Size                      23K LEs             85K LEs
Cost                           ~$150               ~$200-500
Linux Support                  Mainline            Vendor BSP
Community                      Growing             Mature
=============================  ==================  ===================

Expansion Possibilities
=======================

Cape Compatibility
------------------

BeagleBone capes that work:

- Sensor capes (environmental, IMU)
- Motor control capes
- Communication capes (CAN, RS485)
- Display capes
- Audio capes

Custom Hardware
---------------

**Design Your Own:**

- SYZYGY standard for custom I/O
- Direct FPGA pin access
- High-speed differential pairs
- Power management support

**Examples:**

- Camera modules
- LIDAR interfaces
- SDR (Software Defined Radio)
- Motor controllers

Limitations and Considerations
===============================

Current Limitations
-------------------

**Processing Power:**

- Lower CPU frequency than modern ARM
- Suitable for control, not heavy computation
- Use FPGA for intensive tasks

**Memory:**

- 2GB may limit large applications
- No memory expansion slot
- Careful memory management needed

**FPGA Size:**

- 23K LEs moderate size
- Complex designs may need partitioning
- Consider design tradeoffs

**Software Maturity:**

- RISC-V ecosystem still growing
- Some packages may need porting
- Community support improving

Design Considerations
---------------------

**Power Budget:**

- Plan for peak loads
- Consider FPGA power usage
- Use power measurement tools

**Thermal Management:**

- Heatsink recommended for sustained load
- Monitor junction temperature
- Airflow consideration

**Boot Time:**

- FPGA configuration adds delay
- Optimize bitstream size
- Consider partial reconfiguration

Future Hardware Support
=======================

Planned Support
---------------

- Additional RISC-V SBCs
- BeagleV-Ahead (when available)
- SiFive HiFive boards
- QEMU RISC-V emulation

Community Contributions
-----------------------

Hardware ports welcome for:

- Other PolarFire boards
- RISC-V development boards
- Custom carrier boards
- Industrial systems

Resources
=========

Official Documentation
----------------------

- BeagleV-Fire Wiki: https://docs.beagleboard.org/latest/boards/beaglev-fire/
- PolarFire SoC Documentation: https://www.microchip.com/polarfire-soc
- RISC-V Specifications: https://riscv.org/technical/specifications/

Schematics and Design Files
----------------------------

- BeagleV-Fire GitHub: https://github.com/beagleboard/beaglev-fire
- Hardware design files: Open source on GitHub
- BOM and assembly: Available for manufacturing

Community
---------

- BeagleBoard Forum: https://forum.beagleboard.org/
- RISC-V Forums: https://groups.google.com/a/groups.riscv.org/
- Microchip Support: https://www.microchip.com/support

Purchase
--------

- BeagleBoard.org: Official store
- Distributors: DigiKey, Mouser, others
- Price: ~$150 USD

---

Next: `Installation Guide <installation.rst>`_