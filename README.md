

# DrumRobot Project

This project provides a guide on how to control six AK70-10 motors (Tmotor Company) in real-time using an embedded computer (Adventech). 

## Table of Contents

- [Realtime Motor Control Project](#realtime-motor-control-project)
  - [Table of Contents](#table-of-contents)
  - [Getting Started](#getting-started)
    - [Prerequisites](#prerequisites)
    - [Installation](#installation)
  - [System Setup](#system-setup)
    - [Setting up the Network Interface](#setting-up-the-network-interface)
  - [Installing and Configuring the Real-time Kernel](#installing-and-configuring-the-real-time-kernel)
    - [Applying the Real-time Kernel in Code](#applying-the-real-time-kernel-in-code)
  - [Code Usage](#code-usage)
    - [Importing the Required Libraries](#importing-the-required-libraries)
    - [Sending Motor Control Signals](#sending-motor-control-signals)
  - [Interfacing with Tmotor](#interfacing-with-tmotor)
    - [Contacting Tmotor](#contacting-tmotor)
    - [Creating a Repository for Tmotor Control](#creating-a-repository-for-tmotor-control)
  - [License](#license)
  - [Acknowledgments](#acknowledgments)

## Getting Started

### Prerequisites

- [Ubuntu 20.04](https://releases.ubuntu.com/focal/)
- [Realtime kernel](https://docs.ros.org/en/foxy/Tutorials/Miscellaneous/Building-Realtime-rt_preempt-kernel-for-ROS-2.html) compatible with Ubuntu 20.04
- [Embedded computer](https://www.advantech.com/en/products/f6935c48-a0b2-4c68-894a-40eb39c832b3/epc-c301/mod_0cdaa0da-61e8-4078-ac65-0d4cd00789f7) with CAN port
- Six [AK70-10 ](https://store.tmotor.com/goods-1031-AK70-10.html) motors and One [AK10-9 motor](https://store.tmotor.com/product/ak10-9-v2-kv60-dynamical-modular.html) from Tmotor
- Two Maxon [EPOS4 CAN Controler]() and Two Maxon [Motors]()
- One [USBIO](https://www.icpdas.com/en/product/USB-2051-32)
- Four [SMPS](#)
- Six [Proxymity SenSor](#)


## Installing and Configuring the Real-time Kernel

A real-time kernel is a kernel that guarantees to process certain events or data by a specific moment in time. It provides determinism and predictability, which is essential for real-time systems that need to perform tasks within a strict deadline, such as motor control in this project.

1. **Download the real-time kernel.** You can download it from [here](https://www.kernel.org/pub/linux/kernel/projects/rt/).

2. **Follow detailed instructions** on how to install the real-time kernel.


4. **Configure the kernel** to suit the project's needs. This can involve setting certain kernel parameters or enabling/disabling certain features.

5. **Verify that the kernel is configured correctly.** This can be done by checking the kernel version or the enabled features.


## Code Usage

### Importing the Required Libraries




## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

## Acknowledgments

- Many thanks to Tmotor company for their help and support.
