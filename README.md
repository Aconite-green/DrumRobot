

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
- [Adventech embedded computer](https://www.advantech.com/en/products/f6935c48-a0b2-4c68-894a-40eb39c832b3/epc-c301/mod_0cdaa0da-61e8-4078-ac65-0d4cd00789f7) with CAN port
- Six [AK70-10 motors](https://store.tmotor.com/goods-1031-AK70-10.html) from Tmotor

### Installation

1. [Download](https://releases.ubuntu.com/focal/) and install Ubuntu 20.04 on the Adventech embedded computer. 
2. [Install](https://docs.ros.org/en/foxy/Tutorials/Miscellaneous/Building-Realtime-rt_preempt-kernel-for-ROS-2.html) a Realtime kernel that is compatible with Ubuntu 20.04.
3. Connect three AK70-10 motors from [Tmotor](https://store.tmotor.com/goods-1031-AK70-10.html) to each CAN port of the [Adventech](https://www.advantech.com/en/products/f6935c48-a0b2-4c68-894a-40eb39c832b3/epc-c301/mod_0cdaa0da-61e8-4078-ac65-0d4cd00789f7) embedded computer. The computer should be connected to a total of six motors. 
4. Download the provided source code onto the embedded computer.

## System Setup

### Setting up the Network Interface

In this project, you can set up the network interface by running the `CANControllerSetup.sh` script. This script automates the process of setting up the network interface for communication over the CAN interface.

```bash
./CANControllerSetup.sh
```

This script combines the functionality of the two scripts (`insmod_module_platform.sh` and `ipup_RDCCAN_06.sh`) discussed earlier, effectively making the setup process more streamlined.

The commands used in this script are explained below:

- **`modprobe`**: 

  The `modprobe` command is used in Linux to add a loadable kernel module (LKM) into the kernel or to remove an LKM from the kernel. This is used to load the necessary modules that the system needs to communicate over the CAN interface.

    ```
    modprobe can
    modprobe can-raw
    modprobe can-dev
    ```

- **`insmod`**: 

  The `insmod` command is used to insert a module into the Linux kernel. In our script, it installs the `can-ahc0512.ko` driver for the CAN controller.

    ```
    insmod can-ahc0512.ko
    ```

- **`ip link set`**: 

  The `ip link set` command is used to change the state of the device. In our case, it sets the type of `can0` to CAN and configures it with the specified bitrate, sample point, and restart time.

    ```
    ip link set can0 type can bitrate 1000000 sample-point 0.6 restart-ms 1
    ```

- **`ip -details -statistics link show`**: 

  The `ip` command is used to show detailed statistics for the `can0` interface. This helps in verifying if the interface has been set up correctly.

    ```
    ip -details -statistics link show can0
    ```

- **`ip link set up type can`**: 

  The `ip link set` command is also used to activate the `can0` device, making it ready to transmit and receive data.

    ```
    ip link set can0 up type can
    ```



## Installing and Configuring the Real-time Kernel

A real-time kernel is a kernel that guarantees to process certain events or data by a specific moment in time. It provides determinism and predictability, which is essential for real-time systems that need to perform tasks within a strict deadline, such as motor control in this project.

1. **Download the real-time kernel.** You can download it from [here](https://www.kernel.org/pub/linux/kernel/projects/rt/).

2. **Follow detailed instructions** on how to install the real-time kernel.


4. **Configure the kernel** to suit the project's needs. This can involve setting certain kernel parameters or enabling/disabling certain features.

5. **Verify that the kernel is configured correctly.** This can be done by checking the kernel version or the enabled features.


## Code Usage

### Importing the Required Libraries

#### 1. can_utils.h
This header file contains the necessary functions to interface with the CAN protocol for motor control, specifically with Tmotor's MIT mode. Functions are provided to pack and unpack commands, enter or exit control modes, stop the motor, and send frames with replies. Most of these functions generate canframe values that are supported only in Tmotor's MIT mode. It also includes relevant motor and error handling headers.

- **pack_cmd**: Packs the command for motor control. Supported only in Tmotor's MIT mode.
- **unpack_reply**: Unpacks the reply from the motor. Supported only in Tmotor's MIT mode.
- **enterControlmode**: Puts the motor into control mode. Supported only in Tmotor's MIT mode.
- **exitControlmode**: Exits the control mode of the motor. Supported only in Tmotor's MIT mode.
- **stop_motor**: Stops the motor. Supported only in Tmotor's MIT mode.
- **send_frame_and_receive_reply**: Sends a frame and receives a reply from the motor. 


#### 2. error_handling.h
This file includes functions that deal with error handling, providing specific functions to display an error message and exit the program, or to check for socket communication errors.

- **error_exit**: Displays an error message and exits the program.
- **check_socket_error**: Checks for an error in socket communication.

#### 3. motor.h
Defines structures and constants related to motors, including motor limits and commands. This file allows for the storage and manipulation of motor-related data.

- **MotorLimits**: Structure defining the limits for various motor parameters.
- **Motor**: Structure defining the motor's type and ID.
- **MotorCommand**: Structure to encapsulate motor commands.
- **MotorContainer**: Structure containing multiple motors.

#### 4. path_manager.h
Manages paths for motor commands, including loading commands from CSV files and packing them into command structures.

- **Buffer**: Structure to hold motor commands.
- **load_motor_commands_from_csv**: Loads motor commands from a CSV file.
- **pack_cmd_and_save_to_csv**: Packs commands and saves them to a CSV file.

#### 5. socket_config.h
Contains functions to configure and manipulate sockets for CAN communication, including creating, configuring, setting buffer size, loopback, timeouts, and other socket-related functionalities.

- **create_socket**: Creates a socket for communication.
- **configure_sockets**: Configures multiple sockets with various options.
- **set_blocking_mode**: Sets the blocking mode for a socket.
- **set_timeout**: Sets the timeout for a socket.
- **set_buffer_size**: Sets the buffer size for a socket.
- **set_loopback**: Sets the loopback mode for a socket.
- **set_recv_own_msgs**: Sets the socket to receive its own messages.


### Sending Motor Control Signals

In this project, three motors are connected to each CAN port of the Adventech computer. *This distribution is important for maintaining real-time control capabilities.* When operating at a baud rate of 1 Mbps with a real-time requirement of 5 ms or less, the recommended maximum number of motors per CAN port is three. **This ensures the system can meet the real-time requirement and each motor can receive its command and respond within the specified timeframe.**

- Description of how the code uses the clock_t object (CPU tick) to send motor control signals every 10ms.
- Examples of how to modify this part of the code to customize the behavior of the motors.

### Interfacing with Tmotor

#### Using MIT Mode

- Details about using MIT mode with Tmotor, including its benefits, configuration, and how it was specifically utilized in this project.

### Contacting Tmotor

- Steps on how to contact Tmotor for support or to provide them with feedback.
- Template or example messages to send to Tmotor.

### Creating a Repository for Tmotor Control

- Description of how to create a repository for Tmotor control.
- Steps to follow when uploading code or updates to the repository.


## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

## Acknowledgments

- Many thanks to Tmotor company for their help and support.