# RTControlDualArm

`RTControlDualArm` is a high-performance real-time control system for dual-arm manipulators. It is built on Linux with Xenomai RTOS and uses the EtherCAT protocol for low-latency hardware communication.

## Key Features
- **Real-Time Control**: Low-latency control loops implemented with Xenomai 3.
- **EtherCAT Integration**: Robust hardware communication using Etherlab Master.
- **Advanced Control Algorithms**:
  - Joint-space Control (PID, Inverse Dynamics)
  - Task-space Control (CLIK, Operational Space Control)
  - Impedance Control (Joint and Task space)
  - Friction Identification and Compensation
- **Multi-Tasking Architecture**:
  - `Control_proc`: Main real-time control loop (up to 2kHz).
  - `TCPIP_proc`: Network communication for monitoring and remote control.
  - `Console_proc`: Real-time status logging and display.
  - `Event_proc`: Keyboard and event handling.
- **Kinematics & Dynamics**: Integrated with KDL (Kinematics and Dynamics Library) and custom utilities for complex manipulator configurations.

## System Architecture
The system is designed as a modular real-time client (`RTClient`). It manages multiple real-time tasks synchronized to an EtherCAT master. The control logic is decoupled from hardware communication, allowing for flexible algorithm development.

## Implementation Details
- **OS**: Linux with Xenomai 3.0.9 (Kernel 4.9.90)
- **Communication**: Etherlab 1.5.2
- **Build System**: CMake 3.9+
- **Mathematics**: Eigen 3, KDL

## Prerequisites
Install the following dependencies:
```bash
sudo apt install libpoco-dev libeigen3-dev libboost-dev
```
Ensure Xenomai 3 and Etherlab are correctly installed and configured in your system path.

## Project Structure
- `include/`: Standardized header directory.
  - `control/`: Motion control and trajectory headers.
  - `ecat/`: EtherCAT master, slave, and PDO configuration headers.
  - `kdl/`: Kinematics and Dynamics Library integration headers.
  - `net/`: Network communication and packet handler headers.
  - `RTClient.h`: Main application header.
- `src/`: Standardized source directory.
  - `control/`, `ecat/`, `kdl/`, `net/`: Implementation files and module-specific `CMakeLists.txt`.
  - `main.cpp`: Primary application entrance.
- `cmake/`: Custom CMake find modules and helper scripts.

## Installation & Build
1. Clone the repository.
2. Create a build directory:
   ```bash
   mkdir build && cd build
   ```
3. Run CMake:
   ```bash
   cmake ..
   ```
4. Build the project:
   ```bash
   make
   ```

## Usage
Run the compiled binary (requires root privileges for real-time tasks and EtherCAT):
```bash
sudo ./RTClient
```

## Authors
- **Junho Park** - *Initial work*
- **HYU Robotics Control Lab**

## License
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.
