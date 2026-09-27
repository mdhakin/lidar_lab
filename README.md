# LiDAR Lab

Experimental C++ application for working with the SLAMTEC RPLIDAR C1.

The current application:

- Connects to the C1 over USB serial
- Acquires continuous 360-degree LiDAR scans
- Converts SLAMTEC measurements into a hardware-independent `LidarScan`
- Displays live 2D range data using Dear ImGui
- Provides a development environment for experimenting with LiDAR perception and navigation algorithms

## Hardware

Current development hardware:

- SLAMTEC RPLIDAR C1
- USB serial interface
- Linux host
- Serial device: `/dev/ttyUSB0`
- Baud rate: `460800`

## Dependencies

This repository uses Git submodules for:

- Dear ImGui
- SLAMTEC RPLIDAR SDK

Additional Linux packages required include:

- CMake
- Ninja
- GCC/G++
- GLFW
- OpenGL development libraries

On Ubuntu:

```bash
sudo apt install build-essential cmake ninja-build libglfw3-dev libgl1-mesa-dev
```

## Clone

The easiest way to clone the repository is to initialize the submodules at the same time:

```bash
git clone --recurse-submodules git@github.com:mdhakin/lidar_lab.git
cd lidar_lab
```

If the repository has already been cloned without the submodules:

```bash
git submodule update --init --recursive
```

To update the submodules later:

```bash
git submodule update --remote --recursive
```

## Build

The SLAMTEC SDK currently uses its own Make-based build system, so build it first:

```bash
make -C external/rplidar_sdk
```

Then configure and build LiDAR Lab:

```bash
cmake -S . -B build -G Ninja
cmake --build build
```

## Run

Connect the RPLIDAR C1 and verify that the serial device is present:

```bash
ls -l /dev/ttyUSB*
```

Then run:

```bash
./build/lidar_lab
```

The application should connect to the C1 and display the current 2D scan.

## Architecture

LiDAR Lab intentionally isolates the SLAMTEC SDK from the rest of the application:

```text
SLAMTEC RPLIDAR C1
        |
        v
SLAMTEC SDK
        |
        v
   lidar.cpp
        |
        v
    LidarScan
        |
        +--------> ImGui visualization
        |
        +--------> Future scan analysis
        |
        +--------> Future navigation experiments
```

The public LiDAR interface uses normalized application-level types such as angle in degrees and distance in meters. SLAMTEC-specific data types and encodings remain inside the LiDAR implementation.

Scan acquisition runs independently from visualization. The acquisition side continuously produces the latest completed scan, while the GUI reads a snapshot for display.

## Project Goals

LiDAR Lab is intended as an experimental environment for understanding and developing 2D LiDAR perception before integrating it into the Aries crawler platform.

Planned experiments include:

- Nearest-obstacle detection
- Range-sector analysis
- Open-space detection
- Preferred-heading calculation
- Wall and boundary detection
- Reactive obstacle avoidance
- Integration with Aries autonomous control
- ROS 2 / SLAM / Nav2 experimentation

The laboratory application is intentionally kept separate from the vehicle control stack so perception and navigation algorithms can be developed and tested independently.
