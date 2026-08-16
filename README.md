# Go2 Control

A Python SDK for controlling the Unitree Go2 robot and Mujoco Simulator via attachable modules.

For Ben Schepens.

## Installation - How To Setup The Simulator Environment

The wrapper targets **Ubuntu 22.04**. The MuJoCo simulator, however, is also compatible with **Ubuntu 24.04**. However, execution of this wrapper on the robot itself under **Ubuntu 24.04** is not guaranteed to work.

For installation instructions, see [here](docs/public/install/installation.md).


## Examples - How To Start Coding With The Simulator

Below are sample files demonstrating how to progammatically interact with the virtual Go2 robot in the mujoco environment.  Start with these to write your own programs.

### Core Module Examples
- [Movement Examples](examples/modules/movement/movement.md) — Examples for working with the controller's movement capabilities.
- [Video Examples](examples/modules/video/video.md) — Examples for working with the controller's video capabilities.
- [OCR Examples](examples/modules/ocr/ocr.md) — Examples for working with the controller's OCR capabilties.
- [LIDAR Examples](examples/modules/lidar/lidar.md) — Examples for working with the controller's LIDAR capabilities.

### Advanced Examples
- [Aruco Marker Examples](examples/aruco_markers/aruco_markers.md) — Examples for detecting and reacting to Aruco markers.
- [Terrain Generator Examples](examples/terrain_generator/terrain_generator.md) — Examples for working with the simulator's terrain generation capabilties.

### Getting Started

The example files are intended as reference implementations. When creating your own program, **copy the example file to a new location before modifying it** rather than editing the original example directly. This makes it easier to keep the examples unchanged and update them in the future.

Run an example (or your copied version) from the using:

```bash
python3 path/to/file.py
```

When running an example inside the simulator, launch the simulator inside another terminal. Make sure you are inside the conda enviroment in which you installed the `go2sim` package. It is recommended to launch the simulator **before** running any example scripts.

To start the simulator, run the following command.
```bash
go2sim
```
 
## Go2-Control Documentation

[API Documentation](https://go2-control.readthedocs.io/en/latest/)

[FAQ](docs/public/FAQ.md)

[Directory Structure](docs/public/repo_structure.md)


## GSMST Reference Information

Reference information regarding device and environment can be found [here](refs)
