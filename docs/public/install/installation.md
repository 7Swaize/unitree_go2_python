# Setup of Go2-Control System Environment

<img width="2000" height="700" alt="layers" src="https://github.com/user-attachments/assets/2b8c9bad-b888-4dce-b969-148f9e6cd648" />

Note: The order of installation is important. It is not in the order of the layers themselves.

# Create Project Folder
Create a workspace directory where we will install all applicable dependencies. Below, we used `~/go2-workspace`.

```bash
export GO2_WS=~/go2-workspace
mkdir -p $GO2_WS
cd $GO2_WS
```

# Install Platform Manual Dependencies

## Anaconda Workspace Setup

1. Maunally Download and Install [Anaconda](https://www.anaconda.com/download)
2. Create workspace with Python v3.10
3. Activate the workspace
   
```bash
conda create --name dogenv python=3.10
conda activate dogenv
```

## Install Build Dependencies

Before proceeding with installing other dependencies, ensure the following dependencies are installed:

- CMake (version 3.22 or greater)
- GCC (version 11.4 or greater)
- Make
- Cargo (version 1.85 or greater)

You can install the required packages on Ubuntu 22.04 via the following instructions.

```bash
sudo apt-get update
sudo apt-get install -y cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev clang libclang-dev libglfw3-dev mesa-utils
hash -r
```

Install Cargo.

```bash
curl https://sh.rustup.rs -sSf | sh
```

> [!NOTE]
> Choose the option that states "Proceed with standard installation (default - just press enter)."

Configure your current shell to use the `cargo` command. This is prompted to you at the end of the installation.
Typically, we can do this by executing the following command.

```bash
. "$HOME/.cargo/env"
```

## Layer 2: Clone the 'go2-control' Repository

In this step, we will clone the go2-control wrapper code. However, we will build the wrapper last after installing all dependencies.

```bash
cd $GO2_WS
git clone https://github.com/7Swaize/go2-control.git
cd go2-control/
git switch v1.1.0
```

## Layer 3 and 4: Dependency Installation

Install all the following dependencies with instructions listed below first. It is recommended to install dependencies in the following order.

1. [Layer 3 - Tesseract Engine](dependencies/tesseract-engine.md)
2. [Layer 3 - Unitree SDK](dependencies/unitree-sdk.md)
3. [Layer 3 - Librealsense SDK](dependencies/librealsense-sdk.md)
4. [Layer 3 - Iceoryx2](dependencies/iceoryx2.md)
5. [Layer 4 - Mujoco Simulator](dependencies/mujoco-sim.md)
6. [Layer 3 - ROS2 Workspace](dependencies/ros2-ws.md)


## Layer 2: Install the 'go2-control' Repository

We will  use PIP to build/install the Python wrapper library.

```bash
cd $GO2_WS/go2-control
pip install .
```
