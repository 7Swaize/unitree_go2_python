# ROS2 Humble - Installation

**NOTE: ROS2 Humble installation requires Ubuntu 22.04**

If you do not, you will receive the following error:
```bash
E: Unable to locate package ros-humble-desktop
```

Installation directions adapted from: [ROS2 Humble Documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

These are just the commands to install ROS2 Humble. For more guided or in depth directions, see the official ROS2 Humble documentation linked above.

## Set Locale

Make sure you have a locale which supports UTF-8.

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

## Install Universe Repository to System

Execute. 

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```


## Setup Sources

Ensure that the Ubuntu Universe repository is enabled.

To verify, run the following command:

```bash
apt-cache policy | grep universe
```

Part of the output should look similar to:

```bash
 500 http://ports.ubuntu.com/ubuntu-ports jammy-security/universe arm64 Packages
     release v=22.04,o=Ubuntu,a=jammy-security,n=jammy,l=Ubuntu,c=universe,b=arm64

```


## Install Actual ROS packages 
Install the ros2-apt-source package to your system (after Ubunutu Universe is enabled).

```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```


## Install ROS2 Packages

Ensure system is up to date.

```bash
sudo apt update
sudo apt upgrade
```

Install the Desktop release for ROS2.

```bash
sudo apt install ros-humble-desktop
``` 


### Environment Setup

Set up your environment by sourcing the following file. This allows you to use the `ros2` CLI command.

```bash
source /opt/ros/humble/setup.bash
```

This must be done in every shell instance. Manually doing this can become repetitive, so you can add the command to your `.bashrc` file to have it automatically executed whenever a new terminal session is opened.


### Install Build Dependencies

Ensure system is up to date.

```bash
sudo apt update
sudo apt upgrade
```

Install Python packages for the ROS2 build.

```bash
pip install -U \
    colcon-common-extensions \
    catkin_pkg \
    empy==3.3.4 \
    lark-parser
```

Install colcon and related packages. This is needed to run the `colcon` CLI command to actually build our ROS2 workspace.

```bash
sudo apt install -y \
    build-essential \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-pip
```

Initialize `rosdep`.

```bash
sudo rosdep init
rosdep update
```


# Compute - Installation

Install this dependency for our ROS2 nodes. 

```bash
cd $GO2_WS/go2-control/compute
pip install .
```


# ROS2 Workspace - Installation

Enter the ROS2 workspace directory.

```bash
cd $GO2_WS/go2-control/ros2_ws
```

Install package dependencies via `rosdep`. It might fail to detect packages within your virtual environment. That is fine.

```bash
source /opt/ros/humble/setup.bash
rosdep install \
    --from-paths src \
    --ignore-src \
    -r -y
```

Build our ROS2 workspace.

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```


# ROS 2 Workspace — Optional Configuration

This workspace hosts ROS 2 nodes that subscribe to `PointCloud2` streams published by the physical robot and decode the packed LiDAR point data into NumPy arrays. Some optional configuration is available for this process.

Inside the path `$GO2_WS/go2-control/ros2_ws/src/bringup/config`, there is a file called `lidar_processor.yaml`, which looks like this:

```yaml
lidar_decoder:
  ros__parameters:
    collection:
      optimize_collection: true
      skip_nans: true
```

**`optimize_collection`** — When enabled, invokes a C++-backed decoder instead of the standard NumPy/Python decoder. It is **highly recommended** to keep this set to `true`, as it provides significant performance benefits (mentioned [here](https://github.com/7Swaize/go2-control/issues/98#issuecomment-5336533295)).

**`skip_nans`** — Skips NaN values during collection. This incurs a performance hit, since it requires per-point scalar reads from the byte buffer instead of vectorized `memcpy` invocations (in the standard case).