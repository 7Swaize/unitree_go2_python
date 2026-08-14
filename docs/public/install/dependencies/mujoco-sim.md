# Mujoco Simulator Installation

This installation consists of two steps: first, installing the shared Iceoryx2 configuration files, and second, installing the actual simulator.


## Clone the Repository

```bash
cd $GO2_WS
git clone https://github.com/7Swaize/unitree_mujoco.git
git switch v1.1.0
cd unitree_mujoco
```

## Install Mujoco

The cloned repository depends on MuJoCo but does not include the library itself.  
You will need to install MuJoCo separately. Use release version **3.3.6**.

For the GSMST School Linux Laptops use 
* _mujoco-3.3.6-linux-x86_64.tar.gz_


Navigate [here](https://github.com/google-deepmind/mujoco/releases/tag/3.3.6) and download the files for your system architecture.

Through the Linux GUI or command line:
1) Create the directory `~/.mujoco`
2) Change to `~/.mujoco`
3) Extract the download mujoco file to the `~/.mujoco` directory.
4) Remove the original zip file since it is no longer needed

Below is an example for the X86 architecture.

```bash
cd ~/.mujoco
cp ~/Downloads/mujo* .
tar -xvzf mujoco-3.3.6-linux-x86_64.tar.gz 

rm mujoco-3.3.6-linux-x86_64.tar.gz 
```

Create a symlink to the Mujoco source.

```bash
cd $GO2_WS/unitree_mujoco/simulate/
ln -s ~/.mujoco/mujoco-3.3.6 mujoco
```

## Shared Iceoryx2 Configuration Installation

This installation must be done before installing the actual simulator.

Set `CMAKE_PREFIX_PATH` to the installed **iceoryx2-cxx** location.

**Instructions matching our setup**

If you followed the setup instructions for the iceoryx2 library, run the following command using the directory where you cloned the iceoryx2 repository.

```bash
export CMAKE_PREFIX_PATH=$GO2_WS/iceoryx2/target/ff/cc/install
```

Install C++ bindings via CMake.

```bash
cd $GO2_WS/unitree_mujoco/iceoryx_interfaces
cmake -S . -B build
cmake --build build
sudo cmake --install build
```

Install Python bindings via pip.

```bash
pip install .
```


## Mujoco Simulator Installation

Install with verbose logging.

```bash
cd $GO2_WS/unitree_mujoco/simulate
pip install . -v --config-settings=logging.level=INFO
```

# Test Simulator Will Run 
Run the following command in the terminal to test the installation. It should launch the simulator.
```bash
go2sim
```
