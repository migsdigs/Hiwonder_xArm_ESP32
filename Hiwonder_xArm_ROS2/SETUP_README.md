# Setup and Install Readme

This is for the total install and setup of the Hiwonder xArm control with an ESP32 and Micro-ROS.

Some notes:
* The [drivers](https://github.com/madhephaestus/lx16a-servo/tree/master) for the Hiwonder bus servos are written by [Kevin Harrington](https://github.com/madhephaestus).

## Dependencies & Build
The development environment used for this was the VS Code IDE with the [PlatformIO extension](https://platformio.org/install/ide?install=vscode). Prior to continuing please add this extension to your VS code environment (and install [VS Code](https://code.visualstudio.com/) or some other PlatformIO compatible IDE).


## ROS2 & Micro-ROS
The communication from the host machine to the ESP32 is done using micro-ROS and ROS2 nodes. [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) due to its its distant EOL.
For the install of ROS2 Humble, one should ensure that the host machine is running [Ubuntu 22.04](https://releases.ubuntu.com/jammy/), 
and then refer to the Ubuntu (Debian) install page [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). It is then recommended that the user looks at some of 
the tutorials, particularly if they have no previous experience with ROS2. It is suggusted that the user at least skims over the tutorials [configuring environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html) and [creating a workspace](https://github.com/migsdigs/Hiwonder_xArm_ESP32/edit/main/Hiwonder_xArm_ROS2/SETUP_README.md#creating-the-ros2-workspace) (please refer to this below), as it might effect the micro-ROS installation later on.
When configuring the environment it is important to set the `ROS_DOMAIN_ID` to 0, and to ignore the setting of the `ROS_LOCAL_HOST` environment variable, as this will avoid conflicts with micro-ROS.

### Creating the ROS2 Workspace
Once the ROS2 installation is complete, one should create a workspace. For more detailed ROS instructions, please click [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html). Otherwise, proceed with the following:

Source ROS2 environment:
```bash
source /opt/ros/humble/setup.bash
```

Create the new directory:
```bash
mkdir -p ~/hiwonder_xArm_ws/src
cd hiwonder_xArm_ws
```

All your new ROS2 packages pertaining to this workspace should be created in `.../hiwonder_xArm_ws/src`.

Build the workspace environment using Colcon:
```bash
colcon build
```

Running `ls` in this terminal should give:
```bash
build  install  log  src
```


### Micro-ROS installation

For the micro-ROS installation, one should refer to [micro-ROS porting to ESP32](https://micro.ros.org/blog/2020/08/27/esp32/) and its related pages. One should thus proceed with the [core-tutorials](https://micro.ros.org//docs/tutorials/core/overview/) or at the very least complete the [First micro-ROS application on Linux](https://micro.ros.org//docs/tutorials/core/first_application_linux/) to ensure that the micro-ROS is installed and running accordingly. 

If you experience difficulties when testing the micro-ROS app with the Ping Pong example in the "First micro-ROS Application in Linux" tutorial, it is likely due to [configuration settings](https://github.com/migsdigs/Hiwonder_xArm_ESP32/edit/main/Hiwonder_xArm_ROS2/SETUP_README.md#2-ros2-installation---configuration-settings) in the ROS2 installation.


## Configuring Micro-ROS for PlatformIO
Having ensured that micro-ROS is successfully installed, we will configure it for PlatformIO. For this we shall refer to both the [Teensy with Arduino](https://micro.ros.org/docs/tutorials/core/teensy_with_arduino/) tutorial and the [micro-ROS for PlatformIO](https://github.com/micro-ROS/micro_ros_platformio) repo. Perform the following in a new terminal:

```bash
# Source the ROS 2 installation
source /opt/ros/humble/setup.bash
# Create a workspace and download the micro-ROS tools
mkdir esp32_microros_ws
cd esp32_microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

```bash
# Download micro-ROS agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

# Perform a dry run
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```

That should result in something like:
```bash
[1696497991.336987] info | TermiosAgentLinux.cpp | init | Serial port not found. | device: /dev/ttyUSB0, error 2, waiting for connection...
[1696497992.346918] info | TermiosAgentLinux.cpp | init | Serial port not found. | device: /dev/ttyUSB0, error 2, waiting for connection...
```

---

## Cloning and Building
Clone this repository into the `src` folder of the workspace. In a new terminal:
```bash
cd hiwonder_xArm_ws/src/
git clone https://github.com/migsdigs/Hiwonder_xArm_ESP32.git
```

Open the project in the PlatformIO environment, the path to this should be similar to `/home/user_Ubuntu/hiwonder_xArm_ws/src/Hiwonder_xArm_ESP32/Hiwonder_xArm_ROS2`. Ensure that the environment is correctly configured, by opening the [platformio.ini](https://github.com/migsdigs/Hiwonder_xArm_ESP32/blob/main/Hiwonder_xArm_ROS2/platformio.ini) file and ensuring it is configured as follows:
```ini
[env:nodemcu-32s]
platform = espressif32@2.0.0
board = nodemcu-32s
framework = arduino
monitor_speed = 115200
lib_deps = 
    madhephaestus/lx16a-servo@^0.9.3
    https://github.com/micro-ROS/micro_ros_platformio
board_microros_distro = humble
board_microros_transport = serial
```

While the board being used is an ESP32-DevKitC-32E, the LX-16a servo driver library functions only for earlier versions of the ESP32 toolchain. The library is verified to work on the NodeMCU-32S development board with the espressif32@2.0.0 toolchain version, and thus we utilize it.


Now open your IDE and the PlatformIO environment. In the terminal, begin with:
```console
sudo apt install -y git cmake python3-pip
sudo apt install platformio
```

The library should already be included as a git library dependence, but if it is not, add the following to the `platform.ini` file:
```ini
...
lib_deps =
    https://github.com/micro-ROS/micro_ros_platformio
```

In the terminal again, ensure that ROS is not sourced i.e. `source /opt/ros/humble/setup.bash` is not called in the terminal or in `~/.bashrc`, and run the following, ensure it is successful:
```console
pio lib install # Install dependencies
pio run # Build the firmware
pio run --target upload # Flash the firmware
```

To trigger a library build and apply library modification on the platformIO build, one can run:
```console
pio run --target clean_microros  # Clean library
```

Micro-ROS dependencies should now be added to the PlatformIO environment.
If you have not done so already, you can now add ROS sourcing to the `~/.bashrc` file with:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

---


## Flash the ESP32
* Build and Upload the code to the ESP32.
* The light on GPIO2 should blink twice, and then rapidly.
* The light on GPIO22 should blink every 5s.

The system should now be ready for use (see basic [readme](https://github.com/migsdigs/Hiwonder_xArm_ESP32/blob/main/README.md) for running) and development of ROS2 packages.
    
---

## Issues
A few problems were encountered throughout the process of setting up the ESP32 for the Hiwonder xArm. They are listed below, along with some of the suggested solutions.

### 1. Read/write permissions on the serial line.
When trying to upload code to the ESP32, it can fail due to a lack of read write permissions on the serial line of interest. This can be solved by running the following in the terminal:
```bash
sudo chmod 666 /dev/ttyUSBX # X is the relevant USB number
```
or
```bash
sudo chmod a+rw /dev/ttyUSBX # X is the relevant USB number
```

This is not permanent, and the change will need to be repeated every time the device is reconnected. Instead on can use:
```bash
sudo usermod -aG dialout <username>    # <username> is your username
```

### 2. Micro-ROS - ROS2 conflict
After installing ROS2, one of the recommended tutorials is [configuring environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html). This sees the setting of a number of environment variables which lead to conflicts when installing micro-ROS, see [github issue](https://github.com/micro-ROS/micro-ROS-demos/issues/78). If you are experiencing issues with communicating with micro-ROS agents, the setting of these environment variables is a possible cause.

### 3. Servos not moving (LX-16a Servo Library & ESP32 Toolchain)
The open source drivers for the Hiwonder servos are written to work for older versions of the ESP espressif toolchain. Later versions, for example espressif@4.0.0 which was released with the ESP32-DevKitC dev board, do not work with these drivers and you will experience read/write errors when trying to communicate with the servos. To ensure this does not occur please set up your `.ini` file as follows:
```ini
[env:nodemcu-32s]
platform = espressif32@2.0.0
board = nodemcu-32s
; [env:esp32dev]
; platform = espressif32
; board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps = 
    madhephaestus/lx16a-servo@^0.9.3
    https://github.com/micro-ROS/micro_ros_platformio
board_microros_distro = humble
board_microros_transport = serial
```

### 4. ROS2 Micro-ROS timers on ESP32
Micro-ROS has build-in functionality for timers and callbacks. Unfortunately, while we require older toolchain for the servo drivers to function, the micro-ROS timers for fail to function properly for the toolchain espressif@2.0.0. As such, we have made use of native ESP timer and interrupt service routines (ISR).

### 5. Reading from the Servos
It was found that sometimes there is difficulty reading from all the servos. When the servos were all daisy chained together 1-6, and when attempting to read at high rates, the first three servos would often return read errors. It was thus found that for best performance:
* Not more than 3 servos should be chained together i.e. chain 1-3, 4-6 and connect their serial wires in parallel on the ESP32 pin (pin 33 in this case).
* Furthermore connecting servo 1 on its own line i.e. 1, 2-3, 4-6 was found to improve the reading on servo 1.
* Lastly, setting power to the servos after launching the micro-ROS connection to the ESP was found to help.

---
