# Setup and Install Readme

This is for the total install and setup of the Hiwonder xArm control with an ESP32 and Micro-ROS.

Some notes:
* The [drivers](https://github.com/madhephaestus/lx16a-servo/tree/master) for the Hiwonder bus servos are written by [Kevin Harrington](https://github.com/madhephaestus).

## Dependencies & Build
The development environment used for this was the VS Code IDE with the [PlatformIO extension](https://platformio.org/install/ide?install=vscode). Prior to continuing please add this extension to your VS code environment (and install [VS Code](https://code.visualstudio.com/) or some other PlatformIO compatible IDE).

Clone this repository:
```bash
cd
mkdir hiwonder_xArm_ws
git clone https://github.com/migsdigs/Hiwonder_xArm_ESP32.git
```

Open the project in the PlatformIO environment, the path to this should be similar to `/home/user_Ubuntu/hiwonder_xArm_ws/Hiwonder_xArm_ESP32/Hiwonder_xArm_ROS2`. Ensure that the environment is correctly configured, by opening the [platformio.ini](https://github.com/migsdigs/Hiwonder_xArm_ESP32/blob/main/Hiwonder_xArm_ROS2/platformio.ini) file and ensuring it is configured as follows:
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

Next we should ensure the installation and configuration of ROS2 and Micro-ROS.

## ROS2 & Micro-ROS
The communication from the host machine to the ESP32 is done using micro-ROS and ROS2 nodes. [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) due to its its distant EOL.
For the install of ROS2 Humble, one should ensure that the host machine is running [Ubuntu 22.04](https://releases.ubuntu.com/jammy/), 
and then refer to the Ubuntu (Debian) install page [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). It is then recommended that the user looks at some of 
the tutorials, particularly if they have no previous experience with ROS2. It is suggusted that the user at least skims over the tutorials [configuring environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
and [creating a workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html), as it might effect the micro-ROS installation later on.
When configuring the environment it is important to set the ROS domain ID to 0, and to ignore the setting of the ROS_LOCAL_HOST environment variable, as this will avoid conflicts with micro-ROS.

For the micro-ROS installation, one should refer to [micro-ROS porting to ESP32](https://micro.ros.org/blog/2020/08/27/esp32/) and its related pages. One should thus proceed
with the [core-tutorials](https://micro.ros.org//docs/tutorials/core/overview/) or at the very least complete the [First micro-ROS application on Linux](https://micro.ros.org//docs/tutorials/core/first_application_linux/) to ensure that the micro-ROS is installed and running accordingly, and then look at the [Teensy with Arduino](https://micro.ros.org//docs/tutorials/core/teensy_with_arduino/) tutorial. 

If you experience difficulties when testing the micro-ROS app in the "First micro-ROS Application in Linux" tutorial, it is likely due to [configuration settings](https://github.com/migsdigs/Hiwonder_xArm_ESP32/edit/main/Hiwonder_xArm_ROS2/SETUP_README.md#2-ros2-installation---configuration-settings) in the ROS2 installation.


## Flash the ESP32

## Issues
### 1. Read/write permissions on the serial line.

### 2. ROS2 Installation - configuration settings

### 3. Micro-ROS installation

### 4. Servos not moving (LX-16a Servo Library & ESP32 Toolchain)

### 5. ROS2 Micro-ROS timers on ESP32
