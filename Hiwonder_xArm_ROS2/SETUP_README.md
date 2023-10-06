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



## Issues
### 1. Read/write permissions on the serial line.

### 2. ROS2 Installation - configuration settings

### 3. Micro-ROS installation

### 4. Servo Drivers Library and ESP32 Toolchain

### 5. ROS2 Micro-ROS timers on ESP32
