# Setup and Install Readme

This is for the total install and setup of the Hiwonder xArm control with an ESP32 and Micro-ROS.

Some notes:
* The [drivers](https://github.com/madhephaestus/lx16a-servo/tree/master) for the Hiwonder bus servos are written by [Kevin Harrington](https://github.com/madhephaestus).

## ROS2 & Micro-ROS
The communication from the host machine to the ESP32 is done using micro-ROS and ROS2 nodes. [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) due to its its distant EOL.
For the install of ROS2 Humble, one should ensure that the host machine is running [Ubuntu 22.04](https://releases.ubuntu.com/jammy/), 
and then refer to the Ubuntu (Debian) install page [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). It is then recommended that the user looks at some of 
the tutorials, particularly if they have no previous experience with ROS2. It is suggusted that the user at least skims over the tutorials [configuring environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
and [creating a workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html), as it might effect the micro-ROS installation later on.
When configuring the environment it is important to set the ROS domain ID to 0, and to ignore the setting of the ROS_LOCAL_HOST environment variable, as this will avoid conflicts with micro-ROS.

For the micro-ROS installation, one should refer to [micro-ROS porting to ESP32](https://micro.ros.org/blog/2020/08/27/esp32/) and its related pages. 

## Issues
### 1. Read/write permissions on the serial line.

### 2. ROS2 Installation - configuration settings

### 3. Micro-ROS installation

### 4. Servo Drivers Library

### 5. ROS2 Micro-ROS timers on ESP32
