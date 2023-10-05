# Hiwonder xArm ESP32 Control with ROS2
 This repository is intended for the control of a Hiwonder xArm, that utilises 6 bus servos, with an ESP32 micro and micro-ROS for control on a master machine. The ESP32 is essentially utilised as a motor driver, while micro-ROS is utilized to set and read parameters of the motors using defined ROS publishers, subscribers and messages.
  * Parameters that can be set: Servo angle/position (and time to execute respective movements)
  * Parameters that can be read: Servo angle/position, servo input voltage, servo temperature

**For the use of these motors with an ESP32 and ROS1-Rosserial, please go to this [link](https://github.com/migsdigs/Hiwonder_xArm_ESP32/tree/main/Hiwonder_ESP32).**

**For the setup of the environemnt, including ROS2 installation, micro-ROS installation and setup of the IDE please see the [Setup README](https://github.com/migsdigs/Hiwonder_xArm_ESP32/blob/main/Hiwonder_xArm_ROS2/SETUP_README.md).**


**Test System: Ubuntu 22.04 on WSL, ROS2 Humble Hawksbill, Micro-ROS, ESP32-DevkitC-32e**

Author: [Miguel Garcia Naude](https://github.com/migsdigs)

---

## Hardware Setup
* Hiwonder xArm ESP32
* ESP32-DevKitC-32e

Some Caption    | Another Caption
------------- | -------------
![picture alt]( "Pic1")  | ![picture alt]( "Pic2")

---

## Basic Operation
### Power On
1. Ensure that the servos are powered on (their LEDs will light up) with the 7.5V source.
2. Power on the ESP32 by plugging it in with the micro-USB.
3. Ensure that the Serial lines of the servos are connected to **Pin 33** of the ESP32

### Build & Run
Note again: see **[setup readme](https://github.com/migsdigs/Hiwonder_xArm_ESP32/blob/main/Hiwonder_xArm_ROS2/SETUP_README.md)** for installation instructions.

Change directory to that of your micro-ROS. In my case this is done with:
```bash
cd esp32_microros_ws
pwd
/home/miguel_u22/esp32_microros_ws
```

Inside this directory there should be the following:
```bash
miguel_u22@miguelpc:~/esp32_microros_ws$ ls
build  install  log  src
```

Source the setup.bash file in the install folder:
```bash
source install/setup.bash
```

#### Run
1. Unplug the ESP32 from the host computer (Maintain power to the servos).
2. In the micro-ros directory (for me `/home/miguel_u22/esp32_microros_ws`), run the micro-ROS agent:
   `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6`

   This should produce the following response:
   ```
   [1696497991.336987] info     | TermiosAgentLinux.cpp | init                     | Serial port not found. | device: /dev/ttyUSB0, error 2, waiting for connection...
   [1696497992.346918] info     | TermiosAgentLinux.cpp | init                     | Serial port not found. | device: /dev/ttyUSB0, error 2, waiting for connection...
   ```
   
   If there is an issue, ensure that this is the correct serial connection. This can be verified by plugging in the ESP micro and running the following:
   ```
   dmesg | grep tty
   [ 2256.333770] usb 1-1: cp210x converter now attached to ttyUSB0
   ```
   Which indicates that the serial is connected to `ttyUSB0`.
4. nfrjne
5. 
