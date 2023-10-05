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
   ```bash
   [1696497991.336987] info | TermiosAgentLinux.cpp | init | Serial port not found. | device: /dev/ttyUSB0, error 2, waiting for connection...
   [1696497992.346918] info | TermiosAgentLinux.cpp | init | Serial port not found. | device: /dev/ttyUSB0, error 2, waiting for connection...
   ```
   
   If there is an issue, ensure that this is the correct serial connection. This can be verified by plugging in the ESP micro and running the following:
   ```bash
   dmesg | grep tty
   [ 2256.333770] usb 1-1: cp210x converter now attached to ttyUSB0
   ```
   Which indicates that the serial is connected to `ttyUSB0`.
4. Re-connect the ESP32 to the host computer. Something similar to the following should display in the same terminal:
   ```bash
   [1696498217.465170] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3
   [1696498217.465918] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 6
   [1696498226.028623] info     | Root.cpp           | create_client            | create                 | client_key: 0x340D845A, session_id: 0x81
   [1696498226.028907] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x340D845A, address: 0
   [1696498226.029119] debug    | SerialAgentLinux.cpp | send_message             | [** <<SER>> **]        | client_key: 0x340D845A, len: 19, data:
   0000: 81 00 00 00 04 01 0B 00 00 00 58 52 43 45 01 00 01 0F 00
   [1696498226.041597] debug    | SerialAgentLinux.cpp | recv_message             | [==>> SER <<==]        | client_key: 0x340D845A, len: 52, data:
   0000: 81 80 00 00 01 07 2A 00 00 0A 00 01 01 03 00 00 1B 00 00 00 00 01 FB 3F 13 00 00 00 48 69 77 6F
   0020: 6E 64 65 72 5F 78 41 72 6D 5F 6E 6F 64 65 00 00 00 00 00 00
   [1696498226.064239] info     | ProxyClient.cpp    | create_participant       | participant created    | client_key: 0x340D845A, participant_id: 0x000(1)
   [1696498226.064379] debug    | SerialAgentLinux.cpp | send_message             | [** <<SER>> **]        | client_key: 0x340D845A, len: 14, data:
   0000: 81 80 00 00 05 01 06 00 00 0A 00 01 00 00
   [1696498226.064408] debug    | SerialAgentLinux.cpp | send_message             | [** <<SER>> **]        | client_key: 0x340D845A, len: 13, data:
   0000: 81 00 00 00 0A 01 05 00 01 00 00 00 80
   [1696498226.075602] debug    | SerialAgentLinux.cpp | recv_message             | [==>> SER <<==]        | client_key: 0x340D845A, len: 13, data:
   0000: 81 00 00 00 0A 01 05 00 01 00 00 00 80
   [1696498226.082373] debug    | SerialAgentLinux.cpp | recv_message             | [==>> SER <<==]        | client_key: 0x340D845A, len: 96, data:
   ```
  The agent is now running, and we can begin communicating with the ESP32 via ROS2 to control the servos.

#### ROS Topics (might need to update)
Running `ros2 topic list`, the available ROS2 topics should be listed.
```bash
miguel_u22@miguelpc:~$ ros2 topic list
/multi_servo_cmd_sub
/parameter_events
/rosout
/servo_pos_publisher
/servo_temp_publisher
/servo_volt_publisher
```

#### Moving the servos
The position of the servos can be controlled using the `/multi_servo_cmd_sub` topic. Any number of servos can be moved at once by specifying the disired servo angle (centi-degrees) and the time (milli-seconds) for the servos to carry out this command.
