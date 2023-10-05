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

Front View    | Side View
------------- | -------------
![picture alt](https://github.com/migsdigs/Hiwonder_xArm_ESP32/blob/main/assets/hiwonder_arm_numbered.jpg "Front View with Servos Numbered")  | ![picture alt]( "Pic2")


| Servo Number | Model | Range (deg.) | Rotation Speed | Parameter Feedback |
| ----------- | ----------- | ----------- | ----------- | ----------- |
| 1 | ID1 Servo | 160 deg. | 0.39 sec/60deg | Position, Temperture, Voltage |
| 2 | LX-15D Servo | 240 deg. | 0.22 sec/60deg | Position, Temperture, Voltage |
| 3 | LX-15D Servo | 240 deg. | 0.22 sec/60deg | Position, Temperture, Voltage |
| 4 | LX-15D Servo | 240 deg. | 0.22 sec/60deg | Position, Temperture, Voltage |
| 5 | LX-225 Servo | 240 deg. | 0.23 sec/60deg | Position, Temperture, Voltage |
| 6 | LX-15D Servo | 240 deg. | 0.22 sec/60deg | Position, Temperture, Voltage |

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
The position of the servos can be controlled using the `/multi_servo_cmd_sub` topic. Any number of servos can be moved at once by specifying the disired servo angle (centi-degrees) and the time (milli-seconds) for the servos to carry out this command. The publish message from the terminal is structured as follows:
```bash
ros2 topic pub /multi_servo_cmd_sub --once std_msgs/Int64MultiArray "{layout: {dim: [{label: '', size: 0, stride: 0}], data_offset: 0}, data: [12000,12000,12000,12000,12000,12000,500,500,500,500,500,500]}"
```

The data component of the message contains the desired servo positions and the move time. It should always contain 12 integer entries. To be more clear it is structured as follows:

`data:[servo1_desired_pos, servo2_desired_pos, ... , servo6_desired_pos, servo1_move_time, ... , servo6_move_time].` 
In this particular case, servos 2-6 are set to their middle position (120 deg) and the arm moves upright in 500 ms.

The servos will only move if a positive position is given, and if a position is given outside of a servo angular range, the servo will move to its limit. For instance, to move only servo 1 and servo 3, one could give the following:

`ros2 topic pub /multi_servo_cmd_sub --once std_msgs/Int64MultiArray "{layout: {dim: [{label: '', size: 0, stride: 0}], data_offset: 0}, data: [7000,-1,2000,-1,-1,-1,500,500,500,500,500,500]}"`


#### Reading from the servos
As is shown in the table, **Position, Temperature** & **Voltage** can be read from the servos. Voltage and temperature are published on start-up and then every 5 seconds, while servo positions are published at approximately 25 Hz.

1. `/servo_pos_publisher` - publishes servo positions in a JointState message, that includes the servo numbers, positions and time stamps.
   run `ros2 topic echo /servo_pos_publisher` and observe the published servo positions.
   ```
   header:
   stamp:
     sec: 67
     nanosec: 3297586560
   frame_id: frame id
   name:
   - Servo1
   - Servo2
   - Servo3
   - Servo4
   - Servo5
   - Servo6
   position:
   - 15120.0
   - 11952.0
   - 12048.0
   - 11952.0
   - 11904.0
   - 11952.0
   ...
   ```
   
2. `/servo_temp_publisher` - publishes the servo temperature as a Int16MultiArray message. The data is structured as such:
   
   `data:[servo1_temp, servo2_temp, ... , servo6_temp]`.

   Run `ros2 topic echo /servo_temp_publisher` and observe the published servo temperatures.
   ```
   layout:
   dim: []
   data_offset: 0
   data:
   - 47
   - 28
   - 33
   - 29
   - 31
   - 27
   ---
   ```

3. `/servo_volt_publisher` - publishes the servo input voltage (milli-volts) as a Int16MultiArray message. The data is structured as such:
   
   `data:[servo1_vin, servo2_vin, ... , servo6_vin]`.
   
   Run `ros2 topic echo /servo_temp_publisher` and observe the published servo input voltages.
   ```
   layout:
     dim: []
     data_offset: 0
   data:
   - 7607
   - 7629
   - 7629
   - 7585
   - 7618
   - 7583
   ---
   ```


