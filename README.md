# Hiwonder xArm ESP32 Control with ROS2
 This repository is intended for the control of a Hiwonder xArm, that utilises 6 bus servos, with an ESP32 micro and micro-ROS for control on a master machine. The ESP32 is essentially utilised as a motor driver, while micro-ROS is utilized to set and read parameters of the motors using defined ROS publishers, subscribers and messages.
  * Parameters that can be set: Servo angle/position (and time to execute respective movements)
  * Parameters that can be read: Servo angle/position, servo input voltage, servo temperature

**For the use of these motors with an ESP32 and ROS1-Rosserial, please go to this [link](https://github.com/migsdigs/Hiwonder_xArm_ESP32/tree/main/Hiwonder_ESP32).**

**For the setup of the environemnt, including ROS2 installation, micro-ROS installation and setup of the IDE please see the [Setup README](https://github.com/migsdigs/Hiwonder_xArm_ESP32/blob/main/Hiwonder_xArm_ROS2/SETUP_README.md).**

