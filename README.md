# Hiwonder xArm ESP32 Control with ROS-Serial
 This repository is intended for the control of a Hiwonder xArm, that utilises 6 bus servos, with an ESP32 micro and rosserial for control on a master machine. The ESP32 is essentially utilised as a motor driver, while rosserial is utilized to set and read parameters of the motors using defined ROS publishers, subscribers and messages.
  * Parameters that can be set: Servo angle/position (and time to execute respective movements)
  * Parameters that can be read: Servo angle/position, servo input voltage, servo temperature
