# LX-16A, LX-224, HTS-35H and LX-15D Servo Library

Simple Arduino library to operate LX-16A, LX-224, HTS-35H and LX-15D serial servos.

This library sends simple commands to LewanSoul LX-16A, LX-224, HTS-35H  and Hiwonder LX-15D serial bus servos.
It is designed for the Arduino framework and uses the more common 3-pin configuration (TX, RX, direction).

Using the BusLinker v2.2 IS NOT RECOMMENDED, it is not compatible with this library.

# Electrical

The LX-* servos all use a 3.3v driven bi directional asynchronus serial. It is similar to UART, but uses both signals on one pin. Because of this, the Master TX line has to be connected only while transmitting. The correct way to do this is a buffer chip 74HC126. 

https://www.digikey.com/product-detail/en/texas-instruments/SN74HC126N/296-8221-5-ND

This library uses an IO pin passed to the begin() method to flag when the master is transmitting on the bus. When the flag is de-asserted, then the bus is freed for a motor to transmit, with the Masters UART TX line held in high-impedance.

**LX-224 Pinout**

<img src="https://github.com/Hephaestus-Arm/HephaestusArm2/blob/0.1.1/photos/motor_cable.jpg" width="600">

## Detailed Example instructions

How to Wire up an ItsyBitsy to the LewanSoul Bus motors

Here are detailed instructions for how to wire up the Lewansoul motors using the $0.43 chip linked above. https://github.com/Hephaestus-Arm/HephaestusArm2/blob/0.1.1/electronics.md#2-setting-up-the-board

## Generic instructions

```
MCU RX -> Direct Connection -> LX-* Serial Pin
MCU TX -> 74HC126 A   
MCU Flag GPIO -> 74HC126 OE
74HC126 Y -> LX-* Serial Pin
6v-7.5v ->  LX-* Power (center) pin
GND     ->  LX-* GND Pin
```

The MCU Rx pin always listens, and hears its own bytes comming in. This library clears out the incomming bytes and will hang if it could not hear itself talking. 

### ESP32  One Pin Mode

The esp32 microcontrollers have the ability to run the serial Tx in Open Drain mode. This is detected by the library and done automatically. This means the pins for tx and rx are the same pin. 

```
MCU RX/TX -> Direct Connection -> LX-16a Serial Pin
6v-7.5v ->  LX-16a Power (center) pin
GND     ->  LX-16a GND Pin
```
### Other Arduino Quick and dirty/ Hacky one motor setup

This wireing configuration will get you up and running fast. You will get a few failed commands and errored bytes using this method. It is usefull to quickly test a servo on a new system.  

```
MCU RX -> Direct Connection -> LX-16a Serial Pin
MCU TX -> 1K Ohm resistor   -> LX-16a Serial Pin
6v-7.5v ->  LX-16a Power (center) pin
GND     ->  LX-16a GND Pin
```

