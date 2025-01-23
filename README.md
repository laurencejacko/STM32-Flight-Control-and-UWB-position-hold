# STM32-Flight-Control-and-UWB-position-hold

# Introduction
This repository contains my work on an STM32 based flight controller, that incorporates ultra-wideband (UWB) time of flight messages. This allows the UAV to understand it's XYZ coordinate inside a local reference frame, and makes it possible to perform a position hold function. 

The hardware files for the flight controller and UWB transceiver are available, as Altium Designer files, in this repository. Software was written using ST's HAL in C. The software was consistently built on top of previous code, I've attempted to make it readable however some functions might appear to exist for no reason. 

I cobbled a lot of libraries from other people's work on Github, see references below. 

# STM32 Flight Controller
The Flight controller is built around the STM32H743 microcontroller. It uses ICM 42688-P (IMU), LIS3MDL (Magnetometer), MS5607 (Barometer), MAX-M10s (GPS), SX1280 (LoRa Transceiver) and a CAN transceiver. Molex Picoblade connectors are used to connect to motors, CAN devices, power and an external ELRS radio receiver. 


# UWB Transceiver
The UWB Transceiver is built around the STM32F412 microcontroller. It uses the DW3220 (UWB device) from Qorvo to send and receive UWB messages. Qorvo provides a software library for the STM32F405, this requires a small modification to make compatable with the F412. This is done by commenting out a #define STM32F405..." inside the driver files, and replacing with "#define STM32F412...". 

To create a collection of anchor devices and one tag device. See the reference below. 

# Solution to the multilateration problem
See this paper for the solution to the multilateration problem. This process allows us to use the information regarding the anchor node locations, and the most recent time of flight distances between the mobile tag node and the static anchor nodes. To solve for the UAV's position in 3D space. 

For a 2D solution is it required to have at least 3 anchors providing TOF distances. For 3D solution it is required for at least 4 anchors providing TOF distances. I struggled in getting a stable position control when the UWB antenna on my drone was not pointing directly up. I also found if the multilateration solution suddenly returned a large position change (seen with a sudden height drop on the UAV). It was because at least 1 anchor was reporting a larger error. To address this, I recommend more than 5 anchors (I had 5), otherwise I have included an error check. This compares the magnitude given by multilateration and the magnitude found from squaring the found coordinates. If greater than a value (chosen by looking at the flight logs), I do not update the Z coordinate (ignoring the 3rd dimension, return to requiring only 3 good solutions). 

# Position Control
Position control is largly taken from the ardupilot documents. 

A position error is fed via a P controller to become a velocity setpoint. A velocity error becomes an acceleration setpoint. Which is divided by the UAV thrust (m/s2) and applying inverse tangent to generate a roll/pitch setpoint. 

A height error passes via a P controller into velocity setpoint, velocity error into a PI controller to become acceleration setpoint. Acceleration error passes into a PID controller to become a motor correction. See the implementation that introduce a scalar term to convert between throttle units (from the remote control) and units of m/s2. 

I have only included a position hold function. I don't intend to add a waypoint navigation function. Position hold appears to maintain within +- 0.15m of the setpoint. Oscillations are likely due to antenna misalignment, where at least 1 anchor node reports an incorrect distance, causing potentially unstable harmonics. The 0.15m error was not validated with an external system (motion capture), the drone could visually be seen to hover at it's desired setpoint and required no manual input to throttle, roll, pitch or yaw from the operator. 


# CAN Messaging
Data was sent from the UWB transceiver to the flight controller using the CAN 2.0B protocol. ST's HAL was used to send data between boards. 




