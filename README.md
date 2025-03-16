# BLDC Driver

## Overview
This project is an open-source BLDC motor driver designed for drone applications. It implements a sensorless control method using **Back EMF** for efficient and precise motor control.

## Motor Control Methods
There are multiple ways to drive a BLDC motor. The three most common methods are:

1. **Hall Sensor-Based Control**  
   - Uses **hall effect sensors** in the motor to detect rotor position.
   - Suitable for applications requiring high precision.

2. **Back EMF-Based Control (Sensorless)**  
   - Most **common in drone ESCs** (Electronic Speed Controllers).
   - Detects rotor position by measuring **Back EMF voltage** on the floating phase.
   - Eliminates the need for sensors, reducing cost and complexity.

3. **Power Control Mode**  
   - Uses **shunt resistors** to monitor **current in each phase**.
   - Enables precise **torque and power control**.
   - Required for advanced techniques like **Field-Oriented Control (FOC)**.

## Current Implementation
- **Control Method**: Back EMF sensing (sensorless)  
- **Target Application**: Drone ESC

## Future Enhancements
- **CAN Bus Interface** for communication
- **UART Interface** for debugging and configuration
- In the next varient, one board would have the driver for all the 4 motors required for quad copter.

## PCB information 
- Generated gerber files are in gerber folder of the project directory
- PCB file size: 54x29 mm.


