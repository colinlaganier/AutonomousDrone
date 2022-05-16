# Autonomous Indoor Drone Swarm Sanitation System

## Description  

Created by Colin Laganier under the supervision of Prof. Robert Shorten as part of the MEng Final Year Project. 

Progress can be followed on the regularly update logbook for additional information about the project: [https://bit.ly/3Ni6ypL](https://bit.ly/3Ni6ypL)

## Hardware

<img src="./Hardware/solidwork_hardware.jpg" width="500">
<sub>Solidworks Hardware Assembly</sub>
<!-- &nbsp; -->
 
The drone is built upon the Diatone Taycan C25 MK2 frame, with 2004 BetaFPV brushless DC Motors and a Flywoo Goku GN745 Flight Controller ESC unit running ArduCopter 4.1.5 firmware. To fit its purpose of a sanitation drone the following pieces were designed and added:
* Transparent Nylon 100mL Water tank, fitted with a refill valve and a silicone coating to ensure watertightness.
* Waterpump and PLA support for spraying of the cleaning solution.
* Acrylic Raspberry Pi Zero support.
* PLA legs.  

The control software is running on a Raspberry Pi Zero 2 W running Debian on each drone, connected to a central PC running Ubuntu 16.04 using a TCP/IP communication.  

## PCB

<img src="./PCB/PCB_render.jpg" width="500">
<sub>Altium Designer PCB Assembly</sub>

A custom PCB was designed to hold together the DWM1001C module from Decawave, as well as a MPU6050 IMU and a QMC5883L Compass for positioning; as well as a DRV8871 to control the spraying pump. The focus of the PCB is ensuring a small footprint to house all of the components on a small size platform. The PCB was designed with Altium Designer and manufactured and assembled by JLCPCB.

## Room Mapping

## Positioning

## Motion Planning
