# taflab
### Proof of concept: High-bandwidth LIDAR underwater wireless communication 
The goal of the project was to provide a demo demonstrating how LIDAR communication may be achieved
via a swarm of underwater wireless UAVs. We showed this at a very fundamental level by integrating
Arduino microcontrollers into out-of-shelf toy submarines and implemented a controller for position  
and stability control to reflect a laser beam source from the bottom of the water basin, and onto a target on the surface of
the water. 

For simplicity, the laser position was fixed. The pipeline flow consisted of using a simple webcam for live video
processing with Simulink. The Simulink controller output motor commands which were sent via UART communication to an
Arduino with a wireless transmitter attached. The transmitter (315 MHz, lowest available and compatible transmitter we
found) then sent the motor command signals in which the receiver attached to the Arduino in the submarines underwater 
applied to the motors. 

This repo contains the Arduino code for the receivers and transmitters, as well as the simulink model used and the Matlab
script to interface Simulink with Arduino.

## File Descriptions
Serial_Transmitter reads data sent through serial and parses the string as a byte array 
to send through 315 MHz transmitters using VirtualWire library.

Wireless_Reciever_Control reads data from the 315 MHz receiver (motor commands) and sends corresponding PWM.
signals to motors of the submarine.

Arduino_calibration instantiates an Arduino object to interface Matlab and Arduino using the simulink file FullModel_V3.


