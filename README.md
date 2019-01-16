# taflab
Theoretical and Applied Fluids Laboratory Project documentation and code. 

## Description
Serial_Transmitter reads data sent through serial and parses the string as a byte array 
to send through 315 MHz transmitter using VirtualWire library

Wireless_Reciever_Control reads data from the 315 MHz receiver (motor commands) and sends corresponding PWM
signals to motors of the submarine.

Arduino_calibration instantiates an Arduino object to interface Matlab and Arduino using the simulink file FullModel_V3

To view the application of this code visit https://navarrojesme.wixsite.com/website/research
