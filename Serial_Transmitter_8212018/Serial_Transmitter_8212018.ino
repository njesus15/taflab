#include <SPI.h>                  // needed for Arduino versions later than 0018
#include <math.h>

#include <Wire.h>
#include <VirtualWire.h>
//#include <utility/imumaths.h>
  
//Matlab
byte MatlabRawData_Send1[64] = "";
byte MatlabRawData_Send2[64] = ""; 
String MatlabRawData_Received;
String serialString;
boolean checkread = false;
boolean has = false;


float Speed1, Speed2, Speed3, Speed4;
const int32_t frequency = 10000;  // Frequency in Hz


// *********************************************************************** //
// ----------------------- Setup Loop ------------------------------------ //
// *********************************************************************** //

void setup() {


  // ---------------------- Setup Serial Connection ----------------------
  Serial.begin (115200);
  // Initialize the IO and ISR
  //vw_set_tx_pin(12);
  vw_setup(1000); // Bits per sec
  delay(1000);
  }

// *********************************************************************** //
// ----------------------- Main Loop ------------------------------------- //
// *********************************************************************** //
void loop() {

  if (Serial.available() > 0) {
    //Serial.readBytes(MatlabRawData_Send, 64);
    MatlabRawData_Received = Serial.readStringUntil('#');
    //Serial.println(MatlabRawData_Received);
    Serial.println("1");
    checkread = true;
  }

if (checkread) {
  has = true;
  for (int i = 0; i < MatlabRawData_Received.length(); i++) {
    if ( i < 3) {
    MatlabRawData_Send1[i] = (char*) MatlabRawData_Received[i];
    Serial.write(MatlabRawData_Send1[i]);
    } else {
    MatlabRawData_Send2[i-3] = (char*) MatlabRawData_Received[i];
    Serial.write(MatlabRawData_Send2[i]);
    }
  }
  send(MatlabRawData_Send1);
  delay(10);
  send(MatlabRawData_Send2);
  checkread = false;
}
}

//if (checkread) {
//  has = true;
//  for (int i = 0; i < MatlabRawData_Received.length(); i++) {
//    if ( i < 3) {
//    MatlabRawData_Send1[i] = (char*) MatlabRawData_Received[i];
//    } else {
//    MatlabRawData_Send2[i] = (char*) MatlabRawData_Received[i];
//    }
//  }
//  send(MatlabRawData_Send1);
//  send(MatlabRawData_Send2);
//  checkread = false;

//if (has) {
  //send(MatlabRawData_Send);
  //Serial.println("sending");
//}


void send (char *message)
{
vw_send((uint8_t *)message, strlen(message));
vw_wait_tx(); // Wait until the whole message is gone
}


