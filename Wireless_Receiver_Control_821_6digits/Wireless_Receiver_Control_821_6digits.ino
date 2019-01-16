#include <SPI.h>                  // needed for Arduino versions later than 0018
#include <PID_v1.h>
#include <VirtualWire.h>
#include <math.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>



const int k = 30;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

byte message[VW_MAX_MESSAGE_LEN]; // a buffer to store the incoming messages
byte messageLength = VW_MAX_MESSAGE_LEN; // the size of the message

//Matlab
char MatlabRawData_Send[64] = "";
String MatlabRawData_Received;

#define ModeOff 0  //Mode Micro Swimmer Off
#define ModeV 1    //Mode Micro Swimmer Velocity Controlled
#define ModeA 2    //Mode Micro Swimmer Angel Controlled
byte Mode = ModeA;

float SpeedSurge, SpeedSway, SpeedHeave, Speed, SpeedNC;

const int Motor_U1PWMA_pin = 3;   //2;    //Motor 1MA PWM-Output -  Timer3B / PE4
const int Motor_U1AIN1_pin = 14;  //23;   //Motor 1MA Direction 1  - PA1
const int Motor_U1AIN2_pin = 15;  //24;   //Motor 1MB Direction 2  - PA2


const int Motor_U2PWMA_pin = 5;    //45;  //Motor 2MA PWM-Output -  Timer5B / PL4
const int Motor_U2AIN1_pin = 22;  //42;  //Motor 2MA Direction 1  - PL7
const int Motor_U2AIN2_pin = 23;  //43;  //Motor 2MB Direction 2  - PL6
//

const int Motor_U3PWMA_pin = 44;  //2;   //3;   //Motor 3MA PWM-Output -  Timer3C / PE5
const int Motor_U3AIN1_pin = 16;  //23;  //12;  //Motor 3MA Direction 1  - PB6
const int Motor_U3AIN2_pin = 17;  //24;  //11;  //Motor 3MB Direction 2  - PB5

const int Motor_U1PWMB_pin = 46;  //45;  //5;    //Motor 1MB PWM-Output -  Timer3A / PE3
const int Motor_U1BIN1_pin = 18;  //42;  //A14;  //Motor 1MA Direction 2  - PK6
const int Motor_U1BIN2_pin = 19;  //43;  //A13;  //Motor 1MB Direction 1  - PK5

const float MaxSpeedA = 50000.0;  //65535 is maximum Motor Setpoint for Angular Controller
const float MinSpeedA = 13000.0;   //Minimum Motor Setpoint for Angular Controller
float qx = 0, qy = 0, qz = 0, qw = 0, ex = 0, ey = 0, ez = 0;


boolean yawControl = false;

// *********************************************************************** //
// ----------------------- Setup Loop ------------------------------------ //
// *********************************************************************** //

void setup() {

  pinMode(Motor_U1AIN1_pin, OUTPUT);
  pinMode(Motor_U1AIN2_pin, OUTPUT);
  pinMode(Motor_U1BIN1_pin, OUTPUT);
  pinMode(Motor_U1BIN2_pin, OUTPUT);
  pinMode(Motor_U2AIN1_pin, OUTPUT);
  pinMode(Motor_U2AIN2_pin, OUTPUT);
  pinMode(Motor_U3AIN1_pin, OUTPUT);
  pinMode(Motor_U3AIN2_pin, OUTPUT);
  delay(10);

  // ---------------------- Setup Serial Connection ----------------------
  //Serial.begin (115200);
  Serial.begin(115200);
  //vw_set_rx_pin(13);
  vw_setup(1000); // Bits per sec

  vw_rx_start(); // Start the receiver
  /* Initialise the sensor */
  //  if(!bno.begin())
  // {
  /* There was a problem detecting the BNO055 ... check your connections */
  //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //  while(1);
  // }
  // delay(1000);
  //  bno.setExtCrystalUse(true);

  SpeedNC = 0;
  SpeedSurge = 0;
  SpeedHeave = 0;
  SpeedSway = 0;

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}





// *********************************************************************** //
// ----------------------- Main Loop ------------------------------------- //
// *********************************************************************** //
void loop()
{

  // IMU readings: quaterions and euler angles
  imu::Quaternion quat = bno.getQuat();
// Commented for Demo .. .only need ex (yaw)
//  qw = quat.w() * 1000.0 + 5000.0;
//  qx = quat.x() * 1000.0 + 5000.0;
//  qy = quat.y() * 1000.0 + 5000.0;
//  qz = quat.z() * 1000.0 + 5000.0;

  sensors_event_t event;
  bno.getEvent(&event);
  ex = event.orientation.x;
//  ey = event.orientation.y + 2000.0;
//  ez = event.orientation.z + 2000.0;

  // Bluetooth command through serial monitor
  if (Serial.available() > 0)
  {
    MatlabRawData_Received = Serial.readStringUntil('#');
    //Serial.println(MatlabRawData_Received);
    Serial.println("1");
    if (MatlabRawData_Received.substring(0, 1) == "M")
    {
      //SpeedNC = MatlabRawData_Received.substring(11, 15).toFloat() - 5000;
      SpeedSurge = MatlabRawData_Received.substring(1, 5).toFloat()   - 5000;
      //SpeedHeave = MatlabRawData_Received.substring(6, 10).toFloat()  - 5000;
      //SpeedSway = MatlabRawData_Received.substring(16, 20).toFloat() - 5000;
    }

    static float setpointNC, setpointSurge, setpointHeave, setpointSway;

    //setpointNC = (int) (SpeedNC/1000.0 * 255);
    setpointSurge = (int) (SpeedSurge / 1000.0 * 255);
    setpointHeave = (int) (SpeedHeave / 1000.0 * 255);
    //setpointSway = (int) (Speed4/1000.0 * 255);;

    Serial.println("Recived Data From Serial Monitor:");
    Serial.print(MatlabRawData_Received + " " );
    Serial.print(setpointSurge);
    Serial.print(" ");
    Serial.print(setpointHeave);
    Serial.println();

    //MotorControl(setpointNC, Motor_U1PWMA_pin, Motor_U1AIN2_pin, Motor_U1AIN1_pin, Mode); //1MA
    MotorControl(setpointSurge, Motor_U2PWMA_pin, Motor_U2AIN2_pin, Motor_U2AIN1_pin, Mode); //2MA
    MotorControl(setpointHeave, Motor_U3PWMA_pin, Motor_U3AIN1_pin, Motor_U3AIN2_pin, Mode); //3MA
    //MotorControl(setpointNC, Motor_U1PWMB_pin, Motor_U1BIN1_pin, Motor_U1BIN2_pin, Mode); //1MB
  }

  // Wireless command through simulink
  if (vw_get_message(message, &messageLength))
  {
    MatlabRawData_Received = String((char *) message);

    if (MatlabRawData_Received.substring(0, 1) == "S") { //start yaw control
      yawControl = true;
    } else if (MatlabRawData_Received.substring(0, 1) == "C") { // restarting sensor
      bno.begin();
    } else if (MatlabRawData_Received.substring(0, 1) == "Z") { // stop yaw control 
      yawControl = false;
    }

    // split message
    else {
        Speed = MatlabRawData_Received.substring(0, 3).toFloat();
        if (Speed <= 600 && Speed >= 400) {
          SpeedHeave = (Speed - 500) ;
        }
        else {
          SpeedSurge = (Speed - 800);
        }
      }

    static float setpointNC, setpointSurge, setpointHeave;
    setpointSurge = (SpeedSurge / 100.0 * 255);
    setpointHeave = (SpeedHeave / 100.0 * 255);

    for (int i = 0; i < messageLength; i++)
    {
      Serial.write(message[i]);
    }

    //MotorControl(setpointNC, Motor_U1PWMA_pin, Motor_U1AIN2_pin, Motor_U1AIN1_pin, Mode); //1MA
    MotorControl(setpointSurge, Motor_U2PWMA_pin, Motor_U2AIN2_pin, Motor_U2AIN1_pin, Mode); //2MA
    MotorControl(setpointHeave, Motor_U3PWMA_pin, Motor_U3AIN1_pin, Motor_U3AIN2_pin, Mode); //3MA
    //MotorControl(setpointSway, Motor_U1PWMA_pin, Motor_U1AIN2_pin, Motor_U1AIN1_pin, Mode); //1MA

  }

  // Yaw control
  if (yawControl) {
  static float SpeedYaw;
  int setpointSway;
  SpeedYaw = yawPos(ex);
  setpointSway = (int) (SpeedYaw / 1000.0 * 255);
  Serial.println(setpointSway);
  //Serial.print(ex);
  //Serial.print(" ");
  //Serial.println(setpointSway);
  //MotorControl(setpointSway, Motor_U1PWMA_pin, Motor_U1AIN2_pin, Motor_U1AIN1_pin, Mode); //1MA
  MotorControl(setpointSway, Motor_U1PWMB_pin, Motor_U1BIN1_pin, Motor_U1BIN2_pin, Mode); //1MB
  }

}

int yawPos(float y) {
  int err = 0;
  if (y > 180) {
    err = y - 360;
  } else {
    err = y;
  }
  return (int) err * k;
}


boolean speedThreshold (float sp1, float  sp2, float sp3, float  sp4) {
  boolean check = false;
  if (abs(sp1) <= 1000 && abs(sp2)) {
    check = true;
    //Serial.println("received full message");
  } else {
    //Serial.println("did not receive full message");
  }
  return check;
}









