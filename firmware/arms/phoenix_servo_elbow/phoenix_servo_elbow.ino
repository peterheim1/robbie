/*
working servo controller 
change MY_ADDRESS to the correct value
todo
fine tune PID
remove motor noise
change to phoenix_servo.h driver
add extra data for the return
turn off motor driver when not moving

    error
    current
    voltage
    is moving


*/
#include <PID_v1.h>
#include <Wire.h>
#include "Arduino.h"
#include <I2C_Anything.h>
#define Right_tilt_pwm			9
#define Right_tilt_in1			4
#define Right_tilt_in2			5
// change to actuator
//int MinVal = 10;
double target_pos = 180;
const byte MY_ADDRESS = 46;

double Encoder_Position  = 0;
double Right_pwm = 0;
double Right_Gap =0;
double servo_pos =0;

unsigned long milliSecsSinceLastUpdate;
unsigned long CurrentTime = 0;
int FrameRate = 30;

//volatile boolean haveData = false;
volatile long Target = 20;

PID Right_tilt(&Encoder_Position, &Right_pwm, &target_pos, 1.0,0.05,0.00, DIRECT);
//PID Right_tilt(&Encoder_Position, &Right_pwm, &target_pos, 3.5,0.09,0.001, DIRECT);
/*
##############################################################################
##############################################################################


set up


####################################################################################
################################################################################
*/
void setup() 
{
  Wire.begin (MY_ADDRESS);
  Serial.begin (57600);
  Wire.onReceive (receiveEvent); //recieve target from master
  Wire.onRequest(requestEvent); // send data to master
  TCCR1B = TCCR1B & 0b11111000 | 0x01 ;

  pinMode(Right_tilt_in1, OUTPUT);
  pinMode(Right_tilt_in2, OUTPUT);
  pinMode(Right_tilt_pwm, OUTPUT);
  Right_tilt.SetMode(AUTOMATIC);
}  // end of setup
/*
###################################################################################
#####################################################################################

                                          LOOP
                                          
########################################################################################
#####################################################################################

*/


void loop()
{
  Encoder_Position =  analogRead(0);
  
  Serial.print(MY_ADDRESS);
  Serial.print(" ");
  Serial.print(target_pos);
  Serial.print(" ");
  Serial.print(Encoder_Position);
 
  Serial.println(" ");

  Right_Gap = target_pos - Encoder_Position;
  
  //pid timing loop
  milliSecsSinceLastUpdate = millis() - CurrentTime;
  if(milliSecsSinceLastUpdate >= FrameRate)
  {
  
  Right_drive();
  CurrentTime = millis();
  }
    

}  // end of loop

// called by interrupt service routine when incoming target data arrives
void receiveEvent (int howMany)
 {
 if (howMany >= (sizeof Target))
   {
     
   I2C_readAnything (Target); 
   target_pos = Target; 
   
   }  // end if have enough data
 }  // end of receiveEvent

void requestEvent()
{
 
  sendSensor (A0);//sends current position
 
}
// sends data back to master
void sendSensor (const byte which)
  {
  int val = analogRead (which);
  byte buf [2];
  
    buf [0] = val >> 8;
    buf [1] = val & 0xFF;
    Wire.write (buf, 2);
  }  // end of sendSensor

void Right_drive()
{
  
  Right_tilt.Compute();
  Right_tilt.SetOutputLimits(-250 ,250);//reduce speed to minamize bounce
  
  if (Right_pwm < 0){
    digitalWrite(Right_tilt_in1, HIGH);
    digitalWrite(Right_tilt_in2, LOW);
    int right_power = abs(Right_pwm);
    if (right_power < 70) 
    {right_power = 0;}
    analogWrite(Right_tilt_pwm, right_power);
  }
  
  if (Right_pwm > 0){
    digitalWrite(Right_tilt_in1, LOW);
    digitalWrite(Right_tilt_in2, HIGH);
    int right_power = abs(Right_pwm);
    if (right_power < 70) 
    {right_power = 0;}
    analogWrite(Right_tilt_pwm, right_power);
  }
}
  

