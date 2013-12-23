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
#define Right_tilt_pwm			9// old 3
#define Right_tilt_in1			4
#define Right_tilt_in2			5
// change to actuator
int MinVal = 500;
double target_pos = 550;
const byte MY_ADDRESS = 40;

double Encoder_Position  = 0;
double Right_pwm = 0;
double Right_Gap =0;
double servo_pos =0;

unsigned long milliSecsSinceLastUpdate;
unsigned long CurrentTime = 0;
int FrameRate = 30;

//volatile boolean haveData = false;
volatile long Target = 550;

PID Right_tilt(&Encoder_Position, &Right_pwm, &target_pos, 3,0.1,0.001, DIRECT);
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
   if (target_pos < MinVal){
    target_pos = MinVal;
   } 
   //haveData = true;     
   }  // end if have enough data
 }  // end of receiveEvent

void requestEvent()
{
 
  sendSensor (A0);//sends current position
 
}
// sends data back to master
void sendSensor (const byte which)
  {
  int val = getFeedback(which);//analogRead (which);
  byte buf [2];
  
    buf [0] = val >> 8;
    buf [1] = val & 0xFF;
    Wire.write (buf, 2);
  }  // end of sendSensor

void Right_drive()
{
  //if (Right_Gap < 50)
  //{
  //Right_tilt.SetTunings(4, 0, 1);
  //}
  //else
  //{
    //Right_tilt.SetTunings(20, 0, 0);
  //}
  Right_tilt.Compute();
  Right_tilt.SetOutputLimits(-250 ,250);
  
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

///feedback
int getFeedback(int a){
int j;
int mean;
int result;
int test;
int reading[20];
boolean done;

for (j=0; j<20; j++){
reading[j] = analogRead(a); //get raw data from servo potentiometer
delay(3);
} // sort the readings low to high in array
done = false; // clear sorting flag
while(done != true){ // simple swap sort, sorts numbers from lowest to highest
done = true;
for (j=0; j<20; j++){
if (reading[j] > reading[j + 1]){ // sorting numbers here
test = reading[j + 1];
reading [j+1] = reading[j] ;
reading[j] = test;
done = false;
}
}
}
mean = 0;
for (int k=6; k<14; k++){ //discard the 6 highest and 6 lowest readings
mean += reading[k];
}
result = mean/8; //average useful readings
return(result);
}    // END GET FEEDBACK

  

