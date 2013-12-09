/*






*/


#include <math.h>
#include <Servo.h>
#include <PID_v1.h>

#include <Messenger.h>
#include "Arduino.h"

#define Right_tilt_pwm			5
#define Right_tilt_in1			7
#define Right_tilt_in2			8
double Right_tilt_enc  = 0;
double Right_tilt_target  = 0;
double Right_pwm = 0;
double Right_Gap =0;

#define Left_tilt_pwm			3
#define Left_tilt_in1			6
#define Left_tilt_in2			4
double Left_tilt_enc = 0;
double Left_tilt_target  = 500;
double Left_pwm =0;
double Left_Input =0;
double Left_Gap =0;

// Instantiate Messenger object with the message function and the default separator (the space character)
Messenger _Messenger = Messenger();

PID Left_tilt(&Left_tilt_enc, &Left_pwm, &Left_Input, 6,1,0, DIRECT);
PID Right_tilt(&Right_tilt_enc, &Right_pwm, &Left_Input, 4,1,0, DIRECT);

void setup()
{
   Serial.begin(115200);
  _Messenger.attach(OnMssageCompleted);

pinMode(Right_tilt_in1, OUTPUT);
pinMode(Right_tilt_in2, OUTPUT);
pinMode(Left_tilt_in1, OUTPUT);
pinMode(Left_tilt_in2, OUTPUT);
Left_tilt.SetMode(AUTOMATIC);
Right_tilt.SetMode(AUTOMATIC);
//TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
//TCCR2B = _BV(CS22);

digitalWrite(Right_tilt_in1, HIGH);
digitalWrite(Right_tilt_in2, LOW);

}


void loop()
{
Left_Input =  analogRead(1);
Right_tilt_enc =  analogRead(0);
Left_tilt_enc =  analogRead(2);
Left_Gap = Left_Input - Left_tilt_enc;
Right_Gap = Left_Input - Right_tilt_enc;
Ser_print();
  //Left_tilt.Compute();
 // Left_tilt.SetOutputLimits(-150 ,150);
//  Left_drive();
  Right_drive();
  //analogWrite(Left_tilt_pwm, 100);
 // analogWrite(Right_tilt_pwm, 250);
}

void Ser_print()
{
  Serial.print("j"); // joint message
  Serial.print("\t");
  Serial.print(Left_Input);
  Serial.print("\t");
  Serial.print(Right_tilt_enc);
  Serial.print("\t");
  Serial.print(Right_Gap);
  Serial.print("\t");
  Serial.print(Right_pwm);
  Serial.print("\t");
  Serial.print("\n");



}

void Left_drive()
{
  if (Left_Gap < 10)
  {
  Left_tilt.SetTunings(1, 0.2, 0);
  }
  else
  {
    Left_tilt.SetTunings(4.5, 2, .1);
  }
  Left_tilt.Compute();
  Left_tilt.SetOutputLimits(-150 ,150);
  
  if (Left_pwm < 0){
    digitalWrite(Left_tilt_in1, HIGH);
    digitalWrite(Left_tilt_in2, LOW);
    analogWrite(Left_tilt_pwm, abs(Left_pwm));
  }
  
  if (Left_pwm > 0){
    digitalWrite(Left_tilt_in1, LOW);
    digitalWrite(Left_tilt_in2, HIGH);
    analogWrite(Left_tilt_pwm, abs(Left_pwm));
  }
}
void Right_drive()
{
  if (Right_Gap < 10)
  {
  Right_tilt.SetTunings(1, 0.2, 0);
  }
  else
  {
    Right_tilt.SetTunings(4.5, 2, .1);
  }
  Right_tilt.Compute();
  Right_tilt.SetOutputLimits(-150 ,150);
  
  if (Right_pwm < 0){
    digitalWrite(Right_tilt_in1, HIGH);
    digitalWrite(Right_tilt_in2, LOW);
    analogWrite(Right_tilt_pwm, abs(Right_pwm));
  }
  
  if (Right_pwm > 0){
    digitalWrite(Right_tilt_in1, LOW);
    digitalWrite(Right_tilt_in2, HIGH);
    analogWrite(Right_tilt_pwm, abs(Right_pwm));
  }
  
  
  
  
}

void ReadSerial()
{
  while (Serial.available())
  {
    _Messenger.process(Serial.read());
  }
}

// Define messenger function
void OnMssageCompleted()
{
  if (_Messenger.checkString("s"))
  {
    //SetSpeed();
    //OmniSpeed();
    
    return;
    
  }   if (_Messenger.checkString("DriveGeometry"))
  {
    //InitializeDriveGeometry();
    return;
  }

  
// clear out unrecognized content
  while(_Messenger.available())
  {
    _Messenger.readInt();
  }
}

