#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

//motor 1
#define encoder0PinA 2
#define encoder0PinB 8
#define MotorPWM1 10
#define MotorPin1 4
#define MotorPin2 5

//motor 2
#define encoder1PinA 3
#define encoder1PinB 9
#define MotorPWM2 11
#define MotorPin3 6
#define MotorPin4 7

//Variables for Motor 1
int       counts = 0.0;
double    MotorVolt, myBit, Vsupply, myFreq;
double    now_time, now_act_angle, des_angle, now_error;
double    samplingTime, prv_time, prv_error, edot;

//Variables for Motor 2
int       counts1 = 0.0;
double    MotorVolt1, myBit1, Vsupply1, myFreq1;
double    now_time1, now_act_angle1, des_angle1, now_error1;
double    samplingTime1, prv_time1, prv_error1, edot1;

void setup() {
  myFreq          = 10.0;
  myFreq1          = 10.0;
  prv_time        = 0.0;

  // error for each encoder
  prv_error       = 0.0;
  prv_error1       = 0.0;   

  Serial.begin(9600);

  //setting up Encoder 1
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pinMode(MotorPWM1, OUTPUT);
  pinMode(MotorPin1, OUTPUT);
  pinMode(MotorPin2, OUTPUT);
  digitalWrite(encoder0PinA, LOW);
  digitalWrite(encoder0PinB, LOW);
  attachInterrupt(digitalPinToInterrupt(2), readEncoder1, CHANGE);  //Check here for more. https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

  //setting up Encoder 2
  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);
  pinMode(MotorPWM2, OUTPUT);
  pinMode(MotorPin3, OUTPUT);
  pinMode(MotorPin4, OUTPUT);
  digitalWrite(encoder1PinA, LOW);
  digitalWrite(encoder1PinB, LOW);
  attachInterrupt(digitalPinToInterrupt(3), readEncoder2, CHANGE);
}

//read Encoder 1
void readEncoder1()
{
  if (digitalRead(encoder0PinB) == digitalRead(encoder0PinA))
  {
    counts = counts - 1;
  }
  else
  {
    counts = counts + 1;
  }
}

//read Encoder 2
void readEncoder2()
{
  if (digitalRead(encoder1PinB) == digitalRead(encoder1PinA))
  {
    counts1 = counts1 + 1;
  }
  else
  {
    counts1 = counts1 - 1;
  }
}

void loop() {
  now_time    = millis() / 1000.0;  // we will be using a single clock for both motor

  //motor1 control
  now_act_angle = ( (360.0 * 2.0) / (64 * 100.0)) * counts;        //convert encoder counts to angle
  des_angle = 90.0;              //*sin(2*PI*0.3*now_time);       //gear ratio changes
  now_error = des_angle - now_act_angle;                          //P controller
  edot      = (now_error - prv_error) / (now_time - prv_time);    //D controller
  MotorVolt = 0.2 * now_error + 0.01 * edot;
  myBit     = (abs(MotorVolt) / 12.0) * 255; 
  if (myBit > 255)  {
    myBit = 255;
  }
  else              {
    myBit = myBit;
  }
  if (MotorVolt > 0)  {
    digitalWrite(MotorPin1, HIGH);
    digitalWrite(MotorPin2, LOW);                 //CW
    analogWrite(MotorPWM1, 70);                   // 50<x<100
  }
  else {
    digitalWrite(MotorPin1, LOW);
    digitalWrite(MotorPin2, HIGH);                //CCW
    analogWrite(MotorPWM1, 70); 
  }

  if (now_act_angle == des_angle)
  {
    analogWrite(MotorPWM1, 0);
  }
  
  Serial.print (now_act_angle);
  Serial.print(" ");
  Serial.print (des_angle);
  Serial.print(" || ");
  
  prv_error = now_error;                      // update the error for the close loop

  //-------------------------------------------------------------
  //the following control code is the same as motor 1 control

  //motor 2 control
  now_act_angle1 = ( (360.0 * 2.0) / (64 * 100.0)) * counts1;
  des_angle1 = 101.0;
  now_error1 = des_angle1 - now_act_angle1;
  edot1      = (now_error1 - prv_error1) / (now_time - prv_time);
  MotorVolt1 = 0.2 * now_error1 + 0.01 * edot1;
  myBit1     = (abs(MotorVolt1) / 12.0) * 255;
  if (myBit1 > 255)  {
    myBit1 = 255;
  }
  else              {
    myBit1 = myBit1;
  }
  if (MotorVolt1 > 0)  {
    digitalWrite(MotorPin3, HIGH);
    digitalWrite(MotorPin4, LOW);     //CW
    analogWrite(MotorPWM2, 70); 
  }
  else {
    digitalWrite(MotorPin3, LOW);
    digitalWrite(MotorPin4, HIGH);    //CCW
    analogWrite(MotorPWM2, 70); 
  }

  if (now_act_angle1 == des_angle1)
  {
    Serial.println("Motor 2 Stop");
    analogWrite(MotorPWM2, 0);
  }
  
  Serial.print (now_act_angle1);
  Serial.print(" ");
  Serial.print (des_angle1);
  Serial.print(" ");
  Serial.println(" ");
  prv_error1 = now_error1;
  //end of motor 2 control
  
  prv_time  = now_time;         // update the time at the end of the code
}
