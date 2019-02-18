// This is the version 1.1 of the Motor Control Code

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
//#include <cmath.h>

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
int       counts1 = 0.0;
int       counts2 = 0.0;
const float pi = 3.14285714286;
double    MotorVolt, myBit, Vsupply, myFreq, myFreq1;
double    now_time, now_act_angle, now_act_angle1, now_act_angle2, des_angle, now_error, des_angle1, now_error1,des_angle2, now_error2;
double    samplingTime, prv_time, prv_error, prv_error1, prv_error2, edot;

void setup() {
  myFreq          = 10.0;
  myFreq1          = 10.0;
  prv_time        = 0.0;
  
  // error for each encoder
  prv_error       = 0.0;
  prv_error1       = 0.0; 
  prv_error2       = 0.0;   

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
    counts1 = counts1 - 1;
  }
  else
  {
    counts1 = counts1 + 1;
  }
}

//read Encoder 2
void readEncoder2()
{
  if (digitalRead(encoder1PinB) == digitalRead(encoder1PinA))
  {
    counts2 = counts2 + 1;
  }
  else
  {
    counts2 = counts2 - 1;
  }
}

int motorControl(int MotorPinA, int MotorPinB, int MotorPWM, int des_angle, int prv_error, int counts, int PWMspeed, double Kp, double Kd) { 
  
  now_act_angle = ( (360.0 * 2.0) / (64 * 100.0)) * counts;        //convert encoder counts to angle
  now_error = des_angle - now_act_angle;                          //P controller
  edot      = (now_error - prv_error) / (now_time - prv_time);    //D controller
  MotorVolt = Kp * now_error + Kd * edot;
  myBit     = (abs(MotorVolt) / 12.0) * 255; 
  if (myBit > 255)  {
    myBit = 255;
  }
  else              {
    myBit = myBit;
  }
  if (MotorVolt > 0)  {
    digitalWrite(MotorPinA, HIGH);
    digitalWrite(MotorPinB, LOW);                 //CW
    analogWrite(MotorPWM, myBit);              // 50<x<100
  }
  else {
    digitalWrite(MotorPinA, LOW);
    digitalWrite(MotorPinB, HIGH);                //CCW
    analogWrite(MotorPWM, myBit); 
  }

  if (now_act_angle == des_angle)
  {
    analogWrite(MotorPWM, 0);
  }
  prv_error = now_error;              // update the error for the close loop

  return prv_error;
}


void loop() {
  now_time    = millis() / 1000.0;  // we will be using a single clock for both motor

  des_angle1  = 90;
  des_angle2  = 90;
  Serial.print(des_angle1);
  Serial.print("  ");
  Serial.println(des_angle2);
  
  //motor1 control
  prv_error1 = motorControl(MotorPin1, MotorPin2, MotorPWM1, des_angle1, prv_error1, counts1, 70, 0.2, 0.01);                  

  //motor 2 control
  prv_error2 = motorControl(MotorPin3, MotorPin4, MotorPWM2, des_angle2, prv_error2, counts2, 70, 0.2, 0.01);  
  
  prv_time  = now_time;         // update the time at the end of the code
}


//double cod2angD2(double x, double y, double length1, double length2)  {
//  double x2 = x*x;
//  double y2 = y*y;
//  double L1 = length1*length1;
//  double L2 = length2*length2;
//
//  double r  = sqrt(x2+y2);
//  double a2 = acos(x2+y2-L1-L2)/(2*length1*length2);
//  double a2d = a2*180/ pi ;
//  a2d = a2d.real;
// 
//  double a1 = atan(y/x) - atan(length2* sin(a2)/(length1+length2*cos(a2)));
//  double a1d = a1*180/pi;
//  a1d = a1d.real;
//
//  return a1d;
//  return a2d;
//}
