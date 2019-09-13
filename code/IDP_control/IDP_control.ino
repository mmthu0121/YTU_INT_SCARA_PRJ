#include <PID_v1.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

#define encoder0PinA 2
#define encoder0PinB 8
#define MotorPWM1 10
#define MotorPin1 4
#define MotorPin2 5

#define encoder1PinA 3
#define encoder1PinB 9
#define MotorPWM2 11
#define MotorPin3 7
#define MotorPin4 6

#define laserpin 12
#define buzzer 13

double Input, Output, Setpoint;
double Input1, Output1, Setpoint1;

char x;
double Kp1=8.0, Ki1=0.0, Kd1=0.4; //3.5,0.0.4
double Kp2=5.7, Ki2=0.0, Kd2=0.385;

const double link_1 = 15.5;
const double link_2 = 12.2;
const float pi = 3.142;
double suboutput, suboutput1;
volatile long count;
volatile long count1;
//---------------------------
int val1, val2;
double sub1,sub2;

const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;
bool state = false;
bool home_pos = false;
bool start = false;

char messageFromPC[buffSize] = {0};
//-----------------------------------------
struct angle{
  double theta1;
  double theta2;
};


PID m1PID(&Input, &Output, &Setpoint, Kp1, Ki1, Kd1, DIRECT); //directon = DIRECT
PID m2PID(&Input1, &Output1, &Setpoint1, Kp2, Ki2, Kd2, DIRECT); //directon = DIRECT

//---------------------------------------------------------------
void readEncoder1()
{
  if (digitalRead(encoder0PinA)==digitalRead(encoder0PinB)){
    count = count+1;}
  else{
    count = count-1;}
}

void readEncoder2()
{
  if (digitalRead(encoder1PinA)==digitalRead(encoder1PinB)){
    count1 = count1-1;}
  else{
    count1 = count1+1;}
}

//----------------------------------------------------------------

void M1PIDv2(double desire){
  /*
    motor1 PID Control
  */
  Input = ( (360.0 * 2.0) / (64 * 100.0)) * count;
  Setpoint = desire;
  m1PID.Compute();
  Serial.print(Input);
  Serial.print(" ");

  if (Output > 0){
  digitalWrite(MotorPin1, HIGH);//ccw
  digitalWrite(MotorPin2, LOW);
  suboutput = Output;}

  else{
  suboutput = -Output;
  digitalWrite(MotorPin1, LOW);
  digitalWrite(MotorPin2, HIGH);}

  analogWrite(MotorPWM1, suboutput);
  Serial.print(Output);
}

void M2PIDv2(double desire1){
  /*
    motor2 PID Control
  */
  Input1 = ( (360.0 * 2.0) / (64 * 100.0)) * count1;
  Setpoint1 = desire1;
  m2PID.Compute();
  Serial.print(" ");
  Serial.print(Input1);
  Serial.print(" ");

  if (Output1 > 0){
  digitalWrite(MotorPin3, HIGH);//ccw
  digitalWrite(MotorPin4, LOW);
  suboutput1 = Output1;}

  else{
  suboutput1 = -Output1;
  digitalWrite(MotorPin3, LOW);
  digitalWrite(MotorPin4, HIGH);}

  analogWrite(MotorPWM2, suboutput1);
  Serial.println(Output1);
}
//---------------------------------------------------

angle CToAngle(double x, double y){
  /*
   * input : x,y coordinate (cm)
   * output: shoulder and elbow angle (theta1, theta2)
   * the angles of 2-joint arm are calculated using geomatry equations.
   */
  float a1 = ((x*x) + (y*y) - (link_1*link_1) - (link_2*link_2))/(2*link_1*link_2);
  double q2 = acos(a1); //shoulder angle
  double q1 = (atan(y/x)) - atan((link_2*sin(q2)) / (link_1 + (link_2*cos(q2))));
  double q11 = (180.0/pi) * q1;
  double q22 = (180.0/pi) * q2;
  angle a;
  a.theta1 = q11;
  a.theta2 = q22;
  return a;
}


void receiveData() {
  /*
   * Receives String from Arudino in the format "<"Data",x,y>"
   * The symbols "<" and ">" serves as startMarker and endMarker.
   */

  if(Serial.available() > 0) {

    x = Serial.read();

      // the order of these IF clauses is significant

    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      parseData();
      state = true;

    }

    if(readInProgress) {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1;
      }
    }

    if (x == startMarker) {
      bytesRecvd = 0;
      readInProgress = true;
    }
  }
  else{
    digitalWrite(LED_BUILTIN, LOW);
  }
}


void led()
{
  //this is for the serial communication debugging process.
  if(sub1==8.0 && sub2==14.0){
    digitalWrite(LED_BUILTIN, HIGH);}

  else{
    digitalWrite(LED_BUILTIN, LOW);}
}

void parseData() {

   // split the data into its parts

  char *strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(inputBuffer,",");      // get the first part - the string, "," is delimiter
  strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC from

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  val1 = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ",");
  val2 = atoi(strtokIndx);     // convert this part to an integer

  sub1 = (double)val1;
  sub2 = (double)val2;
}


////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pinMode(MotorPWM1, OUTPUT);
  pinMode(MotorPin1, OUTPUT);
  pinMode(MotorPin2, OUTPUT);
  pinMode(laserpin, OUTPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(encoder0PinA, LOW);
  digitalWrite(encoder0PinB, LOW);
  attachInterrupt(digitalPinToInterrupt(2), readEncoder1, CHANGE);

  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);
  pinMode(MotorPWM2, OUTPUT);
  pinMode(MotorPin3, OUTPUT);
  pinMode(MotorPin4, OUTPUT);
  digitalWrite(encoder1PinA, LOW);
  digitalWrite(encoder1PinB, LOW);
  attachInterrupt(digitalPinToInterrupt(3), readEncoder2, CHANGE);

  m1PID.SetOutputLimits(-255,255);
  m1PID.SetMode(AUTOMATIC);
  m2PID.SetOutputLimits(-255,255);
  m2PID.SetMode(AUTOMATIC);
}

void loop()
{
  receiveData();
  digitalWrite(laserpin, HIGH);
  if (state==true){

    angle degree;
    degree = CToAngle(sub1,sub2);
    M1PIDv2(degree.theta1);
    M2PIDv2(degree.theta2);

  }
//  M1PIDv2(30.0);
//  M2PIDv2(40.0);

  //---------------------------------------
  // arm returning home position experiment
  //   while (Input < 40.0 || Input1 < 70.0){
  //     M1PIDv2(40.0);
  //     M2PIDv2(70.0);
  //   }
  // }
  // start = true;
  //
  // if (start==true){
  //   while (Input != 0.0 || Input1 != 0.0){
  //     M1PIDv2(0.0);
  //     M2PIDv2(0.0);
  //   }
  //   while(1);
  // }
}
