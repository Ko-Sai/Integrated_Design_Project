#include <Servo.h> 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

#define PWM1 5 //.............
#define PWM2 6 // Enable Pins.
#define PWM3 46 //.............

#define CW1 7 //..............
#define CW2 4 //Int1Pins......
#define CW3 44 //..............

#define CCW1 8 //.............
#define CCW2 9 //Int2Pins.....
#define CCW3 45 //.............

#define encoderPinA1 2 //.............
#define encoderPinB1 22 //            .
#define encoderPinA2 3 // EncoderPins.
#define encoderPinB2 23 //            . 
#define encoderPinA3 18//            .
#define encoderPinB3 24 //.............

#define lim_sw1 11 // ...............
#define lim_sw2 12 // LimitSwitchPins
#define lim_sw3 13 // ...............

volatile float count1 = 0.0,count2 = 0.0,count3 = 0.0;
double setPoint1,setPoint2,setPoint3,actAngle1,actAngle2,actAngle3;
//double Cont1,Cont2,Cont3;
double Error1,Error2,Error3,prvError1,prvError2,prvError3;
double motor1_PWM,motor2_PWM,motor3_PWM;
double tol1,tol2,tol3;
double input1,input2,input3;
unsigned long nowTime;
unsigned long prvTime,dt;
int var = 1;

Servo myServo;

void setup() {
  pinMode(PWM1,OUTPUT);
  pinMode(CW1,OUTPUT);
  pinMode(CCW1,OUTPUT);
  pinMode(encoderPinA1,INPUT);
  pinMode(encoderPinB1,INPUT);
  myServo.attach(26);
  attachInterrupt(digitalPinToInterrupt(2), readEncoder1 ,CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), readEncoder2 ,CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), readEncoder3 ,CHANGE);
  Serial.begin(9600);
  digitalWrite(lim_sw1,HIGH);
  digitalWrite(lim_sw2,HIGH);
  digitalWrite(lim_sw3,HIGH);  
}
//Funtions to read encoders------------------------------------------------------------------------------------------------------------------------------------------------------

void readEncoder1()
{
  if(digitalRead(encoderPinA1) == digitalRead(encoderPinB1))
  {
    count1 = count1 - 1;
  }
  else
  {
    count1 = count1 + 1;
  }
}
void readEncoder2()
{
  if(digitalRead(encoderPinA2) == digitalRead(encoderPinB2))
  {
    count2 = count2 - 1;
  }
  else
  {
    count2 = count2 + 1;
  }
}
void readEncoder3()
{
  if(digitalRead(encoderPinA3) == digitalRead(encoderPinB3))
  {
    count3 = count3 - 1;
  }
  else
  {
    count3 = count3 + 1;
  }
}
//Function to calculate PID value...............................................................................................................................................

double cal_PID (volatile float count,double Error,double prvError,double Kp,double Kd,double Ki)
{
  double diffError,intError,motorVolt,myBit;
  
   diffError = (Error - prvError)/dt;
   intError = (Error + prvError) * dt;

   motorVolt = (Kp*(Error))+(Kd*(diffError))+(Ki*(intError));
   
   myBit = (abs(motorVolt)/12.0)*255;
   myBit = constrain (myBit,0,255);   
  
  return myBit;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Function to drive Motor--------------------------------------------------------------------------------------------------------------------------------------------------------

void driveMotor(double analogValue, double Error, int CW, int CCW, int PWM)
{
    if (Error >= 0.5)
  {
    digitalWrite(CW,HIGH);
    digitalWrite(CCW,LOW);
  }
  else if ( Error <= -0.5 )
  {
    digitalWrite(CW,LOW);
    digitalWrite(CCW,HIGH);
  }
//  else if ( Error >= -0.5 && Error <= 0.5)
//  {
//    digitalWrite(CW,LOW);
//    digitalWrite(CCW,LOW);
//    analogWrite (PWM,0);
//}
  analogWrite(PWM,analogValue); 
}
//Home Funtion------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Motor_Home()
{
  
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() {
  
  tol1 = 4.88;
  tol2 = 4.4;
  tol3 = 0.45;
  
  setPoint1 = 20.0 - tol1;
  setPoint2 = 20.0 + tol2;
  setPoint3 = 20.0;

  actAngle1 = (360.0/3200.00) * count1; //tolerence 4.88
  actAngle2 = (360.0/1100.00) * count2; //tolerence 3.3
  actAngle3 = (360.0/1100.00) * count3;

  Error1 = setPoint1 - actAngle1;
  Error2 = setPoint2 - actAngle2; 
  Error3 = setPoint3 - actAngle3;
   
  nowTime = millis();
  dt = nowTime - prvTime;

  motor1_PWM = cal_PID(count1,Error1,prvError1,0.3,9.0,0.0);
  motor2_PWM = cal_PID(count2,Error2,prvError2,0.32,28.5,0.0);
  motor3_PWM = cal_PID(count3,Error3,prvError3,0.3,9.0,0.00001);
  
// if (nowTime < 3000 )
// {
//  driveMotor(motor3_PWM,Error3,CW3,CCW3,PWM3);
// }
// else if (nowTime > 3000 && nowTime < 6000 )
// {
//  driveMotor(motor1_PWM,Error1,CW1,CCW1,PWM1);
// }
// else if (nowTime > 6000 && nowTime < 10000);
// {
//  driveMotor(motor2_PWM,Error2,CW2,CCW2,PWM2);
// }

//  input1 = input1 + 1;
//  input1 = constrain(input1,0,motor1_PWM);
//  input2 = input2 + 1;
//  input2 = constrain(input2,0,motor2_PWM);
//  input3 = input3 + 1;
//  input3 = constrain(input3,0,motor3_PWM);
  
  driveMotor(motor3_PWM,Error3,CW3,CCW3,PWM3);
  driveMotor(motor1_PWM,Error1,CW1,CCW1,PWM1);
  driveMotor(motor2_PWM,Error2,CW2,CCW2,PWM2);
  //myServo.write(50);

  prvError1 = Error1;
  prvError2 = Error2;
  prvError3 = Error3;

  Serial.print(actAngle1);
  Serial.print(" ");
  Serial.print(actAngle2);
  Serial.print(" ");
  Serial.println(actAngle3);
}
