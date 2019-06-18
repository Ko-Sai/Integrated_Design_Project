#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

#define PWM 5
#define CW 7
#define CCW 8
#define encoderPinA 2
#define encoderPinB 3
#define lim_sw 10

float count = 0.0;
double des_angle, act_angle, diff_error, int_error, motorVolt;
double Kp, Ki, Kd, error, prverror, myBit, de, dt, sum_error;
double nowtime, prvtime;
int state = 1;


void setup() {
  
  pinMode(PWM,OUTPUT);
  pinMode(CW,OUTPUT);
  pinMode(CCW,OUTPUT);
  pinMode(encoderPinA,INPUT);
  pinMode(encoderPinB,INPUT);
  attachInterrupt(digitalPinToInterrupt(2), readEncoder ,CHANGE);
  Serial.begin(9600);
  digitalWrite(lim_sw,HIGH);

}

void readEncoder()
{
  if(digitalRead(encoderPinA) == digitalRead(encoderPinB))
  {
    count = count - 1;
  }
  else
  {
    count = count + 1;
  }
}

void loop()
{ 
//  while ( digitalRead(lim_sw) == 1 && state == 1)
//  {
//    digitalWrite(CW,LOW);
//    digitalWrite(CCW,HIGH);
//    analogWrite(PWM,50);
//  }
//
// if ( digitalRead(lim_sw) == 0 )
// {
//  state = 0;
//  count = 0;
// }

  if(Serial.available()>0){
    
    nowtime = millis()/1000.0;
    
    de = error - prverror;
    dt = nowtime - prvtime;
    sum_error = error+prverror;
  
    Kp = 0.005; 
    Ki = 0.5; //Gains
    Kd = 1.5;
  
    des_angle = Serial.read(); 
    act_angle = (360.0/3266.5) * count;
    
    
    error = des_angle - act_angle;
    diff_error = de/dt;
    int_error = sum_error*dt;
  
    motorVolt = (Kp*(error))+(Kd*(diff_error))+(Ki*(int_error));
   
    myBit = (abs(motorVolt)/12.0)*255;
    myBit = constrain (myBit,0,255);
  
    if (des_angle > act_angle)
    {
      digitalWrite(CW,HIGH);
      digitalWrite(CCW,LOW);
    }
    else if ( des_angle < act_angle )
    {
      digitalWrite(CW,LOW);
      digitalWrite(CCW,HIGH);
    }
  
    analogWrite(PWM,myBit);
    
    Serial.print("counts");
    Serial.print(" ");
    Serial.print(count);
    Serial.println(" ");
    Serial.print("Error ");
    Serial.print(error);
    Serial.print(" ");
    Serial.print("Mybit ");
    Serial.print(myBit);
    Serial.print("Switch State ");
    Serial.print(digitalRead(lim_sw));
    Serial.print(" Motor Volt");
    Serial.print(" ");
    Serial.print(motorVolt);
    Serial.print("  Actual angle  ");
    Serial.println(act_angle);
    
    prverror = error;
    prvtime = nowtime;
  }
}

