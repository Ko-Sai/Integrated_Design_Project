#include <Servo.h> 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

#define PWM1 5 //.............
#define PWM2 6 // Enable Pins.
#define PWM3 10 //.............

#define CW1 7 //..............
#define CW2 4 //Int1Pins......
#define CW3 44 //..............

#define CCW1 8 //.............
#define CCW2 9 //Int2Pins.....
#define CCW3 45 //.............

#define encoderPinA1 2 //.............
#define encoderPinB1 22 //            .
#define encoderPinA2 3 // EncoderPins.
#define encoderPinB2 23 // 
#define encoderPinA3 18//            .
#define encoderPinB3 24 //.............

#define vac1 30
#define vac2 31
#define vac_pwm 13

#define lim_sw1 11 // ...............

#define lim_sw2 12 // LimitSwitchPins

#define lim_sw3 13 // ...............

 

volatile float count1 = 0.0,count2 = 0.0,count3 = 0.0;

double setPoint1,setPoint2,setPoint3,actAngle1,actAngle2,actAngle3;

//double Cont1,Cont2,Cont3;

double Error1,Error2,Error3,prvError1,prvError2,prvError3;

double motor1_PWM,motor2_PWM,motor3_PWM;

double InPWM1;

double tol1,tol2,tol3;

double input1,input2,input3;

unsigned long nowTime;

unsigned long prvTime,dt;

//---------------------------
int val1, val2, val3, val4;
double sub1, sub2, sub3, sub4;

const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;
bool state = false;

char messageFromPC[buffSize] = {0};
//-----------------------------------------
struct angle{
  double theta1;
  double theta2;
  double theta3;
  double theta4;
};

Servo myServo;

void receiveData() {
  /*
   * Receives String from Arudino in the format "<"Data",x,y>"
   * The symbols "<" and ">" serves as startMarker and endMarker.
   */

  if(Serial.available() > 0) {

    char x = Serial.read();

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

  strtokIndx = strtok(NULL, ",");
  val3 = atoi(strtokIndx);     // convert this part to an integer

//  strtokIndx = strtok(NULL, ",");
//  val4 = atoi(strtokIndx);     // convert this part to an integer

  sub1 = (double)val1;
  sub2 = (double)val2;
  sub3 = (double)val3;
//  sub4 = (double)val4;
}
 

int stage = 0;

int homestate = 0;




 

void setup() {

 

  myServo.attach(36);

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

   intError = (Error + prvError)*dt;

 

   motorVolt = (Kp*(Error))+(Kd*(diffError))+(Ki*(intError));

  

   myBit = (abs(motorVolt)/12.0)*255;

   myBit = constrain (myBit,0,120);  

  

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

  analogWrite(PWM,analogValue);

}

//Home Funtion------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void Motor_Home()

{

  switch (stage)

  {

   case 0:

 

    if (digitalRead(lim_sw1) == HIGH)

    {

    digitalWrite(CW3,LOW);

    digitalWrite(CCW3,HIGH);

    analogWrite(PWM3,70);

    }

   

    else

    {

      stage = 1;

    }

   

    break;

//................................................

    case 1:

   

//    count3 = 0.0;

    delay(1000);

    if (digitalRead(lim_sw2) == HIGH)

    {

    digitalWrite(CW1,LOW);

    digitalWrite(CCW1,HIGH);

    analogWrite(PWM1,40);

    }

   

    else

    {
    digitalWrite(CW1,HIGH);

    digitalWrite(CCW1,LOW);

    analogWrite(PWM1,80);
    delay(200);
      stage = 2;
      

    }

   

    break;

//.........................................

    case 2:

   

//    count1 = 0.0;

  
    digitalWrite(CW1,LOW);

    digitalWrite(CCW1,LOW);

    analogWrite(PWM1,0);
   

    delay(1000);

    if (digitalRead(lim_sw3) == HIGH)

    {

    digitalWrite(CW2,LOW);

    digitalWrite(CCW2,HIGH);

    analogWrite(PWM2,50);

    }

    else

    {

      stage = 3;

    }

    break;

//...................................................

    case 3:

    myServo.write(120);
    
    count1 = 0.0;

    count2 = 0.0;

    count3 = 0.0;

    stage = 4;

    homestate = 1;

 

    break;

  }

}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop()

{
  receiveData();

  if (state==true){
    

//    while (homestate == 0)
//  
//    {
//  
//      Motor_Home();
//  
//      if (homestate == 1)
//  
//      {
//  
//        break;
//  
//      }
//  
//    }
  
  
  
    tol1 = 4.88;
  
    tol2 = 4.4;
  
    tol3 = 0.45;
  
     
  
//    setPoint1 = 84 - tol1;
//  
//    setPoint2 = 65 + tol2;
//  
//    setPoint3 = 37;
  
   
  
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

    if ((nowTime/1000) >= 0 && (nowTime/1000) <= 25){
        
      digitalWrite(vac1, HIGH);
      digitalWrite(vac2, LOW);
      analogWrite(vac_pwm, 90);
      setPoint1 = 91 - tol1;
    
      setPoint2 = 65 + tol2;
    
      setPoint3 = sub3;
     
    
       if ((nowTime/1000) >= 8.0 )
      {
        driveMotor(motor3_PWM,Error3,CW3,CCW3,PWM3);
    }
      if ((nowTime/1000) >= 13.0)
      {
        driveMotor(motor2_PWM,Error2,CW2,CCW2,PWM2);
      }
      if ((nowTime/1000) >= 20.0)
      {
  //      InPWM1 = InPWM1 + 0.2;
  //      InPWM1 = constrain(InPWM1,0,PWM1);
        driveMotor(motor1_PWM,Error1,CW1,CCW1,PWM1);      
      }
//      if ((nowTime/1000) >= 24.0)
//      {
//        myServo.write(160);
//      }
//      if ((nowTime/1000) >= 25.0)
//      {
//        digitalWrite(vac1, HIGH);
//        digitalWrite(vac2, LOW);
//        analogWrite(vac_pwm, 95);
//      }
    }
    else if ((nowTime/1000) >= 24.0)
    {
      setPoint1 = 30 - tol1;
    
      setPoint2 = 0 + tol2;
    
      setPoint3 = 60;
      driveMotor(motor3_PWM,Error3,CW3,CCW3,PWM3);
      driveMotor(motor1_PWM,Error1,CW1,CCW1,PWM1);
      driveMotor(motor2_PWM,Error2,CW2,CCW2,PWM2);     
    }
    else if ((nowTime/1000) >= 28.0){
      digitalWrite(vac1, LOW);
      digitalWrite(vac2, HIGH);
      analogWrite(vac_pwm, 90);
    }
   
  
//    driveMotor(motor3_PWM,Error3,CW3,CCW3,PWM3);
  //  driveMotor(motor1_PWM,Error1,CW1,CCW1,PWM1);
  //  driveMotor(motor2_PWM,Error2,CW2,CCW2,PWM2);
  
   
  
    prvError1 = Error1;
  
    prvError2 = Error2;
  
    prvError3 = Error3;
  
   
  
    prvTime = nowTime;
  
   
  
  Serial.print(actAngle3);
  
  Serial.print(" ");
  
  Serial.print(actAngle2);
  
  Serial.print(" ");
  
  Serial.println(actAngle1);
  
   
  
  //Serial.print(count3);
  //
  //Serial.print("// ");;
  //
  //Serial.print(count1);
  //
  //Serial.print("// ");
  //
  //Serial.println(count2);
  
   
  
  //Serial.print(digitalRead(lim_sw1));
  
  //Serial.print(digitalRead(lim_sw2));
  
  //Serial.println(digitalRead(lim_sw3));
  
  } 
  
  }
  
   
  

