/////////////////////////////////////////////////////
/*The following code was written by Lenze Palaganas*/
/////////////////////////////////////////////////////

#include <Wire.h>
#include <SPI.h>
#include <Servo.h>   
#include "PCA9685.h"          

/*
 Youtube Video Source of IK Function Bases and Calculations
 (This does not include the method by which we moved the servo functions,it
  mainly serves as a base for our IK functionality. It has been edited to 
  work with our current system.)
  
 https://www.youtube.com/watch?v=Y8ueTjqCcAg

 Information We Used for The servo shield
 https://howtomechatronics.com/how-it-works/how-servo-motors-work-how-to-control-servos-using-arduino/
*/

//****** IK Setup ******

 
int pos, CurrentAngle;
int i;

#define PI 3.14159265

float ShoulderLength = 6;
float ElbowLength = 4;
float WristLength = 4;
float Hypot, Slope, CirclePointX, CirclePointY;

//x is now y and y is now z to reflect aerial grid
//z is fixed and hardcoded

int y_calc;
//When further from the origin, add 1 to y

double x1 = 3.5 + 2;
double y1 = 10.4 + 1 ;


double z1 = 2;

double x2 = 0;
double y2 = 0;
double z2 = 0;

float A, B, C,
      PosYAnswer,   //positive Y portion of Quadratic Equation.
      PosZAnswer,   //positive Z portion of Qudratic Equation.
      NegYAnswer,   //negative Y portion of Quadratic Equation.
      NegZAnswer,   //negative Z portion of Quadratic Equation.
      Angle_A,      //Angle located at origin (0,0).
      Angle_A_Temp, //Second 1/2 of Angle located at origin (0,0).Shoulder servo setting.
      Angle_B,      //Elbow servo setting.
      Angle_C;      //Wrist servo setting subtracted from 180.

//********************************

/*Driver Items*/

PCA9685 driver;
// PCA9685 outputs = 12-bit = 4096 steps
// 2.5% of 20ms = 0.5ms ; 12.5% of 20ms = 2.5ms
// 2.5% of 4096 = 102 steps; 12.5% of 4096 = 512 steps


// (-90deg, +90deg)
PCA9685_ServoEvaluator pwmServo(136, 500); 
PCA9685_ServoEvaluator pwmServo5(50, 470);
PCA9685_ServoEvaluator pwmServo6(45, 470);

// (0deg, 90deg, 180deg)
PCA9685_ServoEvaluator pwmServo2(110, 290, 470); 
PCA9685_ServoEvaluator pwmServo3(0, 180, 360); 
PCA9685_ServoEvaluator pwmServo4(0, 180, 360); 

int angle = 0;    

void setup() 
{ 
  Wire.begin();                 
  Wire.setClock(400000);        
  driver.resetDevices();       
  driver.init(B000000);        
  driver.setPWMFrequency(50);  

  Serial.begin(9600); 
  rest();
  delay(500);
  open_claw(); 
}
 
void loop()
{
  /////////////// Slave Arduino Code ///////////////////////
  
//  float val1;
//  Serial.print(Serial.available());
//  if(Serial.available()>0) 
//  {
//    // read the incoming byte:
//    val1 = Serial.read();
//    Serial.print(val1);
//  }
//
//  x1 = val1;

//  if(Serial.available() > 0) 
//  {
//    // read the incoming byte:
//    val2 = Serial.read();
//  
//    // say what you got:
//    Serial.print("I received: ");
//    Serial.println(val, DEC);
//  }
//
//  y1 = val2;
  
  ////////////////////////////////////////////////////////////
  if (x1 != -1 && y1 != 0)
  {
    int y_calc = y1;
    int x_calc = x1;
  
    double x_sq = sq(x1);
    double y_sq = sq(y1);
  
  
    if(x_sq < 0)
    {
      x_sq = -x_sq;
    }
  
    double theta_rad = atan2(y1,x1);
    float theta_deg = (theta_rad*180)/PI;
  
    double sum_xy = x_sq + y_sq;
    float hypot_yc = sqrt(sum_xy);
    float posy = hypot_yc;
    float posz = z1;
  
    if(x1 < 0)
    {
      theta_deg = -theta_deg + 90;
      
    }
    else
    {
      theta_deg = -theta_deg + 90;
    }
    
  
    if(x1 = 0)
    {
      posy = y1;
    }
    
    driver.setChannelPWM(9, pwmServo.pwmForAngle(-theta_deg));
    delay(500);
    for (y1 = posy - 1 ;y1 < posy; y1 = y1 + 1)
    {
      for (z1 = posz - 1 ;z1 < posz; z1 = z1 + 1)
      {
      if ((y1 < ShoulderLength) && (z1 < ShoulderLength)) 
      {
        Special_Calc_Point(); 
        CurrentAngle = round(driver.getChannelPWM(11));
        driver.setChannelPWM(12, pwmServo4.pwmForAngle(80));
        delay(2000);
      }
      else 
      {
        Calc_Point();
        CurrentAngle = round(driver.getChannelPWM(11));
        driver.setChannelPWM(12, pwmServo4.pwmForAngle(80));
        delay(2000);
    
      }
      }
    }
    delay(5000);
    close_claw();
    delay(1000);
    rest();
    drop_off_R();
    rest();

  }
  while(1){}
}

////////////////////////////
//// Open Claw Function ////
////////////////////////////

void open_claw() 
{ 
  driver.setChannelPWM(14, pwmServo6.pwmForAngle(-45));
  delay(500);
}

void close_claw()
{
  driver.setChannelPWM(14, pwmServo6.pwmForAngle(20));
  delay(500);
}

void forward(int ang)
{
  driver.setChannelPWM(10, pwmServo2.pwmForAngle(ang));
  delay(1000);
}

void backward()
{
  driver.setChannelPWM(10, pwmServo2.pwmForAngle(90));
  delay(1000);
}

void rest()
{
  driver.setChannelPWM(9, pwmServo.pwmForAngle(0));
  delay(2000);
  driver.setChannelPWM(10, pwmServo2.pwmForAngle(90));
  delay(2000);
  driver.setChannelPWM(11, pwmServo3.pwmForAngle(45));
  delay(2000);
  driver.setChannelPWM(12, pwmServo4.pwmForAngle(45));
  delay(2000);
  driver.setChannelPWM(13, pwmServo4.pwmForAngle(-90));
  delay(1000);
  close_claw();
  delay(500);
}



/////////////////////////////////
////// Front Pickup Function ////
/////////////////////////////////

void drop_off_L()
{
  driver.setChannelPWM(9, pwmServo.pwmForAngle(90));
  delay(500);
  forward(60);
  delay(500);
  driver.setChannelPWM(12, pwmServo4.pwmForAngle(90));
  delay(1000);
  open_claw();
  delay(500);
  forward(90);
  delay(500);
  driver.setChannelPWM(9, pwmServo.pwmForAngle(0));

}

void drop_off_R()
{
  driver.setChannelPWM(9, pwmServo.pwmForAngle(-90));
  delay(500);
  forward(60);
  delay(500);
  driver.setChannelPWM(12, pwmServo4.pwmForAngle(90));
  delay(1000);
  open_claw();
  delay(500);
  forward(90);
  delay(500);
  driver.setChannelPWM(9, pwmServo.pwmForAngle(0));
}

//////////////////////////////
////// Scan Area Function ////
//////////////////////////////


void scan_area() 
{ 
  //Scanner has yet to be purchased, so the setup is still being determined
  driver.setChannelPWM(9, pwmServo.pwmForAngle(90));
  delay(1000);
  driver.setChannelPWM(9, pwmServo.pwmForAngle(-90));
  delay(1000);
  driver.setChannelPWM(9, pwmServo.pwmForAngle(0));
  delay(1000);
}

//////////////////////////
////// IK Functions //////
//////////////////////////

/*Based on Youtube Video's Calculations
 * Edited to work with Servo Shield programming
*/


void Special_Calc_Point() 
{
  Hypot = sqrt(sq(y1) + sq(z1)) + WristLength;
  A = ElbowLength;
  B = Hypot;
  C = ShoulderLength;

  Angle_A = acos((sq(B) + sq(C) - sq(A)) / (2 * B * C)) * (180 / PI);
  Angle_B =  acos((sq(C) + sq(A) - sq(B)) / (2 * A * C)) * (180 / PI);
  Angle_C = acos((sq(A) + sq(B) - sq(C)) / (2 * A * B)) * (180 / PI);

  if (!isnan(Angle_A)) 
  {     
    driver.setChannelPWM(10, pwmServo2.pwmForAngle(Angle_A));
    delay(1000);
  }

  if (!isnan(Angle_B)) 
  { 

      CurrentAngle = round(driver.getChannelPWM(11))/2;

    if (Angle_B > CurrentAngle) 
    {
      for (i = CurrentAngle; i < Angle_B; i = i + 1) 
      {
        driver.setChannelPWM(11, pwmServo3.pwmForAngle(i));
        delay(5);
      }
    }

    else
    {
      for (i = CurrentAngle; i > Angle_B; i = i - 1) 
      {
        driver.setChannelPWM(11, pwmServo3.pwmForAngle(i));

        delay(5);
      }
    }
  } 


  if (!isnan(Angle_C)) 
  { 
      CurrentAngle = round(driver.getChannelPWM(12))/2;

    if (Angle_C > CurrentAngle) {

      for (i = CurrentAngle; i < Angle_C; i = i + 1) //if Angle_C > CurrentAngle.
      {
        driver.setChannelPWM(12, pwmServo4.pwmForAngle(i));
        delay(5);
      }

    }

    else
    {

      for (i = CurrentAngle; i > Angle_C; i = i - 1) //if Angle_C < CurrentAngle.
      {
        driver.setChannelPWM(12, pwmServo4.pwmForAngle(i));
        delay(5);
      }
    }
  } 
}

void Calc_Circle(float BYValue, float BZValue) 
{
  Hypot = sqrt(sq(BYValue) + sq(BZValue));
  A = ElbowLength;
  B = Hypot;
  C = ShoulderLength;


  Angle_B =  acos((sq(C) + sq(A) - sq(B)) / (2 * A * C)) * (180 / PI);
  Angle_C = acos((sq(A) + sq(B) - sq(C)) / (2 * A * B)) * (180 / PI);
  //Angle_C = 180 - Angle_C; //must calc larger opposite side angle.

  Angle_A = acos((sq(B) + sq(C) - sq(A)) / (2 * B * C)) * (180 / PI);

//  Angle_A = Angle_A * (1 + (Angle_A / (Angle_B + Angle_C))); // Angle A is recalculated due to the servo gearbox used.

  if (Angle_A < 0) {
    Angle_A = 35;
  }//End if < 0.


  if (!isnan(Angle_A)) 
  { 
    driver.setChannelPWM(10, pwmServo2.pwmForAngle(Angle_A));
    //ShoulderTilt.write(Angle_A);
    delay(1000);
  }

  if (!isnan(Angle_B)) 
  { 
      CurrentAngle = round(driver.getChannelPWM(11))/2;

    //CurrentAngle = ElbowTilt.read();

    if (Angle_B > CurrentAngle) {
      for (i = CurrentAngle; i < Angle_B; i = i + 1) //if Angle_B > CurrentAngle.
      {
        driver.setChannelPWM(11, pwmServo3.pwmForAngle(i));
        //ElbowTilt.write(i);
        delay(5);
      }//end for CurrentAngle.

    }//end if Angle_B > CurrentAngle.

    else
    {
      for (i = CurrentAngle; i > Angle_B; i = i - 1) 
      {
        //Tilt Elbow
        driver.setChannelPWM(11, pwmServo3.pwmForAngle(i));
        delay(5);
      }
    }

  } 



  if (!isnan(Angle_C)) { //If Angle is a valid number.
    
      CurrentAngle = round(driver.getChannelPWM(12))/2;

    
    //CurrentAngle = WristTilt.read();

    if (Angle_C > CurrentAngle) {
      for (i = CurrentAngle; i < Angle_C; i = i + 1) 
      {
        //Tilt Wrist
        driver.setChannelPWM(12, pwmServo4.pwmForAngle(i));
        delay(5);
      }

    }

    else
    {
      for (i = CurrentAngle; i > Angle_C; i = i - 1) 
      {
        driver.setChannelPWM(12, pwmServo4.pwmForAngle(i));
        delay(5);
      }
    }
  }     
}

void Calc_Point() 
{
  Slope = (z2 - z1) / (y2 - y1);
  A = 1 + sq(z1 / y1);
  B = (-2 * y1) + (-2 * z1 * Slope);
  C = sq(z1) + sq(y1) - sq(WristLength);

  //Quadratic Calculations
  NegYAnswer = ((-1 * B) - (sqrt(sq(B) - (4 * A * C)))) / (2 * A); 
  NegZAnswer = Slope * NegYAnswer;

  if (!isnan(NegYAnswer)) 
  {
    Calc_Circle(NegYAnswer, NegZAnswer);
  }

  else

    PosYAnswer = ((-1 * B) + (sqrt(sq(B) - (4 * A * C)))) / (2 * A); 
    PosZAnswer = Slope * PosYAnswer;

    if (!isnan(PosYAnswer)) 
    {
      Calc_Circle(PosYAnswer, PosZAnswer);
    }
  }
