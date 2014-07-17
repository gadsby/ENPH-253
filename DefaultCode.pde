#include <phys253.h>          //   ***** from 253 template file
#include <LiquidCrystal.h>    //   ***** from 253 template file
#include <Servo253.h>         //   ***** from 253 template file

void setup()
{
  portMode(0, OUTPUT);      //   ***** from 253 template file
  portMode(1, INPUT);      //   ***** from 253 template file
  RCServo0.attach(RCServo0Output);
  RCServo1.attach(RCServo1Output);
  RCServo2.attach(RCServo2Output);
 
}

void loop()
{

}