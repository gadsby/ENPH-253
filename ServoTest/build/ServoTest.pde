#include <phys253.h>          //   ***** from 253 template file
#include <LiquidCrystal.h>    //   ***** from 253 template file
#include <Servo253.h>         //   ***** from 253 template file

#define MOTOR_NUM 0

void setup()
{
  portMode(0, OUTPUT) ;      //   ***** from 253 template file
  portMode(1, INPUT) ;      //   ***** from 253 template file
  RCServo0.attach(RCServo0Output) ;
  RCServo1.attach(RCServo1Output) ;
  RCServo2.attach(RCServo2Output) ;
  digitalWrite(0, HIGH);
 
}




void loop()
{
  int mspeed = powerForKnob();
  motor.speed(MOTOR_NUM, mspeed);
  int angle = floor(knob(7) * 180.0 / 1024.0);
  RCServo1.write(angle);
  LCD.clear();
  LCD.print(String(angle,DEC)); 
  delay(100);
}

int powerForKnob()
{
  int power = 2 * knob(6) - 1023;
  return power;
}
