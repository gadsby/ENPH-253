#include <phys253.h>          //   ***** from 253 template file
#include <LiquidCrystal.h>    //   ***** from 253 template file
#include <Servo253.h>         //   ***** from 253 template file

#define COLLECTOR_SERVOPIN 1
#define COLLECTOR_SWITCHPIN 0

int angle_neutral = 30, angle_max = 180, angle_min = 0, timeTicker = 0;
bool collector_switchState = false;

int menuState = 1, userSet, collect_status = 1, testRange_status = 1;
bool editable = false, startButtonState = false, stopButtonState = false;


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
  RCServo1.write(angle_neutral);
  collector_switchState = LOW;


    if (timeTicker == 50) {
        timeTicker = 0; 

      if (startbutton() && !startButtonState) {
                if (menuState == 3) {
                    menuState = 1;
                } else {
                    menuState++;
                }
                editable = false;
                startButtonState = true;
                delay(500);
            } else if (stopbutton() && !stopButtonState) {
                editable = true;
                stopButtonState = true;
            } else {
                stopButtonState = false;
                startButtonState = false;
            }
            LCD.clear();    
    

            // Menu System
                // 1:   

            switch (menuState) {    

                case 1:
                    LCD.print("Rest: ");
                    userSet = (int)(knob(6) * 180.0/1024.0);
                    if (userSet != angle_neutral && editable){
                        angle_neutral = userSet;
                    }
                    LCD.print(angle_neutral, DEC);  

                    LCD.setCursor(0,1); 

                    LCD.print("Collect?: Y/N  ");
                    userSet = (int)(knob(7) * 2.0/1024.0);
                    if(userSet != collect_status && editable){
                        collect_status = userSet;
                    }
                    LCD.print(collect_status, DEC); 

                    break;
              
                case 2:
                    LCD.print("Min: ");
                    userSet = (int)(knob(6) * 180.0/1024.0);
                    if(userSet != angle_min && editable){
                        angle_min = userSet;
                    }
                    LCD.print(angle_min, DEC);  

                    LCD.setCursor(0,1); 

                    LCD.print("Max: ");
                    userSet = (int)(knob(7) * 180.0/1024.0);
                    if(userSet != angle_max && editable){
                        angle_max = userSet;
                    }
                    LCD.print(angle_max, DEC);  

                    break;

                case 3:
                    LCD.print("TestRange: ");
                    userSet = 1.5*(int)(knob(6) * 180.0/1024.0);
                    if(userSet != angle_neutral && editable){
                        if (userSet >= 180){
                            angle_neutral = 180;
                        } else{
                            angle_neutral = userSet;
                        }
                    }
                    LCD.print(angle_neutral, DEC);

                    break;
                }   

    }


    if(collect_status == 1){
        collector_switchState = digitalRead(COLLECTOR_SWITCHPIN);   

        if(collector_switchState){
            RCServo1.write(angle_min);
            delay(200);
            RCServo0.write(angle_max);
            delay(200);
            pusher_activate();
            RCServo1.write(angle_neutral);
            collector_switchState = false;
        }   

    }

    timeTicker++;

}

void pusher_activate(){

  // activate pusher
  delay(200);
  // retract pusher
  delay(200);
}
