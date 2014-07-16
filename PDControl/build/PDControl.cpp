#include <phys253.h>         //***** from 253 template file
#include <LiquidCrystal.h>   //***** from 253 template file
#include <servo253.h>       //***** from 253 template file


#define LEFTMOTOR_PIN 0
#define RIGHTMOTOR_PIN 1

#define LEFTQRD_PIN 4
#define RIGHTQRD_PIN 5


#include "WProgram.h"
#include <HardwareSerial.h>
void setup();
void loop();
int bothQRD_thresh = 300;
int leftQRD_curVal, rightQRD_curVal;

int error_curVal, error_lastVal = 0;
int error_timeInErrorState = 0, error_slopeChangeFromLastError;

int proportional_gain, derivative_gain;
int proportional_curVal, derivative_curVal;

int motorSpeed_offset, motorSpeed_base = 0;

int userSet;
int menuState = 1;
bool editable = false;
bool startButtonState = false, stopButtonState = false;

int timeTicker = 0;


void setup() {
    portMode(LEFTQRD_PIN, INPUT);
    portMode(RIGHTQRD_PIN, INPUT);
    
}

void loop() {

    // Read QRD values

    leftQRD_curVal = analogRead(LEFTQRD_PIN);
    rightQRD_curVal = analogRead(RIGHTQRD_PIN);


    // Get tape following status

    if (leftQRD_curVal > bothQRD_thresh && rightQRD_curVal > bothQRD_thresh) {
        error_curVal = 0;
    } else if (leftQRD_curVal > bothQRD_thresh && rightQRD_curVal < bothQRD_thresh) {
        error_curVal = -1;
    } else if (leftQRD_curVal < bothQRD_thresh && rightQRD_curVal > bothQRD_thresh) {
        error_curVal = +1;
    } else if (leftQRD_curVal < bothQRD_thresh && rightQRD_curVal < bothQRD_thresh) {
      if (error_lastVal > 0) {
          error_curVal = 5;
      } else {
          error_curVal = -5;
      }
    }


    // Derivative control handling

    if (error_curVal != error_lastVal) {
        error_slopeChangeFromLastError = error_timeInErrorState + 1;
        error_timeInErrorState = 0;
    } else {
        error_slopeChangeFromLastError = 0;
    }


    // Determine control values

    proportional_curVal = proportional_gain * error_curVal;
    derivative_curVal = (int)((float) derivative_gain * (float)(error_curVal - error_lastVal) / (float)(error_slopeChangeFromLastError + error_timeInErrorState));


    // Set necessary error adjustment

    motorSpeed_offset = proportional_curVal + derivative_curVal;


    // Only activate menu every 50 clock cycles

    if (timeTicker == 50) {
        timeTicker = 0;


        // Get button states

        if (startbutton() && !startButtonState) {
            if (menuState == 4) {
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
            // 1: Set/Get Derivative + Proportional Gain
            // 2: Set/Get QRD Threshold + Base Motor Speed
            // 4: Read QRD Values
            // 5: Read Motor Values

        switch (menuState) {
          
            case 1:
                LCD.print("Derv: ");
                userSet = (int)(knob(6) * 100.0/1024.0);
                if(userSet != derivative_gain && editable){
                    derivative_gain = userSet;
                }
                LCD.print(derivative_gain, DEC);

                LCD.setCursor(0,1);

                LCD.print("Prop: ");
                userSet = (int)(knob(7) * 100.0/1024.0);
                if(userSet != proportional_gain && editable){
                    proportional_gain = userSet;
                }
                LCD.print(proportional_gain, DEC);

                break;

            case 2:
                LCD.print("Thresh: ");
                userSet = (int)(knob(6) * 700.0/1024.0);
                if (userSet != bothQRD_thresh && editable){
                    bothQRD_thresh = userSet;
                }
                LCD.print(bothQRD_thresh, DEC);

                LCD.setCursor(0,1);

                LCD.print("Speed: ");
                userSet = (int)(knob(7) * 700.0/1024.0);
                if (userSet != motorSpeed_base && editable) {
                    motorSpeed_base = userSet;
                } 
                LCD.print(motorSpeed_base, DEC);
                break;

            case 3:
                LCD.print("L QRD: ");
                if (leftQRD_curVal > bothQRD_thresh) {
                  LCD.print(" ON ");
                } else {
                  LCD.print(" OFF ");
                }
                LCD.print(leftQRD_curVal, DEC);

                LCD.setCursor(0,1);

                LCD.print("R QRD: ");
                if (rightQRD_curVal > bothQRD_thresh) {
                  LCD.print(" ON ");
                } else {
                  LCD.print(" OFF ");
                }
                LCD.print(rightQRD_curVal, DEC);
                break;

            case 4:
                LCD.print("M Left: ");
                LCD.print(motorSpeed_base - motorSpeed_offset, DEC);
                LCD.setCursor(0,1);
                LCD.print("M Right: ");
                LCD.print(motorSpeed_base + motorSpeed_offset, DEC);
                break;

            default: 
                break;
            
        }
    }


    // Set motor speeds

    if (motorSpeed_offset < -700) {
        motor.speed(LEFTMOTOR_PIN, 700);
        motor.speed(RIGHTMOTOR_PIN, motorSpeed_base);
        LCD.clear();
        LCD.print("LIM-");
    } else if (motorSpeed_offset > 700) {
        motor.speed(LEFTMOTOR_PIN, motorSpeed_base);
        motor.speed(RIGHTMOTOR_PIN, 700);
        LCD.clear();
        LCD.print("LIM+");
    } else {
        motor.speed(LEFTMOTOR_PIN, motorSpeed_base - motorSpeed_offset);
        motor.speed(RIGHTMOTOR_PIN, motorSpeed_base + motorSpeed_offset);
    }


    // Iterations and clean up

    error_lastVal = error_curVal;
    timeTicker++;
    error_timeInErrorState++;
}

