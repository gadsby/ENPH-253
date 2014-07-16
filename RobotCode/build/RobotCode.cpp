#include <phys253.h>          //   ***** from 253 template file
#include <LiquidCrystal.h>    //   ***** from 253 template file
#include <Servo253.h>         //   ***** from 253 template file

// Main Menu
#define AUTONOMOUS_MODE 0
#define DEMO_MODE 1
#define ADJUST_PARAMETERS 2
#define LOOKUP_PINS 3


// Analog Pins (0-7)
#define LEFTQRD_PIN 4
#define RIGHTQRD_PIN 5
#define MENUKNOB_PIN 6
#define VALUEKNOB_PIN 7

// Digital Pins (0-15)

// Motor Pins (0-3)
#define LEFTMOTOR_PIN 0
#define RIGHTMOTOR_PIN 2

// Servo Pins (0-2)
#define COLLECTORSERVO_PIN 0
#define SWEEPERSERVO_PIN 1


// Adjustable Parameters
#include "WProgram.h"
#include <HardwareSerial.h>
void setup();
void loop();
void accessDemoList();
void adjustParameterVals();
void runAutonomousMode();
void lookUpPins();
void testHBridge();
void testTapeFollowing();
void testQRDs();
void testServos();
void testSwitches();
void testIR();
void testTelescopingArm();
void testZiplineTrigger();
int getMenuKnobPos(int numPos);
int getValueKnobPos(int numPos);
bool userMenuSelection();
bool enterButton();
bool backButton();
void cleanLCD();
void countdown();
int paramAdjust(int originalParam, int maxVal);
int bothQRD_thresh = 300;
int proportional_gain = 30;
int derivative_gain = 0;
int motorSpeed_base = 300;
int motorFast = 700;
int motorSlow = 350;

// Program Constants
int num_mainMenu = 4;
int refreshDelay = 50;
int menuKnobPos;


void setup(){
  portMode(0, OUTPUT);      //   ***** from 253 template file
  portMode(1, INPUT);      //   ***** from 253 template file
  RCServo0.attach(RCServo0Output);
  RCServo1.attach(RCServo1Output);
  RCServo2.attach(RCServo2Output);
}

void loop(){

    cleanLCD();
	menuKnobPos = getMenuKnobPos(num_mainMenu);	

	switch(menuKnobPos){

		case AUTONOMOUS_MODE:
			LCD.print("Full Mode?");	
			if(userMenuSelection()){
				runAutonomousMode();}	
			break;

		case DEMO_MODE:	
			LCD.print("Demo Mode?");	
			if(userMenuSelection()){
				accessDemoList();}	
			break;	

		case ADJUST_PARAMETERS:	
			LCD.print("Adjust Params?");	
			if(userMenuSelection()){
				adjustParameterVals();}
			break;	

		case LOOKUP_PINS:	
			LCD.print("Pinout Guide?");	
			if(userMenuSelection()){
					lookUpPins();}
			break;	

		default:
			break;
	}	

	delay(refreshDelay);
}


// COMPLETE
void accessDemoList(){
	// show demo options

	#define TEST_HBRIDGE 0
	#define TEST_TAPEFOLLOW 1
	#define TEST_SWITCHES 2
	#define TEST_SERVOS 3
	#define TEST_QRD 4
	#define TEST_IR 5
	#define TEST_TEL_ARM 6
	#define TEST_ZL_TRIGGER 7

	int num_demoMenu = 8;


	while(!backButton()){

		menuKnobPos = getMenuKnobPos(num_demoMenu);
		cleanLCD();

		switch(menuKnobPos){	

			case TEST_HBRIDGE:
				LCD.print("Test H-Bridge?");	
				if(userMenuSelection()){
					testHBridge();}	

				break;	

			case TEST_TAPEFOLLOW:	

				LCD.print("Test TapeFollow?");	
				if(userMenuSelection()){
					testTapeFollowing();}	

				break;	

			case TEST_SWITCHES:

				LCD.print("Test Switches?");	
				if(userMenuSelection()){
					testSwitches();}	

				break;	

			case TEST_SERVOS:	

				LCD.print("Test Servos?");	
				if(userMenuSelection()){
					testServos();}	

				break;

			case TEST_QRD:

				LCD.print("Test QRDs?");	
				if(userMenuSelection()){
					testQRDs();}	

				break;	

			case TEST_IR:

				LCD.print("Test IR Detect?");
				if(userMenuSelection()){
					testIR();}	

				break;

			case TEST_TEL_ARM:	

				LCD.print("Test Tel. Arm?");	
				if(userMenuSelection()){
					testTelescopingArm();}	

				break;		

			case TEST_ZL_TRIGGER:	

				LCD.print("Test ZL Trigger?");	
				if(userMenuSelection()){
					testZiplineTrigger();}	

				break;

			default:
				break;

		}

		delay(refreshDelay);
	}
}


// In Progress (NEXT)
void adjustParameterVals(){
	// eventually: write to EEPROM

	#define BOTH_QRD_THRESH 0
	#define PROP_GAIN 1
	#define DERV_GAIN 2
	#define MOTORSPEED_BASE 3
	#define MOTOR_FAST 4
	#define MOTOR_SLOW 5

	int bothQRDThresh_maxVal = 700;
	int proportionalGain_maxVal = 100;
	int derivativeGain_maxVal = 100;
	int motorSpeedBase_maxVal = 700;
	int motorFast_maxVal = 700;
	int motorSlow_maxVal = 700;

	int num_params = 6;

	/* To add new param:
		1) add new global variable
		2) add new #define
		3) add new maxVal
		4) increment num_params
		5) add this to switch block:
				case NEW_PARAM:
					LCD.print("New Param");
					if(userMenuSelection()){
						newParam = paramAdjust(newParam, newParam_maxVal);}
					break;
	*/

	while(!backButton()){

		menuKnobPos = getMenuKnobPos(num_params);
		cleanLCD();

		switch(menuKnobPos){	

			case BOTH_QRD_THRESH:
				LCD.print("QRD Threshold");
				if(userMenuSelection()){
					bothQRD_thresh = paramAdjust(bothQRD_thresh, bothQRDThresh_maxVal);}
				break;	

			case PROP_GAIN:	
				LCD.print("Prop Gain");
				if(userMenuSelection()){
					proportional_gain = paramAdjust(proportional_gain, proportionalGain_maxVal);}
				break;	

			case DERV_GAIN:
				LCD.print("Derv Gain");
				if(userMenuSelection()){
					derivative_gain = paramAdjust(derivative_gain, derivativeGain_maxVal);}
				break;	

			case MOTORSPEED_BASE:	
				LCD.print("Base MotSpeed");
				if(userMenuSelection()){
					motorSpeed_base = paramAdjust(motorSpeed_base, motorSpeedBase_maxVal);}
				break;

			case MOTOR_FAST:
				LCD.print("MotorFast Speed");
				if(userMenuSelection()){
					motorFast = paramAdjust(motorFast, motorFast_maxVal);}
				break;	

			case MOTOR_SLOW:
				LCD.print("MotorSlow Speed");
				if(userMenuSelection()){
					motorSlow = paramAdjust(motorSlow, motorSlow_maxVal);}
				break;

	/*
			case NEW_PARAM:
				LCD.print("New Param");
				if(userMenuSelection()){
					newParam = paramAdjust(newParam, newParam_maxVal);}
				break;
	*/

			default:
				break;

		}

		delay(refreshDelay);
	}
}


// LAST
void runAutonomousMode() {
	// run competetion code
	cleanLCD();
	while(!backButton()){
		LCD.home();
		LCD.print("PLACEHOLDER");
	}
}

// LATER
void lookUpPins(){
	// let pins be accessed
	cleanLCD();
	while(!backButton()){
		LCD.home();
		LCD.print("PLACEHOLDER");
	}
}




// Demo Programs

// Complete
void testHBridge(){

	#define SLOW_FORWARD 0
	#define SLOW_BACKWARD 1
	#define SLOW_LEFT 2
	#define SLOW_RIGHT 3
	#define FAST_FORWARD 4
	#define FAST_BACKWARD 5
	#define FAST_LEFT 6
	#define FAST_RIGHT 7

	int num_hBridgeStates = 8;

	countdown();

	while(!backButton()){

		menuKnobPos = getMenuKnobPos(num_hBridgeStates);

		switch(menuKnobPos){	

			case SLOW_FORWARD:
				cleanLCD();
				LCD.print("Forward SLOW");
				motor.speed(LEFTMOTOR_PIN, motorSlow);
				motor.speed(RIGHTMOTOR_PIN, motorSlow);
				break;	

			case SLOW_BACKWARD:	
				cleanLCD();
				LCD.print("Backward SLOW");
				motor.speed(LEFTMOTOR_PIN, -motorSlow);
				motor.speed(RIGHTMOTOR_PIN, -motorSlow);
				break;	

			case SLOW_LEFT:
				cleanLCD();
				LCD.print("Left SLOW");
				motor.speed(LEFTMOTOR_PIN, -motorSlow);
				motor.speed(RIGHTMOTOR_PIN, motorSlow);
				break;	

			case SLOW_RIGHT:	
				cleanLCD();
				LCD.print("Right SLOW");
				motor.speed(LEFTMOTOR_PIN, motorSlow);
				motor.speed(RIGHTMOTOR_PIN, -motorSlow);
				break;

			case FAST_FORWARD:
				cleanLCD();
				LCD.print("Forward FAST");
				motor.speed(LEFTMOTOR_PIN, motorFast);
				motor.speed(RIGHTMOTOR_PIN, motorFast);
				break;	

			case FAST_BACKWARD:	
				cleanLCD();
				LCD.print("Backward FAST");
				motor.speed(LEFTMOTOR_PIN, -motorFast);
				motor.speed(RIGHTMOTOR_PIN, -motorFast);
				break;	

			case FAST_LEFT:
				cleanLCD();
				LCD.print("Left FAST");
				motor.speed(LEFTMOTOR_PIN, -motorFast);
				motor.speed(RIGHTMOTOR_PIN, motorFast);
				break;	

			case FAST_RIGHT:	
				cleanLCD();
				LCD.print("Right FAST");
				motor.speed(LEFTMOTOR_PIN, motorFast);
				motor.speed(RIGHTMOTOR_PIN, -motorFast);
				break;

			default:
				break;
		}

		delay(refreshDelay);

	}

	motor.speed(LEFTMOTOR_PIN, 0);
	motor.speed(RIGHTMOTOR_PIN, 0);
}

// Complete
void testTapeFollowing(){

	cleanLCD();

	int leftQRD_curVal, rightQRD_curVal;	

	int error_curVal, error_lastVal = 0;
	int error_timeInErrorState = 0, error_slopeChangeFromLastError;	

	int proportional_gain, derivative_gain;
	int proportional_curVal, derivative_curVal;	

	int motorSpeed_offset;	

	int timeTicker = 0;

	countdown();

	while(!backButton()){

	    leftQRD_curVal = analogRead(LEFTQRD_PIN);
	    rightQRD_curVal = analogRead(RIGHTQRD_PIN);	

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

	    if (error_curVal != error_lastVal) {
	        error_slopeChangeFromLastError = error_timeInErrorState + 1;
	        error_timeInErrorState = 0;
	    } else {
	        error_slopeChangeFromLastError = 0;
	    }	

	    proportional_curVal = proportional_gain * error_curVal;
	    derivative_curVal = (int)((float) derivative_gain * (float)(error_curVal - error_lastVal) / (float)(error_slopeChangeFromLastError + error_timeInErrorState));	

	    motorSpeed_offset = proportional_curVal + derivative_curVal;	

	    if (motorSpeed_offset < -700) {
	        motor.speed(LEFTMOTOR_PIN, 700);
	        motor.speed(RIGHTMOTOR_PIN, motorSpeed_base);

	    } else if (motorSpeed_offset > 700) {
	        motor.speed(LEFTMOTOR_PIN, motorSpeed_base);
	        motor.speed(RIGHTMOTOR_PIN, 700);

	    } else {
	        motor.speed(LEFTMOTOR_PIN, motorSpeed_base - motorSpeed_offset);
	        motor.speed(RIGHTMOTOR_PIN, motorSpeed_base + motorSpeed_offset);
	    }

	    if (timeTicker == 100) {
	        timeTicker = 0;	

	        cleanLCD();

	        LCD.print("L MOT: ");
	        LCD.print(motorSpeed_base - motorSpeed_offset, DEC);	

	        LCD.setCursor(0,1);

	        LCD.print("R MOT: ");
	        LCD.print(motorSpeed_base + motorSpeed_offset, DEC);
	    }	

	    error_lastVal = error_curVal;
	    timeTicker++;
	    error_timeInErrorState++; 	

	}

	motor.speed(LEFTMOTOR_PIN, 0);
	motor.speed(RIGHTMOTOR_PIN, 0);	
}

// Complete
void testQRDs(){

	int leftQRD_curVal, rightQRD_curVal;

	while(!backButton()){

		cleanLCD();	

		leftQRD_curVal = analogRead(LEFTQRD_PIN);
		rightQRD_curVal = analogRead(RIGHTQRD_PIN);			

		LCD.print("L QRD: ");
		if (leftQRD_curVal > bothQRD_thresh) {
		  	LCD.print(" ON ");
		} else {
		    LCD.print("OFF ");
		}
		LCD.print(leftQRD_curVal, DEC);		

		LCD.setCursor(0,1);	

		LCD.print("R QRD: ");
		if (rightQRD_curVal > bothQRD_thresh) {
			LCD.print(" ON ");
		} else {
		   	LCD.print("OFF ");
		}
		LCD.print(rightQRD_curVal, DEC);	

		delay(refreshDelay);

	}	
}


// In Progress (NEXT)
void testServos(){

	// scroll through servos
	// change their values independently
	// use 1.1x multiplier

	cleanLCD();
	while(!backButton()){
		LCD.home();
		LCD.print("SERVOS");
		LCD.setCursor(0,1);
		LCD.print("In Progress...");  		
	}	
}


// In Progress (LATER)
void testSwitches(){
	cleanLCD();
	while(!backButton()){
		LCD.home();
		LCD.print("SWITCHES");
		LCD.setCursor(0,1);
		LCD.print("In Progress...");  		
	}	
}

// In Progress(LATER)
void testIR(){
	cleanLCD();
	while(!backButton()){
		LCD.home();
		LCD.print("IR");
		LCD.setCursor(0,1);
		LCD.print("In Progress...");  		
	}	
}

// In Progress (LATER)
void testTelescopingArm(){
	cleanLCD();
	while(!backButton()){
		LCD.home();
		LCD.print("TELESCOPE ARM");
		LCD.setCursor(0,1);
		LCD.print("In Progress...");  		
	}	
}

// In Progress (LATER)
void testZiplineTrigger(){
	cleanLCD();
	while(!backButton()){
		LCD.home();
		LCD.print("ZL Trigger");
		LCD.setCursor(0,1);
		LCD.print("In Progress...");  		
	}	
}


// Abstracted Helper Functions

int getMenuKnobPos(int numPos) {
	int userSet = (int)(1.1*knob(MENUKNOB_PIN) * ((float)numPos)/1024.0);
	if (userSet >= numPos){
		return numPos-1;}
	return userSet;
}

int getValueKnobPos(int numPos) {
	int userSet = (int)(1.1*knob(VALUEKNOB_PIN) * ((float)numPos)/1024.0);
		if (userSet >= numPos){
		return numPos-1;}
	return userSet;
}

bool userMenuSelection(){

	int valueKnobPos = getValueKnobPos(2);

	// Necessary?
	LCD.setCursor(0,1);
	if (valueKnobPos == 1){
		LCD.print("Yes");  
	} else {
		LCD.print("No");
	}
	// END BLOCK

	if (valueKnobPos == 1 && enterButton()){
		return true;
	}
	return false;
}

bool enterButton() {
	// get state of enter button

	bool enterButtonState = false;
	int debounceDelay = 500;

	if (startbutton() && !enterButtonState) {    
		delay(debounceDelay);       
        return true;
    }  else {
        return false;
    }
}

bool backButton() {
	// get state of back button

	bool backButtonState = false;
	int debounceDelay = 500;

	if (stopbutton() && !backButtonState) {
		delay(debounceDelay);
        return true;
    } else {
        return false;
    }
}

void cleanLCD() {
	LCD.clear();
	LCD.home();
}

void countdown(){
	cleanLCD();
	LCD.print("Movement Begins in");

	LCD.setCursor(0,1);
	LCD.print("3...");
	delay(1000);

	LCD.setCursor(0,1);
	LCD.print("2...");
	delay(1000);

	LCD.setCursor(0,1);
	LCD.print("1...");
	delay(1000);

	cleanLCD();
}

int paramAdjust(int originalParam, int maxVal){

	int tempParam;
	int newParam;
	int enterDelay = 2000;

	while(!backButton()){
		LCD.setCursor(0,1);
		LCD.print("                ");
		LCD.setCursor(0,1);

		tempParam = getValueKnobPos(maxVal);

		LCD.print(tempParam, DEC);
		LCD.print(" Cur: ");
		LCD.print(originalParam, DEC);

		if(enterButton()){
			newParam = tempParam;
			LCD.setCursor(0,1);
			LCD.print("                ");
			LCD.setCursor(0,1);
			LCD.print("New: ");
			LCD.print(newParam, DEC);
			delay(enterDelay);
			return newParam;
		}

		delay(refreshDelay);
	}

	return originalParam;
}









