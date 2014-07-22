#include <phys253.h>          //   ***** from 253 template file
#include <LiquidCrystal.h>    //   ***** from 253 template file
#include <Servo253.h>         //   ***** from 253 template file
#include <EEPROM.h>


// Main Menu
#define AUTONOMOUS_MODE 0
#define DEMO_MODE 1
#define ADJUST_PARAMETERS 2
#define LOOKUP_PINS 3
#define NUM_MAINMENU 4

// Analog Pins (0-7)
#define LEFTIR_PIN 1
#define RIGHTIR_PIN 2
#define CENTREQRD_PIN 3
#define LEFTQRD_PIN 4
#define RIGHTQRD_PIN 5
#define MENUKNOB_PIN 6
#define VALUEKNOB_PIN 7

// Digital Pins (0-7 Out, 8-15 In)
#define SONAROUT_PIN 7
#define SONARIN_PIN 8
#define COLLECTIONSWITCH_PIN 9

// Motor Pins (0-3)
#define LEFTMOTOR_PIN 0
#define CONVEYORMOTOR_PIN 1
#define RIGHTMOTOR_PIN 2

// Servo Pins (0-2)
#define COLLECTORSERVO_PIN 0
#define SWEEPERSERVO_PIN 1

// Parameter EEPROM Address
#define LEFTQRDTHRESH_EEPROM 0
#define RIGHTQRDTHRESH_EEPROM 1
#define PROPGAIN_EEPROM 2
#define DERVGAIN_EEPROM 3
#define MOTORSPEEDBASE_EEPROM 4
#define MOTORSPEEDOFFSET_EEPROM 5
#define MOTORFAST_EEPROM 6
#define MOTORSLOW_EEPROM 7
#define SLIGHTADJUST_EEPROM 8
#define LOTSADJUST_EEPROM 9
#define LARGEERRORMULTIPLIER_EEPROM 10
#define SERVODELAYTIME_EEPROM 11
#define SERVOTESTDEFAULTANGLE_EEPROM 12
#define CENTREQRDTHRESH_EEPROM 13
#define COLLECTORDEFAULTANGLE_EEPROM 14
#define SWEEPERDEFAULTANGLE_EEPROM 15
#define HILLSONARTHRESH_EEPROM 16
#define EDGESONARTHRESH_EEPROM 17
#define MOTORSPEEDOFFSETSIGN_EEPROM 18


// Adjustable Parameters
int leftQRD_thresh = EEPROM.read(LEFTQRDTHRESH_EEPROM) * 5;
int rightQRD_thresh = EEPROM.read(RIGHTQRDTHRESH_EEPROM) * 5;
int proportional_gain = EEPROM.read(PROPGAIN_EEPROM) * 5;
int derivative_gain = EEPROM.read(DERVGAIN_EEPROM) * 5;
int motorSpeed_base = EEPROM.read(MOTORSPEEDBASE_EEPROM) * 5;
int motorSpeed_offset = EEPROM.read(MOTORSPEEDOFFSET_EEPROM) * 5;
int motorFast = EEPROM.read(MOTORFAST_EEPROM) * 5;
int motorSlow = EEPROM.read(MOTORSLOW_EEPROM) * 5;
int slightAdjust = EEPROM.read(SLIGHTADJUST_EEPROM) * 5;
int lotsAdjust = EEPROM.read(LOTSADJUST_EEPROM) * 5;
int largeErrorMultiplier = EEPROM.read(LARGEERRORMULTIPLIER_EEPROM);	// set to 5 in autonomous code
int servoDelayTime = EEPROM.read(SERVODELAYTIME_EEPROM);
int servoDefaultAngle = EEPROM.read(SERVOTESTDEFAULTANGLE_EEPROM) * 5;
int centreQRD_thresh = EEPROM.read(CENTREQRDTHRESH_EEPROM) * 5;
int collectorDefaultAngle = EEPROM.read(COLLECTORDEFAULTANGLE_EEPROM);
int sweeperDefaultAngle = EEPROM.read(SWEEPERDEFAULTANGLE_EEPROM);
int hillSonarThresh = EEPROM.read(HILLSONARTHRESH_EEPROM);
int edgeSonarThresh = EEPROM.read(EDGESONARTHRESH_EEPROM);


// Program Constants
int menuKnobPos;
static int const refreshDelay = 50;




void setup(){
  portMode(0, OUTPUT);     //   ***** from 253 template file
  portMode(1, INPUT);      //   ***** from 253 template file
  RCServo0.attach(RCServo0Output);
  RCServo1.attach(RCServo1Output);
  RCServo2.attach(RCServo2Output);
  // set servos to upright here

	if(EEPROM.read(MOTORSPEEDOFFSETSIGN_EEPROM) == 1){
		int motorSpeed_offset = EEPROM.read(MOTORSPEEDOFFSET_EEPROM) * 5;}
	else{
		int motorSpeed_offset = EEPROM.read(MOTORSPEEDOFFSET_EEPROM) * -5;}
}

void loop(){

    cleanLCD();
	menuKnobPos = getMenuKnobPos(NUM_MAINMENU);	

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
	#define TEST_IDOLGRAB 8
	#define NEXT_PAGE 9
	static int const NUM_DEMOMENU = 10;


	while(!backButton()){

		menuKnobPos = getMenuKnobPos(NUM_DEMOMENU);
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

			case TEST_IDOLGRAB:	
				LCD.print("Test IdolGrab?");	
				if(userMenuSelection()){
					testIdolGrab();}	
				break;

			case NEXT_PAGE:
				LCD.print("< PAGE 2 >");
				if(userMenuSelection()){
					accessDemoList_page2();}
				break;

			default:
				break;

		}

		delay(refreshDelay);
	}
}

// COMPLETE
void accessDemoList_page2(){
	#define TEST_FORWARDDRIVING 0
	#define TEST_SONARRANGE 1
	#define TEST_CONVEYOR 2
	static int const NUM_DEMOMENU = 3;
	#define NEXT_PAGE 9

	while(!backButton()){

		menuKnobPos = getMenuKnobPos(NUM_DEMOMENU);
		cleanLCD();

		switch(menuKnobPos){	

			case TEST_FORWARDDRIVING:	
				LCD.print("Test FrwdDrive?");	
				if(userMenuSelection()){
					testFrwdDrive();}	
				break;

			case TEST_SONARRANGE:	
				LCD.print("Test Sonar?");	
				if(userMenuSelection()){
					testSonar();}	
				break;

			case TEST_CONVEYOR:	
				LCD.print("Test Conveyor?");	
				if(userMenuSelection()){
					testConveyor();}	
				break;

			case NEXT_PAGE:
				LCD.print("< PAGE 3 >");
				if(userMenuSelection()){
					// adjustDemoList_page3();
				} break;
		}

		delay(refreshDelay);
	}
}

// COMPLETE
void adjustParameterVals(){

	#define LEFT_QRD_THRESH 0
	#define RIGHT_QRD_THRESH 1
	#define PROP_GAIN 2
	#define DERV_GAIN 3
	#define MOTORSPEED_BASE 4
	#define MOTORSPEED_OFFSET 5
	#define MOTOR_FAST 6
	#define MOTOR_SLOW 7
	#define SLIGHT_ADJUST 8
	#define NEXT_PAGE 9
	static int const NUM_PARAMS = 10;

	int leftQRDThresh_maxVal = 700;
	int rightQRDThresh_maxVal = 700;
	int proportionalGain_maxVal = 100;
	int derivativeGain_maxVal = 100;
	int motorSpeedBase_maxVal = 700;
	int motorSpeedOffset_maxVal = 350;
	int motorFast_maxVal = 700;
	int motorSlow_maxVal = 700;
	int slightAdjust_maxVal = 300;

	int writeVal;

	while(!backButton()){

		menuKnobPos = getMenuKnobPos(NUM_PARAMS);
		cleanLCD();

		switch(menuKnobPos){	

			case LEFT_QRD_THRESH:
				LCD.print("L QRD Thresh");
				if(userMenuSelection()){
					EEPROM.write(LEFTQRDTHRESH_EEPROM, (int)((float)paramAdjust(leftQRD_thresh, leftQRDThresh_maxVal)/5.0));
					leftQRD_thresh = EEPROM.read(LEFTQRDTHRESH_EEPROM) * 5;
				}
				break;	

			case RIGHT_QRD_THRESH:
				LCD.print("R QRD Thresh");
				if(userMenuSelection()){
					EEPROM.write(RIGHTQRDTHRESH_EEPROM, (int)((float)paramAdjust(rightQRD_thresh, rightQRDThresh_maxVal)/5.0));
					rightQRD_thresh = EEPROM.read(RIGHTQRDTHRESH_EEPROM) * 5;
				}
				break;	

			case PROP_GAIN:	
				LCD.print("Prop Gain");
				if(userMenuSelection()){
					EEPROM.write(PROPGAIN_EEPROM, (int)((float)paramAdjust(proportional_gain, proportionalGain_maxVal)/5.0));
					proportional_gain = EEPROM.read(PROPGAIN_EEPROM) * 5;
				}
				break;	

			case DERV_GAIN:
				LCD.print("Derv Gain");
				if(userMenuSelection()){
					EEPROM.write(DERVGAIN_EEPROM, (int)((float)paramAdjust(derivative_gain, derivativeGain_maxVal)/5.0));
					derivative_gain = EEPROM.read(DERVGAIN_EEPROM) * 5;
				}
				break;	

			case MOTORSPEED_BASE:	
				LCD.print("Base MotSpeed");
				if(userMenuSelection()){
					EEPROM.write(MOTORSPEEDBASE_EEPROM, (int)((float)paramAdjust(motorSpeed_base, motorSpeedBase_maxVal)/5.0));
					motorSpeed_base = EEPROM.read(MOTORSPEEDBASE_EEPROM) * 5;
				}
				break;

			case MOTORSPEED_OFFSET:	
				LCD.print("Offset MotSpeed");
				if(userMenuSelection()){
					writeVal = (int)((float)paramAdjustNegVals(motorSpeed_offset, motorSpeedOffset_maxVal)/5.0);
					if(writeVal >= 0){
						EEPROM.write(MOTORSPEEDOFFSET_EEPROM, writeVal);
						EEPROM.write(MOTORSPEEDOFFSETSIGN_EEPROM, 1);
						motorSpeed_offset = EEPROM.read(MOTORSPEEDOFFSET_EEPROM) * 5;
					} else{
						EEPROM.write(MOTORSPEEDOFFSET_EEPROM, writeVal);
						EEPROM.write(MOTORSPEEDOFFSETSIGN_EEPROM, 0);
						motorSpeed_offset = EEPROM.read(MOTORSPEEDOFFSET_EEPROM) * -5;
					}
				}

				break;

			case MOTOR_FAST:
				LCD.print("MotorFast Speed");
				if(userMenuSelection()){
					EEPROM.write(MOTORFAST_EEPROM, (int)((float)paramAdjust(motorFast, motorFast_maxVal)/5.0));
					motorFast = EEPROM.read(MOTORFAST_EEPROM) * 5;
				}
				break;	

			case MOTOR_SLOW:
				LCD.print("MotorSlow Speed");
				if(userMenuSelection()){
					EEPROM.write(MOTORSLOW_EEPROM, (int)((float)paramAdjust(motorSlow, motorSlow_maxVal)/5.0));
					motorSlow = EEPROM.read(MOTORSLOW_EEPROM) * 5;
				}
				break;

			case SLIGHT_ADJUST:
				LCD.print("Slight-Straight");
				if(userMenuSelection()){
					EEPROM.write(SLIGHTADJUST_EEPROM, (int)((float)paramAdjust(slightAdjust, slightAdjust_maxVal)/5.0));
					slightAdjust = EEPROM.read(SLIGHTADJUST_EEPROM) * 5;
				}
				break;

			case NEXT_PAGE:
				LCD.print("< PAGE 2 >");
				if(userMenuSelection()){
					adjustParameterVals_page2();}
				break;

			default:
				break;

		}

		delay(refreshDelay);
	}
}

// COMPLETE
void adjustParameterVals_page2(){

	#define LOTS_ADJUST 0
	#define LARGEERROR_MULTIPLIER 1
	#define SERVO_DELAYTIME 2
	#define SERVO_DEFAULTANGLE 3
	#define CENTRE_QRD 4
	#define COLLECTOR_DEFAULTANGLE 5
	#define SWEEPER_DEFAULTANGLE 6
	#define HILL_SONARTHRESH 7
	#define EDGE_SONARTHRESH 8
	#define NEXT_PAGE 9
	static int const NUM_PARAMS = 10;

	int lotsAdjust_maxVal = 500;
	int largeErrorMultiplier_maxVal = 10;
	int servoDelayTime_maxVal = 26;
	int servoDefaultAngle_maxVal = 185;
	int centreQRDThresh_maxVal = 700;
	int collectorDefaultAngle_maxVal = 180;
	int sweeperDefaultAngle_maxVal = 180;
	int hillSonarThresh_maxVal = 120;
	int edgeSonarThresh_maxVal = 120;


	while(!backButton()){

		menuKnobPos = getMenuKnobPos(NUM_PARAMS);
		cleanLCD();

		switch(menuKnobPos){	

			case LOTS_ADJUST:
				LCD.print("Lots-Straight");
				if(userMenuSelection()){
					EEPROM.write(LOTSADJUST_EEPROM, (int)((float)paramAdjust(lotsAdjust, lotsAdjust_maxVal)/5.0));
					lotsAdjust = EEPROM.read(LOTSADJUST_EEPROM) * 5;
				}
				break;

			case LARGEERROR_MULTIPLIER:
				LCD.print("Large Error Mult");
				if(userMenuSelection()){
					EEPROM.write(LARGEERRORMULTIPLIER_EEPROM, paramAdjustFullRange(largeErrorMultiplier, largeErrorMultiplier_maxVal));
					largeErrorMultiplier = EEPROM.read(LARGEERRORMULTIPLIER_EEPROM);
				}
				break;

			case SERVO_DELAYTIME:
				LCD.print("Servo Delay");
				if(userMenuSelection()){
					EEPROM.write(SERVODELAYTIME_EEPROM, paramAdjustFullRange(servoDelayTime, servoDelayTime_maxVal));
					servoDelayTime = EEPROM.read(SERVODELAYTIME_EEPROM);
				}
				break;

			case SERVO_DEFAULTANGLE:
				LCD.print("Servo DefaultPos");
				if(userMenuSelection()){
					EEPROM.write(SERVOTESTDEFAULTANGLE_EEPROM, (int)((float)paramAdjust(servoDefaultAngle, servoDefaultAngle_maxVal)/5.0));
					servoDefaultAngle = EEPROM.read(SERVOTESTDEFAULTANGLE_EEPROM) * 5;
				}
				break;

			case CENTRE_QRD:
				LCD.print("C QRD Thresh");
				if(userMenuSelection()){
					EEPROM.write(CENTREQRDTHRESH_EEPROM, (int)((float)paramAdjust(centreQRD_thresh, centreQRDThresh_maxVal)/5.0));
					centreQRD_thresh = EEPROM.read(CENTREQRDTHRESH_EEPROM) * 5;
				}
				break;

			case COLLECTOR_DEFAULTANGLE:
				LCD.print("Coll. DefaultPos");
				if(userMenuSelection()){
					EEPROM.write(COLLECTORDEFAULTANGLE_EEPROM, paramAdjustFullRange(collectorDefaultAngle, collectorDefaultAngle_maxVal));
					collectorDefaultAngle = EEPROM.read(COLLECTORDEFAULTANGLE_EEPROM);
				}
				break;

			case SWEEPER_DEFAULTANGLE:
				LCD.print("Sweep DefaultPos");
				if(userMenuSelection()){
					EEPROM.write(SWEEPERDEFAULTANGLE_EEPROM, paramAdjustFullRange(sweeperDefaultAngle, sweeperDefaultAngle_maxVal));
					sweeperDefaultAngle = EEPROM.read(SWEEPERDEFAULTANGLE_EEPROM);
				}
				break;

			case HILL_SONARTHRESH:
				LCD.print("Hill SonarVal");
				if(userMenuSelection()){
					EEPROM.write(HILLSONARTHRESH_EEPROM, paramAdjustFullRange(hillSonarThresh, hillSonarThresh_maxVal));
					hillSonarThresh = EEPROM.read(HILLSONARTHRESH_EEPROM);
				}
				break;

			case EDGE_SONARTHRESH:
				LCD.print("Edge SonarVal");
				if(userMenuSelection()){
					EEPROM.write(EDGESONARTHRESH_EEPROM, paramAdjustFullRange(edgeSonarThresh, edgeSonarThresh_maxVal));
					edgeSonarThresh = EEPROM.read(EDGESONARTHRESH_EEPROM);
				}
				break;

			case NEXT_PAGE:
				LCD.print("< PAGE 3 >");
				if(userMenuSelection()){
					adjustParameterVals_page3();
				}
				break;
		}

		delay(refreshDelay);

	}
}

// COMPLETE
void adjustParameterVals_page3(){
	#define INITIALIZE_PARAMS 0
	static int const NUM_PARAMS = 1;
	#define NEXT_PAGE 9

		/* To add new param:
		1) add new EEPROM address
		2) update initEEPROM function
		3) add new global variable
		4) add new #define in this block
		5) add new maxVal
		6) increment num_params
		7) add this to switch block:
				case NEW_PARAM:
					LCD.print("New Param");
					if(userMenuSelection()){
						EEPROM.write(NEWPARAM_EEPROM, (int)((float)paramAdjust(newParam, newParam_maxVal)/5.0));}
						newParam = EEPROM.read(NEWPARAM_EEPROM) * 5;
					break;
		8) upload code
		9) run initEEPROM

		Note: Getting the 4 storage values speeds things up
			ex. #define SERVOTESTDEFAULTANGLE_EEPROM 12
				#define SERVO_DEFAULTANGLE 3
				int servoDefaultAngle = EEPROM.read(SERVOTESTDEFAULTANGLE_EEPROM) * 5;
				int servoDefaultAngle_maxVal = 180;
	*/

	while(!backButton()){

		menuKnobPos = getMenuKnobPos(NUM_PARAMS);
		cleanLCD();

		switch(menuKnobPos){	

			case INITIALIZE_PARAMS:
				LCD.print("Init EEPROM");
				if(userMenuSelection()){
					initEEPROM();
				}
				break;

			case NEXT_PAGE:
				LCD.print("< PAGE 4 >");
				if(userMenuSelection()){
					// adjustParameterVals_page3();
				}break;
		}

		delay(refreshDelay);

	}
}

// COMPLETE
void lookUpPins(){
	// let pins be accessed

	#define ANALOG_LIST 0
	#define DIGITAL_LIST 1
	#define MOTOR_LIST 2
	#define SERVO_LIST 3
	static int const NUM_PINTYPES = 4;

	while(!backButton()){

		cleanLCD();
		menuKnobPos = getMenuKnobPos(NUM_PINTYPES);

		switch(menuKnobPos){	

			case ANALOG_LIST:
				LCD.print("Analog?");
				if(userMenuSelection()){
					analogList();}
				break;	

			case DIGITAL_LIST:
				LCD.print("Digital?");
				if(userMenuSelection()){
					digitalList();}
				break;	

			case MOTOR_LIST:
				LCD.print("Motor?");
				if(userMenuSelection()){
					motorList();}
				break;	
			case SERVO_LIST:
				LCD.print("Servo?");
				if(userMenuSelection()){
					servoList();}
				break;	

			default:
				break;

		}

		delay(refreshDelay);
	}
}

// In Progress
void runAutonomousMode() {

	tapeAndIdols();
	// IRFollow();
	// lastIdol();
	// acquireZipline();
	// ride2Freedom();
}




// Lookup Tables

// Complete
void analogList(){

	#define PAGE_1 0
	#define PAGE_2 1
	#define PAGE_3 2
	#define PAGE_4 3
	static int const NUM_CASES = 4;

	while(!backButton()){

		cleanLCD();
		menuKnobPos = getMenuKnobPos(NUM_CASES);

		switch(menuKnobPos){	

			case PAGE_1:
				LCD.print("A0: < Unused >");
				LCD.setCursor(0,1);
				LCD.print("A1: < Unused >");
				break;	

			case PAGE_2:
				LCD.print("A2: < Unused >");
				LCD.setCursor(0,1);
				LCD.print("A3: QRD Centre");
				break;	

			case PAGE_3:
				LCD.print("A4: QRD Left");
				LCD.setCursor(0,1);
				LCD.print("A5: QRD Right");
				break;	

			case PAGE_4:
				LCD.print("A6: Menu Knob");
				LCD.setCursor(0,1);
				LCD.print("A7: Value Knob");
				break;	

			default:
				break;

		}

		delay(refreshDelay);
	}
}

// Complete
void digitalList(){

	#define PAGE_1 0
	#define PAGE_2 1
	#define PAGE_3 2
	#define PAGE_4 3
	#define PAGE_5 4
	#define PAGE_6 5
	#define PAGE_7 6
	#define PAGE_8 7
	static int const NUM_CASES = 8;

	while(!backButton()){

		cleanLCD();
		menuKnobPos = getMenuKnobPos(NUM_CASES);

		switch(menuKnobPos){	

			case PAGE_1:
				LCD.print("D0: < Unused >");
				LCD.setCursor(0,1);
				LCD.print("D1: < Unused >");
				break;	

			case PAGE_2:
				LCD.print("D2: < Unused >");
				LCD.setCursor(0,1);
				LCD.print("D3: < Unused >");
				break;	

			case PAGE_3:
				LCD.print("D4: < Unused >");
				LCD.setCursor(0,1);
				LCD.print("D5: < Unused >");
				break;	

			case PAGE_4:
				LCD.print("D6: < Unused >");
				LCD.setCursor(0,1);
				LCD.print("D7: Sonar Out");
				break;	

			case PAGE_5:
				LCD.print("D8: Sonar In");
				LCD.setCursor(0,1);
				LCD.print("D9: IdolCollect");
				break;	

			case PAGE_6:
				LCD.print("D10: < Unused >");
				LCD.setCursor(0,1);
				LCD.print("D11: < Unused >");
				break;	

			case PAGE_7:
				LCD.print("D12: < Unused >");
				LCD.setCursor(0,1);
				LCD.print("D13: < Unused >");
				break;	

			case PAGE_8:
				LCD.print("D14: < Unused >");
				LCD.setCursor(0,1);
				LCD.print("D15: < Unused >");
				break;	

			default:
				break;

		}

		delay(refreshDelay);
	}
}

// Complete
void motorList(){

	#define PAGE_1 0
	#define PAGE_2 1
	static int const NUM_CASES = 2;

	while(!backButton()){

		cleanLCD();
		menuKnobPos = getMenuKnobPos(NUM_CASES);

		switch(menuKnobPos){	

			case PAGE_1:
				LCD.print("M0: Motor Left");
				LCD.setCursor(0,1);
				LCD.print("M1: Conveyor");
				break;	

			case PAGE_2:
				LCD.print("M2: Motor Right");
				LCD.setCursor(0,1);
				LCD.print("M3: < Unused >");
				break;	

			default:
				break;

		}

		delay(refreshDelay);
	}
}

// Complete
void servoList(){

	#define PAGE_1 0
	#define PAGE_2 1
	static int const NUM_CASES = 2;

	while(!backButton()){

		cleanLCD();
		menuKnobPos = getMenuKnobPos(NUM_CASES);

		switch(menuKnobPos){	

			case PAGE_1:
				LCD.print("S0: Collect Arm");
				LCD.setCursor(0,1);
				LCD.print("S1: Sweeper");
				break;	

			case PAGE_2:
				LCD.print("S2: < Unused >");
				break;	

			default:
				break;

		}

		delay(refreshDelay);
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
	static int const NUM_HBRIDGESTATES = 8;

	countdown();

	while(!backButton()){

		menuKnobPos = getMenuKnobPos(NUM_HBRIDGESTATES);

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

	hitBrakes();
}

// Complete
void testTapeFollowing(){

	#define ON 1
	#define OFF 0

	cleanLCD();

	static int const LEFT = -1;
	static int const RIGHT = 1;

	int smallErrorMultiplier = 1;
	int midErrorMultiplier = 3;
	// largeErrorMultiplier is adjustable

	int leftQRD_curVal, rightQRD_curVal, centreQRD_curVal;	

	int error_curVal, error_lastVal = 0;
	int error_timeInErrorState = 0, error_slopeChangeFromLastError;	

	int proportional_curVal, derivative_curVal;	

	int motorSpeed_errorOffset;	

	int timeTicker = 0;

	countdown();

	while(!backButton()){

	    leftQRD_curVal = analogRead(LEFTQRD_PIN);
	    rightQRD_curVal = analogRead(RIGHTQRD_PIN);
	    centreQRD_curVal = analogRead(CENTREQRD_PIN);	


		if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = 0;		

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = 0;		

		} else if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = LEFT * smallErrorMultiplier;		

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = RIGHT * smallErrorMultiplier;		

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal < centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = RIGHT * midErrorMultiplier;		

		} else if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal < centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = LEFT * midErrorMultiplier;
		    
		} else if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal < centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		  if (error_lastVal > 0) {
		     	error_curVal = RIGHT * largeErrorMultiplier;		

		  } else if (error_lastVal < 0) {
		     	error_curVal = LEFT * largeErrorMultiplier;
		  } else {
		  		error_curVal = 0;
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

	    motorSpeed_errorOffset = proportional_curVal + derivative_curVal;	

	    if (motorSpeed_offset < -700) {
	        motor.speed(LEFTMOTOR_PIN, 700);
	        motor.speed(RIGHTMOTOR_PIN, motorSpeed_base - motorSpeed_offset);

	    } else if (motorSpeed_offset > 700) {
	        motor.speed(LEFTMOTOR_PIN, motorSpeed_base + motorSpeed_offset);
	        motor.speed(RIGHTMOTOR_PIN, 700);

	    } else {
	        motor.speed(LEFTMOTOR_PIN, motorSpeed_base + motorSpeed_offset - motorSpeed_errorOffset);
	        motor.speed(RIGHTMOTOR_PIN, motorSpeed_base - motorSpeed_offset + motorSpeed_errorOffset);
	    }

	    if (timeTicker == 100) {
	        timeTicker = 0;	

	        cleanLCD();

	        LCD.print("L MOT: ");
	        LCD.print(motorSpeed_base + motorSpeed_offset - motorSpeed_errorOffset, DEC);	

	        LCD.setCursor(0,1);

	        LCD.print("R MOT: ");
	        LCD.print(motorSpeed_base - motorSpeed_offset + motorSpeed_errorOffset, DEC);
	    }

	    error_lastVal = error_curVal;
	    timeTicker++;
	    error_timeInErrorState++; 	

	}

	hitBrakes();
}

// Complete
void testQRDs(){

	#define ALL_QRDS 0
	#define LEFT_QRD 1
	#define RIGHT_QRD 2
	#define CENTRE_QRD 3
	static int const NUM_QRDSTATES = 4;

	int leftQRD_curVal, rightQRD_curVal, centreQRD_curVal;


	while(!backButton()){

		cleanLCD();
		menuKnobPos = getMenuKnobPos(NUM_QRDSTATES);

		leftQRD_curVal = analogRead(LEFTQRD_PIN);
		rightQRD_curVal = analogRead(RIGHTQRD_PIN);			
		centreQRD_curVal = analogRead(CENTREQRD_PIN);

		switch(menuKnobPos){	

			case ALL_QRDS:
				LCD.print("L");
				if (leftQRD_curVal >= leftQRD_thresh) {
				  	LCD.print("ON");
				} else {
				    LCD.print("OFF");}
				LCD.print(leftQRD_curVal, DEC);

				LCD.print(" R");
				if (rightQRD_curVal >= rightQRD_thresh) {
				  	LCD.print("ON");
				} else {
				    LCD.print("OFF");}
				LCD.print(rightQRD_curVal, DEC);

				LCD.setCursor(0,1);	
				LCD.print("C");
				if (centreQRD_curVal >= centreQRD_thresh) {
				  	LCD.print("ON");
				} else {
				    LCD.print("OFF");}
				LCD.print(centreQRD_curVal, DEC);

				break;	

			case LEFT_QRD:
				LCD.print("L QRD: ");
				if (leftQRD_curVal >= leftQRD_thresh) {
				  	LCD.print(" ON ");
				} else {
				    LCD.print("OFF ");
				}
				LCD.setCursor(0,1);	
				LCD.print(leftQRD_curVal, DEC);	

				break;	

			case RIGHT_QRD:
				LCD.print("R QRD: ");
				if (rightQRD_curVal >= rightQRD_thresh) {
				  	LCD.print(" ON ");
				} else {
				    LCD.print("OFF ");
				}
				LCD.setCursor(0,1);	
				LCD.print(rightQRD_curVal, DEC);	

				break;	

			case CENTRE_QRD:
				LCD.print("C QRD: ");
				if (centreQRD_curVal >= centreQRD_thresh) {
				  	LCD.print(" ON ");
				} else {
				    LCD.print("OFF ");
				}
				LCD.setCursor(0,1);	
				LCD.print(centreQRD_curVal, DEC);	

				break;	

			default:
				break;

		}

		delay(refreshDelay);

	}
}

// Complete
void testFrwdDrive(){
	#define STRAIGHT_FORWARD 0
	#define SLIGHT_LEFT 1
	#define SLIGHT_RIGHT 2
	#define LOTS_LEFT 3
	#define LOTS_RIGHT 4
	static int const NUM_MOTORSTATES = 5;

	int leftSpeed;
	int rightSpeed;

	countdown();

	while(!backButton()){

		menuKnobPos = getMenuKnobPos(NUM_MOTORSTATES);

		switch(menuKnobPos){	

			case STRAIGHT_FORWARD:
				cleanLCD();
				LCD.print("EVEN Straight");

				leftSpeed = motorSpeed_base + motorSpeed_offset;
				rightSpeed = motorSpeed_base - motorSpeed_offset;

				if (leftSpeed > 700){
					leftSpeed = 700;}
				if (rightSpeed > 700){
					rightSpeed = 700;}

				motor.speed(LEFTMOTOR_PIN, leftSpeed);
				motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		        LCD.setCursor(0,1);
		        LCD.print("L: ");
		        LCD.print(leftSpeed, DEC);
		        LCD.print(" R: ");
		        LCD.print(rightSpeed, DEC);
				break;	

			case SLIGHT_LEFT:	
				cleanLCD();
				LCD.print("+");
				LCD.print(slightAdjust, DEC);
				LCD.print(" Left");

				leftSpeed = motorSpeed_base + motorSpeed_offset + slightAdjust;
				rightSpeed = motorSpeed_base - motorSpeed_offset;

				if (leftSpeed > 700){
					leftSpeed = 700;}
				if (rightSpeed > 700){
					rightSpeed = 700;}

				motor.speed(LEFTMOTOR_PIN, leftSpeed);
				motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		        LCD.setCursor(0,1);
		        LCD.print("L: ");
		        LCD.print(leftSpeed, DEC);
		        LCD.print(" R: ");
		        LCD.print(rightSpeed, DEC);
				break;	

			case SLIGHT_RIGHT:
				cleanLCD();
				LCD.print("+");
				LCD.print(slightAdjust, DEC);
				LCD.print(" Right");

				leftSpeed = motorSpeed_base + motorSpeed_offset;
				rightSpeed = motorSpeed_base - motorSpeed_offset + slightAdjust;

				if (leftSpeed > 700){
					leftSpeed = 700;}
				if (rightSpeed > 700){
					rightSpeed = 700;}

				motor.speed(LEFTMOTOR_PIN, leftSpeed);
				motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		        LCD.setCursor(0,1);
		        LCD.print("L: ");
		        LCD.print(leftSpeed, DEC);
		        LCD.print(" R: ");
		        LCD.print(rightSpeed, DEC);
				break;	

			case LOTS_LEFT:	
				cleanLCD();
				LCD.print("+");
				LCD.print(lotsAdjust, DEC);
				LCD.print(" Left");

				leftSpeed = motorSpeed_base + motorSpeed_offset + lotsAdjust;
				rightSpeed = motorSpeed_base - motorSpeed_offset;

				if (leftSpeed > 700){
					leftSpeed = 700;}
				if (rightSpeed > 700){
					rightSpeed = 700;}

				motor.speed(LEFTMOTOR_PIN, leftSpeed);
				motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		        LCD.setCursor(0,1);
		        LCD.print("L: ");
		        LCD.print(leftSpeed, DEC);
		        LCD.print(" R: ");
		        LCD.print(rightSpeed, DEC);
				break;

			case LOTS_RIGHT:
				cleanLCD();
				LCD.print("+");
				LCD.print(lotsAdjust, DEC);
				LCD.print(" Right");

				leftSpeed = motorSpeed_base + motorSpeed_offset;
				rightSpeed = motorSpeed_base - motorSpeed_offset + lotsAdjust;

				if (leftSpeed > 700){
					leftSpeed = 700;}
				if (rightSpeed > 700){
					rightSpeed = 700;}

				motor.speed(LEFTMOTOR_PIN, leftSpeed);
				motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		        LCD.setCursor(0,1);
		        LCD.print("L: ");
		        LCD.print(leftSpeed, DEC);
		        LCD.print(" R: ");
		        LCD.print(rightSpeed, DEC);
				break;	

			default:
				break;
		}

		delay(refreshDelay);

	}

	hitBrakes();
}

// Complete
void testServos(){

	#define COLLECT_ARM 0
	#define SWEEPER 1
	static int const NUM_SERVOS = 2;


	while(!backButton()){

		cleanLCD();
		menuKnobPos = getMenuKnobPos(NUM_SERVOS);

		switch(menuKnobPos){	

			case COLLECT_ARM:
				LCD.print("Collector Arm");
				if(userMenuSelection()){
					turnServo(COLLECTORSERVO_PIN);
				}
				break;	

			case SWEEPER:
				LCD.print("Sweeper");
				if(userMenuSelection()){
					turnServo(SWEEPERSERVO_PIN);
				}
				break;	

			default:
				break;

		}

		delay(refreshDelay);
	}
}

// Complete
void testIdolGrab(){

	int currentCollectorAngle;
	int currentSweeperAngle;

	int state;

	int collectorMaxAngle = 180;
	int collectorMinAngle = 0;
	int sweeperMaxAngle = 0;
	static int const delayTime = 200;

	countdown();

	RCServo0.write(collectorDefaultAngle);
	RCServo1.write(sweeperDefaultAngle);
	delay(1500);

	currentCollectorAngle = collectorDefaultAngle;
	currentSweeperAngle = sweeperDefaultAngle;

	while(!backButton()){

		cleanLCD();
		LCD.print("Looking...");

		if(collectorSwitch()){
			state = 1;
			while(!backButton()){
				idolCollect(currentCollectorAngle, currentSweeperAngle);
				state++;
			}
		}

		delay(refreshDelay);

	}
}

// Complete
void testSwitches(){

	int displayDelay = 1000;

	while(!backButton()){
		cleanLCD();
		LCD.print("Press Switch");

		if (collectorSwitch()){
			cleanLCD();
			LCD.print("Collector Active");
			delay(displayDelay);
		}	

		delay(refreshDelay);
	}
}

// Complete
void testSonar(){

  long dist;

	while(!backButton()){

		dist = sonarRange();

		LCD.print("Dist: ");
		LCD.print(dist, DEC);
		LCD.print("in");

		/*
		if (dist >= edgeDetect){
			cleanBottomLine();
			LCD.print("Edge Detected");
		} else if(dist <= hillDetect){
			cleanBottomLine();
			LCD.print("Hill Detected");
		} else {
			cleanBottomLine();
			LCD.print("Default Detect");
		}
		*/

		delay(refreshDelay);
	}
}

// Complete
void testConveyor(){

	int conveyorSpeed = getValueKnobPos(1400)-700;

	while(!backButton()){
		conveyorSpeed = getValueKnobPos(1400)-700;
		motor.speed(CONVEYORMOTOR_PIN, conveyorSpeed);

		LCD.print("Conv Speed: ");
		LCD.print(conveyorSpeed, DEC);

		delay(refreshDelay);
	}
}

// In Progress (CURRENT)
void testIR(){

	int leftIR_curVal, rightIR_curVal;

	int minThresh = (int)(0.300 * 1024.0/5.0);
	int maxThresh = 1023; // min = 61, max = 1023


	while(!backButton()){

		cleanLCD();

		leftIR_curVal = analogRead(LEFTIR_PIN);
		rightIR_curVal = analogRead(RIGHTIR_PIN);

		LCD.print("L IR: ");
		if (leftIR_curVal >= maxThresh || leftIR_curVal < minThresh){
			LCD.print("OOR");
		} else{
			LCD.print(leftIR_curVal, DEC);}

		LCD.setCursor(0,1);	

		LCD.print("R IR: ");
		if (rightIR_curVal >= maxThresh || rightIR_curVal < minThresh){
			LCD.print("OOR");
		} else{
			LCD.print(rightIR_curVal, DEC);}

		delay(refreshDelay);

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




// Demo Assist Programs

// Complete
void turnServo(int servoNum){


	int destinationAngle;
	int currentAngle;
	int turningRate;
	int maxServoTurnTime = 1500;
	int increment;

	countdown();

	// slowly turn to default angle and delay until this is true
	// but for now...

	LCD.print("Servo");
	LCD.print(servoNum, DEC);
	LCD.print(" @ Pos ");
	LCD.print(servoDefaultAngle, DEC);

	switch(servoNum){

		case 0:
			RCServo0.write(servoDefaultAngle);
			delay(maxServoTurnTime);
			break;

		case 1:
			RCServo1.write(servoDefaultAngle);
			delay(maxServoTurnTime);
			break;

		case 2:
			RCServo2.write(servoDefaultAngle);
			delay(maxServoTurnTime);
			break;
	}

	currentAngle = servoDefaultAngle;

	cleanLCD();
	LCD.print("Servo ");
	LCD.print(servoNum, DEC);
	LCD.print(": ");

	while(!backButton()){

		cleanBottomLine();

		destinationAngle = getValueKnobPos(181);
		LCD.print("Cv: ");
		LCD.print(currentAngle, DEC);
		LCD.print(" Fin: ");
		LCD.print(destinationAngle, DEC);

		for (increment = 0; increment < (int)((float)refreshDelay/(float)(servoDelayTime + 1)); increment++){

			if (currentAngle > destinationAngle){
				currentAngle --;
			} else if (currentAngle < destinationAngle){
				currentAngle ++;}	

			switch(servoNum){	

				case 0:
					RCServo0.write(currentAngle);
					break;	

				case 1:
					RCServo1.write(currentAngle);
					break;	

				case 2:
					RCServo2.write(currentAngle);
					break;

			}	

			delay(servoDelayTime);
		}

	}
}

// Complete
void idolCollect(int currentCollectorAngle, int currentSweeperAngle){


	// make this breakable


	int collectorMaxAngle = 0;
	int collectorMinAngle = 180;
	int sweeperMaxAngle = 29;
	static int const delayTime = 500;

	// start conveyor motor

	cleanLCD();
	LCD.print("Collector:");
	LCD.setCursor(0,1);
	LCD.print("Touch Idol");
	turnServo2Angle(collectorMinAngle, currentCollectorAngle, COLLECTORSERVO_PIN);
	delay(delayTime);		

	cleanLCD();
	LCD.print("Collector:");
	LCD.setCursor(0,1);
	LCD.print("Raise Arm");
	turnServo2Angle(collectorMaxAngle, collectorMinAngle, COLLECTORSERVO_PIN);
	delay(delayTime);		

	cleanLCD();
	LCD.print("Sweeper:");
	LCD.setCursor(0,1);
	LCD.print("Deposit Idol");
	turnServo2Angle(sweeperMaxAngle, currentSweeperAngle, SWEEPERSERVO_PIN);
	delay(delayTime);		

	cleanLCD();
	LCD.print("Coll. + Sweep:");
	LCD.setCursor(0,1);
	LCD.print("Return Home");
	turnServo2Angle(sweeperDefaultAngle, sweeperMaxAngle, SWEEPERSERVO_PIN);
	turnServo2Angle(collectorDefaultAngle, collectorMaxAngle, COLLECTORSERVO_PIN);
	delay(delayTime);

	// stop conveyor motor
}

// Complete
void tapeAndIdols(){

	#define ON 1
	#define OFF 0

	cleanLCD();

	static int const LEFT = -1;
	static int const RIGHT = 1;

	int smallErrorMultiplier = 1;
	int midErrorMultiplier = 3;
	int largeErrorMultiplier = 5;

	int hillMode = OFF;

	int leftQRD_curVal, rightQRD_curVal, centreQRD_curVal;	

	int error_curVal, error_lastVal = 0;
	int error_timeInErrorState = 0, error_slopeChangeFromLastError;	

	int proportional_curVal, derivative_curVal;	

	int motorSpeed_errorOffset;	

	int timeTicker = 0;

	countdown();

	RCServo0.write(collectorDefaultAngle);
	RCServo1.write(sweeperDefaultAngle);
	delay(1500);

	while(!backButton()){

	    leftQRD_curVal = analogRead(LEFTQRD_PIN);
	    rightQRD_curVal = analogRead(RIGHTQRD_PIN);
	    centreQRD_curVal = analogRead(CENTREQRD_PIN);	


		if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = 0;

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = 0;	

		} else if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = LEFT * smallErrorMultiplier;	

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = RIGHT * smallErrorMultiplier;		

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal < centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = RIGHT * midErrorMultiplier;		

		} else if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal < centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = LEFT * midErrorMultiplier;
		    
		} else if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal < centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		  if (error_lastVal > 0) {
		     	error_curVal = RIGHT * largeErrorMultiplier;		

		  } else if (error_lastVal < 0) {
		     	error_curVal = LEFT * largeErrorMultiplier;
		  } else {
		  		error_curVal = 0;
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

	    motorSpeed_errorOffset = proportional_curVal + derivative_curVal;

	    if (timeTicker == 100) {
	        timeTicker = 0;	
	        dispError(error_curVal);
	    }	

	    if (motorSpeed_offset < -700) {
	        motor.speed(LEFTMOTOR_PIN, 700);
	        motor.speed(RIGHTMOTOR_PIN, motorSpeed_base - motorSpeed_offset);

	    } else if (motorSpeed_offset > 700) {
	        motor.speed(LEFTMOTOR_PIN, motorSpeed_base + motorSpeed_offset);
	        motor.speed(RIGHTMOTOR_PIN, 700);

	    } else {
	        motor.speed(LEFTMOTOR_PIN, motorSpeed_base + motorSpeed_offset - motorSpeed_errorOffset);
	        motor.speed(RIGHTMOTOR_PIN, motorSpeed_base - motorSpeed_offset + motorSpeed_errorOffset);
	    }

	    if(collectorSwitch()){
			hitBrakes();
			idolCollect(collectorDefaultAngle, sweeperDefaultAngle);
		}

	    /*
	    if (sonarRange() <= hillSonarThresh){
	    	leftMotorSpeed_base = leftMotorSpeed_base + 150;
	    	rightMotorSpeed_base = leftMotorSpeed_base + 150;
	    	hillMode = ON;
		} else if(sonarRange() >= edgeSonarThresh){
			motor.speed(LEFTMOTOR_PIN, 0);
			motor.speed(RIGHTMOTOR_PIN, 0);
		}

		if (hillMode == ON && sonarRange() > hillSonarThresh) {
			hillMode = OFF;
			leftMotorSpeed_base = leftMotorSpeed_base - 150;
	    	rightMotorSpeed_base = leftMotorSpeed_base - 150;
		}
		*/

	    error_lastVal = error_curVal;
	    timeTicker++;
	    error_timeInErrorState++; 	

	}

	/*
	leftMotorSpeed_base = EEPROM.read(LEFTMOTORSPEEDBASE_EEPROM) * 5;
	rightMotorSpeed_base = EEPROM.read(RIGHTMOTORSPEEDBASE_EEPROM) * 5;
	*/

	hitBrakes();
}

// Complete
void dispError(int error_curVal){

	cleanLCD();

	switch(error_curVal){
		case 0:
			LCD.print("     ######     ");
			LCD.setCursor(0,1);
			LCD.print("     ######     ");
			break;
		case -1:
			LCD.print("###");
			LCD.setCursor(0,1);
			LCD.print("###             ");
			break;
		case -3:
			LCD.print("#####           ");
			LCD.setCursor(0,1);
			LCD.print("#####           ");
			break;
		case -5:
			LCD.print("########        ");
			LCD.setCursor(0,1);
			LCD.print("########        ");
			break;
		case 1:
			LCD.print("             ###");
			LCD.setCursor(0,1);
			LCD.print("             ###");
			break;
		case 3:
			LCD.print("           #####");
			LCD.setCursor(0,1);
			LCD.print("           #####");
			break;
		case 5:
			LCD.print("        ########");
			LCD.setCursor(0,1);
			LCD.print("        ########");
			break;
	}
}

// Complete
long sonarRange() {
	digitalWrite(SONAROUT_PIN, LOW);
	delayMicroseconds(2);
	digitalWrite(SONAROUT_PIN, HIGH);
	delayMicroseconds(5);
	digitalWrite(SONAROUT_PIN, LOW);

	return pulseIn(SONARIN_PIN, HIGH)/148.0;
}

// Complete
void hitBrakes(){
	int leftBrakeSpeed = 0;
	int rightBrakeSpeed = 0;
	motor.speed(LEFTMOTOR_PIN, leftBrakeSpeed);
	motor.speed(RIGHTMOTOR_PIN, rightBrakeSpeed);	
}




// Digital Booleans

bool collectorSwitch(){
	if (digitalRead(COLLECTIONSWITCH_PIN)){
		return true;}
	return false;
}




// Abstracted Helper Functions

int getMenuKnobPos(int numPos){
	int userSet = (int)(1.2*knob(MENUKNOB_PIN) * ((float)numPos)/1024.0);
	if (userSet >= numPos){
		return numPos-1;}
	return userSet;
}

int getValueKnobPos(int numPos){
	int userSet = (int)(1.2*knob(VALUEKNOB_PIN) * ((float)numPos)/1024.0);
		if (userSet >= numPos){
		return numPos-1;}
	return userSet;
}

bool userMenuSelection(){

	int valueKnobPos = getValueKnobPos(2);

	// Is This Necessary?
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

bool enterButton(){
	// get state of enter button

	int debounceDelay = 500;

	if (startbutton()) {    
		delay(debounceDelay);       
        return true;
    }  else {
        return false;
    }
}

bool backButton(){
	// get state of back button

	int debounceDelay = 500;

	if (stopbutton()) {
		delay(debounceDelay);
        return true;
    } else {
        return false;
    }
}

void cleanLCD(){
	LCD.clear();
	LCD.home();
}

void cleanBottomLine(){
	LCD.setCursor(0,1);
	LCD.print("                ");
	LCD.setCursor(0,1);
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
	int enterDelay = 1500;

	while(!backButton()){
		cleanBottomLine();

		tempParam = getValueKnobPos((int)((float)maxVal/5.0));
		tempParam = 5 * tempParam;

		LCD.print(tempParam, DEC);
		LCD.print(" Cur: ");
		LCD.print(originalParam, DEC);

		if(enterButton()){
			newParam = tempParam;
			cleanBottomLine();
			LCD.print("New: ");
			LCD.print(newParam, DEC);
			delay(enterDelay);
			return newParam;
		}

		delay(refreshDelay);
	}

	return originalParam;
}

int paramAdjustNegVals(int originalParam, int maxVal){

	int tempParam;
	int newParam;
	int enterDelay = 1500;

	while(!backButton()){
		cleanBottomLine();

		tempParam = 2 * getValueKnobPos((int)((float)maxVal/2.5)) - (int)((float)maxVal/2.5);
		tempParam = 2.5 * tempParam;

		LCD.print(tempParam, DEC);
		LCD.print(" Cur: ");
		LCD.print(originalParam, DEC);

		if(enterButton()){
			newParam = tempParam;
			cleanBottomLine();
			LCD.print("New: ");
			LCD.print(newParam, DEC);
			delay(enterDelay);
			return newParam;
		}

		delay(refreshDelay);
	}

	return originalParam;
}

int paramAdjustFullRange(int originalParam, int maxVal){

	int tempParam;
	int newParam;
	int enterDelay = 1500;

	while(!backButton()){
		LCD.setCursor(0,1);
		LCD.print("                ");
		LCD.setCursor(0,1);

		tempParam = getValueKnobPos(maxVal);
		tempParam = tempParam;

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

void turnServo2Angle(int destinationAngle, int currentAngle, int servoNum){

	while (currentAngle != destinationAngle) {

		if (currentAngle > destinationAngle){
			currentAngle --;
		} else if (currentAngle < destinationAngle){
			currentAngle ++;}	

		switch(servoNum){	
			case 0:
				RCServo0.write(currentAngle);
				break;	
			case 1:
				RCServo1.write(currentAngle);
				break;	
			case 2:
				RCServo2.write(currentAngle);
				break;
		}	

		delay(servoDelayTime);
	}
}

void initEEPROM(){

	// EEPROM.write(0, ((int)300.0/5.0)); 	// leftQRD_thresh
	// EEPROM.write(1, ((int)300.0/5.0)); 	// rightQRD_thresh
	// EEPROM.write(2, ((int)30.0/5.0)); 	// proportional_gain
	// EEPROM.write(3, ((int)0.0/5.0)); 	// derivative_gain
	// EEPROM.write(4, ((int)325.0/5.0)); 	// motorSpeed_base
	// EEPROM.write(5, ((int)25.0/5.0)); 	// motorSpeed_offset
	// EEPROM.write(6, ((int)700.0/5.0)); 	// motorFast
	// EEPROM.write(7, ((int)350.0/5.0)); 	// motorSlow 
	// EEPROM.write(8, ((int)50.0/5.0)); 	// slightAdjustMotor
	// EEPROM.write(9, ((int)100.0/5.0)); 	// lotsAdjustMotor
	// EEPROM.write(10, 5); 				// largeErrorMulitplier 
	// EEPROM.write(11, 10); 				// servoDelayTime
	// EEPROM.write(12, ((int)0.0/5.0)); 	// servoDefaultAngle
	// EEPROM.write(13, ((int)300.0/5.0)); 	// centreQRD_thresh
	// EEPROM.write(14, 20); 				// collectorDefaultAngle
	// EEPROM.write(15, 20); 				// sweeperDefaultAngle
	// EEPROM.write(16, 3);					// hillSonarThresh
	// EEPROM.write(17, 24);				// edgeSonarThresh
	// EEPROM.write(18, 1);					// motorSpeedOffsetSign


	cleanLCD();
	LCD.print("Params Set");
	delay(1500);
}


// Default Menu Template
/*	
	#define CASE_0 0
	#define CASE_1 1
	static int const NUM_CASES = 2;

	while(!backButton()){

		cleanLCD();
		menuKnobPos = getMenuKnobPos(NUM_CASES);

		switch(menuKnobPos){	

			case CASE_0:
				LCD.print("Case 0");
				if(userMenuSelection()){
					// do stuff
				}
				break;	

			case CASE_1:
				LCD.print("Case 1");
				if(userMenuSelection()){
					// do stuff
				}
				break;	

			default:
				break;

		}

		delay(refreshDelay);
	}
*/


