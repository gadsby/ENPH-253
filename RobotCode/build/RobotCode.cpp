#include <phys253.h>          //   ***** from 253 template file
#include <LiquidCrystal.h>    //   ***** from 253 template file
#include <Servo253.h>         //   ***** from 253 template file
#include <EEPROM.h>


// Main Menu
#define AUTONOMOUS_MODE 0
#define DRIVING_MODE 1
#define DEMO_MODE 2
#define ADJUST_PARAMETERS 3
#define LOOKUP_PINS 4
#define NUM_MAINMENU 5


// Analog Pins (0-7)
#define LEFTIR_PIN 1
#define RIGHTIR_PIN 2
#define CENTREQRD_PIN 3
#define LEFTQRD_PIN 4
#define RIGHTQRD_PIN 5
#define MENUKNOB_PIN 6
#define VALUEKNOB_PIN 7

// Digital Pins (0-7 Out, 8-15 In)
#define RIGHTBARSENSOR_PIN 8
#define COLLECTIONSWITCH_PIN 9
#define LEFTBARSENSOR_PIN 15

// Motor Pins (0-3)
#define LEFTMOTOR_PIN 0
#define RIGHTMOTOR_PIN 2
#define WINCHMOTOR_PIN 3

// Servo Pins (0-2)
#define COLLECTORSERVO_PIN 0
#define SWEEPERSERVO_PIN 1
#define TRMRELEASE_PIN 2


// Parameter EEPROM Address
#define LEFTQRDTHRESH_EEPROM 0
#define RIGHTQRDTHRESH_EEPROM 1
#define CENTREQRDTHRESH_EEPROM 13

#define PROPGAIN_EEPROM 2
#define DERVGAIN_EEPROM 3

#define MOTORSPEEDBASE_EEPROM 4
#define MOTORSPEEDOFFSET_EEPROM 5
#define MOTORSPEEDOFFSETSIGN_EEPROM 18

#define SERVODELAYTIME_EEPROM 11
#define COLLECTORDEFAULTANGLE_EEPROM 14
#define SWEEPERDEFAULTANGLE_EEPROM 15

#define PROPGAINIR_EEPROM 19
#define DERVGAINIR_EEPROM 20


// Adjustable Parameters
#include "WProgram.h"
#include <HardwareSerial.h>
void setup();
void loop();
void drivingDemoList();
void accessDemoList();
void accessDemoList_page2();
void adjustParameterVals();
void adjustParameterVals_page2();
void lookUpPins();
void runAutonomousMode();
void analogList();
void digitalList();
void motorList();
void servoList();
void tapeAndIdols();
void IRandLastIdol();
void acquireZipline();
void ride2Victory();
void testTapeFollowing();
void testIRFollow();
void oneAndDone();
void threeAndDone();
void threeAndDoneBonus();
void testHBridge();
void testQRDs();
void testServos();
void testIdolGrab();
void testSwitches();
void testWinch();
void testIR();
void testZiplineTrigger();
void testRaiseAndLower();
void testReachBar();
void testWinchTimes();
void turnServo(int servoNum);
void idolCollect(int currentCollectorAngle, int currentSweeperAngle);
int idolCollect2(int idolCollectState);
void halfIdolCollect(int currentCollectorAngle, int currentSweeperAngle);
void dispError(int error_curVal);
void hitBrakes();
void fullTurnLeft();
void pivotRight();
void releaseBands();
void runWinch(int winchSpeed);
void stopWinch();
void winchUp();
void winchDown();
void dropToZipline();
bool ziplineDetector();
bool collectorSwitch();
bool detectBarLeft();
bool detectBarRight();
int getMenuKnobPos(int numPos);
int getValueKnobPos(int numPos);
bool userMenuSelection();
bool enterButton();
bool backButton();
void cleanLCD();
void cleanBottomLine();
void countdown();
int paramAdjust(int originalParam, int maxVal);
int paramAdjustNegVals(int originalParam, int maxVal);
int paramAdjustFullRange(int originalParam, int maxVal);
void turnServo2Angle(int destinationAngle, int currentAngle, int servoNum);
int TwelveBitVolts(double voltage);
void initEEPROM();
int leftQRD_thresh = EEPROM.read(LEFTQRDTHRESH_EEPROM) * 5;
int rightQRD_thresh = EEPROM.read(RIGHTQRDTHRESH_EEPROM) * 5;
int centreQRD_thresh = EEPROM.read(CENTREQRDTHRESH_EEPROM) * 5;

int proportional_gain = EEPROM.read(PROPGAIN_EEPROM) * 5;
int derivative_gain = EEPROM.read(DERVGAIN_EEPROM) * 5;

int motorSpeed_base = EEPROM.read(MOTORSPEEDBASE_EEPROM) * 5;
int motorSpeed_offset = EEPROM.read(MOTORSPEEDOFFSET_EEPROM) * 5;

int servoDelayTime = EEPROM.read(SERVODELAYTIME_EEPROM);
int collectorDefaultAngle = EEPROM.read(COLLECTORDEFAULTANGLE_EEPROM);
int sweeperDefaultAngle = EEPROM.read(SWEEPERDEFAULTANGLE_EEPROM);

int proportional_gainIR = EEPROM.read(PROPGAINIR_EEPROM) * 5;
int derivative_gainIR = EEPROM.read(DERVGAINIR_EEPROM) * 5;


// Program Constants
int menuKnobPos;
static int const refreshDelay = 50;
static int const LEFT = -1;
static int const RIGHT = 1;




void setup(){
    portMode(0, OUTPUT);     //   ***** from 253 template file
    portMode(1, INPUT);      //   ***** from 253 template file

    RCServo0.attach(RCServo0Output);
    RCServo1.attach(RCServo1Output);
    RCServo2.attach(RCServo2Output);

	RCServo0.write(0);
	RCServo1.write(sweeperDefaultAngle);
	RCServo2.write(180);

	if(EEPROM.read(MOTORSPEEDOFFSETSIGN_EEPROM) == 1){
		int motorSpeed_offset = EEPROM.read(MOTORSPEEDOFFSET_EEPROM) * 5;}
	else{
		int motorSpeed_offset = EEPROM.read(MOTORSPEEDOFFSET_EEPROM) * -5;}

	hitBrakes();
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

		case DRIVING_MODE:
			LCD.print("Driving Test?");	
			if(userMenuSelection()){
				drivingDemoList();}	
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
void drivingDemoList(){

	#define TEST_TAPEFOLLOW 0
	#define TEST_IR_FOLLOW 1
	#define ONE_AND_DONE 2
	#define THREE_AND_DONE 3
	#define THREE_AND_DONE_BONUS 4
	static int const NUM_SERVOS = 5;


	while(!backButton()){

		cleanLCD();
		menuKnobPos = getMenuKnobPos(NUM_SERVOS);

		switch(menuKnobPos){	

			case TEST_TAPEFOLLOW:	
				LCD.print("Test TapeFollow?");	
				if(userMenuSelection()){
					testTapeFollowing();}	
				break;	

			case TEST_IR_FOLLOW:
				LCD.print("Test IR Follow?");	
				if(userMenuSelection()){
					testIRFollow();}
				break;

			case ONE_AND_DONE:
				LCD.print("Grab 1 & Return?");	
				if(userMenuSelection()){
					oneAndDone();}
				break;

			case THREE_AND_DONE:
				LCD.print("Grab 3 & Return?");	
				if(userMenuSelection()){
					threeAndDone();}
				break;	

			case THREE_AND_DONE_BONUS:
				LCD.print("Grab 3? (NEW)");	
				if(userMenuSelection()){
					threeAndDoneBonus();}
				break;

			default:
				break;

		}

		delay(refreshDelay);
	}
}

// COMPLETE
void accessDemoList(){
	// show demo options

	#define TEST_HBRIDGE 0
	#define TEST_SWITCHES 1
	#define TEST_SERVOS 2
	#define TEST_QRD 3
	#define TEST_IR 4
	#define TEST_TEL_ARM 5
	#define TEST_ZL_TRIGGER 6
	#define TEST_IDOLGRAB 7
	#define TEST_WINCH 8
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
					testRaiseAndLower();}
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

			case TEST_WINCH:	
				LCD.print("Test Winch?");	
				if(userMenuSelection()){
					testWinch();}	
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
	#define TEST_ZIPLINE_RIDE 0
	#define TEST_WINCH_TIME 1
	static int const NUM_DEMOMENU = 2;
	#define NEXT_PAGE 9

	while(!backButton()){

		menuKnobPos = getMenuKnobPos(NUM_DEMOMENU);
		cleanLCD();

		switch(menuKnobPos){	

			case TEST_ZIPLINE_RIDE:
				LCD.print("Test Reach Bar?");
				if(userMenuSelection()){
					testReachBar();}
				break;

			case TEST_WINCH_TIME:
				LCD.print("Test WinchTime?");	
				if(userMenuSelection()){
					testWinchTimes();}
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
	#define CENTRE_QRD 2
	#define PROPTF_GAIN 3
	#define DERVTF_GAIN 4
	#define MOTORSPEED_BASE 5
	#define MOTORSPEED_OFFSET 6
	#define SERVO_DELAYTIME 7
	#define NEXT_PAGE 8
	static int const NUM_PARAMS = 9;

	int leftQRDThresh_maxVal = 700;
	int rightQRDThresh_maxVal = 700;
	int centreQRDThresh_maxVal = 700;
	int proportionalGain_maxVal = 250;
	int derivativeGain_maxVal = 250;
	int motorSpeedBase_maxVal = 700;
	int motorSpeedOffset_maxVal = 350;
	int servoDelayTime_maxVal = 26;

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

			case CENTRE_QRD:
				LCD.print("C QRD Thresh");
				if(userMenuSelection()){
					EEPROM.write(CENTREQRDTHRESH_EEPROM, (int)((float)paramAdjust(centreQRD_thresh, centreQRDThresh_maxVal)/5.0));
					centreQRD_thresh = EEPROM.read(CENTREQRDTHRESH_EEPROM) * 5;
				}
				break;

			case PROPTF_GAIN:	
				LCD.print("Prop Gain TF");
				if(userMenuSelection()){
					EEPROM.write(PROPGAIN_EEPROM, (int)((float)paramAdjust(proportional_gain, proportionalGain_maxVal)/5.0));
					proportional_gain = EEPROM.read(PROPGAIN_EEPROM) * 5;
				}
				break;	

			case DERVTF_GAIN:
				LCD.print("Derv Gain TF");
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
						EEPROM.write(MOTORSPEEDOFFSET_EEPROM, -writeVal);
						EEPROM.write(MOTORSPEEDOFFSETSIGN_EEPROM, 0);
						motorSpeed_offset = EEPROM.read(MOTORSPEEDOFFSET_EEPROM) * -5;
					}
				}
				break;

			case SERVO_DELAYTIME:
				LCD.print("Servo Delay");
				if(userMenuSelection()){
					EEPROM.write(SERVODELAYTIME_EEPROM, paramAdjustFullRange(servoDelayTime, servoDelayTime_maxVal));
					servoDelayTime = EEPROM.read(SERVODELAYTIME_EEPROM);
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

	#define COLLECTOR_DEFAULTANGLE 0
	#define SWEEPER_DEFAULTANGLE 1
	#define PROPIR_GAIN 2
	#define DERVIR_GAIN 3
	#define INITIALIZE_PARAMS 4
	static int const NUM_PARAMS = 5;
	#define NEXT_PAGE 9

	int collectorDefaultAngle_maxVal = 185;
	int sweeperDefaultAngle_maxVal = 185;
	int proportionalgainIR_maxVal = 250;
	int derivativegainIR_maxVal = 250;

		/*
		To add new param:
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

			case PROPIR_GAIN:
				LCD.print("Prop Gain IR");
				if(userMenuSelection()){
					EEPROM.write(PROPGAINIR_EEPROM, (int)((float)paramAdjust(proportional_gainIR, proportionalgainIR_maxVal)/5.0));
					proportional_gainIR = EEPROM.read(PROPGAINIR_EEPROM) * 5;
				}
				break;

			case DERVIR_GAIN:
				LCD.print("Derv Gain IR");
				if(userMenuSelection()){
					EEPROM.write(DERVGAINIR_EEPROM, (int)((float)paramAdjust(derivative_gainIR, derivativegainIR_maxVal)/5.0));
					derivative_gainIR = EEPROM.read(DERVGAINIR_EEPROM) * 5;
				}
				break;

			case INITIALIZE_PARAMS:
				LCD.print("Init EEPROM");
				if(userMenuSelection()){
					initEEPROM();
				}
				break;

			case NEXT_PAGE:
				LCD.print("< PAGE 3 >");
				if(userMenuSelection()){
					//adjustParameterVals_page3();
				}
				break;
		}

		delay(refreshDelay);

	}
}

/*
void adjustParameterVals_page3(){
	#define INITIALIZE_PARAMS 0
	static int const NUM_PARAMS = 1;
	#define NEXT_PAGE 9

		To add new param:
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
*/

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
	countdown();
	tapeAndIdols();
	// IRandLastIdol();
	// acquireZipline();
	// ride2Victory();
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
				LCD.print("A1: Left IR");
				break;	

			case PAGE_2:
				LCD.print("A2: Right IR");
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
	#define PAGE_9 8
	static int const NUM_CASES = 9;

	while(!backButton()){

		cleanLCD();
		menuKnobPos = getMenuKnobPos(NUM_CASES);

		switch(menuKnobPos){	

			case PAGE_1:
				LCD.print("Dig 0-7: Output");
				LCD.setCursor(0,1);
				LCD.print("Dig 8-15: Input");
				break;	

			case PAGE_2:
				LCD.print("D0: < Unused >");
				LCD.setCursor(0,1);
				LCD.print("D1: < Unused >");
				break;	

			case PAGE_3:
				LCD.print("D2: < Unused >");
				LCD.setCursor(0,1);
				LCD.print("D3: < Unused >");
				break;	

			case PAGE_4:
				LCD.print("D4: < Unused >");
				LCD.setCursor(0,1);
				LCD.print("D5: < Unused >");
				break;	

			case PAGE_5:
				LCD.print("D6: < Unused >");
				LCD.setCursor(0,1);
				LCD.print("D7: < Unused >");
				break;	

			case PAGE_6:
				LCD.print("D8: < Unused >");
				LCD.setCursor(0,1);
				LCD.print("D9: Coll. Switch");
				break;

			case PAGE_7:
				LCD.print("D10: StartHill");
				LCD.setCursor(0,1);
				LCD.print("D11: < Unused >");
				break;	

			case PAGE_8:
				LCD.print("D12: < Unused >");
				LCD.setCursor(0,1);
				LCD.print("D13: < Unused >");
				break;	

			case PAGE_9:
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
				LCD.print("M1: Winch");
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
				LCD.print("S2: TRM Release");
				break;	

			default:
				break;

		}

		delay(refreshDelay);
	}
}




// Autonomous Mains

// Complete
void tapeAndIdols(){

	cleanLCD();

	int smallErrorMultiplier = 1;
	int midErrorMultiplier = 3;
	int largeErrorMultiplier = 5;

	bool safetyMode = true;
	int speedDrop = 150;

	int leftQRD_curVal, rightQRD_curVal, centreQRD_curVal;	

	int error_curVal = RIGHT * largeErrorMultiplier;
	int error_lastVal = error_curVal;
	int error_timeInErrorState = 0, error_slopeChangeFromLastError;	

	int proportional_curVal, derivative_curVal;	

	int motorSpeed_errorOffset;	

	int timeTicker = 0;

	int leftSpeed;
	int rightSpeed;

	int IR_detected = 0;
	int leftIR_curVal;
	int rightIR_curVal;
	int minThreshIR = TwelveBitVolts(0.3);

	RCServo0.write(collectorDefaultAngle);
	RCServo1.write(sweeperDefaultAngle);

	// while(true){ <-- when IR is ready
	while(!backButton()){

	    leftQRD_curVal = analogRead(LEFTQRD_PIN);
	    rightQRD_curVal = analogRead(RIGHTQRD_PIN);
	    centreQRD_curVal = analogRead(CENTREQRD_PIN);

		if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = 0;
		    safetyMode = false;

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = 0;	
		    safetyMode = false;

		} else if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = LEFT * smallErrorMultiplier;
		    safetyMode = false;

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = RIGHT * smallErrorMultiplier;
		    safetyMode = false;

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal < centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = RIGHT * midErrorMultiplier;		
		    safetyMode = false;

		} else if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal < centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = LEFT * midErrorMultiplier;
		    safetyMode = false;
		    
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

	        leftIR_curVal = analogRead(LEFTIR_PIN);
	    	rightIR_curVal = analogRead(RIGHTIR_PIN);

	    	if (leftIR_curVal > minThreshIR || rightIR_curVal > minThreshIR){
	    		IR_detected++;
	    		if (IR_detected == 5){
	    			break;}
			} else{
	    		IR_detected = 0;}
	    }	


	    if(collectorSwitch()){
		    hitBrakes();
			idolCollect(collectorDefaultAngle, sweeperDefaultAngle);
	    	safetyMode = true;
	    }


		if (!safetyMode){
		    leftSpeed = motorSpeed_base + motorSpeed_offset - motorSpeed_errorOffset;
		    rightSpeed = motorSpeed_base - motorSpeed_offset + motorSpeed_errorOffset;	

		    if (leftSpeed > 700){
		    	leftSpeed = 700;}	

		    if (rightSpeed > 700){
		    	rightSpeed = 700;}	

	        motor.speed(LEFTMOTOR_PIN, leftSpeed);
        	motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		} else if (error_curVal > 0){

			leftSpeed = speedDrop - (motorSpeed_base + motorSpeed_offset);
		    rightSpeed = (motorSpeed_base - motorSpeed_offset) - speedDrop;	
	        motor.speed(LEFTMOTOR_PIN, leftSpeed);
        	motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		} else if (error_curVal < 0){
			leftSpeed = (motorSpeed_base + motorSpeed_offset) - speedDrop;
		    rightSpeed = speedDrop - (motorSpeed_base - motorSpeed_offset);
	        motor.speed(LEFTMOTOR_PIN, leftSpeed);
        	motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		}


	    error_lastVal = error_curVal;
	    timeTicker++;
	    error_timeInErrorState++; 	

	}

	hitBrakes();
}

// In Progress
void IRandLastIdol(){

	cleanLCD();

	int smallErrorMultiplier = 1;
	int midErrorMultiplier = 3;
	int largeErrorMultiplier = 5;
	int maxErrorMultiplier = 7;

	int minThresh = TwelveBitVolts(0.3);
	int lowThresh = TwelveBitVolts(1.5);
	int midThresh = TwelveBitVolts(3.0);
	int highThresh = TwelveBitVolts(4.0);
	int maxThresh = TwelveBitVolts(5.0);

	int leftIR_curVal, rightIR_curVal;
	int proportional_curVal, derivative_curVal;	

	int error_curVal, error_lastVal = 0;
	int error_timeInErrorState = 0, error_slopeChangeFromLastError;	

	int motorSpeed_errorOffset;	

	int timeTicker = 0;

	int leftSpeed;
	int rightSpeed;

	bool blindMode;

	RCServo0.write(collectorDefaultAngle);
	RCServo1.write(sweeperDefaultAngle);

	// while(true){ <-- for actual program
	while(!backButton()){

	    leftIR_curVal = analogRead(LEFTIR_PIN);
	    rightIR_curVal = analogRead(RIGHTIR_PIN);

		if (leftIR_curVal >= minThresh && rightIR_curVal >= minThresh){
			blindMode = false;
			if (leftIR_curVal >= maxThresh || rightIR_curVal >= maxThresh){
				hitBrakes();
				while(!backButton()){
					cleanLCD();
				       LCD.print("Max Detected");		
				       LCD.setCursor(0,1);		
				       LCD.print("<Stop> to End");
			      		delay(refreshDelay);
				}
				break;
			} else if (leftIR_curVal >= highThresh) {
				if (rightIR_curVal >= midThresh){
					error_curVal = 0;
				} else if (rightIR_curVal >= lowThresh){
					error_curVal = RIGHT * smallErrorMultiplier;
				} else if (rightIR_curVal >= minThresh){
					error_curVal = RIGHT * midErrorMultiplier;
				}
			} else if (leftIR_curVal >= midThresh) {		

				if (rightIR_curVal >= lowThresh){
					error_curVal = 0;
				} else if (rightIR_curVal >= minThresh){
					error_curVal = RIGHT * smallErrorMultiplier;
				}
			} else if (leftIR_curVal >= lowThresh) {		

				if (rightIR_curVal >= highThresh){
					error_curVal = LEFT * smallErrorMultiplier;
				} else if (rightIR_curVal >= minThresh){
					error_curVal = 0;
				}
			} else if (leftIR_curVal >= minThresh) {
				if (rightIR_curVal >= highThresh){
					error_curVal = LEFT * midErrorMultiplier;
				} else if (rightIR_curVal >= midThresh){
					error_curVal = LEFT * smallErrorMultiplier;
				} else if (rightIR_curVal >= minThresh){
					error_curVal = 0;
				}
			}
		} else if (leftIR_curVal >= minThresh && rightIR_curVal < minThresh){
			blindMode = false;
			if (leftIR_curVal >= highThresh){
				error_curVal = RIGHT * smallErrorMultiplier;
			} else if (leftIR_curVal >= midThresh){
				error_curVal = RIGHT * midErrorMultiplier;
			} else if (leftIR_curVal >= lowThresh){
				error_curVal = RIGHT * largeErrorMultiplier;
			} else if (leftIR_curVal >= minThresh){
				error_curVal = RIGHT * largeErrorMultiplier;
			}
		} else if (leftIR_curVal < minThresh && rightIR_curVal >= minThresh){
			blindMode = false;
			if (rightIR_curVal >= highThresh){
				error_curVal = LEFT * smallErrorMultiplier;
			} else if (rightIR_curVal >= midThresh){
				error_curVal = LEFT * midErrorMultiplier;
			} else if (rightIR_curVal >= lowThresh){
				error_curVal = LEFT * largeErrorMultiplier;
			} else if (rightIR_curVal >= minThresh){
				error_curVal = LEFT * largeErrorMultiplier;
			}
		} else if (leftIR_curVal < minThresh && rightIR_curVal < minThresh){		

			blindMode = true;		

			if (error_lastVal > 0) { // right multipliers
				if (error_lastVal == RIGHT * smallErrorMultiplier){
					error_curVal = RIGHT * midErrorMultiplier;
				} else if (error_lastVal == RIGHT * midErrorMultiplier){
					error_curVal = RIGHT * largeErrorMultiplier;
				} else if (error_lastVal == RIGHT * largeErrorMultiplier){
					error_curVal = RIGHT * maxErrorMultiplier;
				}		

			} else if (error_lastVal < 0) { // left multipliers
				if (error_lastVal == LEFT * smallErrorMultiplier){
					error_curVal = LEFT * midErrorMultiplier;
				} else if (error_lastVal == LEFT * midErrorMultiplier){
					error_curVal = LEFT * largeErrorMultiplier;
				} else if (error_lastVal == LEFT * largeErrorMultiplier){
					error_curVal = LEFT * maxErrorMultiplier;
				}		

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

	    proportional_curVal = proportional_gainIR * error_curVal;
	    derivative_curVal = (int)((float) derivative_gainIR * (float)(error_curVal - error_lastVal) / (float)(error_slopeChangeFromLastError + error_timeInErrorState));	

	    motorSpeed_errorOffset = proportional_curVal + derivative_curVal;

	    leftSpeed = motorSpeed_base + motorSpeed_offset - motorSpeed_errorOffset;
	    rightSpeed = motorSpeed_base - motorSpeed_offset + motorSpeed_errorOffset;

	    if (leftSpeed > 700){
	    	leftSpeed = 700;}

	    if (rightSpeed > 700){
	    	rightSpeed = 700;}

        motor.speed(LEFTMOTOR_PIN, leftSpeed);
        motor.speed(RIGHTMOTOR_PIN, rightSpeed);


		if(collectorSwitch()){
	    	hitBrakes();

			idolCollect(collectorDefaultAngle, sweeperDefaultAngle);
			// halfIdolCollect(collectorDefaultAngle, sweeperDefaultAngle);

			// leave in place until acquireZipline() is ready
			while(!backButton()){
				cleanLCD();
	        	LCD.print("Turn Off HBridge");
	        	cleanBottomLine();
	        	LCD.print("& Press <Stop>");
			}
			// END BLOCK

			break;
	    }


	    if (timeTicker == 100) {
	        timeTicker = 0;	
	        dispError(error_curVal);
	    }

		if (blindMode = false){
			error_lastVal = error_curVal;
		}

	    timeTicker++;
	    error_timeInErrorState++; 	

	}

	hitBrakes();
}

// In Progress
void acquireZipline(){

	int safetyDelay = 500;
	int rightMotorAdjust = 50; // calibrate speed for this
	int leftSpeed;
	int rightSpeed;
	winchUp();
	pivotRight(); // calibrate time for this

	// while(true){ <-- for actual program
	while(!backButton()){

		leftSpeed = motorSpeed_base + motorSpeed_offset;
		rightSpeed = motorSpeed_base - motorSpeed_offset + rightMotorAdjust;	

	   	if (leftSpeed > 700){
		   	leftSpeed = 700;}	

	   	if (rightSpeed > 700){
		   	rightSpeed = 700;}	

       	motor.speed(LEFTMOTOR_PIN, leftSpeed);
       	motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		if (ziplineDetector()){
			dropToZipline();
			delay(safetyDelay);
		break;
		}
	}
}

// In Progress
void ride2Victory(){
	releaseBands();
	// Celebratory Stuff?
}




// Driving Programs

// Complete
void testTapeFollowing(){

	cleanLCD();

	int smallErrorMultiplier = 1;
	int midErrorMultiplier = 3;
	int largeErrorMultiplier = 5;

	int leftQRD_curVal, rightQRD_curVal, centreQRD_curVal;	

	int error_curVal = RIGHT * largeErrorMultiplier;
	int error_lastVal = error_curVal;
	int error_timeInErrorState = 0, error_slopeChangeFromLastError;	

	int proportional_curVal, derivative_curVal;	

	int motorSpeed_errorOffset;	

	int timeTicker = 0;

	int leftSpeed;
	int rightSpeed;

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

	    leftSpeed = motorSpeed_base + motorSpeed_offset - motorSpeed_errorOffset;
	    rightSpeed = motorSpeed_base - motorSpeed_offset + motorSpeed_errorOffset;

	    if (leftSpeed > 700){
	    	leftSpeed = 700;}

	    if (rightSpeed > 700){
	    	rightSpeed = 700;}

        motor.speed(LEFTMOTOR_PIN, leftSpeed);
        motor.speed(RIGHTMOTOR_PIN, rightSpeed);


	    if (timeTicker == 100) {
	        timeTicker = 0;	

	        cleanLCD();

	        LCD.print("L MOT: ");
	        LCD.print(leftSpeed, DEC);	

	        LCD.setCursor(0,1);

	        LCD.print("R MOT: ");
	        LCD.print(rightSpeed, DEC);
	    }

	    error_lastVal = error_curVal;
	    timeTicker++;
	    error_timeInErrorState++;

	}

	hitBrakes();
}

// Complete
void testIRFollow(){

	cleanLCD();

	int smallErrorMultiplier = 1;
	int midErrorMultiplier = 3;
	int largeErrorMultiplier = 5;
	int maxErrorMultiplier = 7;

	int minThresh = TwelveBitVolts(0.3);
	int lowThresh = TwelveBitVolts(1.5);
	int midThresh = TwelveBitVolts(3.0);
	int highThresh = TwelveBitVolts(4.0);
	int maxThresh = TwelveBitVolts(5.0);

	int leftIR_curVal, rightIR_curVal;
	int proportional_curVal, derivative_curVal;	

	int error_curVal, error_lastVal = 0;
	int error_timeInErrorState = 0, error_slopeChangeFromLastError;	

	int motorSpeed_errorOffset;	

	int timeTicker = 0;

	int leftSpeed;
	int rightSpeed;

	bool blindMode;

	countdown();

	while(!backButton()){

	    leftIR_curVal = analogRead(LEFTIR_PIN);
	    rightIR_curVal = analogRead(RIGHTIR_PIN);	


		if (leftIR_curVal >= minThresh && rightIR_curVal >= minThresh){
			blindMode = false;
			if (leftIR_curVal >= maxThresh || rightIR_curVal >= maxThresh){
				hitBrakes();
				while(!enterButton()){
					cleanLCD();
				    LCD.print("Max Detected");		
				    LCD.setCursor(0,1);		
				    LCD.print("Move and <Start>");
			      	delay(refreshDelay);
				}
			} else if (leftIR_curVal >= highThresh) {
				if (rightIR_curVal >= midThresh){
					error_curVal = 0;
				} else if (rightIR_curVal >= lowThresh){
					error_curVal = RIGHT * smallErrorMultiplier;
				} else if (rightIR_curVal >= minThresh){
					error_curVal = RIGHT * midErrorMultiplier;
				}
			} else if (leftIR_curVal >= midThresh) {		

				if (rightIR_curVal >= lowThresh){
					error_curVal = 0;
				} else if (rightIR_curVal >= minThresh){
					error_curVal = RIGHT * smallErrorMultiplier;
				}
			} else if (leftIR_curVal >= lowThresh) {		

				if (rightIR_curVal >= highThresh){
					error_curVal = LEFT * smallErrorMultiplier;
				} else if (rightIR_curVal >= minThresh){
					error_curVal = 0;
				}
			} else if (leftIR_curVal >= minThresh) {
				if (rightIR_curVal >= highThresh){
					error_curVal = LEFT * midErrorMultiplier;
				} else if (rightIR_curVal >= midThresh){
					error_curVal = LEFT * smallErrorMultiplier;
				} else if (rightIR_curVal >= minThresh){
					error_curVal = 0;
				}
			}
		} else if (leftIR_curVal >= minThresh && rightIR_curVal < minThresh){
			blindMode = false;
			if (leftIR_curVal >= highThresh){
				error_curVal = RIGHT * smallErrorMultiplier;
			} else if (leftIR_curVal >= midThresh){
				error_curVal = RIGHT * midErrorMultiplier;
			} else if (leftIR_curVal >= lowThresh){
				error_curVal = RIGHT * largeErrorMultiplier;
			} else if (leftIR_curVal >= minThresh){
				error_curVal = RIGHT * largeErrorMultiplier;
			}
		} else if (leftIR_curVal < minThresh && rightIR_curVal >= minThresh){
			blindMode = false;
			if (rightIR_curVal >= highThresh){
				error_curVal = LEFT * smallErrorMultiplier;
			} else if (rightIR_curVal >= midThresh){
				error_curVal = LEFT * midErrorMultiplier;
			} else if (rightIR_curVal >= lowThresh){
				error_curVal = LEFT * largeErrorMultiplier;
			} else if (rightIR_curVal >= minThresh){
				error_curVal = LEFT * largeErrorMultiplier;
			}
		} else if (leftIR_curVal < minThresh && rightIR_curVal < minThresh){		
			blindMode = true;		
			if (error_lastVal > 0) { // right multipliers
				if (error_lastVal == RIGHT * smallErrorMultiplier){
					error_curVal = RIGHT * midErrorMultiplier;
				} else if (error_lastVal == RIGHT * midErrorMultiplier){
					error_curVal = RIGHT * largeErrorMultiplier;
				} else if (error_lastVal == RIGHT * largeErrorMultiplier){
					error_curVal = RIGHT * maxErrorMultiplier;
				}		

			} else if (error_lastVal < 0) { // left multipliers
				if (error_lastVal == LEFT * smallErrorMultiplier){
					error_curVal = LEFT * midErrorMultiplier;
				} else if (error_lastVal == LEFT * midErrorMultiplier){
					error_curVal = LEFT * largeErrorMultiplier;
				} else if (error_lastVal == LEFT * largeErrorMultiplier){
					error_curVal = LEFT * maxErrorMultiplier;
				}		

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

	    proportional_curVal = proportional_gainIR * error_curVal;
	    derivative_curVal = (int)((float) derivative_gainIR * (float)(error_curVal - error_lastVal) / (float)(error_slopeChangeFromLastError + error_timeInErrorState));	

	    motorSpeed_errorOffset = proportional_curVal + derivative_curVal;

	    leftSpeed = motorSpeed_base + motorSpeed_offset - motorSpeed_errorOffset;
	    rightSpeed = motorSpeed_base - motorSpeed_offset + motorSpeed_errorOffset;

	    if (leftSpeed > 700){
	    	leftSpeed = 700;}

	    if (rightSpeed > 700){
	    	rightSpeed = 700;}

        motor.speed(LEFTMOTOR_PIN, leftSpeed);
        motor.speed(RIGHTMOTOR_PIN, rightSpeed);

	    if (timeTicker == 100) {
	        timeTicker = 0;	

	        cleanLCD();

	        LCD.print("L MOT: ");
	        LCD.print(leftSpeed, DEC);	

	        LCD.setCursor(0,1);

	        LCD.print("R MOT: ");
	        LCD.print(rightSpeed, DEC);
	    }

		if (blindMode = false){
			error_lastVal = error_curVal;
		}

	    timeTicker++;
	    error_timeInErrorState++; 	

	}

	hitBrakes();
}

// Complete
void oneAndDone(){

	cleanLCD();

	int smallErrorMultiplier = 1;
	int midErrorMultiplier = 3;
	int largeErrorMultiplier = 5;

	bool collectionReady = true;

	bool safetyMode = true;
	int speedDrop = 100;

	int leftQRD_curVal, rightQRD_curVal, centreQRD_curVal;	

	int error_curVal = RIGHT * largeErrorMultiplier;
	int error_lastVal = error_curVal;
	int error_timeInErrorState = 0, error_slopeChangeFromLastError;	

	int proportional_curVal, derivative_curVal;	

	int motorSpeed_errorOffset;	

	int timeTicker = 0;

	int leftSpeed;
	int rightSpeed;

	countdown();

	RCServo0.write(collectorDefaultAngle);
	RCServo1.write(sweeperDefaultAngle);

	while(!backButton()){

	    leftQRD_curVal = analogRead(LEFTQRD_PIN);
	    rightQRD_curVal = analogRead(RIGHTQRD_PIN);
	    centreQRD_curVal = analogRead(CENTREQRD_PIN);


		if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = 0;
		    safetyMode = false;

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = 0;	
		    safetyMode = false;

		} else if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = LEFT * smallErrorMultiplier;
		    safetyMode = false;

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = RIGHT * smallErrorMultiplier;
		    safetyMode = false;

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal < centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = RIGHT * midErrorMultiplier;		
		    safetyMode = false;

		} else if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal < centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = LEFT * midErrorMultiplier;
		    safetyMode = false;
		    
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

	    if(collectorSwitch() && collectionReady){

	    	collectionReady = !collectionReady;

			hitBrakes();

			turnServo2Angle(180, collectorDefaultAngle, COLLECTORSERVO_PIN);
			delay(500);		
			turnServo2Angle(0, 180, COLLECTORSERVO_PIN);

			safetyMode = true;
			error_curVal = RIGHT * largeErrorMultiplier;
			fullTurnLeft();
		}


		if (!safetyMode){
		    leftSpeed = motorSpeed_base + motorSpeed_offset - motorSpeed_errorOffset;
		    rightSpeed = motorSpeed_base - motorSpeed_offset + motorSpeed_errorOffset;	

		    if (leftSpeed > 700){
		    	leftSpeed = 700;}	

		    if (rightSpeed > 700){
		    	rightSpeed = 700;}	

	        motor.speed(LEFTMOTOR_PIN, leftSpeed);
        	motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		} else if (error_curVal > 0){

			leftSpeed = speedDrop - (motorSpeed_base + motorSpeed_offset);
		    rightSpeed = (motorSpeed_base - motorSpeed_offset) - speedDrop;	
	        motor.speed(LEFTMOTOR_PIN, leftSpeed);
        	motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		} else if (error_curVal < 0){
			leftSpeed = (motorSpeed_base + motorSpeed_offset) - speedDrop;
		    rightSpeed = speedDrop - (motorSpeed_base - motorSpeed_offset);
	        motor.speed(LEFTMOTOR_PIN, leftSpeed);
        	motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		}

	    error_lastVal = error_curVal;
	    timeTicker++;
	    error_timeInErrorState++; 	

	}

	hitBrakes();
}

// Complete
void threeAndDone(){

	cleanLCD();

	int smallErrorMultiplier = 1;
	int midErrorMultiplier = 3;
	int largeErrorMultiplier = 5;

	bool collectionReady = true;
	int idolsCount = 0;

	bool safetyMode = true;
	int speedDrop = 100;

	int backwardsSpeedDrop = 100;

	int leftQRD_curVal, rightQRD_curVal, centreQRD_curVal;	

	int error_curVal = RIGHT * largeErrorMultiplier;
	int error_lastVal = error_curVal;
	int error_timeInErrorState = 0, error_slopeChangeFromLastError;	

	int proportional_curVal, derivative_curVal;	

	int motorSpeed_errorOffset;	

	int timeTicker = 0;

	int leftSpeed;
	int rightSpeed;

	countdown();

	RCServo0.write(collectorDefaultAngle);
	RCServo1.write(sweeperDefaultAngle);

	while(!backButton()){

	    leftQRD_curVal = analogRead(LEFTQRD_PIN);
	    rightQRD_curVal = analogRead(RIGHTQRD_PIN);
	    centreQRD_curVal = analogRead(CENTREQRD_PIN);

		if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = 0;
		    safetyMode = false;

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = 0;	
		    safetyMode = false;

		} else if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = LEFT * smallErrorMultiplier;
		    safetyMode = false;

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = RIGHT * smallErrorMultiplier;
		    safetyMode = false;

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal < centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = RIGHT * midErrorMultiplier;		
		    safetyMode = false;

		} else if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal < centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = LEFT * midErrorMultiplier;
		    safetyMode = false;
		    
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


	    if(collectorSwitch() && collectionReady){
	    	idolsCount++;
	    	if (idolsCount <= 3){
		    	hitBrakes();
				idolCollect(collectorDefaultAngle, sweeperDefaultAngle);
				safetyMode = true;
				if (idolsCount == 3){
					collectionReady = !collectionReady;
					error_curVal = RIGHT * largeErrorMultiplier;
					fullTurnLeft();
					motorSpeed_base = motorSpeed_base - backwardsSpeedDrop;
				}
	    	}
	    }

		if (!safetyMode){
		    leftSpeed = motorSpeed_base + motorSpeed_offset - motorSpeed_errorOffset;
		    rightSpeed = motorSpeed_base - motorSpeed_offset + motorSpeed_errorOffset;	

		    if (leftSpeed > 700){
		    	leftSpeed = 700;}	

		    if (rightSpeed > 700){
		    	rightSpeed = 700;}	

	        motor.speed(LEFTMOTOR_PIN, leftSpeed);
        	motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		} else if (error_curVal > 0){

			leftSpeed = speedDrop - (motorSpeed_base + motorSpeed_offset);
		    rightSpeed = (motorSpeed_base - motorSpeed_offset) - speedDrop;	
	        motor.speed(LEFTMOTOR_PIN, leftSpeed);
        	motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		} else if (error_curVal < 0){
			leftSpeed = (motorSpeed_base + motorSpeed_offset) - speedDrop;
		    rightSpeed = speedDrop - (motorSpeed_base - motorSpeed_offset);
	        motor.speed(LEFTMOTOR_PIN, leftSpeed);
        	motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		}

	    error_lastVal = error_curVal;
	    timeTicker++;
	    error_timeInErrorState++; 	

	}

	motorSpeed_base = EEPROM.read(MOTORSPEEDBASE_EEPROM) * 5;
	hitBrakes();
}

// Complete
void threeAndDoneBonus(){

	cleanLCD();

	int smallErrorMultiplier = 1;
	int midErrorMultiplier = 3;
	int largeErrorMultiplier = 5;

	bool collectionReady = true;
	int idolsCount = 0;

	bool safetyMode = true;
	int speedDrop = 100;

	int backwardsSpeedDrop = 100;

	int leftQRD_curVal, rightQRD_curVal, centreQRD_curVal;	

	int error_curVal = RIGHT * largeErrorMultiplier;
	int error_lastVal = error_curVal;
	int error_timeInErrorState = 0, error_slopeChangeFromLastError;	

	int proportional_curVal, derivative_curVal;	

	int motorSpeed_errorOffset;	

	int idolCollectState = 0;

	int timeTicker = 0;
	int idolCollectTicker = 0;

	int leftSpeed;
	int rightSpeed;

	countdown();

	RCServo0.write(collectorDefaultAngle);
	RCServo1.write(sweeperDefaultAngle);

	while(!backButton()){

	    leftQRD_curVal = analogRead(LEFTQRD_PIN);
	    rightQRD_curVal = analogRead(RIGHTQRD_PIN);
	    centreQRD_curVal = analogRead(CENTREQRD_PIN);

		if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = 0;
		    safetyMode = false;

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = 0;	
		    safetyMode = false;

		} else if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = LEFT * smallErrorMultiplier;
		    safetyMode = false;

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal >= centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = RIGHT * smallErrorMultiplier;
		    safetyMode = false;

		} else if (leftQRD_curVal >= leftQRD_thresh && centreQRD_curVal < centreQRD_thresh && rightQRD_curVal < rightQRD_thresh) {
		    error_curVal = RIGHT * midErrorMultiplier;		
		    safetyMode = false;

		} else if (leftQRD_curVal < leftQRD_thresh && centreQRD_curVal < centreQRD_thresh && rightQRD_curVal >= rightQRD_thresh) {
		    error_curVal = LEFT * midErrorMultiplier;
		    safetyMode = false;
		    
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







	    if(collectorSwitch() && collectionReady){
	    	idolsCount++;
	    	if (idolsCount < 3){
				idolCollectState = 1;
	    	} else {
	    		hitBrakes();
				idolCollectState = 1;
	    	}
	    }

	   	if (idolsCount <= 3 && idolCollectState == 3){
			safetyMode = true;
			collectionReady = false;
			error_curVal = RIGHT * largeErrorMultiplier;
			fullTurnLeft();
			motorSpeed_base = motorSpeed_base - backwardsSpeedDrop;
		}


	    if (idolCollectState != 0){
	    	collectionReady = false;
	    }

	    if (timeTicker == 100) {
	        timeTicker = 0;	
	        dispError(error_curVal);

	        if (idolCollectTicker == 50){
	        	idolCollectTicker = 0;	
	        	idolCollectState = idolCollect2(idolCollectState);
	        }

	        idolCollectTicker++;
	    }	






		if (!safetyMode){
		    leftSpeed = motorSpeed_base + motorSpeed_offset - motorSpeed_errorOffset;
		    rightSpeed = motorSpeed_base - motorSpeed_offset + motorSpeed_errorOffset;	

		    if (leftSpeed > 700){
		    	leftSpeed = 700;}	

		    if (rightSpeed > 700){
		    	rightSpeed = 700;}	

	        motor.speed(LEFTMOTOR_PIN, leftSpeed);
        	motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		} else if (error_curVal > 0){

			leftSpeed = speedDrop - (motorSpeed_base + motorSpeed_offset);
		    rightSpeed = (motorSpeed_base - motorSpeed_offset) - speedDrop;	
	        motor.speed(LEFTMOTOR_PIN, leftSpeed);
        	motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		} else if (error_curVal < 0){
			leftSpeed = (motorSpeed_base + motorSpeed_offset) - speedDrop;
		    rightSpeed = speedDrop - (motorSpeed_base - motorSpeed_offset);
	        motor.speed(LEFTMOTOR_PIN, leftSpeed);
        	motor.speed(RIGHTMOTOR_PIN, rightSpeed);

		}

	    error_lastVal = error_curVal;
	    timeTicker++;
	    error_timeInErrorState++; 	

	}

	motorSpeed_base = EEPROM.read(MOTORSPEEDBASE_EEPROM) * 5;
	hitBrakes();
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
	int motorSlow = 350;
	int motorFast = 700;

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
void testServos(){

	#define COLLECT_ARM 0
	#define SWEEPER 1
	#define TRMTRIGGER 2
	static int const NUM_SERVOS = 3;


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

			case TRMTRIGGER:
				LCD.print("TRM Trigger");
				if(userMenuSelection()){
					turnServo(TRMRELEASE_PIN);
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
			idolCollect(currentCollectorAngle, currentSweeperAngle);
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

		if (detectBarLeft()){
			cleanLCD();
			LCD.print("Left ZL Active");
			delay(displayDelay);
		}

		if (detectBarRight()){
			cleanLCD();
			LCD.print("Right ZL Active");
			delay(displayDelay);
		}

		delay(refreshDelay);
	}
}

// Complete
void testWinch(){

	int winchSpeed = getValueKnobPos(1400)-700;

	countdown();

	while(!backButton()){
		cleanLCD();
		winchSpeed = getValueKnobPos(1400)-700;
		motor.speed(WINCHMOTOR_PIN, winchSpeed);

		LCD.print("Winch Speed: ");
		cleanBottomLine();
		LCD.print(winchSpeed, DEC);

		delay(refreshDelay);
	}

	motor.speed(WINCHMOTOR_PIN, 0);
}

// Complete
void testIR(){

	int leftIR_curVal, rightIR_curVal;

	int minThresh = 0;
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

// Complete
void testZiplineTrigger(){

	while(!backButton()){
		cleanLCD();
		LCD.print("Waiting...");

		if(startbutton()){
			releaseBands();
		}

		delay(refreshDelay);
	}
}

// Complete
void testRaiseAndLower(){

	bool upReady = true;
	bool downReady = false;

	while(!backButton()){

		cleanLCD();
		LCD.print("Waiting...");

		if(startbutton() && upReady){
			cleanLCD();
			LCD.print("Raise Winch...");
			winchUp();
			delay(1000);
			upReady = false;
			downReady = true;
		}

		if(startbutton() && downReady){
			cleanLCD();
			LCD.print("Lower Winch...");
			winchDown();
			delay(1000);
			downReady = false;
			upReady = true;
		}

		delay(refreshDelay);
	}

	stopWinch();
}

// Complete
void testReachBar(){

	bool ready = true;

	while(!backButton()){

		cleanLCD();
		LCD.print("Waiting...");

		if(enterButton()){
			cleanLCD();
			LCD.print("Raising...");
			winchUp();

			// Replace with sensor system eventually
			while (ready){
				cleanLCD();
				LCD.print("In Position?");
				cleanBottomLine();
				LCD.print("Press <Start>");

				if (enterButton()){
					dropToZipline();
					break;
				} else if (backButton()){
					ready = false;
					break;
				}

				delay(refreshDelay);
			}

			while (ready){
				cleanLCD();
				LCD.print("Ready for Jump?");
				cleanBottomLine();
				LCD.print("Press <Start>");

				if (enterButton()){
					break;
				} else if (backButton()){
					ready = false;
					break;
				}

				delay(refreshDelay);
			}

			if (ready){
				cleanLCD();
				LCD.print("BOOM");
				cleanBottomLine();
				LCD.print("RUBBER BANDS");

				countdown();
				releaseBands();
			}
		}

		delay(refreshDelay);
	}

	stopWinch();
}

// Complete
void testWinchTimes(){

	#define WINCHUP 0
	#define WINCHDOWN 1
	static int const NUM_STATES = 2;

	int winch_upSpeed = -1023;
	int winch_downSpeed = 1023;

	int delayTime;

	while(!backButton()){

		cleanLCD();
		menuKnobPos = getMenuKnobPos(NUM_STATES);
		delayTime = getValueKnobPos(4000);

		switch(menuKnobPos){	

			case WINCHUP:
				LCD.print("Up? t=");
				LCD.print(delayTime, DEC);
				if(userMenuSelection()){
					runWinch(winch_upSpeed);
					delay(delayTime);
					stopWinch();
				}
				break;	

			case WINCHDOWN:
				LCD.print("Down? t=");
				LCD.print(delayTime, DEC);
				if(userMenuSelection()){
					runWinch(winch_downSpeed);
					delay(delayTime);
					stopWinch();
				}
				break;

			default:
				break;

		}

		delay(refreshDelay);
	}

	stopWinch();
}




// Demo Assist Programs

// Complete
void turnServo(int servoNum){


	int servoDefaultAngle = 90;
	int destinationAngle;
	int currentAngle;
	int turningRate;
	int maxServoTurnTime = 1500;
	int increment;

	countdown();

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

	int collectorMaxAngle = 0;
	int collectorMinAngle = 180;
	int sweeperMaxAngle = 52;
	static int const delayTime = 500;

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

	delay(delayTime);
	delay(delayTime);
}

// Complete
int idolCollect2(int idolCollectState){

	int collectorMaxAngle = 0;
	int collectorMinAngle = 180;
	int sweeperMaxAngle = 52;

	switch (idolCollectState){
		case 0:
			return 0;

		case 1:
			RCServo0.write(collectorMinAngle);
			return 2;	

		case 2:
			RCServo0.write(collectorMaxAngle);
			return 3;

		case 3:
			return 4;	

		case 4:
			RCServo1.write(sweeperMaxAngle);
			return 5;	

		case 5:
			RCServo1.write(sweeperDefaultAngle);
			RCServo0.write(collectorDefaultAngle);
			return 0;
	}
}

// Complete
void halfIdolCollect(int currentCollectorAngle, int currentSweeperAngle){

	int collectorMaxAngle = 0;
	int collectorMinAngle = 180;
	static int const delayTime = 500;		

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
	delay(delayTime);
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
			LCD.print("#######         ");
			LCD.setCursor(0,1);
			LCD.print("#######         ");
			break;
		case -7:
			LCD.print("#########       ");
			LCD.setCursor(0,1);
			LCD.print("#########       ");
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
			LCD.print("         #######");
			LCD.setCursor(0,1);
			LCD.print("         #######");
			break;
		case 7:
			LCD.print("       #########");
			LCD.setCursor(0,1);
			LCD.print("       #########");
			break;
	}
}

// Complete
void hitBrakes(){
	int leftBrakeSpeed = 10;
	int rightBrakeSpeed = 10;
	motor.speed(LEFTMOTOR_PIN, leftBrakeSpeed);
	motor.speed(RIGHTMOTOR_PIN, rightBrakeSpeed);	
}

// Complete
void fullTurnLeft(){

	int collectorUp = 0;
	int leftSpeed;
	int rightSpeed;
	int speedDrop = 150;

	RCServo0.write(collectorUp);

	rightSpeed = (motorSpeed_base - motorSpeed_offset) - speedDrop;
	leftSpeed = speedDrop - (motorSpeed_base + motorSpeed_offset);

	motor.speed(LEFTMOTOR_PIN, leftSpeed);
	motor.speed(RIGHTMOTOR_PIN, rightSpeed);
	delay(1500);
}

// Complete
void pivotRight(){

	int collectorUp = 0;
	int leftSpeed;
	int rightSpeed;
	int speedDrop = 150;

	RCServo0.write(collectorUp);

	leftSpeed = (motorSpeed_base + motorSpeed_offset) - speedDrop;
	rightSpeed = speedDrop - (motorSpeed_base - motorSpeed_offset);

	motor.speed(LEFTMOTOR_PIN, leftSpeed);
	motor.speed(RIGHTMOTOR_PIN, rightSpeed);
	delay(1000);
}

// Complete
void releaseBands(){

	int triggerMaxAngle = 50;
	int triggerDefaultAngle = 180;
	static int const delayTime = 500;

	cleanLCD();
	LCD.print("Trigger: ");
	LCD.setCursor(0,1);
	LCD.print("Release");
	RCServo2.write(triggerMaxAngle);
	delay(delayTime);

	cleanLCD();
	LCD.print("Trigger:");
	LCD.setCursor(0,1);
	LCD.print("Return");
	RCServo2.write(triggerDefaultAngle);
	delay(delayTime);
}

// Complete
void runWinch(int winchSpeed){
	motor.speed(WINCHMOTOR_PIN, winchSpeed);
}

// Complete
void stopWinch(){
	motor.speed(WINCHMOTOR_PIN, 0);
}

// Complete
void winchUp(){
	int winch_upSpeed = -1023;
	int winch_upTime = 1000;
	int winch_holdSpeed = -600;

	runWinch(winch_upSpeed);
	delay(winch_upTime);
	runWinch(winch_holdSpeed);
}

// Complete
void winchDown(){
	int winch_downSpeed = 1023;
	int winch_downTime = 1000;

	runWinch(winch_downSpeed);
	delay(winch_downTime);
	stopWinch();
}

// Complete
void dropToZipline(){
	stopWinch(); // Is this enough or should we run the winch backwards?

	/*
	runWinch(winch_downSpeed);
	delay(winch_ziplineDropTime);
	stopWinch();
	*/
}

// In Progress
bool ziplineDetector(){
	// use switches to detect bar
	return false;
}




// Digital Booleans

bool collectorSwitch(){
	if (digitalRead(COLLECTIONSWITCH_PIN)){
		return true;}
	return false;
}

bool detectBarLeft(){
	if (digitalRead(LEFTBARSENSOR_PIN)){
		return true;}
	return false;
}

bool detectBarRight(){
	if (digitalRead(RIGHTBARSENSOR_PIN)){
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

	LCD.setCursor(0,1);
	LCD.print("Press <Start>");

	if (enterButton()){
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

int TwelveBitVolts(double voltage){
	return (int)(voltage / 5.0 * 1023);
}

void initEEPROM(){

	// EEPROM.write(LEFTQRDTHRESH_EEPROM, ((int)300.0/5.0)); 		// leftQRD_thresh
	// EEPROM.write(RIGHTQRDTHRESH_EEPROM, ((int)300.0/5.0)); 		// rightQRD_thresh
	// EEPROM.write(CENTREQRDTHRESH_EEPROM, ((int)300.0/5.0)); 		// centreQRD_thresh

	// EEPROM.write(PROPGAIN_EEPROM, ((int)30.0/5.0)); 				// proportional_gain
	// EEPROM.write(DERVGAIN_EEPROM, ((int)0.0/5.0)); 				// derivative_gain

	// EEPROM.write(MOTORSPEEDBASE_EEPROM, ((int)325.0/5.0)); 		// motorSpeed_base
	// EEPROM.write(MOTORSPEEDOFFSET_EEPROM, ((int)25.0/5.0)); 		// motorSpeed_offset
	// EEPROM.write(MOTORSPEEDOFFSETSIGN_EEPROM, 1);				// motorSpeedOffsetSign

	// EEPROM.write(SERVODELAYTIME_EEPROM, 10); 					// servoDelayTime
	// EEPROM.write(COLLECTORDEFAULTANGLE_EEPROM, 20); 				// collectorDefaultAngle
	// EEPROM.write(SWEEPERDEFAULTANGLE_EEPROM, 20); 				// sweeperDefaultAngle

	// EEPROM.write(PROPGAINIR_EEPROM, ((int)30.0/5.0)); 			// proportional_gainIR
	// EEPROM.write(DERVGAINIR_EEPROM, ((int)0.0/5.0)); 			// derivative_gainIR


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



