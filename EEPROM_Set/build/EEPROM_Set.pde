#include <phys253.h>          //   ***** from 253 template file
#include <LiquidCrystal.h>    //   ***** from 253 template file
#include <Servo253.h>         //   ***** from 253 template file
#include <EEPROM.h>

void setup()
{
	portMode(0, OUTPUT);      //   ***** from 253 template file
	portMode(1, INPUT);      //   ***** from 253 template file
	RCServo0.attach(RCServo0Output);
	RCServo1.attach(RCServo1Output);
	RCServo2.attach(RCServo2Output);	


// DONE

	EEPROM.write(0, ((int)300.0/5.0)); 	// leftQRD_thresh
	EEPROM.write(1, ((int)300.0/5.0)); 	// rightQRD_thresh
	EEPROM.write(2, ((int)30.0/5.0)); 	// proportional_gain
	EEPROM.write(3, ((int)0.0/5.0)); 	// derivative_gain
	EEPROM.write(4, ((int)300.0/5.0)); 	// leftMotorSpeed_base
	EEPROM.write(5, ((int)300.0/5.0)); 	// rightMotorSpeed_base
	EEPROM.write(6, ((int)700.0/5.0)); 	// motorFast
	EEPROM.write(7, ((int)350.0/5.0)); 	// motorSlow

	LCD.print("Done");
	 
}

void loop()
{

}
