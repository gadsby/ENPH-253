#include <phys253.h>         //***** from 253 template file
#include <LiquidCrystal.h>   //***** from 253 template file
#include <servo253.h>       //***** from 253 template file

void setup() {
    
}

void loop() {

	int knobval = (int)(knob(6)*180.0/1024.0);
	RCServo0.write(knobval);
	RCServo1.write(knobval);
	RCServo2.write(knobval);

	LCD.clear();
    LCD.print(knobval, DEC);

}
