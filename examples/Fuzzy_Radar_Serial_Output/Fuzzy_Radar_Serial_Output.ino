/*
This sketch uses the FuzzyRadar class. The class has incorporated
the initialization process for multiple sensors. It also contains
filtering algorithm, and output two values: distance in millimeters,
and angle in degrees.


*/


#include "Fuzzy_Radar.h"

const uint8_t NumberOfSensors = 9;
const uint8_t XshutnControlPin = 2;
const float SeperationDegrees = 10;


//The number of sensors value has to be provided in the constructor
FuzzyRadar radar(NumberOfSensors);


void setup()
{
	Serial.begin(115200);
	Serial.println(F("Starting sketch - Fuzzy Radar Serial Output example."));

	//Two values: XshutnControlPin and SeperationDegrees need to be provided in the begin() function.
	radar.begin(XshutnControlPin, SeperationDegrees);
}

void loop()
{
	radar.update(); //This member function has to be called constantly in the loop()

	/*
	Use available() member function to check if there is any new data.
	Calling either getDistanceMM() or getAngleDegree() will clear internal available flag.
	Ranging time for VL53L0X in fast continuous mode is around 21-23 milliseconds.
	Internal calculation period is set at 24 milliseconds, so the radar scan frequency is approximately 42 Hz.
	*/

	if (radar.available() == true)
	{
		//Has new data
		if (radar.getDistanceMM() > 0)
		{
			uint16_t distance = radar.getDistanceMM();
			int16_t angle = radar.getAngleDegree();
			Serial.print(F("Distance = "));
			Serial.print((distance < 10 ? "0" : ""));
			Serial.print((distance < 100 ? "0" : ""));
			Serial.print((distance < 1000 ? "0" : ""));
			Serial.print(distance);
			Serial.print(F(" "));
			Serial.print(F("Angle = "));
			if (angle >= 0)
			{
				Serial.print(" ");
				Serial.print((angle < 10 ? "0" : ""));
				Serial.print((angle < 100 ? "0" : ""));
				Serial.print(angle);
			}
			else
			{
				Serial.print(F("-"));
				Serial.print((angle > -10 ? "0" : ""));
				Serial.print((angle > -100 ? "0" : ""));
				Serial.print(abs(angle));
			}

			Serial.println();
		}
		else
		{
			Serial.println(F("No object detected."));
		}
	}
}
