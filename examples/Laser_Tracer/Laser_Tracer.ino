#include "Fuzzy_Radar.h"

#include <Servo.h>

#include "Fuzzy_Radar.h"

const uint8_t NumberOfSensors = 9;
const uint8_t XshutnControlPin = 2;
const uint8_t SeperationDegrees = 10;


//The number of sensors value has to be provided in the constructor
FuzzyRadar radar(NumberOfSensors);

const uint8_t YawServoPin = 10;
Servo yawServo;

const uint8_t PitchServoPin = 11;
Servo pitchServo;

const uint8_t LaserControlPin = 3;

const uint16_t LaserJointHeightMM = 240; //The height for laser head servo, in millimeters.

void setup()
{
	Serial.begin(115200);
	Serial.println(F("Starting sketch - Fuzzy Radar - Laser Tracer example."));

	//Two values: XshutnControlPin and SeperationDegrees need to be provided in the begin() function.
	radar.begin(XshutnControlPin, SeperationDegrees);

	yawServo.attach(YawServoPin);

	pitchServo.attach(PitchServoPin);

	pinMode(LaserControlPin, OUTPUT);
	digitalWrite(LaserControlPin, LOW);
}

void loop()
{
	radar.update();
	if (radar.available() == true)
	{
		if (radar.getDistanceMM() > 0)
		{
			uint16_t distance = radar.getDistanceMM();
			int16_t angle = radar.getAngleDegree();
			uint16_t yawValue = map(angle, -40, 40, 2000,900); //This mapping needs some trial-and-error for your servo
			yawServo.writeMicroseconds(yawValue);

			//Set minimum distance. Also reduce the distance for laser head servo tilt. Aim at the foot area.
			if (distance > 100)
			{
				distance -= 100;
			}
			else
			{
				distance = 100;
			}

			float tiltDegree = atan(float(LaserJointHeightMM) / (float)(distance)) * 57.3;
			uint16_t pitchValue = map(tiltDegree, 10, 90, 1300, 700);  //This mapping needs some trial-and-error for your servo as well
			pitchServo.writeMicroseconds(pitchValue);

			//Output data to serial window
			Serial.print(distance);
			Serial.print(" ");
			Serial.print(angle);
			Serial.print(" ");
			Serial.print(tiltDegree);
			Serial.println();

			digitalWrite(LaserControlPin, HIGH); //Turn on the laser head
		}
		else
		{
			digitalWrite(LaserControlPin, LOW); //Turn off the laser head
		}
	}
}
