/*
Raw Data Serial Output

This sketch is used to output distance data to serial windows.
The distance is in millimeters. 

*/

#include <Wire.h>
#include "VL53L0X.h"


//User should change the number of sensors here.
#define NUM_OF_SENSORS 9



VL53L0X sensor[NUM_OF_SENSORS];

#define STARTING_ADDRESS 0x53
uint8_t address[NUM_OF_SENSORS];

#define STARTING_CHIP_XSHUTN_PIN 2

uint32_t readDataTimer;
#define READ_DATA_DURATION 24

uint16_t distance[NUM_OF_SENSORS];
#define DEBUG_PRINT_TO_SERIAL

#define MAX_RANGE 1000

void setup()
{
	Serial.begin(115200);
	Serial.println("Starting sketch - Fuzzy Radar - Raw Data Serial Output.");
	Wire.begin();

	//Initialize the I2C address array.
	uint8_t addressOffset = 0; //Avoid using default address 0x52 as new address.
	for (uint8_t index = 0; index < NUM_OF_SENSORS; index++)
	{
		if ((STARTING_ADDRESS + index) == 0x52) addressOffset = 1;
		address[index] = STARTING_ADDRESS + index + addressOffset;
	}

	/* Chip shutdown in now controlled by XSHUTN, using a NMOS inverter.
	XSHUTN is not pulled up nor pulled down.
	Only the first chip is controlled by arduino pin
	*/

	Serial.println("Set chip 0 into reset mode.");
	pinMode(STARTING_CHIP_XSHUTN_PIN, OUTPUT);
	digitalWrite(STARTING_CHIP_XSHUTN_PIN, HIGH);//set chip 0 into reset mode. All subsequent chips should go into reset mode as well.
	Serial.println("All status LEDs should be off.");
	//delay(2000);


	Serial.println("Now configuring the sensors. LED should light up one by one.");
	//delay(1000);

	for (uint8_t index = 0; index < NUM_OF_SENSORS; index++)
	{
		Serial.print("Configuring chip ");
		Serial.println(index);

		//Bring one chip out of reset mode
		if (index == 0)
		{
			//First chip
			digitalWrite(STARTING_CHIP_XSHUTN_PIN, LOW);//Enable first chip
		}
		else
		{
			//Subsequent chips, index = 1,2,3,4...
			sensor[index - 1].setGPIO(LOW); //Enable chips other than the first chip
		}
		delay(5);//Required for VL53L0X firmware booting.

		Serial.print("  - Reset I2C address to ");
		Serial.println(address[index]);
		sensor[index].setAddress(address[index]);

		Serial.println("  - Initialize the sensor.");
		sensor[index].init();
		sensor[index].setTimeout(500);

		//delay(1000);
	}
	Serial.println("Radar array configuration completed.");


	//Start continuous reading mode.
	for (uint8_t index = 0; index < NUM_OF_SENSORS; index++)
	{
		Serial.print("Start continuous ranging mode for chip ");
		Serial.println(index);
		sensor[index].startContinuous(20);
	}
}

void loop()
{
	if (millis() - readDataTimer >= READ_DATA_DURATION)
	{
		//The time required for new data is around 21-22 ms.
		readDataTimer = millis();
		readData();
		#ifdef DEBUG_PRINT_TO_SERIAL
		printDataToSerial();
		#endif
	}
}

void readData()
{
	for (uint8_t index = 0; index < NUM_OF_SENSORS; index++)
	{
		distance[index] = sensor[index].readReg16Bit(sensor[index].RESULT_RANGE_STATUS + 10);
		if (distance[index] > MAX_RANGE) distance[index] = 0;
	}
}

void printDataToSerial()
{
	for (uint8_t index = 0; index < NUM_OF_SENSORS; index++)
	{
		if (distance[index] != 0)
		{
			Serial.print((distance[index] < 10 ? "0" : ""));
			Serial.print((distance[index] < 100 ? "0" : ""));
			Serial.print((distance[index] < 1000 ? "0" : ""));
			Serial.print(distance[index]);
		}
		else
		{
			Serial.print("----");
		}
		Serial.print(" ");
	}
	Serial.println();
}