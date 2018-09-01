/*
Using continuous mode for this example. Multiple sensors
can keep in this mode without affecting each other.

*/

#include <Wire.h>
#include "VL53L0X.h"

#define NUM_OF_SENSORS 9

VL53L0X sensor[NUM_OF_SENSORS];

#define STARTING_ADDRESS 0x60
uint8_t address[NUM_OF_SENSORS];

#define STARTING_CHIP_XSHUTN_PIN 2


uint32_t readDataTimer;
#define READ_DATA_DURATION 25

uint16_t distance[NUM_OF_SENSORS];

#define DEBUG_PRINT_TO_SERIAL

void setup()
{
	Serial.begin(500000);
	Serial.println("Starting sketch.");
	Wire.begin();

	for (uint8_t index = 0; index < NUM_OF_SENSORS; index++)
	{
		address[index] = STARTING_ADDRESS + index;
	}

	/* Chip shutdown in now controlled by XSHUTN, using a NMOS inverter.
	XSHUTN is not pulled up nor pulled down.
	Only the first chip is controlled by arduino pin
	*/
		
	pinMode(STARTING_CHIP_XSHUTN_PIN, OUTPUT);
	digitalWrite(STARTING_CHIP_XSHUTN_PIN, HIGH);//set chip 0 into reset mode. All subsequent chips should go into reset mode as well.
	
	delay(1000);

	for (uint8_t index = 0; index < NUM_OF_SENSORS; index++)
	{
		Serial.print("Enable chip index = ");
		Serial.println(index);
		//Bring one chip out of reset mode

		//sensor[index].writeReg(11, 0x01);//SYSTEM_INTERRUPT_CLEAR

		if (index == 0)
		{
			//First chip
			digitalWrite(STARTING_CHIP_XSHUTN_PIN, LOW);//Enable first chip
		}
		else
		{
			//Subsequent chips, index = 1,2,3,4...
			sensor[index-1].setGPIO(LOW); //Enable chips other than the first chip
		}
		delay(5);
		//delay(100);


		Serial.print("Reset I2C address to ");
		Serial.println(address[index]);
		sensor[index].setAddress(address[index]);
		//delay(100);
		

		Serial.println("Init.");
		sensor[index].init();
		delay(100);
		//blink();

		Serial.println("Set Timeout");
		sensor[index].setTimeout(500);
		delay(1000);
	}

	for (uint8_t index = 0; index < NUM_OF_SENSORS; index++)
	{
		Serial.print("Start continuous reading for chip ");
		Serial.println(index);
		sensor[index].startContinuous(20);
	}

	
}

void loop()
{
	readData();
}

void blink()
{
	while (1)
	{
		Serial.println("Set GPIO high.");
		sensor[0].setGPIO(HIGH);
		delay(2000);

		Serial.println("Set GPIO low.");
		sensor[0].setGPIO(LOW);
		delay(2000);
	}

}

void readData()
{

	/*
	High speed mode takes around 23-24 ms for each reading.
	Continuous reading takes around 1ms.
	*/


	if (millis() - readDataTimer < READ_DATA_DURATION) return;
	readDataTimer = millis();

	uint16_t num[NUM_OF_SENSORS];
	for (uint8_t index = 0; index < NUM_OF_SENSORS; index++)
	{
		distance[index] = sensor[index].readReg16Bit(sensor[index].RESULT_RANGE_STATUS + 10);
		if (distance[index] > 1000) distance[index] = 0;
	}

	#ifdef DEBUG_PRINT_TO_SERIAL
	printDataToSerial();
	#endif
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