/*
 Name:		Fuzzy_Radar.cpp
 Created:	8/17/2018 11:25:14 PM
 Author:	georgychen
 Editor:	http://www.visualmicro.com
*/

#include "Fuzzy_Radar.h"


FuzzyRadar::FuzzyRadar(uint8_t _numberOfSensors)
	:sensor(new VL53L0X[_numberOfSensors])
	,address(new uint8_t[_numberOfSensors])
	,distance(new int16_t[_numberOfSensors])
	,weight(new float[_numberOfSensors])
{
	numberOfSensors = _numberOfSensors;

}
FuzzyRadar::~FuzzyRadar() 
{
	free(address);
	address = NULL;

	free(distance);
	distance = NULL;

	free(weight);
	weight = NULL;
}

void FuzzyRadar::begin(uint8_t _xshutnPin, uint8_t _seperationDegrees)
{
	xshutnPin = _xshutnPin;
	seperationDegrees = _seperationDegrees;
	startingSensorIndex = 0;
	endingSensorIndex = numberOfSensors-14;
	maximumRange = MAXIMUM_RANGE;

	Wire.begin();

	
	//Initialize the I2C address array.
	uint8_t addressOffset = 0; //Avoid using default address 0x52 as new address.
	for (uint8_t index = 0; index < numberOfSensors; index++)
	{
		if ((STARTING_ADDRESS + index) == 0x52) addressOffset = 1;
		address[index] = STARTING_ADDRESS + index + addressOffset;
		if (address[index] > 127)
		{
			address[index] -= 127;
		}
	}



	/* Chip shutdown in now controlled by XSHUTN, using a NMOS inverter.
	XSHUTN is not pulled up nor pulled down.
	Only the first chip is controlled by arduino pin
	*/

	#ifdef DEBUG_PRINT_INITILAZATION_PROGRESS
	Serial.println(F("Set chip 0 into reset mode."));
	#endif //DEBUG_PRINT_INITILAZATION_PROGRESS

	pinMode(xshutnPin, OUTPUT);
	digitalWrite(xshutnPin, HIGH);//set chip 0 into reset mode. All subsequent chips should go into reset mode as well.

	#ifdef DEBUG_PRINT_INITILAZATION_PROGRESS
	Serial.println(F("All status LEDs should be off."));
	Serial.println(F("Now configuring the sensors. LED should light up one by one."));
	#endif //DEBUG_PRINT_INITILAZATION_PROGRESS

	//delay(1000);

	for (uint8_t index = 0; index < numberOfSensors; index++)
	{
		#ifdef DEBUG_PRINT_INITILAZATION_PROGRESS
		Serial.print(F("Configuring chip "));
		Serial.println(index);
		#endif //DEBUG_PRINT_INITILAZATION_PROGRESS


		//Bring one chip out of reset mode
		if (index == 0)
		{
			//First chip
			digitalWrite(xshutnPin, LOW);//Enable first chip
		}
		else
		{
			//Subsequent chips, index = 1,2,3,4...
			sensor[index - 1].setGPIO(LOW); //Enable chips other than the first chip
		}
		delay(5);//Required for VL53L0X firmware booting (1.2ms max).

		#ifdef DEBUG_PRINT_INITILAZATION_PROGRESS
		Serial.print(F("  - Reset I2C address to "));
		Serial.println(address[index]);
		#endif //DEBUG_PRINT_INITILAZATION_PROGRESS

		sensor[index].setAddress(address[index]);

		#ifdef DEBUG_PRINT_INITILAZATION_PROGRESS
		Serial.println(F("  - Initialize the sensor."));
		#endif //DEBUG_PRINT_INITILAZATION_PROGRESS

		sensor[index].init();
		sensor[index].setTimeout(500);

		//delay(1000);
	}

	#ifdef DEBUG_PRINT_INITILAZATION_PROGRESS
	Serial.println(F("Radar array configuration completed."));
	#endif //DEBUG_PRINT_INITILAZATION_PROGRESS



	//Start continuous reading mode.
	for (uint8_t index = 0; index < numberOfSensors; index++)
	{
		#ifdef DEBUG_PRINT_INITILAZATION_PROGRESS
		Serial.print(F("Start continuous ranging mode for chip "));
		Serial.println(index);
		#endif //DEBUG_PRINT_INITILAZATION_PROGRESS

		sensor[index].startContinuous(20);
	}

	seperation = 10;
	startSensorOffset = -seperation * 4;

	hasNewData = false;
}

void FuzzyRadar::update()
{
	readData();
}

int16_t FuzzyRadar::getAngleDegree()
{
	hasNewData = false;
	return filteredAngle;
}

uint16_t FuzzyRadar::getDistanceMM()
{
	hasNewData = false;
	return filteredMeanDistance;
}

void FuzzyRadar::readData()
{
	if (millis() - readDataTimer < READ_DATA_DURATION) return;
	readDataTimer = millis();


	for (uint8_t index = startingSensorIndex; index <= endingSensorIndex; index++)
	{
		distance[index] = sensor[index].readReg16Bit(sensor[index].RESULT_RANGE_STATUS + 10);
		if (distance[index] > maximumRange) distance[index] = 0;
	}


	calculateData();
}

void FuzzyRadar::calculateData()
{
	resetDataValues();
	calculateMeanDistance();

	#ifdef DEBUG_PRINT_RAW_DATA_BEFORE_FILTER
	printRawData();
	#endif //DEBUG_PRINT_RAW_DATA_BEFORE_FILTER

	if (meanDistance == 0)
	{
		readingCounter = 0;
	}

	if (meanDistance > 0)
	{
		//Deviation removal: remove the data that are too far away from mean value.
		bool recalculateMeanDistance = false;
		for (uint8_t index = startingSensorIndex; index <= endingSensorIndex; index++)
		{
			if ((distance[index]>0) && (abs(distance[index] - meanDistance) > DEVIATION_THRESHOLD))
			{
				distance[index] = 0;
				recalculateMeanDistance = true;
			}
		}
		if (recalculateMeanDistance == true)
		{

			resetDataValues();
			calculateMeanDistance();
		}
		//Noise removal
		if (readingCounter < NOISE_LENGTH)
		{
			readingCounter++;
			clearDataValues();
		}
	}

	
	if (meanDistance > 0)
	{

		/*
		Primary target filtering:
		Only the group with most number of sensor readings remains.
		If there are multiple groups with same number of readings, the closet target remains.

		1. Scan sequentially
		2. Note down the starting index for a group
		3. Note down the length for end of group (zero or end of sensor)
		4. Note down the mean of the group
		5. Replace the starting index/length to the stored primary if the new group length is larger.
		6. If the length is the same as stored, compare the mean. Replace if the new group is closer.

		*/

		uint8_t currentGroupReadingCounter = 0;
		uint8_t currentGroupLength = 0;
		uint8_t currentGroupStartingIndex = 0;
		uint16_t currentGroupTotal = 0;
		uint16_t currentGroupMeanDistance = 0;
		uint8_t primaryGroupLength = 0;
		uint8_t primaryGroupStartingIndex = 0;
		uint16_t primaryGroupMeanDistance = 0;
		for (uint8_t index = startingSensorIndex; index <= endingSensorIndex; index++) //scanning
		{
			if (distance[index] > 0)
			{
				if (currentGroupReadingCounter > 0)
				{
					//continue a group
					//Serial.print(" CG=");
					//Serial.print(index);
				}
				else
				{
					//starting a new group
					currentGroupStartingIndex = index;
					//Serial.print(" NG=");
					//Serial.print(index);

				}
				currentGroupReadingCounter++;
			}

			if (currentGroupReadingCounter > 0) //a group has started
			{
				//Serial.print(" GS=");
				//Serial.print(index);
				if ((distance[index] == 0) || (index == endingSensorIndex))
				{
					//ending a group, or end of scanning
					//calculate length
					if (distance[index] == 0)
					{
						//zero reading, no matter end of scanning or not
						currentGroupLength = index - currentGroupStartingIndex;
						//Serial.print(" ZR=");
						//Serial.print(index);
					}
					else
					{
						//available reading, but end of scanning
						currentGroupLength = index - currentGroupStartingIndex + 1;
						//Serial.print(" ES=");
						//Serial.print(index);

					}
					//Serial.print(" GL=");
					//Serial.print(currentGroupLength);
					//calculate mean
					currentGroupTotal = 0;
					for (uint8_t groupIndex = currentGroupStartingIndex; groupIndex <= index; groupIndex++)
					{
						currentGroupTotal += distance[groupIndex];
					}
					currentGroupMeanDistance = currentGroupTotal / currentGroupLength;

					//replace primary target if required
					if (currentGroupLength > primaryGroupLength)
					{
						//get the largest target
						primaryGroupLength = currentGroupLength;
						primaryGroupStartingIndex = currentGroupStartingIndex;
						primaryGroupMeanDistance = currentGroupMeanDistance;
						//Serial.print(" LT=");
						//Serial.print(index);
					}
					else if (currentGroupLength == primaryGroupLength)
					{
						//Serial.print(" CM=");
						//Serial.print(currentGroupMeanDistance);
						//Serial.print(" PM=");
						//Serial.print(primaryGroupMeanDistance);

						if (currentGroupMeanDistance < primaryGroupMeanDistance)
						{
							//same size, get the closer target
							primaryGroupLength = currentGroupLength;
							primaryGroupStartingIndex = currentGroupStartingIndex;
							primaryGroupMeanDistance = currentGroupMeanDistance;
							//Serial.print(" CL=");
							//Serial.print(index);
						}

					}

					//reset values for next scanning
					currentGroupReadingCounter = 0;
					currentGroupLength = 0;
					currentGroupStartingIndex = 0;
					currentGroupTotal;
					currentGroupMeanDistance = 0;
				}

			} //if (currentGroupReadingCounter > 0) //a group has started //if (currentGroupReadingCounter > 0) //a group has started


		} //for (uint8_t index = startingSensorIndex; index <= endingSensorIndex; index++) //scanning


		if (primaryGroupLength != 0)
		{
			//remove non-primary data
			for (uint8_t index = startingSensorIndex; index <= endingSensorIndex; index++)
			{
				if ((index<primaryGroupStartingIndex) || (index>(primaryGroupStartingIndex + primaryGroupLength - 1)))
				{
					distance[index] = 0;
				}
			}
			calculateMeanDistance();
		}

		//get weight for each detected sensor
		for (uint8_t index = startingSensorIndex; index <= endingSensorIndex; index++)
		{
			if (distance[index] > 0)
			{
				weight[index] = (float)meanDistance / (float)distance[index];

				weightedTotal += weight[index] * index;
			}

		}

		weightedIndex = weightedTotal / numberOfReadings;

		angle = (weightedIndex * seperation + startSensorOffset);

	}


	if (meanDistanceRegister == 0)
	{
		//fill the filter with first sample value
		meanDistanceRegister = meanDistance << MEAN_DISTANCE_FILTER_SHIFT;
		filteredMeanDistance = meanDistance;

		angleRegister = angle << ANGLE_FILTER_SHIFT;
		filteredAngle = angle;
	}
	else
	{
		if (meanDistance > 0)
		{
			//non-zero reading, filter the data
			meanDistanceRegister = meanDistanceRegister - (meanDistanceRegister >> MEAN_DISTANCE_FILTER_SHIFT) + meanDistance;
			filteredMeanDistance = meanDistanceRegister >> MEAN_DISTANCE_FILTER_SHIFT;

			angleRegister = angleRegister - (angleRegister >> ANGLE_FILTER_SHIFT) + angle;
			filteredAngle = angleRegister >> ANGLE_FILTER_SHIFT;
		}
		else
		{
			//zero reading, flush the filter register
			meanDistanceRegister = 0;
			filteredMeanDistance = 0;

			angleRegister = 0;
			filteredAngle = 0;
		}
	}

	
	#ifdef DEBUG_PRINT_RAW_DATA_AFTER_FILTER
	printRawData();
	#endif //DEBUG_PRINT_RAW_DATA_AFTER_FILTER

	#ifdef DEBUG_PRINT_DISTANCE_ANGLE
	Serial.print(" ");
	Serial.print("Distance = ");
	Serial.print((filteredMeanDistance < 10 ? "0" : ""));
	Serial.print((filteredMeanDistance < 100 ? "0" : ""));
	Serial.print((filteredMeanDistance < 1000 ? "0" : ""));
	Serial.print(filteredMeanDistance);
	Serial.print(" ");
	Serial.print("Angle = ");
	if (filteredAngle >= 0)
	{
		Serial.print(" ");
		Serial.print((filteredAngle < 10 ? "0" : ""));
		Serial.print((filteredAngle < 100 ? "0" : ""));
		Serial.print(filteredAngle);
	}
	else
	{
		Serial.print("-");
		Serial.print((filteredAngle > -10 ? "0" : ""));
		Serial.print((filteredAngle > -100 ? "0" : ""));
		Serial.print(abs(filteredAngle));
	}

	Serial.println();
	#endif //DEBUG_PRINT_DISTANCE_ANGLE

	hasNewData = true;
}

void FuzzyRadar::printRawData()
{
	Serial.print(" [");
	for (uint8_t index = startingSensorIndex; index <= endingSensorIndex; index++)
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
	Serial.print("(");
	Serial.print((meanDistance < 10 ? "0" : ""));
	Serial.print((meanDistance < 100 ? "0" : ""));
	Serial.print((meanDistance < 1000 ? "0" : ""));
	Serial.print(meanDistance);
	Serial.print(")]");

}

void FuzzyRadar::resetDataValues()
{
	numberOfReadings = 0;
	total = 0;
	meanDistance = 0;
	weightedTotal = 0;
	weightedIndex = 0;
	angle = 0;
}

void FuzzyRadar::calculateMeanDistance()
{
	numberOfReadings = 0;
	total = 0;
	for (uint8_t index = startingSensorIndex; index <= endingSensorIndex; index++)
	{
		if (distance[index] > 0)
		{
			numberOfReadings++;
			total += distance[index];
		}
	}

	if (numberOfReadings > 0)
	{
		meanDistance = total / numberOfReadings;
	}
}

bool FuzzyRadar::available()
{
	return hasNewData;
}

void FuzzyRadar::clearAvailableFlag()
{
	hasNewData = false;
}

void FuzzyRadar::clearDataValues()
{
	//Serial.print("clearDataValues()");
	resetDataValues();
	for (uint8_t index = startingSensorIndex; index <= endingSensorIndex; index++)
	{
		distance[index] = 0;
	}
}