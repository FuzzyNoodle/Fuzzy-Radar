/*
 Name:		Fuzzy_Radar.cpp
 Created:	8/17/2018 11:25:14 PM
 Author:	georgychen
 Editor:	http://www.visualmicro.com
*/

#include "Fuzzy_Radar.h"


FuzzyRadar::FuzzyRadar()
{

}

void FuzzyRadar::begin()
{
	for (uint8_t index = 0; index < NUM_OF_SENSORS; index++)
	{
		address[index] = STARTING_ADDRESS + index;
		xshutPin[index] = STARTING_XSHUT_PIN + index;
	}

	//XSHUT pin is not protected. Do not set output high (5v) to the XSHUT pin.
	//Use pinMode input instead, to release and pull-high by the Chip Vdd.
	for (uint8_t index = 0; index < NUM_OF_SENSORS; index++)
	{
		digitalWrite(xshutPin[index], LOW);
		pinMode(xshutPin[index], OUTPUT);
	}
	delay(1);

	for (uint8_t index = 0; index < NUM_OF_SENSORS; index++)
	{
		pinMode(xshutPin[index], INPUT); //Bring the selected sensor out of shutdown mode
		delay(5);
		sensor[index].setAddress(address[index]); //Reset this sensor i2c address
		sensor[index].init();
		sensor[index].setTimeout(500);
		sensor[index].startContinuous(20);
	}

	data.seperation = 16.67;
	data.startSensorOffset = -data.seperation * 4.5;

	hasNewData = false;
}

void FuzzyRadar::update()
{
	readData();
}

int16_t FuzzyRadar::getAngleDegree()
{
	hasNewData = false;
	return data.filteredAngle;
}

int16_t FuzzyRadar::getDistanceMM()
{
	hasNewData = false;
	return data.filteredMeanDistance;
}

void FuzzyRadar::readData()
{
	if (millis() - readDataTimer < READ_DATA_DURATION) return;
	readDataTimer = millis();


	for (uint8_t index = STARTING_SENSOR_INDEX; index <= ENDING_SENSOR_INDEX; index++)
	{
		data.distance[index] = sensor[index].readReg16Bit(sensor[index].RESULT_RANGE_STATUS + 10);
		if (data.distance[index] > MAXIMUM_RANGE) data.distance[index] = 0;
	}


	calculateData();
}

void FuzzyRadar::calculateData()
{
	resetDataValues();
	calculateMeanDistance();


	if (data.meanDistance > 0)
	{
		//Deviation removal: remove the data that are too far away from mean value.
		bool recalculateMeanDistance = false;
		for (uint8_t index = STARTING_SENSOR_INDEX; index <= ENDING_SENSOR_INDEX; index++)
		{
			if ((data.distance[index]>0) && (abs(data.distance[index] - data.meanDistance) > DEVIATION_THRESHOLD))
			{
				data.distance[index] = 0;
				recalculateMeanDistance = true;
			}
		}
		if (recalculateMeanDistance == true)
		{

			resetDataValues();
			calculateMeanDistance();
		}
	}

	//printRawData();
	if (data.meanDistance > 0)
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
		for (uint8_t index = STARTING_SENSOR_INDEX; index <= ENDING_SENSOR_INDEX; index++) //scanning
		{
			if (data.distance[index] > 0)
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
				if ((data.distance[index] == 0) || (index == ENDING_SENSOR_INDEX))
				{
					//ending a group, or end of scanning
					//calculate length
					if (data.distance[index] == 0)
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
						currentGroupTotal += data.distance[groupIndex];
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


		} //for (uint8_t index = STARTING_SENSOR_INDEX; index <= ENDING_SENSOR_INDEX; index++) //scanning


		if (primaryGroupLength != 0)
		{
			//remove non-primary data
			for (uint8_t index = STARTING_SENSOR_INDEX; index <= ENDING_SENSOR_INDEX; index++)
			{
				if ((index<primaryGroupStartingIndex) || (index>(primaryGroupStartingIndex + primaryGroupLength - 1)))
				{
					data.distance[index] = 0;
				}
			}
			calculateMeanDistance();
		}

		//get weight for each detected sensor
		for (uint8_t index = STARTING_SENSOR_INDEX; index <= ENDING_SENSOR_INDEX; index++)
		{
			if (data.distance[index] > 0)
			{
				data.weight[index] = (float)data.meanDistance / (float)data.distance[index];

				data.weightedTotal += data.weight[index] * index;
			}

		}

		data.weightedIndex = data.weightedTotal / data.numberOfReadings;

		data.angle = data.weightedIndex * data.seperation + data.startSensorOffset;

	}


	if (data.meanDistanceRegister == 0)
	{
		//fill the filter with first sample value
		data.meanDistanceRegister = data.meanDistance << MEAN_DISTANCE_FILTER_SHIFT;
		data.filteredMeanDistance = data.meanDistance;

		data.angleRegister = data.angle << ANGLE_FILTER_SHIFT;
		data.filteredAngle = data.angle;
	}
	else
	{
		if (data.meanDistance > 0)
		{
			//non-zero reading, filter the data
			data.meanDistanceRegister = data.meanDistanceRegister - (data.meanDistanceRegister >> MEAN_DISTANCE_FILTER_SHIFT) + data.meanDistance;
			data.filteredMeanDistance = data.meanDistanceRegister >> MEAN_DISTANCE_FILTER_SHIFT;

			data.angleRegister = data.angleRegister - (data.angleRegister >> ANGLE_FILTER_SHIFT) + data.angle;
			data.filteredAngle = data.angleRegister >> ANGLE_FILTER_SHIFT;
		}
		else
		{
			//zero reading, flush the filter register
			data.meanDistanceRegister = 0;
			data.filteredMeanDistance = 0;

			data.angleRegister = 0;
			data.filteredAngle = 0;
		}
	}

	





	#ifdef DEBUG_SERIAL
	printRawData();
	#endif //DEBUG_SERIAL

	hasNewData = true;
}

void FuzzyRadar::printRawData()
{
	for (uint8_t index = STARTING_SENSOR_INDEX; index <= ENDING_SENSOR_INDEX; index++)
	{
		if (data.distance[index] != 0)
		{
			Serial.print((data.distance[index] < 10 ? "0" : ""));
			Serial.print((data.distance[index] < 100 ? "0" : ""));
			Serial.print((data.distance[index] < 1000 ? "0" : ""));
			Serial.print(data.distance[index]);
		}
		else
		{
			Serial.print("----");
		}
		Serial.print(" ");
	}
	Serial.print(data.numberOfReadings);
	Serial.print(" ");
	Serial.print(data.filteredMeanDistance);
	Serial.print(" ");

	Serial.print(data.filteredAngle);
	Serial.print(" ");
	Serial.println();
}

void FuzzyRadar::resetDataValues()
{
	data.numberOfReadings = 0;
	data.total = 0;
	data.meanDistance = 0;
	data.weightedTotal = 0;
	data.weightedIndex = 0;
	data.angle = 0;
}

void FuzzyRadar::calculateMeanDistance()
{
	data.numberOfReadings = 0;
	data.total = 0;
	for (uint8_t index = STARTING_SENSOR_INDEX; index <= ENDING_SENSOR_INDEX; index++)
	{
		if (data.distance[index] > 0)
		{
			data.numberOfReadings++;
			data.total += data.distance[index];
		}
	}

	if (data.numberOfReadings > 0)
	{
		data.meanDistance = data.total / data.numberOfReadings;
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