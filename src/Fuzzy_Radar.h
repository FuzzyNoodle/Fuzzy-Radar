/*
 Name:		Fuzzy_Radar.h
 Created:	8/17/2018 11:25:14 PM
 Author:	georgychen
 Editor:	http://www.visualmicro.com
*/

#ifndef _Fuzzy_Radar_h
#define _Fuzzy_Radar_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Wire.h>
#include "VL53L0X.h"



//#define NUM_OF_SENSORS 9

//#define STARTING_SENSOR_INDEX 0
//#define ENDING_SENSOR_INDEX 8
#define MAXIMUM_RANGE 800
#define DEVIATION_THRESHOLD 200 //signals are removed if the readings is more than this threshold value (mm)
#define STARTING_ADDRESS 0x53
#define READ_DATA_DURATION 24
#define MEAN_DISTANCE_FILTER_SHIFT 1
#define ANGLE_FILTER_SHIFT 1
#define NOISE_LENGTH 1 //For consecutive readings that length is equal of less than this value, the readings are considered noise and omitted.


#ifdef DEBUG_PRINT_RAW_DATA_BEFORE_FILTER
#endif //DEBUG_PRINT_RAW_DATA_BEFORE_FILTER
//#define DEBUG_PRINT_INITILAZATION_PROGRESS
//#define DEBUG_PRINT_RAW_DATA_BEFORE_FILTER
//#define DEBUG_PRINT_RAW_DATA_AFTER_FILTER
#define DEBUG_PRINT_DISTANCE_ANGLE





class FuzzyRadar
{
public:
	FuzzyRadar(uint8_t _numberOfSensors);
	~FuzzyRadar();
	void begin(uint8_t _xshutnPin, uint8_t _seperationDegrees);
	void update();
	int16_t getAngleDegree();
	uint16_t getDistanceMM();
	bool available();
	void clearAvailableFlag();
	
private:
	VL53L0X *sensor;
	uint8_t *address;
	uint8_t numberOfSensors;
	uint8_t xshutnPin;
	uint8_t seperationDegrees;
	int16_t *distance;
	uint8_t numberOfReadings;
	float seperation;
	float startSensorOffset;
	uint32_t total;
	int16_t meanDistance;
	uint32_t meanDistanceRegister;
	float *weight;
	float weightedTotal;
	float weightedIndex;
	int16_t angle;
	int32_t angleRegister;
	uint16_t filteredMeanDistance;
	int16_t filteredAngle;
	uint8_t readingCounter;
	uint32_t readDataTimer;
	uint8_t startingSensorIndex;
	uint8_t endingSensorIndex;
	int16_t maximumRange;

	void readData();
	void calculateData();
	void printRawData();
	void resetDataValues();
	void calculateMeanDistance();
	bool hasNewData;
	void clearDataValues();
	
};

#endif

