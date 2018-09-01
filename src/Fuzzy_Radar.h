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



#define NUM_OF_SENSORS 10

#define STARTING_SENSOR_INDEX 0
#define ENDING_SENSOR_INDEX 9
#define MAXIMUM_RANGE 600
#define DEVIATION_THRESHOLD 200 //signals are removed if the readings is more than this threshold value (mm)
#define STARTING_ADDRESS 0x52
#define STARTING_XSHUT_PIN 2
#define READ_DATA_DURATION 25
#define MEAN_DISTANCE_FILTER_SHIFT 1
#define ANGLE_FILTER_SHIFT 1

//#define DEBUG_SERIAL
#ifdef DEBUG_SERIAL
#endif //DEBUG_SERIAL


struct DataSet
{
	int16_t distance[NUM_OF_SENSORS];
	uint8_t numberOfReadings;
	float seperation;
	float startSensorOffset;
	uint32_t total;
	int16_t meanDistance;
	float weight[NUM_OF_SENSORS];
	float weightedTotal;
	float weightedIndex;
	int16_t angle;
	uint32_t meanDistanceRegister;
	uint16_t filteredMeanDistance;
	int32_t angleRegister;
	int16_t filteredAngle;

};



class FuzzyRadar
{
public:
	FuzzyRadar();
	void begin();
	void update();
	int16_t getAngleDegree();
	int16_t getDistanceMM();
	bool available();
	void clearAvailableFlag();
	void printRawData();
private:
	VL53L0X sensor[NUM_OF_SENSORS];
	uint8_t address[NUM_OF_SENSORS];
	uint8_t xshutPin[NUM_OF_SENSORS];
	uint32_t readDataTimer;
	DataSet data;
	void readData();
	void calculateData();
	
	void resetDataValues();
	void calculateMeanDistance();
	bool hasNewData;

};

#endif

