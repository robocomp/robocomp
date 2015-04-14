#ifndef ULTRASOUND_ICE
#define ULTRASOUND_ICE

module RoboCompUltrasound
{
	exception HardwareFailedException{ string what; };
	exception UnknownSensorException{ string what; };

	struct BusParams
	{
		int numSensors;
		int baudRate;
		int basicPeriod;
	};
	struct SensorData
	{
		float dist;
		float ang;
	};

	dictionary<string, SensorData> SensorMap;

	interface Ultrasound
	{
		BusParams getBusParams();
		SensorData getSensorData(string sensor);
		string getAllSensorData();

    	};
};

#endif
