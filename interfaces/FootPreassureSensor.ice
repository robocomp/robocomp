/************************************************************************************************/
/* Interface for components providing access to sensor                                        */
/* Possible  sources:                                                                           */
/*                                                                                              */
/*    Logitech                                                                                   */
/*                                                                                              */
/*  Dependences: Base                                                                           */
/************************************************************************************************/


/*
 * Dependences: Base
 */
#ifndef SENSOR_ICE
#define SENSOR_ICE
/** \mainpage RoboComp Interfaces: sensor.ice
 *
 * \section intro_sec Introduction
* Interface for sensorComp  
*                                                                   
*    sensor   
*    
*/

/** \namespace RoboCompSensor
  *@brief Name space sensor
  */
module RoboCompFootPreassureSensor
{
    dictionary<string,int> Buffer;

    interface FootPreassureSensor
    {
        Buffer readSensors();
	int readSensor(string name);
    };
};

#endif
