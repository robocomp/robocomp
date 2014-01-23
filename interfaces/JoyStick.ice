/************************************************************************************************/
/* Interface for components providing access to Joystick                                        */
/* Possible  sources:                                                                           */
/*                                                                                              */
/*    Logitech                                                                                   */
/*                                                                                              */
/*  Dependences: Base                                                                           */
/************************************************************************************************/


/*
 * Dependences: Base
 */
#ifndef JOYSTICK_ICE
#define JOYSTICK_ICE
/** \mainpage RoboComp Interfaces: JoyStick.ice
 *
 * \section intro_sec Introduction
* Interface for joystickComp  
*                                                                   
*    Joystick control of robot movements. <br>                                 
*    PORT 10002 <br>   
*    
*/

/** \namespace RoboCompJoyStick
  *@brief Name space joystick
  */
module RoboCompJoyStick
{

    sequence<byte> JoyStickBufferedData;

    /** \interface JoyStick
    *@brief interface JoyStick
    */
     interface JoyStick
    {
        idempotent void readJoyStickBufferedData( out JoyStickBufferedData gbd );
        idempotent void writeJoyStickBufferedData( JoyStickBufferedData gbd );
    };
};

#endif
