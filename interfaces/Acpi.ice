#ifndef ACPI_ICE
#define ACPI_ICE
/** \mainpage RoboComp Interfaces: Acpi.ice
 *
 * \section intro_sec Introduction
* Interface for acpi
*                                                                   
* returns a struct with significant data on the laptop on board the robot, such as, charging the battery, the wifi signal strength ...
*                                                                                              
*    PORT 10025 <br>   
*/
/** \namespace RoboCompAcpi
  *@brief name space acpi
  */
module RoboCompAcpi {

  /** \struct Data
  *@brief Struct data 
  */
  struct Data
  {
    int battery;
    int tcpu;
    int tchipset;
    byte charging;
    int wifiLink;
    int wifiLevel;
    int wifiNoise;
  };

  /** \interface Acpi
  *@brief interface Acpi
  */
  interface Acpi {
    idempotent string getObjectUUID();
    Data getData();
  };
};

#endif
