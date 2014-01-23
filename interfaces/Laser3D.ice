#ifndef LASER3D_ICE
#define LASER3D_ICE

// Hyoko URG Laser parameters (04LX)
//  Max range 4094 mm.
//  Resolution +-1mm
//  Min Dist 20mm
//  Total sweeping angle 240 degrees
//  Angular resolution 768 degrees

/** \mainpage RoboComp Interfaces: Laser.ice
 *
 * \section intro_sec Introduction
* Interface for laser3DComp  
*                                                                   
* Laser acquisition and preprocessing. Supports both Hokuyo URG-04LX and Gazebo laser interfaces
*
* \section features_sec Features Hyoko URG Laser parameters
*  Max range 4094 mm. <br>
*  Resolution +-1mm <br>
*  Min Dist 20mm <br>
*  Total sweeping angle 240 degrees <br>
*  Angular resolution 768 degrees <br>
*
*    PORT 10003 <br>   
*/

/** \namespace RoboCompLaser
  *@brief Name space laser3D
  */
module RoboCompLaser3D
{

  /** \struct TCamParams
  *@brief struct camera params
  */
  struct Laser3DConfData 
  {
     string driver;     // Underlying hardware: HokuyoURG/Gazebo
     string device;     // Laser device: hardware dependent
     int staticConf;    // 0 means it has a dynamic laser configuration
     int maxMeasures;   // Total number of possible measures (Laser specific)
     int maxDegrees;    // Angular measurement range degrees (Laser specific)
     int maxRange;      // Maximun distance measurable mm (Laser specific)
     int minRange;      // Minimun distance measurable mm (Laser specific)
     int iniRange;      // (0-totalRange) Initial measuring position
     int endRange;      // (0-totalRange) End measuring position
     int cluster;       // (0-99) Number of neighboor positions grouped
     int sampleRate;    // Adquisition period in msecs
     float angleRes;    // Angle resolution
     float angleIni;    // Initial angle
  };
  
  /** \struct TMeasure
  *@brief Data for one measure of the 3D laser: range,sweepAngle,tiltAngle
  */
  struct TMeasure
  {
    short range;
    float sweepAngle;
    float tiltAngle;
  };


  /** \struct TLaserData
  *@brief Set of measures for a complete laser sweep, plus an index of measurement in the tilting sequence and  a timestamp
  */
  
  sequence<TMeasure> Data;

  struct TLaserData
  {
      Data vector;
      int seqNumber;
      string timeStamp;
  };

  /** \interface Laser
  *@brief interface Laser functions
  */	
  interface Laser3D
  {
    /// Sends the laser to minAngle position and programs a tilt sweep between maxAng and minAng. Returns the number of scans that the command will produce and scanning period.
    /// In continuous mode (once=false) nScans returns -1;
    void moveToAndSweepAtSpeed( float minAng, float maxAng, float speed , bool once, int numScans,out int nScans, out float period);
    void stop();
    TLaserData getLaserData();
    Laser3DConfData getLaserConfData();
  };

};

#endif
