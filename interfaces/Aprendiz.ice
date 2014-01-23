/** \mainpage RoboComp Interfaces: Aprendiz.ice
 *
 * \section intro_sec Introduction
* Interface for aprendizComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef APRENDIZ_ICE
#define APRENDIZ_ICE

/** \namespace RoboCompAprendiz
  *@brief Name space Aprendiz
  */
module RoboCompAprendiz
{
	struct coor3D 
	{
	float x; 
	float y; 
	float z;	
	};
	sequence <coor3D> poseType;
	sequence <float> matriz;

	  /**@brief Exception hardware*/
  exception HardwareFailedException { string what; };

  /** \interface Aprendiz
  *@brief interface Aprendiz
  */ 	
  interface Aprendiz
  {
	  
/*	idempotent void setCintura(coor3D pose) throws HardwareFailedException;*/
	idempotent void setWrist(coor3D pose) throws HardwareFailedException;
	idempotent void setElbowWrist(coor3D poseElbow, coor3D poseWrist, matriz m) throws HardwareFailedException;
	idempotent void getShoulders(out coor3D leftShoulder, out coor3D rightShoulder) throws HardwareFailedException;
	idempotent void getElbows(out coor3D leftElbow, out coor3D rightElbow) throws HardwareFailedException;
	//idempotent void setAngleElbowWrist(float elbow, float wrist) throws HardwareFailedException;
	//idempotent void getAngleElbowWrist(out float elbow, out float wrist) throws HardwareFailedException;
  };
};

#endif
