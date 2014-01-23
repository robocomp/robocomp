/** \mainpage RoboComp Interfaces: Ir.ice
 *
 * \section intro_sec Introduction
* Interface for irComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef IR_ICE
#define IR_ICE

/** \namespace RoboCompIr
  *@brief Name space Ir
  */
module RoboCompIr
{
	/** \struct TMechParams
  *@brief Struct mechanical params
  */
  struct IrConfParams
  {
		string device;
		string handler;
		int rayCount;
		int rangeCount;
		int maxAngle;
		int minAngle;
		int maxRange;
		int minRange;
		int origin;  //Este así si que no tiene sentido¡¡¡	
  };	
  
  /** \struct IrData
  *@brief Ir Data
  */
  struct IrData //esto no puede ser así
  {
	int irCount;
	int rangeCount;
	double ranges;
	//faltaria pose. Pensada¡¡¡¡¡¡¡¡¡¡¡
  };

  /** \interface Ir
  *@brief interface Ir
  */ 	
  interface Ir
  {
      IrData getIrData();
      IrConfParams getIrParams();
  };
};

#endif
