/** \mainpage RoboComp Interfaces: Cxtrelan.ice
 *
 * \section intro_sec Introduction
* Interface for cxtrelanComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef CXTRELAN_ICE
#define CXTRELAN_ICE

#include <Camera.ice>


/** \namespace RoboCompCxtrelan
  *@brief Name space Cxtrelan
  */
module RoboCompCxtrelan
{
  struct punto
  {
    int x;
    int z;
  };

  sequence<punto> ListaPuntos;
  /** \interface Cxtrelan
  *@brief interface Cxtrelan
  */ 	
  interface Cxtrelan
  {
    void imagenReal(out RoboCompCamera::imgType real);
    void imagenBinaria(out RoboCompCamera::imgType binaria);
    void mover(float avance, float giro);
    void vectorPuntos (out ListaPuntos lp );
    void vectorPuntosImagen (out ListaPuntos lp );
//    ListaPuntos puntos();
    void setThreshold(int threashold,bool tipo);    
  };
};

#endif
