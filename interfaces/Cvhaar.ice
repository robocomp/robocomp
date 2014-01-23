/** \mainpage RoboComp Interfaces: Cvhaar.ice
 *
 * \section intro_sec Introduction
* Interface for cvhaarComp. 
*                                                                   
* 
*                                                                                              
*    PORT  <br>   
*/
#ifndef CVHAAR_ICE
#define CVHAAR_ICE

/** \namespace RoboCompCvhaar
  *@brief Name space Cvhaar
  */
module RoboCompCvhaar
{
  struct Rectangle
  {
    int x;
    int y;
    int width;
    int height;
  };
  sequence<Rectangle> RectangleList;

  /** \interface Cvhaar
  *@brief interface Cvhaar
  */
  interface Cvhaar
  {
    RectangleList getObjects();
    bool detectPerson();

  };
};

#endif
