#ifndef LINEVISION_ICE
#define LINEVISION_ICE

/** \mainpage RoboComp Interfaces: LineVision.ice
 *
 * \section intro_sec Introduction
* Interface for linevisionComp
*                             
* Line segments extractor using Pedro Lopez de Teruel's et al algorithm <br>
* Extracts line segments from canny images and maintains a vector of segments <br>
*    
*    PORT 10024 <br>   
*    
*/

module RoboCompLineVision
{
  /** \struct LineSegment
  *@brief struct line segment
  */
  struct LineSegment
  {
    int numPoints;
    ///x coordinate of the center
    float cx;
    ///y coordinate of the center	
    float cy;
    ///angle	
    float angle;
    ///segment length
    float length;           /* Centro, ángulo y longitud del segmento.*/
    ///Perpendicular standar deviation
    float desvPerp;         /* Desviación típica en dirección perpendicular. */
    ///x coordinate initial of the extreme	
    float x1;
    ///y coordinate initial of the extreme	
    float y1;
    ///x coordinate end of the extreme	
    float x2;
    ///y coordinate end of the extreme		
    float y2;               /* Extremos. */
    ///Medium gray values	
    byte grayIzq;
    ///Medium gray values
    byte grayDer;           /* Valores de gris medianos. */
  };

  sequence<LineSegment> SegmentSequence;

  interface LineVision
    {
      /**@brief Returns a list of segments */
      idempotent SegmentSequence getData();
    };
  };

#endif
