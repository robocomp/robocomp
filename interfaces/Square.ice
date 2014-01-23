#ifndef SQUARE_ICE
#define SQUARE_ICE

module RoboCompSquare
{
  exception HardwareFailedException { string what; };
  exception MovingImageException { string what; };

    sequence<byte> imgType;
    sequence<int> intVector;
  
  struct TSquareParams
  {
    /* Depende del tamaño de la captura y tamaño del cuadrado a detectar. */
    int minPixels;           /* Número mínimo de pixels necesarios para procesar un segmento. */
  
    /* Parámetros que dependen del tamaño de la imagen capturada. */
    int nPixelsWidth;         /* Número de pixels horizontales (coordenada X) que agrupamos para considerar el mismo pixel en la matriz -> Rejilla. */
    int nPixelsHeight;         /* Número de pixels verticales (coordenada Y) que agrupamos para considerar el mismo pixel en la matriz -> Rejilla. */
    int umbralIgualdadCoordenadas;   /* Umbral que indica la similitud de componentes x ó y. */
    int umbralRecta;         /* Umbral que indica la similitud (cercanía) entre rectas.*/

    /*string driver;*/
  };

  interface Square
  {
    TSquareParams getSquareParams();          // Devuelve los parámetros relevantes de este componente.

  
  };
};

#endif
