#ifndef PINZA_ICE
#define PINZA_ICE

module RoboCompPinza
{
  interface Pinza
  {
	// Set Pinza Position in mm; upwards positive, downwards negative from zero
	bool setPinzaPosition( int pos );

	//Get Pinza position
	int getPinzaPosition();

	//Set Zero
	void setZeroPosition();   
	
	//Set down
	bool setDownPosition();

	//set up	
	bool setUpPosition();
	
  };
};

#endif
